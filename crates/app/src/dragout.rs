//! Dragging files *out* of the app into other applications (slicers, file
//! managers) on X11, by speaking the XDND protocol from a second X connection.
//!
//! winit cannot start an OS-level drag, but XDND needs no pointer grab on the
//! source side: while the user keeps the mouse button down (our window still
//! has the implicit grab and keeps receiving the release), we find the
//! XDND-aware window under the pointer with `QueryPointer`, exchange
//! Enter/Position/Status client messages with it, send Drop on release and
//! finally serve the file as `text/uri-list` through the XdndSelection.

#![cfg(target_os = "linux")]

use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use x11rb::connection::Connection;
use x11rb::protocol::shape::{self, ConnectionExt as _};
use x11rb::protocol::xproto::{
    AtomEnum, ChangeWindowAttributesAux, ClientMessageData, ClientMessageEvent, ColormapAlloc, ConfigureWindowAux, ConnectionExt,
    CreateGCAux, CreateWindowAux, EventMask, ImageFormat, ImageOrder, KeyButMask, PropMode, SelectionNotifyEvent, StackMode,
    VisualClass, Window, WindowClass, SELECTION_NOTIFY_EVENT,
};
use x11rb::protocol::Event;
use x11rb::rust_connection::RustConnection;
use x11rb::wrapper::ConnectionExt as _;
use x11rb::CURRENT_TIME;

x11rb::atom_manager! {
    Atoms: AtomsCookie {
        XdndAware,
        XdndEnter,
        XdndPosition,
        XdndStatus,
        XdndLeave,
        XdndDrop,
        XdndFinished,
        XdndSelection,
        XdndActionCopy,
        XdndTypeList,
        TARGETS,
        UTF8_STRING,
        TEXT_URI_LIST: b"text/uri-list",
        TEXT_PLAIN: b"text/plain",
        L360_TIME: b"_LUSION360_TIME",
        NET_WM_WINDOW_TYPE: b"_NET_WM_WINDOW_TYPE",
        NET_WM_WINDOW_TYPE_DND: b"_NET_WM_WINDOW_TYPE_DND",
    }
}

#[derive(Default)]
struct Shared {
    /// payload served to the drop target
    uri_list: Vec<u8>,
    /// current target and its XDND version
    target: Option<(Window, u32)>,
    /// target said it will accept
    accepted: bool,
    /// target finished reading the data
    finished: bool,
    /// latest server timestamp seen
    time: u32,
    /// drag icon should follow the pointer (when outside our window)
    icon_want: bool,
    /// the drag icon window, once created
    icon_win: Option<Window>,
}

/// The override-redirect window showing the drag preview outside our window.
struct IconWin {
    win: Window,
    pixmap: u32,
    gc: u32,
    depth: u8,
}

pub struct DragOut {
    conn: Arc<RustConnection>,
    atoms: Atoms,
    root: Window,
    win: Window,
    own: Vec<Window>,
    shared: Arc<Mutex<Shared>>,
    last_pos: Mutex<Option<(i16, i16)>>,
    icon: Mutex<Option<IconWin>>,
    /// 32-bit ARGB visual (only with a running compositor)
    argb: Option<(u32, u8)>,
}

/// Keyboard modifiers held during the drag (read from the X server, so they
/// work even when our window does not have keyboard focus).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Mods {
    pub shift: bool,
    pub ctrl: bool,
}

/// Where the drag currently is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Over {
    /// over our own window (or nothing XDND-aware)
    Inside,
    /// over another application's window that does not (yet) accept
    Refused,
    /// over another application's window that accepts the file
    Accepting,
}

impl DragOut {
    /// Connect to the X server. `own` = our top-level window id (from winit).
    pub fn new(own: Option<u32>) -> Option<DragOut> {
        let (conn, screen) = x11rb::connect(None).ok()?;
        let root = conn.setup().roots.get(screen)?.root;
        let atoms = Atoms::new(&conn).ok()?.reply().ok()?;
        let win = conn.generate_id().ok()?;
        conn.create_window(
            0,
            win,
            root,
            -10,
            -10,
            1,
            1,
            0,
            WindowClass::INPUT_ONLY,
            0,
            &CreateWindowAux::new().event_mask(EventMask::PROPERTY_CHANGE),
        )
        .ok()?;
        conn.flush().ok()?;
        // translucent rounded icon only when a compositor runs (otherwise ARGB shows black corners)
        let argb = (|| {
            let cm = conn.intern_atom(false, format!("_NET_WM_CM_S{screen}").as_bytes()).ok()?.reply().ok()?.atom;
            if conn.get_selection_owner(cm).ok()?.reply().ok()?.owner == x11rb::NONE {
                return None;
            }
            let scr = &conn.setup().roots[screen];
            scr.allowed_depths
                .iter()
                .filter(|d| d.depth == 32)
                .flat_map(|d| d.visuals.iter())
                .find(|v| v.class == VisualClass::TRUE_COLOR)
                .map(|v| (v.visual_id, 32u8))
        })();
        // top-level ancestor of our window (the WM frame, if any)
        let own_top = own.and_then(|w| {
            let mut w = w;
            for _ in 0..8 {
                let t = conn.query_tree(w).ok()?.reply().ok()?;
                if t.parent == root || t.parent == x11rb::NONE {
                    return Some(w);
                }
                w = t.parent;
            }
            Some(w)
        });
        let conn = Arc::new(conn);
        let shared = Arc::new(Mutex::new(Shared::default()));
        // icon follower: smooth even when the UI renders slowly
        {
            let scr = &conn.setup().roots[screen];
            let size = (scr.width_in_pixels as i32, scr.height_in_pixels as i32);
            let conn = conn.clone();
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("xdnd-icon".into())
                .spawn(move || icon_follow(conn, root, size, own_top, shared))
                .ok()?;
        }
        // event thread: selection requests and XDND replies
        {
            let conn = conn.clone();
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("xdnd".into())
                .spawn(move || event_loop(conn, atoms, win, shared))
                .ok()?;
        }
        Some(DragOut { conn, atoms, root, win, own: own.into_iter().collect(), shared, last_pos: Mutex::new(None), icon: Mutex::new(None), argb })
    }

    /// Start offering `files`.
    pub fn begin(&self, files: &[PathBuf]) {
        {
            let mut s = self.shared.lock().unwrap();
            *s = Shared { time: s.time, icon_win: s.icon_win, ..Default::default() };
        }
        self.set_files(files);
        *self.last_pos.lock().unwrap() = None;
        // obtain a real server timestamp (some toolkits reject CurrentTime)
        let _ = self.conn.change_property8(PropMode::APPEND, self.win, self.atoms.L360_TIME, AtomEnum::STRING, &[]);
        let _ = self.conn.flush();
        for _ in 0..20 {
            if self.shared.lock().unwrap().time != 0 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        let t = self.time();
        let _ = self.conn.set_selection_owner(self.win, self.atoms.XdndSelection, t);
        let _ = self.conn.flush();
    }

    /// Change what is offered (e.g. another format) without restarting the drag.
    pub fn set_files(&self, files: &[PathBuf]) {
        let mut uri = Vec::new();
        for f in files {
            uri.extend_from_slice(file_uri(f).as_bytes());
            uri.extend_from_slice(b"\r\n");
        }
        self.shared.lock().unwrap().uri_list = uri;
    }

    /// True if translucent (rounded) icons are possible.
    pub fn rounded_icons(&self) -> bool {
        self.argb.is_some()
    }

    /// Set the drag preview image (`w`×`h`, packed premultiplied ARGB).
    pub fn set_icon(&self, w: u16, h: u16, argb: &[u32]) {
        let mut icon = self.icon.lock().unwrap();
        if icon.is_none() {
            *icon = self.create_icon(w, h);
            self.shared.lock().unwrap().icon_win = icon.as_ref().map(|i| i.win);
        }
        let Some(i) = icon.as_ref() else { return };
        let msb = self.conn.setup().image_byte_order == ImageOrder::MSB_FIRST;
        let mut bytes = Vec::with_capacity(argb.len() * 4);
        for p in argb {
            bytes.extend_from_slice(&if msb { p.to_be_bytes() } else { p.to_le_bytes() });
        }
        let _ = self.conn.put_image(ImageFormat::Z_PIXMAP, i.pixmap, i.gc, w, h, 0, 0, 0, i.depth, &bytes);
        let _ = self.conn.clear_area(false, i.win, 0, 0, 0, 0);
        let _ = self.conn.flush();
    }

    fn create_icon(&self, w: u16, h: u16) -> Option<IconWin> {
        let c = &self.conn;
        let win = c.generate_id().ok()?;
        let mut aux = CreateWindowAux::new().override_redirect(1).border_pixel(0).background_pixel(0).save_under(1);
        let (depth, visual) = match self.argb {
            Some((v, d)) => {
                let cmap = c.generate_id().ok()?;
                c.create_colormap(ColormapAlloc::NONE, cmap, self.root, v).ok()?;
                aux = aux.colormap(cmap);
                (d, v)
            }
            None => (0, 0), // CopyFromParent
        };
        c.create_window(depth, win, self.root, -1000, -1000, w, h, 0, WindowClass::INPUT_OUTPUT, visual, &aux).ok()?;
        let real_depth = if depth == 0 { c.setup().roots.iter().find(|s| s.root == self.root).map(|s| s.root_depth).unwrap_or(24) } else { depth };
        let pixmap = c.generate_id().ok()?;
        c.create_pixmap(real_depth, pixmap, win, w, h).ok()?;
        let gc = c.generate_id().ok()?;
        c.create_gc(gc, pixmap, &CreateGCAux::new()).ok()?;
        c.change_window_attributes(win, &ChangeWindowAttributesAux::new().background_pixmap(pixmap)).ok()?;
        let _ = c.change_property32(PropMode::REPLACE, win, self.atoms.NET_WM_WINDOW_TYPE, AtomEnum::ATOM, &[self.atoms.NET_WM_WINDOW_TYPE_DND]);
        // never receive the pointer: drop targets underneath must stay reachable
        let _ = c.shape_rectangles(shape::SO::SET, shape::SK::INPUT, x11rb::protocol::xproto::ClipOrdering::UNSORTED, win, 0, 0, &[]);
        let _ = c.flush();
        Some(IconWin { win, pixmap, gc, depth: real_depth })
    }

    /// Show the drag icon whenever the pointer is outside our window.
    pub fn show_icon(&self, on: bool) {
        self.shared.lock().unwrap().icon_want = on;
    }

    fn time(&self) -> u32 {
        let t = self.shared.lock().unwrap().time;
        if t == 0 {
            CURRENT_TIME
        } else {
            t
        }
    }

    fn send(&self, to: Window, ty: u32, data: [u32; 5]) {
        let ev = ClientMessageEvent::new(32, to, ty, ClientMessageData::from(data));
        let _ = self.conn.send_event(false, to, EventMask::NO_EVENT, ev);
    }

    /// XDND version advertised by `w`, if it is drop-aware.
    fn aware(&self, w: Window) -> Option<u32> {
        let r = self.conn.get_property(false, w, self.atoms.XdndAware, AtomEnum::ATOM, 0, 1).ok()?.reply().ok()?;
        r.value32().and_then(|mut v| v.next())
    }

    /// Find the drop-aware window under the pointer (descending from the root).
    fn target_under_pointer(&self, mods: &mut Mods) -> Option<(Window, u32, i16, i16)> {
        let p = self.conn.query_pointer(self.root).ok()?.reply().ok()?;
        let (x, y) = (p.root_x, p.root_y);
        mods.shift = p.mask.contains(KeyButMask::SHIFT);
        mods.ctrl = p.mask.contains(KeyButMask::CONTROL);
        let mut w = p.child;
        for _ in 0..12 {
            if w == x11rb::NONE {
                return None;
            }
            if let Some(v) = self.aware(w) {
                return Some((w, v, x, y));
            }
            let q = self.conn.query_pointer(w).ok()?.reply().ok()?;
            w = q.child;
        }
        None
    }

    fn is_own(&self, w: Window) -> bool {
        self.own.contains(&w)
    }

    /// Call every frame while the button is held.
    pub fn update(&self) -> (Over, Mods) {
        let mut mods = Mods::default();
        let found = self.target_under_pointer(&mut mods);
        let (prev, accepted) = {
            let s = self.shared.lock().unwrap();
            (s.target, s.accepted)
        };
        let new = found.filter(|(w, ..)| !self.is_own(*w));
        let t = self.time();
        match (prev, new) {
            (Some((pw, _)), Some((nw, ..))) if pw == nw => {}
            (old, new) => {
                if let Some((ow, _)) = old {
                    self.send(ow, self.atoms.XdndLeave, [self.win, 0, 0, 0, 0]);
                }
                {
                    let mut s = self.shared.lock().unwrap();
                    s.target = new.map(|(w, v, ..)| (w, v.min(5)));
                    s.accepted = false;
                }
                if let Some((nw, v, ..)) = new {
                    let ver = v.min(5);
                    self.send(nw, self.atoms.XdndEnter, [self.win, ver << 24, self.atoms.TEXT_URI_LIST, self.atoms.TEXT_PLAIN, 0]);
                }
                *self.last_pos.lock().unwrap() = None;
            }
        }
        if let Some((nw, _, x, y)) = new {
            let mut lp = self.last_pos.lock().unwrap();
            if *lp != Some((x, y)) {
                *lp = Some((x, y));
                let pos = ((x as u16 as u32) << 16) | (y as u16 as u32);
                self.send(nw, self.atoms.XdndPosition, [self.win, 0, pos, t, self.atoms.XdndActionCopy]);
            }
        }
        let _ = self.conn.flush();
        let over = match new {
            None => Over::Inside,
            Some(_) if accepted => Over::Accepting,
            Some(_) => Over::Refused,
        };
        (over, mods)
    }

    /// Button released: drop on the current target. Returns true if a drop was sent.
    pub fn release(&self) -> bool {
        // give the target a moment to answer the last position
        for _ in 0..10 {
            let s = self.shared.lock().unwrap();
            if s.target.is_none() || s.accepted {
                break;
            }
            drop(s);
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        let (target, accepted) = {
            let s = self.shared.lock().unwrap();
            (s.target, s.accepted)
        };
        let t = self.time();
        let dropped = match target {
            Some((w, _)) if accepted => {
                self.send(w, self.atoms.XdndDrop, [self.win, 0, t, 0, 0]);
                true
            }
            Some((w, _)) => {
                self.send(w, self.atoms.XdndLeave, [self.win, 0, 0, 0, 0]);
                false
            }
            None => false,
        };
        let _ = self.conn.flush();
        {
            let mut s = self.shared.lock().unwrap();
            s.target = None;
            s.icon_want = false;
        }
        dropped
    }

    pub fn cancel(&self) {
        let target = {
            let mut s = self.shared.lock().unwrap();
            s.icon_want = false;
            s.target.take()
        };
        if let Some((w, _)) = target {
            self.send(w, self.atoms.XdndLeave, [self.win, 0, 0, 0, 0]);
            let _ = self.conn.flush();
        }
    }
}

/// Keep the drag icon next to the pointer (outside our own window only).
fn icon_follow(conn: Arc<RustConnection>, root: Window, screen: (i32, i32), own_top: Option<Window>, shared: Arc<Mutex<Shared>>) {
    let mut mapped = false;
    let mut last = (i32::MIN, i32::MIN);
    let (iw_w, iw_h) = (crate::dragicon::ICON_W as i32, crate::dragicon::ICON_H as i32);
    loop {
        let (want, iw) = {
            let s = shared.lock().unwrap();
            (s.icon_want, s.icon_win)
        };
        let Some(iw) = iw else {
            std::thread::sleep(std::time::Duration::from_millis(50));
            continue;
        };
        let mut show = false;
        if want {
            if let Ok(Ok(p)) = conn.query_pointer(root).map(|c| c.reply()) {
                show = Some(p.child) != own_top && p.child != iw;
                // below-right of the cursor, flipped near the screen edges
                let (px, py) = (p.root_x as i32, p.root_y as i32);
                let x = if px + 18 + iw_w > screen.0 { px - 14 - iw_w } else { px + 18 };
                let y = if py + 16 + iw_h > screen.1 { py - 12 - iw_h } else { py + 16 };
                if show && (x, y) != last {
                    last = (x, y);
                    let _ = conn.configure_window(iw, &ConfigureWindowAux::new().x(x).y(y).stack_mode(StackMode::ABOVE));
                }
            }
        }
        if show != mapped {
            mapped = show;
            let _ = if show { conn.map_window(iw) } else { conn.unmap_window(iw) };
        }
        let _ = conn.flush();
        std::thread::sleep(std::time::Duration::from_millis(if want { 8 } else { 40 }));
    }
}

fn event_loop(conn: Arc<RustConnection>, atoms: Atoms, win: Window, shared: Arc<Mutex<Shared>>) {
    loop {
        let Ok(ev) = conn.wait_for_event() else { return };
        match ev {
            Event::PropertyNotify(e) if e.window == win => {
                shared.lock().unwrap().time = e.time;
            }
            Event::ClientMessage(e) if e.type_ == atoms.XdndStatus => {
                let d = e.data.as_data32();
                let mut s = shared.lock().unwrap();
                if s.target.map(|t| t.0) == Some(d[0]) {
                    s.accepted = d[1] & 1 == 1;
                }
            }
            Event::ClientMessage(e) if e.type_ == atoms.XdndFinished => {
                shared.lock().unwrap().finished = true;
            }
            Event::SelectionRequest(r) => {
                let payload = shared.lock().unwrap().uri_list.clone();
                let mut prop = r.property;
                if prop == x11rb::NONE {
                    prop = r.target;
                }
                let ok = if r.target == atoms.TARGETS {
                    conn.change_property32(PropMode::REPLACE, r.requestor, prop, AtomEnum::ATOM, &[atoms.TARGETS, atoms.TEXT_URI_LIST, atoms.TEXT_PLAIN, atoms.UTF8_STRING]).is_ok()
                } else if r.target == atoms.TEXT_URI_LIST || r.target == atoms.TEXT_PLAIN || r.target == atoms.UTF8_STRING {
                    conn.change_property8(PropMode::REPLACE, r.requestor, prop, r.target, &payload).is_ok()
                } else {
                    false
                };
                let notify = SelectionNotifyEvent {
                    response_type: SELECTION_NOTIFY_EVENT,
                    sequence: 0,
                    time: r.time,
                    requestor: r.requestor,
                    selection: r.selection,
                    target: r.target,
                    property: if ok { prop } else { x11rb::NONE },
                };
                let _ = conn.send_event(false, r.requestor, EventMask::NO_EVENT, notify);
                let _ = conn.flush();
            }
            _ => {}
        }
    }
}

fn file_uri(p: &Path) -> String {
    let abs = std::fs::canonicalize(p).unwrap_or_else(|_| p.to_path_buf());
    let mut s = String::from("file://");
    for b in abs.to_string_lossy().bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'/' | b'-' | b'_' | b'.' | b'~' => s.push(b as char),
            _ => s.push_str(&format!("%{b:02X}")),
        }
    }
    s
}

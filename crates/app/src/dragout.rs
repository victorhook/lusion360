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
use x11rb::protocol::xproto::{
    AtomEnum, ClientMessageData, ClientMessageEvent, ConnectionExt, CreateWindowAux, EventMask, PropMode,
    SelectionNotifyEvent, Window, WindowClass, SELECTION_NOTIFY_EVENT,
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
}

pub struct DragOut {
    conn: Arc<RustConnection>,
    atoms: Atoms,
    root: Window,
    win: Window,
    own: Vec<Window>,
    shared: Arc<Mutex<Shared>>,
    last_pos: Mutex<Option<(i16, i16)>>,
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
        let conn = Arc::new(conn);
        let shared = Arc::new(Mutex::new(Shared::default()));
        // event thread: selection requests and XDND replies
        {
            let conn = conn.clone();
            let shared = shared.clone();
            std::thread::Builder::new()
                .name("xdnd".into())
                .spawn(move || event_loop(conn, atoms, win, shared))
                .ok()?;
        }
        Some(DragOut { conn, atoms, root, win, own: own.into_iter().collect(), shared, last_pos: Mutex::new(None) })
    }

    /// Start offering `files`.
    pub fn begin(&self, files: &[PathBuf]) {
        let mut uri = Vec::new();
        for f in files {
            uri.extend_from_slice(file_uri(f).as_bytes());
            uri.extend_from_slice(b"\r\n");
        }
        {
            let mut s = self.shared.lock().unwrap();
            *s = Shared { uri_list: uri, time: s.time, ..Default::default() };
        }
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
    fn target_under_pointer(&self) -> Option<(Window, u32, i16, i16)> {
        let p = self.conn.query_pointer(self.root).ok()?.reply().ok()?;
        let (x, y) = (p.root_x, p.root_y);
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
    pub fn update(&self) -> Over {
        let found = self.target_under_pointer();
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
        match new {
            None => Over::Inside,
            Some(_) if accepted => Over::Accepting,
            Some(_) => Over::Refused,
        }
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
        self.shared.lock().unwrap().target = None;
        dropped
    }

    pub fn cancel(&self) {
        let target = self.shared.lock().unwrap().target.take();
        if let Some((w, _)) = target {
            self.send(w, self.atoms.XdndLeave, [self.win, 0, 0, 0, 0]);
            let _ = self.conn.flush();
        }
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

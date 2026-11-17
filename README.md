# Lusion360

A small, fast, **local-first parametric CAD modeller** for Linux (and Windows), written in Rust.
It covers the Fusion 360 workflow most people actually use: sketch → constrain → extrude, sketch on a face, fillet, edit anything later in the timeline, then export straight to your slicer.

No cloud, no login, no telemetry. Designs are plain JSON files (`.l360`) you can diff and version with git.

![workflow](docs/screenshot.png)

## Features

**Modelling**
- **Sketches** on origin planes, construction planes or any planar face. Tools: line (chained), rectangle, centre rectangle, circle, 3-point arc, point, **sketch fillet** (rounds corners and keeps virtual sharps, so the overall dimensions still hold), trim, **text** (for embossed or engraved labels), construction toggle.
- **Type while drawing**, like Fusion: start a line and type `40`, start a rectangle and type `60x40`, or start a circle and type the diameter. The matching dimensions are added automatically.
- **Constraints**: coincident, point-on-curve, horizontal/vertical, parallel, perpendicular, equal, tangent, concentric, midpoint, fix. Lines that are nearly horizontal or vertical get H/V automatically while you draw. The sketch shows its remaining degrees of freedom: free geometry is blue, fully constrained geometry is white.
- **Dimensions** (length, distance, horizontal/vertical, point-to-line, radius, diameter, angle). Each one takes an **expression**, e.g. `width/2 + 3`, `1.5 in`, `sin(30deg)*r`.
- **Snapping** to sketch points, midpoints and curves, plus **model geometry** (edge endpoints, midpoints and circle centres projected onto the sketch plane).
- **Profiles** are detected automatically, including overlapping shapes and holes. You can click any closed region.
- **Extrude**: one side, symmetric or two sides; start offset; new body / join / cut / intersect. Drag the arrow in the viewport to set the distance. Pulling into a body switches to cut automatically. You can also extrude ("press-pull") planar faces.
- **Revolve** around an origin axis, a sketch line or an edge.
- **Fillet** and **chamfer**. Click edges, or click a face to take all of its edges.
- **Shell**, **combine** (join/cut/intersect bodies), **move/copy**, **pattern** (circular or linear/grid) and **mirror**.
- **Offset construction planes**, useful for sketching at different heights.

**Parametric timeline**
- Every operation is a feature in the timeline. Double-click a feature to edit it. You can drag the marker to roll back, suppress, reorder or delete.
- **Stable references.** Faces and edges get persistent names from the operation history, so a sketch on a face, or a fillet on an edge, keeps working after you change an upstream dimension. This is the "topological naming problem" that makes FreeCAD fragile.
- **User parameters** (Edit → Parameters) can be referenced from any value.
- Recompute is **incremental**: only the changed feature and the ones after it are rebuilt. Typical edits recompute in a few milliseconds.

**Organisation**
- Components can be nested to any depth. You can activate a component, move bodies between components, colour and hide them, and export per component.
- A "project" is simply a folder. The Project tab lists the designs and 3D files in it.

**Files**
- Open/view: native `.l360`, **STEP**, BREP, **STL**, **3MF** (including Bambu Studio files), **OBJ**, **glTF/GLB**, **PLY**, **OFF**. You can also drag & drop files onto the window. A `.l360` opens; mesh/STEP files are added to the current design.
- **Drag out** (X11): grab a body in the viewport or the browser, or drag the **3D Print** / **Print** button, and drop it onto Bambu Studio, another slicer or a file manager. The bodies are written to a 3MF in `~/.cache/lusion360/drag/` and handed over as a normal file drop. Esc cancels.
- Export: STEP, STL, 3MF, OBJ, BREP. Use **Print** (top right) to write a 3MF and optionally open it straight in **Bambu Studio**, OrcaSlicer, PrusaSlicer or Cura. Installed slicers are detected from PATH, flatpak and `~/Applications` AppImages.
- Undo/redo, crash-safe **autosave with recovery**, and an atomic save.

## Controls

| Action | Input |
|---|---|
| Orbit (around the point under the cursor) | Right-drag, or Shift + middle-drag |
| Pan | Middle-drag |
| Zoom to cursor | Wheel |
| Standard views | View cube (top right), Home / F6 |
| Command search | `S` |
| Extrude | Double-click a profile/face, or the toolbar |
| Fillet | `F` |
| Sketch tools | `L` line, `R` rectangle, `C` circle, `A` arc, `P` point, `D` dimension, `T` trim, `G` corner fillet, `X` construction, `Del` delete |
| While drawing | Type a number (or `60x40` for rectangles) + Enter |
| Finish sketch / cancel | `Esc` (steps back: tool → selection → finish) |
| Undo / redo | Ctrl+Z / Ctrl+Y |
| Save / open / new / import / export | Ctrl+S / Ctrl+O / Ctrl+N / Ctrl+I / Ctrl+E |

## Building

Requirements: Rust (stable), a C++17 compiler and the usual X11/Wayland/GTK3 development packages:

```bash
sudo apt install build-essential libgtk-3-dev   # Ubuntu/Debian
cargo build --release
./target/release/lusion360 [file-or-folder]
./install.sh            # optional: installs to ~/.local/bin + desktop entry
```

The geometry kernel is **OpenCASCADE 8** (the same kernel FreeCAD uses). On the first build a prebuilt static OCCT is downloaded automatically, about 70 MB. It comes from the [cadrum](https://github.com/lzpel/cadrum) project's release assets, and builds exist for Linux x86-64/aarch64 and Windows MSVC/GNU. To use your own build, set `OCCT_ROOT=/path/to/occt-install` (it needs `include/opencascade` and `lib`), or set `LUSION_OCCT_URL` to point at a tarball.

Windows: the same `cargo build --release` works with MSVC. The code has no Linux-only parts, but only Linux has been tested so far.

## AI agents (MCP)

Lusion360 includes a local stdio [Model Context Protocol](https://modelcontextprotocol.io/) server. It runs directly on the parametric core — it does not start the GUI or open a network port — so it is suitable for local coding/agent clients:

```bash
cargo build --release -p lusion-mcp
```

Configure the client to launch `target/release/lusion-mcp`. The server keeps documents in memory for the session and writes normal editable `.l360` files when asked. It can create/open/save native designs, set named parameters, create constrained rectangle/circle sketches on origin planes or stable planar face references, extrude solved profiles, fillet/chamfer named edges, shell selected faces, export CAD/mesh files, and write a fast isometric SVG preview.

All modelling calls are transactional: an operation that fails to recompute is rolled back. An agent should call `lusion_query_geometry` before selecting faces or edges; it returns persistent names with useful geometry metadata. Use named parameters and inspect after meaningful changes, then save the resulting `.l360` for opening in the desktop application.

## Architecture

```
crates/
  kernel/   Rust ⇄ C++ (cxx) shim over OpenCASCADE: sketch edges, planar region
            detection, prism/revolve/boolean/fillet/chamfer/shell/mirror,
            tessellation, STEP/BREP IO. Every op exposes its history.
  core/     Document model (serde), expression language, sketch constraint
            solver (damped least squares + DOF analysis), timeline evaluation
            with cached per-feature states and persistent face/edge naming,
            import/export.
  meshio/   Fast STL/OBJ/glTF/3MF/PLY/OFF readers and STL/OBJ/3MF writers.
  app/      egui + wgpu UI: viewport renderer (MSAA, thick lines, silhouettes),
            picking (BVH), sketch mode, feature dialogs, timeline, browser.
```

Persistent naming: every face is named after the feature and the sketch curve or edge that generated it, for example `f12.side(s3.c7)` or `f12.end`. Booleans and fillets carry these names forward through OpenCASCADE's `Modified`/`Generated` history. An edge is named by the pair of faces that meet at it. If a name ever disappears, the reference re-attaches to the nearest geometry and the feature shows a warning instead of failing silently.

## Why this exists (research notes)

- Fusion 360 has no Linux version, and Wine is unreliable for it. Users' main complaints are cloud dependence (logins, sync, outages), slowness and crashes, and recent licensing restrictions. ([Autodesk forum](https://forums.autodesk.com/t5/fusion-support-forum/why-is-fusion-360-such-junk/m-p/13782907/highlight/true), [All3DP](https://all3dp.com/2/fusion-360-for-linux-how-to-install-it/), [Autodesk performance article](https://autodesk.com/support/technical/article/How-to-optimize-settings-in-Fusion-360-for-performance))
- Users like Fusion for its timeline, its fast sketch/constraint workflow, sketching on faces, and press-pull extrudes. This app copies that model.
- **FreeCAD 1.0** finally mitigated the topological naming problem, but the UX is still widely seen as clunky. ([Hackster](https://hackster.io/news/freecad-hits-1-0-after-two-decades-finally-sees-an-end-to-the-toponaming-problem-7e7ac2df3e63), [FreeCAD wiki](https://wiki.freecad.org/Topological_naming_problem/en))
- **Dune 3D** (GTK, OCCT + SolveSpace solver) is the closest existing idea, still early. SolveSpace is solid but dated. ([Hackaday](https://hackaday.com/2024/05/05/dune-3d-open-source-3d-parametric-modeler-from-the-maker-of-horizon-eda))
- Rust kernels: **Fornjot** was archived in June 2026. **truck** cannot do robust fillets. CADmium (truck-based, in the browser) stalled. So the kernel here is OpenCASCADE, reached through a thin, history-aware C++ shim. ([Fornjot](https://github.com/hannobraun/fornjot), [CADmium](https://github.com/CADmium-Co/CADmium))

## Known limitations / roadmap

- Projected model geometry snaps to positions but is not linked: if the model changes, the sketch point stays where it was.
- No sketch offset/spline tools yet. No sweep/loft/hole/thread features yet. Text uses one bundled font (DejaVu Sans Bold).
- No assembly joints; components are organisational only.
- Mesh bodies (STL/3MF imports) can be viewed, exported, hidden and recoloured, but not used in booleans.
- Windows builds are untested.

License: MIT OR Apache-2.0. OpenCASCADE is LGPL-2.1 with exception. Phosphor icons are MIT. DejaVu fonts use the Bitstream Vera license.

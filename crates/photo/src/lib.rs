//! Photogrammetry for Lusion360: photos or videos → a printable, watertight mesh.
//!
//! * [`pipeline`] drives COLMAP (camera solving) and OpenMVS (dense reconstruction and
//!   meshing) as external programs: proven tools that would be years of work to rewrite.
//! * [`prepare`] turns the raw reconstruction into something a slicer accepts: finds the
//!   table and stands the object on it, removes the background, scales to real size,
//!   and rebuilds a closed solid ([`solidify`]) with a flat base.

pub mod decimate;
pub mod frames;
pub mod mesh;
pub mod pipeline;
pub mod prepare;
pub mod solidify;
pub mod tools;

pub use frames::Inputs;
pub use mesh::{Mesh, MeshStats};
pub use pipeline::{Quality, ScanResult, Settings};
pub use prepare::{Alignment, PrepSettings, Prepared, SizeAxis};
pub use tools::Toolchain;

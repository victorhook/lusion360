//! Locates (or downloads) a static OpenCASCADE build and compiles the C++ shim.
//!
//! Resolution order:
//!  1. `OCCT_ROOT` env var pointing at an install with `include/opencascade` (or `inc`) and `lib`.
//!  2. A cached prebuilt in `<target>/occt/<name>` (downloaded once).
//!  3. Download the prebuilt static OCCT 8.0.1 tarball published by the `cadrum` project
//!     (https://github.com/lzpel/cadrum, LGPL-2.1 OCCT binaries). Override the URL with
//!     `LUSION_OCCT_URL` (also accepts `file:///path/to.tar.gz`).
use std::env;
use std::path::{Path, PathBuf};

const OCCT_TAG: &str = "occt-8_0_1_rev3";

const OCC_LIBS: &[&str] = &[
    "TKDESTEP", "TKDECascade", "TKDE", "TKXSBase", "TKFillet", "TKOffset", "TKBool", "TKBO", "TKShHealing",
    "TKMesh", "TKPrim", "TKTopAlgo", "TKGeomAlgo", "TKBRep", "TKGeomBase", "TKG3d", "TKG2d", "TKMath", "TKernel",
];

fn main() {
    println!("cargo:rerun-if-env-changed=OCCT_ROOT");
    println!("cargo:rerun-if-env-changed=LUSION_OCCT_URL");
    println!("cargo:rerun-if-changed=cpp/shim.h");
    println!("cargo:rerun-if-changed=cpp/shim.cpp");
    println!("cargo:rerun-if-changed=src/ffi.rs");

    let target = env::var("TARGET").unwrap();
    let root = match env::var("OCCT_ROOT") {
        Ok(r) => PathBuf::from(r),
        Err(_) => {
            let dir = target_dir().join("occt").join(format!("{}-{}", OCCT_TAG, target.replace('-', "_")));
            if find_layout(&dir).is_none() {
                download(&dir, &target);
            }
            dir
        }
    };
    let (inc, lib) = find_layout(&root).unwrap_or_else(|| {
        panic!("OpenCASCADE not found under {} (expected include/opencascade and lib)", root.display())
    });

    let mut build = cxx_build::bridge("src/ffi.rs");
    build
        .file("cpp/shim.cpp")
        .include(&inc)
        .include(env::var("CARGO_MANIFEST_DIR").unwrap())
        .std("c++17")
        .define("_USE_MATH_DEFINES", None)
        .flag_if_supported("-w");
    if env::var("CARGO_CFG_TARGET_ENV").as_deref() == Ok("msvc") {
        build.flag("/utf-8").flag("/EHsc").flag("/D_USE_STD_VECTOR_ALGORITHMS=0");
    }
    build.compile("lusion_shim");

    println!("cargo:rustc-link-search=native={}", lib.display());
    for l in OCC_LIBS {
        println!("cargo:rustc-link-lib=static={}", l);
    }
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        for l in ["user32", "advapi32", "ws2_32"] {
            println!("cargo:rustc-link-lib={}", l);
        }
    }
}

fn target_dir() -> PathBuf {
    // OUT_DIR = <target>/<profile>/build/<pkg>/out  (or <target>/<triple>/<profile>/...)
    let out = PathBuf::from(env::var("OUT_DIR").unwrap());
    let mut p = out.as_path();
    for _ in 0..4 {
        p = p.parent().unwrap();
    }
    let triple = env::var("TARGET").unwrap();
    if p.file_name().map_or(false, |n| n == triple.as_str()) {
        p = p.parent().unwrap();
    }
    p.to_path_buf()
}

fn find_layout(root: &Path) -> Option<(PathBuf, PathBuf)> {
    let inc = [root.join("include").join("opencascade"), root.join("inc"), root.join("include")]
        .into_iter()
        .find(|p| p.join("TopoDS_Shape.hxx").exists())?;
    let lib = [root.join("lib"), root.join("win64").join("vc14").join("lib"), root.join("win64").join("gcc").join("lib")]
        .into_iter()
        .find(|p| p.exists())?;
    Some((inc, lib))
}

fn download(dest: &Path, target: &str) {
    let name = format!("{}-{}", OCCT_TAG, target.replace('-', "_"));
    let url = env::var("LUSION_OCCT_URL")
        .unwrap_or_else(|_| format!("https://github.com/lzpel/cadrum/releases/download/{OCCT_TAG}/{name}.tar.gz"));
    println!("cargo:warning=Downloading prebuilt OpenCASCADE from {url} (one-time)");
    let parent = dest.parent().unwrap();
    std::fs::create_dir_all(parent).unwrap();
    let reader: Box<dyn std::io::Read> = if let Some(p) = url.strip_prefix("file://") {
        Box::new(std::fs::File::open(p).expect("open local OCCT tarball"))
    } else {
        let resp = minreq::get(&url).with_timeout(600).send().expect("download OCCT");
        if resp.status_code != 200 {
            panic!("OCCT download failed: HTTP {} for {url}. Set OCCT_ROOT to a local OCCT install.", resp.status_code);
        }
        Box::new(std::io::Cursor::new(resp.into_bytes()))
    };
    let gz = flate2::read::GzDecoder::new(reader);
    tar::Archive::new(gz).unpack(parent).expect("extract OCCT tarball");
    let extracted = parent.join(&name);
    if extracted != dest && extracted.exists() {
        let _ = std::fs::remove_dir_all(dest);
        std::fs::rename(&extracted, dest).unwrap();
    }
}

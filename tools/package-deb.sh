#!/usr/bin/env bash
# Build a Debian/Ubuntu package: target/deb/lusion360_<version>_<arch>.deb
#   tools/package-deb.sh            # release build + package
#   NO_BUILD=1 tools/package-deb.sh # package the existing release binaries
# Install with:  sudo apt install ./target/deb/lusion360_*.deb
set -euo pipefail
cd "$(dirname "$0")/.."

version=$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -1)
arch=$(dpkg --print-architecture)
pkg=lusion360
stage=target/deb/${pkg}_${version}_${arch}

if [ -z "${NO_BUILD:-}" ]; then
  cargo build --release -p lusion360 -p lusion-mcp
fi
[ -f packaging/icons/lusion360-256.png ] || python3 tools/render-icons.py

rm -rf "$stage"
mkdir -p "$stage/DEBIAN" "$stage/usr/bin" "$stage/usr/share/applications" "$stage/usr/share/mime/packages" \
  "$stage/usr/share/icons/hicolor/scalable/apps" "$stage/usr/share/doc/$pkg" "$stage/usr/share/metainfo"

install -m755 target/release/lusion360 "$stage/usr/bin/lusion360"
[ -f target/release/lusion-mcp ] && install -m755 target/release/lusion-mcp "$stage/usr/bin/lusion-mcp"
strip --strip-unneeded "$stage"/usr/bin/* 2>/dev/null || true

install -m644 packaging/lusion360.desktop "$stage/usr/share/applications/lusion360.desktop"
install -m644 packaging/lusion360-mime.xml "$stage/usr/share/mime/packages/lusion360.xml"
install -m644 packaging/lusion360.svg "$stage/usr/share/icons/hicolor/scalable/apps/lusion360.svg"
for s in 16 24 32 48 64 128 256 512; do
  d="$stage/usr/share/icons/hicolor/${s}x${s}/apps"
  mkdir -p "$d"
  install -m644 "packaging/icons/lusion360-$s.png" "$d/lusion360.png"
done
cat > "$stage/usr/share/metainfo/lusion360.metainfo.xml" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<component type="desktop-application">
  <id>lusion360</id>
  <name>Lusion360</name>
  <summary>Fast local parametric 3D CAD</summary>
  <metadata_license>CC0-1.0</metadata_license>
  <project_license>MIT OR Apache-2.0</project_license>
  <launchable type="desktop-id">lusion360.desktop</launchable>
  <description><p>Sketch, constrain, extrude, fillet and edit everything later in a parametric timeline. Local files, no cloud. Exports straight to your slicer.</p></description>
</component>
XML
cat > "$stage/usr/share/doc/$pkg/copyright" <<TXT
Lusion360 — MIT OR Apache-2.0.
Bundled: OpenCASCADE (LGPL-2.1 with exception, statically linked), Phosphor icons (MIT),
DejaVu fonts (Bitstream Vera license).
TXT

# shared-library dependencies, worked out from the binaries
deps=$(
  tmp=$(mktemp -d)
  mkdir -p "$tmp/debian"
  printf 'Source: %s\n\nPackage: %s\nArchitecture: any\n' "$pkg" "$pkg" > "$tmp/debian/control"
  bins=$(readlink -f "$stage"/usr/bin/*)
  (cd "$tmp" && dpkg-shlibdeps -O $bins 2>/dev/null | sed -n 's/^shlibs:Depends=//p')
  rm -rf "$tmp"
)
# loaded at runtime (dlopen), not visible to dpkg-shlibdeps
extra="libxkbcommon0, libxkbcommon-x11-0, libvulkan1, mesa-vulkan-drivers | vulkan-icd, libx11-xcb1, libxcursor1, libxi6, libxrandr2, libwayland-client0"
size=$(du -sk "$stage/usr" | cut -f1)

cat > "$stage/DEBIAN/control" <<CTL
Package: $pkg
Version: $version
Architecture: $arch
Maintainer: Lusion360 <lusion360@localhost>
Installed-Size: $size
Depends: ${deps:+$deps, }$extra
Recommends: xdg-utils
Section: graphics
Priority: optional
Homepage: https://github.com/victorhook/lusion360
Description: Fast local parametric 3D CAD
 Sketch, constrain and extrude, sketch on faces, fillet, holes and threads,
 and edit anything later in a parametric timeline. Designs are plain .l360
 files. Exports STEP/STL/3MF/OBJ and opens prints straight in Bambu Studio,
 OrcaSlicer, PrusaSlicer or Cura. Includes lusion-mcp, a local MCP server
 for AI agents.
CTL

dpkg-deb --build --root-owner-group "$stage" "target/deb/${pkg}_${version}_${arch}.deb" >/dev/null
echo "built target/deb/${pkg}_${version}_${arch}.deb"
echo "install:  sudo apt install ./target/deb/${pkg}_${version}_${arch}.deb"

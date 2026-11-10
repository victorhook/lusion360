#!/usr/bin/env bash
# Install Lusion360 for the current user (~/.local).
set -euo pipefail
cd "$(dirname "$0")"
cargo build --release
mkdir -p ~/.local/bin ~/.local/share/applications ~/.local/share/icons/hicolor/scalable/apps ~/.local/share/mime/packages
install -m755 target/release/lusion360 ~/.local/bin/lusion360
install -m644 packaging/lusion360.desktop ~/.local/share/applications/lusion360.desktop
install -m644 packaging/lusion360.svg ~/.local/share/icons/hicolor/scalable/apps/lusion360.svg
install -m644 packaging/lusion360-mime.xml ~/.local/share/mime/packages/lusion360.xml
update-mime-database ~/.local/share/mime >/dev/null 2>&1 || true
update-desktop-database ~/.local/share/applications >/dev/null 2>&1 || true
echo "Installed: ~/.local/bin/lusion360 (make sure ~/.local/bin is on your PATH)"

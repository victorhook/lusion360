#!/bin/bash
# Start a private Xvfb (:99) + Lusion360 on it. Usage: start.sh [file]; PROFILE=release to use the release build.
H=$(cd "$(dirname "$0")" && pwd)
OUT=${HARNESS_OUT:-/tmp/lusion-harness}; mkdir -p "$OUT/xdg/data" "$OUT/xdg/config"
pkill -x Xvfb; pkill -x lusion360; sleep 0.5; rm -f "$OUT"/xdg/data/lusion360/autosave*.l360
Xvfb :99 -screen 0 1600x1000x24 >/dev/null 2>&1 &
sleep 1
cd "$H/../.."
XDG_DATA_HOME=$OUT/xdg/data XDG_CONFIG_HOME=$OUT/xdg/config DISPLAY=:99 VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json WAYLAND_DISPLAY= \
  RUST_LOG=${RUST_LOG:-warn,lusion360=info} timeout 600 ./target/${PROFILE:-debug}/lusion360 "$@" > "$OUT/app.log" 2>&1 &
# watchdog: kill the app above 3 GB RSS
( for i in $(seq 1 600); do sleep 1; pid=$(pgrep -x lusion360 | head -1); [ -z "$pid" ] && break; rss=$(awk '/VmRSS/{print $2}' /proc/$pid/status 2>/dev/null); [ -n "$rss" ] && [ "$rss" -gt 3000000 ] && kill -9 $pid && echo "WATCHDOG KILL rss=$rss" >> "$OUT/app.log"; done ) >/dev/null 2>&1 &
sleep ${WAIT:-4}

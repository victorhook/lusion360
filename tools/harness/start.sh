#!/bin/bash
# Start a private Xvfb + Lusion360 on it. Usage: start.sh [args…]
#   HARNESS_DISPLAY=:99 (default)  PROFILE=release  WAIT=4  HARNESS_OUT=/tmp/lusion-harness
# Only processes started by an earlier run with the same HARNESS_OUT are stopped,
# so several sessions can test side by side on different displays.
H=$(cd "$(dirname "$0")" && pwd)
OUT=${HARNESS_OUT:-/tmp/lusion-harness}; mkdir -p "$OUT/xdg/data" "$OUT/xdg/config"
DISP=${HARNESS_DISPLAY:-:99}
if [ -f "$OUT/pids" ]; then kill $(cat "$OUT/pids") 2>/dev/null; sleep 0.5; fi
rm -f "$OUT"/xdg/data/lusion360/autosave*.l360
Xvfb "$DISP" -screen 0 1600x1000x24 >/dev/null 2>&1 &
XPID=$!
sleep 1
cd "$H/../.."
XDG_DATA_HOME=$OUT/xdg/data XDG_CONFIG_HOME=$OUT/xdg/config DISPLAY=$DISP VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/lvp_icd.json WAYLAND_DISPLAY= \
  RUST_LOG=${RUST_LOG:-warn,lusion360=info} timeout 600 ./target/${PROFILE:-debug}/lusion360 "$@" > "$OUT/app.log" 2>&1 &
APID=$!
echo "$XPID $APID" > "$OUT/pids"
echo "$DISP" > "$OUT/display"
# watchdog: kill the app above 3 GB RSS
( for i in $(seq 1 600); do sleep 1; kill -0 $APID 2>/dev/null || break; rss=$(awk '/VmRSS/{print $2}' /proc/$APID/status 2>/dev/null); [ -n "$rss" ] && [ "$rss" -gt 3000000 ] && kill -9 $APID && echo "WATCHDOG KILL rss=$rss" >> "$OUT/app.log"; done ) >/dev/null 2>&1 &
sleep ${WAIT:-4}

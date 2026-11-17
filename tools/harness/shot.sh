#!/bin/bash
# Screenshot of the harness display: shot.sh [file.png]
OUT=${HARNESS_OUT:-/tmp/lusion-harness}; mkdir -p "$OUT"
DISP=${HARNESS_DISPLAY:-$(cat "$OUT/display" 2>/dev/null || echo :99)}
import -display "$DISP" -window root "${1:-$OUT/shot.png}"

#!/bin/bash
# Stop the Xvfb + app started by start.sh for this HARNESS_OUT.
OUT=${HARNESS_OUT:-/tmp/lusion-harness}
[ -f "$OUT/pids" ] && kill $(cat "$OUT/pids") 2>/dev/null; rm -f "$OUT/pids"

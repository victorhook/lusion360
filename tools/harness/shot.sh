#!/bin/bash
OUT=${HARNESS_OUT:-/tmp/lusion-harness}; mkdir -p "$OUT"
import -display :99 -window root "${1:-$OUT/shot.png}"

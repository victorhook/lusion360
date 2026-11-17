#!/usr/bin/env python3
"""Render packaging/lusion360.svg to PNG sizes (packaging/icons/) and the
app's 128 px window icon (crates/app/assets/icon128.rgba). Needs PyGObject
with Rsvg and pycairo (python3-gi, gir1.2-rsvg-2.0, python3-cairo)."""
import os, sys
import gi
gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg
import cairo

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
svg = os.path.join(root, "packaging", "lusion360.svg")
out = os.path.join(root, "packaging", "icons")
os.makedirs(out, exist_ok=True)
h = Rsvg.Handle.new_from_file(svg)

def render(size):
    surf = cairo.ImageSurface(cairo.FORMAT_ARGB32, size, size)
    ctx = cairo.Context(surf)
    vp = Rsvg.Rectangle()
    vp.x, vp.y, vp.width, vp.height = 0, 0, size, size
    h.render_document(ctx, vp)
    surf.flush()
    return surf

for size in (16, 24, 32, 48, 64, 128, 256, 512):
    render(size).write_to_png(os.path.join(out, f"lusion360-{size}.png"))

# window icon: straight-alpha RGBA bytes
s = render(128)
data = bytes(s.get_data())
rgba = bytearray()
for i in range(0, len(data), 4):
    b, g, r, a = data[i], data[i + 1], data[i + 2], data[i + 3]  # premultiplied BGRA (little endian)
    if a:
        r, g, b = min(255, r * 255 // a), min(255, g * 255 // a), min(255, b * 255 // a)
    rgba += bytes((r, g, b, a))
with open(os.path.join(root, "crates", "app", "assets", "icon128.rgba"), "wb") as f:
    f.write(rgba)
print("rendered icons to", out)

#!/usr/bin/env python3
"""Tiny XTest driver for GUI tests on a private Xvfb display ($DISPLAY).
cmds: focus | focusdlg | move x y | click x y [btn] | dclick x y | down x y [btn] | up x y [btn]
      drag x1 y1 x2 y2 btn | scroll x y n | key name (ctrl+z, shift+Delete, Escape...) | keydown/keyup name (hold shift) | type text | sleep s"""
import ctypes, ctypes.util, sys, time, subprocess, re
X = ctypes.cdll.LoadLibrary(ctypes.util.find_library('X11'))
T = ctypes.cdll.LoadLibrary(ctypes.util.find_library('Xtst'))
X.XOpenDisplay.restype = ctypes.c_void_p
X.XStringToKeysym.restype = ctypes.c_ulong
X.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
d = ctypes.c_void_p(X.XOpenDisplay(None))
def flush(): X.XFlush(d); time.sleep(0.03)
def move(x, y): T.XTestFakeMotionEvent(d, -1, int(float(x)), int(float(y)), 0); flush()
def btn(b, down): T.XTestFakeButtonEvent(d, int(b), bool(down), 0); flush()
MODS = {'ctrl': 'Control_L', 'shift': 'Shift_L', 'alt': 'Alt_L'}
NAMES = {' ': 'space', '.': 'period', '-': 'minus', '+': 'plus', '/': 'slash', '*': 'asterisk',
         '(': 'parenleft', ')': 'parenright', '_': 'underscore', ',': 'comma'}
def key(name, down=None):
    kc = X.XKeysymToKeycode(d, X.XStringToKeysym(name.encode()))
    if down is None:
        T.XTestFakeKeyEvent(d, kc, True, 0); flush(); T.XTestFakeKeyEvent(d, kc, False, 0); flush()
    else:
        T.XTestFakeKeyEvent(d, kc, down, 0); flush()
def focus(pat):
    out = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
    m = re.search(pat, out)
    if m: X.XSetInputFocus(d, ctypes.c_ulong(int(m.group(1), 16)), 2, 0); flush()
a = sys.argv[1:]; i = 0
while i < len(a):
    c = a[i]; i += 1
    if c == 'focus': focus(r'(0x[0-9a-f]+) "[^"]*Lusion360"')
    elif c == 'focusdlg': focus(r'(0x[0-9a-f]+) "(Save File|Open File|Select Folder)"')
    elif c == 'move': move(a[i], a[i+1]); i += 2
    elif c in ('click', 'down', 'up', 'dclick'):
        x, y = a[i], a[i+1]; i += 2; b = 1
        if i < len(a) and a[i].isdigit() and len(a[i]) == 1: b = int(a[i]); i += 1
        move(x, y); time.sleep(0.05)
        if c == 'click': btn(b, 1); btn(b, 0)
        elif c == 'dclick': btn(b, 1); btn(b, 0); btn(b, 1); btn(b, 0)
        elif c == 'down': btn(b, 1)
        else: btn(b, 0)
        time.sleep(0.1)
    elif c == 'drag':
        x1, y1, x2, y2 = map(float, a[i:i+4]); b = int(a[i+4]); i += 5
        move(x1, y1); btn(b, 1)
        for k in range(1, 21): move(x1 + (x2-x1)*k/20, y1 + (y2-y1)*k/20); time.sleep(0.02)
        btn(b, 0); time.sleep(0.1)
    elif c == 'scroll':
        x, y, n = a[i], a[i+1], int(a[i+2]); i += 3; move(x, y)
        for _ in range(abs(n)): btn(4 if n > 0 else 5, 1); btn(4 if n > 0 else 5, 0)
    elif c == 'key':
        k = a[i]; i += 1
        if '+' in k and len(k) > 1:
            parts = k.split('+'); mods = [MODS.get(m, m) for m in parts[:-1]]
            for m in mods: key(m, True)
            key(parts[-1])
            for m in reversed(mods): key(m, False)
        else: key(k)
        time.sleep(0.05)
    elif c in ('keydown', 'keyup'):
        k = a[i]; i += 1; key(MODS.get(k, k), c == 'keydown')
    elif c == 'type':
        for ch in a[i]: key(NAMES.get(ch, ch))
        i += 1
    elif c == 'sleep': time.sleep(float(a[i])); i += 1
    else: print('unknown', c); sys.exit(1)

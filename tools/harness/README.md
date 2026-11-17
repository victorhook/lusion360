# GUI test harness

Runs Lusion360 on a private virtual display (Xvfb, `:99` unless `HARNESS_DISPLAY` says otherwise; software Vulkan) and drives it with XTest,
so GUI flows can be tested and screenshotted without touching your desktop.

```bash
HARNESS_OUT=/some/dir tools/harness/start.sh examples/bracket.l360
DISPLAY=:99 tools/harness/x.py focus key s type sketch key Return
tools/harness/shot.sh            # -> $HARNESS_OUT/shot.png
```
Notes: there is no window manager, so `focus` (or `focusdlg` for GTK file dialogs) is needed before
keyboard input. Settings/autosaves go to `$HARNESS_OUT/xdg`. A watchdog kills the app above 3 GB.

Several sessions can test at once: give each its own `HARNESS_OUT` and `HARNESS_DISPLAY`
(e.g. `:98`). `start.sh` only stops processes it started itself for that `HARNESS_OUT`;
`stop.sh` shuts them down. Use the same display for `x.py` (`DISPLAY=$HARNESS_DISPLAY`).

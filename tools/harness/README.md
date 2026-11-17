# GUI test harness

Runs Lusion360 on a private virtual display (Xvfb :99, software Vulkan) and drives it with XTest,
so GUI flows can be tested and screenshotted without touching your desktop.

```bash
HARNESS_OUT=/some/dir tools/harness/start.sh examples/bracket.l360
DISPLAY=:99 tools/harness/x.py focus key s type sketch key Return
tools/harness/shot.sh            # -> $HARNESS_OUT/shot.png
```
Notes: there is no window manager, so `focus` (or `focusdlg` for GTK file dialogs) is needed before
keyboard input. Settings/autosaves go to `$HARNESS_OUT/xdg`. A watchdog kills the app above 3 GB.

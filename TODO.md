# TODO / ideas

Parked plans and loose ends for the `rick` branch. Newest thinking first.

## Rooms per workspace (parked)

Turn 3D from a fullscreen takeover into a per-workspace "layout": a workspace
either is a room or it isn't. Already done: the room draws behind the
Quickshell bar (`RENDER_POST_WINDOWS`), top/overlay layers stay 2D and take
pointer and keyboard input.

- **Marking rooms:** a `rooms` list in `hypr3d.config`
  (e.g. `rooms = { "3", "name:dev" }`) plus a `hypr3d:room` dispatcher that
  toggles the current workspace. open/close/toggle act on the current
  workspace.
- **Switching:** listen to `workspace.active`. Entering a room workspace
  fades into 3D, leaving fades back to 2D. Windows on other workspaces are
  never touched. Today switching while in 3D resets the room's windows into
  a pile.
- **Per-room state:** each room remembers the camera (position, yaw/pitch)
  and every window's placement (center, size/scale, yaw/pitch/roll). Windows
  that return reappear where they were; new ones spawn in front of the
  camera.
- **Persistence (open question):** memory only first, or saved to disk from
  the start. On disk, windows have to be matched by class + title, since
  addresses change between sessions.
- **Preferred layouts:** room-based preferred layouts (arrangements the
  room restores or applies) build on the stored state.
- Supersedes the earlier "dedicated room workspace" idea and the question of
  what happens to windows already on the monitor: a workspace is a room or
  not.

## Quickshell as a 3D HUD

Now that top/overlay layers draw over the room and take input, Quickshell
can serve as a heads-up display inside 3D (status, room controls, mode
indicator).

## Loose ends

- **Natural scrolling:** scrolling a window under the typing-mode cursor
  sends the raw wheel delta; it may ignore `input:natural_scroll` and the
  scroll factor.
- **F3 while typing elsewhere:** F3 toggles the debug HUD even while typing
  into a window on another monitor in typing mode.
- **Same-path reload:** only the Jolt statics were made safe for a reload of
  the same `.so` path (the mapping is never unloaded, see `joltInit`). Other
  statics may also carry over. Matters to anyone reloading via `hyprpm`;
  `reload.sh` sidesteps it by loading a unique copy.
- **Cursor sensitivity on small/far windows:** the typing-mode cursor moves
  1:1 in window pixels, which means big hand movements on a far-away or
  small window. A sensitivity option may be needed.
- **Mouse look while a layer has the keyboard:** in moving mode the camera
  still turns behind an open launcher.

## Upstream candidates (samine825/Hypr3D)

- Open: #2 close crash, #3 spawn box monitor offset.
- Could follow: the same-path reload fix (Jolt statics), keeping
  monitor-sized bottom layers (wallpapers) out of the room.

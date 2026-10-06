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

## To test after the upstream merge (2026-10-06)

- **Third-person view (F5) with the typing-mode cursor:** the cursor's
  screen ray and projection use `g_scene.camera()` (the eye). If third
  person renders from a different camera, cursor placement and clicks
  will be off.
- **Volumetric windows (`windows.depth`, default 0.05) with the typing
  cursor:** the cursor is drawn on the window's centre plane and picking
  uses the flat quad; on a thick slab it may sit slightly inside the
  front face.

## Loose ends

- **Scene changes need a plugin reload:** switching `scene.map.path` (and
  its transform) in the config and running `hyprctl reload` did not load
  the new model; it only appeared after `reload.sh`. Probably the scene
  object keeps its loaded model across a config reload when only the path
  changes, or the async load (PR #5) is not restarted.
- **Spawning inside scans:** photogrammetry scans often have objects (trees,
  lamps, vehicles) at their centre. Centring a model on the spawn can put
  the player inside one; a spawn check (or "find open ground") would help.

- **Natural scrolling:** scrolling a window under the typing-mode cursor
  sends the raw wheel delta; it may ignore `input:natural_scroll` and the
  scroll factor.
- **F3 while typing elsewhere:** F3 toggles the debug HUD even while typing
  into a window on another monitor in typing mode.
- **Same-path reload:** largely solved by #5's `-fno-gnu-unique` (the
  plugin is really unloaded now). `reload.sh` still loads a unique copy,
  which is harmless; it could go back to a plain unload/load.
- **Cursor sensitivity on small/far windows:** the typing-mode cursor moves
  1:1 in window pixels, which means big hand movements on a far-away or
  small window. A sensitivity option may be needed.
- **Mouse look while a layer has the keyboard:** in moving mode the camera
  still turns behind an open launcher.

## Upstream candidates (samine825/Hypr3D)

- Merged: #1 multi-monitor, #2 close crash, #3 spawn box monitor offset.
  #5 (AfrobamaYT: unload/reload crashes, async model loading, JPEG
  textures) is merged into `rick` but still open upstream.
- Planned PRs:
  - **Typing mode** (free cursor, the cursor on windows, client cursor
    images): on the existing Super + Left Alt keyboard-mode toggle,
    enabled by a config option, with the extra toggle button (BTN_BACK)
    assignable in config.
  - **Quickshell/layer integration:** draw the room under the top and
    overlay layers and hand them pointer and keyboard input, as a config
    choice of which layer the room is drawn under.
- Could follow: keeping monitor-sized bottom layers (wallpapers) out of
  the room, `hl.plugin.hypr3d.active()`.

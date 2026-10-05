# Hypr3D =^..^=

![alt text](images/Screenshot1.png)

**A new perspective on window management -- literally.**
A Hyprland plugin that turns your workspace into a walkable 3D space

> Experimental, pinned to Hyprland 0.56.2.

# Installation
## Hyprpm

```bash
# Install latest
hyprpm add https://github.com/samine825/Hypr3D
hyprpm enable hypr3d

# Update
hyprpm update
```



## Manual

```bash
# Build
cmake -S . -B build -DHYPRLAND_HEADERS=/var/cache/hyprpm/$USER/headersRoot
cmake --build build -j$(nproc)

# Load
hyprctl plugin load "$PWD/build/hypr3d.so"
```

# Use

To enable 3D:

```bash
hyprctl eval 'hl.plugin.hypr3d.toggle()'
```

Or bind in lua config:

```lua
hl.bind("SUPER + F12", hl.plugin.hypr3d.toggle)
```

## Controls

| Input                         | Action                                         |
| -------------------------------| ------------------------------------------------|
| Mouse move                    | Look around                                    |
| WASD                          | Move                                           |
| Space                         | Move up (flying) / Jump                        |
| Shift                         | Move down (flying)                             |
| Ctrl                          | Sprint                                         |
| C                             | Zoom, wheel adjusts                            |
| V                             | Toggle noclip (fly through walls and floors)   |
| Super + Left click            | Drag window                                    |
| Super + Right click           | Resize window                                  |
| Super + Mouse wheel click     | rotate window                                  |
| Super + Mouse wheel scrolling | Zoom window                                    |
| Super + Left Alt / Back button | Toggle typing mode: the view freezes and a cursor appears on the windows, moving in each window's own pixels (click, drag-select, scroll as usual); past the room's edge it continues onto the other monitors |
| F3                            | Toggle debug HUD                               |

# Configuration
## Lua config example:

```lua
if hl.plugin.hypr3d then
    hl.bind("SUPER + F12", hl.plugin.hypr3d.toggle)
    hl.plugin.hypr3d.config({
        world = {
            panorama = "~/Pictures/room.png",
            grid = true,
        },
        windows = {
            window_scale = 0.5,
            spawn_distance = 5,
        },
        player = {
            look_sensitivity = 0.0025,
            look_inertia = 0.03,
            move_inertia = 0.05,
            move_speed = 4.0,
            spawn = { x = 0, y = 0, z = 0 },
            flying = true, 
            walk_bob = true
        },
        scene = { -- as many models as you like, any names
            map = {
                path = "~/map.glb",
                transform = {
                    position = { 0, 0, 0 },
                    rotation = { 0, 0, 0 },
                    scale    = { 1, 1, 1 },
                },
                collision = true,
                static = true,
                physics = false,
            },
            -- name = { path = "...", static = false, physics = true },
        },
    })
    -- That's all for now :p
end
```



## Parameters

Everything is optional -- only set what you want to change. 
Vectors can be written either way: `{ x = 1, y = 2, z = 3 }` or just `{ 1, 2, 3 }`.

### World

| Option   | Type   | Default | Description                             |
| ----------| --------| ---------| -----------------------------------------|
| panorama | string | ""      | 360° background image (equirectangular) |
| grid     | bool   | true    | the starting 40x40 platform             |

### Windows

| Option         | Type  | Default | Description                             |
| ----------------| -------| ---------| -----------------------------------------|
| window_scale   | float | 0.5     | window size multiplier (at 100 px/m)    |
| spawn_distance | float | 5.0     | how far from you new windows appear (m) |

### Player

| Option           | Type    | Default     | Description                                             |
| ------------------| ---------| -------------| ---------------------------------------------------------|
| look_sensitivity | float   | 0.0025      | how fast the camera turns (radians per pointer count)   |
| look_inertia     | float   | 0.03        | camera coasting after you stop the mouse (in seconds)   |
| move_inertia     | float   | 0.05        | coasting after you stop walking in seconds (in seconds) |
| move_speed       | float   | 4.0         | how fast you walk (m/s, running - 2.5x)                 |
| spawn            | vector3 | { 0, 0, 0 } | player spawn point                                      |
| flying           | bool    | false       | disables falling                                        |
| noclip           | bool    | false       | pass through everything (implies flying); V toggles it  |
| walk_bob         | bool    | true        | simulate the rhythm of walking                          |

### Scene object

| Option         | Type      | Default   | Description                                          |
| ----------------| -----------| -----------| ------------------------------------------------------|
| path           | string    | ""        | the model file to load (.glb/.gltf)                  |
| transform      | transform | see below | idk                                                  |
| emissive_scale | float     | 1.0       | how bright the model's own light is                  |
| flat           | bool      | true      | trust the model's lighting as-is                     |
| collision      | bool      | true      | you can stand on it and bump into it                 |
| static         | bool      | true      | you can't pick it up and carry it; suitable for maps |
| physics        | bool      | false     | enable jolt physics (only if static=false)           |
| center         | string    | logical   | origin / logical                                     |
| center_offset  | vector3   | {0, 0, 0} |                                                      |

Want only part of a model to be solid? Rename those nodes in Blender to
start with `nocol` -- they'll still render, but you'll walk right through.

### Transform

| Option   | Type    | Default   | Description     |
| ----------| ---------| -----------| -----------------|
| position | vector3 | {0, 0, 0} | idk             |
| rotation | vector3 | {0, 0, 0} | idk, in degrees |
| scale    | vector3 | {1, 1, 1} | idkk            |

# Screenshots
![alt text](images/Screenshot5.png)
![alt text](images/Screenshot4.png)
![alt text](images/Screenshot2.png)
![alt text](images/Screenshot3.png)

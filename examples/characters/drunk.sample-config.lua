-- Sample hypr3d config with an animated character: the drunk guy.
--
-- drunk.glb is a Mixamo character merged with tools/mixamo2glb.py: one
-- skinned mesh with six clips -- idle, idle.alt1, idle.alt2, walk,
-- walk-back, run. Copy the parts you want into your own config (e.g.
-- ~/.config/hypr/custom/plugins.lua) and point `path` at wherever this
-- repository lives.
--
-- In the room:
--   * He idles in place, a random idle clip each time one ends.
--   * Walk into him: he stands in an upright collision cylinder.
--   * Super + left-drag carries him; he stays upright and faces you.
--   * Keys 2 / 3 draw a path on the ground (smooth / sharp points), 1 goes
--     back to the cursor. Drop him on a path and he walks it; double-click
--     him to switch between walking and running.

if hl.plugin.hypr3d then
	hl.bind("SUPER + F12", hl.plugin.hypr3d.toggle, { description = "Toggle Hypr3D" })

	hl.plugin.hypr3d.config({
		world = {
			grid = true,  -- the default 40 x 40 platform to stand on
			tools = true, -- tool slots 1-5: path drawing
		},
		player = {
			spawn = { x = 0, y = 0, z = 0 }, -- you start here, looking along -z
		},
		characters = {
			drunk = {
				path = "~/path/to/hypr3d/examples/characters/drunk.glb",
				-- 3 m in front of the spawn. Mixamo characters face +z, so
				-- with no rotation he faces the player.
				transform = {
					position = { 0, 0, -3 }, -- his feet
					rotation = { 0, 0, 0 },
					scale = { 1, 1, 1 },
				},
				idle = { "idle", "idle.alt1", "idle.alt2" },
				walk = "walk", -- played while walking a path
				run = "run",   -- played while running (double-click him)
				radius = 0.3,  -- collision cylinder, metres
				height = 1.8,
			},
		},
	})
end

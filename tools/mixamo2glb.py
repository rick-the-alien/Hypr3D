"""Merge a Mixamo character and its animation FBXs into one animated .glb.

Mixamo exports the skinned character and every animation as separate FBX
files. This imports the character, moves each animation's action onto the
character's armature as its own NLA track (named after the animation), and
exports a single binary glTF with one animation per track -- the format the
plugin's animated models read.

Usage (Blender 4.4+, headless):

    blender -b -P tools/mixamo2glb.py -- <dir> <out.glb>

<dir> holds character.fbx plus the animation FBXs. If it has a
character.json ({"character": "...", "animations": {...}}), its character
and the animation files it lists are used; otherwise every other *.fbx in
the directory is an animation. Each clip is named after its file without
the extension (idle.alt1.fbx -> "idle.alt1").
"""

import json
import os
import sys

import bpy


def args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if len(argv) != 2:
        sys.exit("usage: blender -b -P tools/mixamo2glb.py -- <dir> <out.glb>")
    return os.path.abspath(argv[0]), os.path.abspath(argv[1])


def sources(directory):
    """(character fbx, [animation fbx...]) for the directory."""
    manifest = os.path.join(directory, "character.json")
    if os.path.isfile(manifest):
        with open(manifest) as f:
            spec = json.load(f)
        character = spec.get("character", "character.fbx")
        files = []
        for entry in spec.get("animations", {}).values():
            for name in entry if isinstance(entry, list) else [entry]:
                if name not in files:
                    files.append(name)
    else:
        character = "character.fbx"
        files = sorted(
            f for f in os.listdir(directory)
            if f.lower().endswith(".fbx") and f != character)

    return (os.path.join(directory, character),
            [os.path.join(directory, f) for f in files])


def import_fbx(path):
    """Imports an FBX and returns the objects it added."""
    before = set(bpy.data.objects)
    bpy.ops.import_scene.fbx(filepath=path, ignore_leaf_bones=True,
                             automatic_bone_orientation=False)
    return [o for o in bpy.data.objects if o not in before]


def armature_of(objects):
    for o in objects:
        if o.type == "ARMATURE":
            return o
    return None


def main():
    directory, out = args()
    character_path, animations = sources(directory)

    bpy.ops.wm.read_factory_settings(use_empty=True)

    character = armature_of(import_fbx(character_path))
    if character is None:
        sys.exit(f"no armature in {character_path}")

    if character.animation_data is None:
        character.animation_data_create()

    # The character file's own (bind-pose or T-pose) action is not a clip.
    character.animation_data.action = None

    for path in animations:
        clip = os.path.basename(path)[:-len(".fbx")]
        added = import_fbx(path)
        source = armature_of(added)
        action = source.animation_data.action if (
            source and source.animation_data) else None

        if action is None:
            print(f"mixamo2glb: {clip}: no animation, skipped")
        else:
            action.name = clip
            action.use_fake_user = True

            track = character.animation_data.nla_tracks.new()
            track.name = clip
            strip = track.strips.new(clip, int(action.frame_range[0]), action)

            # Slotted actions (4.4+): bind the clip's slot to the character,
            # or the strip animates nothing on export.
            if hasattr(strip, "action_slot") and len(action.slots):
                strip.action_slot = action.slots[0]

            frames = action.frame_range[1] - action.frame_range[0]
            print(f"mixamo2glb: {clip}: {frames:.0f} frames")

        # Animation FBXs carry their own copy of the rig (and often the
        # mesh); only the action is kept.
        for o in added:
            bpy.data.objects.remove(o, do_unlink=True)

    os.makedirs(os.path.dirname(out), exist_ok=True)
    bpy.ops.export_scene.gltf(
        filepath=out,
        export_format="GLB",
        export_animations=True,
        export_animation_mode="NLA_TRACKS",
        export_skins=True,
        export_anim_slide_to_zero=True,
    )
    print(f"mixamo2glb: wrote {out}")


main()

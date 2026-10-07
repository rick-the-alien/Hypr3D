#!/usr/bin/env bash
# Reload the locally built plugin into the running Hyprland session.
#
# The .so is never really unmapped: Hyprland's headers give it
# STB_GNU_UNIQUE symbols, so dlclose is a no-op, and loading the SAME path
# again just hands back the old code with its old statics. Each reload
# therefore loads a fresh copy from a unique path.
#
# hyprctl exits 0 even on failure, so success is judged by its output.
set -u

BUILD="$HOME/workspace/rick-the-alien/hypr3d/build/hypr3d.so"
DIR="${XDG_RUNTIME_DIR:-/tmp}/hypr3d"
STATE="$DIR/loaded"

if [[ ! -f "$BUILD" ]]; then
    echo "reload: $BUILD not found (build first)" >&2
    exit 1
fi

mkdir -p "$DIR"

# After a crash/restart an inherited HYPRLAND_INSTANCE_SIGNATURE points at a
# dead instance: use the one whose socket is our WAYLAND_DISPLAY.
LIVE=$(hyprctl instances -j 2>/dev/null | jq -r --arg w "${WAYLAND_DISPLAY:-}" \
    '.[] | select(.wl_socket==$w) | .instance' | head -1)
[ -n "$LIVE" ] && export HYPRLAND_INSTANCE_SIGNATURE="$LIVE"

unload() {
    local OUT
    OUT=$(hyprctl plugin unload "$1" 2>&1)
    case "$OUT" in
        ok)                  echo "reload: unloaded $1"; return 0 ;;
        "plugin not loaded") return 0 ;;
        *)                   echo "reload: unload of $1 failed: $OUT" >&2; return 1 ;;
    esac
}

# The previous copy this script loaded, and the build path itself in case
# the plugin was loaded from there directly.
PREV=$(cat "$STATE" 2>/dev/null || true)
if [[ -n "$PREV" ]]; then
    unload "$PREV" || exit 1
fi
unload "$BUILD" || exit 1

COPY="$DIR/hypr3d-$(date +%s%N).so"
cp "$BUILD" "$COPY" || { echo "reload: copy to $COPY failed" >&2; exit 1; }

OUT=$(hyprctl plugin load "$COPY" 2>&1)
if [[ "$OUT" != ok ]]; then
    echo "reload: load failed: $OUT" >&2
    rm -f "$COPY"
    exit 1
fi
echo "$COPY" > "$STATE"
echo "reload: loaded $COPY"

# Old copies are still mapped in this session but no longer needed on disk.
find "$DIR" -name 'hypr3d-*.so' ! -path "$COPY" -delete

# The config only binds keys and applies options when hl.plugin.hypr3d
# exists at evaluation time.
OUT=$(hyprctl reload 2>&1)
if [[ "$OUT" != ok ]]; then
    echo "reload: config reload failed: $OUT" >&2
    exit 1
fi
echo "reload: config reloaded"

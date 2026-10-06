#pragma once

#include <hyprland/src/plugins/PluginAPI.hpp>
#include <hyprland/src/desktop/DesktopTypes.hpp>
#include <hyprland/src/layout/space/Space.hpp>

#include <cstdint>
#include <vector>

namespace H3D::Compat {

// One participating surface, in monitor-local logical coordinates. Either a
// regular window or a layer-shell surface (panels, bars, quickshell
// PanelWindow), which live outside the window list but are shown in the room
// all the same.
struct SWindowInfo {
    std::uintptr_t id = 0;
    PHLWINDOW      window;
    PHLLS          layer;     // set when isLayer
    bool           isLayer = false;
    CBox           monitorLocalBox; // FULL decorated box (content + border/titlebar), top-left origin
    Vector2D       surfaceOffset;   // top-left of the client surface inside monitorLocalBox (border margin)
    Vector2D       surfaceSize;     // client surface size in logical px
    bool           floating = false;
};

// Enumerate mapped, non-hidden windows that live on a visible workspace of
// `monitor`. Windows on invisible workspaces are skipped (the snapshot path
// refuses to render them anyway).
// `underLayers` (world.under_layers): the room is drawn under the top and
// overlay layers, so only bottom-layer surfaces join it; otherwise every
// layer above the background does.
std::vector<SWindowInfo> enumerateEligibleWindows(const PHLMONITOR& monitor,
                                                  bool underLayers = false);

// True if the window still belongs to the eligible set (mapped, not hidden).
bool isWindowEligible(const PHLWINDOW& window, const PHLMONITOR& monitor);

// Full decorated box (client surface + border/titlebar) in monitor-local
// logical px, plus the client surface's offset inside it and its size. Used
// both for the entity/UV geometry and -- crucially -- recorded together with
// every snapshot, so the quad always samples the box the snapshot was
// actually rendered at. Returns false when the geometry is unusable.
bool decoratedSurfaceBox(
    const PHLWINDOW&  window,
    const PHLMONITOR& monitor,
    CBox&             fullBox,
    Vector2D&         surfOffset,
    Vector2D&         surfSize
);

// --- layout ghosting --------------------------------------------------------
//
// While 3D mode is active the tiling layout must not manage the participating
// windows: each window keeps its last 2D geometry but is removed from the
// layout's target list (Layout::ITarget::setSpaceGhost). On exit the target is
// re-assigned to its space and its exact saved box is restored.
//
// IMPORTANT: save the geometry of ALL windows first (saveWindowLayout), then
// ghost them (applyWindowGhost) — ghosting one window triggers a relayout of
// the remaining ones, which would corrupt boxes saved afterwards.
struct SWindowLayoutSave {
    std::uintptr_t id   = 0;
    PHLWINDOWREF   window;
    CBox           box;   // global logical box at save time
    SP<Layout::CSpace> space;
    bool            wasFloating = false;
};

SWindowLayoutSave saveWindowLayout(const PHLWINDOW& window);
void              applyWindowGhost(SWindowLayoutSave& save);
void              restoreWindowLayout(SWindowLayoutSave& save);

CBox currentWindowBox(const PHLWINDOW& window);
bool setWindowBox(const PHLWINDOW& window, const CBox& box);

// --- pointer delivery -------------------------------------------------------
//
// Maps a crosshair hit onto the window's client surface and delivers a pointer
// button event through the seat, so the client actually receives the click.
// `localLogical` is window-local (top-left origin, logical px, decorations
// ignored for the MVP).
void deliverMotion(
    const PHLWINDOW& window,
    const Vector2D& localLogical,
    uint32_t timeMs
);

void deliverClick(
    const PHLWINDOW& window,
    const Vector2D& localLogical,
    uint32_t button,
    bool pressed,
    uint32_t timeMs
);

// Layer-surface delivery: same seat events, pointed at the layer's client
// surface, so panels/launchers receive ordinary pointer input.
void deliverMotion(
    const PHLLS& layer,
    const Vector2D& localLogical,
    uint32_t timeMs
);

void deliverClick(
    const PHLLS& layer,
    const Vector2D& localLogical,
    uint32_t button,
    bool pressed,
    uint32_t timeMs
);

// Layer lookup by entity id (nullptr when the id is not a live layer).
PHLLS findLayerById(std::uintptr_t id);

void clearPointerFocus();

// Whether a top or overlay layer surface (bar, sidebar, launcher,
// notification) or a layer popup takes pointer input at a global point on
// this monitor -- Hyprland's own hit test, input regions included. Those
// draw over the room and are used as ordinary 2D surfaces.
bool interactiveLayerAt(const PHLMONITOR& monitor, const Vector2D& global);

// Whether keyboard focus is on a layer surface (a launcher, a sidebar's
// text field): the room then leaves the keyboard and focus alone.
bool layerHasKeyboardFocus();

// Scroll for whatever surface holds pointer focus (set by deliverMotion).
// `value120` is the hi-res wheel value (120 per detent, 0 for smooth).
void deliverAxis(
    uint32_t                           timeMs,
    wl_pointer_axis                    axis,
    double                             value,
    int32_t                            value120,
    wl_pointer_axis_source             source,
    wl_pointer_axis_relative_direction relative
);

} // namespace H3D::Compat

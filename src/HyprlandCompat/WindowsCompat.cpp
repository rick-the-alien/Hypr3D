#include "HyprlandCompat/WindowsCompat.hpp"

#include <hyprland/src/desktop/state/FocusState.hpp>
#include <hyprland/src/desktop/state/WindowState.hpp>
#include <hyprland/src/desktop/view/Window.hpp>
#include <hyprland/src/desktop/view/LayerSurface.hpp>
#include <hyprland/src/desktop/view/Popup.hpp>
#include <hyprland/src/desktop/Workspace.hpp>
#include <hyprland/src/layout/target/Target.hpp>
#include <hyprland/src/managers/SeatManager.hpp>
#include <hyprland/src/protocols/LayerShell.hpp>
// CLayerShellResource's m_surface (CWLSurfaceResource) needs the full type
// for .lock()->m_current access in deliver* paths.
#include <hyprland/src/protocols/types/SurfaceState.hpp>
#include <hyprland/src/state/MonitorState.hpp>
#include <hyprland/src/render/Renderer.hpp>

#include <hyprutils/utils/ScopeGuard.hpp>
#include <cmath>

namespace H3D::Compat {

std::vector<SWindowInfo> enumerateEligibleWindows(const PHLMONITOR& monitor) {
    std::vector<SWindowInfo> out;

    if (!monitor || !Desktop::windowState())
        return out;

    for (const auto& window : Desktop::windowState()->windows()) {
        if (!window || !window->m_isMapped || window->isHidden())
            continue;

        const auto WORKSPACE = window->m_workspace;

        if (!WORKSPACE)
            continue;

        // The window must live on a workspace attached to this monitor that is
        // actually being rendered (the snapshot path requires it).
        if (WORKSPACE->m_monitor != monitor)
            continue;

        if (!WORKSPACE->isVisible() && !window->m_pinned && !WORKSPACE->m_forceRendering)
            continue;

        SWindowInfo info;
        info.id       = reinterpret_cast<std::uintptr_t>(window.get());
        info.window   = window;
        info.floating = window->m_isFloating;

        if (!decoratedSurfaceBox(
                window,
                monitor,
                info.monitorLocalBox,
                info.surfaceOffset,
                info.surfaceSize))
            continue;

        // X11 menus are override-redirect windows at absolute screen
        // positions: they belong next to the window they came from.
        if (window->m_isX11 && window->isX11OverrideRedirect()) {
            info.attached = true;
            if (const auto PARENT = window->x11Parent())
                info.parentId = reinterpret_cast<std::uintptr_t>(PARENT.get());
        }

        out.push_back(std::move(info));
    }

    // Popups (menus, tooltips) of the windows above: attached surfaces at
    // their offset from the parent. The head node of each tree is a dummy.
    const size_t WINDOWS = out.size();
    for (size_t w = 0; w < WINDOWS; ++w) {
        const auto WINDOW = out[w].window;
        if (!WINDOW || !WINDOW->m_popupHead)
            continue;

        struct SCollect {
            std::vector<SWindowInfo>* out;
            PHLMONITOR                monitor;
            std::uintptr_t            parentId;
            Desktop::View::CPopup*    head;
        } COLLECT{&out, monitor, out[w].id, WINDOW->m_popupHead.get()};

        WINDOW->m_popupHead->breadthfirst(
            [](SP<Desktop::View::CPopup> popup, void* data) {
                auto* C = static_cast<SCollect*>(data);
                if (!popup || popup.get() == C->head || !popup->m_mapped ||
                    !popup->visible())
                    return;

                const Vector2D SIZE = popup->size();
                if (SIZE.x <= 0 || SIZE.y <= 0)
                    return;

                SWindowInfo info;
                info.id       = reinterpret_cast<std::uintptr_t>(popup.get());
                info.popup    = popup;
                info.attached = true;
                info.parentId = C->parentId;
                info.monitorLocalBox =
                    CBox{popup->coordsGlobal() - C->monitor->m_position, SIZE};
                info.surfaceOffset = Vector2D{0, 0};
                info.surfaceSize   = SIZE;
                C->out->push_back(std::move(info));
            },
            &COLLECT);
    }

    // Layer-shell surfaces (panels, bars, quickshell PanelWindow) live
    // outside the window list; show them in the room as regular entities.
    // The background layer is skipped -- that is where wallpapers live, and a
    // full-screen wallpaper would swallow the whole room. Their boxes are
    // monitor-local and decoration-free.
    constexpr uint32_t LAYER_BACKGROUND = 0;

    for (const auto& LAYERLIST : monitor->m_layerSurfaceLayers) {
        for (const auto& LSREF : LAYERLIST) {
            const auto LS = LSREF.lock();

            if (!LS || !LS->visible() || LS->m_layer == LAYER_BACKGROUND)
                continue;

            const auto BOXOPT = LS->surfaceLogicalBox();

            if (!BOXOPT || BOXOPT->w <= 0 || BOXOPT->h <= 0)
                continue;

            SWindowInfo info;
            info.id      = reinterpret_cast<std::uintptr_t>(LS.get());
            info.layer   = LS;
            info.isLayer = true;

            info.monitorLocalBox = *BOXOPT;
            info.surfaceOffset   = Vector2D{0, 0};
            info.surfaceSize     = Vector2D{BOXOPT->w, BOXOPT->h};

            out.push_back(std::move(info));
        }
    }

    return out;
}

bool decoratedSurfaceBox(
    const PHLWINDOW&  window,
    const PHLMONITOR& monitor,
    CBox&             fullBox,
    Vector2D&         surfOffset,
    Vector2D&         surfSize
) {
    if (!window || !monitor)
        return false;

    // The main surface box is the content only; decorations (the border ring,
    // titlebars) extend beyond it and are part of what a snapshot renders, so
    // the 3D quad must span the unified box to show them.
    const auto SURF = window->getWindowMainSurfaceBox();
    const auto FULL = window->getWindowBoxUnified(Desktop::View::FULL_EXTENTS);

    // A dimAround rule makes the unified box cover the whole monitor; clamp
    // each side margin so a rogue rule cannot blow the quad up.
    constexpr double MAX_MARGIN = 150.0;
    const double LEFT   = std::clamp<double>(SURF.x - FULL.x, 0.0, MAX_MARGIN);
    const double TOP    = std::clamp<double>(SURF.y - FULL.y, 0.0, MAX_MARGIN);
    const double RIGHT  = std::clamp<double>((FULL.x + FULL.w) - (SURF.x + SURF.w), 0.0, MAX_MARGIN);
    const double BOTTOM = std::clamp<double>((FULL.y + FULL.h) - (SURF.y + SURF.h), 0.0, MAX_MARGIN);

    const auto MONPOS = monitor->m_position;

    fullBox = CBox{
        SURF.x - LEFT - MONPOS.x,
        SURF.y - TOP - MONPOS.y,
        SURF.w + LEFT + RIGHT,
        SURF.h + TOP + BOTTOM,
    };

    // Surface-local input coordinates are the hit position minus this offset,
    // so growing the quad to include borders never shifts input.
    surfOffset = Vector2D{LEFT, TOP};
    surfSize   = Vector2D{SURF.w, SURF.h};

    return fullBox.w > 0 && fullBox.h > 0;
}

bool isWindowEligible(const PHLWINDOW& window, const PHLMONITOR& monitor) {
    if (!window || !window->m_isMapped || window->isHidden())
        return false;

    const auto WORKSPACE = window->m_workspace;

    if (!WORKSPACE || !monitor)
        return false;

    if (WORKSPACE->m_monitor != monitor)
        return false;

    if (!WORKSPACE->isVisible() && !window->m_pinned && !WORKSPACE->m_forceRendering)
        return false;

    return true;
}

SWindowLayoutSave saveWindowLayout(const PHLWINDOW& window) {
    SWindowLayoutSave save;

    if (!window || !window->m_target)
        return save;

    save.id          = reinterpret_cast<std::uintptr_t>(window.get());
    save.window      = window;
    save.box         = window->m_target->position();
    save.space       = window->m_target->space();
    save.wasFloating = window->m_target->floating();

    return save;
}

void applyWindowGhost(SWindowLayoutSave& save) {
    if (save.window.expired())
        return;

    const auto WINDOW = save.window.lock();

    if (!WINDOW || !WINDOW->m_target)
        return;

    // The 3D view owns geometry. Force the target floating first, then ghost
    // it out of the layout so a tiled layout cannot overwrite our changes.
    WINDOW->m_target->setFloating(true);
    WINDOW->m_target->setSpaceGhost(save.space);
}

void restoreWindowLayout(SWindowLayoutSave& save) {
    if (save.window.expired())
        return;

    const auto WINDOW = save.window.lock();

    if (!WINDOW || !WINDOW->m_target)
        return;

    // Put the target back into its space while it is still floating. For a
    // window that was floating before 3D, retain the real box produced by the
    // 3D resize. Tiled windows deliberately return to their original layout.
    const CBox RESTORE_BOX = save.wasFloating ? WINDOW->m_target->position() : save.box;

    if (save.space)
        WINDOW->m_target->assignToSpace(save.space);
    else
        WINDOW->m_target->setSpaceGhost(nullptr);

    WINDOW->m_target->setFloating(true);
    WINDOW->m_target->setPositionGlobal(RESTORE_BOX);
    WINDOW->m_target->rememberFloatingSize(Vector2D{RESTORE_BOX.w, RESTORE_BOX.h});
    WINDOW->m_target->setFloating(save.wasFloating);

    g_pHyprRenderer->damageWindow(WINDOW);
}

CBox currentWindowBox(const PHLWINDOW& window) {
    if (!window || !window->m_target)
        return {};

    return window->m_target->position();
}

bool setWindowBox(const PHLWINDOW& window, const CBox& box) {
    if (!window || !window->m_target)
        return false;

    const CBox CURRENT = window->m_target->position();

    const bool unchanged =
        std::fabs(CURRENT.x - box.x) < 0.01 &&
        std::fabs(CURRENT.y - box.y) < 0.01 &&
        std::fabs(CURRENT.w - box.w) < 0.01 &&
        std::fabs(CURRENT.h - box.h) < 0.01;

    if (unchanged)
        return false;

    window->m_target->setFloating(true);
    window->m_target->setPositionGlobal(box);
    window->m_target->rememberFloatingSize(Vector2D{box.w, box.h});

    if (g_pHyprRenderer)
        g_pHyprRenderer->damageWindow(window);

    return true;
}

PHLWINDOW findPopupParentById(std::uintptr_t id) {
    if (id == 0 || !Desktop::windowState())
        return nullptr;

    for (const auto& WINDOW : Desktop::windowState()->windows()) {
        if (!WINDOW || !WINDOW->m_isMapped || !WINDOW->m_popupHead)
            continue;

        struct SFind {
            std::uintptr_t id;
            bool           found = false;
        } FIND{id};
        WINDOW->m_popupHead->breadthfirst(
            [](SP<Desktop::View::CPopup> popup, void* data) {
                auto* F = static_cast<SFind*>(data);
                if (popup && reinterpret_cast<std::uintptr_t>(popup.get()) == F->id)
                    F->found = true;
            },
            &FIND);
        if (FIND.found)
            return WINDOW;
    }

    return nullptr;
}

PHLLS findLayerById(std::uintptr_t id) {
    if (id == 0)
        return nullptr;

    for (const auto& MONITOR : State::monitorState()->monitors()) {
        for (const auto& LAYERLIST : MONITOR->m_layerSurfaceLayers) {
            for (const auto& LSREF : LAYERLIST) {
                const auto LS = LSREF.lock();

                if (LS && reinterpret_cast<std::uintptr_t>(LS.get()) == id)
                    return LS;
            }
        }
    }

    return nullptr;
}

void clearPointerFocus() {
    if (!g_pSeatManager)
        return;

    g_pSeatManager->setPointerFocus(nullptr, {});
}

// The surface under a point given in the window's surface-local px: one of
// its open popups (menus are drawn attached to the window, at their real
// offset, so their hits arrive in the window's space) or the window itself.
// `local` becomes local to the returned surface.
static SP<CWLSurfaceResource> surfaceAt(const PHLWINDOW& window, Vector2D& local) {
    if (window->m_popupHead) {
        const auto SURF = window->getWindowMainSurfaceBox();
        const Vector2D GLOBAL = Vector2D{SURF.x, SURF.y} + local;
        if (const auto POPUP = window->m_popupHead->at(GLOBAL, true);
            POPUP && POPUP.get() != window->m_popupHead.get()) {
            if (const auto RES = POPUP->resource()) {
                local = GLOBAL - POPUP->coordsGlobal();
                return RES;
            }
        }
    }
    return window->resource();
}

void deliverMotion(
    const PHLWINDOW& window,
    const Vector2D& localLogical,
    uint32_t timeMs
) {
    if (!window || !g_pSeatManager)
        return;

    Vector2D local = localLogical;
    const auto SURFACE = surfaceAt(window, local);
    if (!SURFACE)
        return;

    g_pSeatManager->setPointerFocus(SURFACE, local);
    g_pSeatManager->sendPointerMotion(timeMs, local);
    g_pSeatManager->sendPointerFrame();
}

void deliverClick(
    const PHLWINDOW& window,
    const Vector2D&  localLogical,
    uint32_t         button,
    bool             pressed,
    uint32_t         timeMs
) {
    if (!window)
        return;

    Vector2D local = localLogical;
    const auto SURFACE = surfaceAt(window, local);

    if (!SURFACE || !g_pSeatManager)
        return;

    // Point the seat at the aimed surface, put the virtual pointer at the hit
    // coordinate, then send the button and a frame. The application therefore
    // sees an ordinary Wayland pointer event even though the physical cursor
    // is captured by the 3D view.
    g_pSeatManager->setPointerFocus(SURFACE, local);
    g_pSeatManager->sendPointerMotion(timeMs, local);
    g_pSeatManager->sendPointerButton(
        timeMs,
        button,
        pressed ? WL_POINTER_BUTTON_STATE_PRESSED : WL_POINTER_BUTTON_STATE_RELEASED
    );
    g_pSeatManager->sendPointerFrame();
}

void deliverMotion(
    const PHLLS& layer,
    const Vector2D& localLogical,
    uint32_t timeMs
) {
    if (!layer || !g_pSeatManager)
        return;

    const auto SURFACE =
        layer->m_layerSurface ? layer->m_layerSurface->m_surface.lock() : nullptr;

    if (!SURFACE)
        return;

    g_pSeatManager->setPointerFocus(SURFACE, localLogical);
    g_pSeatManager->sendPointerMotion(timeMs, localLogical);
    g_pSeatManager->sendPointerFrame();
}

void deliverClick(
    const PHLLS& layer,
    const Vector2D& localLogical,
    uint32_t button,
    bool pressed,
    uint32_t timeMs
) {
    if (!layer || !g_pSeatManager)
        return;

    const auto SURFACE =
        layer->m_layerSurface ? layer->m_layerSurface->m_surface.lock() : nullptr;

    if (!SURFACE)
        return;

    g_pSeatManager->setPointerFocus(SURFACE, localLogical);
    g_pSeatManager->sendPointerMotion(timeMs, localLogical);
    g_pSeatManager->sendPointerButton(
        timeMs,
        button,
        pressed ? WL_POINTER_BUTTON_STATE_PRESSED : WL_POINTER_BUTTON_STATE_RELEASED
    );
    g_pSeatManager->sendPointerFrame();
}

} // namespace H3D::Compat

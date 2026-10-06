#include <hyprland/src/plugins/PluginAPI.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/PlaneShape.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/NarrowPhaseQuery.h>
#include <Jolt/Physics/Body/BodyFilter.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <hyprland/src/event/EventBus.hpp>
#include <hyprland/src/render/OpenGL.hpp>
#include <hyprland/src/render/Renderer.hpp>
#include <hyprland/src/render/pass/PassElement.hpp>
#include <hyprland/src/render/gl/GLFramebuffer.hpp>
#include <hyprland/src/output/Monitor.hpp>
#include <hyprland/src/desktop/view/Window.hpp>
#include <hyprland/src/managers/SeatManager.hpp>
#include <hyprland/src/managers/eventLoop/EventLoopManager.hpp>
#include <hyprland/src/managers/fullscreen/FullscreenController.hpp>
#include <hyprland/src/config/values/ConfigValues.hpp>
#include <hyprland/src/desktop/state/WindowState.hpp>
#include <hyprland/src/config/ConfigValue.hpp>
#include <hyprland/src/devices/IKeyboard.hpp>
#include <hyprland/src/devices/IPointer.hpp>
#include <hyprland/src/pointer/PointerManager.hpp>
#include <hyprland/src/pointer/PointerController.hpp>
#include <hyprland/src/state/MonitorState.hpp>
#include <hyprland/src/managers/SessionLockManager.hpp>
#include <hyprland/src/render/Texture.hpp>
#include <hyprland/src/protocols/core/Compositor.hpp>
#include <hyprutils/memory/UniquePtr.hpp>

// This system's lua headers (5.5) lost the extern "C" guard: including them
// from C++ mangles the API names and the plugin fails to load with
// "undefined symbol: _Z8lua_typeP9lua_Statei". liblua exports plain C
// symbols, so the linkage is forced here.
extern "C" {
#include <lua.h>
#include <lauxlib.h>
}

#include <linux/input-event-codes.h>
#include <xkbcommon/xkbcommon.h>

#include "Render/GLScene.hpp"
#include "Input/AimFocus.hpp"
#include "Input/InputController.hpp"
#include "World/MapCollision.hpp"
#include "World/World3D.hpp"
#include "HyprlandCompat/WindowsCompat.hpp"
#include "HyprlandCompat/FocusCompat.hpp"
#include "HyprlandCompat/WindowCapture.hpp"
#include "HyprlandCompat/PointerHook.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace H3D {

static HANDLE PHANDLE = nullptr;

static GLScene         g_scene;
static InputController g_input;
static World3D::CWorld g_world;
static AimFocus        g_aim;
static Compat::CWindowCapture g_capture;

static bool g_active = false;

static float g_transition = 0.0f;
static float g_transitionTarget = 0.0f;

static PHLMONITOR g_monitor = nullptr;

static const auto g_inputClockStart = std::chrono::steady_clock::now();
static std::chrono::steady_clock::time_point g_lastTick =
    g_inputClockStart;

static bool g_renderedOnce = false;
static bool g_reportedFramebufferError = false;
static bool g_reportedRenderError = false;
static std::string g_lastRenderGate;  // why the 3D pass was skipped last frame
static double g_msUpdate3D = 0.0;     // per-section profiling (exponential avg)
static double g_msJolt     = 0.0;
static double g_msRender   = 0.0;
static std::string g_lastError;        // last caught handler exception

static void dumpErrorNow(const std::string& what) {
    g_lastError = what;
    std::ofstream out("/tmp/hypr3d-status.txt", std::ios::app);
    out << "!!! EXCEPTION " << what << "\n";
}
static bool g_reportedPointerHookError = false;

// Layout snapshot taken on entry; empty once the windows are back under the
// layout's control.
static std::vector<Compat::SWindowLayoutSave> g_layoutSaves;
static bool g_ghosted = false;

// What to draw this frame, rebuilt once per frame from the world.
static std::vector<GLScene::WindowRender> g_renderWindows;

// Per-window depth-slab silhouette (analytic rounded rectangle), rebuilt
// when the captured box or the reported corner radius changes. Cleared with
// the frame -- entries for closed windows die with it.
struct SWinOutline {
    int w = 0, h = 0, r = -1;
    std::shared_ptr<const std::vector<SOutlineLoop>> loops;
};
static std::unordered_map<std::uintptr_t, SWinOutline> g_winOutlines;

// Set when a resize gesture ends: the next capture pass makes one FORCED
// snapshot of that window, so the wall silhouette runs its full-resolution
// "box settled" refresh -- an idle window would otherwise never snapshot
// again and keep the last mid-drag (reduced-resolution) outline.
static std::uintptr_t g_skirtFinalRefreshId = 0;

// The window that currently owns keyboard focus, as the aim logic last set it.
static std::uintptr_t g_lastFocusId = 0;

// 3D FPS-style mouse gestures. Super is the modifier: LMB moves the aimed
// window in the 3D room, RMB resizes its real Hyprland/Wayland geometry.
enum class EPointerGesture : uint8_t {
    None,
    Move3D,
    ResizeReal,
    WheelRoll,
    MapDrag, // carrying a dynamic (static = false) scene object
    CharDrag // carrying a character
};

static bool            g_pointerDown = false;
static EPointerGesture g_pointerGesture = EPointerGesture::None;
static uint32_t        g_pointerButton = 0;
static bool             g_superHeld = false;

static PHLWINDOW        g_clientButtonWindow;
static PHLLS            g_clientButtonLayer;
static Vector2D          g_clientButtonLocal{};
static uint32_t         g_clientButton = 0;
static bool              g_clientButtonDown = false;

struct SResizeGesture {
    bool          active = false;
    std::uintptr_t id = 0;
    PHLWINDOW     window;
    CBox          startBox{};
    Vec3          startCenter{};
    float         startWorldWidth = 0.0f;
    float         startWorldHeight = 0.0f;
    Vec3          planePoint{};
    Vec3          planeNormal{0.0f, 0.0f, 1.0f};
    // Crosshair position on the real window at grab time, in surface px. The
    // resize is driven by the aim's DELTA from this point, so the grabbed
    // corner never snaps to the crosshair when the gesture starts.
    Vector2D      grabPx{};
    int           edgeX = 0; // -1 left, +1 right
    int           edgeY = 0; // +1 top, -1 bottom
};

// Player spawn point in the room, set via hl.plugin.hypr3d.config().
static SResizeGesture g_resize{};


// Guards against a snapshot triggering another copy of the plugin inside
// Hyprland's nested offscreen render.
static bool g_capturing = false;



// Snapshot scheduling: the snapshot pass is the heaviest per-frame cost, so
// windows whose committed buffer and box are unchanged skip it entirely
// (checked inside the capture layer). `force` refreshes regardless -- for the
// aimed window, any gesture target, and the keyboard-focused window (whose
// border styling changes with focus). This is what keeps the room cheap while
// a screen capture (OBS/pipewire) copies every damaged frame.
static uint64_t g_captureFrames = 0;
static std::uintptr_t g_lastAimedId = 0;

// Super+wheel-PRESS roll: rotating the aimed window around its normal by
// sweeping the crosshair around the window center. The swept angle is
// measured in WORLD space around the normal, which is invariant to the roll
// itself -- only camera rotation drives it, so the gesture never feeds back
// into itself.
static struct SWheelRotate {
    bool           active     = false;
    bool           model      = false;  // rolling a scene object, not a window
    size_t         modelIndex = SIZE_MAX;
    std::uintptr_t id         = 0;
    Vec3           center     = {};
    Vec3           normal     = {};
    Vec3           reference  = {};  // crosshair point - center at grab
    float          startRoll  = 0.0f;
} s_wheelRot;

// Super+wheel hover zoom: glides the aimed window along the ray from the
// camera through the window toward the target distance (exponential lerp,
// same feel as the drag zoom). Cleared when a drag takes over the window or
// the entity disappears.
static std::uintptr_t s_zoomId     = 0;
static Vec3           s_zoomDir    = {};
static Vec3           s_zoomAnchor = {}; // camera position at the last wheel event
static double         s_zoomCur    = 0.0;
static double         s_zoomTarget = 0.0;

// Fullscreen passthrough: when Hyprland fullscreens a window while the 3D
// view is open, the window's quad animates to face the (frozen) camera and
// stretch exactly over the frustum, then the plugin hands the screen back to
// the 2D compositor: input is released, the cursor reappears, the 3D scene
// stops rendering. Exiting fullscreen reverses it -- the quad is back, input
// captured, cursor hidden. The real window is forced to the monitor box
// during 2D so the compositor shows it truly fullscreened.
static void resetPointerGesture();

enum class EFullscreenPhase : uint8_t { None, To2D, In2D, To3D };
static EFullscreenPhase g_fsPhase = EFullscreenPhase::None;
static PHLWINDOWREF     g_fsWindow;
static CBox             g_fsRestoreBox{}; // floating box before the fullscreen
static CBox             g_fsMonitorBox{}; // the fullscreen target box
static float            g_fsDistance = 8.0f; // fov-derived, set by startTo2D
static Vec3             g_fsStartCenter{}, g_fsEndCenter{};
static float g_fsStartYaw = 0.f, g_fsStartPitch = 0.f;
static float g_fsEndYaw = 0.f, g_fsEndPitch = 0.f;
static float g_fsStartRoll = 0.f;
static std::chrono::steady_clock::time_point g_fsPhaseStart{};
static bool  g_fsWasOn = false; // a fullscreen window existed since the last poll
static PHLWINDOWREF g_fsLastFSWindow; // the most recent fullscreen window
static std::uintptr_t g_fsCurrentId = 0; // the id of the CURRENT fullscreen window, 0 = none
static float g_fsAlpha = 1.0f;  // composite alpha during the transition

// The room's own fade for the NON-fullscreen windows: 1 = visible. Driven by
// the pump (deterministic ticks), sampled into WindowRender::alpha -- the
// snapshot alpha is baked at capture time and never advances (the buffer
// does not change during an alpha fade), so a snapshot-driven fade freezes
// after a couple of frames. Hyprland's own fade animates the real channels;
// ours guarantees the room matches it.
static float g_fsFade = 1.0f;
static std::chrono::steady_clock::time_point g_fsFadeLast{};
static float g_fsRawP   = 0.0f;  // raw (pre-smoothstep) transition progress
static float g_fsSavedRoll   = 0.0f; // the window's roll before the fullscreen
static float g_fsRollAtStart = 0.0f; // roll at the To3D start (aborts may differ)
static int   g_fsAssertFrames = 0;   // re-assert the restored box...
static CBox  g_fsAssertBox{};
static int   g_fsStableCount  = 0;   // consecutive frames the box matched

// Last known real box of every window OUTSIDE any fullscreen transition,
// refreshed each frame. The pre-fullscreen box must come from here: by the
// time the pump detects the fullscreen, Hyprland has already resized the
// window to the monitor, so the live geometry is NOT the pre-FS box.
static std::unordered_map<std::uintptr_t, CBox> g_fsStableBoxes;
static bool  g_captureReleasePending = false;
static constexpr float kFsAnimDuration = 0.6f;

static void pollFullscreen();
static void startTo2D(const PHLWINDOW& window, bool captureRestoreBox = true);

// Windows that appear while the view is open spawn as floating panels of
// this logical size, windows.spawn_width/height (see ghostWindows), and enter
// the room in front of the camera, facing it. A larger box with a smaller
// window_scale gives the same panel in the room with more pixels (sharper,
// more content), as if the window were further away.
static float g_cfgSpawnWidth  = 960.0f;
static float g_cfgSpawnHeight = 540.0f;
static constexpr float kSpawnDistance = 10.0f;

// Room poses remembered across 3D sessions: leaving 3D saves where every
// window and widget stood, and coming back puts them there again instead of
// spawning them in front of the camera. Plugin memory only -- a reload or a
// restart starts fresh. Keyed by address, guarded by a weak reference so a
// new window reusing a closed one's address starts fresh too. The size is
// not stored: it always follows the window's real box.
struct SRememberedPose {
    PHLWINDOWREF window;
    PHLLSREF     layer;
    Vec3         center{};
    float        yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
};
static std::unordered_map<std::uintptr_t, SRememberedPose> g_rememberedPoses;

static const SRememberedPose* rememberedPose(const Compat::SWindowInfo& info) {
    const auto IT = g_rememberedPoses.find(info.id);
    if (IT == g_rememberedPoses.end())
        return nullptr;

    const bool SAME = info.isLayer ?
        info.layer && IT->second.layer.lock() == info.layer :
        info.window && IT->second.window.lock() == info.window;

    return SAME ? &IT->second : nullptr;
}

// A normal damage cycle stops when nothing else in Hyprland changes. 3D mode is
// itself an animated scene, so keep a small render pump alive while it is open.
// 8 ms targets roughly 120 Hz without making the event loop spin continuously.
static SP<CEventLoopTimer> g_framePump;
static constexpr auto kFramePumpInterval = std::chrono::milliseconds(8);

// Two keyboard modes, toggled with Super + Left Alt:
//   Space  -- WASD/Space/Shift/Ctrl drive the camera;
//   Window -- every key reaches the focused window for typing.
// The mouse behaves identically in both: it always looks around, drags
// windows (Super+LMB), resizes (Super+RMB) and clicks through to clients.
enum class EKeyboardMode : uint8_t { Space, Window };
static EKeyboardMode g_keyboardMode = EKeyboardMode::Space;

// input.typing_cursor: window (typing) mode also frees the pointer and gives
// it a cursor on the windows. Off, window mode only redirects the keyboard
// and the mouse keeps steering the camera.
static bool     g_cfgTypingCursor = false;
// input.typing_button: a mouse button (evdev code) that toggles window mode,
// 0 = none.
static uint32_t g_cfgTypingButton = 0;
static bool          g_altHeld      = false;

// The free cursor was last seen over the room (see onMouseMove).
static bool g_freeOnRoom = false;

// A press that switched modes; its release is swallowed too.
static uint32_t g_swallowRelease = 0;

// Camera movement keys, set only while the 3D view owns the keyboard.
static bool g_keyFwd = false;
static bool g_keyBack = false;
static bool g_keyLeft = false;
static bool g_keyRight = false;
static bool g_keyUp = false;
static bool g_keyDown = false;
static bool g_keySprint = false;

static bool g_hookInstalled = false;

// Plugin settings, set from lua via hl.plugin.hypr3d.config({...}). Missing
// keys keep their current value, so a partial config only touches what it
// names; wrong-typed keys raise a lua error. No value ranges: what you
// write is what the plugin uses.
// --- world ------------------------------------------------------------------
static std::string g_cfgPanorama;                // panorama image path
static std::string g_cfgMonitor;                 // monitor name (e.g. "DP-1"), empty = focused
static bool        g_cfgGrid = true;
// world.under_layers: draw the room under the top and overlay layers (bars,
// launchers, notifications stay 2D on top and take input) instead of over
// everything.
static bool        g_cfgUnderLayers = false;             // base grid platform on/off

// --- windows ----------------------------------------------------------------
static float       g_cfgWindowScale   = 0.5f;    // room multiplier on window size
static float       g_cfgSpawnDistance = 5.0f;    // units in front of the camera
static float       g_cfgWindowDepth   = 0.05f;   // slab thickness, 0 = flat quads

// --- player -----------------------------------------------------------------
static float       g_cfgLookInertia   = 0.03f;   // seconds, 0 = off
static float       g_cfgMoveInertia   = 0.05f;   // seconds, 0 = off
static float       g_cfgMoveSpeed     = 4.0f;    // world units / second
static float       g_cfgSensitivity   = 0.0025f; // radians per pointer count
static bool        g_playerFlying     = true;    // false = walk / jump / gravity
static bool        g_playerNoclip     = false;   // pass through everything (V)

// Noclip always flies: there is nothing to stand on.
static bool playerFlies() {
    return g_playerFlying || g_playerNoclip;
}
static bool        g_playerCollision  = true;    // reserved: the capsule is
        // the movement system, so this only gates future per-object contact
static bool        g_cfgWalkBob       = true;    // view-only walk bob (walking only)
static GLScene::SPlayerCfg g_playerCfg;          // the player character
static float        g_playerAnimSpeed[4] = {1.f, 1.f, 1.f, 1.f};

// Feet position; eyes ride kEyeHeight above (spawn 0,0,0 = standing on
// the grid platform at world zero).
static Vec3 g_playerSpawn{0.0f, 0.0f, 0.0f};

// Walking physics state (grounded comes from the Jolt body's ground ray).
static bool g_grounded = false;

// --- scene: unlimited named glTF objects ------------------------------------
struct SSceneObjectCfg {
    std::string name; // the lua table key ("scene.<name>")
    std::string path;
    Vec3        position{}, rotationDeg{}, scale{1.0f, 1.0f, 1.0f};
    float       emissiveScale = 1.0f;
    bool        flat = false; // false = headlight half-lambert shading
    bool        collision = true;
    bool        dynamic = false; // static = false -> grabbable with Super+LMB
    bool        physics = false; // gravity + world collisions (dynamic only)
    CMapModel::ECenter center = CMapModel::ECenter::Logical;
    Vec3        centerOffset{}; // extra pivot shift, local units
};

static std::vector<SSceneObjectCfg> g_sceneObjects;
static std::vector<SSceneObjectCfg> g_sceneLuaState; // the last parsed config
static bool g_sceneLuaValid = false;


// Index of the currently grabbed dynamic object, SIZE_MAX when none.
static size_t g_mapGrabIndex = SIZE_MAX;

// Per-object physics (scene object physics = true): fall velocity + the AABB
// cache (recomputed when the object's generation changes).
struct SObjPhys {
    Vec3 vel{};
};
static std::vector<SObjPhys>   g_objPhys;
static std::vector<Vec3>       g_objAABBLo, g_objAABBHi;
static std::vector<uint32_t>   g_objAABBGens;

// Super+wheel hover zoom for a dynamic scene object (the model-class twin of
// the window hover zoom): the aimed object's center rides the zoomed
// distance on the camera-object line.
static size_t s_modelZoomId   = SIZE_MAX;
static float  s_modelZoomDist = 0.0f;
static float  g_mapGrabDist  = 0.0f; // camera-to-object-CENTER distance


// F3 debug HUD: collision wireframe + room info overlay.
static bool        g_debugHud = false;
static float       g_debugFps = 0.0f;

// C-key view zoom: g_zoomLevel glides toward the target (the wheel-adjusted
// magnification while C is held, 1x when released). The wheel level resets
// to kZoomBase on every press -- it does not survive the key release.
static bool  g_zoomHeld  = false;
static float g_zoomWheel = 2.0f;
static float g_zoomLevel = 1.0f;
static constexpr float kZoomBase = 2.0f;

// F5 view modes: 0 first person, 1 third person behind, 2 third person front.
static int g_viewMode = 0;
static float g_playerCamDist = 0.f; // smoothed third-person distance
static constexpr float kThirdDist = 2.5f;

// Per-object collision trees (see update3D): one small BVH per scene
// object, used by picking (modelRayHit). The static map's tree builds once;
// the grid platform slab lives in Jolt (syncFloorBody).
static std::vector<CMapCollision> g_objTrees;
static std::vector<uint32_t>    g_objTreeGens;

// --- Jolt Physics: rigid body simulation for scene objects ------------------
// Object layers: static geometry vs moving bodies. Moving bodies collide
// with everything; static-vs-static never (they cannot move anyway).
constexpr JPH::ObjectLayer LAYER_STATIC = 0;
constexpr JPH::ObjectLayer LAYER_MOVING = 1;
// A carried object is pose-driven: it sits in NO contact pair at all, so it
// can neither shove the player body around nor fight its own teleport.
constexpr JPH::ObjectLayer LAYER_GRABBED = 2;
constexpr JPH::ObjectLayer NUM_OBJECT_LAYERS = 3;

constexpr JPH::BroadPhaseLayer BP_LAYER_STATIC(0);
constexpr JPH::BroadPhaseLayer BP_LAYER_MOVING(1);

class SBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface {
  public:
    SBroadPhaseLayerInterface() {
        mObjectToBroadPhase[LAYER_STATIC] = BP_LAYER_STATIC;
        m_objectToBroadPhase2[LAYER_MOVING] = BP_LAYER_MOVING;
        m_objectToBroadPhase2[LAYER_GRABBED] = BP_LAYER_MOVING;
    }

    JPH::uint GetNumBroadPhaseLayers() const override {
        return 2;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        return layer == LAYER_STATIC ? BP_LAYER_STATIC : BP_LAYER_MOVING;
    }

  private:
    JPH::BroadPhaseLayer m_objectToBroadPhase2[NUM_OBJECT_LAYERS];
    JPH::BroadPhaseLayer mObjectToBroadPhase[NUM_OBJECT_LAYERS] = {};
};

class SObjectVsBroadPhase final : public JPH::ObjectVsBroadPhaseLayerFilter {
  public:
    bool ShouldCollide(JPH::ObjectLayer layer, JPH::BroadPhaseLayer bp) const override {
        if (layer == LAYER_STATIC)
            return bp == BP_LAYER_MOVING;
        return true; // moving collides with everything
    }
};

class SObjectLayerPair final : public JPH::ObjectLayerPairFilter {
  public:
    bool ShouldCollide(JPH::ObjectLayer a, JPH::ObjectLayer b) const override {
        if (a == LAYER_GRABBED || b == LAYER_GRABBED)
            return false; // carried: pose-driven, collides with nothing
        if (a == LAYER_STATIC)
            return b == LAYER_MOVING;
        return true; // moving vs moving and moving vs static
    }
};

static SBroadPhaseLayerInterface g_bpLayers;
static SObjectVsBroadPhase      g_objVsBp;
static SObjectLayerPair         g_objPair;

static JPH::PhysicsSystem*  g_joltSystem = nullptr;
static JPH::BodyInterface*  g_bodyIf     = nullptr;

// Per-scene-object Jolt body state (index-aligned with g_sceneObjects).
struct SObjJolt {
    JPH::BodyID body{};
    uint32_t    meshGen = 0; // model generation the body was built from
    Vec3        builtScale{1.0f, 1.0f, 1.0f}; // body-space mapping used at
    Vec3        builtPivot{};                 // build time (scale + pivot)
    bool        valid = false;
};
static std::vector<SObjJolt> g_joltBodies;

// A static object's mesh shape, cooked off the main thread: Jolt building
// the MeshShape of a 95 000-triangle model held Hyprland ~180 ms (-O2,
// perf). Jolt is made for this -- MeshShape creation was tuned for many
// threads building meshes at once.
struct SShapeJob {
    uint32_t             meshGen = 0; // what the shape is built from
    Vec3                 scale{}, pivot{};
    JPH::Ref<JPH::Shape> shape;       // empty when Jolt refused the mesh
    std::atomic<bool>    done{false};
    std::jthread         thread;      // last: joined before the rest goes
};
static std::vector<std::unique_ptr<SShapeJob>> g_shapeJobs; // by object

// A static object's first shape is still cooking: there is nothing to stand
// on yet where it will be.
static bool joltShapesPending() {
    for (size_t i = 0; i < g_shapeJobs.size() && i < g_joltBodies.size(); ++i)
        if (g_shapeJobs[i] && !g_joltBodies[i].valid &&
            !g_shapeJobs[i]->done.load(std::memory_order_acquire))
            return true;
    return false;
}

// Gravity factor saved while a wheel-roll gesture suspends the object's
// simulation (captured ONCE at grab -- re-reading it per frame would store
// the suspended zero and never restore it).
static float g_rolledGravity = 1.0f;

// Creates/removes/teleports the Jolt body of each scene object so it
// matches the config state. Body classes:
//   static = true              -> Static body (the location's geometry)
//   dynamic && physics         -> Dynamic body (the simulation owns it)
//   dynamic && !physics        -> Kinematic body (grabbable, solid, inert)
//   collision = false          -> no body (no occlusion, no collision)


static bool joltInit() {
    // Keyed on the system itself, not a once-flag: a build with
    // STB_GNU_UNIQUE symbols (anything not built with -fno-gnu-unique) is
    // never unmapped, and loading the same path again re-runs PLUGIN_INIT
    // on these statics after joltShutdown.
    if (g_joltSystem)
        return true;

    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();

    g_joltSystem = new JPH::PhysicsSystem();
    g_joltSystem->Init(512, 0, 1024, 1024, g_bpLayers, g_objVsBp, g_objPair);
    // Same gravity as the player's walk physics.
    g_joltSystem->SetGravity(JPH::Vec3(0.f, -14.f, 0.f));

    g_bodyIf = &g_joltSystem->GetBodyInterface();
    return true;
}

static void joltShutdown() {
    // A shape still cooking finishes first; Jolt cannot stop mid-build.
    g_shapeJobs.clear();
    g_bodyIf = nullptr;

    if (g_joltSystem) {
        delete g_joltSystem;
        g_joltSystem = nullptr;
    }
    if (JPH::Factory::sInstance) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

// --- the player's own physics body ------------------------------------------
// A dynamic Jolt capsule driven by per-frame velocity control: the keys own
// the horizontal axes (the inertia glide computes s_moveVel), Jolt owns
// gravity, contacts and the pose. Active physics bodies meet it through the
// exact same contact pipeline they use between themselves -- it can push
// them, stand on them, be stood on. Rotation is locked out with
// translation-only DOFs: the solver keeps the capsule upright, and the look
// direction stays in our own yaw/pitch angles.
static JPH::BodyID g_playerBody{};      // invalid until joltInit succeeds
static JPH::BodyID g_floorBody{};       // grid platform slab (world.grid)
static bool        g_playerJumpQueued = false;

// Body center -> eye offset: the eye rides kEyeHeight above the feet.
static constexpr float PLAYER_EYE_OFF =
    Camera::kEyeHeight - Camera::kBodyHeight * 0.5f;

static void ensurePlayerBody() {
    if (!g_joltSystem || !g_playerBody.IsInvalid())
        return;

    JPH::CapsuleShapeSettings CAPSULE(
        Camera::kBodyHeight * 0.5f - Camera::kBodyHalfWidth, // cylinder half
        Camera::kBodyHalfWidth);                             // -> 0.6x1.8
    CAPSULE.SetEmbedded();
    auto RES = CAPSULE.Create();
    if (RES.HasError())
        return;

    // Spawned under the camera (the camera was placed by the spawn reset).
    const auto& CAM = g_scene.camera();
    JPH::BodyCreationSettings BCS(
        RES.Get(),
        JPH::RVec3(CAM.position.x, CAM.position.y - PLAYER_EYE_OFF,
                   CAM.position.z),
        JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, LAYER_MOVING);
    BCS.mAllowedDOFs   = JPH::EAllowedDOFs::TranslationX |
                         JPH::EAllowedDOFs::TranslationY |
                         JPH::EAllowedDOFs::TranslationZ;
    BCS.mMotionQuality = JPH::EMotionQuality::LinearCast; // cast the step:
                          // never tunnels the map's zero-thickness sheets
    // Friction ZERO. The commanded into-wall velocity makes the solver
    // cancel ~5 m/s every sub-step -- a normal force on the order of
    // 450*m -- and any friction then lets the wall hold the player against
    // gravity (measured: a -0.16 m/s creep instead of free fall) and
    // stick-slip while sliding (the wall-press shaking). The player is
    // velocity-driven: friction only ever hurt.
    BCS.mFriction      = 0.0f;
    BCS.mRestitution   = 0.0f;
    BCS.mAllowSleeping = false; // always controlled
    BCS.mGravityFactor = 0.0f;  // set per-frame by the movement mode

    auto* B = g_bodyIf->CreateBody(BCS);
    if (!B)
        return;
    g_bodyIf->AddBody(B->GetID(), JPH::EActivation::Activate);
    g_playerBody = B->GetID();
}

// The grid platform as real physics: a slab matching the visible grid
// extent (the lines run -20..20), top at world zero. Both the player and
// physics objects land on it.
static void syncFloorBody() {
    if (!g_joltSystem)
        return;

    if (g_cfgGrid && g_floorBody.IsInvalid()) {
        JPH::BoxShapeSettings SLAB(JPH::Vec3(20.0f, 0.5f, 20.0f));
        SLAB.SetEmbedded();
        auto RES = SLAB.Create();
        if (RES.HasError())
            return;

        JPH::BodyCreationSettings BCS(
            RES.Get(), JPH::RVec3(0.f, -0.5f, 0.f), JPH::Quat::sIdentity(),
            JPH::EMotionType::Static, LAYER_STATIC);
        auto* B = g_bodyIf->CreateBody(BCS);
        if (!B)
            return;
        g_bodyIf->AddBody(B->GetID(), JPH::EActivation::DontActivate);
        g_floorBody = B->GetID();
    } else if (!g_cfgGrid && !g_floorBody.IsInvalid()) {
        g_bodyIf->RemoveBody(g_floorBody);
        g_bodyIf->DestroyBody(g_floorBody);
        g_floorBody = JPH::BodyID();
    }
}

// Ground probe: a short ray straight down from the body center (feet with
// 0.15 slack). Back-face culling skips the capsule's own underside.
static bool playerGrounded() {
    if (!g_joltSystem || g_playerBody.IsInvalid())
        return false;

    const auto POS = g_bodyIf->GetPosition(g_playerBody);
    const JPH::RRayCast RAY{
        POS, JPH::Vec3(0.f, -(Camera::kBodyHeight * 0.5f + 0.15f), 0.f)};
    JPH::RayCastResult HIT;
    // The ray STARTS INSIDE the player capsule: unfiltered, the cast reports
    // the player's own shape at fraction 0 and grounded is true for the
    // whole flight (flappy-bird jumps). Exclude self.
    const JPH::IgnoreSingleBodyFilter SKIP_SELF(g_playerBody);
    return g_joltSystem->GetNarrowPhaseQuery().CastRay(
        RAY, HIT, JPH::BroadPhaseLayerFilter(), JPH::ObjectLayerFilter(),
        SKIP_SELF);
}

// Euler (degrees, our RY*RX*RZ convention) -> Jolt quaternion.
static JPH::Quat eulerToQuat(const Vec3& rotationDeg) {
    constexpr float DEG = 3.14159265358979f / 180.0f;
    const float Y = rotationDeg.y * DEG, X = rotationDeg.x * DEG,
                Z = rotationDeg.z * DEG;

    JPH::Quat Q = JPH::Quat::sRotation(JPH::Vec3::sAxisY(), Y);
    Q = Q * JPH::Quat::sRotation(JPH::Vec3::sAxisX(), X);
    Q = Q * JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), Z);
    return Q;
}

// Jolt quaternion -> Euler (degrees), the inverse of eulerToQuat: for
// R = RY(yaw)*RX(pitch)*RZ(roll), pitch = asin(-R12), yaw = atan2(R02, R00),
// roll = atan2(R10, R11) with R in math row-major.
static Vec3 quatToEuler(const JPH::Quat& q) {
    const JPH::Mat44 M = JPH::Mat44::sRotation(q);
    const JPH::Vec3 C0 = M.GetColumn3(0);
    const JPH::Vec3 C1 = M.GetColumn3(1);
    const JPH::Vec3 C2 = M.GetColumn3(2);

    constexpr float RAD = 180.0f / 3.14159265358979f;

    return Vec3{
        std::asin(std::clamp(-C2.GetY(), -1.0f, 1.0f)) * RAD,
        std::atan2(C2.GetX(), C2.GetZ()) * RAD,
        std::atan2(C0.GetY(), C1.GetY()) * RAD,
    };
}

static void joltSyncBodies() {
    g_joltBodies.resize(g_sceneObjects.size());
    g_shapeJobs.resize(g_sceneObjects.size());

    for (size_t i = 0; i < g_sceneObjects.size(); ++i) {
        auto& JB = g_joltBodies[i];
        const auto& OBJ = g_sceneObjects[i];
        const auto* MODEL = g_scene.sceneModel(i);

        const bool WANT = OBJ.collision && MODEL && MODEL->loaded() &&
            !MODEL->triangles().empty();
        const uint32_t MESH_VER = MODEL ? MODEL->meshVersion() : 0;

        if (!WANT) {
            g_shapeJobs[i].reset();
            if (JB.valid) {
                g_bodyIf->RemoveBody(JB.body);
                g_bodyIf->DestroyBody(JB.body);
                JB = {};
            }
            continue;
        }

        // Rebuild ONLY on a mesh-content change (or a body-space mapping
        // change: scale/pivot edits): the generation also bumps on every
        // transform move, and a falling object would otherwise recreate its
        // ConvexHull every frame (the 0.5s-per-tick freeze).
        const Vec3 SCL = MODEL->scale();
        const Vec3 PIV = MODEL->pivot();
        const bool MAPPING_CHANGED =
            JB.builtScale.x != SCL.x || JB.builtScale.y != SCL.y ||
            JB.builtScale.z != SCL.z || JB.builtPivot.x != PIV.x ||
            JB.builtPivot.y != PIV.y || JB.builtPivot.z != PIV.z;

        if (!JB.valid || JB.meshGen != MESH_VER || MAPPING_CHANGED) {
            // Body-space triangles: the mesh scaled, with the PIVOT at the
            // body origin. The body transform (config position/rotation)
            // then maps body space onto the world exactly like the render
            // matrix does -- collision follows the visual for ANY config
            // transform, and a center_offset edit only moves the pivot.
            // (World-baked vertices here would apply the body transform a
            // SECOND time: an object placed at {2,0,1} got its collision at
            // {4,0,2}.)
            const auto BODY_PT = [SCL, PIV](const Vec3& p) {
                return Vec3{SCL.x * p.x - SCL.x * PIV.x,
                            SCL.y * p.y - SCL.y * PIV.y,
                            SCL.z * p.z - SCL.z * PIV.z};
            };

            const auto& LOCAL = MODEL->localTriangles();

            const bool DYN = OBJ.dynamic && OBJ.physics;
            const JPH::EMotionType MOTION =
                OBJ.dynamic ? (OBJ.physics ? JPH::EMotionType::Dynamic
                                           : JPH::EMotionType::Kinematic)
                            : JPH::EMotionType::Static;
            const JPH::ObjectLayer LAYER =
                MOTION == JPH::EMotionType::Static ? LAYER_STATIC : LAYER_MOVING;

            JPH::Ref<JPH::Shape> SHAPE;
            if (MOTION == JPH::EMotionType::Static) {
                // Static meshes keep their exact triangle soup, cooked on a
                // worker. The old body stays until the new shape is there.
                auto& JOB = g_shapeJobs[i];
                const bool CURRENT = JOB && JOB->meshGen == MESH_VER &&
                    JOB->scale.x == SCL.x && JOB->scale.y == SCL.y &&
                    JOB->scale.z == SCL.z && JOB->pivot.x == PIV.x &&
                    JOB->pivot.y == PIV.y && JOB->pivot.z == PIV.z;

                if (!CURRENT) {
                    JOB.reset(); // an outdated one is waited for
                    JOB          = std::make_unique<SShapeJob>();
                    JOB->meshGen = MESH_VER;
                    JOB->scale   = SCL;
                    JOB->pivot   = PIV;
                    JOB->thread  = std::jthread([J = JOB.get(), LOCAL, BODY_PT] {
                        // An exception leaving this thread would terminate
                        // Hyprland.
                        try {
                            JPH::TriangleList TL;
                            // Two-sided: each triangle twice (both windings)
                            // -- Jolt's narrow phase ignores back faces, and
                            // single-sided authored geometry (ceilings!)
                            // would let bodies tunnel through.
                            TL.reserve(LOCAL.size() * 2);
                            for (const auto& T : LOCAL) {
                                const Vec3 A = BODY_PT(T.a), B = BODY_PT(T.b),
                                           C = BODY_PT(T.c);
                                TL.push_back(JPH::Triangle(
                                    JPH::Float3(A.x, A.y, A.z),
                                    JPH::Float3(B.x, B.y, B.z),
                                    JPH::Float3(C.x, C.y, C.z)));
                                TL.push_back(JPH::Triangle(
                                    JPH::Float3(A.x, A.y, A.z),
                                    JPH::Float3(C.x, C.y, C.z),
                                    JPH::Float3(B.x, B.y, B.z)));
                            }

                            JPH::MeshShapeSettings SETTINGS(TL);
                            SETTINGS.SetEmbedded();
                            auto RES = SETTINGS.Create();
                            if (!RES.HasError())
                                J->shape = RES.Get();
                        } catch (...) {
                            J->shape = nullptr;
                        }
                        J->done.store(true, std::memory_order_release);
                    });
                    continue;
                }

                // Still cooking, or Jolt refused this mesh (kept, so it is
                // not tried again until the mesh changes).
                if (!JOB->done.load(std::memory_order_acquire) || !JOB->shape)
                    continue;

                JOB->thread.join();
                SHAPE = JOB->shape;
                JOB.reset();
            } else {
                if (JB.valid) {
                    g_bodyIf->RemoveBody(JB.body);
                    g_bodyIf->DestroyBody(JB.body);
                    JB = {};
                }

                // Dynamic/kinematic bodies use a convex hull (Jolt requires
                // it; the hull also tumbles believably). Dense meshes are
                // sampled down first: hulling every vertex of a ~2M-triangle
                // model (6M points) stalled the compositor, and a hull of a
                // few tens of thousands of points looks the same.
                constexpr size_t MAX_HULL_TRIS = 20000;
                const size_t STRIDE =
                    std::max<size_t>(1, LOCAL.size() / MAX_HULL_TRIS);

                JPH::Array<JPH::Vec3> POINTS;
                POINTS.reserve((LOCAL.size() / STRIDE + 1) * 3);
                for (size_t t = 0; t < LOCAL.size(); t += STRIDE)
                    for (const Vec3* P : {&LOCAL[t].a, &LOCAL[t].b, &LOCAL[t].c}) {
                        const Vec3 Q = BODY_PT(*P);
                        POINTS.push_back(JPH::Vec3(Q.x, Q.y, Q.z));
                    }

                JPH::ConvexHullShapeSettings HS(POINTS);
                HS.SetEmbedded();
                auto RES = HS.Create();
                if (RES.HasError())
                    continue;
                SHAPE = RES.Get();
            }

            if (JB.valid) {
                g_bodyIf->RemoveBody(JB.body);
                g_bodyIf->DestroyBody(JB.body);
                JB = {};
            }

            JPH::BodyCreationSettings BCS(
                SHAPE,
                JPH::RVec3(OBJ.position.x, OBJ.position.y, OBJ.position.z),
                eulerToQuat(OBJ.rotationDeg), MOTION,
                i == g_mapGrabIndex ? LAYER_GRABBED : LAYER);
            BCS.mLinearDamping  = 0.05f;
            BCS.mAngularDamping = 0.05f;
            BCS.mAllowSleeping  = true;
            BCS.mFriction       = 0.6f;
            BCS.mRestitution    = 0.05f;

            JPH::Body* B = g_bodyIf->CreateBody(BCS);
            g_bodyIf->AddBody(B->GetID(), JPH::EActivation::Activate);

            JB.body       = B->GetID();
            JB.meshGen    = MESH_VER;
            JB.builtScale = SCL;
            JB.builtPivot = PIV;
            JB.valid      = true;
        } else {
            // Config-owned objects (static/kinematic/no-physics): follow the
            // config transform if it moved (grab, config edit).
            const auto WANT = JPH::RVec3(OBJ.position.x, OBJ.position.y,
                                         OBJ.position.z);
            const auto CUR = g_bodyIf->GetPosition(JB.body);

            g_bodyIf->SetPositionAndRotationWhenChanged(
                JB.body, WANT, eulerToQuat(OBJ.rotationDeg),
                JPH::EActivation::DontActivate);
        }
    }
}

// Nearest scene-model hit of a ray. Occlusion (windows behind models lose
// clicks/focus/aim) and the model gestures both go through this: collision-
// enabled objects answer through their BVH trees (fast), a carried object is
// skipped (it is on the crosshair already). collision = false objects neither
// occlude nor grab.
struct SModelRayHit {
    bool   hit = false;
    size_t index = SIZE_MAX;
    float  dist = 0.0f;
    bool   dynamic = false;
};

static SModelRayHit modelRayHit(const Vec3& origin, const Vec3& dir,
                                bool dynamicOnly) {
    SModelRayHit R;

    for (size_t i = 0; i < g_sceneObjects.size(); ++i) {
        if (i >= g_objTrees.size())
            break; // config reloaded mid-frame: the trees lag one resize

        if (dynamicOnly && !g_sceneObjects[i].dynamic)
            continue;
        if (!g_sceneObjects[i].collision)
            continue;
        if (i == g_mapGrabIndex)
            continue;

        float t = -1.f;
        if (!g_objTrees[i].empty())
            t = g_objTrees[i].rayCast(origin, dir);
        else if (const auto* MODEL = g_scene.sceneModel(i);
                 MODEL && MODEL->loaded())
            t = MODEL->rayCast(origin, dir);

        if (t > 0.f && (!R.hit || t < R.dist)) {
            R.hit     = true;
            R.index   = i;
            R.dist    = t;
            R.dynamic = g_sceneObjects[i].dynamic;
        }
    }

    return R;
}

// A model is IN FRONT of the aimed window when its ray hit is closer (or
// there is no window hit at all).
// --- characters: placement, collision, carrying ------------------------------
//
// Each character (config: characters = { ... }) stands on its feet position
// with an upright Jolt cylinder around it: a flat bottom on the floor it was
// placed on, kinematic, so it blocks the player but never tumbles. Super +
// left-drag carries it: it stays upright, turns to face the player, and its
// feet follow the ground under the crosshair. The runtime placement survives
// config reloads unless the config's own position/rotation changed (the same
// rule as carried scene objects).

static void damageCurrentMonitor();

struct SCharState {
    GLScene::SCharacterSpec cfg;        // as parsed: the file's truth
    Vec3                    feet{};     // runtime placement
    float                   yawDeg = 0; // runtime facing (Y), degrees
    float                   radius = 0.3f, height = 1.8f;
    JPH::BodyID             body{};
};
static std::vector<SCharState> g_chars;
static size_t                  g_charGrabIndex = SIZE_MAX;
static float                   g_charGrabDist  = 0.0f;

// Ground height under a point: the nearest collidable scene geometry below
// it, else the grid platform (world zero), else `fallback`.
static float groundBelow(const Vec3& p, float fallback) {
    const Vec3 FROM{p.x, p.y + 0.5f, p.z};
    const auto HIT = modelRayHit(FROM, Vec3{0.f, -1.f, 0.f}, false);
    if (HIT.hit)
        return FROM.y - HIT.dist;
    return g_cfgGrid ? Camera::kFloorY : fallback;
}

// Nearest positive hit of a ray on an upright cylinder standing on `feet`
// (sides and both caps), or -1.
static float rayUprightCylinder(const Vec3& o, const Vec3& d, const Vec3& feet,
                                float r, float h) {
    float best = -1.f;
    const auto KEEP = [&](float t) {
        if (t > 0.f && (best < 0.f || t < best))
            best = t;
    };

    const float OX = o.x - feet.x, OZ = o.z - feet.z;
    const float A = d.x * d.x + d.z * d.z;
    if (A > 1e-8f) {
        const float B = 2.f * (OX * d.x + OZ * d.z);
        const float C = OX * OX + OZ * OZ - r * r;
        const float DISC = B * B - 4.f * A * C;
        if (DISC >= 0.f) {
            const float SQ = std::sqrt(DISC);
            for (const float T : {(-B - SQ) / (2.f * A), (-B + SQ) / (2.f * A)}) {
                const float Y = o.y + d.y * T;
                if (Y >= feet.y && Y <= feet.y + h)
                    KEEP(T);
            }
        }
    }

    if (std::fabs(d.y) > 1e-8f) {
        for (const float CAP : {feet.y, feet.y + h}) {
            const float T = (CAP - o.y) / d.y;
            const float X = o.x + d.x * T - feet.x;
            const float Z = o.z + d.z * T - feet.z;
            if (X * X + Z * Z <= r * r)
                KEEP(T);
        }
    }

    return best;
}

static void destroyCharBody(SCharState& c) {
    if (g_bodyIf && !c.body.IsInvalid()) {
        g_bodyIf->RemoveBody(c.body);
        g_bodyIf->DestroyBody(c.body);
    }
    c.body = {};
}

// Creates missing character bodies and keeps every body on its character.
static void syncCharacterBodies() {
    if (!g_joltSystem || !g_bodyIf)
        return;

    for (size_t i = 0; i < g_chars.size(); ++i) {
        auto& C = g_chars[i];
        const float HALF = C.height * 0.5f;
        const JPH::RVec3 CENTER(C.feet.x, C.feet.y + HALF, C.feet.z);
        const JPH::Quat ROT = JPH::Quat::sRotation(
            JPH::Vec3::sAxisY(), C.yawDeg * 3.14159265f / 180.f);

        if (C.body.IsInvalid()) {
            JPH::CylinderShapeSettings SHAPE(HALF, C.radius);
            auto RES = SHAPE.Create();
            if (RES.HasError())
                continue;

            JPH::BodyCreationSettings BCS(RES.Get(), CENTER, ROT,
                                          JPH::EMotionType::Kinematic,
                                          LAYER_MOVING);
            if (auto* B = g_bodyIf->CreateBody(BCS)) {
                g_bodyIf->AddBody(B->GetID(), JPH::EActivation::Activate);
                C.body = B->GetID();
            }
            continue;
        }

        g_bodyIf->SetObjectLayer(C.body, i == g_charGrabIndex ? LAYER_GRABBED
                                                              : LAYER_MOVING);
        g_bodyIf->SetPositionAndRotationWhenChanged(C.body, CENTER, ROT,
                                                    JPH::EActivation::DontActivate);
    }
}

// The character under a ray and its distance, or SIZE_MAX.
static size_t pickCharacter(const Vec3& o, const Vec3& d, float& dist) {
    size_t best = SIZE_MAX;
    for (size_t i = 0; i < g_chars.size(); ++i) {
        const auto& C = g_chars[i];
        const float T = rayUprightCylinder(o, d, C.feet, C.radius, C.height);
        if (T > 0.f && (best == SIZE_MAX || T < dist)) {
            best = i;
            dist = T;
        }
    }
    return best;
}

// Carrying: the character's feet follow the ground under the crosshair at
// the pickup distance, and it turns (yaw only) to face the player.
static void carryCharacter(float dt) {
    if (g_charGrabIndex >= g_chars.size())
        return;

    auto& C = g_chars[g_charGrabIndex];
    const auto& CAM = g_scene.camera();
    const Vec3 TARGET = CAM.position + CAM.centerRay() * g_charGrabDist;

    // Probe from above whichever is higher, the target or the eye: looking
    // down at the floor nearby puts the target below it.
    const Vec3 PROBE{TARGET.x, std::max(TARGET.y, CAM.position.y), TARGET.z};
    C.feet = Vec3{TARGET.x, groundBelow(PROBE, C.feet.y), TARGET.z};

    const float TARGET_YAW =
        std::atan2(CAM.position.x - C.feet.x, CAM.position.z - C.feet.z) *
        180.f / 3.14159265f;
    float dYaw = TARGET_YAW - C.yawDeg;
    while (dYaw > 180.f)
        dYaw -= 360.f;
    while (dYaw < -180.f)
        dYaw += 360.f;
    C.yawDeg += dYaw * (1.0f - std::exp(-10.0f * dt));

    damageCurrentMonitor();
}

// --- tools (keys 1-5) ---------------------------------------------------------
//
// Weapon-style tool slots in moving mode, shown on the HUD slot bar:
//   1 pointer (default) -- the crosshair as before
//   2 curve -- left-click drops a smooth point (the curve bends through
//              it) on the ground under the crosshair
//   3 lines -- the same with a sharp point (a corner)
//   4, 5    -- free
// Paths close themselves; new points go after the active path's end (just
// before its start). Clicking an existing point converts it to the tool's
// type and makes its path active; right-click deletes the point under the
// crosshair, else the active path's last one, and a path without points is
// gone. Pressing the active path tool's key again starts a new path. Plugin
// memory only; meant for characters to walk.

enum class ETool : int { Pointer = 1, Curve = 2, Lines = 3 };

struct SPath {
    std::vector<Vec3> points;
    std::vector<bool> smooth; // per point
};

static int                g_tool = static_cast<int>(ETool::Pointer);
static std::vector<SPath> g_paths;
static int                g_activePath = -1; // -1: the next click starts one

static bool pathTool() {
    return g_tool == static_cast<int>(ETool::Curve) ||
        g_tool == static_cast<int>(ETool::Lines);
}

// HUD labels; the path tools show which path new points go to.
static std::vector<std::string> toolLabels() {
    const std::string WHICH = g_activePath >= 0 &&
            g_activePath < static_cast<int>(g_paths.size())
        ? "P" + std::to_string(g_activePath + 1) : "NEW";
    return {"POINTER", "CURVE " + WHICH, "LINES " + WHICH, "", ""};
}

// The path point nearest a ground position (within reach), as
// (path, point), or (-1, -1).
static std::pair<int, int> pathPointNear(const Vec3& at) {
    constexpr float REACH = 0.35f;
    std::pair<int, int> best{-1, -1};
    float bestD = REACH * REACH;
    for (size_t p = 0; p < g_paths.size(); ++p)
        for (size_t i = 0; i < g_paths[p].points.size(); ++i) {
            const Vec3 D = g_paths[p].points[i] - at;
            if (std::fabs(D.y) > 0.6f)
                continue;
            const float D2 = D.x * D.x + D.z * D.z;
            if (D2 < bestD) {
                bestD = D2;
                best  = {static_cast<int>(p), static_cast<int>(i)};
            }
        }
    return best;
}

static void removePathPoint(int p, int i) {
    auto& P = g_paths[p];
    P.points.erase(P.points.begin() + i);
    P.smooth.erase(P.smooth.begin() + i);
    if (P.points.empty()) {
        g_paths.erase(g_paths.begin() + p);
        if (g_activePath == p)
            g_activePath = -1;
        else if (g_activePath > p)
            --g_activePath;
    }
}

// A path point on the ground under the crosshair: the nearest collidable
// surface (or the grid platform) the centre ray hits, dropped onto the
// ground below it -- aiming at a wall puts the point at its foot.
static bool groundUnderCrosshair(Vec3& out) {
    const auto& CAM = g_scene.camera();
    const Vec3 DIR = CAM.centerRay();

    float t = -1.f;
    if (const auto HIT = modelRayHit(CAM.position, DIR, false); HIT.hit)
        t = HIT.dist;
    if (g_cfgGrid && DIR.y < -1e-4f) {
        const float TG = (Camera::kFloorY - CAM.position.y) / DIR.y;
        if (TG > 0.f && (t < 0.f || TG < t))
            t = TG;
    }
    if (t <= 0.f)
        return false;

    const Vec3 P = CAM.position + DIR * (t - 0.02f); // just in front of it
    out = Vec3{P.x, groundBelow(Vec3{P.x, P.y + 0.05f, P.z}, P.y), P.z};
    return true;
}

static bool modelInFront(const Vec3& origin, const Vec3& dir,
                         const World3D::SHit& windowHit) {
    const auto MR = modelRayHit(origin, dir, /*dynamicOnly=*/false);
    return MR.hit && (!windowHit.hit || MR.dist < windowHit.distance);
}


static void notify(const std::string& text, const CHyprColor& color);

// Feeds the requested path to the scene every frame. Tilde is expanded, a
// missing file is reported once per path, and the scene itself reloads the
// texture when the file's mtime changes.
static void updatePanorama() {
    std::string path = g_cfgPanorama;

    if (!path.empty() && path.starts_with('~')) {
        if (const char* HOME = getenv("HOME"))
            path = std::string{HOME} + path.substr(1);
    }

    if (!path.empty()) {
        std::error_code ec;

        if (!std::filesystem::exists(path, ec)) {
            static std::string notified;

            if (notified != path) {
                notified = path;
                notify(
                    "[hypr3d] panorama file not found: " + path,
                    CHyprColor{1.0f, 0.6f, 0.2f, 1.0f}
                );
            }
            return;
        }
    }

    g_scene.setPanoramaPath(path);
}

// --- diagnostics ------------------------------------------------------------
// Written to /tmp/hypr3d-status.txt while the view is live. Both reported
// bugs (mouse not rotating, 2D desktop showing through) admit several
// plausible causes, and guessing between them from the outside cost far more
// time than one dump of the actual numbers.
static uint64_t g_diagMoveEvents = 0;
static uint64_t g_diagSinkCalls   = 0;
static Vector2D g_diagLastPos{};
static double   g_diagLastDx = 0.0;
static double   g_diagLastDy = 0.0;
static double   g_diagLastZoomStep = 0.0; // normalized wheel steps (+ = forward)
static Vector2D g_diagPinned{};
static bool     g_diagPinnedValid = false;
static bool     g_warping         = false;
static uint64_t g_diagWarpCalls   = 0;
static Vector2D g_diagWarpAfter{};
static Vector2D g_diagMgrPos{};

// How far (logical px) the cursor may stray from the crosshair before it is
// warped back. Declared with the other diagnostics so the dump and the code
// that uses it cannot drift apart.
static constexpr double kWarpRadiusPx = 8.0;
static float    g_diagAlpha = 0.0f;
static uint64_t g_diagFrames = 0;
static std::chrono::steady_clock::time_point g_diagLastDump{};

static void notify(
    const std::string& text,
    const CHyprColor& color
) {
    if (!PHANDLE)
        return;

    HyprlandAPI::addNotification(
        PHANDLE,
        text,
        color,
        3000
    );
}

static uint32_t inputTimeMs() {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - g_inputClockStart
        ).count();

    return static_cast<uint32_t>(std::max<int64_t>(0, elapsed));
}

static void damageCurrentMonitor() {
    if (!g_monitor)
        return;

    if (!g_pHyprRenderer)
        return;

    g_pHyprRenderer->damageMonitor(
        g_monitor
    );
}

static PHLMONITOR g_currentRenderMon = nullptr;
static bool       g_roomDrawnThisFrame = false; // see onRenderStage

// The monitor the 3D view is built for: configured monitor, or focused monitor,
// falling back to whatever render.pre last reported.
static PHLMONITOR targetMonitor() {
    if (!g_cfgMonitor.empty() && State::monitorState()) {
        for (const auto& mon : State::monitorState()->monitors()) {
            if (mon && mon->m_name == g_cfgMonitor)
                return mon;
        }
    }

    if (const auto FOCUSED = Compat::focusedMonitor())
        return FOCUSED;

    return g_monitor;
}

// Where the crosshair sits in logical coordinates: the centre of the monitor
// the view is built for. The pointer is pinned around this point, and the
// picking ray is independent of the cursor and comes from the camera.
static Vector2D crosshairLogical() {
    const auto MON = targetMonitor();

    if (!MON)
        return Vector2D{0.0, 0.0};

    return MON->m_position +
           Vector2D{MON->m_size.x * 0.5, MON->m_size.y * 0.5};
}

static void resetMovementKeys() {
    g_keyFwd = g_keyBack = g_keyLeft = g_keyRight = false;
    g_keyUp = g_keyDown = g_keySprint = false;
    g_superHeld = false;
}

// Clears only the camera keys, keeping the gesture modifiers honest -- used
// when switching into window mode while keys are physically held.
static void resetCameraKeys() {
    g_keyFwd = g_keyBack = g_keyLeft = g_keyRight = false;
    g_keyUp = g_keyDown = g_keySprint = false;
}

// True while the 3D view owns the pointer and the keyboard.
static bool ownsInput() {
    // In2D is the fullscreen passthrough: Hyprland owns everything.
    return g_active && g_transition >= 0.9f &&
        g_fsPhase != EFullscreenPhase::In2D;
}

// Window (typing) mode frees the pointer: the camera freezes, the real cursor
// comes back and can leave for the other monitors. Space mode captures it
// again behind the crosshair.
static bool pointerFree() {
    return g_cfgTypingCursor && g_keyboardMode == EKeyboardMode::Window;
}

// Typing-mode virtual cursor state; see the block above onRenderStage.
enum class ECursorSpace : uint8_t { Desktop, Screen, Window };

static ECursorSpace   g_vcSpace = ECursorSpace::Desktop;
static Vector2D       g_vcScreen{}; // room-monitor-local logical px
static std::uintptr_t g_vcId = 0;   // Window: the entity under the cursor
static Vector2D       g_vcLocal{};  // Window: decorated-box px, top-left origin

static bool virtualCursor() {
    return pointerFree() && g_hookInstalled;
}

// Whether the hook should swallow pointer motion right now.
static bool captureWanted() {
    if (!g_hookInstalled || !ownsInput())
        return false;

    return !pointerFree() || g_vcSpace != ECursorSpace::Desktop;
}

static void vcMove(double dx, double dy);
static void requestDeactivate3D();
static void refreshCursorImage();
static bool clientCursorImage(SP<Render::ITexture>& tex, Vector2D& size,
                              Vector2D& hotspot);

// Whether a global logical point lies on the monitor the room is drawn on.
static bool onRoomMonitor(const Vector2D& pos) {
    const auto MON = targetMonitor();

    return MON && CBox{MON->m_position, MON->m_size}.containsPoint(pos);
}

static void clearAimFocus() {
    g_aim.reset();
    g_lastFocusId = 0;
    Compat::clearFocus();
    Compat::clearPointerFocus();
}

// --- layout ghosting --------------------------------------------------------
//
// Every window's box is saved first and only then ghosted: ghosting one window
// triggers a relayout of the ones still attached, which would corrupt boxes
// captured afterwards. Ghosting is what stops tiling from managing windows
// while they are living in 3D space.
static void ghostWindows(const PHLMONITOR& mon) {
    if (!mon)
        return;

    const auto INFOS = Compat::enumerateEligibleWindows(mon, g_cfgUnderLayers);

    if (!g_ghosted) {
        // First pass: save EVERY window before ghosting any of them, since
        // ghosting one window triggers a relayout of the ones still attached,
        // which would corrupt boxes captured afterwards.
        g_layoutSaves.clear();
        g_layoutSaves.reserve(INFOS.size());

        for (const auto& info : INFOS) {
            auto SAVE = Compat::saveWindowLayout(info.window);

            if (SAVE.window)
                g_layoutSaves.push_back(std::move(SAVE));
        }

        for (auto& save : g_layoutSaves)
            Compat::applyWindowGhost(save);

        g_ghosted = true;
        return;
    }

    // Steady state: a window that mapped while the view is open must get the
    // same treatment the same frame it becomes eligible, otherwise it stays a
    // live layout target and every later spawn or close re-tiles the space
    // around it. The live weak reference guards against a new window reusing
    // a closed one's address.
    for (const auto& info : INFOS) {
        bool known = false;

        for (const auto& save : g_layoutSaves) {
            if (save.id == info.id && !save.window.expired()) {
                known = true;
                break;
            }
        }

        if (known)
            continue;

        // A window that appears while the view is open becomes a small
        // floating panel instead of a fullscreen tile -- resized BEFORE the
        // layout save, so the box restored on exit is the same 720x480 the
        // user actually worked with. setWindowBox takes GLOBAL layout
        // coordinates: centre on this monitor, not on the layout origin
        // (which belongs to whichever monitor sits at 0,0, or none at all).
        if (info.window) {
            const double CX =
                mon->m_position.x + mon->m_size.x * 0.5 - g_cfgSpawnWidth * 0.5;
            const double CY =
                mon->m_position.y + mon->m_size.y * 0.5 - g_cfgSpawnHeight * 0.5;

            Compat::setWindowBox(
                info.window,
                CBox{CX, CY, g_cfgSpawnWidth, g_cfgSpawnHeight}
            );
        }

        auto SAVE = Compat::saveWindowLayout(info.window);

        if (!SAVE.window)
            continue;

        Compat::applyWindowGhost(SAVE);
        g_layoutSaves.push_back(std::move(SAVE));
    }
}

static void unghostWindows() {
    if (!g_ghosted)
        return;

    // Force the exact saved geometry back, so leaving 3D never disturbs the
    // user's 2D arrangement. The 3D arrangement is a view, not an edit.
    for (auto& save : g_layoutSaves)
        Compat::restoreWindowLayout(save);

    g_layoutSaves.clear();
    g_ghosted = false;
}

// --- continuous 3D frame pump ----------------------------------------------

static void stopFramePump() {
    if (!g_framePump)
        return;

    if (g_pEventLoopManager)
        g_pEventLoopManager->removeTimer(g_framePump);

    g_framePump.reset();
}

static void startFramePump() {
    if (!g_pEventLoopManager)
        return;

    if (!g_framePump) {
        g_framePump = makeShared<CEventLoopTimer>(
            std::nullopt,
            [](SP<CEventLoopTimer> self, void*) {
                if (!g_active) {
                    self->updateTimeout(std::nullopt);
                    return;
                }

                // A session lock ends 3D at once. While locked Hyprland
                // draws only the lock surfaces, so a fade would never finish,
                // and a lock (with the monitor teardown that can follow) is no
                // time to keep windows ghosted out of their layout. Back in, a
                // toggle reopens it.
                if (g_pSessionLockManager &&
                    g_pSessionLockManager->isSessionLocked()) {
                    g_transitionTarget = 0.0f;
                    g_transition       = 0.0f;
                    requestDeactivate3D();
                    self->updateTimeout(kFramePumpInterval);
                    return;
                }

                // 2D passthrough: no damage (Hyprland renders by its own
                // damage); the tick still polls the fullscreen state so the
                // exit transition can fire.
                pollFullscreen();

                // Snapshot release deferred from deactivate3D: safe to
                // destroy GL objects here, outside any render pass.
                if (g_captureReleasePending && !g_active) {
                    g_capture.releaseAll();
                    g_captureReleasePending = false;
                }

                if (g_fsPhase != EFullscreenPhase::In2D)
                    damageCurrentMonitor();

                self->updateTimeout(kFramePumpInterval);
            },
            nullptr
        );

        g_pEventLoopManager->addTimer(g_framePump);
    }

    g_framePump->updateTimeout(kFramePumpInterval);
}

// --- live window capture -----------------------------------------------------

static void refreshCaptures(
    const std::vector<Compat::SWindowInfo>& infos,
    const PHLMONITOR& mon
) {
    if (infos.empty())
        return;

    std::vector<std::uintptr_t> keep;
    keep.reserve(infos.size());

    for (const auto& info : infos)
        keep.push_back(info.id);

    g_capture.retainOnly(keep);

    const auto FOCUSED = Compat::focusedWindow();
    const std::uintptr_t FOCUSED_ID = FOCUSED ? Compat::windowId(FOCUSED) : 0;

    bool consumedSkirt = false;

    for (const auto& info : infos) {
        // Focus-change feedback (the active/inactive opacity fade and the
        // border color tween) is compositor-side -- no client commit happens,
        // so the buffer-change check would freeze both animations mid-way
        // and make the window snap to its final look. While any of them
        // runs, the snapshot refreshes at full rate. The FADE and FULLSCREEN
        // channels belong to the same family: the fullscreen handoff fades
        // every non-FS window through them, and without this the last
        // snapshot keeps the MID-FADE alpha baked in -- static windows
        // stayed semi-transparent until a hover forced a repaint.
        // Alpha channels in flight force full-rate retakes -- but the LAST
        // retake must happen AFTER the animation finishes: while
        // isBeingAnimated() is still true the value can be 0.99, the
        // animation then completes and no further retake runs, leaving the
        // snapshot baked one tick short of the goal (the lingering
        // semi-transparency on static windows). So once a channel was seen
        // in flight, the window keeps forcing retakes for a grace period
        // past the end -- the final retake captures the goal state exactly.
        static std::unordered_map<std::uintptr_t,
            std::chrono::steady_clock::time_point> alphaGrace;

        const auto NOW = std::chrono::steady_clock::now();

        const bool FADING = info.window &&
            (info.window->alpha(Desktop::View::WINDOW_ALPHA_ACTIVE)
                 ->isBeingAnimated() ||
                info.window->alpha(Desktop::View::WINDOW_ALPHA_FADE)
                 ->isBeingAnimated() ||
                info.window->alpha(Desktop::View::WINDOW_ALPHA_FULLSCREEN)
                 ->isBeingAnimated() ||
                info.window->m_borderFadeAnimationProgress->isBeingAnimated());

        if (FADING)
            alphaGrace[info.id] = NOW + std::chrono::milliseconds(200);
        else if (auto IT = alphaGrace.find(info.id);
                 IT != alphaGrace.end() && NOW > IT->second)
            alphaGrace.erase(IT); // expired: keep the map from growing

        const bool ALPHA_GRACE = info.window &&
            alphaGrace.count(info.id) > 0;

        const bool FORCE =
            FADING ||
            ALPHA_GRACE ||
            info.id == g_lastAimedId ||
            (g_world.dragActive() && g_world.draggedId() == info.id) ||
            (g_resize.active && g_resize.id == info.id) ||
            info.id == FOCUSED_ID;

        bool consumedSkirt = false;

        // The frame after a resize gesture ended: one forced snapshot, whose
        // only purpose is the settled full-resolution silhouette refresh.
        bool finalSkirt = false;
        if (info.id == g_skirtFinalRefreshId) {
            finalSkirt = true;
            consumedSkirt = true;
        }

        if (info.isLayer)
            g_capture.makeSnapshotLayer(info.layer, mon, FORCE || finalSkirt);
        else
            g_capture.makeSnapshot(info.window, mon, FORCE || finalSkirt);
    }

    if (consumedSkirt)
        g_skirtFinalRefreshId = 0;

    ++g_captureFrames;
}

// Called from render.pre. Hyprland emits render.pre before beginRender() and
// before the compositor's main render pass, so makeSnapshotFB can safely open
// its own offscreen render here. Nested snapshot renders set g_capturing and
// therefore skip all plugin rendering callbacks.
static void serviceCapture() {
    if (!g_active || g_capturing || !g_pHyprRenderer)
        return;

    // 2D passthrough: the room is dormant, no captures needed.
    if (g_fsPhase == EFullscreenPhase::In2D)
        return;

    const auto MON = targetMonitor();
    if (!MON)
        return;

    g_capturing = true;

    ghostWindows(MON);
    refreshCaptures(Compat::enumerateEligibleWindows(MON, g_cfgUnderLayers), MON);

    g_capturing = false;
}

static World3D::SHit aimHit();

// Handoff the keyboard to whatever the crosshair is on -- and to nothing else.
// Aiming at empty space drops focus entirely rather than leaving the last
// window typed into.
static void updateAimFocus(float dt) {
    // Typing mode: the virtual cursor owns focus (focus follows it).
    if (g_pointerDown || g_clientButtonDown || pointerFree() ||
        Compat::layerHasKeyboardFocus())
        return;

    const auto& cam = g_scene.camera();

    const World3D::SHit HIT = aimHit();

    // A scene model in front of the aimed window occludes it: no window
    // focus through geometry (one distance space for both classes).
    std::uintptr_t AIMED = HIT.hit ? HIT.id : 0;
    if (AIMED != 0 && modelInFront(cam.position, cam.centerRay(), HIT))
        AIMED = 0;
    g_lastAimedId = AIMED;

    const std::uintptr_t FOCUS = g_aim.update(AIMED, dt);

    if (FOCUS == g_lastFocusId)
        return;

    const auto WINDOW =
        FOCUS != 0 ? Compat::findWindowById(FOCUS) : nullptr;

    if (WINDOW) {
        Compat::focusWindow(WINDOW);
        g_lastFocusId = FOCUS;
    }
    else {
        Compat::clearFocus();
        g_lastFocusId = 0;
    }
}

// Raw (pre-smoothstep) fullscreen transition progress. Shared so syncWorld
// and applyFullscreenAnimation agree on the same instant.
static float fsRawProgress() {
    return std::clamp(
        std::chrono::duration<float>(std::chrono::steady_clock::now() -
                                     g_fsPhaseStart).count() /
            kFsAnimDuration,
        0.0f, 1.0f);
}

// The room's uniform window scale. Shared by syncWorld (quad size) and the
// fullscreen animation (its endpoints must match what the room renders, or
// the transition visibly jumps).
static float configWindowScale() {
    return g_cfgWindowScale;
}

static void syncWorld(const PHLMONITOR& mon, float dt) {
    // Deliberately free of side effects on the layout and the renderer. This
    // runs from render.stage, i.e. between the frame's startRenderPass() and
    // endRender(). Ghosting would trigger a relayout while the pass is half
    // built, and makeSnapshotFB opens and closes its OWN nested pass
    // (beginFullFakeRender -> startRenderPass -> endRender), which rebinds the
    // monitor blur FBs; the compositor's outer endRender() then aborted in
    // CMonitor::useFP16(). Capture and ghosting therefore happen outside the
    // frame -- see serviceCapture().
    const auto INFOS = Compat::enumerateEligibleWindows(mon, g_cfgUnderLayers);

    g_winOutlines.clear();

    const float MONW = mon->m_size.x;
    const float MONH = mon->m_size.y;

    const float WIN_SCALE = configWindowScale();

    std::vector<World3D::SEntity> ENTITIES;
    ENTITIES.reserve(INFOS.size());

    // Windows new to the room in this rebuild -- all of them when the view
    // is turned on, except those with a remembered pose from an earlier
    // session (g_rememberedPoses) -- stand side by side on an arc around the camera at the
    // spawn distance, in the order the 2D layout had them, left to right
    // (then top to bottom). All at the one spawn point, they covered each
    // other and the room showed a single panel. One alone still spawns
    // straight ahead. Each slot's angle is the arc length of the windows
    // before it over the radius, with a gap between neighbours.
    std::unordered_map<std::uintptr_t, float> freshAngle;
    {
        std::vector<const Compat::SWindowInfo*> FRESH;
        for (const auto& info : INFOS)
            if (!info.isLayer && !g_world.find(info.id) && !rememberedPose(info))
                FRESH.push_back(&info);

        if (FRESH.size() > 1) {
            std::sort(FRESH.begin(), FRESH.end(), [](const auto* a, const auto* b) {
                if (a->monitorLocalBox.x != b->monitorLocalBox.x)
                    return a->monitorLocalBox.x < b->monitorLocalBox.x;
                return a->monitorLocalBox.y < b->monitorLocalBox.y;
            });

            const float RADIUS = std::max(g_cfgSpawnDistance, 0.5f);
            const float GAP    = World3D::toWorld(48.0f) * WIN_SCALE;
            float       total  = GAP * static_cast<float>(FRESH.size() - 1);
            for (const auto* f : FRESH)
                total += World3D::toWorld(f->monitorLocalBox.w) * WIN_SCALE;

            float along = -total * 0.5f;
            for (const auto* f : FRESH) {
                const float W = World3D::toWorld(f->monitorLocalBox.w) * WIN_SCALE;
                freshAngle[f->id] = (along + W * 0.5f) / RADIUS;
                along += W + GAP;
            }
        }
    }

    for (const auto& info : INFOS) {
        // Track each window's last stable (non-transition) box -- see
        // g_fsStableBoxes. Skipped while the exit re-assert is running: a
        // late Hyprland-side restore could pollute the memory with the
        // monitor-sized box, and the NEXT fullscreen cycle would then
        // restore the giant size (the intermittent bug).
        if (!info.isLayer && info.window &&
            g_fsPhase == EFullscreenPhase::None && g_fsAssertFrames == 0 &&
            info.id != g_fsCurrentId) // a fullscreened window's box is the
                                      // monitor -- transient, never "stable"
            g_fsStableBoxes[info.id] = Compat::currentWindowBox(info.window);

        // Not captured yet means not on screen, and something that is not on
        // screen must not be aimable -- otherwise focus could land on an
        // invisible window.
        if (!g_capture.has(info.id))
            continue;

        const auto* SNAPSHOT = g_capture.get(info.id);

        if (!SNAPSHOT)
            continue;

        // Everything below uses the geometry captured WITH the snapshot, not
        // the live box: updateRealResize writes the real window after
        // render.pre, so the live box can be a frame ahead of the captured
        // pixels. Quad, UV subrect and picking all follow the snapshot box,
        // which keeps the drawn content and the crosshair mapping aligned
        // during resizes -- otherwise the edges smear across the frame delta.
        const CBox& BOX = SNAPSHOT->sampledBox;

        World3D::SEntity entity;
        entity.id = info.id;

        // BOX is where the content sits INSIDE the captured texture (it is
        // the real box unless the window was pulled on-screen for the
        // snapshot), so the UV subrect must follow it.
        entity.logicalLeft   = BOX.x;
        entity.logicalTop    = BOX.y;
        entity.logicalWidth  = BOX.w;
        entity.logicalHeight = BOX.h;

        // Where the client surface sits inside the full decorated box. The
        // quad spans content + borders, so surface-local input is the hit
        // position minus this offset -- borders render but never shift input.
        entity.surfaceOffsetX = SNAPSHOT->surfaceOffset.x;
        entity.surfaceOffsetY = SNAPSHOT->surfaceOffset.y;
        entity.surfaceWidth   = SNAPSHOT->surfaceSize.x;
        entity.surfaceHeight  = SNAPSHOT->surfaceSize.y;

        // Depth-slab FALLBACK silhouette for WINDOWS: an analytic rounded
        // rectangle of the decorated box -- the corner radius the compositor
        // reports plus the border inset. Used only when the texture-traced
        // outline is unavailable (the GPU copy failed or found no shape);
        // the primary source lives in the capture layer.
        if (!info.isLayer && info.window) {
            const float R = std::max(
                0.0f, info.window->rounding() +
                          static_cast<float>(SNAPSHOT->surfaceOffset.x));

            auto& C = g_winOutlines[info.id];

            if (!C.loops || C.w != static_cast<int>(BOX.w) ||
                C.h != static_cast<int>(BOX.h) || C.r != static_cast<int>(R)) {
                C.w = static_cast<int>(BOX.w);
                C.h = static_cast<int>(BOX.h);
                C.r = static_cast<int>(R);

                SOutlineLoop LOOP =
                    roundedRectLoop(BOX.w, BOX.h, R, 3.0f, 10);

                C.loops = std::make_shared<const std::vector<SOutlineLoop>>(
                    LOOP.pts.size() >= 3
                        ? std::vector<SOutlineLoop>{std::move(LOOP)}
                        : std::vector<SOutlineLoop>{});
            }
        }

        // Uniform window scale from config, applied to every entity every
        // frame so a runtime change resizes the whole room. The scale is
        // uniform so the captured content never distorts.
        entity.spawnScale = WIN_SCALE;

        // Seed pose: existing entities own their world position and rotation.
        // NEW windows spawn straight in front of the camera at a fixed read
        // distance, facing it.
        if (const auto* EXISTING = g_world.find(info.id)) {
            entity.center = EXISTING->center;
            entity.yaw = EXISTING->yaw;
            entity.pitch = EXISTING->pitch;
            entity.roll = EXISTING->roll;
        }
        else if (const auto* SAVED = rememberedPose(info)) {
            // Back from an earlier 3D session: where it was left.
            entity.center = SAVED->center;
            entity.yaw    = SAVED->yaw;
            entity.pitch  = SAVED->pitch;
            entity.roll   = SAVED->roll;
        }
        else {
            const auto& CAM = g_scene.camera();
            Vec3 FWD = CAM.forward();

            // Its slot on the arc (see freshAngle): turned about the
            // camera's up, to the right for a positive angle.
            if (const auto SLOT = freshAngle.find(info.id); SLOT != freshAngle.end()) {
                const Vec3 RIGHT = CAM.right();
                FWD = normalize(FWD * std::cos(SLOT->second) + RIGHT * std::sin(SLOT->second));
            }

            entity.center = CAM.position + FWD * g_cfgSpawnDistance;

            // Face the camera: with this model's convention (the normal's Y
            // component is -sin(pitch)) the target is the camera's own yaw
            // and pitch.
            entity.yaw   = std::atan2(-FWD.x, -FWD.z);
            entity.pitch = std::asin(std::clamp(FWD.y, -1.0f, 1.0f));
        }

        // The quad size ALWAYS follows the snapshot box (times the spawn
        // scale): the UV subrect and the input mapping are box-relative, so a
        // stale world size would squish the content and shrink the input zone
        // on every resize.
        entity.width  = World3D::toWorld(BOX.w) * entity.spawnScale;
        entity.height = World3D::toWorld(BOX.h) * entity.spawnScale;

        // Fullscreen transition: the animation owns this quad's size, and it
        // MUST be applied here -- the draw list below is built from these
        // values, so overriding later in applyFullscreenAnimation only
        // reaches the render a frame late. That late frame is what leaked
        // the config scale into the monitor-facing handoff and left a seam.
        // The scale glides between the room's config scale and the screen's
        // exact 1:1, following the same eased progress as the box lerp.
        if (g_fsPhase != EFullscreenPhase::None) {
            if (auto FSW = g_fsWindow.lock();
                FSW && Compat::windowId(FSW) == info.id) {
                const float RAWP = fsRawProgress();
                const float p = RAWP * RAWP * (3.0f - 2.0f * RAWP);
                const float ROOM_S = configWindowScale();
                const float S0 =
                    g_fsPhase == EFullscreenPhase::To2D ? ROOM_S : 1.0f;
                const float S1 =
                    g_fsPhase == EFullscreenPhase::To2D ? 1.0f : ROOM_S;
                const float S = S0 + (S1 - S0) * p;

                entity.width  = World3D::toWorld(BOX.w) * S;
                entity.height = World3D::toWorld(BOX.h) * S;
            }
        }

        if (g_resize.active && g_resize.id == info.id) {
            const float DW = entity.width - g_resize.startWorldWidth;
            const float DH = entity.height - g_resize.startWorldHeight;

            entity.center = g_resize.startCenter;
            entity.center += g_world.rightOf(info.id) *
                (static_cast<float>(g_resize.edgeX) * DW * 0.5f);
            entity.center += g_world.upOf(info.id) *
                (static_cast<float>(g_resize.edgeY) * DH * 0.5f);
        }

        ENTITIES.push_back(entity);
    }

    g_world.setEntities(std::move(ENTITIES), false);

    // --- build the draw list from world + snapshot ---
    g_renderWindows.clear();
    g_renderWindows.reserve(g_world.entities().size());

    for (const auto& entity : g_world.entities()) {
        const auto* SNAPSHOT = g_capture.get(entity.id);

        if (!SNAPSHOT || (SNAPSHOT->texID == 0 && !SNAPSHOT->bigTex))
            continue;

        // The captured texture spans texSpan -- the monitor size, or the
        // enlarged virtual monitor when the window did not fit. Dividing by
        // the live monitor size would rescale an oversized window's content
        // and distort it.
        const float SPANW = SNAPSHOT->texSpan.x;
        const float SPANH = SNAPSHOT->texSpan.y;

        if (SPANW <= 0.0f || SPANH <= 0.0f)
            continue;

        GLScene::WindowRender render;
        render.id      = entity.id;
        render.texture = SNAPSHOT->bigTex ? SNAPSHOT->bigTex : SNAPSHOT->texID;

        render.x = entity.center.x;
        render.y = entity.center.y;
        render.z = entity.center.z;
        render.yaw = entity.yaw;
        render.pitch = entity.pitch;
        render.roll = entity.roll;

        render.width  = entity.width;
        render.height = entity.height;

        // The room fade (see g_fsFade): every non-FS window rides it. The
        // fullscreen window must NEVER ride it, in any of its three
        // identities: the one fullscreened right now (g_fsCurrentId), the
        // one being animated back to the room (To3D -- FSW is already null
        // there, but g_fsWindow still holds it), and the one that JUST
        // exited (g_fsLastFSWindow persists past the exit -- without this
        // the ex-FS panel fades in together with everyone else even though
        // it was never hidden and must sit at 100% immediately).
        const auto FS_WIN      = g_fsWindow.lock();
        const auto LAST_FS_WIN = g_fsLastFSWindow.lock();

        const bool IS_FS_WINDOW =
            entity.id == g_fsCurrentId ||
            (FS_WIN && entity.id == Compat::windowId(FS_WIN)) ||
            (LAST_FS_WIN && entity.id == Compat::windowId(LAST_FS_WIN));

        render.alpha = IS_FS_WINDOW ? 1.0f : g_fsFade;

        // Depth slab (windows.depth): silhouette source priority -- the
        // outline traced from the snapshot texture's real alpha (exact
        // corner shape; the GPU-side copy is the only reliable read), then
        // the analytic rounded rect (mask refresh failures, first frames),
        // then GLScene's plain box.
        render.depth = g_cfgWindowDepth;

        if (SNAPSHOT->outlines && !SNAPSHOT->outlines->empty())
            render.outlines = SNAPSHOT->outlines;
        else if (auto IT = g_winOutlines.find(entity.id);
                 IT != g_winOutlines.end() && IT->second.loops &&
                 !IT->second.loops->empty())
            render.outlines = IT->second.loops;

        // The snapshot framebuffer covers the whole monitor, so the window is
        // a subrect of it. Hyprland renders its framebuffers with logical Y
        // (top-left origin, growing down) mapped straight into NDC, so the
        // resulting texture is TOP-DOWN: v = 0 is the monitor's top row and
        // v = logicalY / monH. The quad convention here is v0 at the world
        // bottom of the window, so the content's bottom edge (logicalY =
        // top + height) must feed v0. Flipping with 1 - (...) instead sampled
        // a mirrored band and rendered windows upside down relative to the
        // picked coordinates.
        render.u0 = std::clamp(entity.logicalLeft / SPANW, 0.0f, 1.0f);
        render.u1 = std::clamp(
            (entity.logicalLeft + entity.logicalWidth) / SPANW, 0.0f, 1.0f);
        render.v0 = std::clamp(
            (entity.logicalTop + entity.logicalHeight) / SPANH, 0.0f, 1.0f);
        render.v1 = std::clamp(entity.logicalTop / SPANH, 0.0f, 1.0f);

        g_renderWindows.push_back(render);
    }

    // Aim focus updates freeze during the fullscreen transition: the flying
    // quad sweeps the crosshair across other windows and would thrash focus.
    if (g_fsPhase == EFullscreenPhase::None)
        updateAimFocus(dt);
}

// Window-local (top-left origin, logical px) point for a crosshair hit.
static Vector2D localFromHit(const World3D::SHit& hit) {
    const auto* ENTITY = g_world.find(hit.id);
    if (!ENTITY)
        return {};

    // RayHit::u/v span the full decorated box (content + border) with a
    // top-left origin growing downwards, matching Wayland surface-local
    // coordinates. Do not invert them a second time. The client surface sits
    // inside that box at surfaceOffset, so subtracting it converts to
    // surface-local px: content maps 1:1 and the border ring clamps to the
    // surface edge, so drawing borders never shifts input.
    const float X = std::clamp(hit.u, 0.0f, 1.0f) * ENTITY->logicalWidth -
        ENTITY->surfaceOffsetX;
    const float Y = std::clamp(hit.v, 0.0f, 1.0f) * ENTITY->logicalHeight -
        ENTITY->surfaceOffsetY;

    return {
        std::clamp(X, 0.0f, ENTITY->surfaceWidth),
        std::clamp(Y, 0.0f, ENTITY->surfaceHeight),
    };
}

// Nearest hit along a camera ray whose pixels are actually visible. A window
// that was never captured is drawn as nothing (the renderer skips it too), so
// it must not catch the aim, focus or a click either -- an off-screen window
// would otherwise sit invisibly in front of the view. An invisible layer
// overlay (transparent quickshell PanelWindow) must not swallow the ray
// either: empty pixels fall through to the surface underneath. This is also
// what keeps aim/focus from flapping between an overlay and the window it
// covers.
static World3D::SHit pickVisible(const Vec3& dir) {
    const auto& cam = g_scene.camera();

    for (const auto& HIT : g_world.pickAll(cam.position, dir)) {
        const auto* SNAPSHOT = g_capture.get(HIT.id);

        if (!SNAPSHOT || (!SNAPSHOT->bigTex && !SNAPSHOT->texID))
            continue;

        if (SNAPSHOT->alphaValid && !SNAPSHOT->alphaMask.empty()) {
            const int MX = std::min(SNAPSHOT->alphaW - 1,
                static_cast<int>(HIT.u * SNAPSHOT->alphaW));
            const int MY = std::min(SNAPSHOT->alphaH - 1,
                static_cast<int>(HIT.v * SNAPSHOT->alphaH));

            const int A = SNAPSHOT->alphaMask[static_cast<size_t>(MY) * SNAPSHOT->alphaW + MX];

            if (A < 8)
                continue; // transparent pixel of an overlay: next surface
        }

        return HIT;
    }

    return {};
}

static World3D::SHit aimHit() {
    return pickVisible(g_scene.camera().centerRay());
}

// Resolves a hit id into either a window or a layer surface.
struct SHitTarget {
    PHLWINDOW window;
    PHLLS     layer;
};

static SHitTarget targetFromHit(std::uintptr_t id) {
    SHitTarget target;

    if (id == 0)
        return target;

    target.layer = Compat::findLayerById(id);

    if (!target.layer)
        target.window = Compat::findWindowById(id);

    return target;
}

static bool rayPlanePoint(
    const Vec3& origin,
    const Vec3& dir,
    const Vec3& planePoint,
    const Vec3& planeNormal,
    Vec3& out
) {
    const float denom = dot(dir, planeNormal);
    if (!std::isfinite(denom) || std::fabs(denom) < 0.000001f)
        return false;

    const float t = dot(planePoint - origin, planeNormal) / denom;
    if (!std::isfinite(t) || t <= 0.0f)
        return false;

    out = origin + dir * t;
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.z);
}

static Vector2D worldPointToGlobalPx(
    const PHLMONITOR& mon,
    const Vec3& point
) {
    (void)mon;

    const Vec3 LOCAL = g_world.localPoint(g_resize.id, point);
    const CBox CURRENT = Compat::currentWindowBox(g_resize.window);

    // The quad spans toWorld(box) * spawnScale world units while carrying
    // box pixels of content, so the px density on the quad is the room's
    // base density divided by the scale.
    const auto* RESIZE_ENTITY = g_world.find(g_resize.id);
    const double scale =
        RESIZE_ENTITY ? RESIZE_ENTITY->spawnScale : 1.0f;

    const double currentCenterX = CURRENT.x + CURRENT.w * 0.5;
    const double currentCenterY = CURRENT.y + CURRENT.h * 0.5;

    return {
        currentCenterX + static_cast<double>(LOCAL.x) *
            World3D::LOGICAL_PX_PER_UNIT / scale,
        currentCenterY - static_cast<double>(LOCAL.y) *
            World3D::LOGICAL_PX_PER_UNIT / scale,
    };
}

static CBox resizeBoxFromAim(const Vec3& point, const PHLMONITOR& mon) {
    // The grabbed edge moves by the crosshair's travel since the grab, not
    // to its absolute position: the box starts identical to the window, so
    // starting a resize never snaps the nearest corner under the crosshair.
    const Vector2D PX = worldPointToGlobalPx(mon, point);
    const double DX = PX.x - g_resize.grabPx.x;
    const double DY = PX.y - g_resize.grabPx.y;

    const CBox& start = g_resize.startBox;
    const double RIGHT = start.x + start.w;
    const double BOTTOM = start.y + start.h;

    CBox out = start;

    if (g_resize.edgeX > 0) {
        out.x = start.x;
        out.w = start.w + DX;
    } else {
        out.x = start.x + DX;
        out.w = start.w - DX;
    }

    if (g_resize.edgeY > 0) {
        out.y = start.y + DY;
        out.h = start.h - DY;
    } else {
        out.y = start.y;
        out.h = start.h + DY;
    }

    const auto MIN = g_resize.window->minSize().value_or(Vector2D{1.0, 1.0});
    const auto MAX = g_resize.window->maxSize().value_or(Vector2D{INFINITY, INFINITY});

    out.w = std::clamp(out.w, MIN.x, MAX.x);
    out.h = std::clamp(out.h, MIN.y, MAX.y);

    if (g_resize.edgeX > 0)
        out.x = start.x;
    else
        out.x = RIGHT - out.w;

    if (g_resize.edgeY > 0)
        out.y = BOTTOM - out.h;
    else
        out.y = start.y;

    return out;
}

// --- fullscreen passthrough -------------------------------------------------

static float wrapPi(float a) {
    while (a > 3.14159265f)
        a -= 6.28318530f;
    while (a < -3.14159265f)
        a += 6.28318530f;
    return a;
}

// To2D: capture the room pose, force the real window to the monitor box and
// aim the animation at a quad that exactly covers the frozen camera frustum.
static void startTo2D(const PHLWINDOW& window, bool captureRestoreBox) {
    const auto MON = targetMonitor();

    if (!MON)
        return;

    const auto ID = Compat::windowId(window);

    if (!g_world.find(ID))
        return; // not in the room: nothing to animate

    g_fsWindow = window;

    // The floating box: the POSITION from the stable memory (where the
    // window lived), the SIZE from Hyprland's own remembered floating size.
    // When the FS was engaged, Hyprland remembered the pre-fullscreen size
    // there (setTargetFullscreenModeInternal does it BEFORE stretching the
    // window to the monitor) -- exactly the size the 2D FS-exit restores.
    // The stable memory is the fallback (e.g. a tiled window has no
    // floating history: its tile size is the honest original).
    if (captureRestoreBox) {
        const auto IT = g_fsStableBoxes.find(ID);
        CBox RESTORE = IT != g_fsStableBoxes.end() ?
            IT->second : Compat::currentWindowBox(window);

        if (window->m_target) {
            const auto LF = window->m_target->lastFloatingSize();

            if (LF.x > 5.0 && LF.y > 5.0)
                RESTORE = CBox{RESTORE.x, RESTORE.y, LF.x, LF.y};
        }

        g_fsRestoreBox = RESTORE;
    }

    // The fullscreen TARGET box (the animation moves the real box there
    // gradually -- see applyFullscreenAnimation).
    g_fsMonitorBox = CBox{
        MON->m_position.x, MON->m_position.y, MON->m_size.x, MON->m_size.y};

    // Pin Hyprland's remembered floating size to the captured PRE-FS size:
    // the transition's per-frame setWindowBox calls would otherwise
    // re-member intermediate (up to monitor-sized) values, and Hyprland's
    // own FS-exit applies that memory.
    if (window->m_target)
        window->m_target->rememberFloatingSize(
            {g_fsRestoreBox.w, g_fsRestoreBox.h});

    // The screen-covering distance derives from the vertical FOV and the
    // monitor's NATURAL world size (logical px / 100):
    // d = (worldHeight / 2) / tan(vfov / 2). At that distance a
    // monitor-sized quad subtends exactly the full frustum.
    const float MON_H_WORLD =
        static_cast<float>(MON->m_size.y) / World3D::LOGICAL_PX_PER_UNIT;
    g_fsDistance = (MON_H_WORLD * 0.5f) /
        std::tan(kFovDeg * 3.14159265f / 360.0f);

    if (const auto* E = g_world.find(ID)) {
        g_fsStartCenter = E->center;
        g_fsStartYaw    = E->yaw;
        g_fsStartPitch  = E->pitch;
        g_fsStartRoll   = wrapPi(E->roll);
        g_fsSavedRoll   = wrapPi(E->roll); // restored when the FS exits

    }

    const auto& CAM = g_scene.camera();
    const Vec3 FWD = CAM.forward();

    g_fsEndCenter = CAM.position + FWD * g_fsDistance;

    // Face the FROZEN camera: with the model convention (normal Y is
    // -sin(pitch)) facing the camera means yaw = -camYaw, pitch = +camPitch.
    // The content lands unmirrored: the quad's local +X maps exactly onto
    // the camera's right vector.
    g_fsEndYaw    = -CAM.yaw;
    g_fsEndPitch  = CAM.pitch;

    resetPointerGesture(); // no gestures on the animating window
    g_input.reset();       // no look jump when look resumes

    g_fsPhase        = EFullscreenPhase::To2D;
    g_fsPhaseStart   = std::chrono::steady_clock::now();
    g_fsWasOn        = true;
    g_fsAssertFrames = 0;
    damageCurrentMonitor();
}

// In2D -> To3D: restore the real floating box, capture input, hide the
// cursor; the animation runs the quad back to its room pose.
static void startTo3D() {
    if (auto W = g_fsWindow.lock()) {
        // The To3D start is the window's CURRENT pose: the tracked fullscreen
        // pose after a normal exit, or wherever the aborted To2D got to. The
        // real floating box is restored when the animation COMPLETES (restoring
        // it here would make the quad show magnified small-window content).
        if (auto* E = g_world.find(Compat::windowId(W))) {
            g_fsEndCenter = E->center;
            g_fsEndYaw    = E->yaw;
            g_fsEndPitch  = E->pitch;
            g_fsRollAtStart = wrapPi(E->roll);
        }

        if (!pointerFree() && Compat::setCursorHidden(true))
            Pointer::mgr()->resetCursorImage();

        Compat::setPointerCapture(g_hookInstalled);

        g_fsPhase      = EFullscreenPhase::To3D;
        g_fsPhaseStart = std::chrono::steady_clock::now();
        g_input.reset();
        damageCurrentMonitor();
    }
}

// None -> To2D when a fullscreen window appears; In2D -> To3D when it goes.
// Hyprland fades windows with per-window alpha animations that only make
// progress while the window is being damaged: during the fullscreen
// handoff the non-FS windows fade out (their FADE/FULLSCREEN alpha channels
// animate toward 0), and on the way back they fade in; focus changes fade
// the ACTIVE channel the same way. Mid-fade the compositor stops damaging
// them -- the animation freezes a couple of frames in, and the room shows
// windows stuck at partial alpha until an aim hover forces a frame (the
// semi-transparent-after-FS bug). Damage every window whose alpha channels
// are still in flight; the damage drives the fade to completion and the
// loop stops on its own.
static void damageWindowsWithLiveAlpha(const PHLMONITOR& mon) {
    if (!Desktop::windowState() || !mon || !mon->m_activeWorkspace)
        return;

    for (const auto& W : Desktop::windowState()->windows()) {
        if (!W || W->m_workspace != mon->m_activeWorkspace)
            continue;

        const auto IN_FLIGHT = [&](Desktop::View::eWindowAlpha channel) {
            return W->alphaValue(channel) != W->alphaGoal(channel);
        };

        if (IN_FLIGHT(Desktop::View::WINDOW_ALPHA_FADE) ||
            IN_FLIGHT(Desktop::View::WINDOW_ALPHA_ACTIVE) ||
            IN_FLIGHT(Desktop::View::WINDOW_ALPHA_FULLSCREEN))
            g_pHyprRenderer->damageWindow(W);
    }
}

static void pollFullscreen() {
    const auto MON = targetMonitor();

    if (!MON)
        return;

    // The alpha keepalive rides the pump (8 ms) rather than the render pass,
    // so the fade keeps ticking even when a frame is slow.
    damageWindowsWithLiveAlpha(MON);

    const auto FSW = Fullscreen::controller()->getFullscreenWindow(MON);

    g_fsCurrentId = FSW ? Compat::windowId(FSW) : 0;

    // Our own room fade: while a fullscreen exists (or a transition runs)
    // the other windows glide to invisible; when it is gone they glide
    // back. Deterministic wall-clock ticks -- never dependent on Hyprland's
    // animation engine or on snapshot retakes.
    constexpr float FADE_DURATION = 0.25f;

    const auto FADE_NOW = std::chrono::steady_clock::now();
    const float FADE_DT = g_fsFadeLast.time_since_epoch().count() == 0 ?
        0.0f :
        std::chrono::duration<float>(FADE_NOW - g_fsFadeLast).count();
    g_fsFadeLast = FADE_NOW;

    const float FADE_TARGET =
        (FSW || g_fsPhase != EFullscreenPhase::None) ? 0.0f : 1.0f;

    if (FADE_DT > 0.0f && g_fsFade != FADE_TARGET) {
        const float STEP = std::min(FADE_DT / FADE_DURATION, 1.0f);
        g_fsFade = FADE_TARGET > g_fsFade ?
            std::min(g_fsFade + STEP, FADE_TARGET) :
            std::max(g_fsFade - STEP, FADE_TARGET);
    }

    if (g_fsPhase == EFullscreenPhase::None) {
        if (FSW && !g_fsWasOn && ownsInput())
            startTo2D(FSW);
        else {
            g_fsWasOn = FSW != nullptr;

            // A fullscreen EXITED while the room is plain 3D (no transition
            // was running -- e.g. the FS predated the 3D entry): make the
            // window a spawn-sized floating panel AND hold it with the same
            // condition-driven assert the To3D completion uses -- Hyprland's
            // floating layout re-applies ITS remembered size (possibly
            // monitor-sized from an old cycle), and a one-shot set loses to
            // it.
            if (!FSW && g_fsWasOn) {
                if (auto OLDW = g_fsLastFSWindow.lock()) {
                    const auto CUR = Compat::currentWindowBox(OLDW);

                    g_fsAssertBox    = CBox{CUR.x, CUR.y, g_cfgSpawnWidth,
                                            g_cfgSpawnHeight};
                    g_fsAssertFrames = 1;
                    g_fsStableCount  = 0;

                    Compat::setWindowBox(OLDW, g_fsAssertBox);
                }
            }
        }

        g_fsLastFSWindow = FSW;
    } else if (g_fsPhase == EFullscreenPhase::In2D) {
        // Exit passthrough when OUR window is no longer the fullscreen one
        // (toggled off, or fullscreen moved to another window entirely).
        const auto W = g_fsWindow.lock();

        if (!FSW || (W && FSW != W))
            startTo3D();
        else
            g_fsWasOn = true;
    } else if (g_fsPhase == EFullscreenPhase::To2D) {
        // Fullscreen was toggled off MID-ANIMATION: hand back to 3D from the
        // current animated pose instead of finishing onto a stale screen.
        if (!FSW)
            startTo3D();
    } else if (g_fsPhase == EFullscreenPhase::To3D) {
        g_fsWasOn = FSW != nullptr; // remembered for after the animation
    }
}

// To2D/To3D: pose the fullscreening window between its room pose and the
// screen-covering pose (smoothstep). Runs AFTER syncWorld, overriding the
// per-frame reseed.
static void applyFullscreenAnimation() {
    if (g_fsPhase != EFullscreenPhase::To2D && g_fsPhase != EFullscreenPhase::To3D)
        return;

    const auto W = g_fsWindow.lock();
    auto* E = W ? g_world.find(Compat::windowId(W)) : nullptr;

    if (!E) {
        // the window closed mid-transition: land the phase
        g_fsPhase  = EFullscreenPhase::None;
        g_fsWindow = {};
        return;
    }

    const float RAWP = fsRawProgress();
    g_fsRawP = RAWP; // feeds the composite alpha below
    float p = RAWP * RAWP * (3.0f - 2.0f * RAWP); // smoothstep

    // To2D: the SCREEN pose tracks the LIVE camera -- looking around during
    // the transition keeps the quad converging onto the current view, so the
    // handoff never pops.
    if (g_fsPhase == EFullscreenPhase::To2D) {
        const auto& CAM = g_scene.camera();
        const Vec3 FWD = CAM.forward();

        g_fsEndCenter = CAM.position + FWD * g_fsDistance;
        g_fsEndYaw    = -CAM.yaw;
        g_fsEndPitch  = CAM.pitch;
    }

    // To2D: room -> screen; To3D: screen -> room
    const auto  A = g_fsPhase == EFullscreenPhase::To2D;
    const Vec3  C0 = A ? g_fsStartCenter : g_fsEndCenter;
    const Vec3  C1 = A ? g_fsEndCenter : g_fsStartCenter;
    const float Y0 = A ? g_fsStartYaw : g_fsEndYaw;
    const float Y1 = A ? g_fsEndYaw : g_fsStartYaw;
    const float P0 = A ? g_fsStartPitch : g_fsEndPitch;
    const float P1 = A ? g_fsEndPitch : g_fsStartPitch;

    E->center = C0 + (C1 - C0) * p;
    E->yaw    = Y0 + wrapPi(Y1 - Y0) * p;
    E->pitch  = P0 + (P1 - P0) * p;

    // The roll fades out on the way to 2D (the compositor renders windows
    // unrolled) and fades back to the saved value on the way to the room.
    if (A)
        E->roll = g_fsSavedRoll * (1.0f - p);
    else
        E->roll = g_fsRollAtStart + (g_fsSavedRoll - g_fsRollAtStart) * p;

    // The REAL box animates between the floating box and the fullscreen box.
    // The client's buffer is stretched to this box by the compositor, so the
    // content scale inside the quad stays constant through the whole
    // transition. On the way back the box lands at the SPAWN size (windows.spawn_width/height):
    // the post-FS size is deterministic and never inherits garbage from
    // earlier broken cycles.
    const CBox B0 = A ? g_fsRestoreBox : g_fsMonitorBox;
    const CBox B1 = A ? g_fsMonitorBox : g_fsRestoreBox;

    const CBox BOX{
        B0.x + (B1.x - B0.x) * p,
        B0.y + (B1.y - B0.y) * p,
        B0.w + (B1.w - B0.w) * p,
        B0.h + (B1.h - B0.h) * p,
    };

    if (auto W = g_fsWindow.lock())
        Compat::setWindowBox(W, BOX);

    // The quad follows the animated box at the room's pixel density. The
    // fullscreen quad maps 1:1 onto the monitor by definition, so the room's
    // window scale is suspended for its duration (syncWorld re-applies it
    // once the window lands back in the room). The scale itself is lerped
    // across the animation so the endpoints match their neighbours: the room
    // side is the window at config scale, the monitor side is exact 1:1 --
    // otherwise the quad visibly jumps at the handoff frames.
    E->spawnScale = 1.0f;

    const float ROOM_SCALE = configWindowScale();
    const float S0 = A ? ROOM_SCALE : 1.0f;
    const float S1 = A ? 1.0f : ROOM_SCALE;
    const float S = S0 + (S1 - S0) * p; // p is the eased progress

    E->width  = World3D::toWorld(BOX.w) * S;
    E->height = World3D::toWorld(BOX.h) * S;

    // The roll decays to zero: the 2D fullscreen view has no roll, and the
    // quad must land matching what the compositor will render.

    // Composite alpha: minimal fades at the handoffs (a couple of frames).
    // The snapshot and the live 2D render can be a frame apart, and a long
    // fade makes that desync visible; a 2-frame crossfade hides it.
    g_fsAlpha = A ?
        std::clamp(1.0f - (g_fsRawP - 0.96f) / 0.04f, 0.0f, 1.0f) :
        std::clamp(g_fsRawP / 0.04f, 0.0f, 1.0f);

    // Phase completion:
    //   To2D done -> hand the screen to the 2D compositor (In2D): input is
    //   released, the cursor reappears, the 3D scene stops rendering (the
    //   onRenderStage gate) and the pump polls for the fullscreen exit.
    //   To3D done -> restore the floating box (the client re-renders at its
    //   room size) and return to the plain 3D room.
    if (p >= 1.0f) {
        if (g_fsPhase == EFullscreenPhase::To2D) {
            g_fsPhase = EFullscreenPhase::In2D;

            Compat::setPointerCapture(false);

            // Our transition's per-frame setWindowBox calls polluted
            // Hyprland's remembered floating size with intermediate (up to
            // monitor-sized) values. Re-pin the captured PRE-FS size.
            if (auto W2 = g_fsWindow.lock())
                W2->m_target->rememberFloatingSize(
                    {g_fsRestoreBox.w, g_fsRestoreBox.h});

            if (Compat::setCursorHidden(false) && g_pHyprRenderer)
                g_pHyprRenderer->setCursorFromName("default", true);
        } else {
            // Rapid exit+reenter: the FS state may already be ON again for
            // this window -- fly it back to the screen (real 2D) instead of
            // dropping it into the room small while fullscreened.
            const auto MON = targetMonitor();
            const auto W2 = g_fsWindow.lock();

            if (W2 && MON &&
                Fullscreen::controller()->getFullscreenWindow(MON) == W2) {
                g_fsPhaseStart = std::chrono::steady_clock::now();
                startTo2D(W2, /*captureRestoreBox*/ false);
            } else {
                if (W2) {
                    // The original position AND the pre-fullscreen size:
                    // Hyprland remembered the size when the FS was engaged,
                    // the box lerp above already animated the real box there.
                    Compat::setWindowBox(W2, g_fsRestoreBox);

                    // Hyprland's own fullscreen-exit restore animates the
                    // window toward ITS remembered floating size -- which our
                    // transition kept updating -- and can retarget the box
                    // AFTER this point. Re-assert the restore box for a few
                    // frames so the final size is ours.
                    // The SAME box the quad just landed at (the spawn
                    // size) -- asserting the raw g_fsRestoreBox here made
                    // our own guard resize the window back to the polluted
                    // monitor size right after the landing.
                    g_fsAssertBox    = g_fsRestoreBox;
                    g_fsAssertFrames = 1;    // condition-driven: runs until stable
                    g_fsStableCount  = 0;

                    // The FS fade dimmed EVERY other workspace window to
                    // alpha 0 when the fullscreen started; our transition can
                    // leave the OUT fade mid-flight. Reset the channel
                    // explicitly, or windows stay barely visible.
                    if (MON && MON->m_activeWorkspace) {
                        for (auto const& W3 :
                             Desktop::windowState()->windows()) {
                            if (W3 && W3->m_workspace ==
                                          MON->m_activeWorkspace &&
                                !W3->m_pinned)
                                *W3->alpha(
                                     Desktop::View::WINDOW_ALPHA_FULLSCREEN) =
                                    1.F;
                        }
                    }
                }

                g_fsPhase = EFullscreenPhase::None;
            }
        }
    }
}

// Ends a model roll gesture: restores the suspended gravity on every exit
// path (button release, crosshair leaving, model vanishing) and WAKES the
// body -- during the roll it was held motionless, so Jolt put it to sleep,
// and a sleeping body would stay frozen forever even with gravity back.
static void endModelRoll() {
    if (s_wheelRot.model && s_wheelRot.modelIndex < g_joltBodies.size()) {
        const auto BODY = g_joltBodies[s_wheelRot.modelIndex].body;
        g_bodyIf->SetGravityFactor(BODY, g_rolledGravity);
        g_bodyIf->ActivateBody(BODY);
    }
    s_wheelRot.model      = false;
    s_wheelRot.active     = false;
    s_wheelRot.modelIndex = SIZE_MAX;
}

// Per-frame roll gesture: rotate the window around its normal by the signed
// angle the crosshair swept around the window center (measured in world
// space around the normal -- invariant to the roll itself, so the gesture
// never feeds back into its own measurement).
static void updateWheelRoll() {
    if (!s_wheelRot.active)
        return;

    if (s_wheelRot.model) {
        // A config reload can shrink the object list mid-roll.
        if (s_wheelRot.modelIndex >= g_sceneObjects.size()) {
            endModelRoll();
            return;
        }

        auto* MODEL = g_scene.sceneModel(s_wheelRot.modelIndex);
        if (!MODEL || !MODEL->loaded()) {
            endModelRoll();
            return;
        }

        const auto& CAM = g_scene.camera();
        Vec3 P;
        if (!rayPlanePoint(CAM.position, CAM.centerRay(), s_wheelRot.center,
                           s_wheelRot.normal, P))
            return;

        const Vec3 V = P - s_wheelRot.center;
        const float ANGLE = std::atan2(
            dot(cross(s_wheelRot.reference, V), s_wheelRot.normal),
            dot(s_wheelRot.reference, V));

        constexpr float DEG = 3.14159265358979f / 180.0f;
        auto& ROT = g_sceneObjects[s_wheelRot.modelIndex].rotationDeg;
        ROT.z = (s_wheelRot.startRoll + ANGLE) / DEG;

        g_scene.setSceneObjectTransform(s_wheelRot.modelIndex,
                                        MODEL->position(), ROT);

        // Rolling a physics object suspends its simulation: gravity off,
        // velocities zeroed -- otherwise the falling/dragging motion fights
        // the user's rotation and the behavior looks erratic. The new
        // rotation is ALSO pushed into the body: the per-frame readback
        // reads the body transform, and without this it would revert the
        // roll every frame.
        if (s_wheelRot.model && s_wheelRot.modelIndex < g_joltBodies.size()) {
            const auto JB = g_joltBodies[s_wheelRot.modelIndex].body;
            g_bodyIf->SetGravityFactor(JB, 0.f);
            g_bodyIf->SetLinearVelocity(JB, JPH::Vec3::sZero());
            g_bodyIf->SetAngularVelocity(JB, JPH::Vec3::sZero());
            g_bodyIf->SetPositionAndRotationWhenChanged(
                JB,
                JPH::RVec3(MODEL->position().x, MODEL->position().y,
                           MODEL->position().z),
                eulerToQuat(ROT), JPH::EActivation::DontActivate);
        }

        damageCurrentMonitor();
        return;
    }

    const auto HIT = aimHit();

    if (!HIT.hit || HIT.id != s_wheelRot.id) {
        s_wheelRot.active = false; // crosshair left the window: end it
        return;
    }

    auto* ENTITY = g_world.find(s_wheelRot.id);

    if (!ENTITY) {
        s_wheelRot.active = false;
        return;
    }

    const Vec3 V = HIT.point - s_wheelRot.center;
    const float ANGLE = std::atan2(
        dot(cross(s_wheelRot.reference, V), s_wheelRot.normal),
        dot(s_wheelRot.reference, V));

    // Wrapped to [-pi, pi]: the angle is 2pi-periodic, so the visual is
    // identical, but the fullscreen transition interpolates this value
    // linearly -- an unwrapped 700-degree roll would unwind like a top.
    ENTITY->roll = wrapPi(s_wheelRot.startRoll + ANGLE);
}

static void updateRealResize() {
    if (!g_resize.active || !g_resize.window)
        return;

    const auto MON = targetMonitor();
    if (!MON)
        return;

    const auto* ENTITY = g_world.find(g_resize.id);
    if (!ENTITY)
        return;

    // The edge being dragged is part of the real, resized window, so the
    // interaction plane follows the window's current centre every frame.
    g_resize.planePoint = ENTITY->center;
    g_resize.planeNormal = g_world.normalOf(g_resize.id);

    Vec3 point;
    if (!rayPlanePoint(
            g_scene.camera().position,
            g_scene.camera().centerRay(),
            g_resize.planePoint,
            g_resize.planeNormal,
            point))
        return;

    const CBox BOX = resizeBoxFromAim(point, MON);
    Compat::setWindowBox(g_resize.window, BOX);
}

static void resetPointerGesture() {
    g_pointerDown = false;
    g_pointerGesture = EPointerGesture::None;
    g_pointerButton = 0;

    // A finished resize gesture: schedule one forced snapshot of the resized
    // window for the next capture pass -- its purpose is the silhouette's
    // settled full-resolution refresh (see CWindowCapture::refreshSkirtMask).
    if (g_resize.active)
        g_skirtFinalRefreshId = g_resize.id;

    g_resize = {};
    // A finished roll gesture restores the object's gravity.
    endModelRoll();

    // Dropping a carried scene object: it re-enters every contact pair at
    // its current position (and its BVH tree rebuilds for picking).
    if (g_bodyIf && g_mapGrabIndex != SIZE_MAX &&
        g_mapGrabIndex < g_joltBodies.size() &&
        g_joltBodies[g_mapGrabIndex].valid)
        g_bodyIf->SetObjectLayer(
            g_joltBodies[g_mapGrabIndex].body,
            g_sceneObjects[g_mapGrabIndex].dynamic ? LAYER_MOVING
                                                    : LAYER_STATIC);
    g_mapGrabIndex = SIZE_MAX;
    g_charGrabIndex = SIZE_MAX; // the body rejoins the contacts next frame
}

static void finishClientButton(uint32_t timeMs) {
    if (!g_clientButtonDown)
        return;

    if (g_clientButtonLayer) {
        Compat::deliverClick(
            g_clientButtonLayer,
            g_clientButtonLocal,
            g_clientButton,
            false,
            timeMs
        );
    } else if (g_clientButtonWindow) {
        Compat::deliverClick(
            g_clientButtonWindow,
            g_clientButtonLocal,
            g_clientButton,
            false,
            timeMs
        );
    }

    g_clientButtonWindow = nullptr;
    g_clientButtonLayer  = nullptr;
    g_clientButton = 0;
    g_clientButtonDown = false;
}

static void forwardPointerToAim(uint32_t timeMs) {
    if (!ownsInput() || g_pointerDown)
        return;

    const auto HIT = aimHit();
    const auto TARGET = HIT.hit ? targetFromHit(HIT.id) : SHitTarget{};

    if (g_clientButtonDown && (g_clientButtonWindow || g_clientButtonLayer)) {
        // A pressed client keeps pointer ownership while the camera turns. If
        // the crosshair leaves that surface, hold the last local point instead
        // of teleporting pointer focus into a different client mid-drag.
        const bool SAME =
            (g_clientButtonWindow && TARGET.window == g_clientButtonWindow) ||
            (g_clientButtonLayer && TARGET.layer == g_clientButtonLayer);

        if (SAME && HIT.hit)
            g_clientButtonLocal = localFromHit(HIT);

        if (g_clientButtonLayer)
            Compat::deliverMotion(g_clientButtonLayer, g_clientButtonLocal, timeMs);
        else if (g_clientButtonWindow)
            Compat::deliverMotion(g_clientButtonWindow, g_clientButtonLocal, timeMs);
        return;
    }

    if (TARGET.layer) {
        Compat::deliverMotion(TARGET.layer, localFromHit(HIT), timeMs);
        return;
    }

    if (!TARGET.window) {
        Compat::clearPointerFocus();
        return;
    }

    Compat::deliverMotion(TARGET.window, localFromHit(HIT), timeMs);
}

// Absolute-position fallback for installations where the PointerManager hook
// is unavailable. When the hook works, it feeds onPointerMotion directly and
// this event only keeps the baseline warm.
static bool onPointerMotion(double dx, double dy) {
    if (!ownsInput())
        return false;

    if (pointerFree()) {
        if (g_vcSpace == ECursorSpace::Desktop)
            return false; // the real cursor is out on the desktop

        vcMove(dx, dy);
        damageCurrentMonitor();
        return true;
    }

    g_input.addMotion(dx, dy);
    damageCurrentMonitor();
    return true;
}

static void onRenderPre(PHLMONITOR mon) {
    if (g_capturing)
        return;

    const auto TARGET = targetMonitor();
    if (TARGET && mon != TARGET) {
        g_currentRenderMon = nullptr;
        return;
    }

    g_currentRenderMon   = mon;
    g_roomDrawnThisFrame = false;

    if (!g_active) {
        g_monitor = mon;
        return;
    }

    g_monitor = mon;
    serviceCapture();
    refreshCursorImage();
}


// --- lifecycle --------------------------------------------------------------

// Saves every room entity's pose for the next 3D session (see
// g_rememberedPoses); closed windows are pruned on the way.
static void rememberPoses() {
    std::erase_if(g_rememberedPoses, [](const auto& KV) {
        return KV.second.window.expired() && KV.second.layer.expired();
    });

    const auto FSW = g_fsWindow.lock();

    for (const auto& E : g_world.entities()) {
        const auto TARGET = targetFromHit(E.id);
        if (!TARGET.window && !TARGET.layer)
            continue;

        SRememberedPose POSE;
        POSE.window = TARGET.window;
        POSE.layer  = TARGET.layer;

        // A window caught in a fullscreen transition (or in 2D fullscreen)
        // keeps the pose it had before going fullscreen.
        if (g_fsPhase != EFullscreenPhase::None && FSW &&
            Compat::windowId(FSW) == E.id) {
            POSE.center = g_fsStartCenter;
            POSE.yaw    = g_fsStartYaw;
            POSE.pitch  = g_fsStartPitch;
            POSE.roll   = g_fsSavedRoll;
        } else {
            POSE.center = E.center;
            POSE.yaw    = E.yaw;
            POSE.pitch  = E.pitch;
            POSE.roll   = E.roll;
        }

        g_rememberedPoses[E.id] = POSE;
    }
}

static void deactivate3D() {
    rememberPoses();

    finishClientButton(0);
    resetPointerGesture();

    g_active = false;
    stopFramePump();

    // Fullscreen passthrough state: back to plain 3D-off. Restore the real
    // box if a transition was mid-flight (the window would otherwise stay
    // monitor-sized). The SIZE is the spawn size -- never the possibly
    // polluted restore box.
    if (auto W = g_fsWindow.lock())
        Compat::setWindowBox(
            W,
            CBox{g_fsRestoreBox.x, g_fsRestoreBox.y, g_cfgSpawnWidth,
                 g_cfgSpawnHeight});

    g_fsPhase        = EFullscreenPhase::None;
    g_fsWindow       = {};
    g_fsWasOn        = false;
    g_fsAlpha        = 1.0f;
    g_fsFade         = 1.0f;
    g_fsAssertFrames = 0;

    // Restore the host cursor before anything else touches focus: removing
    // pointer focus makes the client re-apply its own cursor image on the
    // next pointer enter, and the default shape shows immediately.
    if (Compat::setCursorHidden(false) && g_pHyprRenderer)
        g_pHyprRenderer->setCursorFromName("default", true);

    clearAimFocus();
    g_world.clear();
    g_renderWindows.clear();
    g_input.reset();
    resetMovementKeys();

    g_capture.releaseAll();
    Compat::setPointerCapture(false);

    unghostWindows();
    g_renderedOnce = false;
}

// The close transition finishes inside render.stage, i.e. between the frame's
// startRenderPass() and endRender(). deactivate3D destroys the snapshot
// framebuffers and restores the layout, which broke the compositor's outer
// endRender() (SEGV in CMonitor::useFP16) -- the same failure syncWorld
// documents. Run it from the event loop instead; PLUGIN_EXIT cancels a pending
// call so it never runs into unloaded code.
static uint64_t g_deactivateLater = 0;

static void requestDeactivate3D() {
    if (g_deactivateLater || !g_pEventLoopManager)
        return;

    g_deactivateLater = g_pEventLoopManager->doLater([] {
        g_deactivateLater = 0;

        // Reopened before the deferred teardown ran.
        if (!g_active || g_transitionTarget > 0.0f)
            return;

        deactivate3D();
        damageCurrentMonitor();
    });
}

static void enter3D() {
    // Without this the render stage bails out immediately and the toggle does
    // nothing at all.
    g_active = true;
    startFramePump();

    // Hide the host cursor: the 3D view aims with its own crosshair. Client
    // cursor updates are gated by the hooks; the client re-applies its own
    // image automatically when pointer focus re-enters on exit.
    if (!Compat::setCursorHidden(true))
        notify(
            "[hypr3d] cursor hooks unavailable: the frozen cursor will stay visible",
            CHyprColor{1.0f, 0.6f, 0.2f, 1.0f}
        );
    else
        Pointer::mgr()->resetCursorImage();

    // Focus is absent by default: it only ever appears when the crosshair is
    // on a window.
    clearAimFocus();

    finishClientButton(0);
    resetPointerGesture();

    g_scene.reset();
    g_input.reset();
    g_world.clear();
    g_renderWindows.clear();

    // Player spawn point (config player_spawn): the coordinates are the
    // player's FEET, so spawning at 0,0,0 stands on the grid platform at
    // world zero instead of falling through it. Eyes ride kEyeHeight above.
    {
        auto& CAM = g_scene.camera();
        CAM.position = Vec3{
            g_playerSpawn.x,
            g_playerSpawn.y + Camera::kEyeHeight,
            g_playerSpawn.z,
        };

        // Camera forward is {sin yaw, ., -cos yaw}: looking at the origin
        // from (x, z) means yaw = atan2(-x, z).
        CAM.yaw = std::atan2(-g_playerSpawn.x, g_playerSpawn.z);
    }

    g_capture.releaseAll();

    resetMovementKeys();

    g_grounded = false;
    g_viewMode = 0;
    g_scene.camera().mirrorView = false;
    g_scene.setPlayerVisible(false);
    g_scene.setPlayerDebugCapsule(Vec3{}, false);

    g_keyboardMode = EKeyboardMode::Space;
    g_scene.setCrosshairVisible(true);
    g_freeOnRoom     = false;
    g_swallowRelease = 0;
    g_vcSpace        = ECursorSpace::Desktop;
    g_vcId           = 0;
    g_scene.setPointer({});
    g_altHeld      = false;
    s_zoomId       = 0;

    // A fullscreen window that already exists: it joins the room as a
    // standard spawn-sized floating panel (a fullscreened/tiled box would
    // otherwise enter the room monitor-sized AND pollute the stable-box
    // memory). The passthrough still triggers on the fullscreen EVENT.
    if (const auto MON = targetMonitor()) {
        if (const auto FSW =
                Fullscreen::controller()->getFullscreenWindow(MON)) {
            Compat::setWindowBox(
                FSW,
                CBox{MON->m_position.x + MON->m_size.x * 0.5 -
                         g_cfgSpawnWidth * 0.5,
                     MON->m_position.y + MON->m_size.y * 0.5 -
                         g_cfgSpawnHeight * 0.5,
                     g_cfgSpawnWidth, g_cfgSpawnHeight});

            g_fsLastFSWindow = FSW;
        }

        g_fsWasOn =
            Fullscreen::controller()->getFullscreenWindow(MON) != nullptr;
    } else
        g_fsWasOn = false;

    g_fsStableBoxes.clear();

    g_renderedOnce = false;
    g_captureFrames = 0;
    g_reportedFramebufferError = false;
    g_reportedRenderError = false;

    // The first full live capture is performed from render.pre, where
    // Hyprland has not entered its main render pass yet.
}

static void toggle3D() {
    g_transitionTarget =
        g_transitionTarget > 0.5f ? 0.0f : 1.0f;

    if (g_transitionTarget > 0.5f)
        enter3D();

    damageCurrentMonitor();

    notify(
        "[hypr3d] toggle",
        CHyprColor{0.2f, 0.8f, 0.4f, 1.0f}
    );
}

static void open3D() {
    g_transitionTarget = 1.0f;
    enter3D();
    damageCurrentMonitor();

    notify(
        "[hypr3d] open",
        CHyprColor{0.2f, 0.8f, 0.4f, 1.0f}
    );
}

static void close3D() {
    g_transitionTarget = 0.0f;
    damageCurrentMonitor();

    notify(
        "[hypr3d] close",
        CHyprColor{0.2f, 0.8f, 0.4f, 1.0f}
    );
}

static float updateTransition() {
    const auto now = std::chrono::steady_clock::now();

    const float dt =
        std::chrono::duration<float>(now - g_lastTick).count();

    g_lastTick = now;

    constexpr float duration = 0.55f;
    constexpr float speed = 1.0f / duration;

    if (g_transition < g_transitionTarget)
        g_transition = std::min(g_transition + dt * speed, 1.0f);
    else if (g_transition > g_transitionTarget)
        g_transition = std::max(g_transition - dt * speed, 0.0f);

    return std::clamp(dt, 0.0f, 0.1f);
}

// --- frame ------------------------------------------------------------------

// World-space glide velocity for the WASD inertia (units/second). Lives
// across frames so releasing the keys coasts down instead of cutting dead.
static Vec3 s_moveVel{};

// Walk bob (view-only): sine phase in radians + eased amplitude. The body
// pose stays physical -- the offset rides on the camera each frame and is
// re-derived from the clean eye point, so it never feeds back into Jolt.
static float s_bobPhase = 0.0f;
static float s_bobAmp   = 0.0f;
static constexpr float kBobAmplitude = 0.035f; // meters at full walk speed
static constexpr float kBobRate      = 6.0f;   // radians per meter walked
static constexpr float kBobRoll      = 0.008f; // radians (~0.9 deg) head tilt

// Computes the key-driven world-space velocity into s_moveVel (units/s)
// with the move-inertia glide. The PLAYER BODY applies it -- Jolt owns the
// pose, so this no longer moves the camera directly.
static void applyCameraMovement(float dt) {
    float FORWARD  = (g_keyFwd ? 1.f : 0.f) - (g_keyBack ? 1.f : 0.f);
    float STRAFE   = (g_keyRight ? 1.f : 0.f) - (g_keyLeft ? 1.f : 0.f);
    float VERTICAL = (g_keyUp ? 1.f : 0.f) - (g_keyDown ? 1.f : 0.f);

    auto& CAM = g_scene.camera();

    // Walking mode: Shift does nothing (gravity owns vertical), Space is a
    // jump impulse queued in the key handler.
    if (!playerFlies())
        VERTICAL = 0.f;

    // Diagonals must not be faster than a straight run: W+D used to add the
    // two key speeds into a sqrt(2)x diagonal.
    {
        const float LEN = std::sqrt(FORWARD * FORWARD + STRAFE * STRAFE);
        if (LEN > 1.0f) {
            FORWARD /= LEN;
            STRAFE /= LEN;
        }
    }

    const bool  MOVING = FORWARD != 0.f || STRAFE != 0.f || VERTICAL != 0.f;
    const float tau    = g_cfgMoveInertia;

    if (!playerFlies()) {
        // --- walking: the keys own the horizontal plane; Jolt's gravity
        // (system -14, the old player constant) owns the vertical one ---
        const float SPEED = CAM.moveSpeed * (g_keySprint ? 2.5f : 1.0f);
        const Vec3  TARGET =
            CAM.flatForward() * (FORWARD * SPEED) +
            CAM.right() * (STRAFE * SPEED);

        if (tau > 0.0f && dt > 0.0f)
            s_moveVel += (TARGET - s_moveVel) * (1.0f - std::exp(-dt / tau));
        else
            s_moveVel = TARGET;

        const bool MOVING_H = FORWARD != 0.f || STRAFE != 0.f;
        if (!MOVING_H && std::fabs(s_moveVel.x) + std::fabs(s_moveVel.z) < 0.01f)
            s_moveVel = {};
        return;
    }

    // --- flying: all three axes are key-driven ---
    const float SPEED =
        CAM.moveSpeed * (g_keySprint ? 2.5f : 1.0f) * (MOVING ? 1.0f : 0.0f);
    const Vec3 TARGET =
        CAM.flatForward() * (FORWARD * SPEED) +
        CAM.right() * (STRAFE * SPEED) +
        Vec3{0.f, 1.f, 0.f} * (VERTICAL * SPEED);

    if (tau <= 0.0f || dt <= 0.0f) {
        s_moveVel = TARGET;
        return;
    }

    // Glide toward the key-driven velocity with tau as the time constant;
    // with the keys released the target is zero, so motion decays smoothly.
    s_moveVel += (TARGET - s_moveVel) * (1.0f - std::exp(-dt / tau));

    // Snap the decay tail off once it is far below a frame of movement, so
    // the glide always terminates.
    if (!MOVING &&
        std::fabs(s_moveVel.x) + std::fabs(s_moveVel.y) +
            std::fabs(s_moveVel.z) < 0.01f)
        s_moveVel = {};
}

static void update3D(float dt) {
    // Release capture unconditionally when the view no longer owns input, so
    // a disappearing monitor or a closing transition can never strand the
    // pointer in captured mode.
    Compat::setPointerCapture(captureWanted());

    float yawDelta = 0.0f;
    float pitchDelta = 0.0f;

    // Look/movement stay live during the fullscreen transition: the To2D end
    // pose tracks the camera, so the quad keeps converging onto the view.
    // Plugin settings come from lua (hl.plugin.hypr3d.config) and are
    // already clamped at set time.
    g_input.setLookSmoothing(g_cfgLookInertia);
    g_scene.camera().moveSpeed = g_cfgMoveSpeed;
    g_input.setSensitivity(g_cfgSensitivity);

    // Map: the scene owns the file/GL side; collision rebuilds its BVH
    // whenever the map (re)loaded.
    std::vector<GLScene::SSceneSpec> SPECS;
    SPECS.reserve(g_sceneObjects.size());

    for (const auto& OBJ : g_sceneObjects) {
        GLScene::SSceneSpec SPEC;
        SPEC.path          = OBJ.path;
        SPEC.position      = OBJ.position;
        SPEC.rotationDeg   = OBJ.rotationDeg;
        SPEC.scale         = OBJ.scale;
        SPEC.emissiveScale = OBJ.emissiveScale;
        SPEC.flat          = OBJ.flat;
        SPEC.center        = OBJ.center;
        SPEC.centerOffset  = OBJ.centerOffset;
        SPECS.push_back(SPEC);
    }

    g_scene.setSceneObjects(SPECS);
    g_scene.setGridVisible(g_cfgGrid);

    // Frame rate for the F3 HUD: slow exponential average over dt.
    if (dt > 0.0f)
        g_debugFps = g_debugFps * 0.9f + (1.0f / dt) * 0.1f;
    g_scene.setDebugFps(g_debugFps);

    // Window alpha keepalive lives on the FS pump (damageWindowsWithLiveAlpha)
    // -- it must tick even when a frame is slow.

    // Collision = PER-OBJECT trees: each scene object owns a small BVH over
    // its world triangles, rebuilt only when ITS OWN generation changes.
    // The static map's big tree builds once and never rebuilds for the FS
    // cycles or object carries -- the release hitch was the full 70k-triangle
    // rebuild; now a release rebuilds only the carried prop's small tree.
    // The carried object's tree also rebuilds per frame (a prop is small)
    // but is EXCLUDED from the queries while carried, so the object the
    // player holds cannot push them around.
    // --- Jolt: rigid body simulation for scene objects -------------------
    // Bodies live in Jolt; transforms read back into the config objects (the
    // render + collision layers follow the config as always). Jolt sleeps
    // resting bodies, so a settled scene costs nothing.
    const auto JOLT_T0 = std::chrono::steady_clock::now();
    joltSyncBodies();

    // While a scene object is still being decoded, or its first shape
    // cooked, its collision is missing: the player and every prop would fall
    // through a map that is not there yet. Time stands still until it is --
    // as it did while the load froze the whole compositor.
    if (g_joltSystem && !g_scene.scenePending() && !joltShapesPending()) {
        // All jobs execute on the main thread (0 workers), but through the
        // FULL thread-pool job system: it handles the dependency graph of
        // PhysicsSystem::Update, unlike JobSystemSingleThreaded, which
        // asserts (SIGTRAP) as soon as a dynamic body creates dependent
        // integration jobs.
        static JPH::TempAllocatorMalloc TEMP_ALLOC;
        static JPH::JobSystemThreadPool JOB_SYSTEM(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 0);
        const float SIM_DT = std::clamp(dt, 1.0f / 240.0f, 1.0f / 30.0f);
        // Sub-step the sim. One step of up to 1/30 moves a 5 m/s body 16 cm
        // -- enough to hop onto seam/trim geometry and fall back every few
        // frames (the wall-press jitter; 8.8 cm of position bounce measured
        // against the real map, zero with sub-steps). 3 sub-steps cap the
        // effective step at 1/90.
        g_joltSystem->Update(SIM_DT, 3, &TEMP_ALLOC, &JOB_SYSTEM);

        for (size_t i = 0; i < g_sceneObjects.size(); ++i) {
            auto& JB = g_joltBodies[i];
            auto& OBJ = g_sceneObjects[i];

            if (!OBJ.dynamic || !OBJ.physics || !JB.valid || i == g_mapGrabIndex)
                continue;

            const auto POS = g_bodyIf->GetPosition(JB.body);
            const auto ROT = g_bodyIf->GetRotation(JB.body);

            const Vec3 NEW_POS{POS.GetX(), POS.GetY(), POS.GetZ()};
            const Vec3 NEW_ROT = quatToEuler(ROT);

            const bool CHANGED =
                std::fabs(NEW_POS.x - OBJ.position.x) > 1e-6f ||
                std::fabs(NEW_POS.y - OBJ.position.y) > 1e-6f ||
                std::fabs(NEW_POS.z - OBJ.position.z) > 1e-6f ||
                std::fabs(NEW_ROT.x - OBJ.rotationDeg.x) > 1e-4f ||
                std::fabs(NEW_ROT.y - OBJ.rotationDeg.y) > 1e-4f ||
                std::fabs(NEW_ROT.z - OBJ.rotationDeg.z) > 1e-4f;

            if (CHANGED) {
                OBJ.position    = NEW_POS;
                OBJ.rotationDeg = NEW_ROT;
                g_scene.setSceneObjectTransform(i, NEW_POS, NEW_ROT);
            }
        }

        // Player readback: the camera derives from the body's eye point --
        // first person, or third person behind/front (F5) along the look
        // axes (the front view looks back at the character). While a
        // fullscreen transition animates the transition owns the camera.
        // (g_transition is the ROOM progress -- 1.0 in normal 3D -- it must
        // NOT gate this.)
        if (!g_playerBody.IsInvalid()) {
            const auto PPOS = g_bodyIf->GetPosition(g_playerBody);
            const Vec3 EYE{PPOS.GetX(), PPOS.GetY() + PLAYER_EYE_OFF,
                           PPOS.GetZ()};
            auto& CAM = g_scene.camera();

            if (g_fsPhase == EFullscreenPhase::None && g_viewMode == 0) {
                // First person: the camera IS the eye. The third-person
                // orbit is applied AFTER this frame's look update (see the
                // movement block) -- computing it here would position the
                // camera by the OLD yaw while the view points by the NEW
                // one, and the whole world would step on every turn.
                if (EYE.x != CAM.position.x || EYE.y != CAM.position.y ||
                    EYE.z != CAM.position.z) {
                    CAM.position = EYE;
                    damageCurrentMonitor();
                }
            }
            CAM.mirrorView = g_viewMode == 2;

            g_grounded = playerGrounded();

            // The character renders in third person only; F3 adds the
            // capsule outline.
            g_scene.setPlayerVisible(g_viewMode != 0);
            g_scene.setPlayerDebugCapsule(
                Vec3{PPOS.GetX(), PPOS.GetY(), PPOS.GetZ()}, g_debugHud);

            // Character state: air = jump (plays once and holds), grounded =
            // walk/run by horizontal speed, else idle. The POSE (the facing)
            // is applied after this frame's look update below -- applying it
            // here would render the model one camera-frame behind, stepping
            // after the mouse on every turn.
            if (g_scene.player()->loaded()) {
                const auto VEL = g_bodyIf->GetLinearVelocity(g_playerBody);
                const float HSP = std::sqrt(VEL.GetX() * VEL.GetX() +
                                            VEL.GetZ() * VEL.GetZ());
                const auto ST = !g_grounded
                    ? CPlayerModel::EState::Jump
                    : HSP > 0.4f ? (g_keySprint ? CPlayerModel::EState::Run
                                                : CPlayerModel::EState::Walk)
                                 : CPlayerModel::EState::Idle;
                g_scene.player()->setState(ST);
            }
        }

        g_msJolt = g_msJolt * 0.9 +
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - JOLT_T0).count() * 0.1;
    }

    g_objTrees.resize(g_sceneObjects.size());
    g_objTreeGens.resize(g_sceneObjects.size(), 0);

    for (size_t i = 0; i < g_sceneObjects.size(); ++i) {
        const uint32_t GEN =
            g_scene.sceneModel(i) ? g_scene.sceneModel(i)->generation() : 0;

        // An AWAKE dynamic Jolt body moves every frame: its collision tree
        // would cost O(n log n) per rebuild, so while it moves the tree is
        // cleared and excluded from player queries; Jolt puts the body to
        // sleep at rest and the tree then builds once.
        const bool AWAKE =
            g_sceneObjects[i].dynamic && g_joltBodies[i].valid && g_bodyIf &&
            g_bodyIf->IsActive(g_joltBodies[i].body);

        const bool SKIP = !g_sceneObjects[i].collision ||
            g_mapGrabIndex == i || AWAKE;

        if (SKIP) {
            // Collision off / carried / still moving: the tree must be EMPTY
            // (the clear belongs to this branch only -- clearing after a
            // build wiped every just-built tree and killed all collision).
            if (!g_objTrees[i].empty())
                g_objTrees[i].clear();
            g_objTreeGens[i] = GEN;
        } else if (g_objTreeGens[i] != GEN) {
            if (const auto* MODEL = g_scene.sceneModel(i))
                g_objTrees[i].build(MODEL->triangles());
            g_objTreeGens[i] = GEN;
        }
    }

    if (g_input.consumeLook(yawDelta, pitchDelta, dt)) {
        g_scene.rotateView(yawDelta, pitchDelta);
        damageCurrentMonitor();
    }

    // C-key view zoom glide: exponential toward the target (the wheel level
    // while held, 1x when released), so both directions are smooth.
    const float ZOOM_TARGET = g_zoomHeld ? g_zoomWheel : 1.0f;
    g_zoomLevel += (ZOOM_TARGET - g_zoomLevel) *
        (1.0f - std::exp(-8.0f * dt));
    g_scene.setZoom(g_zoomLevel);

    // ---- player physics body: input -> velocity, Jolt owns the pose ----
    ensurePlayerBody();
    syncFloorBody();

    if (g_pointerGesture == EPointerGesture::CharDrag && g_pointerDown)
        carryCharacter(dt);
    syncCharacterBodies();
    g_scene.setHud(g_tool, toolLabels());
    {
        std::vector<GLScene::SPathView> VIEWS;
        VIEWS.reserve(g_paths.size());
        for (const auto& P : g_paths)
            VIEWS.push_back({P.points, P.smooth});
        g_scene.setPaths(VIEWS, g_activePath, pathTool());
    }
    for (size_t i = 0; i < g_chars.size(); ++i)
        g_scene.setCharacterPose(
            i, g_chars[i].feet,
            Vec3{g_chars[i].cfg.rotationDeg.x, g_chars[i].yawDeg,
                 g_chars[i].cfg.rotationDeg.z});

    if (!g_playerBody.IsInvalid()) {
        auto& CAM = g_scene.camera();

        // collision = false: the capsule joins no contact pair at all --
        // it flies/falls through everything (the honest noclip).
        g_bodyIf->SetObjectLayer(
            g_playerBody,
            g_playerCollision ? LAYER_MOVING : LAYER_GRABBED);

        // The camera was moved by something else (spawn reset, fullscreen
        // transition handoff): teleport the body under it. Skipped while a
        // transition animates (To2D/To3D own the camera then); the
        // transition's end pose resyncs on the first normal frame. First
        // person only: in third person the camera is DERIVED from the body.
        if (g_fsPhase == EFullscreenPhase::None && g_viewMode == 0) {
            const auto CUR = g_bodyIf->GetPosition(g_playerBody);
            const float DX = CAM.position.x - CUR.GetX();
            const float DY = CAM.position.y - (CUR.GetY() + PLAYER_EYE_OFF);
            const float DZ = CAM.position.z - CUR.GetZ();
            if (DX * DX + DY * DY + DZ * DZ > 0.25f)
                g_bodyIf->SetPositionAndRotationWhenChanged(
                    g_playerBody,
                    JPH::RVec3(CAM.position.x, CAM.position.y - PLAYER_EYE_OFF,
                               CAM.position.z),
                    JPH::Quat::sIdentity(), JPH::EActivation::Activate);
        }

        // Noclip: the body sits on the layer that collides with nothing
        // (the one carried objects use), so walls, floors and the grid
        // slab let it through; the camera still rides it.
        const JPH::ObjectLayer WANT_LAYER =
            g_playerNoclip ? LAYER_GRABBED : LAYER_MOVING;
        if (g_bodyIf->GetObjectLayer(g_playerBody) != WANT_LAYER)
            g_bodyIf->SetObjectLayer(g_playerBody, WANT_LAYER);

        applyCameraMovement(dt); // -> s_moveVel

        // The character's facing: RY(pi - yaw) maps the authored +Z front
        // onto the look direction. Applied AFTER this frame's look update
        // and assigned DIRECTLY -- any smoothing here is second-order on
        // top of the look inertia and reads as a staircase on fast turns.
        {
            const auto PPOS = g_bodyIf->GetPosition(g_playerBody);
            g_scene.setPlayerPose(
                Vec3{PPOS.GetX(), PPOS.GetY() - Camera::kBodyHeight * 0.5f,
                     PPOS.GetZ()},
                3.14159265f - CAM.yaw);

            // The third-person camera SPHERICALLY orbits the player: yaw
            // sweeps the horizontal ring, pitch lifts/drops the camera
            // around the anchor (behind on 0 pitch, above on negative,
            // below on positive), at the same eye level as first person on
            // the zero pitch. Collision-aware: a ray from the head toward
            // the orbit position pulls the camera in front of walls.
            if (g_viewMode != 0) {
                const Vec3 ORBIT_EYE{PPOS.GetX(),
                                     PPOS.GetY() + PLAYER_EYE_OFF,
                                     PPOS.GetZ()};
                const Vec3 FLAT = CAM.flatForward();
                const float P = CAM.pitch;
                Vec3 DIR =
                    FLAT * (-std::cos(P)) + Vec3{0.f, 1.f, 0.f} * (-std::sin(P));
                if (g_viewMode == 2)
                    DIR.x *= -1.f, DIR.y *= -1.f, DIR.z *= -1.f;

                // The distance glides toward the full orbit radius, but is
                // CLAMPED by the ray each frame: the ray probes the FULL
                // orbit distance, and the camera never sits beyond the hit
                // (a hand's width before the wall). Turning changes the hit
                // distance gradually, so the camera slides along walls; the
                // pop-out glides; the push-in can never cross.
                g_playerCamDist += (kThirdDist - g_playerCamDist) *
                    (1.0f - std::exp(-8.0f * dt));

                if (kThirdDist > 1e-4f) {
                    const JPH::RRayCast RAY{
                        JPH::RVec3(ORBIT_EYE.x, ORBIT_EYE.y, ORBIT_EYE.z),
                        JPH::Vec3(DIR.x * kThirdDist, DIR.y * kThirdDist,
                                  DIR.z * kThirdDist)};
                    JPH::RayCastResult HIT;
                    const JPH::IgnoreSingleBodyFilter SKIP_SELF(g_playerBody);
                    if (g_joltSystem->GetNarrowPhaseQuery().CastRay(
                            RAY, HIT, JPH::BroadPhaseLayerFilter(),
                            JPH::ObjectLayerFilter(), SKIP_SELF)) {
                        g_playerCamDist = std::min(
                            g_playerCamDist,
                            std::max(0.05f,
                                     HIT.mFraction * kThirdDist - 0.15f));
                    }
                }
                g_playerCamDist = std::max(g_playerCamDist, 0.05f);

                const Vec3 WANT = ORBIT_EYE + DIR * g_playerCamDist;

                if (WANT.x != CAM.position.x || WANT.y != CAM.position.y ||
                    WANT.z != CAM.position.z) {
                    CAM.position = WANT;
                    damageCurrentMonitor();
                }
            }
        }

        // Flying: all three axes key-driven, gravity asleep. Walking: the
        // keys own the horizontal plane, Jolt's gravity the vertical one.
        if (playerFlies()) {
            g_bodyIf->SetGravityFactor(g_playerBody, 0.0f);
            g_bodyIf->SetLinearVelocity(g_playerBody, JPH::Vec3(
                s_moveVel.x, s_moveVel.y, s_moveVel.z));
        } else {
            g_bodyIf->SetGravityFactor(g_playerBody, 1.0f);

            auto VEL = g_bodyIf->GetLinearVelocity(g_playerBody);
            float VY = VEL.GetY();
            if (VY < -40.0f) // terminal velocity, as before
                VY = -40.0f;

            if (g_playerJumpQueued) {
                g_playerJumpQueued = false;
                if (g_grounded)
                    VY = 5.5f;
            }

            g_bodyIf->SetLinearVelocity(g_playerBody, JPH::Vec3(
                s_moveVel.x, VY, s_moveVel.z));
        }

        // Walk bob (view-only, walking + grounded): a small vertical sway
        // synced to the distance traveled, plus a head TILT at half the
        // bob frequency (in a real gait the full left-right roll cycle
        // spans two vertical bounces). The amplitude eases in and out, so
        // jumps, stops and the walk<->fly switch never pop.
        const float BOB_SPEED = std::sqrt(s_moveVel.x * s_moveVel.x +
                                          s_moveVel.z * s_moveVel.z);
        const bool BOBING = g_cfgWalkBob && !playerFlies() && g_grounded &&
            g_fsPhase == EFullscreenPhase::None && g_viewMode == 0;
        const float TARGET_AMP =
            BOBING ? kBobAmplitude *
                std::min(1.0f, BOB_SPEED / std::max(CAM.moveSpeed, 0.5f))
                   : 0.0f;
        s_bobAmp += (TARGET_AMP - s_bobAmp) * (1.0f - std::exp(-8.0f * dt));

        // The phase wraps at 4pi: two bob cycles, so sin(phase/2) walks a
        // FULL tilt cycle (left, then right) and the wrap lands on zero.
        if (BOBING)
            s_bobPhase = std::fmod(s_bobPhase + BOB_SPEED * dt * kBobRate,
                                   12.5663706f);

        // Both offsets ride the same eased envelope: the roll fades out
        // with the bob (walk_bob off, flight, air, transitions).
        if (s_bobAmp > 0.0005f) {
            CAM.position.y += s_bobAmp * std::sin(s_bobPhase);
            CAM.roll = (s_bobAmp * (1.0f / kBobAmplitude)) * kBobRoll *
                std::sin(s_bobPhase * 0.5f);
        } else {
            CAM.roll = 0.0f;
        }
    }

    // Map-drag carry: the grabbed object's CENTER rides the crosshair at
    // the pickup distance, and the object turns to face the player (the
    // same billboard convention as window dragging). The new placement is
    // written INTO THE CONFIG OBJECT first: update3D pushes the config
    // specs into the scene every frame, and a carry that only touched the
    // scene slot would be overwritten right back -- the object teleported
    // home on release and both writes recomputed the geometry every frame
    // (the freezes). With the config object as the single source of truth
    // the per-frame push compares equal and skips.
    if (g_pointerGesture == EPointerGesture::MapDrag && g_pointerDown &&
        g_mapGrabIndex != SIZE_MAX) {
        if (g_mapGrabIndex >= g_sceneObjects.size()) {
            g_mapGrabIndex = SIZE_MAX; // config reloaded mid-carry: drop it
        } else {
        const auto& CAM = g_scene.camera();
        const Vec3 TARGET = CAM.position + CAM.centerRay() * g_mapGrabDist;

        // Face the player: the object's +Z normal toward the camera (the
        // model convention maps the normal's Y to -sin(pitch), so pitch =
        // asin(-TO.y) -- the same math as the window drag billboard).
        const Vec3 TO_CAM = normalize(CAM.position - TARGET);
        const float TARGET_YAW = std::atan2(TO_CAM.x, TO_CAM.z);
        const float TARGET_PITCH =
            std::asin(std::clamp(-TO_CAM.y, -1.0f, 1.0f));

        // Smoothly chase the target orientation (wrap-aware on yaw).
        auto& ROT = g_sceneObjects[g_mapGrabIndex].rotationDeg;
        constexpr float DEG = 3.14159265358979f / 180.0f;

        float curYaw = ROT.y * DEG;
        float curPitch = ROT.x * DEG;

        float dYaw = TARGET_YAW - curYaw;
        while (dYaw > 3.14159265f)
            dYaw -= 6.28318531f;
        while (dYaw < -3.14159265f)
            dYaw += 6.28318531f;

        const float K = 1.0f - std::exp(-10.0f * dt);
        curYaw += dYaw * K;
        curPitch += (TARGET_PITCH - curPitch) * K;

        // Full facing on all three axes: the roll eases to zero as well,
        // same exponential as yaw/pitch.
        ROT.z += (0.0f - ROT.z) * K;

        ROT.x = curPitch / DEG;
        ROT.y = curYaw / DEG;

        g_sceneObjects[g_mapGrabIndex].position = TARGET;
        g_scene.setSceneObjectTransform(g_mapGrabIndex, TARGET, ROT);

        if (g_bodyIf && g_mapGrabIndex < g_joltBodies.size() &&
            g_joltBodies[g_mapGrabIndex].valid) {
            const auto JB = g_joltBodies[g_mapGrabIndex].body;
            g_bodyIf->SetLinearVelocity(JB, JPH::Vec3::sZero());
            // Pose-driven while carried: no contact pairs, or the object
            // would shove the player body around on the way.
            g_bodyIf->SetObjectLayer(JB, LAYER_GRABBED);
            g_bodyIf->SetPositionAndRotationWhenChanged(
                JB, JPH::RVec3(TARGET.x, TARGET.y, TARGET.z),
                eulerToQuat(ROT), JPH::EActivation::Activate);
        }

        damageCurrentMonitor();
        }
    }

    // Super+wheel hover zoom: glide the window along its ray toward the
    // target distance. Paused while a fullscreen transition animates (its
    // own glide is one-shot and completes separately).
    if (s_zoomId != 0 && g_fsPhase == EFullscreenPhase::None) {
        if (auto* ZOOMED = g_world.find(s_zoomId)) {
            s_zoomCur += (s_zoomTarget - s_zoomCur) *
                (1.0 - std::exp(-8.0 * dt));

            // Glide along the ray ANCHORED at the last wheel event: the
            // window stays world-fixed instead of riding the camera.
            ZOOMED->center = s_zoomAnchor +
                s_zoomDir * static_cast<float>(s_zoomCur);

            // One-shot glide: once the target is reached the state is
            // dropped and the window is world-fixed again -- a live zoom
            // state would keep the window glued to a moving camera.
            if (std::fabs(s_zoomTarget - s_zoomCur) < 0.02)
                s_zoomId = 0;
        } else {
            s_zoomId = 0;
        }
    }

    if (g_transition <= 0.0f)
        return;

    // Fullscreen-exit box re-assertion: CONDITION-driven, not frame-count --
    // keep re-asserting until the window's box matches ours for 15
    // consecutive frames, outlasting ANY late Hyprland-side restore.
    if (g_fsAssertFrames > 0) {
        if (auto W = g_fsWindow.lock()) {
            Compat::setWindowBox(W, g_fsAssertBox);

            const auto CUR = Compat::currentWindowBox(W);

            if (CUR.x == g_fsAssertBox.x && CUR.y == g_fsAssertBox.y &&
                CUR.w == g_fsAssertBox.w && CUR.h == g_fsAssertBox.h) {
                if (++g_fsStableCount >= 15) {
                    g_fsAssertFrames = 0;
                    g_fsWindow       = {};
                }
            } else {
                g_fsStableCount = 0;
            }
        } else {
            g_fsAssertFrames = 0;
        }
    }

    const auto MON = targetMonitor();

    if (!MON)
        return;

    updatePanorama();

    // The camera pose has changed for this frame. Use that same new centre ray
    // for the active gesture, so movement and resize track exactly what the
    // user is looking at rather than the previous frame's ray.
    if (g_pointerGesture == EPointerGesture::Move3D && g_pointerDown) {
        g_world.updateDrag(
            g_scene.camera().position,
            g_scene.camera().centerRay(),
            dt
        );
    } else if (g_pointerGesture == EPointerGesture::ResizeReal && g_pointerDown) {
        updateRealResize();
    } else if (g_pointerGesture == EPointerGesture::WheelRoll && g_pointerDown) {
        updateWheelRoll();
    }

    syncWorld(MON, dt);

    applyFullscreenAnimation();

    // Normal client interaction is a virtual pointer located exactly at the
    // crosshair. It is updated every frame after camera motion, so buttons,
    // text fields, scrollbars, etc. receive ordinary Wayland pointer motion.
    if (!g_pointerDown && g_fsPhase == EFullscreenPhase::None &&
        !pointerFree())
        forwardPointerToAim(inputTimeMs());
}

class CHypr3DPassElement final : public IPassElement {
  public:
    CHypr3DPassElement(float alpha, float dt) :
        m_alpha(alpha), m_dt(dt) {}

    std::vector<UP<IPassElement>> draw() override {
        const auto GATE = [&](const std::string& why) {
            g_lastRenderGate = why;
            reportFramebufferErrorOnce(why);
            return std::vector<UP<IPassElement>>{};
        };

        if (!g_pHyprRenderer)
            return GATE("[hypr3d] g_pHyprRenderer is null");

        if (g_pHyprRenderer->type() != Render::IHyprRenderer::RT_GL)
            return GATE("[hypr3d] renderer is not OpenGL");

        auto& renderData = g_pHyprRenderer->m_renderData;

        if (!renderData.currentFB)
            return GATE("[hypr3d] current framebuffer is null");

        auto* framebuffer =
            dynamic_cast<Render::GL::CGLFramebuffer*>(
                renderData.currentFB.get()
            );

        if (!framebuffer)
            return GATE("[hypr3d] current framebuffer is not CGLFramebuffer");

        const int width = std::max(
            1,
            static_cast<int>(std::round(renderData.currentFB->m_size.x))
        );

        const int height = std::max(
            1,
            static_cast<int>(std::round(renderData.currentFB->m_size.y))
        );

        const GLuint framebufferID = framebuffer->getFBID();

        if (framebufferID == 0)
            return GATE("[hypr3d] current framebuffer has ID 0");

        if (!Render::GL::g_pHyprOpenGL) {
            reportFramebufferErrorOnce("[hypr3d] g_pHyprOpenGL is null");
            return {};
        }

        Render::GL::g_pHyprOpenGL->makeEGLCurrent();

        const auto R_T0 = std::chrono::steady_clock::now();
        const bool result = g_scene.render(
            framebufferID,
            width,
            height,
            m_alpha,
            m_dt,
            g_renderWindows
        );
        g_msRender = g_msRender * 0.9 +
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - R_T0).count() * 0.1;

        if (!result) {
            if (!g_reportedRenderError) {
                g_reportedRenderError = true;
                notify(
                    "[hypr3d] GLScene::render failed",
                    CHyprColor{1.0f, 0.2f, 0.2f, 1.0f}
                );
            }
            g_lastRenderGate = "GLScene::render returned false";
            return {};
        }

        g_lastRenderGate.clear();

        if (!g_renderedOnce) {
            g_renderedOnce = true;

            notify(
                "[hypr3d] 3D pass executed " +
                    std::to_string(width) + "x" + std::to_string(height),
                CHyprColor{0.2f, 1.0f, 0.5f, 1.0f}
            );
        }

        return {};
    }

    bool needsLiveBlur() override {
        return false;
    }

    bool needsPrecomputeBlur() override {
        return false;
    }

    const char* passName() override {
        return "Hypr3D";
    }

    ePassElementType type() override {
        return EK_CUSTOM;
    }

    bool undiscardable() override {
        return true;
    }

    bool disableSimplification() override {
        return true;
    }

    std::optional<CBox> boundingBox() override {
        return std::nullopt;
    }

    CRegion opaqueRegion() override {
        return {};
    }

  private:
    static void reportFramebufferErrorOnce(const std::string& text) {
        if (g_reportedFramebufferError)
            return;

        g_reportedFramebufferError = true;

        notify(text, CHyprColor{1.0f, 0.2f, 0.2f, 1.0f});
    }

    float m_alpha;
    float m_dt;
};

// Throttled snapshot of the live state, written where I can read it directly
// instead of asking the user to describe what they see.
static void dumpStatus() {
    const auto NOW = std::chrono::steady_clock::now();

    if (std::chrono::duration<float>(NOW - g_diagLastDump).count() < 0.25f)
        return;

    g_diagLastDump = NOW;
    ++g_diagFrames;

    // Requested here, filled in later this frame by the pass element, read by
    // the next dump -- one frame of lag on a value that is not changing.
    g_scene.requestProbe();

    // History log: APPEND with a timestamp. The one-shot snapshot kept
    // missing the bug (it self-healed before the read); the timeline catches
    // the transition frame by frame. Bounded at ~128 KB, trimmed from the
    // head.
    std::ofstream out("/tmp/hypr3d-status.txt",
                      std::ios::app | std::ios::in);

    {
        std::error_code              ec;
        const auto                   SZ = std::filesystem::file_size(
            "/tmp/hypr3d-status.txt", ec);
        if (!ec && SZ > 128 * 1024) {
            std::ifstream  in("/tmp/hypr3d-status.txt");
            std::string    data((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
            in.close();

            const auto CUT = data.find('\n', data.size() / 2);
            if (CUT != std::string::npos) {
                std::ofstream trim("/tmp/hypr3d-status.txt",
                                   std::ios::trunc);
                trim << data.substr(CUT + 1);
            }
        }
    }

    out << "--- frame " << g_diagFrames << " t="
        << std::chrono::duration<float>(std::chrono::steady_clock::now() -
                                        g_inputClockStart)
               .count()
        << "s\n";

    if (!out)
        return;

    const auto MON = targetMonitor();

    out << "active=" << (g_active ? 1 : 0) << " frames=" << g_diagFrames
        << " transition=" << g_transition
        << " target=" << g_transitionTarget
        << " alphaSent=" << g_diagAlpha
        << " renderedOnce=" << (g_renderedOnce ? 1 : 0) << "\n";

    out << "hookInstalled=" << (g_hookInstalled ? 1 : 0)
        << " hookActive=" << (Compat::pointerHookActive() ? 1 : 0)
        << " sinkCalls=" << g_diagSinkCalls
        << " moveEvents=" << g_diagMoveEvents << "\n";

    out << "lastPos=" << g_diagLastPos.x << "," << g_diagLastPos.y
        << " pinned=" << g_diagPinned.x << "," << g_diagPinned.y
        << " lastDelta=" << g_diagLastDx << "," << g_diagLastDy << "\n";

    out << "lastZoomStep=" << g_diagLastZoomStep
        << " (positive = wheel forward = push away)\n";

    // warpAfter is what the compositor reported immediately after the last
    // warp: equal to `center` means the pin sticks, equal to the pre-warp
    // position means warpTo is a no-op here.
    out << "warpCalls=" << g_diagWarpCalls
        << " warpAfter=" << g_diagWarpAfter.x << "," << g_diagWarpAfter.y
        << " warpRadius=" << kWarpRadiusPx << "\n";

    // mgrPos vs lastPos: if these differ persistently, the event's position
    // and Pointer::mgr()'s position are different quantities and only one of
    // them can be used to derive a delta.
    out << "mgrPos=" << g_diagMgrPos.x << "," << g_diagMgrPos.y << "\n";

    const Vector2D CENTER = crosshairLogical();

    if (MON)
        out << "monitor=" << MON->m_name
            << " pos=" << MON->m_position.x << "," << MON->m_position.y
            << " size=" << MON->m_size.x << "x" << MON->m_size.y
            << " pixel=" << MON->m_pixelSize.x << "x" << MON->m_pixelSize.y
            << " center=" << CENTER.x << "," << CENTER.y << "\n";
    else
        out << "monitor=none\n";

    // The decisive number for the "2D desktop showing through" report: the
    // alpha actually sitting in the offscreen scene buffer at screen centre.
    if (g_scene.probeValid()) {
        const unsigned char* P = g_scene.probeRGBA();
        out << "scenePixel=" << int(P[0]) << "," << int(P[1]) << ","
            << int(P[2]) << "," << int(P[3]) << "\n";
    }
    else
        out << "scenePixel=unavailable\n";

    out << "renderWindows=" << g_renderWindows.size() << "\n";

    // Window-depth pipeline trace: the configured thickness and the
    // silhouette each window draws its walls from (windows: traced from the
    // snapshot texture via the GPU copy, fallback analytic; layers: traced
    // from the picking mask). skirt=0/err=N says the copy failed and why.
    for (const auto& RW : g_renderWindows) {
        out << "  rwin=" << RW.id << " depth=" << RW.depth
            << " outlines=" << (RW.outlines ? (long)RW.outlines->size() : -1);

        if (RW.outlines && !RW.outlines->empty()) {
            const auto& L = (*RW.outlines)[0];
            out << " pts=" << L.pts.size();
            if (!L.pts.empty())
                out << " p0=" << L.pts[0].x << "," << L.pts[0].y;
        }

        if (const auto* SN = g_capture.get(RW.id))
            out << " skirt=" << (SN->skirtValid ? 1 : 0)
                << " err=" << SN->skirtError
                << " mask=" << SN->skirtW << "x" << SN->skirtH;

        out << "\n";
    }

    out << "renderGate=" << (g_lastRenderGate.empty() ? "none" : g_lastRenderGate)
        << "\n";
    out << "renderer=" << (g_pHyprRenderer ? "ok" : "NULL")
        << " rtype=" << (g_pHyprRenderer ? (int)g_pHyprRenderer->type() : -1)
        << " (RT_GL=" << (int)Render::IHyprRenderer::RT_GL << ")\n";
    out << "msUpdate3D=" << g_msUpdate3D << " msJolt=" << g_msJolt
        << " msRender=" << g_msRender << "\n";
    out << "lastError=" << (g_lastError.empty() ? "none" : g_lastError)
        << "\n";
    out << "sceneObjects=" << g_sceneObjects.size() << " joltBodies="
        << g_joltBodies.size() << " objTrees=" << g_objTrees.size() << "\n";

    out << "fsPhase=" << static_cast<int>(g_fsPhase)
        << " fsWasOn=" << (g_fsWasOn ? 1 : 0)
        << " fsAlpha=" << g_fsAlpha << "\n";

    if (const auto FSW = Fullscreen::controller()->getFullscreenWindow(
            targetMonitor())) {
        const auto BOX = Compat::currentWindowBox(FSW);
        float fsRoll = 0.0f;
        if (auto* E = g_world.find(Compat::windowId(FSW)))
            fsRoll = E->roll;

        out << "fsWindow=" << Compat::windowId(FSW)
            << " box=" << BOX.x << "," << BOX.y << "," << BOX.w << "," << BOX.h
            << " restore=" << g_fsRestoreBox.w << "x" << g_fsRestoreBox.h
            << " roll=" << fsRoll << "\n";

    } else {
        out << "fsWindow=none\n";
    }
    // Scene-object physics state: one line per object.
    for (size_t i = 0; i < g_sceneObjects.size() && i < g_objTrees.size(); ++i) {
        const auto& OBJ = g_sceneObjects[i];
        const auto& J = i < g_joltBodies.size() ? g_joltBodies[i]
                                                : SObjJolt{};
        out << "  obj" << i << "="
            << OBJ.path.substr(OBJ.path.size() -
                               std::min<size_t>(OBJ.path.size(), 24))
            << " dyn=" << (OBJ.dynamic ? 1 : 0)
            << " phys=" << (OBJ.physics ? 1 : 0)
            << " col=" << (OBJ.collision ? 1 : 0)
            << " pos=" << OBJ.position.x << "," << OBJ.position.y << ","
            << OBJ.position.z
            << " tree=" << (g_objTrees[i].empty() ? 0 : 1)
            << " jolt=" << (J.valid ? 1 : 0)
            << "\n";
    }

    // Per-window channel alphas + Hyprland's remembered floating size:
    // the semi-transparency and the giant-size bugs live in THESE state
    // channels; the dump pins which one is stuck and for whom.
    if (Desktop::windowState() && MON && MON->m_activeWorkspace) {
        for (auto const& W3 : Desktop::windowState()->windows()) {
            if (!W3 || W3->m_workspace != MON->m_activeWorkspace)
                continue;

            const auto WB = W3->m_target ? W3->m_target->position() :
                                           CBox{};
            out << "  win=" << Compat::windowId(W3)
                << " float=" << (W3->m_isFloating ? 1 : 0)
                << " aFull="
                << W3->alphaValue(Desktop::View::WINDOW_ALPHA_FULLSCREEN)
                << " aActive="
                << W3->alphaValue(Desktop::View::WINDOW_ALPHA_ACTIVE)
                << " aFade="
                << W3->alphaValue(Desktop::View::WINDOW_ALPHA_FADE)
                << " box=" << WB.x << "," << WB.y << "," << WB.w << ","
                << WB.h << " lastFloat="
                << (W3->m_target ? W3->m_target->lastFloatingSize().x : -1.f)
                << "x"
                << (W3->m_target ? W3->m_target->lastFloatingSize().y : -1.f)
                << "\n";
        }
    }

    out << "captureFrames=" << g_captureFrames << "\n";

    {
        const auto& REQ = Compat::lastCursorRequest();
        SP<Render::ITexture> TEX;
        Vector2D SIZE, HOT;
        const bool OK = clientCursorImage(TEX, SIZE, HOT);

        out << "cursorImage: serial=" << REQ.serial
            << " kind=" << (REQ.buffer ? "buffer" : REQ.surface ? "surface" : "none")
            << " usable=" << OK;
        if (TEX)
            out << " tex=" << TEX->m_texID << " type=" << (int)TEX->m_type
                << " texSize=" << TEX->m_size.x << "x" << TEX->m_size.y
                << " fmt=0x" << std::hex << TEX->m_drmFormat << std::dec;
        out << " size=" << SIZE.x << "x" << SIZE.y << " hot=" << HOT.x << ","
            << HOT.y << " vcSpace=" << (int)g_vcSpace << "\n";
    }

    // Player body trace: wall glue, stick-slip or a resync fight show up
    // here directly (position/velocity vs the commanded velocity).
    if (!g_playerBody.IsInvalid() && g_bodyIf) {
        const auto PP = g_bodyIf->GetPosition(g_playerBody);
        const auto PV = g_bodyIf->GetLinearVelocity(g_playerBody);
        out << "player: pos=(" << PP.GetX() << "," << PP.GetY() << ","
            << PP.GetZ() << ") vel=(" << PV.GetX() << "," << PV.GetY()
            << "," << PV.GetZ() << ") moveVel=(" << s_moveVel.x << ","
            << s_moveVel.y << "," << s_moveVel.z << ") grounded="
            << g_grounded << " flying=" << g_playerFlying << " noclip=" << g_playerNoclip << "\n";
    }
}

// --- typing-mode virtual cursor -----------------------------------------------
//
// In typing mode the pointer stays captured and drives a cursor of the
// plugin's own, which lives in one of three spaces:
//   Window  -- on a window, in the window's own pixels: mouse motion moves it
//              1:1 across the content whatever the window's angle, the client
//              gets ordinary pointer input there, and it is drawn in the
//              window's plane.
//   Screen  -- off windows, in the room monitor's pixels. Every move casts a
//              ray through it; crossing a window drops into that window's
//              space and focuses it.
//   Desktop -- past the room monitor's edge: capture is released and the real
//              cursor carries on across the other monitors as usual; coming
//              back over the room recaptures it.
// Without the pointer hook there is no relative motion to drive it, and
// typing mode falls back to the plain free cursor.

// The camera's screen basis, rolled exactly like Camera::view() and with the
// zoomed fov the scene renders with (same derivation as the panorama).
struct SViewBasis {
    Vec3  pos, fwd, right, up;
    float tanX = 1.0f, tanY = 1.0f;
};

static SViewBasis viewBasis(const PHLMONITOR& mon) {
    const auto& CAM = g_scene.camera();

    SViewBasis B;
    B.pos = CAM.position;
    B.fwd = CAM.forward();

    const Vec3 RIGHT = CAM.right();
    const Vec3 UP    = cross(RIGHT, B.fwd);

    B.right = RIGHT * std::cos(CAM.roll) - UP * std::sin(CAM.roll);
    B.up    = UP * std::cos(CAM.roll) + RIGHT * std::sin(CAM.roll);

    B.tanY = std::tan(kFovDeg * 3.14159265f / 360.0f) /
        std::max(g_scene.zoom(), 0.01f);
    B.tanX = B.tanY * static_cast<float>(mon->m_size.x / mon->m_size.y);
    return B;
}

static Vec3 screenRay(const PHLMONITOR& mon, const Vector2D& screen) {
    const auto B = viewBasis(mon);

    const float NX = static_cast<float>(screen.x / mon->m_size.x) * 2.0f - 1.0f;
    const float NY = 1.0f - static_cast<float>(screen.y / mon->m_size.y) * 2.0f;

    return normalize(B.fwd + B.right * (NX * B.tanX) + B.up * (NY * B.tanY));
}

static bool projectToScreen(const PHLMONITOR& mon, const Vec3& point,
                            Vector2D& out) {
    const auto B = viewBasis(mon);
    const Vec3 D = point - B.pos;
    const float Z = dot(D, B.fwd);

    if (Z <= 1e-4f)
        return false; // behind the camera

    const float NX = dot(D, B.right) / (Z * B.tanX);
    const float NY = dot(D, B.up) / (Z * B.tanY);

    out = Vector2D{(NX + 1.0f) * 0.5f * mon->m_size.x,
                   (1.0f - NY) * 0.5f * mon->m_size.y};
    return true;
}

// World point of a decorated-box px position on an entity (it may lie
// outside the box, on the window's plane).
static Vec3 windowPoint(const World3D::SEntity& e, const Vector2D& local) {
    const float X =
        (static_cast<float>(local.x) / e.logicalWidth - 0.5f) * e.width;
    const float Y =
        (0.5f - static_cast<float>(local.y) / e.logicalHeight) * e.height;

    return e.center + g_world.rightOf(e.id) * X + g_world.upOf(e.id) * Y;
}

// Decorated-box px -> client-surface px, the border ring clamped to the
// surface edge (the same mapping as localFromHit).
static Vector2D surfaceLocal(const World3D::SEntity& e, const Vector2D& local) {
    return {
        std::clamp(local.x - e.surfaceOffsetX, 0.0, (double)e.surfaceWidth),
        std::clamp(local.y - e.surfaceOffsetY, 0.0, (double)e.surfaceHeight),
    };
}

static void vcDeliverMotion() {
    const auto* E = g_world.find(g_vcId);
    if (!E)
        return;

    const auto TARGET = targetFromHit(g_vcId);
    const auto LOCAL  = surfaceLocal(*E, g_vcLocal);

    if (TARGET.layer)
        Compat::deliverMotion(TARGET.layer, LOCAL, inputTimeMs());
    else if (TARGET.window)
        Compat::deliverMotion(TARGET.window, LOCAL, inputTimeMs());
}

static void vcEnterWindow(const World3D::SHit& hit) {
    const auto* E = g_world.find(hit.id);
    if (!E)
        return;

    g_vcSpace = ECursorSpace::Window;
    g_vcId    = hit.id;
    g_vcLocal = Vector2D{std::clamp(hit.u, 0.0f, 1.0f) * E->logicalWidth,
                         std::clamp(hit.v, 0.0f, 1.0f) * E->logicalHeight};

    // Focus follows the cursor; layers (bars, panels) never take it, and a
    // layer holding the keyboard (a launcher) keeps it until a click.
    const auto TARGET = targetFromHit(hit.id);
    if (TARGET.window && g_lastFocusId != hit.id &&
        !Compat::layerHasKeyboardFocus()) {
        Compat::focusWindow(TARGET.window);
        g_lastFocusId = hit.id;
    }

    vcDeliverMotion();
}

static PHLMONITOR monitorAt(const Vector2D& global) {
    if (!State::monitorState())
        return nullptr;

    for (const auto& MON : State::monitorState()->monitors()) {
        if (MON && CBox{MON->m_position, MON->m_size}.containsPoint(global))
            return MON;
    }

    return nullptr;
}

static void vcLeaveToDesktop(const Vector2D& global) {
    Compat::clearPointerFocus();

    g_vcSpace = ECursorSpace::Desktop;
    g_vcId    = 0;

    // The desktop's focus-follows-mouse takes over; re-entering any room
    // window, even the one just left, must focus it again.
    g_lastFocusId = 0;

    Compat::setPointerCapture(false);
    Pointer::mgr()->warpTo(global);
    Compat::setCursorHidden(false);

    if (g_pHyprRenderer)
        g_pHyprRenderer->setCursorFromName("default", true);
}

// Screen space at g_vcScreen: leave for a neighbouring monitor past the edge
// (clamp where there is none), else drop into whatever window is under it.
static void vcResolveScreen(const PHLMONITOR& mon) {
    const Vector2D SIZE = mon->m_size;

    if (g_vcScreen.x < 0 || g_vcScreen.y < 0 || g_vcScreen.x >= SIZE.x ||
        g_vcScreen.y >= SIZE.y) {
        const Vector2D GLOBAL = mon->m_position + g_vcScreen;

        if (const auto NEXT = monitorAt(GLOBAL); NEXT && NEXT != mon) {
            vcLeaveToDesktop(GLOBAL);
            return;
        }

        g_vcScreen = Vector2D{std::clamp(g_vcScreen.x, 0.0, SIZE.x - 1),
                              std::clamp(g_vcScreen.y, 0.0, SIZE.y - 1)};
    }

    // A bar, sidebar, launcher or notification drawn over the room: hand
    // the pointer to the real cursor there, Hyprland drives it natively.
    if (const Vector2D GLOBAL = mon->m_position + g_vcScreen;
        g_cfgUnderLayers && Compat::interactiveLayerAt(mon, GLOBAL)) {
        vcLeaveToDesktop(GLOBAL);
        return;
    }

    const World3D::SHit HIT = pickVisible(screenRay(mon, g_vcScreen));

    if (HIT.hit) {
        vcEnterWindow(HIT);
        return;
    }

    if (g_vcSpace == ECursorSpace::Window)
        Compat::clearPointerFocus();

    g_vcSpace = ECursorSpace::Screen;
    g_vcId    = 0;
}

static void vcMove(double dx, double dy) {
    const auto MON = targetMonitor();
    if (!MON)
        return;

    if (g_vcSpace == ECursorSpace::Window) {
        const auto* E = g_world.find(g_vcId);

        if (E && E->logicalWidth > 0 && E->logicalHeight > 0) {
            Vector2D next = g_vcLocal + Vector2D{dx, dy};

            // A held button keeps the window: a drag-select runs to the
            // edge and stays there, like on a flat desktop.
            if (g_clientButtonDown) {
                next = Vector2D{std::clamp(next.x, 0.0, (double)E->logicalWidth),
                                std::clamp(next.y, 0.0, (double)E->logicalHeight)};
                g_vcLocal = next;
                g_clientButtonLocal = surfaceLocal(*E, g_vcLocal);
                vcDeliverMotion();
                return;
            }

            const bool INSIDE = next.x >= 0 && next.y >= 0 &&
                next.x <= E->logicalWidth && next.y <= E->logicalHeight;

            Vector2D projected;
            const bool ON_SCREEN =
                projectToScreen(MON, windowPoint(*E, next), projected);

            if (INSIDE) {
                // The part of the window under a bar is the bar's.
                if (ON_SCREEN && g_cfgUnderLayers &&
                    Compat::interactiveLayerAt(MON, MON->m_position + projected)) {
                    g_vcScreen = projected;
                    vcLeaveToDesktop(MON->m_position + projected);
                    return;
                }

                g_vcLocal = next;
                if (ON_SCREEN)
                    g_vcScreen = projected;
                vcDeliverMotion();
                return;
            }

            // Off the window's edge: carry on in screen space from where the
            // exit point appears on screen.
            if (ON_SCREEN)
                g_vcScreen = projected;
        }

        Compat::clearPointerFocus();
        g_vcSpace = ECursorSpace::Screen;
        g_vcId    = 0;
        vcResolveScreen(MON);
        return;
    }

    if (g_vcSpace != ECursorSpace::Screen)
        return;

    g_vcScreen = g_vcScreen + Vector2D{dx, dy};
    vcResolveScreen(MON);
}

// Typing mode starts at the crosshair: whatever the view was aiming at is
// already under the cursor.
static void vcBegin() {
    const auto MON = targetMonitor();

    g_vcSpace = ECursorSpace::Screen;
    g_vcId    = 0;

    if (!MON)
        return;

    g_vcScreen = MON->m_size * 0.5;
    vcResolveScreen(MON);
}

// The real cursor came back over the room: capture it there again.
static void vcEnterFromDesktop(const Vector2D& global) {
    const auto MON = targetMonitor();
    if (!MON)
        return;

    g_vcScreen = global - MON->m_position;
    g_vcSpace  = ECursorSpace::Screen;

    // Pointer focus still sits on the desktop window the cursor came from.
    Compat::clearPointerFocus();
    Compat::setPointerCapture(true);

    if (Compat::setCursorHidden(true))
        Pointer::mgr()->resetCursorImage();

    vcResolveScreen(MON);
}

// The client's cursor image, recorded by the pointer hooks while the real
// cursor is hidden. A theme or cursor-shape cursor arrives as a buffer and is
// uploaded once per request here, in render.pre (outside the frame's pass); a
// client-drawn cursor surface already carries its texture.
static uint64_t             g_cursorSerial = 0;
static SP<Render::ITexture> g_cursorBufferTex;

// Kept alive until the next frame: the scene samples it inside the pass.
static SP<Render::ITexture> g_cursorTexHold;

static void refreshCursorImage() {
    const auto& REQ = Compat::lastCursorRequest();

    if (REQ.serial == g_cursorSerial)
        return;

    g_cursorSerial = REQ.serial;
    g_cursorBufferTex.reset();

    if (REQ.buffer && g_pHyprRenderer) {
        if (Render::GL::g_pHyprOpenGL)
            Render::GL::g_pHyprOpenGL->makeEGLCurrent();

        g_cursorBufferTex = g_pHyprRenderer->createTexture(REQ.buffer);
    }
}

// The current client cursor image: texture, logical size and hotspot. Only
// plain RGBA textures; an external (EGLImage) one needs a different sampler,
// and the built-in arrow stands in for it.
static bool clientCursorImage(SP<Render::ITexture>& tex, Vector2D& size,
                              Vector2D& hotspot) {
    const auto& REQ = Compat::lastCursorRequest();

    if (const auto SURF = REQ.surface.lock()) {
        const auto RES = SURF->resource();
        if (!RES)
            return false;

        tex  = RES->m_current.texture;
        size = RES->m_current.size;
    } else if (REQ.buffer && g_cursorBufferTex) {
        tex  = g_cursorBufferTex;
        size = REQ.buffer->size / REQ.scale;
    } else
        return false;

    hotspot = REQ.hotspot;

    return tex && tex->m_texID != 0 && size.x > 0 && size.y > 0 &&
        (tex->m_type == Render::TEXTURE_RGBA ||
         tex->m_type == Render::TEXTURE_RGBX);
}

// Where the scene draws the virtual cursor this frame.
static void updatePointerVisual() {
    GLScene::SPointer P;

    const auto MON = targetMonitor();

    if (virtualCursor() && MON && g_vcSpace != ECursorSpace::Desktop) {
        const auto* E = g_vcSpace == ECursorSpace::Window ?
            g_world.find(g_vcId) : nullptr;

        if (E && E->logicalWidth > 0) {
            P.mode    = GLScene::SPointer::EMode::World;
            P.tip     = windowPoint(*E, g_vcLocal);
            P.right   = g_world.rightOf(E->id);
            P.down    = g_world.upOf(E->id) * -1.0f;
            P.pxWorld = E->width / E->logicalWidth;

            // On a window it shows what that client asked for (text beam,
            // resize arrows, hand...), in the window's plane.
            SP<Render::ITexture> TEX;
            Vector2D SIZE, HOT;
            if (clientCursorImage(TEX, SIZE, HOT)) {
                g_cursorTexHold = TEX;
                P.texture = TEX->m_texID;
                P.texW    = static_cast<float>(SIZE.x);
                P.texH    = static_cast<float>(SIZE.y);
                P.hotX    = static_cast<float>(HOT.x);
                P.hotY    = static_cast<float>(HOT.y);
            }
        } else {
            // The window went away under the cursor: the next move
            // re-resolves from the last screen point.
            if (g_vcSpace == ECursorSpace::Window) {
                g_vcSpace = ECursorSpace::Screen;
                g_vcId    = 0;
            }

            P.mode = GLScene::SPointer::EMode::Screen;
            P.ndcX = static_cast<float>(g_vcScreen.x / MON->m_size.x) * 2.0f - 1.0f;
            P.ndcY = 1.0f - static_cast<float>(g_vcScreen.y / MON->m_size.y) * 2.0f;
        }
    }

    g_scene.setPointer(P);
}

static void onRenderStage(eRenderStage stage) {
    if (g_capturing)
        return;

    if (!g_active)
        return;

    if (!g_monitor)
        return;

    // world.under_layers: after the windows, before the top and overlay
    // layers -- bars, launchers and notifications draw over the room as
    // ordinary 2D. Otherwise the room covers everything.
    if (stage != (g_cfgUnderLayers ? RENDER_POST_WINDOWS : RENDER_LAST_MOMENT))
        return;

    if (!g_currentRenderMon || g_currentRenderMon != g_monitor)
        return;

    // The stage fires per rendered workspace (two during a workspace
    // switch); the room is drawn once per frame.
    if (g_roomDrawnThisFrame)
        return;
    g_roomDrawnThisFrame = true;

    dumpStatus();

    const float dt = updateTransition();

    if (g_transition <= 0.0f && g_transitionTarget <= 0.0f) {
        requestDeactivate3D();
        return;
    }

    if (!g_pHyprRenderer)
        return;

    if (g_pHyprRenderer->type() != Render::IHyprRenderer::RT_GL)
        return;

    // 2D passthrough: Hyprland renders the fullscreen window; the room and
    // its input are dormant. The frame pump polls for the fullscreen exit.
    if (g_fsPhase == EFullscreenPhase::In2D)
        return;

    const auto U3_T0 = std::chrono::steady_clock::now();
    update3D(dt);
    g_msUpdate3D = g_msUpdate3D * 0.9 +
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - U3_T0).count() * 0.1;

    g_diagAlpha = std::clamp(g_transition, 0.0f, 1.0f);

    if (g_fsPhase == EFullscreenPhase::To2D || g_fsPhase == EFullscreenPhase::To3D)
        g_diagAlpha = g_fsAlpha;

    updatePointerVisual();

    g_pHyprRenderer->addPassElement(
        makeUnique<CHypr3DPassElement>(
            g_diagAlpha,
            dt
        )
    );

    damageCurrentMonitor();
}

// --- event handlers ---------------------------------------------------------

// The pointer hook gives us true relative motion and consumes it, so the OS
// cursor remains parked and cannot hit a screen edge. If the hook is missing,
// the absolute event path derives a delta and keeps the camera usable.
static bool hookSink(double dx, double dy) {
    ++g_diagSinkCalls;
    return onPointerMotion(dx, dy);
}

static void setKeyboardMode(EKeyboardMode mode) {
    if (g_keyboardMode == mode)
        return;

    g_keyboardMode = mode;

    // Both directions start from the crosshair: the cursor appears where the
    // view was aiming, and returning puts it back on the room monitor (it may
    // have wandered to another one).
    Pointer::mgr()->warpTo(crosshairLogical());

    if (pointerFree()) {
        resetCameraKeys(); // held camera keys must not keep walking
        resetPointerGesture();
        finishClientButton(inputTimeMs());
        g_input.reset(); // drop pending look: the view stops dead

        Compat::clearPointerFocus();

        if (virtualCursor()) {
            // The pointer stays captured and drives the plugin's cursor.
            vcBegin();
        } else {
            Compat::setPointerCapture(false);
            Compat::setCursorHidden(false);

            if (g_pHyprRenderer)
                g_pHyprRenderer->setCursorFromName("default", true);

            g_freeOnRoom = true;
        }
    } else {
        finishClientButton(inputTimeMs());
        Compat::clearPointerFocus();
        g_vcSpace = ECursorSpace::Desktop;
        g_vcId    = 0;

        Compat::setPointerCapture(g_hookInstalled && ownsInput());

        if (Compat::setCursorHidden(true))
            Pointer::mgr()->resetCursorImage();

        // Hand the keyboard back to the aimed window: another monitor may
        // have taken focus while the cursor was over there.
        g_lastFocusId = 0;
    }

    g_scene.setCrosshairVisible(!pointerFree());

    notify(
        g_keyboardMode == EKeyboardMode::Space ?
            "[hypr3d] keyboard: space (wasd / space / shift / ctrl)" :
        !pointerFree() ?
            "[hypr3d] keyboard: window (typing reaches the focused window)" :
        virtualCursor() ?
            "[hypr3d] typing: cursor on the windows (toggle again to return)" :
            "[hypr3d] typing: free cursor (toggle again / click the room to return)",
        CHyprColor{0.2f, 0.8f, 0.4f, 1.0f}
    );

    damageCurrentMonitor();
}

static void onMouseMove(Vector2D pos, Event::SCallbackInfo& info) {
    ++g_diagMoveEvents;
    g_diagLastPos = pos;

    if (!ownsInput())
        return;

    if (virtualCursor()) {
        // In the room the hook owns the motion. Out on the desktop the
        // ordinary cursor runs until it comes back over the room.
        if (g_vcSpace == ECursorSpace::Desktop) {
            // Over a layer on the room monitor the real cursor stays: the
            // bar and its popups get ordinary input.
            if (onRoomMonitor(pos) &&
                !(g_cfgUnderLayers &&
                  Compat::interactiveLayerAt(targetMonitor(), pos))) {
                vcEnterFromDesktop(pos);
                info.cancelled = true;
            }
            return;
        }

        info.cancelled = true;
        return;
    }

    if (pointerFree()) {
        // Over the room, the 2D windows under the cursor are the ghosted
        // originals the scene hides: keep Hyprland's hover and
        // focus-follows-mouse off them, so typing stays on the aimed window.
        // Elsewhere it is the ordinary desktop.
        const bool ON_ROOM = onRoomMonitor(pos);

        if (ON_ROOM && !g_freeOnRoom) {
            Compat::clearPointerFocus();

            if (g_pHyprRenderer)
                g_pHyprRenderer->setCursorFromName("default", true);
        }

        g_freeOnRoom = ON_ROOM;

        if (ON_ROOM)
            info.cancelled = true;

        return;
    }

    if (g_hookInstalled && g_diagSinkCalls > 0) {
        // The relative hook already owns the real delta. The absolute event is
        // intentionally ignored for rotation so it cannot double-count it.
        info.cancelled = true;
        return;
    }

    if (!g_diagPinnedValid) {
        g_diagPinned = pos;
        g_diagPinnedValid = true;
        info.cancelled = true;
        return;
    }

    const double dx = pos.x - g_diagPinned.x;
    const double dy = pos.y - g_diagPinned.y;
    g_diagPinned = pos;

    onPointerMotion(dx, dy);
    info.cancelled = true;
}

static void onMouseAxis(
    IPointer::SAxisEvent event,
    Event::SCallbackInfo& info
) {
    if (!ownsInput())
        return;

    // Virtual cursor: scroll the window under it; empty space swallows it.
    if (virtualCursor()) {
        if (g_vcSpace == ECursorSpace::Desktop)
            return;

        info.cancelled = true;

        if (g_vcSpace == ECursorSpace::Window) {
            vcDeliverMotion();
            Compat::deliverAxis(event.timeMs, event.axis, event.delta,
                                event.deltaDiscrete, event.source,
                                event.relativeDirection);
        }
        return;
    }

    // Free cursor: scrolling belongs to the desktop, never to the hidden
    // 2D windows under the room.
    if (pointerFree()) {
        if (onRoomMonitor(Pointer::mgr()->position()))
            info.cancelled = true;
        return;
    }

    if (event.axis != WL_POINTER_AXIS_VERTICAL_SCROLL)
        return;

    // Physical wheels report whole detents in deltaDiscrete; touchpads send
    // a smooth delta instead. Hi-res wheels report many detents per physical
    // click -- clamped, or a light scroll flings the window across the room.
    //
    // Sign, per the Wayland axis spec: a vertical delta is negative when the
    // top of the wheel rolls AWAY from the user ("forward") with
    // IDENTICAL direction, and positive when the client is told INVERTED
    // (natural scroll). Normalize to +1 step = wheel forward in both cases,
    // so the zoom below can be written against the physical wheel motion
    // instead of guessing the compositor's sign.
    const double RAW = event.deltaDiscrete != 0 ?
        static_cast<double>(event.deltaDiscrete) :
        event.delta * 0.05;

    const bool INVERTED = event.relativeDirection ==
        WL_POINTER_AXIS_RELATIVE_DIRECTION_INVERTED;

    double STEPS = std::clamp(INVERTED ? RAW : -RAW, -2.0, 2.0);

    if (STEPS == 0.0)
        return;

    g_diagLastZoomStep = STEPS;

    // C-held view zoom: the wheel adjusts the magnification live
    // (min 1x, no max). Wheel forward = zoom in.
    if (g_zoomHeld) {
        g_zoomWheel = std::max(
            1.0f, g_zoomWheel * static_cast<float>(std::pow(1.06, STEPS)));
        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    // During an LMB drag the wheel zooms the dragged window.
    if (g_pointerGesture == EPointerGesture::Move3D && g_pointerDown) {
        g_world.dragZoom(STEPS);
        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    // Carrying a scene object: the wheel scales the grab distance, so the
    // object approaches or recedes along the crosshair. (modelRayHit skips
    // the carried object -- it is under the crosshair already -- so without
    // this branch the wheel would do nothing during a carry.)
    if (g_pointerGesture == EPointerGesture::MapDrag && g_pointerDown &&
        g_mapGrabIndex != SIZE_MAX) {
        g_mapGrabDist = std::max(
            0.05f, g_mapGrabDist * static_cast<float>(std::pow(1.06, STEPS)));
        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    // Any other time: Super + wheel over a window zooms it along the ray
    // from the camera through the window. Without Super the wheel reaches
    // the focused client unchanged (scroll).
    if (!g_superHeld)
        return;

    // A DYNAMIC scene object closer than any window zooms instead: its
    // center slides along the camera-object line, same multiplicative step,
    // same no-limits policy as the window zoom.
    {
        const auto& CAMZ = g_scene.camera();
        const auto MRAY = modelRayHit(CAMZ.position, CAMZ.centerRay(), true);
        const World3D::SHit WHIT = aimHit();

        const bool MODEL_CLOSER =
            MRAY.hit && (!WHIT.hit || MRAY.dist < WHIT.distance);

        if (!MODEL_CLOSER) {
            if (s_modelZoomId != SIZE_MAX)
                s_modelZoomId = SIZE_MAX; // aimed away: drop the zoom
        } else {
            const auto* MODEL = g_scene.sceneModel(MRAY.index);

            if (!MODEL || !MODEL->loaded()) {
                s_modelZoomId = SIZE_MAX;
                return;
            }

            if (s_modelZoomId != MRAY.index)
                s_modelZoomDist = std::sqrt(
                    (MODEL->position() - CAMZ.position).x *
                        (MODEL->position() - CAMZ.position).x +
                    (MODEL->position() - CAMZ.position).y *
                        (MODEL->position() - CAMZ.position).y +
                    (MODEL->position() - CAMZ.position).z *
                        (MODEL->position() - CAMZ.position).z);
            s_modelZoomId = MRAY.index;

            const float NEW_DIST = s_modelZoomDist * std::pow(1.06, STEPS);
            const Vec3 DIR = normalize(MODEL->position() - CAMZ.position);

            g_sceneObjects[MRAY.index].position =
                CAMZ.position + DIR * NEW_DIST;
            g_scene.setSceneObjectTransform(MRAY.index,
                                            g_sceneObjects[MRAY.index].position,
                                            MODEL->rotationDeg());

            if (g_bodyIf && MRAY.index < g_joltBodies.size() &&
                g_joltBodies[MRAY.index].valid)
                g_bodyIf->SetPositionAndRotationWhenChanged(
                    g_joltBodies[MRAY.index].body,
                    JPH::RVec3(g_sceneObjects[MRAY.index].position.x,
                               g_sceneObjects[MRAY.index].position.y,
                               g_sceneObjects[MRAY.index].position.z),
                    eulerToQuat(g_sceneObjects[MRAY.index].rotationDeg),
                    JPH::EActivation::Activate);

            s_modelZoomDist = NEW_DIST;

            info.cancelled = true;
            damageCurrentMonitor();
            return;
        }
    }

    const auto HIT = aimHit();

    if (!HIT.hit)
        return;

    const auto* ENTITY = g_world.find(HIT.id);

    if (!ENTITY)
        return;

    const auto& CAM = g_scene.camera();
    const Vec3 OFFSET = ENTITY->center - CAM.position;
    const float DIST = std::sqrt(
        OFFSET.x * OFFSET.x + OFFSET.y * OFFSET.y + OFFSET.z * OFFSET.z);

    if (!std::isfinite(DIST) || DIST < 0.05f)
        return;

    const bool NEW = s_zoomId != HIT.id;
    s_zoomId     = HIT.id;
    s_zoomDir    = OFFSET * (1.0f / DIST);
    s_zoomAnchor = CAM.position;

    if (NEW) {
        s_zoomCur    = DIST;
        s_zoomTarget = DIST;
    }

    // Wheel forward (+) pushes the window away, wheel back pulls it closer;
    // the drag zoom receives the same normalized sign, so both wheel paths
    // agree. No distance limits -- the target stays positive, so scrolling
    // just keeps multiplying; the window can come arbitrarily close or far.
    s_zoomTarget *= std::pow(1.06, STEPS);

    info.cancelled = true;
    damageCurrentMonitor();
}

static void onMouseButton(
    IPointer::SButtonEvent event,
    Event::SCallbackInfo& info
) {
    if (!ownsInput())
        return;

    const bool PRESSED =
        event.state == WL_POINTER_BUTTON_STATE_PRESSED;

    if (!PRESSED && g_swallowRelease == event.button) {
        g_swallowRelease = 0;
        info.cancelled = true;
        return;
    }

    // input.typing_button switches between moving and typing.
    if (g_cfgTypingButton && event.button == g_cfgTypingButton) {
        if (PRESSED) {
            setKeyboardMode(g_keyboardMode == EKeyboardMode::Window ?
                                EKeyboardMode::Space :
                                EKeyboardMode::Window);
            g_swallowRelease = event.button;
        }

        info.cancelled = true;
        return;
    }

    // Virtual cursor: buttons go to the window under it, natively. Empty
    // space swallows them; out on the desktop they are ordinary clicks.
    if (virtualCursor()) {
        if (g_vcSpace == ECursorSpace::Desktop)
            return;

        info.cancelled = true;

        if (!PRESSED) {
            if (g_clientButtonDown && event.button == g_clientButton)
                finishClientButton(event.timeMs);
            return;
        }

        if (g_vcSpace != ECursorSpace::Window || g_clientButtonDown)
            return;

        const auto* E = g_world.find(g_vcId);
        const auto TARGET = targetFromHit(g_vcId);

        if (!E || (!TARGET.window && !TARGET.layer))
            return;

        const auto LOCAL = surfaceLocal(*E, g_vcLocal);

        // A click focuses, as on the desktop -- also taking the keyboard
        // back from a launcher or sidebar.
        if (TARGET.window) {
            Compat::focusWindow(TARGET.window);
            g_lastFocusId = g_vcId;
        }

        if (TARGET.layer)
            Compat::deliverClick(TARGET.layer, LOCAL, event.button, true, event.timeMs);
        else
            Compat::deliverClick(TARGET.window, LOCAL, event.button, true, event.timeMs);

        g_clientButtonWindow = TARGET.window;
        g_clientButtonLayer  = TARGET.layer;
        g_clientButton       = event.button;
        g_clientButtonLocal  = LOCAL;
        g_clientButtonDown   = true;
        return;
    }

    // Free cursor: clicks on other monitors are ordinary desktop clicks. A
    // press on the room returns to moving and never reaches a client.
    if (pointerFree()) {
        if (PRESSED && onRoomMonitor(Pointer::mgr()->position())) {
            setKeyboardMode(EKeyboardMode::Space);
            g_swallowRelease = event.button;
            info.cancelled = true;
        }

        return;
    }

    // Release the plugin gesture that owns this physical button.
    if (!PRESSED && g_pointerDown && event.button == g_pointerButton) {
        resetPointerGesture();
        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    // Path tools: left-click adds a point on the ground under the
    // crosshair, right-click removes the last one. Never reaches a client.
    if (pathTool() && !g_superHeld &&
        (event.button == BTN_LEFT || event.button == BTN_RIGHT)) {
        if (PRESSED) {
            const bool SMOOTH = g_tool == static_cast<int>(ETool::Curve);
            Vec3 at;
            const bool ON_GROUND = groundUnderCrosshair(at);
            const auto NEAR = ON_GROUND ? pathPointNear(at)
                                        : std::pair<int, int>{-1, -1};

            if (event.button == BTN_LEFT && NEAR.first >= 0) {
                // An existing point: take the tool's type, and its path
                // becomes the one being extended.
                g_paths[NEAR.first].smooth[NEAR.second] = SMOOTH;
                g_activePath = NEAR.first;
            } else if (event.button == BTN_LEFT && ON_GROUND) {
                if (g_activePath < 0 ||
                    g_activePath >= static_cast<int>(g_paths.size())) {
                    g_paths.push_back({});
                    g_activePath = static_cast<int>(g_paths.size()) - 1;
                }
                g_paths[g_activePath].points.push_back(at);
                g_paths[g_activePath].smooth.push_back(SMOOTH);
            } else if (event.button == BTN_RIGHT && NEAR.first >= 0)
                removePathPoint(NEAR.first, NEAR.second);
            else if (event.button == BTN_RIGHT && g_activePath >= 0 &&
                     g_activePath < static_cast<int>(g_paths.size()))
                removePathPoint(g_activePath,
                                static_cast<int>(g_paths[g_activePath].points.size()) - 1);

            g_swallowRelease = event.button;
            damageCurrentMonitor();
        }
        info.cancelled = true;
        return;
    }

    // Super + wheel-press: grab the aimed window's roll around its normal.
    // Sweeping the crosshair around the window center then rotates the
    // window by the swept angle; release ends it. Never reaches the client.
    if (PRESSED && g_superHeld && g_fsPhase == EFullscreenPhase::None &&
        event.button == BTN_MIDDLE) {
        const World3D::SHit HIT = aimHit();
        const auto* ENTITY = HIT.hit ? g_world.find(HIT.id) : nullptr;

        // A dynamic scene object closer than any window: roll IT. The roll
        // axis is its facing normal; the sweep angle is measured on the
        // plane through its center, exactly like the window roll.
        {
            const auto& CAM = g_scene.camera();
            const auto MRAY = modelRayHit(CAM.position, CAM.centerRay(),
                                          /*dynamicOnly=*/true);

            if (MRAY.hit && (!HIT.hit || MRAY.dist < HIT.distance)) {
                const auto* MODEL = g_scene.sceneModel(MRAY.index);
                const Vec3 CENTER = MODEL->position();
                const Vec3 NORMAL = normalize(CAM.position - CENTER);

                Vec3 PLANE_POINT;
                if (rayPlanePoint(CAM.position, CAM.centerRay(), CENTER,
                                  NORMAL, PLANE_POINT) &&
                    dot(PLANE_POINT - CENTER, PLANE_POINT - CENTER) > 0.0004f) {
                    s_wheelRot.active     = true;
                    s_wheelRot.model      = true;
                    s_wheelRot.modelIndex = MRAY.index;
                    s_wheelRot.id         = 0;
                    s_wheelRot.center     = CENTER;
                    s_wheelRot.normal     = NORMAL;
                    s_wheelRot.reference  = PLANE_POINT - CENTER;
                    s_wheelRot.startRoll  = MODEL->rotationDeg().z *
                        (3.14159265358979f / 180.0f);

                    // Capture the gravity factor ONCE: the per-frame update
                    // sets it to 0 for the suspension, and re-reading it
                    // there would store the zero and never restore it.
                    if (MRAY.index < g_joltBodies.size() &&
                        g_joltBodies[MRAY.index].valid)
                        g_rolledGravity = g_bodyIf->GetGravityFactor(
                            g_joltBodies[MRAY.index].body);

                    g_pointerGesture = EPointerGesture::WheelRoll;
                    g_pointerButton  = BTN_MIDDLE;
                    g_pointerDown    = true;
                    g_resize         = {};

                    info.cancelled = true;
                    damageCurrentMonitor();
                    return;
                }

                info.cancelled = true;
                return;
            }
        }

        if (ENTITY) {
            const Vec3 REFERENCE = HIT.point - ENTITY->center;

            // The sweep angle is undefined when the crosshair sits exactly
            // on the center -- refuse the grab there.
            if (dot(REFERENCE, REFERENCE) > 0.0004f) {
                s_wheelRot.active     = true;
                s_wheelRot.id         = HIT.id;
                s_wheelRot.center     = ENTITY->center;
                s_wheelRot.normal     = g_world.normalOf(HIT.id);
                s_wheelRot.reference  = REFERENCE;
                s_wheelRot.startRoll  = ENTITY->roll;

                g_pointerGesture = EPointerGesture::WheelRoll;
                g_pointerButton  = BTN_MIDDLE;
                g_pointerDown    = true;
                g_resize         = {};

                info.cancelled = true;
                damageCurrentMonitor();
                return;
            }
        }

        info.cancelled = true;
        return;
    }

    // Super+LMB / Super+RMB are exclusively plugin gestures. Nothing is sent
    // to the client for these physical button events. Layer surfaces can be
    // dragged like windows, but not resized -- their real geometry is owned
    // by the shell that anchored them.
    if (PRESSED && g_superHeld && g_fsPhase == EFullscreenPhase::None &&
        (event.button == BTN_LEFT || event.button == BTN_RIGHT)) {
        const World3D::SHit HIT = aimHit();
        const auto TARGET = HIT.hit ? targetFromHit(HIT.id) : SHitTarget{};

        // A DYNAMIC scene object closer than any window is grabbed with the
        // left button (one class of grabbable objects: windows and models).
        // The grabbed object leaves the collision set for the duration (it
        // follows the crosshair and must not push the player), and returns
        // to it on release.
        if (event.button == BTN_LEFT) {
            const auto& CAM = g_scene.camera();
            const Vec3 DIR = CAM.centerRay();

            float bestT = -1.f;
            size_t bestIdx = SIZE_MAX;

            for (size_t i = 0; i < g_sceneObjects.size(); ++i) {
                if (!g_sceneObjects[i].dynamic)
                    continue;

                const auto* MODEL = g_scene.sceneModel(i);
                if (!MODEL || !MODEL->loaded())
                    continue;

                const float T = MODEL->rayCast(CAM.position, DIR);
                if (T > 0.f && (bestT < 0.f || T < bestT)) {
                    bestT   = T;
                    bestIdx = i;
                }
            }

            // The model must also be CLOSER than the aimed window -- one
            // distance space for both classes.
            if (bestIdx != SIZE_MAX && bestT < 0.f)
                bestIdx = SIZE_MAX;
            if (bestIdx != SIZE_MAX && HIT.hit && bestT >= HIT.distance)
                bestIdx = SIZE_MAX;

            // A character closer than the aimed window and any model is
            // picked up instead (upright, feet on the ground).
            float charT = 0.f;
            const size_t CHAR = pickCharacter(CAM.position, DIR, charT);
            if (CHAR != SIZE_MAX && (!HIT.hit || charT < HIT.distance) &&
                (bestIdx == SIZE_MAX || charT < bestT)) {
                const auto& FEET = g_chars[CHAR].feet;
                const Vec3 TO = FEET - CAM.position;
                g_charGrabIndex = CHAR;
                g_charGrabDist  = std::sqrt(TO.x * TO.x + TO.y * TO.y + TO.z * TO.z);

                g_pointerGesture = EPointerGesture::CharDrag;
                g_pointerButton  = BTN_LEFT;
                g_pointerDown    = true;
                g_resize         = {};

                info.cancelled = true;
                damageCurrentMonitor();
                return;
            }

            if (bestIdx != SIZE_MAX) {
                const auto* MODEL = g_scene.sceneModel(bestIdx);

                // Center grab: the object's center rides the crosshair, so
                // the pickup distance is the camera-to-CENTER distance and
                // there is no point offset.
                g_mapGrabIndex = bestIdx;
                g_mapGrabDist  = std::sqrt(
                    (MODEL->position() - CAM.position).x *
                        (MODEL->position() - CAM.position).x +
                    (MODEL->position() - CAM.position).y *
                        (MODEL->position() - CAM.position).y +
                    (MODEL->position() - CAM.position).z *
                        (MODEL->position() - CAM.position).z);

                s_modelZoomId = SIZE_MAX; // the drag owns the object now

                g_pointerGesture = EPointerGesture::MapDrag;
                g_pointerButton  = BTN_LEFT;
                g_pointerDown    = true;
                g_resize         = {};

                info.cancelled = true;
                damageCurrentMonitor();
                return;
            }
        }

        if (!TARGET.window && !TARGET.layer) {
            info.cancelled = true;
            return;
        }

        const auto& CAM = g_scene.camera();

        if (TARGET.window)
            Compat::focusWindow(TARGET.window);

        // Resize is a window-only control: a scene model in front of the
        // crosshair must not let the gesture reach a window behind it.
        if (event.button == BTN_RIGHT) {
            if (!TARGET.window || modelInFront(CAM.position,
                                               CAM.centerRay(), HIT)) {
                info.cancelled = true;
                return;
            }
        }

        if (event.button == BTN_LEFT) {
            if (!g_world.startDrag(
                    HIT.id, HIT, CAM.position, CAM.forward())) {
                info.cancelled = true;
                return;
            }
            s_zoomId = 0; // the drag owns this window's distance now

            // A drag on the assert window: the drag wins, stop fighting.
            if (g_fsAssertFrames > 0 && HIT.id == Compat::windowId(TARGET.window)) {
                g_fsAssertFrames = 0;
                g_fsWindow = {};
            }

            g_pointerGesture = EPointerGesture::Move3D;
            g_pointerButton = BTN_LEFT;
            g_pointerDown = true;
            g_resize = {};
        }
        else {
            const auto ENTITY = g_world.find(HIT.id);
            if (!ENTITY) {
                info.cancelled = true;
                return;
            }

            g_resize = {};
            g_resize.active = true;
            g_resize.id = HIT.id;
            g_resize.window = TARGET.window;
            g_resize.startBox = Compat::currentWindowBox(TARGET.window);
            g_resize.startCenter = ENTITY->center;
            g_resize.startWorldWidth = ENTITY->width;
            g_resize.startWorldHeight = ENTITY->height;
            g_resize.planePoint = ENTITY->center;
            g_resize.planeNormal = g_world.normalOf(HIT.id);

            // The grab point only seeds which edge follows the crosshair;
            // updateRealResize re-evaluates that every frame from the aim's
            // side relative to the window centre, so the grab lands anywhere
            // on the window and the pull direction decides the rest. As the
            // camera turns, the current centre ray is intersected with this
            // same window plane, so the real window stretches exactly toward
            // the point being aimed at.
            // RayHit::v is already top-to-bottom. Top half follows +1,
            // bottom half follows -1 in the CBox edge convention below.
            g_resize.edgeX = HIT.u < 0.5f ? -1 : 1;
            g_resize.edgeY = HIT.v < 0.5f ? 1 : -1;
            g_resize.grabPx = worldPointToGlobalPx(targetMonitor(), HIT.point);

            g_pointerGesture = EPointerGesture::ResizeReal;
            g_pointerButton = BTN_RIGHT;
            g_pointerDown = true;
        }

        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    // Regular buttons are virtual-pointer buttons at the crosshair. Track the
    // pressed surface so release goes back to the same client even if the
    // camera turns away before release. A scene model closer than the aimed
    // window occludes it: the click hits geometry, not the client.
    const auto& CAMC = g_scene.camera();
    const World3D::SHit HIT = aimHit();

    if (modelInFront(CAMC.position, CAMC.centerRay(), HIT)) {
        info.cancelled = true;
        return;
    }

    const auto TARGET = HIT.hit ? targetFromHit(HIT.id) : SHitTarget{};

    if (!TARGET.window && !TARGET.layer) {
        info.cancelled = true;
        return;
    }

    const Vector2D LOCAL = localFromHit(HIT);

    if (PRESSED) {
        if (TARGET.window)
            Compat::focusWindow(TARGET.window);

        if (TARGET.layer)
            Compat::deliverClick(TARGET.layer, LOCAL, event.button, true, event.timeMs);
        else
            Compat::deliverClick(TARGET.window, LOCAL, event.button, true, event.timeMs);

        g_clientButtonWindow = TARGET.window;
        g_clientButtonLayer  = TARGET.layer;
        g_clientButton = event.button;
        g_clientButtonLocal = LOCAL;
        g_clientButtonDown = true;
    }
    else {
        if (g_clientButtonDown && (g_clientButtonWindow || g_clientButtonLayer)) {
            if (g_clientButtonLayer)
                Compat::deliverClick(
                    g_clientButtonLayer,
                    g_clientButtonLocal,
                    g_clientButton,
                    false,
                    event.timeMs
                );
            else
                Compat::deliverClick(
                    g_clientButtonWindow,
                    g_clientButtonLocal,
                    g_clientButton,
                    false,
                    event.timeMs
                );
            g_clientButtonWindow = nullptr;
            g_clientButtonLayer  = nullptr;
            g_clientButton = 0;
            g_clientButtonDown = false;
        }
    }

    info.cancelled = true;
    damageCurrentMonitor();
}

// Space mode: only the camera movement keys are swallowed -- everything else
// still reaches the window the crosshair is aiming at. Minecraft creative-
// flight scheme: Space ascends, Shift descends, Ctrl sprints.
static bool isMovementSym(xkb_keysym_t sym) {
    switch (sym) {
        case XKB_KEY_w:
        case XKB_KEY_a:
        case XKB_KEY_s:
        case XKB_KEY_d:
        case XKB_KEY_space:
        case XKB_KEY_Shift_L:
        case XKB_KEY_Shift_R:
        case XKB_KEY_Control_L:
        case XKB_KEY_Control_R:
        case XKB_KEY_Super_L:
        case XKB_KEY_Super_R: return true;
        default: return false;
    }
}

static void setMovementSym(xkb_keysym_t sym, bool down) {
    switch (sym) {
        case XKB_KEY_w: g_keyFwd = down; break;
        case XKB_KEY_s: g_keyBack = down; break;
        case XKB_KEY_a: g_keyLeft = down; break;
        case XKB_KEY_d: g_keyRight = down; break;
        case XKB_KEY_space: g_keyUp = down; break;
        case XKB_KEY_Shift_L:
        case XKB_KEY_Shift_R: g_keyDown = down; break;
        case XKB_KEY_Control_L:
        case XKB_KEY_Control_R: g_keySprint = down; break;
        default: break;
    }
}

static void onKeyboardKey(
    IKeyboard::SKeyEvent event,
    Event::SCallbackInfo& info
) {
    if (!ownsInput())
        return;

    if (!g_pSeatManager || g_pSeatManager->m_keyboard.expired())
        return;

    const auto KEYBOARD = g_pSeatManager->m_keyboard.lock();

    if (!KEYBOARD || !KEYBOARD->m_xkbSymState)
        return;

    // libinput reports evdev codes; xkb wants them offset by 8.
    const auto SYM = xkb_state_key_get_one_sym(
        KEYBOARD->m_xkbSymState,
        event.keycode + 8
    );

    const bool PRESSED = event.state == WL_KEYBOARD_KEY_STATE_PRESSED;

    // Modifier state is tracked in both modes: Super drives the mouse
    // gestures, and Super + Left Alt toggles the keyboard mode.
    if (SYM == XKB_KEY_Super_L || SYM == XKB_KEY_Super_R)
        g_superHeld = PRESSED;
    else if (SYM == XKB_KEY_Alt_L)
        g_altHeld = PRESSED;

    // A layer holds the keyboard (launcher, sidebar text field): every key
    // is its, in both modes. Held camera keys must not keep walking.
    if (Compat::layerHasKeyboardFocus()) {
        if (PRESSED)
            resetCameraKeys();
        return;
    }

    // Walking mode: Space jumps off whatever the capsule stands on. The
    // held state still reaches setMovementSym, but the walking movement
    // path zeroes the vertical input, so holding Space does not fly.
    // SPACE KEYBOARD MODE ONLY: in Window mode the key belongs to the
    // focused window -- a jump here would fire on every typed space.
    if (PRESSED && SYM == XKB_KEY_space &&
        g_keyboardMode == EKeyboardMode::Space &&
        !playerFlies() && g_grounded)
        g_playerJumpQueued = true; // applied to the body in update3D

    // F3 toggles the debug HUD (collision wireframe + info overlay) in both
    // keyboard modes: it never belongs to the focused window.
    if (PRESSED && SYM == XKB_KEY_F3) {
        g_debugHud = !g_debugHud;
        g_scene.setMapDebugCollisions(g_debugHud);
        g_scene.setDebugOverlay(g_debugHud);

        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    if (PRESSED && g_superHeld && g_altHeld &&
        (SYM == XKB_KEY_Alt_L || SYM == XKB_KEY_Super_L)) {
        setKeyboardMode(
            g_keyboardMode == EKeyboardMode::Space ?
                EKeyboardMode::Window :
                EKeyboardMode::Space);

        info.cancelled = true;
        return;
    }

    // Window mode: every key reaches the focused window untouched.
    if (g_keyboardMode == EKeyboardMode::Window)
        return;

    // 1-5 pick a tool (Super + digit stays the compositor's).
    if (!g_superHeld && SYM >= XKB_KEY_1 && SYM <= XKB_KEY_5) {
        if (PRESSED) {
            const int TOOL = static_cast<int>(SYM - XKB_KEY_0);
            // The active path tool's key again: the next point starts a new
            // path.
            if (TOOL == g_tool && pathTool())
                g_activePath = -1;
            g_tool = TOOL;
            damageCurrentMonitor();
        }
        info.cancelled = true;
        return;
    }

    // V toggles noclip: walk through walls and doors.
    if (SYM == XKB_KEY_v) {
        if (PRESSED) {
            g_playerNoclip = !g_playerNoclip;
            notify(g_playerNoclip ? "[hypr3d] noclip on" : "[hypr3d] noclip off",
                   CHyprColor{0.2f, 0.8f, 0.4f, 1.0f});
        }

        info.cancelled = true;
        return;
    }

    // F5: cycle the view -- first person, third person behind, third
    // person in front.
    if (PRESSED && SYM == XKB_KEY_F5) {
        g_viewMode = (g_viewMode + 1) % 3;
        info.cancelled = true;
        damageCurrentMonitor();
        return;
    }

    if (SYM == XKB_KEY_c) {
        // View zoom: hold to magnify, release to ease back to 1x. The
        // wheel level restarts at the base on every press.
        g_zoomHeld = PRESSED;
        if (PRESSED)
            g_zoomWheel = kZoomBase;
        info.cancelled = true;
        return;
    }

    if (!isMovementSym(SYM))
        return;

    setMovementSym(SYM, PRESSED);

    info.cancelled = true;
}

// --- plugin entry -----------------------------------------------------------

static int luaConfig(lua_State* L) {
    // hl.plugin.hypr3d.config({
    //     world = {
    //         panorama = "~/picture.png",   -- 360-degree room background
    //         grid = true,                  -- base grid platform (visible +
    //                                       -- collidable, at world zero)
    //     },
    //     windows = {
    //         window_scale = 0.5,           -- room multiplier on window size
    //         spawn_distance = 5.0,         -- units in front of the camera
    //         depth = 0.05,                 -- window slab thickness, world
    //                                       -- units (0 = flat quads; walls
    //                                       -- follow rounded corners and
    //                                       -- show the texture's edge)
    //     },
    //     player = {
    //         look_sensitivity = 0.0025,    -- radians per pointer count
    //         look_inertia = 0.03,          -- look glide, seconds (0 = off)
    //         move_inertia = 0.05,          -- walk glide, seconds (0 = off)
    //         move_speed = 4.0,             -- world units / second
    //         spawn = { x = 0, y = 0, z = 0 }, -- FEET position
    //         flying = true,                -- false: gravity, Space jumps,
    //                                       -- Shift does nothing
    //         walk_bob = true,              -- camera sway while walking
    //     },
    //     map = {
    //         path = "~/map.glb",
    //         transform = {
    //             position = { x = 0, y = 0, z = 0 },
    //             rotation = { x = 0, y = 0, z = 0 }, -- degrees, XYZ
    //             scale = { x = 1, y = 1, z = 1 },    -- per-axis
    //         },
    //         emissive_scale = 1.0,
    //         flat = true,                  -- baked-map look (no dynamic light)
    //         collision = true,
    //     },
    // })
    //
    // Missing keys keep their current value; wrong-typed keys raise a lua
    // error.
    if (!lua_istable(L, 1))
        return luaL_error(L, "hypr3d.config expects a single table");

    // Field accessors. TIDX = stack index of the section table (0 = absent).
    const auto SET_NUM = [&](int tidx, const char* key, float& out,
                             const char* path) -> bool {
        lua_getfield(L, tidx, key);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return true;
        }
        if (!lua_isnumber(L, -1)) {
            lua_pop(L, 1);
            return false;
        }
        out = static_cast<float>(lua_tonumber(L, -1));
        lua_pop(L, 1);
        return true;
    };

    const auto SET_BOOL = [&](int tidx, const char* key, bool& out,
                              const char* path) -> bool {
        lua_getfield(L, tidx, key);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return true;
        }
        if (!lua_isboolean(L, -1)) {
            lua_pop(L, 1);
            return false;
        }
        out = lua_toboolean(L, -1) != 0;
        lua_pop(L, 1);
        return true;
    };

    const auto SET_STRING = [&](int tidx, const char* key, std::string& out,
                                const char* path) -> bool {
        lua_getfield(L, tidx, key);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return true;
        }
        if (!lua_isstring(L, -1)) {
            lua_pop(L, 1);
            return false;
        }
        size_t LEN = 0;
        const char* STR = lua_tolstring(L, -1, &LEN);
        out.assign(STR, LEN);
        lua_pop(L, 1);
        return true;
    };

    // Missing axes keep their current values.
    const auto SET_VEC3 = [&](int tidx, const char* key, Vec3& out,
                              const char* path) -> bool {
        lua_getfield(L, tidx, key);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return true;
        }
        if (!lua_istable(L, -1)) {
            lua_pop(L, 1);
            return false;
        }

        const int T = lua_gettop(L);

        // Positional form: { x, y, z } == { 1, 2, 3 }.
        float pos[3] = {0.f, 0.f, 0.f};
        bool havePos = false;
        for (int k = 1; k <= 3; ++k) {
            lua_rawgeti(L, T, k);
            if (lua_isnil(L, -1)) {
                lua_pop(L, 1);
                continue;
            }
            if (!lua_isnumber(L, -1)) {
                lua_pop(L, 1);
                return false;
            }
            pos[k - 1] = static_cast<float>(lua_tonumber(L, -1));
            havePos = true;
            lua_pop(L, 1);
        }

        const auto AXIS = [&](const char* name, float& v) {
            lua_getfield(L, T, name);
            if (lua_isnumber(L, -1))
                v = static_cast<float>(lua_tonumber(L, -1));
            lua_pop(L, 1);
        };

        float x = out.x, y = out.y, z = out.z;
        AXIS("x", x);
        AXIS("y", y);
        AXIS("z", z);

        // Positional wins when both forms are mixed in one table.
        if (havePos)
            out = Vec3{pos[0], pos[1], pos[2]};
        else
            out = Vec3{x, y, z};

        lua_pop(L, 1);
        return true;
    };

    // A sub-table section: returns its stack index, or 0 when absent/wrong.
    // A wrong-typed section is an error, a missing one is skipped.
    const auto SECTION = [&](const char* key, const char* path) -> int {
        lua_getfield(L, 1, key);
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return 0;
        }
        if (!lua_istable(L, -1))
            return -1; // caller reports
        return lua_gettop(L);
    };

    int idx = SECTION("world", "world");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: world must be a table");
    if (idx > 0) {
        if (!SET_STRING(idx, "panorama", g_cfgPanorama, "world.panorama"))
            return luaL_error(L, "hypr3d.config: world.panorama must be a string");
        if (!SET_STRING(idx, "monitor", g_cfgMonitor, "world.monitor"))
            return luaL_error(L, "hypr3d.config: world.monitor must be a string");
        if (!SET_BOOL(idx, "under_layers", g_cfgUnderLayers, "world.under_layers"))
            return luaL_error(L, "hypr3d.config: world.under_layers must be a boolean");
        if (!SET_BOOL(idx, "grid", g_cfgGrid, "world.grid"))
            return luaL_error(L, "hypr3d.config: world.grid must be a boolean");
        lua_pop(L, 1);
    }

    idx = SECTION("windows", "windows");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: windows must be a table");
    if (idx > 0) {
        if (!SET_NUM(idx, "window_scale", g_cfgWindowScale,
                     "windows.window_scale"))
            return luaL_error(L, "hypr3d.config: windows.window_scale must be a number");
        if (!SET_NUM(idx, "spawn_distance", g_cfgSpawnDistance,
                     "windows.spawn_distance"))
            return luaL_error(L, "hypr3d.config: windows.spawn_distance must be a number");
        if (!SET_NUM(idx, "spawn_width", g_cfgSpawnWidth,
                     "windows.spawn_width"))
            return luaL_error(L, "hypr3d.config: windows.spawn_width must be a number");
        if (!SET_NUM(idx, "spawn_height", g_cfgSpawnHeight,
                     "windows.spawn_height"))
            return luaL_error(L, "hypr3d.config: windows.spawn_height must be a number");
        g_cfgSpawnWidth  = std::max(g_cfgSpawnWidth, 100.0f);
        g_cfgSpawnHeight = std::max(g_cfgSpawnHeight, 100.0f);
        if (!SET_NUM(idx, "depth", g_cfgWindowDepth,
                     "windows.depth"))
            return luaL_error(L, "hypr3d.config: windows.depth must be a number");

        // Thickness is a distance: 0 (the default) draws the flat quads,
        // anything below is clamped up to it.
        g_cfgWindowDepth = std::max(0.0f, g_cfgWindowDepth);

        lua_pop(L, 1);
    }

    // input = { typing_cursor = bool, typing_button = "back" | code }
    idx = SECTION("input", "input");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: input must be a table");
    if (idx > 0) {
        bool cursor = g_cfgTypingCursor;
        if (!SET_BOOL(idx, "typing_cursor", cursor, "input.typing_cursor"))
            return luaL_error(L, "hypr3d.config: input.typing_cursor must be a boolean");
        // Leaving typing mode with the cursor off would strand a freed pointer.
        if (!cursor && g_cfgTypingCursor && g_keyboardMode == EKeyboardMode::Window)
            setKeyboardMode(EKeyboardMode::Space);
        g_cfgTypingCursor = cursor;

        lua_getfield(L, idx, "typing_button");
        if (lua_type(L, -1) == LUA_TNUMBER)
            g_cfgTypingButton = static_cast<uint32_t>(lua_tointeger(L, -1));
        else if (lua_type(L, -1) == LUA_TSTRING) {
            const std::string NAME = lua_tostring(L, -1);
            static const std::pair<const char*, uint32_t> NAMES[] = {
                {"", 0},
                {"side", BTN_SIDE},       {"extra", BTN_EXTRA},
                {"forward", BTN_FORWARD}, {"back", BTN_BACK},
                {"task", BTN_TASK},
            };
            bool found = false;
            for (const auto& [N, CODE] : NAMES)
                if (NAME == N) {
                    g_cfgTypingButton = CODE;
                    found = true;
                }
            if (!found) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: input.typing_button must be side, extra, forward, back, task or a button code");
            }
        } else if (!lua_isnil(L, -1)) {
            lua_pop(L, 1);
            return luaL_error(L, "hypr3d.config: input.typing_button must be a name or a button code");
        }
        lua_pop(L, 1);
        lua_pop(L, 1);
    }

    idx = SECTION("player", "player");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: player must be a table");
    if (idx > 0) {
        if (!SET_NUM(idx, "look_sensitivity", g_cfgSensitivity,
                     "player.look_sensitivity"))
            return luaL_error(L, "hypr3d.config: player.look_sensitivity must be a number");
        if (!SET_NUM(idx, "look_inertia", g_cfgLookInertia,
                     "player.look_inertia"))
            return luaL_error(L, "hypr3d.config: player.look_inertia must be a number");
        if (!SET_NUM(idx, "move_inertia", g_cfgMoveInertia,
                     "player.move_inertia"))
            return luaL_error(L, "hypr3d.config: player.move_inertia must be a number");
        if (!SET_NUM(idx, "move_speed", g_cfgMoveSpeed,
                     "player.move_speed"))
            return luaL_error(L, "hypr3d.config: player.move_speed must be a number");
        if (!SET_BOOL(idx, "flying", g_playerFlying, "player.flying"))
            return luaL_error(L, "hypr3d.config: player.flying must be a boolean");
        if (!SET_BOOL(idx, "noclip", g_playerNoclip, "player.noclip"))
            return luaL_error(L, "hypr3d.config: player.noclip must be a boolean");
        if (!SET_BOOL(idx, "walk_bob", g_cfgWalkBob, "player.walk_bob"))
            return luaL_error(L, "hypr3d.config: player.walk_bob must be a boolean");
        if (!SET_BOOL(idx, "collision", g_playerCollision, "player.collision"))
            return luaL_error(L, "hypr3d.config: player.collision must be a boolean");

        // The shared mesh-block parser: path + transform + material
        // overrides -- the SAME description for the player and the scene
        // objects. transform.position anchors the mesh, rotation.y corrects
        // the authored facing, scale stretches it.
        const auto PARSE_MESH = [&](int MT, std::string& path, Vec3& pos,
                                    Vec3& rotDeg, Vec3& scale,
                                    float& emissive, bool& flat,
                                    std::string& center,
                                    Vec3& centerOff) -> bool {
            if (!SET_STRING(MT, "path", path, "mesh.path"))
                return false;

            lua_getfield(L, MT, "transform");
            if (lua_isnil(L, -1)) {
                lua_pop(L, 1);
            } else if (!lua_istable(L, -1)) {
                lua_pop(L, 1);
                return false;
            } else {
                const int TIDX = lua_gettop(L);
                if (!SET_VEC3(TIDX, "position", pos, "mesh.transform.position") ||
                    !SET_VEC3(TIDX, "rotation", rotDeg, "mesh.transform.rotation") ||
                    !SET_VEC3(TIDX, "scale", scale, "mesh.transform.scale"))
                    return false;
                lua_pop(L, 1);
            }

            if (!SET_NUM(MT, "emissive_scale", emissive, "mesh.emissive_scale") ||
                !SET_BOOL(MT, "flat", flat, "mesh.flat") ||
                !SET_STRING(MT, "center", center, "mesh.center") ||
                !SET_VEC3(MT, "center_offset", centerOff, "mesh.center_offset"))
                return false;
            return true;
        };

        // player.mesh: the character's visual, described exactly like a
        // scene object's mesh. The transform.position anchors the model
        // relative to the feet, rotation.y corrects the authored facing.
        lua_getfield(L, idx, "mesh");
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
        } else if (!lua_istable(L, -1)) {
            lua_pop(L, 1);
            return luaL_error(L, "hypr3d.config: player.mesh must be a table");
        } else {
            const int MT = lua_gettop(L);
            if (!PARSE_MESH(MT, g_playerCfg.path, g_playerCfg.posOffset,
                            g_playerCfg.rotDeg, g_playerCfg.scale,
                            g_playerCfg.emissiveScale, g_playerCfg.flat,
                            g_playerCfg.center, g_playerCfg.centerOffset))
                return luaL_error(L, "hypr3d.config: player.mesh is invalid");
            lua_pop(L, 1);
        }

        // Legacy single keys, kept as a fallback for older configs. Applied
        // ONLY when the key is actually present: model_scale used to rebuild
        // the scale as {x, x, x} on every parse, silently collapsing the
        // mesh block's per-axis transform.scale.
        lua_getfield(L, idx, "model");
        if (!lua_isnil(L, -1)) {
            lua_pop(L, 1);
            if (!SET_STRING(idx, "model", g_playerCfg.path, "player.model"))
                return luaL_error(L, "hypr3d.config: player.model must be a string");
        } else {
            lua_pop(L, 1);
        }
        lua_getfield(L, idx, "model_scale");
        if (!lua_isnil(L, -1)) {
            lua_pop(L, 1);
            float SCALE_F = g_playerCfg.scale.x;
            if (!SET_NUM(idx, "model_scale", SCALE_F, "player.model_scale"))
                return luaL_error(L, "hypr3d.config: player.model_scale must be a number");
            g_playerCfg.scale = Vec3{SCALE_F, SCALE_F, SCALE_F};
        } else {
            lua_pop(L, 1);
        }
        lua_getfield(L, idx, "model_turn");
        if (!lua_isnil(L, -1)) {
            lua_pop(L, 1);
            if (!SET_NUM(idx, "model_turn", g_playerCfg.rotDeg.y, "player.model_turn"))
                return luaL_error(L, "hypr3d.config: player.model_turn must be a number");
        } else {
            lua_pop(L, 1);
        }

        const auto SET_ANIM = [&](const char* key, int slot) -> bool {
            lua_getfield(L, idx, key);
            if (lua_isnil(L, -1)) {
                lua_pop(L, 1);
                return true;
            }
            if (lua_isnumber(L, -1)) {
                g_playerCfg.animIdx[slot] = static_cast<int>(lua_tonumber(L, -1));
                g_playerCfg.animName[slot].clear();
            } else if (lua_isstring(L, -1)) {
                size_t LEN = 0;
                const char* STR = lua_tolstring(L, -1, &LEN);
                g_playerCfg.animName[slot].assign(STR, LEN);
                g_playerCfg.animIdx[slot] = -1;
            } else {
                lua_pop(L, 1);
                return false;
            }
            lua_pop(L, 1);
            return true;
        };
        if (!SET_ANIM("anim_idle", 0) || !SET_ANIM("anim_walk", 1) ||
            !SET_ANIM("anim_run", 2) || !SET_ANIM("anim_jump", 3))
            return luaL_error(L, "hypr3d.config: player.anim_* must be an animation index or name");

        // animations = { idle = {source = "Idle" | 0, duration_scale = 1.0},
        //                ... } -- source is a clip name or an index;
        // duration_scale is the playback SPEED multiplier (1 = as authored).
        static const char* const STATE_KEYS[4] = {"idle", "walk", "run", "jump"};
        lua_getfield(L, idx, "animations");
        if (lua_isnil(L, -1)) {
            lua_pop(L, 1);
        } else if (!lua_istable(L, -1)) {
            lua_pop(L, 1);
            return luaL_error(L, "hypr3d.config: player.animations must be a table");
        } else {
            const int AT = lua_gettop(L);
            for (int slot = 0; slot < 4; ++slot) {
                lua_getfield(L, AT, STATE_KEYS[slot]);
                if (lua_isnil(L, -1)) {
                    lua_pop(L, 1);
                    continue;
                }
                if (!lua_istable(L, -1)) {
                    lua_pop(L, 1);
                    return luaL_error(L,
                        "hypr3d.config: player.animations.%s must be a table",
                        STATE_KEYS[slot]);
                }
                const int ST = lua_gettop(L);

                lua_getfield(L, ST, "source");
                if (lua_isnumber(L, -1)) {
                    g_playerCfg.animIdx[slot] = static_cast<int>(lua_tonumber(L, -1));
                    g_playerCfg.animName[slot].clear();
                } else if (lua_isstring(L, -1)) {
                    size_t LEN = 0;
                    const char* STR = lua_tolstring(L, -1, &LEN);
                    g_playerCfg.animName[slot].assign(STR, LEN);
                    g_playerCfg.animIdx[slot] = -1;
                } else {
                    lua_pop(L, 2);
                    return luaL_error(L,
                        "hypr3d.config: player.animations.%s.source must be a clip name or index",
                        STATE_KEYS[slot]);
                }
                lua_pop(L, 1); // source

                float SPEED = 1.0f;
                if (!SET_NUM(ST, "duration_scale", SPEED,
                             "player.animations.<state>.duration_scale"))
                    return luaL_error(L,
                        "hypr3d.config: player.animations.<state>.duration_scale must be a number");
                g_playerAnimSpeed[slot] = SPEED;
                g_playerCfg.animSpeed[slot] = SPEED;

                lua_pop(L, 1); // the state table
            }
            lua_pop(L, 1); // animations
        }
        if (!SET_VEC3(idx, "spawn", g_playerSpawn, "player.spawn"))
            return luaL_error(L, "hypr3d.config: player.spawn must be a table { x = .., y = .., z = .. }");
        lua_pop(L, 1);
    }

    // scene = { <any name> = { path, transform, emissive_scale, flat,
    //           collision, static }, ... } -- unlimited named objects.
    // Object names are free-form (for the user's readability only).
    idx = SECTION("scene", "scene");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: scene must be a table");
    if (idx > 0) {
        std::vector<SSceneObjectCfg> OBJECTS;

        lua_pushnil(L);
        while (lua_next(L, idx) != 0) {
            // stack: [key, value]
            if (!lua_istable(L, -1)) {
                lua_pop(L, 1); // keep the key for the next iteration
                continue;
            }

            SSceneObjectCfg OBJ;
            const int OIDX = lua_gettop(L);

            // The table key names the object. Reconciliation matches by
            // NAME, not index: lua_next order is NOT stable across config
            // parses (string hashes are seeded per lua state, so two parses
            // of the SAME file can iterate the objects in opposite orders),
            // and an index-matched reconcile cross-wired the objects -- the
            // untouched static map inherited a dynamic object's fallen pose
            // and rotated away out of view.
            if (lua_type(L, OIDX - 1) == LUA_TSTRING) {
                size_t KLEN = 0;
                const char* KSTR = lua_tolstring(L, OIDX - 1, &KLEN);
                OBJ.name.assign(KSTR, KLEN);
            }

            if (!SET_STRING(OIDX, "path", OBJ.path, "scene.<name>.path")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.path must be a string");
            }

            // transform = { position, rotation, scale } -- a NESTED table.
            // (An earlier parser revision looked for position/rotation/scale
            // directly on the object, found nothing, and every object
            // rendered at its identity transform.)
            lua_getfield(L, OIDX, "transform");
            if (lua_isnil(L, -1)) {
                lua_pop(L, 1);
            } else if (!lua_istable(L, -1)) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.transform must be a table with position/rotation/scale");
            } else {
                const int TIDX = lua_gettop(L);

                if (!SET_VEC3(TIDX, "position", OBJ.position,
                              "scene.<name>.transform.position"))
                    return luaL_error(L, "hypr3d.config: scene.<name>.transform.position must be a table { x = .., y = .., z = .. }");
                if (!SET_VEC3(TIDX, "rotation", OBJ.rotationDeg,
                              "scene.<name>.transform.rotation"))
                    return luaL_error(L, "hypr3d.config: scene.<name>.transform.rotation must be a table { x = .., y = .., z = .. } (degrees)");
                if (!SET_VEC3(TIDX, "scale", OBJ.scale,
                              "scene.<name>.transform.scale"))
                    return luaL_error(L, "hypr3d.config: scene.<name>.transform.scale must be a table { x = .., y = .., z = .. }");

                lua_pop(L, 1);
            }

            if (!SET_NUM(OIDX, "emissive_scale", OBJ.emissiveScale,
                         "scene.<name>.emissive_scale")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.emissive_scale must be a number");
            }
            if (!SET_BOOL(OIDX, "flat", OBJ.flat, "scene.<name>.flat")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.flat must be a boolean");
            }
            if (!SET_BOOL(OIDX, "collision", OBJ.collision, "scene.<name>.collision")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.collision must be a boolean");
            }

            // static = true (default): immovable location geometry.
            // static = false: a dynamic object -- grabbable with Super+LMB.
            bool staticObj = true;
            if (!SET_BOOL(OIDX, "static", staticObj, "scene.<name>.static")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.static must be a boolean");
            }
            OBJ.dynamic = !staticObj;

            // physics = true: gravity + world collisions (dynamic only;
            // ignored on static objects).
            if (!SET_BOOL(OIDX, "physics", OBJ.physics, "scene.<name>.physics")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.physics must be a boolean");
            }

            // Rotation pivot: "logical" rotates around the mesh's local AABB
            // center, "origin" around its own origin; center_offset shifts
            // the pivot on top in local units.
            std::string CENTER = "logical";
            if (!SET_STRING(OIDX, "center", CENTER, "scene.<name>.center")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.center must be a string");
            }
            OBJ.center = CENTER == "origin" ? CMapModel::ECenter::Origin
                                            : CMapModel::ECenter::Logical;
            if (!SET_VEC3(OIDX, "center_offset", OBJ.centerOffset,
                          "scene.<name>.center_offset")) {
                lua_pop(L, 1);
                return luaL_error(L, "hypr3d.config: scene.<name>.center_offset must be a table { x = .., y = .., z = .. }");
            }

            // mesh = { ... }: the shared visual description -- overrides
            // the legacy flat keys when present.
            lua_getfield(L, OIDX, "mesh");
            if (lua_istable(L, -1)) {
                const int MT = lua_gettop(L);
                std::string CENTER = "origin";
                Vec3 MROT{};
                bool MFLAT = OBJ.flat;
                float MEMIS = OBJ.emissiveScale;
                if (!SET_STRING(MT, "path", OBJ.path, "mesh.path") ||
                    !SET_VEC3(MT, "center_offset", OBJ.centerOffset,
                              "mesh.center_offset") ||
                    !SET_NUM(MT, "emissive_scale", MEMIS, "mesh.emissive_scale") ||
                    !SET_BOOL(MT, "flat", MFLAT, "mesh.flat") ||
                    !SET_STRING(MT, "center", CENTER, "mesh.center"))
                    return luaL_error(L, "hypr3d.config: scene.<name>.mesh is invalid");
                OBJ.emissiveScale = MEMIS;
                OBJ.flat = MFLAT;
                OBJ.center = CENTER == "origin"
                    ? CMapModel::ECenter::Origin : CMapModel::ECenter::Logical;

                lua_getfield(L, MT, "transform");
                if (lua_isnil(L, -1)) {
                    lua_pop(L, 1);
                } else if (!lua_istable(L, -1)) {
                    lua_pop(L, 1);
                    return luaL_error(L, "hypr3d.config: scene.<name>.mesh.transform must be a table");
                } else {
                    const int TIDX = lua_gettop(L);
                    if (!SET_VEC3(TIDX, "position", OBJ.position,
                                  "mesh.transform.position") ||
                        !SET_VEC3(TIDX, "rotation", OBJ.rotationDeg,
                                  "mesh.transform.rotation") ||
                        !SET_VEC3(TIDX, "scale", OBJ.scale,
                                  "mesh.transform.scale"))
                        return luaL_error(L, "hypr3d.config: scene.<name>.mesh.transform is invalid");
                    lua_pop(L, 1);
                }
            }
            lua_pop(L, 1); // pop mesh (or its nil)

            OBJECTS.push_back(OBJ);
            lua_pop(L, 1); // pop the value; the key remains for lua_next
        }

        // Canonical order: lua_next order varies between parses (seeded
        // hashes), and every index-keyed structure downstream (GLScene
        // slots, Jolt bodies, BVH trees) would churn -- or swap model files
        // between slots -- on every reload. Sorting by name makes the
        // parsed list deterministic.
        std::sort(OBJECTS.begin(), OBJECTS.end(),
                  [](const SSceneObjectCfg& A, const SSceneObjectCfg& B) {
                      return A.name < B.name;
                  });

        // Snapshot the RAW parsed lua FIRST: this is the file's truth, and
        // the next reload compares against it to tell "user edited the file"
        // from "the simulation moved the object". (Snapshotting after the
        // reconciliation below would record simulated positions as config
        // values -- the following reload then saw a phantom change and kept
        // teleporting carried/fallen objects back to the config placement.)
        g_sceneLuaState = OBJECTS;
        g_sceneLuaValid = true;

        // Reconciliation, per field, matched BY NAME: a field whose lua
        // value did not change keeps the simulated state (carry / zoom /
        // roll / physics); a field edited in the file takes the new config
        // value. Scale is never simulated, so it always comes from the
        // config. Static objects are config-owned by definition -- they
        // never move by simulation and never take simulated state, so a
        // past parse glitch cannot persist through them either.
        for (auto& NEW : OBJECTS) {
            if (!NEW.dynamic)
                continue;

            const SSceneObjectCfg* OLD = nullptr;
            for (const auto& O : g_sceneLuaState)
                if (O.name == NEW.name) {
                    OLD = &O;
                    break;
                }
            if (!OLD)
                continue;

            const SSceneObjectCfg* SIM = nullptr;
            for (const auto& O : g_sceneObjects)
                if (O.name == NEW.name) {
                    SIM = &O;
                    break;
                }
            if (!SIM)
                continue;

            if (OLD->position.x == NEW.position.x &&
                OLD->position.y == NEW.position.y &&
                OLD->position.z == NEW.position.z)
                NEW.position = SIM->position;

            if (OLD->rotationDeg.x == NEW.rotationDeg.x &&
                OLD->rotationDeg.y == NEW.rotationDeg.y &&
                OLD->rotationDeg.z == NEW.rotationDeg.z)
                NEW.rotationDeg = SIM->rotationDeg;
        }

        g_sceneObjects = std::move(OBJECTS);
        lua_pop(L, 1);
    }

    // characters = { name = { path, transform, flat, idle = { clips } } }:
    // animated (skinned) glTF models playing their idle clips in place,
    // picked at random each time a clip ends. No collision or physics yet.
    idx = SECTION("characters", "characters");
    if (idx == -1)
        return luaL_error(L, "hypr3d.config: characters must be a table");
    {
        std::vector<GLScene::SCharacterSpec> CHARS;

        if (idx > 0) {
            lua_pushnil(L);
            while (lua_next(L, idx) != 0) {
                if (!lua_istable(L, -1) || lua_type(L, -2) != LUA_TSTRING) {
                    lua_pop(L, 1);
                    continue;
                }

                GLScene::SCharacterSpec C;
                const int CIDX = lua_gettop(L);
                C.name = lua_tostring(L, CIDX - 1);

                if (!SET_STRING(CIDX, "path", C.path, "characters.<name>.path") ||
                    !SET_BOOL(CIDX, "flat", C.flat, "characters.<name>.flat"))
                    return luaL_error(L, "hypr3d.config: characters.%s: path must be a string, flat a boolean",
                                      C.name.c_str());

                lua_getfield(L, CIDX, "transform");
                if (lua_istable(L, -1)) {
                    const int TIDX = lua_gettop(L);
                    if (!SET_VEC3(TIDX, "position", C.position, "characters.<name>.transform.position") ||
                        !SET_VEC3(TIDX, "rotation", C.rotationDeg, "characters.<name>.transform.rotation") ||
                        !SET_VEC3(TIDX, "scale", C.scale, "characters.<name>.transform.scale"))
                        return luaL_error(L, "hypr3d.config: characters.%s.transform is invalid",
                                          C.name.c_str());
                } else if (!lua_isnil(L, -1))
                    return luaL_error(L, "hypr3d.config: characters.%s.transform must be a table",
                                      C.name.c_str());
                lua_pop(L, 1);

                lua_getfield(L, CIDX, "idle");
                if (lua_istable(L, -1)) {
                    const int IIDX = lua_gettop(L);
                    const lua_Integer N = luaL_len(L, IIDX);
                    for (lua_Integer i = 1; i <= N; ++i) {
                        lua_rawgeti(L, IIDX, i);
                        if (lua_type(L, -1) == LUA_TSTRING)
                            C.idle.emplace_back(lua_tostring(L, -1));
                        lua_pop(L, 1);
                    }
                } else if (!lua_isnil(L, -1))
                    return luaL_error(L, "hypr3d.config: characters.%s.idle must be a list of clip names",
                                      C.name.c_str());
                lua_pop(L, 1);

                float RADIUS = 0.3f, HEIGHT = 1.8f;
                if (!SET_NUM(CIDX, "radius", RADIUS, "characters.<name>.radius") ||
                    !SET_NUM(CIDX, "height", HEIGHT, "characters.<name>.height"))
                    return luaL_error(L, "hypr3d.config: characters.%s: radius and height must be numbers",
                                      C.name.c_str());
                C.radius = std::max(0.05f, RADIUS);
                C.height = std::max(0.1f, HEIGHT);

                if (!C.path.empty())
                    CHARS.push_back(std::move(C));
                lua_pop(L, 1); // the value; the key stays for lua_next
            }
            lua_pop(L, 1); // the section
        }

        // Reconcile with the running characters by name: a character keeps
        // where it was carried to unless its config placement changed.
        std::vector<SCharState> NEXT;
        for (const auto& SPEC : CHARS) {
            SCharState ST;
            ST.cfg    = SPEC;
            ST.feet   = SPEC.position;
            ST.yawDeg = SPEC.rotationDeg.y;
            ST.radius = SPEC.radius;
            ST.height = SPEC.height;

            for (auto& OLD : g_chars) {
                if (OLD.cfg.name != SPEC.name)
                    continue;
                const bool SAME_PLACE =
                    OLD.cfg.position.x == SPEC.position.x &&
                    OLD.cfg.position.y == SPEC.position.y &&
                    OLD.cfg.position.z == SPEC.position.z &&
                    OLD.cfg.rotationDeg.y == SPEC.rotationDeg.y;
                if (SAME_PLACE) {
                    ST.feet   = OLD.feet;
                    ST.yawDeg = OLD.yawDeg;
                }
                // The body is rebuilt when its size changed.
                if (OLD.radius == ST.radius && OLD.height == ST.height)
                    std::swap(ST.body, OLD.body);
                break;
            }
            NEXT.push_back(std::move(ST));
        }
        for (auto& OLD : g_chars)
            destroyCharBody(OLD);
        g_chars = std::move(NEXT);
        g_charGrabIndex = SIZE_MAX; // indices may have shifted

        g_scene.setCharacters(CHARS);
    }

    g_scene.setPlayerConfig(g_playerCfg);
    for (int slot = 0; slot < 4; ++slot)
        g_scene.player()->setAnimSpeed(static_cast<CPlayerModel::EState>(slot),
                                       g_playerAnimSpeed[slot]);

    return 0;
}

static int luaToggle(lua_State*) {
    toggle3D();
    return 0;
}

static int luaOpen(lua_State*) {
    open3D();
    return 0;
}

static int luaClose(lua_State*) {
    close3D();
    return 0;
}

// Whether the 3D view is on (or fading in): true from open until close,
// so a config-side toggle can tell the two apart without tracking state
// that a config reload would lose.
static int luaActive(lua_State* L) {
    lua_pushboolean(L, g_active && g_transitionTarget > 0.5f);
    return 1;
}

static SDispatchResult dispatchToggle(std::string) {
    toggle3D();
    return {};
}

static SDispatchResult dispatchOpen(std::string) {
    open3D();
    return {};
}

static SDispatchResult dispatchClose(std::string) {
    close3D();
    return {};
}

APICALL EXPORT std::string PLUGIN_API_VERSION() {
    return HYPRLAND_API_VERSION;
}

APICALL EXPORT PLUGIN_DESCRIPTION_INFO PLUGIN_INIT(HANDLE handle) {
    joltInit();

    PHANDLE = handle;

    const std::string serverHash = __hyprland_api_get_hash();
    const std::string clientHash = __hyprland_api_get_client_hash();

    if (serverHash != clientHash)
        throw std::runtime_error("[hypr3d] Hyprland API hash mismatch");

    // Relative pointer capture is what lets the mouse look around freely
    // instead of hitting the edge of whatever screen it started on.
    g_hookInstalled =
        Compat::installPointerHook(PHANDLE, &hookSink);

    if (!g_hookInstalled) {
        g_reportedPointerHookError = true;

        notify(
            "[hypr3d] pointer hook unavailable: mouse-look falls back to "
            "cursor-relative motion and will stop at screen edges",
            CHyprColor{1.0f, 0.6f, 0.2f, 1.0f}
        );
    }

    HyprlandAPI::addNotification(
        PHANDLE,
        "[hypr3d] renderer loaded",
        CHyprColor{0.2f, 0.8f, 0.4f, 1.0f},
        3000
    );

    if (!HyprlandAPI::addDispatcherV2(PHANDLE, "hypr3d:toggle", dispatchToggle))
        throw std::runtime_error("[hypr3d] failed to register toggle dispatcher");

    if (!HyprlandAPI::addDispatcherV2(PHANDLE, "hypr3d:open", dispatchOpen))
        throw std::runtime_error("[hypr3d] failed to register open dispatcher");

    if (!HyprlandAPI::addDispatcherV2(PHANDLE, "hypr3d:close", dispatchClose))
        throw std::runtime_error("[hypr3d] failed to register close dispatcher");

    if (!HyprlandAPI::addLuaFunction(PHANDLE, "hypr3d", "toggle", luaToggle))
        throw std::runtime_error("[hypr3d] failed to register Lua toggle");

    if (!HyprlandAPI::addLuaFunction(PHANDLE, "hypr3d", "open", luaOpen))
        throw std::runtime_error("[hypr3d] failed to register Lua open");

    if (!HyprlandAPI::addLuaFunction(PHANDLE, "hypr3d", "active", luaActive))
        throw std::runtime_error("[hypr3d] failed to register Lua active");
    if (!HyprlandAPI::addLuaFunction(PHANDLE, "hypr3d", "close", luaClose))
        throw std::runtime_error("[hypr3d] failed to register Lua close");

    if (!HyprlandAPI::addLuaFunction(PHANDLE, "hypr3d", "config", luaConfig))
        throw std::runtime_error("[hypr3d] failed to register Lua config");

    // Every handler is wrapped: an exception must NEVER escape into
    // Hyprland (std::terminate there kills the whole compositor). The
    // exception text lands in the status dump instead.
    static auto renderPre =
        Event::bus()->m_events.render.pre.listen(
            [](PHLMONITOR mon) {
                try {
                    onRenderPre(mon);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"renderPre: "} + e.what());
                } catch (...) {
                    dumpErrorNow("renderPre: unknown exception");
                }
            }
        );

    static auto renderStage =
        Event::bus()->m_events.render.stage.listen(
            [](eRenderStage stage) {
                try {
                    onRenderStage(stage);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"renderStage: "} + e.what());
                } catch (...) {
                    dumpErrorNow("renderStage: unknown exception");
                }
            }
        );

    static auto mouseMove =
        Event::bus()->m_events.input.mouse.move.listen(
            [](Vector2D pos, Event::SCallbackInfo& info) {
                try {
                    onMouseMove(pos, info);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"mouseMove: "} + e.what());
                } catch (...) {
                    dumpErrorNow("mouseMove: unknown exception");
                }
            }
        );

    static auto mouseButton =
        Event::bus()->m_events.input.mouse.button.listen(
            [](IPointer::SButtonEvent event, Event::SCallbackInfo& info) {
                try {
                    onMouseButton(event, info);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"mouseButton: "} + e.what());
                } catch (...) {
                    dumpErrorNow("mouseButton: unknown exception");
                }
            }
        );

    static auto mouseAxis =
        Event::bus()->m_events.input.mouse.axis.listen(
            [](IPointer::SAxisEvent event, Event::SCallbackInfo& info) {
                try {
                    onMouseAxis(event, info);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"mouseAxis: "} + e.what());
                } catch (...) {
                    dumpErrorNow("mouseAxis: unknown exception");
                }
            }
        );

    static auto keyboardKey =
        Event::bus()->m_events.input.keyboard.key.listen(
            [](IKeyboard::SKeyEvent event, Event::SCallbackInfo& info) {
                try {
                    onKeyboardKey(event, info);
                } catch (const std::exception& e) {
                    dumpErrorNow(std::string{"keyboardKey: "} + e.what());
                } catch (...) {
                    dumpErrorNow("keyboardKey: unknown exception");
                }
            }
        );

    (void)renderPre;
    (void)renderStage;
    (void)mouseMove;
    (void)mouseButton;
    (void)mouseAxis;
    (void)keyboardKey;

    if (!HyprlandAPI::reloadConfig()) {
        notify(
            "[hypr3d] reloadConfig failed",
            CHyprColor{1.0f, 0.6f, 0.2f, 1.0f}
        );
    }

    return {
        "hypr3d",
        "A new perspective on window management",
        "Samine825",
        "0.5.0"
    };
}

APICALL EXPORT void PLUGIN_EXIT() {
    // Hyprland keeps the last frame's pass elements until the next
    // beginRender() clears them -- ours included. After dlclose() that clear()
    // ran the destructor of a CHypr3DPassElement whose code was gone: unloading
    // while the 3D view was on crashed Hyprland (SIGSEGV in
    // IHyprRenderer::beginRender, measured on 0.56.2). `hyprctl plugin unload`
    // calls this from the event loop, between frames, so the element is done.
    if (g_pHyprRenderer)
        g_pHyprRenderer->m_renderPass.removeAllOfType("Hypr3D");

    if (g_deactivateLater && g_pEventLoopManager)
        g_pEventLoopManager->removeDoLater(g_deactivateLater);
    g_deactivateLater = 0;

    joltShutdown();

    // The bodies died with the system; stale IDs would be read by the next
    // PLUGIN_INIT of this same mapping (see joltInit).
    g_playerBody = {};
    g_floorBody  = {};
    g_joltBodies.clear();
    for (auto& C : g_chars)
        C.body = {}; // died with the system

    g_active = false;
    stopFramePump();
    g_transition = 0.0f;
    g_transitionTarget = 0.0f;

    if (Compat::setCursorHidden(false) && g_pHyprRenderer)
        g_pHyprRenderer->setCursorFromName("default", true);

    clearAimFocus();

    g_world.clear();
    g_renderWindows.clear();

    // Defer the snapshot release to the event loop: this runs INSIDE
    // Hyprland's render pass, and destroying framebuffers/textures here
    // breaks the frame that is still being composed (windows left
    // transparent until their next repaint).
    g_captureReleasePending = true;

    // Put the windows back under the layout before the plugin goes away.
    unghostWindows();

    // The FS fade channel: on FS-on Hyprland dims EVERY other workspace
    // window to alpha 0 (WINDOW_ALPHA_FULLSCREEN). If our transition raced
    // the fade, windows can be left semi-transparent after the 3D exit.
    // Reset the channel for the active workspace unconditionally.
    if (const auto MON = targetMonitor(); MON && MON->m_activeWorkspace) {
        for (auto const& W3 : Desktop::windowState()->windows()) {
            if (W3 && W3->m_workspace == MON->m_activeWorkspace &&
                !W3->m_pinned)
                *W3->alpha(Desktop::View::WINDOW_ALPHA_FULLSCREEN) = 1.F;
        }
    }

    // The ghosted windows were skipped by Hyprland's main render during 3D;
    // force a full monitor repaint so every window is re-composited from its
    // own buffer immediately (static clients would otherwise stay invisible
    // until their first repaint).
    if (g_pHyprRenderer && g_monitor)
        g_pHyprRenderer->damageMonitor(g_monitor);

    Compat::setPointerCapture(false);
    Compat::removePointerHook();

    g_monitor = nullptr;

    if (Render::GL::g_pHyprOpenGL) {
        Render::GL::g_pHyprOpenGL->makeEGLCurrent();
        g_cursorBufferTex.reset();
        g_cursorTexHold.reset();
        g_scene.shutdown();
    }
}

} // namespace H3D

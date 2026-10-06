#pragma once

#include <array>
#include <cmath>

namespace H3D {

struct Vec2 {
    float x = 0.f;
    float y = 0.f;
};

struct Vec3 {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
};

struct Mat4 {
    std::array<float, 16> m{};

    static Mat4 identity();
    static Mat4 perspective(float fovYRad, float aspect, float zNear, float zFar);
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up);
    static Mat4 rotationY(float r);
    static Mat4 rotationX(float r);
    static Mat4 rotationZ(float r);
    static Mat4 translation(const Vec3& v);
    static Mat4 scale(const Vec3& v);

    // Adopt a column-major float[16] (the cgltf/glTF node matrix layout).
    static Mat4 fromColumnMajor(const float src[16]);
};

Mat4 operator*(const Mat4& a, const Mat4& b);

Vec3 normalize(const Vec3& v);
Vec3 cross(const Vec3& a, const Vec3& b);
float dot(const Vec3& a, const Vec3& b);

// First-person camera. This is the single source of truth for the view pose:
// mouse-look, movement and crosshair picking all read the same yaw/pitch, so
// what the crosshair points at is by construction what the camera renders.
class Camera {
  public:
    // Look limits. Pitch is clamped so the camera cannot flip over; yaw wraps
    // so it never loses float precision after long sessions.
    static constexpr float kPitchLimit = 1.5f;
    static constexpr float kTau        = 6.28318530717958647692f;
    // The grid platform's plane: world zero. Feet-level coords (player
    // spawn, walking clamp) are measured against it.
    static constexpr float kFloorY     = 0.0f;

    // Minecraft-style player body: an axis-aligned 0.6 x 1.8 x 0.6 box. The
    // eye sits 1.62 above the feet, so standing on the floor means the camera
    // stops at kFloorY + kEyeHeight. Half width is what window collision
    // inflates quads by.
    static constexpr float kBodyHalfWidth = 0.3f;
    static constexpr float kBodyHeight    = 1.8f;
    static constexpr float kEyeHeight     = 1.62f;

    Vec3  position{0.f, 0.f, 12.f};
    float yaw   = 0.f;
    float pitch = 0.f;
    // View-only roll (radians): the walk bob tilts the head around the
    // view axis. Never touched by look/movement -- bob writes it, view()
    // consumes it, everything else sees 0.
    float roll  = 0.f;

    // Third-person front view: view() and centerRay() look BACK along the
    // look axis while movement and the look state stay unchanged.
    bool  mirrorView = false;
    float moveSpeed = 8.f;
    float mouseSensitivity = 0.0025f;

    // Apply an already-converted rotation in radians. Prefer this: it keeps the
    // sensitivity policy in InputController where it can be tested.
    void applyLook(float yawDelta, float pitchDelta);

    // Raw pointer counts -> rotation, applying mouseSensitivity. Convenience
    // for callers that do not go through InputController.
    void mouseDelta(float dx, float dy);

    void move(float forward, float right, float vertical, float dt);

    // Forward direction projected onto the ground plane (walking axis).
    Vec3 flatForward() const;

    // Apply an already-computed world-space velocity (units/second) with the
    // same floor clamp as move(). The inertia layer in main.cpp integrates
    // its own glide toward the key-driven velocity and lands here.
    void displace(const Vec3& worldVelocity, float dt);

    // Unit look direction.
    Vec3 forward() const;

    // Unit strafe direction (camera right).
    Vec3 right() const;

    // The picking ray: through the centre of the screen, i.e. the crosshair.
    // Deliberately independent of where the OS cursor happens to be.
    Vec3 centerRay() const {
        return mirrorView ? forward() * -1.0f : forward();
    }

    Mat4 view() const;
};

} // namespace H3D

#include "World/Camera.hpp"

#include <algorithm>

namespace H3D {

Vec3 normalize(const Vec3& v) {
    const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (l < 0.00001f)
        return {};
    return {v.x / l, v.y / l, v.z / l};
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float wrapPi(float a) {
    // Shift by +pi first so the fmod result is in [0, 2pi), then subtract pi
    // to land in [-pi, pi). Shifting by 2pi instead would map 0 to -pi and
    // send the camera facing backwards.
    a = std::fmod(a + Camera::kTau * 0.5f, Camera::kTau);

    if (a < 0.f)
        a += Camera::kTau;

    return a - Camera::kTau * 0.5f;
}

Mat4 Mat4::identity() {
    Mat4 r;
    r.m = {1.f,0.f,0.f,0.f, 0.f,1.f,0.f,0.f, 0.f,0.f,1.f,0.f, 0.f,0.f,0.f,1.f};
    return r;
}

Mat4 Mat4::perspective(float fovYRad, float aspect, float zNear, float zFar) {
    Mat4 r{};
    const float f = 1.f / std::tan(fovYRad * 0.5f);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (zFar + zNear) / (zNear - zFar);
    r.m[11] = -1.f;
    r.m[14] = (2.f * zFar * zNear) / (zNear - zFar);
    return r;
}

Mat4 Mat4::lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = normalize(center - eye);
    const Vec3 s = normalize(cross(f, up));
    const Vec3 u = cross(s, f);

    Mat4 r = identity();
    r.m[0] = s.x; r.m[4] = s.y; r.m[8]  = s.z;
    r.m[1] = u.x; r.m[5] = u.y; r.m[9]  = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -dot(s, eye);
    r.m[13] = -dot(u, eye);
    r.m[14] = dot(f, eye);
    return r;
}

Mat4 Mat4::rotationY(float r) {
    Mat4 out = identity();
    const float c = std::cos(r), s = std::sin(r);
    out.m[0] = c; out.m[2] = -s;
    out.m[8] = s; out.m[10] = c;
    return out;
}

Mat4 Mat4::rotationX(float r) {
    Mat4 out = identity();
    const float c = std::cos(r), s = std::sin(r);
    out.m[5] = c; out.m[6] = s;
    out.m[9] = -s; out.m[10] = c;
    return out;
}

Mat4 Mat4::rotationZ(float r) {
    Mat4 out = identity();
    const float c = std::cos(r), s = std::sin(r);
    out.m[0] = c;  out.m[1] = s;   // col 0 = (c, s, 0): +X rotates toward +Y
    out.m[4] = -s; out.m[5] = c;   // col 1 = (-s, c, 0)
    return out;
}

Mat4 Mat4::translation(const Vec3& v) {
    Mat4 out = identity();
    out.m[12] = v.x; out.m[13] = v.y; out.m[14] = v.z;
    return out;
}

Mat4 Mat4::scale(const Vec3& v) {
    Mat4 out = identity();
    out.m[0] = v.x; out.m[5] = v.y; out.m[10] = v.z;
    return out;
}

Mat4 Mat4::fromColumnMajor(const float src[16]) {
    Mat4 r;
    std::copy(src, src + 16, r.m.begin());
    return r;
}

Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int c = 0; c < 4; ++c)
        for (int row = 0; row < 4; ++row)
            r.m[c * 4 + row] =
                a.m[0 * 4 + row] * b.m[c * 4 + 0] +
                a.m[1 * 4 + row] * b.m[c * 4 + 1] +
                a.m[2 * 4 + row] * b.m[c * 4 + 2] +
                a.m[3 * 4 + row] * b.m[c * 4 + 3];
    return r;
}

void Camera::applyLook(float yawDelta, float pitchDelta) {
    if (!std::isfinite(yawDelta) || !std::isfinite(pitchDelta))
        return;

    yaw   = wrapPi(yaw + yawDelta);
    pitch = std::clamp(pitch + pitchDelta, -kPitchLimit, kPitchLimit);
}

void Camera::mouseDelta(float dx, float dy) {
    if (!std::isfinite(dx) || !std::isfinite(dy))
        return;

    // Same sign convention as InputController: dy arrives with Y growing DOWN
    // (libinput/compositor pointer delta), so a positive dy pitches the view
    // down. The two paths must agree or whichever one is wired up later
    // silently inverts pitch.
    applyLook(static_cast<float>(dx) * mouseSensitivity,
              static_cast<float>(-dy) * mouseSensitivity);
}

void Camera::move(float forward, float right, float vertical, float dt) {
    if (!std::isfinite(dt) || dt <= 0.f)
        return;

    // `right` here is the caller's strafe input and shadows right() -- the
    // strafe basis vector must be qualified through `this`.
    position += this->right() * (right * moveSpeed * dt);
    position += Vec3{0.f, 1.f, 0.f} * (vertical * moveSpeed * dt);

    // Strafe along the ground plane rather than into the floor/ceiling, which
    // is what makes WASD feel like walking instead of like a flying camera.
    position += flatForward() * (forward * moveSpeed * dt);

    // No world floor clamp: the visible grid platform is a real collidable
    // slab (built in main.cpp), and everywhere past its edge the player can
    // descend freely.
}

Vec3 Camera::flatForward() const {
    return normalize({std::sin(yaw), 0.f, -std::cos(yaw)});
}

void Camera::displace(const Vec3& worldVelocity, float dt) {
    if (!std::isfinite(dt) || dt <= 0.f)
        return;

    position += worldVelocity * dt;
}

Vec3 Camera::forward() const {
    return {
        std::sin(yaw) * std::cos(pitch),
        std::sin(pitch),
        -std::cos(yaw) * std::cos(pitch),
    };
}

Vec3 Camera::right() const {
    return normalize(cross(forward(), {0.f, 1.f, 0.f}));
}

Mat4 Camera::view() const {
    // Roll: rotate the up vector around the view axis (Rodrigues with
    // axis=forward, and forward is perpendicular to world up, so
    // up' = up*cos(r) + right*sin(r)).
    const Vec3 U = Vec3{0.f, 1.f, 0.f} * std::cos(roll) +
                   right() * std::sin(roll);
    const Vec3 DIR = mirrorView ? forward() * -1.0f : forward();
    return Mat4::lookAt(position, position + DIR, U);
}

} // namespace H3D

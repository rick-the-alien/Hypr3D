#include "World/MapCollision.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace H3D {

namespace {

constexpr uint32_t kLeafSize = 8;

// Touch tolerance: resting exactly ON a surface is not a collision.
constexpr float EPS = 1e-3f;

Vec3 triMin(const CMapCollision::STL& t) {
    return {
        std::min({t.a.x, t.b.x, t.c.x}),
        std::min({t.a.y, t.b.y, t.c.y}),
        std::min({t.a.z, t.b.z, t.c.z}),
    };
}

Vec3 triMax(const CMapCollision::STL& t) {
    return {
        std::max({t.a.x, t.b.x, t.c.x}),
        std::max({t.a.y, t.b.y, t.c.y}),
        std::max({t.a.z, t.b.z, t.c.z}),
    };
}

// Möller–Trumbore. Two-sided: the map may be entered from either side.
bool rayTriangle(const Vec3& o, const Vec3& d, const CMapCollision::STL& t,
                 float& outT) {
    return mapRayTriangle(o, d, t.a, t.b, t.c, outT);
}

// Closest point on a 2D triangle (x, z projected) to point p. Degenerate
// (zero-area) projections fall back to edge searches.
static Vec3 closestTri2D(const CMapCollision::STL& t, float px, float pz) {
    const float A[2] = {t.a.x, t.a.z}, B[2] = {t.b.x, t.b.z}, C[2] = {t.c.x, t.c.z};
    const float P[2] = {px, pz};

    const float AB[2] = {B[0] - A[0], B[1] - A[1]};
    const float AC[2] = {C[0] - A[0], C[1] - A[1]};
    const float AP[2] = {P[0] - A[0], P[1] - A[1]};

    const float DOT_AB_AC = AB[0] * AC[0] + AB[1] * AC[1];
    const float DOT_AB_AP = AB[0] * AP[0] + AB[1] * AP[1];
    const float DOT_AC_AP = AC[0] * AP[0] + AC[1] * AP[1];
    const float DOT_AB_AB = AB[0] * AB[0] + AB[1] * AB[1];
    const float DOT_AC_AC = AC[0] * AC[0] + AC[1] * AC[1];

    const float DET = DOT_AB_AB * DOT_AC_AC - DOT_AB_AC * DOT_AB_AC;
    if (DET < 1e-12f) {
        // Degenerate projection: nearest point among the three edges.
        const auto EDGE = [&](const float U[2], const float V[2]) {
            const float E[2] = {V[0] - U[0], V[1] - U[1]};
            float TT = ((P[0] - U[0]) * E[0] + (P[1] - U[1]) * E[1]) /
                (E[0] * E[0] + E[1] * E[1] + 1e-20f);
            TT = std::clamp(TT, 0.f, 1.f);
            return Vec3{U[0] + E[0] * TT, 0.f, U[1] + E[1] * TT};
        };
        const Vec3 E1 = EDGE(A, B), E2 = EDGE(B, C), E3 = EDGE(C, A);
        const auto D2 = [&](const Vec3& Q) {
            const float DX = Q.x - px, DZ = Q.z - pz;
            return DX * DX + DZ * DZ;
        };
        Vec3 best = E1;
        if (D2(E2) < D2(best))
            best = E2;
        if (D2(E3) < D2(best))
            best = E3;
        return best;
    }

    float S = (DOT_AC_AP * DOT_AB_AC - DOT_AB_AP * DOT_AC_AC) / DET;
    float T = (DOT_AB_AP * DOT_AB_AC - DOT_AC_AP * DOT_AB_AB) / DET;

    if (S < 0.f || T < 0.f || S + T > 1.f) {
        // Outside: nearest of the three edges.
        float bestD = 1e30f;
        Vec3  best = t.a;
        const auto TRY_EDGE = [&](const float U[2], const float V[2]) {
            const float E[2] = {V[0] - U[0], V[1] - U[1]};
            float TT = ((P[0] - U[0]) * E[0] + (P[1] - U[1]) * E[1]) /
                (E[0] * E[0] + E[1] * E[1] + 1e-20f);
            TT = std::clamp(TT, 0.f, 1.f);
            const float QX = U[0] + E[0] * TT, QZ = U[1] + E[1] * TT;
            const float D = (QX - px) * (QX - px) + (QZ - pz) * (QZ - pz);
            if (D < bestD) {
                bestD = D;
                best  = Vec3{QX, 0.f, QZ};
            }
        };
        TRY_EDGE(A, B);
        TRY_EDGE(B, C);
        TRY_EDGE(C, A);
        return best;
    }

    return Vec3{A[0] + AB[0] * S + AC[0] * T, 0.f, A[1] + AB[1] * S + AC[1] * T};
}

// Ericson: closest point on a 3D triangle to p.
static Vec3 closestTri3D(const CMapCollision::STL& t, const Vec3& p) {
    const Vec3 AB = t.b - t.a;
    const Vec3 AC = t.c - t.a;
    const Vec3 AP = p - t.a;

    const float D1 = dot(AB, AP);
    const float D2 = dot(AC, AP);
    if (D1 <= 0.f && D2 <= 0.f)
        return t.a;

    const Vec3 BP = p - t.b;
    const float D3 = dot(AB, BP);
    const float D4 = dot(AC, BP);
    if (D3 >= 0.f && D4 <= D3)
        return t.b;

    const Vec3 CP = p - t.c;
    const float D5 = dot(AB, CP);
    const float D6 = dot(AC, CP);
    if (D6 >= 0.f && D5 <= D6)
        return t.c;

    const float VC = D1 * D4 - D3 * D2;
    if (VC <= 0.f && D1 >= 0.f && D3 <= 0.f) {
        const float V = D1 / (D1 - D3);
        return t.a + AB * V;
    }

    const float VB = D5 * D2 - D1 * D6;
    if (VB <= 0.f && D2 >= 0.f && D6 <= 0.f) {
        const float V = D2 / (D2 - D6);
        return t.a + AC * V;
    }

    const float VA = D3 * D6 - D5 * D4;
    if (VA <= 0.f && (D4 - D3) >= 0.f && (D5 - D6) >= 0.f) {
        const float V = (D4 - D3) / ((D4 - D3) + (D5 - D6));
        return t.b + (t.c - t.b) * V;
    }

    const float DEN = 1.f / (VA + VB + VC);
    const float V = VB * DEN;
    const float W = VC * DEN;
    return t.a + AB * V + AC * W;
}

// Closest points between a (near-vertical) segment and a triangle.
static void segTriClosest(const Vec3& a, const Vec3& b,
                          const CMapCollision::STL& t, Vec3& outSeg,
                          Vec3& outTri) {
    // Seed with the triangle point closest to the vertical line, then
    // refine both ways once. The segment is vertical in this engine, so the
    // 2D seed is near-optimal. The segment point MUST stay on the segment's
    // own axis (x, z from a) -- taking the triangle point's x/z made the
    // pair collapse to zero distance exactly at contact and killed the
    // push-out.
    const float YMIN = std::min(a.y, b.y), YMAX = std::max(a.y, b.y);

    Vec3 P = closestTri2D(t, a.x, a.z);
    P.y = std::clamp(P.y, YMIN, YMAX);

    Vec3 tri = closestTri3D(t, P);
    Vec3 seg{a.x, std::clamp(tri.y, YMIN, YMAX), a.z};
    tri = closestTri3D(t, seg);

    outSeg = seg;
    outTri = tri;
}

} // namespace

// Möller–Trumbore. Two-sided: the map may be entered from either side. This
// is the H3D-scope definition the header's rayTriangles template calls.
bool mapRayTriangle(const Vec3& o, const Vec3& d, const Vec3& a,
                    const Vec3& b, const Vec3& c, float& outT) {
    const Vec3 E1 = b - a;
    const Vec3 E2 = c - a;
    const Vec3 P  = cross(d, E2);
    const float DET = dot(E1, P);

    if (std::fabs(DET) < 1e-9f)
        return false;

    const float INV = 1.0f / DET;
    const Vec3  T   = o - a;
    const float U   = dot(T, P) * INV;

    if (U < 0.f || U > 1.f)
        return false;

    const Vec3 Q = cross(T, E1);
    const float V = dot(d, Q) * INV;

    if (V < 0.f || U + V > 1.f)
        return false;

    const float TIME = dot(E2, Q) * INV;
    if (TIME < 1e-5f)
        return false;

    outT = TIME;
    return true;
}

// --- build ------------------------------------------------------------------

void CMapCollision::buildTree() {
    m_nodes.clear();

    if (m_tris.empty())
        return;

    std::vector<uint32_t> order(m_tris.size());
    std::iota(order.begin(), order.end(), 0u);

    m_nodes.reserve(m_tris.size() / std::max(1u, kLeafSize) * 2 + 16);

    // Recursive by an explicit stack of (begin, end, node index).
    struct SSpan {
        uint32_t begin, end;
        int32_t  node;
    };

    const auto EMIT = [&](uint32_t begin, uint32_t end) {
        SNode n{};
        n.start = begin;
        n.count = end - begin;

        Vec3 lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
        for (uint32_t i = begin; i < end; ++i) {
            const auto& T = m_tris[order[i]];
            const auto TMIN = triMin(T), TMAX = triMax(T);
            lo = {std::min(lo.x, TMIN.x), std::min(lo.y, TMIN.y), std::min(lo.z, TMIN.z)};
            hi = {std::max(hi.x, TMAX.x), std::max(hi.y, TMAX.y), std::max(hi.z, TMAX.z)};
        }
        n.min = lo;
        n.max = hi;
        m_nodes.push_back(n);
        return static_cast<int32_t>(m_nodes.size() - 1);
    };

    std::vector<SSpan> work;
    work.push_back({0, static_cast<uint32_t>(order.size()), -1});

    while (!work.empty()) {
        const auto SPAN = work.back();
        work.pop_back();

        const int32_t NODE = EMIT(SPAN.begin, SPAN.end);

        // Attach to the parent that reserved us, or to the root.
        if (SPAN.node >= 0) {
            if (m_nodes[SPAN.node].left < 0)
                m_nodes[SPAN.node].left = NODE;
            else
                m_nodes[SPAN.node].right = NODE;
        }

        if (SPAN.end - SPAN.begin <= kLeafSize)
            continue;

        const auto& B = m_nodes[NODE];
        const Vec3 EXT{B.max.x - B.min.x, B.max.y - B.min.y, B.max.z - B.min.z};

        // Median split on the longest axis.
        int axis = 0;
        if (EXT.y > EXT.x && EXT.y > EXT.z)
            axis = 1;
        else if (EXT.z > EXT.x && EXT.z > EXT.y)
            axis = 2;

        const uint32_t MID = SPAN.begin + (SPAN.end - SPAN.begin) / 2;

        std::nth_element(order.begin() + SPAN.begin, order.begin() + MID,
                         order.begin() + SPAN.end,
                         [&](uint32_t lhs, uint32_t rhs) {
                             const auto& L = m_tris[lhs];
                             const auto& R = m_tris[rhs];
                             if (axis == 0)
                                 return L.a.x + L.b.x + L.c.x < R.a.x + R.b.x + R.c.x;
                             if (axis == 1)
                                 return L.a.y + L.b.y + L.c.y < R.a.y + R.b.y + R.c.y;
                             return L.a.z + L.b.z + L.c.z < R.a.z + R.b.z + R.c.z;
                         });

        // Reserve the parent slot ordering: children attach by `left` first.
        // We push right first so left is assigned on the later pop... actually
        // both children are emitted after the parent exists, so pass the
        // parent index and let the attach logic fill left/right.
        work.push_back({MID, SPAN.end, NODE});
        work.push_back({SPAN.begin, MID, NODE});
    }

    // The root is node 0 only if it was emitted first -- it was: the first
    // span covered everything with node == -1 and EMIT pushes it first.

    // Leaf ranges index into the PERMUTED triangle order, not into m_tris
    // directly: nth_element shuffles it, so the tree must remember the final
    // permutation. Returning slot indices instead made every query on a
    // non-trivial map hand back wrong triangles -- the map had no collisions
    // at all because of this.
    m_order = std::move(order);
}

void CMapCollision::clear() {
    m_tris.clear();
    m_order.clear();
    m_nodes.clear();
}

// --- queries ----------------------------------------------------------------

void CMapCollision::query(const Vec3& min, const Vec3& max,
                          std::vector<uint32_t>& out) const {
    if (m_nodes.empty())
        return;

    // Explicit stack, root first.
    std::vector<int32_t> stack{0};

    while (!stack.empty()) {
        const auto  N = stack.back();
        stack.pop_back();

        const auto& NODE = m_nodes[N];

        if (NODE.max.x < min.x || NODE.min.x > max.x ||
            NODE.max.y < min.y || NODE.min.y > max.y ||
            NODE.max.z < min.z || NODE.min.z > max.z)
            continue;

        if (NODE.left < 0 && NODE.right < 0) {
            for (uint32_t i = 0; i < NODE.count; ++i)
                out.push_back(m_order[NODE.start + i]);
            continue;
        }

        if (NODE.left >= 0)
            stack.push_back(NODE.left);
        if (NODE.right >= 0)
            stack.push_back(NODE.right);
    }
}

bool CMapCollision::triBoxOverlap(const STL& t, const Vec3& c, const Vec3& h) {
    // Triangle vs AABB: reject on the triangle's own AABB, then the
    // separating-axis suite for the three edge cross products (the box face
    // axes are covered by the AABB test).
    //
    // TOUCHING is not colliding: the player rests exactly on the surface it
    // landed on, and without this bias every horizontal step would be
    // clamped against that surface's own AABB.

    const auto TMIN = triMin(t);
    const auto TMAX = triMax(t);

    if (TMAX.x <= c.x - h.x + EPS || TMIN.x >= c.x + h.x - EPS ||
        TMAX.y <= c.y - h.y + EPS || TMIN.y >= c.y + h.y - EPS ||
        TMAX.z <= c.z - h.z + EPS || TMIN.z >= c.z + h.z - EPS)
        return false;

    const Vec3 E1 = t.b - t.a;
    const Vec3 E2 = t.c - t.b;
    const Vec3 E3 = t.a - t.c;

    const Vec3 AXES[] = {
        {0.f, -E1.z, E1.y},
        {0.f, -E2.z, E2.y},
        {0.f, -E3.z, E3.y},
        {E1.z, 0.f, -E1.x},
        {E2.z, 0.f, -E2.x},
        {E3.z, 0.f, -E3.x},
        {-E1.y, E1.x, 0.f},
        {-E2.y, E2.x, 0.f},
        {-E3.y, E3.x, 0.f},
    };

    for (const auto& A : AXES) {
        const float LEN = std::sqrt(dot(A, A));
        if (LEN < 1e-9f)
            continue;

        // Project the triangle and the box onto the axis; separate when the
        // intervals miss. Same touch tolerance as above.
        const float T0 = dot(t.a, A), T1 = dot(t.b, A), T2 = dot(t.c, A);
        const float TRI_LO = std::min({T0, T1, T2});
        const float TRI_HI = std::max({T0, T1, T2});

        const float R = h.x * std::fabs(A.x) + h.y * std::fabs(A.y) +
            h.z * std::fabs(A.z);
        const float CNT = dot(c, A);

        if (TRI_LO >= CNT + R - EPS || TRI_HI <= CNT - R + EPS)
            return false;
    }

    return true;
}

bool CMapCollision::moveAABB(Vec3& position, const Vec3& delta,
                             const Vec3& half) const {
    std::vector<const CMapCollision*> self{this};
    bool grounded = false;
    moveAABBOn(self, position, delta, half, grounded);
    return grounded;
}

void CMapCollision::moveAABBOn(const std::vector<const CMapCollision*>& trees,
                               Vec3& position, const Vec3& delta,
                               const Vec3& half, bool& groundedOut) {
    bool anyTris = false;
    for (const auto* T : trees)
        if (T && !T->m_tris.empty())
            anyTris = true;

    if (!anyTris)
        return;

    bool grounded = false;

    // (tree, triangle) candidates gathered from every tree per query.
    std::vector<std::pair<const CMapCollision*, uint32_t>> near_;

    // Auto step-up: thresholds, floor seams and small lips must not stop the
    // walk. When a horizontal move was blocked, the whole move is retried
    // from STEP above and settled back down; it is accepted only when it
    // actually gains horizontal progress and lands on ground.
    constexpr float STEP = 0.35f;

    const auto RESOLVE = [&](Vec3& pos, int axis, float d) -> bool {
        if (d == 0.f)
            return false;

        bool clamped = false;

        const float BEFORE = (&pos.x)[axis];
        (&pos.x)[axis] += d;

        // Query region: the box swept along this one axis, i.e. the union of
        // the box at the old and the new position -- not just the new box,
        // or fast moves would skip the triangles in between.
        Vec3 lo{pos.x - half.x, pos.y - half.y, pos.z - half.z};
        Vec3 hi{pos.x + half.x, pos.y + half.y, pos.z + half.z};

        (&lo.x)[axis] = std::min((&lo.x)[axis], BEFORE - (&half.x)[axis]);
        (&hi.x)[axis] = std::max((&hi.x)[axis], BEFORE + (&half.x)[axis]);

        near_.clear();
        for (const auto* T : trees) {
            std::vector<uint32_t> local;
            T->query(lo, hi, local);
            for (const uint32_t I : local)
                near_.push_back({T, I});
        }

        const float AFTER = (&pos.x)[axis];
        const Vec3  CENTER{pos.x, pos.y, pos.z};

        for (const auto& [TREE, I] : near_) {
            const auto& T = TREE->m_tris[I];
            const auto TMIN = triMin(T);
            const auto TMAX = triMax(T);

            // The triangle can only block motion along `axis` if the box
            // still overlaps it on the two OTHER axes (touch tolerance).
            const int A1 = (axis + 1) % 3, A2 = (axis + 2) % 3;
            const bool SIDE =
                ((&TMAX.x)[A1] > (&CENTER.x)[A1] - (&half.x)[A1] - EPS &&
                 (&TMIN.x)[A1] < (&CENTER.x)[A1] + (&half.x)[A1] + EPS &&
                 (&TMAX.x)[A2] > (&CENTER.x)[A2] - (&half.x)[A2] - EPS &&
                 (&TMIN.x)[A2] < (&CENTER.x)[A2] + (&half.x)[A2] + EPS);

            if (!SIDE)
                continue;

            // And its own range on the moved axis must sit between where the
            // box face started and where it ended (a swept-face contact), or
            // the final box must already penetrate it (spawning inside).
            const bool CROSS =
                d > 0.f
                    ? ((&TMIN.x)[axis] >= BEFORE + (&half.x)[axis] - EPS &&
                       (&TMIN.x)[axis] <= AFTER + (&half.x)[axis])
                    : ((&TMAX.x)[axis] <= BEFORE - (&half.x)[axis] + EPS &&
                       (&TMAX.x)[axis] >= AFTER - (&half.x)[axis]);

            if (!CROSS && !triBoxOverlap(T, CENTER, half))
                continue;

            // Clamp along the moved axis until the box touches the
            // triangle's own AABB (exact for axis-aligned maps, conservative
            // elsewhere -- good enough for walking).
            if (d > 0.f)
                (&pos.x)[axis] =
                    std::min(AFTER, (&TMIN.x)[axis] - (&half.x)[axis]);
            else
                (&pos.x)[axis] =
                    std::max(AFTER, (&TMAX.x)[axis] + (&half.x)[axis]);

            if (axis == 1 && d < 0.f)
                grounded = true;

            clamped = true;
        }

        return clamped;
    };

    const Vec3   START = position;
    const bool BLOCKED_X = RESOLVE(position, 0, delta.x);
    const bool BLOCKED_Z = RESOLVE(position, 2, delta.z);
    const bool Y_GROUNDED = RESOLVE(position, 1, delta.y);
    grounded = Y_GROUNDED;

    // Step-up retry: the same horizontal move from STEP above, then settle.
    // The probe's own downward settle must NOT leak its grounded flag into
    // the result when the step is rejected.
    if (BLOCKED_X || BLOCKED_Z) {
        Vec3 stepped{START.x, START.y + STEP, START.z};
        RESOLVE(stepped, 0, delta.x);
        RESOLVE(stepped, 2, delta.z);
        const bool LANDED = RESOLVE(stepped, 1, -(STEP + 0.02f));

        const float PROG_BASE = std::fabs(position.x - START.x) +
            std::fabs(position.z - START.z);
        const float PROG_STEP = std::fabs(stepped.x - START.x) +
            std::fabs(stepped.z - START.z);

        if (LANDED && PROG_STEP > PROG_BASE + 1e-3f) {
            position = stepped;
            grounded = true;
        } else {
            grounded = Y_GROUNDED; // the probe's settle must not leak
        }
    }

    groundedOut = grounded;
}

bool CMapCollision::moveCapsule(Vec3& feet, const Vec3& delta, float radius,
                                float height, bool* ceiling) const {
    std::vector<const CMapCollision*> self{this};
    return moveCapsuleOn(self, feet, delta, radius, height, ceiling);
}

bool CMapCollision::moveCapsuleOn(
    const std::vector<const CMapCollision*>& trees, Vec3& feet,
    const Vec3& delta, float radius, float height, bool* ceiling) {
    bool anyTris = false;
    for (const auto* T : trees)
        if (T && !T->m_tris.empty())
            anyTris = true;

    if (!anyTris) {
        if (ceiling)
            *ceiling = false;
        return false;
    }

    bool grounded = false;

    // (tree, triangle) candidates gathered from every tree per query.
    std::vector<std::pair<const CMapCollision*, uint32_t>> near_;

    // Auto step-up: thresholds and seams must not stop the walk.
    constexpr float STEP = 0.35f;

    const auto SEG = [&](const Vec3& f, Vec3& a, Vec3& b) {
        a = Vec3{f.x, f.y + radius, f.z};
        b = Vec3{f.x, f.y + height - radius, f.z};
    };

    // Push the capsule out of every nearby triangle, a few iterations so
    // corner contacts settle. `up` = an upward contact grounded the capsule;
    // `down` = a downward contact hit a ceiling.
    const auto DEPENETRATE = [&](Vec3& f, bool& down) -> bool {
        bool up = false;
        down = false;

        for (int it = 0; it < 4; ++it) {
            Vec3 a, b;
            SEG(f, a, b);

            Vec3 lo{std::min(a.x, b.x) - radius, std::min(a.y, b.y) - radius,
                    std::min(a.z, b.z) - radius};
            Vec3 hi{std::max(a.x, b.x) + radius, std::max(a.y, b.y) + radius,
                    std::max(a.z, b.z) + radius};

            near_.clear();
            for (const auto* T : trees) {
                std::vector<uint32_t> local;
                T->query(lo, hi, local);
                for (const uint32_t I : local)
                    near_.push_back({T, I});
            }

            bool pushed = false;
            for (const auto& [TREE, I] : near_) {
                Vec3 qa, qb;
                segTriClosest(a, b, TREE->m_tris[I], qa, qb);

                const Vec3 D = qa - qb;
                const float DIST = std::sqrt(dot(D, D));
                if (DIST >= radius || DIST < 1e-7f)
                    continue;

                const Vec3 N = D * (1.f / DIST);
                f = f + N * (radius - DIST);
                pushed = true;

                if (N.y > 0.5f)
                    up = true;
                if (N.y < -0.5f)
                    down = true;

                SEG(f, a, b); // refresh for the remaining triangles
            }

            if (!pushed)
                break;
        }

        return up;
    };

    const Vec3 START = feet;

    feet += delta;
    bool ceilingHit = false;
    const bool WALK_GROUNDED = DEPENETRATE(feet, ceilingHit);

    // Only the real move may report the bump: the step-up probe's contacts
    // are discarded along with its position.
    if (ceiling)
        *ceiling = ceilingHit;

    // Horizontal progress vs intent: if the push-out ate most of the move,
    // retry the whole move from STEP above and settle back down.
    const float PROG = std::fabs(feet.x - START.x) + std::fabs(feet.z - START.z);
    const float WANT = std::fabs(delta.x) + std::fabs(delta.z);

    if (WANT > 1e-5f && PROG < WANT * 0.5f) {
        Vec3 stepped{START.x, START.y + STEP, START.z};
        stepped += delta;
        bool probeDown = false;
        DEPENETRATE(stepped, probeDown);
        stepped += Vec3{0.f, -(STEP + 0.02f), 0.f};
        const bool LANDED = DEPENETRATE(stepped, probeDown);

        const float PROG_STEP = std::fabs(stepped.x - START.x) +
            std::fabs(stepped.z - START.z);

        if (LANDED && PROG_STEP > PROG + 1e-3f) {
            feet     = stepped;
            grounded = true;
        } else {
            grounded = WALK_GROUNDED;
        }
    } else {
        grounded = WALK_GROUNDED;
    }

    return grounded;
}

float CMapCollision::rayCast(const Vec3& origin, const Vec3& dir) const {
    if (m_tris.empty() || m_nodes.empty())
        return -1.f;

    // Down the tree with a slab test per node (Williams et al., "An Efficient
    // and Robust Ray-Box Intersection Algorithm", 2005), the nearer child
    // first, skipping every node that starts beyond the nearest hit so far.
    // Querying the tree with the ray's bounding box instead took in nearly
    // every triangle of a model one looks at: one aim ray cost ~1 ms a frame
    // at a 47 MB model. Hits stay within the 1000 units that box reached.
    constexpr float kFar = 1000.f;

    // A zero component would give an infinite inverse, and 0 * inf is NaN on
    // a box face; a tiny one keeps every slab distance a number.
    const auto INVERSE = [](float d) {
        return 1.f / (std::fabs(d) > 1e-20f ? d : std::copysign(1e-20f, d));
    };
    const Vec3 INV{INVERSE(dir.x), INVERSE(dir.y), INVERSE(dir.z)};

    // Where the ray enters a node's box, or -1 when it misses it before LIMIT.
    // The exit is widened by pbrt's 1 + 2 * gamma(3) so rounding cannot drop
    // a box the ray only grazes -- a floor's box is flat.
    const auto ENTRY = [&](const SNode& n, float limit) {
        float t0 = 0.f, t1 = limit;
        const auto SLAB = [&](float lo, float hi, float o, float inv) {
            float a = (lo - o) * inv, b = (hi - o) * inv;
            if (a > b)
                std::swap(a, b);
            t0 = std::max(t0, a);
            t1 = std::min(t1, b * 1.0000004f);
        };
        SLAB(n.min.x, n.max.x, origin.x, INV.x);
        SLAB(n.min.y, n.max.y, origin.y, INV.y);
        SLAB(n.min.z, n.max.z, origin.z, INV.z);
        return t0 <= t1 ? t0 : -1.f;
    };

    float best  = kFar;
    bool  found = false;

    struct SVisit {
        int32_t node;
        float   entry;
    };
    std::vector<SVisit> stack;
    stack.reserve(64);

    if (const float T = ENTRY(m_nodes[0], best); T >= 0.f)
        stack.push_back({0, T});

    while (!stack.empty()) {
        const auto VISIT = stack.back();
        stack.pop_back();

        if (VISIT.entry > best)
            continue;

        const auto& NODE = m_nodes[VISIT.node];

        if (NODE.left < 0 && NODE.right < 0) {
            for (uint32_t i = 0; i < NODE.count; ++i) {
                float t = -1.f;
                if (rayTriangle(origin, dir, m_tris[m_order[NODE.start + i]], t) &&
                    t <= best) {
                    best  = t;
                    found = true;
                }
            }
            continue;
        }

        const float TL = NODE.left >= 0 ? ENTRY(m_nodes[NODE.left], best) : -1.f;
        const float TR = NODE.right >= 0 ? ENTRY(m_nodes[NODE.right], best) : -1.f;

        // The nearer child goes on top, so it is walked first.
        if (TL >= 0.f && TR >= 0.f) {
            const bool LEFT_FIRST = TL <= TR;
            stack.push_back({LEFT_FIRST ? NODE.right : NODE.left, LEFT_FIRST ? TR : TL});
            stack.push_back({LEFT_FIRST ? NODE.left : NODE.right, LEFT_FIRST ? TL : TR});
        } else if (TL >= 0.f) {
            stack.push_back({NODE.left, TL});
        } else if (TR >= 0.f) {
            stack.push_back({NODE.right, TR});
        }
    }

    return found ? best : -1.f;
}

} // namespace H3D

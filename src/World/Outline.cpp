#include "World/Outline.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace H3D {

namespace {

struct SPt {
    float x = 0.f, y = 0.f;
};

// Directed unit step along a cell boundary, in corner-grid coordinates
// (0..w, 0..h). The enumeration below emits every solid/empty adjacency
// exactly once, with the solid side consistently on the direction's right
// (in mask coordinates, x right / y down).
struct SEdge {
    int fx = 0, fy = 0;
    int tx = 0, ty = 0;
};

bool solidAt(const unsigned char* mask, int w, int h, int x, int y,
             unsigned char threshold) {
    if (x < 0 || y < 0 || x >= w || y >= h)
        return false;
    return mask[static_cast<size_t>(y) * w + x] >= threshold;
}

// Perpendicular distance from p to the segment a-b.
float segDistance(const SPt& p, const SPt& a, const SPt& b) {
    const float DX = b.x - a.x, DY = b.y - a.y;
    const float LEN2 = DX * DX + DY * DY;

    if (LEN2 < 1e-12f)
        return std::hypot(p.x - a.x, p.y - a.y);

    float t = ((p.x - a.x) * DX + (p.y - a.y) * DY) / LEN2;
    t = std::max(0.0f, std::min(1.0f, t));

    return std::hypot(p.x - (a.x + DX * t), p.y - (a.y + DY * t));
}

// Ramer-Douglas-Peucker on the open sub-polyline P[first..last]; keeps
// endpoints, marks survivors in `keep`.
void rdp(const std::vector<SPt>& P, size_t first, size_t last, float eps,
         std::vector<bool>& keep) {
    if (last <= first + 1)
        return;

    float best = -1.0f;
    size_t bestIdx = first;

    for (size_t i = first + 1; i < last; ++i) {
        const float D = segDistance(P[i], P[first], P[last]);
        if (D > best) {
            best = D;
            bestIdx = i;
        }
    }

    if (best <= eps) {
        return; // whole run collapses onto the segment
    }

    keep[bestIdx] = true;
    rdp(P, first, bestIdx, eps, keep);
    rdp(P, bestIdx, last, eps, keep);
}

// Closed-loop simplify: split the loop at the point farthest from the start
// and RDP both halves. Returns a point count <= maxPoints, going coarser
// only as far as the loop's shape allows.
std::vector<SPt> simplifyLoop(const std::vector<SPt>& loop, int maxPoints) {
    if (static_cast<int>(loop.size()) <= maxPoints || loop.size() < 3)
        return loop;

    // Farthest point from loop[0] splits the closed curve into two open
    // runs whose endpoints survive RDP, so the loop stays closed.
    float best = -1.0f;
    size_t split = 1;
    for (size_t i = 1; i < loop.size(); ++i) {
        const float D = std::hypot(loop[i].x - loop[0].x, loop[i].y - loop[0].y);
        if (D > best) {
            best = D;
            split = i;
        }
    }

    std::vector<SPt> seq = loop;
    seq.push_back(loop[0]); // close the curve for the second run

    float eps = 1.0f;
    std::vector<SPt> result;

    for (int pass = 0; pass < 8; ++pass) {
        std::vector<bool> keep(seq.size(), false);
        keep[0] = keep[split] = keep[seq.size() - 1] = true;

        rdp(seq, 0, split, eps, keep);
        rdp(seq, split, seq.size() - 1, eps, keep);

        result.clear();
        for (size_t i = 0; i + 1 < seq.size(); ++i)
            if (keep[i])
                result.push_back(seq[i]);

        if (static_cast<int>(result.size()) <= maxPoints)
            break;

        eps *= 1.5f;
    }

    return result;
}

} // namespace

std::vector<SOutlineLoop> traceOutlines(
    const unsigned char* mask, int w, int h, unsigned char threshold,
    float insetTexels, int maxLoops, int maxPoints) {

    std::vector<SOutlineLoop> out;
    if (!mask || w <= 0 || h <= 0)
        return out;

    maxLoops  = std::max(1, maxLoops);
    maxPoints = std::max(8, maxPoints);

    // --- boundary edges ----------------------------------------------------
    std::vector<SEdge> edges;
    edges.reserve(64);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (!solidAt(mask, w, h, x, y, threshold))
                continue;

            if (!solidAt(mask, w, h, x, y - 1, threshold))
                edges.push_back({x, y, x + 1, y});
            if (!solidAt(mask, w, h, x + 1, y, threshold))
                edges.push_back({x + 1, y, x + 1, y + 1});
            if (!solidAt(mask, w, h, x, y + 1, threshold))
                edges.push_back({x + 1, y + 1, x, y + 1});
            if (!solidAt(mask, w, h, x - 1, y, threshold))
                edges.push_back({x, y + 1, x, y});
        }
    }

    if (edges.empty())
        return out;

    // Outgoing edges indexed by their start corner.
    std::unordered_map<uint64_t, std::vector<size_t>> outgoing;
    outgoing.reserve(edges.size() * 2);

    const auto CORNER = [&](int x, int y) -> uint64_t {
        return static_cast<uint64_t>(y) * static_cast<uint64_t>(w + 1) +
               static_cast<uint64_t>(x);
    };

    for (size_t i = 0; i < edges.size(); ++i)
        outgoing[CORNER(edges[i].fx, edges[i].fy)].push_back(i);

    // --- chain edges into loops --------------------------------------------
    std::vector<bool> used(edges.size(), false);

    for (size_t seed = 0; seed < edges.size(); ++seed) {
        if (used[seed])
            continue;

        std::vector<SPt> loop;
        loop.reserve(16);

        size_t cur = seed;
        const int SX = edges[seed].fx, SY = edges[seed].fy;

        bool ok = true;
        while (true) {
            used[cur] = true;
            loop.push_back(SPt{float(edges[cur].fx), float(edges[cur].fy)});

            if (edges[cur].tx == SX && edges[cur].ty == SY)
                break; // closed

            // Next edge: straight continuation, or at a pinch (diagonally
            // touching blobs share a corner) the sharpest RIGHT turn, which
            // keeps the walk hugging the blob it started on instead of
            // jumping across the diagonal to the neighbour.
            const int DX = edges[cur].tx - edges[cur].fx;
            const int DY = edges[cur].ty - edges[cur].fy;

            const auto& CANDS = outgoing[CORNER(edges[cur].tx, edges[cur].ty)];
            int next = -1;
            int nextRank = -4; // cross in [-1..1], dot breaks ties

            for (const size_t C : CANDS) {
                if (used[C])
                    continue;

                const int CX = edges[C].tx - edges[C].fx;
                const int CY = edges[C].ty - edges[C].fy;

                // y-down screen coords: cross > 0 is a right turn.
                const int CROSS = DX * CY - DY * CX;
                const int DOT   = DX * CX + DY * CY;
                const int RANK  = CROSS * 2 + DOT;

                if (RANK > nextRank) {
                    nextRank = RANK;
                    next     = static_cast<int>(C);
                }
            }

            if (next < 0 || loop.size() > edges.size() + 1) {
                ok = false; // disconnected fragment; drop it
                break;
            }

            cur = static_cast<size_t>(next);
        }

        if (!ok || loop.size() < 3)
            continue;

        // --- merge collinear runs ------------------------------------------
        // Keep a point unless its predecessor and successor continue in a
        // perfectly straight line through it (exact for the integer corner
        // grid). A straight run collapses to its two endpoints.
        std::vector<SPt> merged;
        merged.reserve(loop.size());

        const size_t N = loop.size();
        for (size_t i = 0; i < N; ++i) {
            const SPt& A = loop[(i + N - 1) % N];
            const SPt& B = loop[i];
            const SPt& C = loop[(i + 1) % N];

            const bool COLLINEAR =
                (B.x - A.x) * (C.y - B.y) - (B.y - A.y) * (C.x - B.x) == 0.f;

            if (!COLLINEAR)
                merged.push_back(B);
        }

        if (merged.size() < 3)
            continue;

        // --- simplify to maxPoints ------------------------------------------
        std::vector<SPt> simplified = simplifyLoop(merged, maxPoints);
        if (simplified.size() < 3)
            continue;

        // --- drop slivers, then emit ----------------------------------------
        float area2 = 0.0f;
        for (size_t i = 0; i < simplified.size(); ++i) {
            const SPt& A = simplified[i];
            const SPt& B = simplified[(i + 1) % simplified.size()];
            area2 += A.x * B.y - B.x * A.y;
        }

        // A unit cell has shoelace area2 = 2; anything smaller is a
        // degenerate zero-area fragment of the walk, not a blob.
        if (std::fabs(area2) < 1.9f)
            continue;

        SOutlineLoop OUT;
        OUT.pts.reserve(simplified.size());
        OUT.uvs.reserve(simplified.size());

        SPt centroid{};
        for (const SPt& P : simplified) {
            centroid.x += P.x;
            centroid.y += P.y;
        }
        centroid.x /= static_cast<float>(simplified.size());
        centroid.y /= static_cast<float>(simplified.size());

        for (const SPt& P : simplified) {
            OUT.pts.push_back({P.x / static_cast<float>(w),
                               P.y / static_cast<float>(h)});

            // Sampling point: step toward the centroid in CELL space (the
            // normalized box is anisotropic), clamped so it never crosses
            // the centroid on thin shapes.
            const float DX = centroid.x - P.x;
            const float DY = centroid.y - P.y;
            const float LEN = std::hypot(DX, DY);

            SPt q = P;
            if (LEN > 1e-4f) {
                const float STEP =
                    std::min(insetTexels, LEN * 0.5f) / LEN;
                q = SPt{P.x + DX * STEP, P.y + DY * STEP};
            }

            OUT.uvs.push_back({q.x / static_cast<float>(w),
                               q.y / static_cast<float>(h)});
        }

        out.push_back(std::move(OUT));
    }

    // Largest first, then cap.
    const auto AREA = [](const SOutlineLoop& L) {
        float A = 0.0f;
        for (size_t i = 0; i < L.pts.size(); ++i) {
            const Vec2& P = L.pts[i];
            const Vec2& Q = L.pts[(i + 1) % L.pts.size()];
            A += P.x * Q.y - Q.x * P.y;
        }
        return std::fabs(A);
    };

    std::stable_sort(out.begin(), out.end(),
                     [&AREA](const SOutlineLoop& A, const SOutlineLoop& B) {
                         return AREA(A) > AREA(B);
                     });

    if (static_cast<int>(out.size()) > maxLoops)
        out.resize(static_cast<size_t>(maxLoops));

    return out;
}

SOutlineLoop roundedRectLoop(int wPx, int hPx, float radiusPx,
                             float insetPx, int arcSegments) {
    SOutlineLoop OUT;
    if (wPx <= 0 || hPx <= 0)
        return OUT;

    const float FW = static_cast<float>(wPx);
    const float FH = static_cast<float>(hPx);
    const float R  = std::clamp(radiusPx, 0.0f, std::min(FW, FH) * 0.5f);
    const int   SEG = std::max(2, arcSegments);

    std::vector<SPt> pts;
    pts.reserve(static_cast<size_t>(SEG) * 4 + 8);

    const auto ARC = [&pts](float cx, float cy, float a0, float a1, float r,
                            int seg) {
        const float RAD = 3.14159265f / 180.0f;
        for (int i = 0; i <= seg; ++i) {
            const float A = (a0 + (a1 - a0) * static_cast<float>(i) / seg) * RAD;
            pts.push_back({cx + r * std::cos(A), cy + r * std::sin(A)});
        }
    };

    if (R < 0.5f) {
        pts.push_back({0.f, 0.f});
        pts.push_back({FW, 0.f});
        pts.push_back({FW, FH});
        pts.push_back({0.f, FH});
    } else {
        // Clockwise in y-down screen coords, starting on the top edge at
        // (R, 0): top -> TR arc -> right -> BR arc -> bottom -> BL arc ->
        // left -> TL arc, closing at the start.
        ARC(FW - R, R,     270.f, 360.f, R, SEG); // top-right corner
        ARC(FW - R, FH - R,  0.f,  90.f, R, SEG); // bottom-right corner
        ARC(R,      FH - R, 90.f, 180.f, R, SEG); // bottom-left corner
        ARC(R,      R,     180.f, 270.f, R, SEG); // top-left corner
    }

    // Drop consecutive duplicates (arc endpoints meet at the box sides).
    std::vector<SPt> clean;
    clean.reserve(pts.size());
    for (const SPt& P : pts) {
        if (!clean.empty() && std::fabs(clean.back().x - P.x) < 1e-4f &&
            std::fabs(clean.back().y - P.y) < 1e-4f)
            continue;
        clean.push_back(P);
    }
    while (clean.size() > 1 && std::fabs(clean.front().x - clean.back().x) < 1e-4f &&
           std::fabs(clean.front().y - clean.back().y) < 1e-4f)
        clean.pop_back();

    if (clean.size() < 3)
        return OUT;

    const SPt CENTER{FW * 0.5f, FH * 0.5f};

    OUT.pts.reserve(clean.size());
    OUT.uvs.reserve(clean.size());

    for (const SPt& P : clean) {
        OUT.pts.push_back({P.x / FW, P.y / FH});

        // Sampling point: `insetPx` toward the box center, clamped so it
        // never crosses the middle of thin shapes.
        const float DX = CENTER.x - P.x;
        const float DY = CENTER.y - P.y;
        const float LEN = std::hypot(DX, DY);

        SPt q = P;
        if (LEN > 1e-4f) {
            const float STEP = std::min(insetPx, LEN * 0.5f) / LEN;
            q = SPt{P.x + DX * STEP, P.y + DY * STEP};
        }

        OUT.uvs.push_back({q.x / FW, q.y / FH});
    }

    return OUT;
}

} // namespace H3D

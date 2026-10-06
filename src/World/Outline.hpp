#pragma once

#include "World/Camera.hpp"

#include <vector>

namespace H3D {

// One closed silhouette loop of a window snapshot's alpha mask, in the
// window box's normalized coordinates: (0,0) is the box's top-left corner,
// (1,1) its bottom-right (row 0 of the mask = the top row).
//
// `uvs` mirrors `pts` index-for-index and is the same point pulled slightly
// toward the loop's interior: the silhouette texels sit on the alpha
// antialiasing ramp (their RGB is darkened by the blend against the
// transparent framebuffer), so walls sample from `uvs` to get solid edge
// colors while the geometry itself follows `pts`.
struct SOutlineLoop {
    std::vector<Vec2> pts;
    std::vector<Vec2> uvs;
};

// Traces the closed outlines of a binary mask (alpha >= threshold is solid).
// `mask` is w*h bytes, row 0 = top. Diagonally touching blobs stay separate
// loops. Returns at most maxLoops loops, largest area first, each simplified
// to at most maxPoints points; `insetTexels` sets the interior UV inset in
// mask texels. Pure geometry -- no GL, no Hyprland, unit-testable.
std::vector<SOutlineLoop> traceOutlines(
    const unsigned char* mask, int w, int h, unsigned char threshold,
    float insetTexels, int maxLoops, int maxPoints);

// Analytic rounded-rectangle silhouette in a wPx x hPx box (origin
// top-left, y down): the shape Hyprland's rounding shader leaves visible on
// a decorated window. Corners are quarter arcs of `radiusPx` (clamped to
// the box) sampled `arcSegments` times each; straight edges collapse to
// their endpoints. The snapshot framebuffer's alpha channel cannot be read
// back reliably (GPU-sampled it is opaque, glReadPixels returns garbage),
// so window slabs derive their silhouette from the window geometry
// instead. `uvs` mirror the points, pulled `insetPx` toward the box center
// so walls sample solid border texels instead of the antialiased fringe.
// Returns an empty loop for degenerate boxes.
SOutlineLoop roundedRectLoop(int wPx, int hPx, float radiusPx,
                             float insetPx, int arcSegments);

} // namespace H3D

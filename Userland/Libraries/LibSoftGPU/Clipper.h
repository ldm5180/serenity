/*
 * Copyright (c) 2021, Jesse Buhagiar <jooster669@gmail.com>
 * Copyright (c) 2022, the SerenityOS developers.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <AK/Vector.h>
#include <LibGfx/Vector4.h>
#include <LibSoftGPU/Vertex.h>

namespace SoftGPU {

enum class ClipPlane : u8 {
    LEFT = 0,
    RIGHT,
    TOP,
    BOTTOM,
    NEAR,
    FAR
};

template<ClipPlane plane>
static constexpr bool point_within_clip_plane(FloatVector4 const& vertex)
{
    if constexpr (plane == ClipPlane::LEFT) {
        return vertex.x() >= -vertex.w();
    } else if constexpr (plane == ClipPlane::RIGHT) {
        return vertex.x() <= vertex.w();
    } else if constexpr (plane == ClipPlane::TOP) {
        return vertex.y() <= vertex.w();
    } else if constexpr (plane == ClipPlane::BOTTOM) {
        return vertex.y() >= -vertex.w();
    } else if constexpr (plane == ClipPlane::NEAR) {
        return vertex.z() >= -vertex.w();
    } else if constexpr (plane == ClipPlane::FAR) {
        return vertex.z() <= vertex.w();
    }
    return false;
}

template<ClipPlane plane>
static constexpr Vertex clip_intersection_point(Vertex const& p1, Vertex const& p2)
{
    constexpr FloatVector4 clip_plane_normals[] = {
        { 1, 0, 0, 0 },  // Left Plane
        { -1, 0, 0, 0 }, // Right Plane
        { 0, -1, 0, 0 }, // Top Plane
        { 0, 1, 0, 0 },  // Bottom plane
        { 0, 0, 1, 0 },  // Near Plane
        { 0, 0, -1, 0 }  // Far Plane
    };
    constexpr auto clip_plane_normal = clip_plane_normals[to_underlying(plane)];

    // See https://www.microsoft.com/en-us/research/wp-content/uploads/1978/01/p245-blinn.pdf
    // "Clipping Using Homogeneous Coordinates" Blinn/Newell, 1978

    float const w1 = p1.clip_coordinates.w();
    float const w2 = p2.clip_coordinates.w();
    float const x1 = clip_plane_normal.dot(p1.clip_coordinates);
    float const x2 = clip_plane_normal.dot(p2.clip_coordinates);
    float const a = (w1 + x1) / ((w1 + x1) - (w2 + x2));

    Vertex out;
    out.position = mix(p1.position, p2.position, a);
    out.eye_coordinates = mix(p1.eye_coordinates, p2.eye_coordinates, a);
    out.clip_coordinates = mix(p1.clip_coordinates, p2.clip_coordinates, a);
    out.color = mix(p1.color, p2.color, a);
    for (size_t i = 0; i < NUM_SAMPLERS; ++i)
        out.tex_coords[i] = mix(p1.tex_coords[i], p2.tex_coords[i], a);
    out.normal = mix(p1.normal, p2.normal, a);
    return out;
}

template<ClipPlane plane>
static constexpr void clip_plane(Vector<Vertex>& read_list, Vector<Vertex>& write_list)
{
    auto read_from = &read_list;
    auto write_to = &write_list;

    write_to->clear_with_capacity();
    for (size_t i = 0; i < read_from->size(); i++) {
        auto const& curr_vec = read_from->at((i + 1) % read_from->size());
        auto const& prev_vec = read_from->at(i);

        bool const is_curr_point_within_clip_plane = point_within_clip_plane<plane>(curr_vec.clip_coordinates);
        bool const is_prev_point_within_clip_plane = point_within_clip_plane<plane>(prev_vec.clip_coordinates);
        if (is_curr_point_within_clip_plane != is_prev_point_within_clip_plane) {
            auto const intersect = clip_intersection_point<plane>(prev_vec, curr_vec);
            write_to->append(intersect);
        }

        if (is_curr_point_within_clip_plane)
            write_to->append(curr_vec);
    }
    swap(write_list, read_list);
}

class Clipper final {
public:
    Clipper() = default;

    void clip_triangle_against_frustum(Vector<Vertex>& input_vecs)
    {
        list_a = input_vecs;
        list_b.clear_with_capacity();

        // FIXME C++23. Static reflection will provide looping over all enum values.
        clip_plane<ClipPlane::LEFT>(list_a, list_b);
        clip_plane<ClipPlane::RIGHT>(list_a, list_b);
        clip_plane<ClipPlane::TOP>(list_a, list_b);
        clip_plane<ClipPlane::BOTTOM>(list_a, list_b);
        clip_plane<ClipPlane::NEAR>(list_a, list_b);
        clip_plane<ClipPlane::FAR>(list_a, list_b);

        input_vecs = list_a;
    }

private:
    Vector<Vertex> list_a;
    Vector<Vertex> list_b;
};

}

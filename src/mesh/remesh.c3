module collision;

import std::io;
import std::math;
import std::collections::list;

const float WELD_EPSILON @local = 0.005;

fn bool vertices_equal(Vec3 a, Vec3 b) @local => a.distance_sq(b) < WELD_EPSILON;

<*
 Welds duplicate vertices and removes degenerate triangles.
 @require !vertices.is_empty() && !triangles.is_empty() : `Vertices and Triangles must not be empty`
*>
fn void weld_remesh(List{Vec3}* vertices, List{Triangle}* triangles) {
    List{Vec3} new_vertices;
    List{Triangle} new_triangles;

    defer {
        new_vertices.free();
        new_triangles.free();
    }

    // Build index remap table: old vertex index -> new vertex index
    List{uint} index_remap;
    defer index_remap.free();

    foreach (uint i, point: vertices) {
        bool found_duplicate = false;
        uint new_index = 0;

        // Check if this vertex matches any already added vertex
        foreach (uint j, new_point: new_vertices) {
            if (vertices_equal(point, new_point)) {
                found_duplicate = true;
                new_index = j;
                break;
            }
        }

        if (!found_duplicate) {
            // Unique vertex - add it
            new_index = (uint)new_vertices.len();
            new_vertices.push(point);
        }

        // Store the mapping for this vertex
        index_remap.push(new_index);
    }

    // Remap triangles and filter degenerate ones
    foreach (tri: triangles) {
        uint i0 = index_remap[tri[0]];
        uint i1 = index_remap[tri[1]];
        uint i2 = index_remap[tri[2]];

        // Skip degenerate triangles (where 2+ vertices merged)
        if (i0 != i1 && i0 != i2 && i1 != i2) {
            new_triangles.push({i0, i1, i2});
        }
    }

    // Replace original data
    vertices.clear();
    triangles.clear();

    foreach (v: new_vertices) vertices.push(v);
    foreach (t: new_triangles) triangles.push(t);
}

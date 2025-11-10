module collision;

import std::io;
import std::math;
import std::collections::list;

const Number EPSILON @local = 0.0001;

alias Vertices @local = List{Vec4};
alias Indices @local = List{usz};


fn bool are_equal(Vec3 a, Vec3 b) => a.distance_sq(b) < EPSILON * EPSILON;

<*
 @require !vertices.is_empty() && !indices.is_empty() : `Vertices and Indices must not be empty`
*>
fn void weld_remesh(Vertices* vertices, Indices* indices) {

    Vertices new_vertices;
    Indices new_indices;
    Indices vertex_map;

    vertex_map.init(mem, vertices.len());
    vertex_map.set_size(vertices.len());

    defer {
        new_vertices.free();
        vertex_map.free();
        new_indices.free();
    }

    foreach (int i, point: vertices) {
        bool found_duplicate = false;
        int new_index = -1;

        foreach (int j, new_point: new_vertices) {
            if (are_equal(point.xyz, new_point.xyz)) {
                found_duplicate = true;
                new_index = j;
                break;
            }
        }

        if (found_duplicate == false) {
            // This is a unique vertex. Add it to the new list.
            new_vertices.push(point);
            new_index = (int)new_vertices.len(); // Its new index is the current count
        }

        vertex_map.insert_at(i, new_index);
    }

    for (usz i = 0; i < indices.len(); i+= 3) {
        usz n0 = vertex_map[i];
        usz n1 = vertex_map[i+1];
        usz n2 = vertex_map[i+2];

        // Check for degenerate faces.
        // A face is degenerate if two or more of its vertices
        // were merged into the same new vertex.
        if (n0 != n1 && n0 != n2 && n1 != n2) {
            // This is a valid, non-degenerate triangle. Add it.
           new_indices.push_all({n0, n1, n2});
        }
    }

    vertices.clear();
    indices.clear();

    vertices.add_all(&new_vertices);
    indices.add_all(&new_indices);
}
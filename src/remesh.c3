module collision;

import std::io;
import std::math;
import std::collections::list;

const Number EPSILON @local = 0.005;

alias Vertices @local = List{Vec3};
alias Indices @local = List{ushort};


fn bool are_equal(Vec3 a, Vec3 b) => a.distance_sq(b) < EPSILON;

<*
 @require !vertices.is_empty() && !indices.is_empty() : `Vertices and Indices must not be empty`
*>
fn void weld_remesh(Vertices* vertices, Indices* indices) {

    Vertices new_vertices;
    Indices new_indices;

    defer {
        new_vertices.free();
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
            new_index = (int)new_vertices.len() - 1; // Its new index is the current count

            foreach (usz j, idx: indices) {
                if (idx == i) indices.set_at(j, (ushort)new_index);
            }
        }
    }


    usz max_index = new_vertices.len() - 1;
    for (usz i = 0; i < indices.len(); i+= 3) {
        ushort i1 = indices.get(i);
        ushort i2 = indices.get(i + 1);
        ushort i3 = indices.get(i + 2);

        if (i1 > max_index || i2 > max_index || i3 > max_index) continue;
        // Check for degenerate faces.
        // A face is degenerate if two or more of its vertices
        // were merged into the same new vertex.

        new_indices.push_all({i1, i2, i3});
        // if (i1 != i2 && i1 != i3 && i2 != i3) {
        //     // This is a valid, non-degenerate triangle. Add it.
        //    new_indices.push_all({i1, i2, i3});
        // }
    }

    vertices.clear();
    indices.clear();

    vertices.add_all(&new_vertices);
    indices.add_all(&new_indices);
}
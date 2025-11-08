module collision;

import std::io;
import std::math;
import std::collections::list;

alias Vertex @local = int[<3>];
alias Edge @local = int[<2>];
alias IndexList @local = List{usz};
alias EdgeList @local = List{Edge};
alias Points @local = Vec3[];

const Number EPSILON @local = 0.0001;

struct QuickHull {
    Points points;
    List{HullFace} faces;
    List{Vec3} result;
}

struct HullFace @local {
    Vertex v;
    bool visible;
    Number offset;
    bool visited;
    Vec3 normal;
    IndexList outside;
}

<*
 @require points.len > 4 : `Hull input size must be > 4`
*>
fn QuickHull hull_from_points(Points points) {
    
    QuickHull hull = {
        .points = points
    };

    // Initial tetrahedon
    int[6] extremes;

    int p1_idx = -1;
    int p2_idx = -1;
    int p3_idx = -1;
    int p4_idx = -1;
    
    // 497,498,175,103
    foreach (int i, point: points) {
        if (point.x > points[extremes[0]].x) extremes[0] = i;
        if (point.x < points[extremes[1]].x) extremes[1] = i;
        if (point.y > points[extremes[2]].y) extremes[2] = i;
        if (point.y < points[extremes[3]].y) extremes[3] = i;
        if (point.z > points[extremes[4]].z) extremes[4] = i;
        if (point.z < points[extremes[5]].z) extremes[5] = i;
    }

    Number max_dist = 0;

    for (int i = 0;i < extremes.len ;i++) {
        for (int j = i + 1;i < extremes.len ;i++) {
            Vec3 min = extremes[i];
            Vec3 max = extremes[j];

            Number dist = (Number)(points[extremes[i]] - points[extremes[j]]).sq_magnitude();

            if (dist > max_dist) {
                max_dist = dist;
                p1_idx = extremes[i];
                p2_idx = extremes[j];
            }
        }
    }

    assert(max_dist > EPSILON, "All points are coincident.");

    max_dist = 0;

    Vec3 line_dir = (points[p2_idx] - points[p1_idx]).normalize();
    foreach (int i, point: points) {
        if (i == p1_idx || i == p2_idx) continue;
        Vec3 to_p = (points[i] - points[p1_idx]);
        Vec3 proj = line_dir * to_p.dot(line_dir);

        Number dist = (Number)math::sqrt((to_p - proj).sq_magnitude());

        if (dist > max_dist) {
            max_dist = dist;
            p3_idx = i;
        }
    }

    assert(max_dist > EPSILON, "All points are collinear.");

    max_dist = EPSILON;

    foreach (int i, point: points) {
        if (i == p1_idx || i == p2_idx || i == p3_idx) continue;
        Number dist = math::abs(hull.create_face({p1_idx, p2_idx, p3_idx}, {0,0,0}).distance_to_plane(points[i]));

        if (dist > max_dist) {
            max_dist = dist;
            p4_idx = i;
        }
    }

    Vec3 initial_centroid = (points[p1_idx] + points[p2_idx] + points[p3_idx] + points[p4_idx]) * 0.25f;

    HullFace f1 = hull.create_face({p1_idx, p2_idx, p3_idx}, initial_centroid);
    HullFace f2 = hull.create_face({p1_idx, p3_idx, p4_idx}, initial_centroid);
    HullFace f3 = hull.create_face({p1_idx, p4_idx, p2_idx}, initial_centroid);
    HullFace f4 = hull.create_face({p2_idx, p4_idx, p3_idx}, initial_centroid);

    hull.faces.push_all({f1, f2, f3, f4});

    // Find the face this point is "most outside" of
    foreach (usz i, point: points) {
        if (i == p1_idx || i == p2_idx || i == p3_idx || i == p4_idx) continue;

        int best_face_idx = -1;
        max_dist = EPSILON;

        foreach (usz j, face: hull.faces) {
            Number dist = face.distance_to_plane(point);

            if (dist > max_dist) {
                max_dist = dist;
                best_face_idx = (int)j;
            }
        }

        if (best_face_idx != -1) {
            hull.faces[best_face_idx].outside.push(i);
        }
    }

    usz current_hull_size = hull.faces.len();
    for (int i = 0; i < current_hull_size; i++) {
        hull.recurse(i);
    }

    IndexList indices;
    defer indices.free();

    foreach (face: hull.faces) {
        if (face.v.x != -1) {
            indices.push(face.v[0]);
            indices.push(face.v[1]);
            indices.push(face.v[2]);
        }
    }

    foreach (idx: indices) {
        hull.result.push(points[idx]);
    }

    return hull;
}

fn void QuickHull.free(&self) {
    foreach (face: self.faces) face.outside.free();
    self.faces.free();
    self.result.free();
}

fn Number HullFace.distance_to_plane(&self, Vec3 point) @local => (Number)self.normal.dot(point) - self.offset;

fn HullFace QuickHull.create_face(&self, Vertex indices, Vec3 inner_point) @local {
    Vec3 normal = (self.points[indices.y] - self.points[indices.x]).cross(self.points[indices.z] - self.points[indices.x]).normalize();

    HullFace face = {
        .v = indices,
        .visited = false,
        .offset = (Number)normal.dot(self.points[indices.x]),
        .normal = normal
    };

    if (face.distance_to_plane(inner_point) > EPSILON) {
        face.normal *= -1;
        face.offset *= -1;
        @swap(face.v.y, face.v.z);
    }

    return face;
}

/**
 * Main recursive step of the Quickhull algorithm.
 */
fn void QuickHull.recurse(&self, usz face_index) @local {
    HullFace* seed_face = self.faces.get_ref(face_index);

    if (seed_face.v[0] == -1 || seed_face.outside.len() == 0) {
        return;
    }

    int furthest_point_index = -1;
    Number max_dist = 0;

    foreach (usz index: seed_face.outside) {
        Number dist = seed_face.distance_to_plane(self.points[index]);

        if (dist > max_dist) {
            max_dist = dist;
            furthest_point_index = (int)index;
        }
    }

    if (furthest_point_index == -1) {
        // All points were within tolerance, consider this face done
        return;
    }

    IndexList new_face_indices;
    IndexList reassign;
    IndexList visible_faces;
    EdgeList horizon;

    defer {
        visible_faces.free();
        horizon.free();
        new_face_indices.free();
        reassign.free();
    }

    foreach (&face: self.faces) face.visited = false;

    self.find_horizon(face_index, (usz)furthest_point_index, &visible_faces, &horizon);

    Vec3 inner_point = {0, 0, 0};
    int visible_count_for_centroid = 0;

    foreach (usz i: visible_faces) {
        HullFace face = self.faces[i]; 
        if (face.v.x == 0) continue;
  
        inner_point += self.points[face.v.x];
        visible_count_for_centroid++;
    }

    if (visible_count_for_centroid > EPSILON) {
        inner_point /= visible_count_for_centroid;
    } else {
        inner_point = self.points[seed_face.v.x];
        inner_point += self.points[seed_face.v.y];
        inner_point += self.points[seed_face.v.z];
        inner_point /= 3.0;
    }

    // io::printf("face index %d \n", face_index);
    // io::printf("edge count %d \n", horizon.len());

    foreach (edge: horizon) {
        HullFace face = self.create_face(indices: {edge.x, edge.y, furthest_point_index}, inner_point: inner_point);
        new_face_indices.push(self.faces.len());
        self.faces.push(face);
    }

    foreach (face_idx: visible_faces) {
        HullFace face = self.faces[face_idx];
        if (face.v.x == -1) continue;

        foreach (idx: face.outside) {
            if (idx == furthest_point_index) continue;
            reassign.push(idx);
        }

        face.outside.clear();
    }

    // Re-assign points
    foreach (p_idx: reassign) {
        int best_face_idx = -1;
        max_dist = EPSILON;

        // Find the *new* face this point is furthest from
        foreach (face_idx: new_face_indices) {
            HullFace face = self.faces[face_idx];
            if (face.v.x == -1) continue;

            Number dist = face.distance_to_plane(self.points[p_idx]);

            if (dist > max_dist) {
                max_dist = dist;
                best_face_idx = (int)face_idx;
            }
        }

        if (best_face_idx != -1) {
            self.faces.get_ref(best_face_idx).outside.push(p_idx);
        };
    }

    // --- 5. Remove the old visible faces ---
    // We mark them as invalid by setting v[0] to -1
    // A proper implementation would use a linked list or compact the array
    foreach (idx: visible_faces) {
        self.faces.get_ref(idx).v.x = -1;
    }

    // --- 6. Recurse for each new face ---
    foreach (face_idx: new_face_indices) self.recurse(face_idx);
}

fn void QuickHull.find_horizon(&self, usz face_index, usz eye_index, IndexList* visible_faces, EdgeList* horizon ) @local {

    Vec3 eye_point = self.points[eye_index];
    
    HullFace* face = self.faces.get_ref(face_index);
    face.visited = true;

    if (face.distance_to_plane(eye_point) > EPSILON) {
        visible_faces.push(face_index);
       
        // Check its neighbors
        foreach NEIGHBOURS: (usz i, neighbor: self.faces) {
            if (neighbor.visited == true) continue;
            if (neighbor.v.x == -1) continue;

            // Find a shared edge (this is the slow, non-half-edge way)
            for (usz j = 0; j < 3; j++) {
                usz v0 = face.v[j];
                usz v1 = face.v[(j + 1) % 3];

                for (usz k = 0; k < 3; k++) {
                    usz nv0 = neighbor.v[k];
                    usz nv1 = neighbor.v[(k + 1) % 3];

                    // Check for shared edge (in either direction)
                    if ((v0 == nv0 && v1 == nv1) || (v0 == nv1 && v1 == nv0)) {
                        
                        // Found shared edge, recurse
                        self.find_horizon(i, eye_index, visible_faces, horizon);
                        continue NEIGHBOURS;
                    }
                }
            }
        }
    } else {
        for (usz j = 0; j < 3; j++) {
            int v0 = face.v[j];
            int v1 = face.v[(j + 1) % 3];

            foreach FACES: (usz i, neighbor: self.faces) {
                if (i == face_index) continue;
                if (neighbor.visited == false) continue;
                if (neighbor.v.x == -1) continue;

                // Check if neighbor is in the visible set
                bool neighbor_is_visible = false;
                foreach (v_idx: visible_faces) {
                    if (v_idx == i) {
                        neighbor_is_visible = true;
                        break;
                    }
                }

                if (neighbor_is_visible) {
                    for (int k = 0; k < 3; k++) {
                        int nv0 = neighbor.v[k];
                        int nv1 = neighbor.v[(k + 1) % 3];
                     
                        if ((v0 == nv0 && v1 == nv1) || (v0 == nv1 && v1 == nv0)) {    
                            if (v0 == nv1 && v1 == nv0) {
                                horizon.push({nv0, nv1});
                            } else {
                                horizon.push({v0, v1});
                            }

                            break FACES;
                        }
                    }
                }
            }
        }
    }
}

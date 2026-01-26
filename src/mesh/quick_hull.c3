module collision;

import std::math;
import std::collections::list;


// QuickHull 3D Convex Hull Algorithm
// Used for computing collision meshes from complex GLTF geometry

// Use type-appropriate epsilon - float has ~7 digits, double has ~15
// Vec3.sizeof is 12 for float[<3>] (3*4) or 24 for double[<3>] (3*8)
const double HULL_EPSILON = Vec3.sizeof == 12 ? 1e-5 : 1e-10;

// Face of the convex hull during construction
struct HullFace @local {
    usz[3] vertices;      // Indices into point array
    Vec3 normal;
    double distance;      // Distance from origin along normal
    bool alive;           // False if face has been removed
    List{int} outside;    // Points outside this face
}

fn void HullFace.free(&self) {
    self.outside.free();
}

// Compute face normal and distance from 3 points
fn void HullFace.compute_plane(&self, Vec3[] points) {
    Vec3 a = points[self.vertices[0]];
    Vec3 b = points[self.vertices[1]];
    Vec3 c = points[self.vertices[2]];

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    self.normal = ab.cross(ac).normalize();
    self.distance = self.normal.dot(a);
}

// Signed distance from point to face plane
fn double HullFace.point_distance(&self, Vec3 p) {
    return self.normal.dot(p) - self.distance;
}

// Find the farthest point from origin along a direction
fn int find_farthest_point(Vec3[] points, Vec3 direction) @local {
    int best = 0;
    double best_dist = direction.dot(points[0]);

    for (usz i = 1; i < points.len; i++) {
        double dist = direction.dot(points[i]);
        if (dist > best_dist) {
            best_dist = dist;
            best = (int)i;
        }
    }
    return best;
}

// Find extreme points along each axis to build initial simplex
fn int[6] find_extreme_points_hull(Vec3[] points) @local {
    int[6] extreme = { 0, 0, 0, 0, 0, 0 };

    for (usz i = 1; i < points.len; i++) {
        if (points[i].x < points[extreme[0]].x) extreme[0] = (int)i;
        if (points[i].x > points[extreme[1]].x) extreme[1] = (int)i;
        if (points[i].y < points[extreme[2]].y) extreme[2] = (int)i;
        if (points[i].y > points[extreme[3]].y) extreme[3] = (int)i;
        if (points[i].z < points[extreme[4]].z) extreme[4] = (int)i;
        if (points[i].z > points[extreme[5]].z) extreme[5] = (int)i;
    }
    return extreme;
}

// Find two points that are farthest apart
fn int[2] find_most_separated(Vec3[] points, int[6] extreme) @local {
    double max_dist = 0;
    int[2] result = { 0, 1 };

    for (int i = 0; i < 6; i++) {
        for (int j = i + 1; j < 6; j++) {
            Vec3 diff = points[extreme[i]] - points[extreme[j]];
            double dist = diff.dot(diff);
            if (dist > max_dist) {
                max_dist = dist;
                result[0] = extreme[i];
                result[1] = extreme[j];
            }
        }
    }
    return result;
}

// Find point farthest from line defined by two points
fn int find_farthest_from_line(Vec3[] points, int p0, int p1) @local {
    Vec3 a = points[p0];
    Vec3 b = points[p1];
    Vec3 ab = b - a;
    double ab_len_sq = ab.dot(ab);

    double max_dist = 0;
    int result = -1;

    for (usz i = 0; i < points.len; i++) {
        if ((int)i == p0 || (int)i == p1) continue;

        Vec3 ap = points[i] - a;
        double t = ap.dot(ab) / ab_len_sq;
        Vec3 closest = a + ab * (Vec3){(float)t, (float)t, (float)t};
        Vec3 diff = points[i] - closest;
        double dist = diff.dot(diff);

        if (dist > max_dist) {
            max_dist = dist;
            result = (int)i;
        }
    }
    return result;
}

// Find point farthest from plane defined by three points
fn int find_farthest_from_plane(Vec3[] points, int p0, int p1, int p2) @local {
    Vec3 a = points[p0];
    Vec3 b = points[p1];
    Vec3 c = points[p2];

    Vec3 normal = (b - a).cross(c - a).normalize();
    double d = normal.dot(a);

    double max_dist = 0;
    int result = -1;

    for (usz i = 0; i < points.len; i++) {
        if ((int)i == p0 || (int)i == p1 || (int)i == p2) continue;

        double dist = math::abs(normal.dot(points[i]) - d);
        if (dist > max_dist) {
            max_dist = dist;
            result = (int)i;
        }
    }
    return result;
}

// Build initial tetrahedron from 4 non-coplanar points
fn bool build_initial_tetrahedron(Vec3[] points, List{HullFace}* faces) @local {
    if (points.len < 4) return false;

    // Find extreme points
    int[6] extreme = find_extreme_points_hull(points);

    // Find two most separated points
    int[2] base = find_most_separated(points, extreme);
    int p0 = base[0];
    int p1 = base[1];

    if (p0 == p1) return false;

    // Find third point farthest from line p0-p1
    int p2 = find_farthest_from_line(points, p0, p1);
    if (p2 < 0) return false;

    // Find fourth point farthest from plane p0-p1-p2
    int p3 = find_farthest_from_plane(points, p0, p1, p2);
    if (p3 < 0) return false;

    // Check if p3 is above or below the plane to ensure consistent winding
    Vec3 normal = (points[p1] - points[p0]).cross(points[p2] - points[p0]);
    double dist = normal.dot(points[p3] - points[p0]);

    // Swap p1 and p2 if p3 is above the plane (ensures outward normals)
    if (dist > 0) {
        int temp = p1;
        p1 = p2;
        p2 = temp;
    }

    // Create 4 faces of tetrahedron with outward-facing normals
    HullFace f0 = { .vertices = {p0, p1, p2}, .alive = true };
    HullFace f1 = { .vertices = {p0, p2, p3}, .alive = true };
    HullFace f2 = { .vertices = {p0, p3, p1}, .alive = true };
    HullFace f3 = { .vertices = {p1, p3, p2}, .alive = true };

    f0.compute_plane(points);
    f1.compute_plane(points);
    f2.compute_plane(points);
    f3.compute_plane(points);

    faces.push(f0);
    faces.push(f1);
    faces.push(f2);
    faces.push(f3);

    return true;
}

// Assign each point to the face it's farthest outside of
fn void assign_points_to_faces(Vec3[] points, List{HullFace}* faces, usz[4] tetra_verts) @local {
    for (usz i = 0; i < points.len; i++) {
        // Skip tetrahedron vertices
        bool is_tetra = false;
        for (int j = 0; j < 4; j++) {
            if ((int)i == tetra_verts[j]) {
                is_tetra = true;
                break;
            }
        }
        if (is_tetra) continue;

        // Find face this point is farthest outside of
        double max_dist = HULL_EPSILON;
        int best_face = -1;

        for (usz f = 0; f < faces.len(); f++) {
            if (!faces.get(f).alive) continue;
            double dist = faces.get_ref(f).point_distance(points[i]);
            if (dist > max_dist) {
                max_dist = dist;
                best_face = (int)f;
            }
        }

        if (best_face >= 0) {
            faces.get_ref(best_face).outside.push((int)i);
        }
    }
}

// Find horizon edges - edges between visible and non-visible faces
fn void find_horizon(List{HullFace}* faces, Vec3 eye_point, Vec3[] points,
                     List{Pair}* horizon, List{int}* visible_faces) @local {
    // Mark all faces as visible or not from eye point
    for (usz i = 0; i < faces.len(); i++) {
        if (!faces.get(i).alive) continue;

        double dist = faces.get_ref(i).point_distance(eye_point);
        if (dist > HULL_EPSILON) {
            visible_faces.push((int)i);
        }
    }

    // Find horizon edges (edges shared between visible and non-visible faces)
    foreach (vi : *visible_faces) {
        HullFace* vf = faces.get_ref(vi);

        // Check each edge of visible face
        for (int e = 0; e < 3; e++) {
            usz v0 = vf.vertices[e];
            usz v1 = vf.vertices[(e + 1) % 3];

            // Check if this edge is shared with a non-visible face
            bool is_horizon = true;
            for (usz j = 0; j < faces.len(); j++) {
                if (!faces.get(j).alive) continue;
                if ((int)j == vi) continue;

                // Check if face j is visible
                bool j_visible = false;
                foreach (vj : *visible_faces) {
                    if (vj == (int)j) {
                        j_visible = true;
                        break;
                    }
                }

                if (j_visible) continue;

                // Check if face j shares this edge
                HullFace* f = faces.get_ref(j);
                for (int fe = 0; fe < 3; fe++) {
                    usz fv0 = f.vertices[fe];
                    usz fv1 = f.vertices[(fe + 1) % 3];
                    if ((fv0 == v0 && fv1 == v1) || (fv0 == v1 && fv1 == v0)) {
                        // Shared edge with non-visible face = horizon edge
                        // Use winding from non-visible face for correct orientation
                        horizon.push({ fv1, fv0 });
                        is_horizon = false;
                        break;
                    }
                }
            }
        }
    }
}

// Main QuickHull iteration - expand hull to include a point
// Returns true if hull was modified, false otherwise
fn bool expand_hull(List{HullFace}* faces, Vec3[] points, int point_idx) @local {
    Vec3 eye = points[point_idx];

    List{Pair} horizon;
    List{int} visible_faces;
    defer {
        horizon.free();
        visible_faces.free();
    }

    find_horizon(faces, eye, points, &horizon, &visible_faces);

    if (horizon.len() == 0 || visible_faces.len() == 0) return false;

    // Remove visible faces
    foreach (vi : visible_faces) {
        faces.get_ref(vi).alive = false;
    }

    // Create new faces connecting eye point to horizon edges
    foreach (edge : horizon) {
        HullFace new_face = {
            .vertices = {edge.a, edge.b, point_idx},
            .alive = true
        };
        new_face.compute_plane(points);
        faces.push(new_face);
    }

    return true;
}

// Redistribute points from removed faces to new faces
fn void redistribute_points(List{HullFace}* faces, Vec3[] points, usz first_new_face, bool[] hull_vertices) @local {
    // Collect orphaned points from dead faces
    List{int} orphans;
    defer orphans.free();

    for (usz i = 0; i < first_new_face; i++) {
        if (!faces.get(i).alive) {
            foreach (p : faces.get(i).outside) {
                // Skip points that are now hull vertices
                if (!hull_vertices[p]) {
                    orphans.push(p);
                }
            }
            faces.get_ref(i).outside.clear();
        }
    }

    // Reassign orphans to new faces
    foreach (pi : orphans) {
        double max_dist = HULL_EPSILON;
        int best_face = -1;

        for (usz f = first_new_face; f < faces.len(); f++) {
            if (!faces.get(f).alive) continue;
            double dist = faces.get_ref(f).point_distance(points[pi]);
            if (dist > max_dist) {
                max_dist = dist;
                best_face = (int)f;
            }
        }

        if (best_face >= 0) {
            faces.get_ref(best_face).outside.push(pi);
        }
    }
}

// Compute convex hull of a point cloud using QuickHull algorithm
fn Mesh quickhull(Vec3[] points) {
    Mesh result;

    if (points.len < 4) {
        // Degenerate case - return empty or simple mesh
        return result;
    }

    List{HullFace} faces;
    defer {
        foreach (&f : faces) {
            f.free();
        }
        faces.free();
    }

    // Build initial tetrahedron
    if (!build_initial_tetrahedron(points, &faces)) {
        return result;
    }

    // Get tetrahedron vertices for initial point assignment
    usz[4] tetra = {
        faces[0].vertices[0],
        faces[0].vertices[1],
        faces[0].vertices[2],
        faces[1].vertices[2]
    };

    // Track which points are already hull vertices (prevents infinite loops with float precision)
    bool[] hull_vertices = mem::new_array(bool, points.len);
    defer mem::free(hull_vertices);
    hull_vertices[tetra[0]] = true;
    hull_vertices[tetra[1]] = true;
    hull_vertices[tetra[2]] = true;
    hull_vertices[tetra[3]] = true;

    // Assign all other points to faces
    assign_points_to_faces(points, &faces, tetra);

    // Process faces with outside points
    bool changed = true;
    int max_iterations = (int)points.len * 3;  // Safety limit
    int iterations = 0;

    while (changed && iterations < max_iterations) {
        changed = false;
        iterations++;

        for (usz i = 0; i < faces.len(); i++) {
            if (!faces[i].alive) continue;
            if (faces[i].outside.len() == 0) continue;

            // Find farthest outside point and its index in the list
            double max_dist = 0;
            int farthest = -1;
            usz farthest_idx = 0;

            for (usz j = 0; j < faces[i].outside.len(); j++) {
                int pi = faces[i].outside[j];
                // Skip points already in hull (shouldn't happen, but safety check)
                if (hull_vertices[pi]) continue;

                double dist = faces[i].point_distance(points[pi]);
                if (dist > max_dist) {
                    max_dist = dist;
                    farthest = pi;
                    farthest_idx = j;
                }
            }

            if (farthest < 0) continue;

            // Mark this point as a hull vertex
            hull_vertices[farthest] = true;

            // Remove the point from outside list before processing
            faces[i].outside.remove_at(farthest_idx);

            // Remember where new faces start
            usz first_new = faces.len();

            // Expand hull to include this point
            bool expanded = expand_hull(&faces, points, farthest);

            if (expanded) {
                // Redistribute orphaned points from dead faces to new faces
                redistribute_points(&faces, points, first_new, hull_vertices);
                changed = true;
            }

            break;  // Restart from beginning
        }
    }

    // Build result mesh from alive faces
    // First, collect unique vertices
    List{int} vertex_map;  // original index -> new index
    defer vertex_map.free();

    // Initialize with -1
    for (usz i = 0; i < points.len; i++) {
        vertex_map.push(-1);
    }

    // Mark used vertices
    foreach (&f : faces) {
        if (!f.alive) continue;
        for (int j = 0; j < 3; j++) {
            usz vi = f.vertices[j];
            if (vertex_map[vi] < 0) {
                vertex_map[vi] = (int)result.vertices.len();
                result.vertices.push(points[vi]);
            }
        }
    }

    // Add triangles with remapped indices
    foreach (&f : faces) {
        if (!f.alive) continue;
        Triangle tri = {
            vertex_map[f.vertices[0]],
            vertex_map[f.vertices[1]],
            vertex_map[f.vertices[2]]
        };
        result.triangles.push(tri);
    }

    return result;
}

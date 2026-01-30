module collision;
import std::math, std::collections::list, std::io;

alias Cells = List{List{Vec3}};


// Border edge for cap surface generation
struct BorderEdge @local {
    Vec3 p0;
    Vec3 p1;
}

// Find index of point in list, or -1 if not found
fn isz find_point_index(List{Vec3}* points, Vec3 p, float eps) @local {
    foreach (i, existing : *points) {
        Vec3 diff = existing - p;
        if (diff.dot(diff) < eps * eps) {
            return (isz)i;
        }
    }
    return -1;
}

// Find root with path compression (iterative)
fn isz find_root_iterative(isz[] parent, isz x) @local {
    isz root = x;
    // Find root
    while (parent[root] != root) {
        root = parent[root];
    }
    // Path compression
    while (parent[x] != root) {
        isz next = parent[x];
        parent[x] = root;
        x = next;
    }
    return root;
}

// Count connected loops in border edge graph using union-find
fn usz count_border_loops(List{BorderEdge}* border, List{Vec3}* points) @local {
    if (points.len() == 0) return 0;

    // Union-find parent array
    isz[] parent = mem::new_array(isz, points.len());
    defer mem::free(parent);

    for (usz i = 0; i < points.len(); i++) {
        parent[i] = (isz)i;
    }

    // Union edges
    const float EPS = 0.0001f;
    foreach (edge : *border) {
        isz i0 = find_point_index(points, edge.p0, EPS);
        isz i1 = find_point_index(points, edge.p1, EPS);
        if (i0 >= 0 && i1 >= 0) {
            isz r0 = find_root_iterative(parent, i0);
            isz r1 = find_root_iterative(parent, i1);
            if (r0 != r1) {
                parent[r0] = r1;
            }
        }
    }

    // Count unique roots
    usz num_loops = 0;
    for (usz i = 0; i < points.len(); i++) {
        if (find_root_iterative(parent, (isz)i) == (isz)i) {
            num_loops++;
        }
    }

    return num_loops;
}

// Add cap surface triangles using fan triangulation from centroid
// Skips cap generation if border has multiple disconnected loops (holes)
fn void add_cap_surface(List{BorderEdge}* border, Plane plane, Mesh* mesh, bool flip_winding) @local {
    if (border.len() < 3) return;

    // Collect unique border points
    List{Vec3} border_points;
    defer border_points.free();

    const float MERGE_DIST = 0.0001f;

    foreach (edge : *border) {
        // Add p0 if not already present
        if (find_point_index(&border_points, edge.p0, MERGE_DIST) < 0) {
            border_points.push(edge.p0);
        }
        // Add p1 if not already present
        if (find_point_index(&border_points, edge.p1, MERGE_DIST) < 0) {
            border_points.push(edge.p1);
        }
    }

    if (border_points.len() < 3) return;

    // For complex meshes, skip expensive loop counting if we have many border points
    // Simple heuristic: if cut has many edges, it's likely complex (circular, etc)
    if (border.len() > 50) {
        // Too many edges - skip cap generation (likely circular cut or complex shape)
        return;
    }

    // Check for multiple disconnected loops (indicates hole in cut surface)
    // If we have multiple loops, skip cap generation to avoid filling holes
    usz num_loops = count_border_loops(border, &border_points);
    if (num_loops > 1) {
        // Multiple loops detected - this is a polygon with holes
        // Skip cap generation to preserve the hole
        return;
    }

    // Compute centroid
    Vec3 centroid = {0, 0, 0};
    foreach (p : border_points) {
        centroid += p;
    }
    centroid /= (float)border_points.len();

    // Sort border points by angle around centroid (in the plane)
    // First, find two perpendicular vectors in the plane
    Vec3 n = plane.normal;
    Vec3 up = {0, 1, 0};
    if (math::abs(n.dot(up)) > 0.9f) {
        up = {1, 0, 0};
    }
    Vec3 u = n.cross(up).normalize();
    Vec3 v = n.cross(u);

    // Compute angles and sort
    float[] angles = mem::new_array(float, border_points.len());
    defer mem::free(angles);

    for (usz i = 0; i < border_points.len(); i++) {
        Vec3 d = border_points[i] - centroid;
        angles[i] = (float)math::atan2(d.dot(v), d.dot(u));
    }

    // Simple bubble sort by angle (border usually has few points)
    for (usz i = 0; i < border_points.len() - 1; i++) {
        for (usz j = 0; j < border_points.len() - 1 - i; j++) {
            if (angles[j] > angles[j + 1]) {
                float tmp_a = angles[j];
                angles[j] = angles[j + 1];
                angles[j + 1] = tmp_a;

                Vec3 tmp_p = border_points[j];
                border_points[j] = border_points[j + 1];
                border_points[j + 1] = tmp_p;
            }
        }
    }

    // Create fan triangles from centroid
    for (usz i = 0; i < border_points.len(); i++) {
        usz j = (i + 1) % border_points.len();
        Vec3 p0 = border_points[i];
        Vec3 p1 = border_points[j];

        if (flip_winding) {
            mesh.add_triangle(centroid, p1, p0);
        } else {
            mesh.add_triangle(centroid, p0, p1);
        }
    }
}

// MeshClipper functions - properly splits triangles at plane intersection
fn void clip_triangle(Mesh* mesh, Triangle tri, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh, List{BorderEdge}* border) @local {
    Vec3 v0 = mesh.vertices[tri.x];
    Vec3 v1 = mesh.vertices[tri.y];
    Vec3 v2 = mesh.vertices[tri.z];

    double d0 = plane.distance(v0);
    double d1 = plane.distance(v1);
    double d2 = plane.distance(v2);

    const double EPS = 1e-6;

    // Classify vertices
    int s0 = d0 > EPS ? 1 : (d0 < -EPS ? -1 : 0);
    int s1 = d1 > EPS ? 1 : (d1 < -EPS ? -1 : 0);
    int s2 = d2 > EPS ? 1 : (d2 < -EPS ? -1 : 0);

    int pos_count = (s0 == 1 ? 1 : 0) + (s1 == 1 ? 1 : 0) + (s2 == 1 ? 1 : 0);
    int neg_count = (s0 == -1 ? 1 : 0) + (s1 == -1 ? 1 : 0) + (s2 == -1 ? 1 : 0);

    // All on positive side
    if (neg_count == 0) {
        pos_mesh.add_triangle(v0, v1, v2);
        return;
    }

    // All on negative side
    if (pos_count == 0) {
        neg_mesh.add_triangle(v0, v1, v2);
        return;
    }

    // Triangle crosses the plane - need to split
    // Collect positive and negative vertices, and border intersection points
    Vec3[4] pos_verts;
    Vec3[4] neg_verts;
    Vec3[2] intersections;
    int pos_idx = 0;
    int neg_idx = 0;
    int int_idx = 0;

    // Process each edge
    Vec3[3] verts = { v0, v1, v2 };
    double[3] dists = { d0, d1, d2 };
    int[3] sides = { s0, s1, s2 };

    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        Vec3 vi = verts[i];
        Vec3 vj = verts[j];
        int si = sides[i];
        int sj = sides[j];

        // Add current vertex to appropriate side
        if (si >= 0) {
            pos_verts[pos_idx++] = vi;
        }
        if (si <= 0) {
            neg_verts[neg_idx++] = vi;
        }

        // If edge crosses plane, add intersection point to both sides
        if ((si > 0 && sj < 0) || (si < 0 && sj > 0)) {
            Vec3 intersection = intersect_edge(vi, vj, dists[i], dists[j]);
            pos_verts[pos_idx++] = intersection;
            neg_verts[neg_idx++] = intersection;
            if (int_idx < 2) {
                intersections[int_idx++] = intersection;
            }
        }
    }

    // Collect border edge for cap surface generation
    if (int_idx == 2 && border != null) {
        border.push({ .p0 = intersections[0], .p1 = intersections[1] });
    }

    // Build triangles from collected vertices
    if (pos_idx >= 3) {
        pos_mesh.add_triangle(pos_verts[0], pos_verts[1], pos_verts[2]);
        if (pos_idx == 4) {
            pos_mesh.add_triangle(pos_verts[0], pos_verts[2], pos_verts[3]);
        }
    }

    if (neg_idx >= 3) {
        neg_mesh.add_triangle(neg_verts[0], neg_verts[1], neg_verts[2]);
        if (neg_idx == 4) {
            neg_mesh.add_triangle(neg_verts[0], neg_verts[2], neg_verts[3]);
        }
    }
}

fn void clip_mesh(Mesh* mesh, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh) @local {
    foreach (tri : mesh.triangles) {
        clip_triangle(mesh, tri, plane, pos_mesh, neg_mesh, null);
    }
    // Note: No cap surfaces - for collision we only need vertices, not watertight meshes
}

// ============================================================================
// CoACD - Collision-Aware Convex Decomposition
// Based on: "CoACD: Collision-Aware Convex Decomposition" (Wei et al.)
// Reference: https://github.com/SarahWeiii/CoACD
// ============================================================================

struct CoacdParameters {
    float threshold;             // Concavity threshold (0.01-0.1, lower = more precise)
    int max_hulls;               // Maximum output hull count
    int plane_candidates;        // Candidate planes per axis for cut search
    int max_iterations;          // Maximum decomposition iterations
    float k;                     // Volume metric weight (default 0.3)
}

// Preset configurations
const CoacdParameters COACD_VERY_FAST = {
    .threshold = 0.15,
    .max_hulls = 8,
    .plane_candidates = 3,
    .max_iterations = 50,
    .k = 0.3
};

const CoacdParameters COACD_FAST = {
    .threshold = 0.04,
    .max_hulls = 12,
    .plane_candidates = 5,
    .max_iterations = 30,
    .k = 0.3
};

const CoacdParameters COACD_MEDIUM = {
    .threshold = 0.05,
    .max_hulls = 32,
    .plane_candidates = 8,
    .max_iterations = 150,
    .k = 0.3
};

const CoacdParameters COACD_SLOW = {
    .threshold = 0.03,
    .max_hulls = 64,
    .plane_candidates = 12,
    .max_iterations = 300,
    .k = 0.3
};

// ----------------------------------------------------------------------------
// Internal structures for CoACD
// ----------------------------------------------------------------------------

// Part in the decomposition queue
struct CoacdPart @local {
    Mesh mesh;
    float concavity;
}

fn void CoacdPart.free(&self) {
    self.mesh.free();
}

// Cutting plane defined by normal and distance from origin
struct CutPlane @local {
    Vec3 normal;
    float d;  // Distance from origin (plane equation: normal.dot(p) = d)
}

// ----------------------------------------------------------------------------
// Concavity Metrics (CoACD Paper)
// Uses Rv (volume-based radius) as primary metric for speed
// Rv = k * (3 * |V_hull - V_mesh| / (4 * PI))^(1/3)
// ----------------------------------------------------------------------------

// Compute Rv: radius of sphere with equivalent volume difference
fn float compute_rv(Mesh* mesh, Mesh* hull, float k) @local {
    double mesh_vol = math::abs(mesh.get_volume());
    double hull_vol = math::abs(hull.get_volume());
    double vol_diff = math::abs(hull_vol - mesh_vol);

    if (vol_diff < 1e-10) return 0;

    double rv = math::pow(3.0 * vol_diff / (4.0 * math::PI), 1.0/3.0);
    return (float)(rv * k);
}

// Compute Rv for merged hulls (used in cut evaluation)
// v1, v2 = volumes of two parts, v_combined = volume of combined hull
fn float compute_rv_merged(double v1, double v2, double v_combined, float k) @local {
    double vol_diff = math::abs(v1 + v2 - v_combined);
    if (vol_diff < 1e-10) return 0;

    double rv = math::pow(3.0 * vol_diff / (4.0 * math::PI), 1.0/3.0);
    return (float)(rv * k);
}

// Compute concavity as normalized Rv (scale-invariant)
fn float compute_concavity(Mesh* mesh, Mesh* hull, float k, float diagonal) @local {
    float rv = compute_rv(mesh, hull, k);
    // Normalize by mesh diagonal for scale invariance
    return rv / math::max(diagonal, 0.001f);
}

// ----------------------------------------------------------------------------
// Cutting Plane Generation and Evaluation
// Uses triangle normals for cutting planes (works well for hollow shapes)
// ----------------------------------------------------------------------------

// Split mesh by a cutting plane
fn void split_mesh_by_plane(Mesh* mesh, CutPlane cut, Mesh* pos_mesh, Mesh* neg_mesh) @local {
    Plane plane = create_plane(cut.normal, cut.d);
    clip_mesh(mesh, plane, pos_mesh, neg_mesh);
}

// Count vertices on each side of a cutting plane
fn bool count_vertices_on_sides(Mesh* mesh, CutPlane cut, int* pos_count, int* neg_count) @local {
    *pos_count = 0;
    *neg_count = 0;

    foreach (v : mesh.vertices) {
        float dist = cut.normal.dot(v) - cut.d;
        if (dist > 0.001f) {
            (*pos_count)++;
        } else if (dist < -0.001f) {
            (*neg_count)++;
        }
    }

    return *pos_count >= 4 && *neg_count >= 4;
}

// Score a cut by balance ratio (lower = more balanced = better)
fn float score_cut(Mesh* mesh, CutPlane cut) @local {
    int pos_count, neg_count;
    if (!count_vertices_on_sides(mesh, cut, &pos_count, &neg_count)) {
        return float.max;
    }
    return (float)math::max(pos_count, neg_count) / (float)math::min(pos_count, neg_count);
}

// ----------------------------------------------------------------------------
// Best Cut Search using Triangle Normals
// This is the key for hollow shapes - uses mesh geometry to find good cuts
// ----------------------------------------------------------------------------

// Find the best cutting plane by examining triangle normals
// For hollow shapes, inner faces have normals pointing inward - using those
// as cutting planes will cut through the hollow space properly
fn CutPlane find_best_cut(Mesh* mesh, Aabb3 bounds, CoacdParameters* params, float diagonal) @local {
    if (mesh.triangles.len() == 0) {
        // Fallback: center cut on X axis
        return { .normal = {1, 0, 0}, .d = (bounds.min.x + bounds.max.x) * 0.5f };
    }

    Vec3[] verts = mesh.vertices.array_view();

    // Collect candidate planes from triangles with significant area
    // We sample triangles to get diverse cutting directions
    CutPlane best_cut = { .normal = {1, 0, 0}, .d = 0 };
    float best_score = float.max;

    // Calculate total mesh area for threshold
    double total_area = 0;
    foreach (tri : mesh.triangles) {
        total_area += tri.get_area(verts);
    }
    double min_area = total_area / (double)mesh.triangles.len() * 0.5;  // At least half of average

    // Sample stride - don't check every triangle for large meshes
    usz stride = math::max((usz)1, mesh.triangles.len() / (usz)50);
    int candidates_checked = 0;
    int max_candidates = params.plane_candidates * 3;

    for (usz i = 0; i < mesh.triangles.len() && candidates_checked < max_candidates; i += stride) {
        Triangle tri = mesh.triangles[i];
        double area = tri.get_area(verts);

        // Skip small triangles
        if (area < min_area) continue;

        Vec3 normal = tri.get_normal(verts);
        if (normal.length() < 0.001f) continue;

        normal = normal.normalize();
        Vec3 center = tri.get_center(verts);
        float d = normal.dot(center);

        CutPlane cut = { .normal = normal, .d = d };
        float score = score_cut(mesh, cut);

        candidates_checked++;

        if (score < best_score) {
            best_score = score;
            best_cut = cut;
        }

        // Also try the negated normal (opposite direction)
        CutPlane cut_neg = { .normal = -normal, .d = -d };
        float score_neg = score_cut(mesh, cut_neg);
        if (score_neg < best_score) {
            best_score = score_neg;
            best_cut = cut_neg;
        }
    }

    // If no good triangle-based cut found, fallback to axis-aligned
    if (best_score >= float.max) {
        Vec3 size = bounds.size();
        // Choose longest axis
        if (size.x >= size.y && size.x >= size.z) {
            best_cut = { .normal = {1, 0, 0}, .d = (bounds.min.x + bounds.max.x) * 0.5f };
        } else if (size.y >= size.z) {
            best_cut = { .normal = {0, 1, 0}, .d = (bounds.min.y + bounds.max.y) * 0.5f };
        } else {
            best_cut = { .normal = {0, 0, 1}, .d = (bounds.min.z + bounds.max.z) * 0.5f };
        }
    }

    return best_cut;
}


// ----------------------------------------------------------------------------
// Main Decomposition Function
// Uses is_convex() check - keeps splitting until pieces become convex
// This is the key for hollow shapes!
// ----------------------------------------------------------------------------

// Part in queue with depth tracking
struct DecompPart @local {
    Mesh mesh;
    int depth;
}

fn void DecompPart.free(&self) {
    self.mesh.free();
}

fn List{Mesh} coacd_decompose(Mesh* mesh, CoacdParameters params = COACD_MEDIUM) {
    List{Mesh} result;

    if (mesh.vertices.len() < 4 || mesh.triangles.len() < 2) {
        return result;
    }

    // Validate and clean up mesh
    mesh.validate();
    mesh.remesh();

    io::printfn("CoACD: Input mesh: %d tris, %d verts", mesh.triangles.len(), mesh.vertices.len());

    // If already convex, return hull directly
    if (mesh.is_convex()) {
        io::printfn("CoACD: Mesh is already convex");
        Mesh hull = quickhull(mesh.vertices.array_view());
        if (hull.vertices.len() >= 4) {
            result.push(hull);
        }
        return result;
    }

    Aabb3 bounds = mesh.get_aabb();
    float diagonal = bounds.size().length();

    // Minimum volume threshold - hulls smaller than this are filtered out
    // This prevents tiny degenerate pieces but keeps thin wall sections
    double total_volume = mesh.get_volume();
    double min_hull_volume = total_volume * 0.001;  // 0.1% of total volume (very small)

    // Working queue
    List{DecompPart} queue;
    defer {
        foreach (&part : queue) {
            part.free();
        }
        queue.free();
    }

    // Create initial part
    Mesh mesh_copy;
    mesh_copy.vertices.push_all(mesh.vertices.array_view());
    mesh_copy.triangles.push_all(mesh.triangles.array_view());

    queue.push({
        .mesh = mesh_copy,
        .depth = 0
    });

    int max_depth = 8;  // Need more splits to properly separate hollow box walls
    int iteration = 0;
    int filtered_count = 0;  // Track how many small hulls were filtered

    while (queue.len() > 0 && result.len() < params.max_hulls && iteration < params.max_iterations) {
        iteration++;

        // Get first part from queue
        DecompPart part = queue[0];
        queue.remove_at(0);

        // If convex or max depth, add to results
        if (part.depth >= max_depth || part.mesh.is_convex()) {
            Mesh hull = quickhull(part.mesh.vertices.array_view());
            // Only add hulls with sufficient volume (filter out tiny corner pieces)
            if (hull.vertices.len() >= 4 && hull.get_volume() >= min_hull_volume) {
                io::printfn("  Adding hull at depth %d: %d verts, convex=%s",
                    part.depth, hull.vertices.len(), part.mesh.is_convex() ? "yes" : "no(max_depth)");
                result.push(hull);
            } else {
                filtered_count++;
                hull.free();
            }
            part.free();
            continue;
        }

        // Find best cutting plane using triangle normals
        Aabb3 part_bounds = part.mesh.get_aabb();
        CutPlane best_cut = find_best_cut(&part.mesh, part_bounds, &params, diagonal);

        // Apply the cut
        Mesh pos_mesh;
        Mesh neg_mesh;
        split_mesh_by_plane(&part.mesh, best_cut, &pos_mesh, &neg_mesh);

        // Clean up split meshes - weld duplicate vertices
        pos_mesh.remesh();
        neg_mesh.remesh();

        bool pos_valid = pos_mesh.vertices.len() >= 4 && pos_mesh.triangles.len() >= 2;
        bool neg_valid = neg_mesh.vertices.len() >= 4 && neg_mesh.triangles.len() >= 2;

        if (!pos_valid && !neg_valid) {
            // Can't split further - add hull to results if large enough
            Mesh hull = quickhull(part.mesh.vertices.array_view());
            if (hull.vertices.len() >= 4 && hull.get_volume() >= min_hull_volume) {
                result.push(hull);
            } else {
                filtered_count++;
                hull.free();
            }
            part.free();
            pos_mesh.free();
            neg_mesh.free();
            continue;
        }

        int new_depth = part.depth + 1;

        // Add valid parts to queue for further processing
        if (pos_valid) {
            queue.push({
                .mesh = pos_mesh,
                .depth = new_depth
            });
        } else {
            pos_mesh.free();
        }

        if (neg_valid) {
            queue.push({
                .mesh = neg_mesh,
                .depth = new_depth
            });
        } else {
            neg_mesh.free();
        }

        part.free();
    }

    // Process remaining queue items
    foreach (&part : queue) {
        if (result.len() >= params.max_hulls) break;

        Mesh hull = quickhull(part.mesh.vertices.array_view());
        if (hull.vertices.len() >= 4 && hull.get_volume() >= min_hull_volume) {
            result.push(hull);
        } else {
            filtered_count++;
            hull.free();
        }
        part.mesh.free();
        part.mesh = {};
    }

    io::printfn("CoACD: Complete - %d hulls in %d iterations (filtered %d small, min_vol=%.4f)", result.len(), iteration, filtered_count, min_hull_volume);

    // Debug: show hull info
    foreach (i, &hull : result) {
        Aabb3 hb = hull.get_aabb();
        double vol = hull.get_volume();
        io::printfn("  Hull %d: %d verts, vol=%.4f, size=(%.2f, %.2f, %.2f)",
            i, hull.vertices.len(), vol,
            hb.max.x - hb.min.x, hb.max.y - hb.min.y, hb.max.z - hb.min.z);
    }

    return result;
}

// Convenience wrapper
fn List{Mesh} decompose(Mesh* mesh, CoacdParameters params = COACD_MEDIUM) {
    return coacd_decompose(mesh, params);
}

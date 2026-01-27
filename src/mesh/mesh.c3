module collision;
import std::math, std::collections::list, std::io;

alias Cells = List{List{Vec3}};

// // 3D Vector structure
const double VEC_EPSILON = Vec3.sizeof == 12 ? 1e-5 : 1e-9;

fn bool Vec3.equals(Vec3* self, Vec3 v) {
    return math::abs(self.x - v.x) < VEC_EPSILON &&
           math::abs(self.y - v.y) < VEC_EPSILON &&
           math::abs(self.z - v.z) < VEC_EPSILON;
}

// Triangle structure
alias Triangle = uint[<3>];

fn Vec3 Triangle.get_normal(Triangle* self, Vec3[] vertices) {
    // Bounds check
    if (self.x >= vertices.len || self.y >= vertices.len || self.z >= vertices.len) {
        return {0, 1, 0};  // Return up vector for invalid triangles
    }
    Vec3 e1 = vertices[self.y] - vertices[self.x];
    Vec3 e2 = vertices[self.z] - vertices[self.x];
    return e1.cross(e2).normalize();
}

fn Vec3 Triangle.get_center(Triangle* self, Vec3[] vertices) {
    if (self.x >= vertices.len || self.y >= vertices.len || self.z >= vertices.len) {
        return {0, 0, 0};
    }
    return (vertices[self.x] + vertices[self.y] + vertices[self.z]) / 3.0;
}

fn double Triangle.get_area(Triangle* self, Vec3[] vertices) {
    if (self.x >= vertices.len || self.y >= vertices.len || self.z >= vertices.len) {
        return 0;
    }
    Vec3 e1 = vertices[self.y] - vertices[self.x];
    Vec3 e2 = vertices[self.z] - vertices[self.x];
    return e1.cross(e2).length() * 0.5;
}

// Mesh structure
struct Mesh (CollisionShape) {
    List{Vec3} vertices;
    List{Triangle} triangles;
    TriBVHNode* bvh;
}

fn usz? Mesh.to_format(&self, Formatter* formatter) @dynamic {
	usz lenght = 0;
	foreach (vert: self.vertices) {
		lenght += formatter.printf("%s,%s,%s\n", vert.x, vert.y, vert.z)!;
	}

	return lenght;
}

fn Mesh create_mesh(Vec3[] vertices, uint[] indices) {
	Mesh mesh;
	mesh.vertices.push_all(vertices);
	mesh.set_indices(indices);
	return mesh;
}

// Build BVH tree for accelerated collision detection
// Uses indexed mode - references mesh vertices directly without copying
fn void Mesh.build_bvh(&self) {
    if (self.triangles.len() == 0) return;
    if (self.bvh != null) return;  // Already built

    // Use indexed BVH - no vertex duplication
    self.bvh = create_indexed_tri_bvh(
        self.vertices.entries,
        self.triangles.entries,
        self.triangles.len()
    );
}

fn bool Mesh.has_bvh(&self) => self.bvh != null;

fn void Mesh.free(&self) @dynamic {
    self.vertices.free();
    self.triangles.free();
    if (self.bvh != null) {
        self.bvh.free_indexed();
        mem::free(self.bvh);
    }
}

fn void Mesh.add_triangle(&self, Vec3 v0, Vec3 v1, Vec3 v2) {
    int idx0 = (int)self.vertices.len();
    self.vertices.push(v0);
    self.vertices.push(v1);
    self.vertices.push(v2);
    self.triangles.push({idx0, idx0 + 1, idx0 + 2});
}

// Load from flat index array (GLTF-style)
fn void Mesh.set_indices(&self, uint[] indices) {
    self.triangles.clear();
    for (usz i = 0; i + 2 < indices.len; i += 3) {
        self.triangles.push({indices[i], indices[i+1], indices[i+2]});
    }
}
fn Vec3 Mesh.get_center(&self) {
    Vec3 center = {0, 0, 0};
    foreach (v : self.vertices) {
        center += v;
    }
    return center / self.vertices.len();
}

// Validate mesh triangles, removing any with out-of-bounds indices
fn void Mesh.validate(&self) {
    usz vert_count = self.vertices.len();
    if (vert_count == 0) return;

    // Filter out invalid triangles
    List{Triangle} valid_tris;
    foreach (tri : self.triangles) {
        if (tri.x < vert_count && tri.y < vert_count && tri.z < vert_count) {
            valid_tris.push(tri);
        }
    }

    if (valid_tris.len() != self.triangles.len()) {
        self.triangles.free();
        self.triangles = valid_tris;
    } else {
        valid_tris.free();
    }
}

fn double Mesh.get_volume(&self) {
    double volume = 0.0;
    usz vert_count = self.vertices.len();

    foreach (tri : self.triangles) {
        // Skip invalid triangles
        if (tri.x >= vert_count || tri.y >= vert_count || tri.z >= vert_count) continue;

        Vec3 v0 = self.vertices[tri.x];
        Vec3 v1 = self.vertices[tri.y];
        Vec3 v2 = self.vertices[tri.z];

        volume += v0.dot(v1.cross(v2)) / 6.0;
    }

    return math::abs(volume);
}

fn bool Mesh.is_convex(&self) {
    if (self.triangles.len() < 4) return true;

    const double EPSILON = Vec3.sizeof == 4 ? 1e-4 : 1e-6;
    usz vert_count = self.vertices.len();

    foreach (tri : self.triangles) {
        // Bounds check for invalid meshes
        if (tri.x >= vert_count || tri.y >= vert_count || tri.z >= vert_count) {
            return false;  // Treat invalid mesh as non-convex (will trigger decomposition)
        }

        Vec3[] verts = self.vertices.array_view();
        Vec3 normal = tri.get_normal(verts);
        Vec3 p0 = self.vertices[tri.x];
        double d = normal.dot(p0);

        foreach (v : self.vertices) {
            double dist = normal.dot(v) - d;
            if (dist > EPSILON) {
                return false;
            }
        }
    }

    return true;
}

<*
 @require !self.vertices.is_empty() : `Vertices should not be empty`
*>
fn Aabb3 Mesh.get_aabb(&self) @dynamic {
    Vec3 min = {float.max, float.max, float.max};
    Vec3 max = {-float.max, -float.max, -float.max};

    foreach (point: self.vertices) {
        min.x = math::min(point.x, min.x);
        min.y = math::min(point.y, min.y);
        min.z = math::min(point.z, min.z);

        max.x = math::max(point.x, max.x);
        max.y = math::max(point.y, max.y);
        max.z = math::max(point.z, max.z);
	}

    return {
        min,
        max
    };
}

fn Vec3 Mesh.furthest_point(&self, Vec3 direction) @dynamic {
    if (self.vertices.len() == 0) {
        return {0, 0, 0};  // Safeguard for empty mesh
    }

    Vec3 furthest_point = self.vertices[0];
    float max_distance = furthest_point.dot(direction);

	foreach (point: self.vertices) {
		float distance = point.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = point;
        }
	}

    return furthest_point;
}

fn Vec3 Mesh.center_of_mass(self) {
    float total_detJ = 0;
    Vec3 weighted_com = {0, 0, 0};
    Vec3 final_com = {0, 0, 0};

	foreach (triangle: self.triangles) {
		Vec3 v1 = self.vertices[triangle.x].xyz;
        Vec3 v2 = self.vertices[triangle.y].xyz;
        Vec3 v3 = self.vertices[triangle.z].xyz;

        total_detJ += v1.dot(v2.cross(v3));

        weighted_com.x += total_detJ * (v1.x + v2.x + v3.x);
        weighted_com.y += total_detJ * (v1.y + v2.y + v3.y);
        weighted_com.z += total_detJ * (v1.z + v2.z + v3.z);
	}

    if (math::abs(total_detJ) > VEC_EPSILON) final_com = 0.25 * weighted_com / total_detJ;

    return final_com;
}

fn Matrix3f Mesh.inertia_tensor(&self, float mass) @dynamic {
    Matrix3f inertia = MATRIX3F_IDENTITY;

    float density = 1.0;

    Vec3 c = self.center_of_mass();

    foreach (triangle: self.triangles) {
		Vec3 v1 = self.vertices[triangle.x].xyz;
        Vec3 v2 = self.vertices[triangle.y].xyz;
        Vec3 v3 = self.vertices[triangle.z].xyz;

        float tetra_mass;
        Matrix3f tetra_I_origin = MATRIX3F_IDENTITY;

        inertia += compute_inertia_contribution(v1, v2, v3, density);
	}

    inertia.m00 -= mass * (c.y * c.y + c.z * c.z);
    inertia.m11 -= mass * (c.x * c.x + c.z * c.z);
    inertia.m22 -= mass * (c.x * c.x + c.y * c.y);

    inertia.m01 -= mass * (-c.x * c.y);
    inertia.m02 -= mass * (-c.x * c.z);
    inertia.m12 -= mass * (-c.y * c.z);

    inertia.m10 -= mass * (-c.y * c.x);
    inertia.m20 -= mass * (-c.z * c.x);
    inertia.m21 -= mass * (-c.z * c.y);

    return inertia;
}

fn ShapeType Mesh.get_type(&self) @dynamic => ShapeType.CONVEX;


// Plane structure for clipping
struct Plane {
    Vec3 normal;
    double d;
}

fn Plane create_plane(Vec3 n = {0, 0, 0}, double dist = 0.0) {
    return {n.normalize(), dist};
}

fn double Plane.distance(Plane* self, Vec3 p) {
    return self.normal.dot(p) - self.d;
}

fn Plane Plane.from_points(&self, Vec3 p0, Vec3 p1, Vec3 p2) {
    Vec3 n = (p1 - p0).cross(p2 - p0).normalize();
    return create_plane(n, n.dot(p0));
}

// Compute intersection point between two vertices across a plane
fn Vec3 intersect_edge(Vec3 v0, Vec3 v1, double d0, double d1) @local {
    double t = d0 / (d0 - d1);
    return v0 + (v1 - v0) * (float)t;
}

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
    List{BorderEdge} border;
    defer border.free();

    foreach (tri : mesh.triangles) {
        clip_triangle(mesh, tri, plane, pos_mesh, neg_mesh, &border);
    }

    // Add cap surfaces to close the meshes along the cut plane
    if (border.len() >= 3) {
        add_cap_surface(&border, plane, pos_mesh, false);  // Normal facing positive
        add_cap_surface(&border, plane, neg_mesh, true);   // Normal facing negative
    }
}

// ============================================================================
// CoACD - Collision-Aware Convex Decomposition
// Based on: https://github.com/Jondolf/CoACD-rs
// Paper: "CoACD: Collision-Aware Convex Decomposition" (Wei et al.)
// ============================================================================

struct CoacdParameters {
    float concavity_threshold;   // Max concavity before splitting (0.01-0.1)
    int resolution;              // Boundary sample count for Hausdorff
    int mcts_iterations;         // MCTS search iterations
    int mcts_max_depth;          // Max depth in MCTS rollout
    int mcts_nodes;              // Candidate planes per axis
    int max_convex_hulls;        // Maximum output hull count
    float k;                     // Rv scaling factor (volume metric weight)
}

// Preset configurations - matching CoACD-rs parameters
const CoacdParameters COACD_VERY_FAST = {
    .concavity_threshold = 0.1,
    .resolution = 500,
    .mcts_iterations = 100,
    .mcts_max_depth = 1,
    .mcts_nodes = 5,
    .max_convex_hulls = 16,
    .k = 0.3
};

const CoacdParameters COACD_FAST = {
    .concavity_threshold = 0.075,
    .resolution = 1000,
    .mcts_iterations = 150,
    .mcts_max_depth = 2,
    .mcts_nodes = 10,
    .max_convex_hulls = 32,
    .k = 0.3
};

const CoacdParameters COACD_MEDIUM = {
    .concavity_threshold = 0.05,
    .resolution = 2000,
    .mcts_iterations = 200,
    .mcts_max_depth = 3,
    .mcts_nodes = 20,
    .max_convex_hulls = 48,
    .k = 0.3
};

const CoacdParameters COACD_SLOW = {
    .concavity_threshold = 0.025,
    .resolution = 5000,
    .mcts_iterations = 500,
    .mcts_max_depth = 5,
    .mcts_nodes = 30,
    .max_convex_hulls = 64,
    .k = 0.3
};

// ----------------------------------------------------------------------------
// Internal structures for CoACD
// ----------------------------------------------------------------------------

// Decomposition part in the processing queue
struct CoacdPart @local {
    Mesh mesh;
    Mesh hull;
    float concavity;
    int depth;  // Recursion depth to prevent infinite splitting
}

fn void CoacdPart.free(&self) {
    self.mesh.free();
    self.hull.free();
}

const int COACD_MAX_DEPTH = 10;       // Maximum recursion depth
const int COACD_MAX_ITERATIONS = 100; // Maximum main loop iterations

// Cutting plane with axis and position
struct CutPlane @local {
    int axis;        // 0=X, 1=Y, 2=Z
    float position;
}

// ----------------------------------------------------------------------------
// Concavity Metrics (CoACD Paper Equation 6)
// Concavity = max(Hb, k * Rv)
// where Hb = boundary Hausdorff distance, Rv = volume-based radius
// ----------------------------------------------------------------------------

// Compute Rv: radius of sphere with volume equal to hull-mesh volume difference
// Rv = (3 * |V_hull - V_mesh| / (4 * PI))^(1/3)
fn float compute_rv(Mesh* mesh, Mesh* hull, float k) @local {
    double mesh_vol = mesh.get_volume();
    double hull_vol = hull.get_volume();
    double vol_diff = math::abs(hull_vol - mesh_vol);

    if (vol_diff < 1e-10) return 0;

    double rv = math::pow(3.0 * vol_diff / (4.0 * math::PI), 1.0/3.0);
    return (float)(rv * k);
}

// Project point onto triangle, return closest point
fn Vec3 project_point_to_triangle(Vec3 p, Vec3 v0, Vec3 v1, Vec3 v2) @local {
    Vec3 e0 = v1 - v0;
    Vec3 e1 = v2 - v0;
    Vec3 v = v0 - p;

    float a = e0.dot(e0);
    float b = e0.dot(e1);
    float c = e1.dot(e1);
    float d = e0.dot(v);
    float e = e1.dot(v);

    float det = a * c - b * b;
    float s = b * e - c * d;
    float t = b * d - a * e;

    if (s + t <= det) {
        if (s < 0) {
            if (t < 0) {
                // Region 4
                if (d < 0) {
                    s = math::clamp(-d / a, 0.0f, 1.0f);
                    t = 0;
                } else {
                    s = 0;
                    t = math::clamp(-e / c, 0.0f, 1.0f);
                }
            } else {
                // Region 3
                s = 0;
                t = math::clamp(-e / c, 0.0f, 1.0f);
            }
        } else if (t < 0) {
            // Region 5
            s = math::clamp(-d / a, 0.0f, 1.0f);
            t = 0;
        } else {
            // Region 0 (inside triangle)
            float inv_det = 1.0f / det;
            s *= inv_det;
            t *= inv_det;
        }
    } else {
        if (s < 0) {
            // Region 2
            float tmp0 = b + d;
            float tmp1 = c + e;
            if (tmp1 > tmp0) {
                float numer = tmp1 - tmp0;
                float denom = a - 2*b + c;
                s = math::clamp(numer / denom, 0.0f, 1.0f);
                t = 1 - s;
            } else {
                s = 0;
                t = math::clamp(-e / c, 0.0f, 1.0f);
            }
        } else if (t < 0) {
            // Region 6
            float tmp0 = b + e;
            float tmp1 = a + d;
            if (tmp1 > tmp0) {
                float numer = tmp1 - tmp0;
                float denom = a - 2*b + c;
                t = math::clamp(numer / denom, 0.0f, 1.0f);
                s = 1 - t;
            } else {
                t = 0;
                s = math::clamp(-d / a, 0.0f, 1.0f);
            }
        } else {
            // Region 1
            float numer = (c + e) - (b + d);
            float denom = a - 2*b + c;
            s = math::clamp(numer / denom, 0.0f, 1.0f);
            t = 1 - s;
        }
    }

    return v0 + e0 * s + e1 * t;
}

// Compute distance from point to nearest triangle in mesh
fn float point_to_mesh_distance(Vec3 p, Mesh* mesh) @local {
    float min_dist_sq = float.max;
    Vec3[] verts = mesh.vertices.array_view();

    foreach (tri : mesh.triangles) {
        Vec3 closest = project_point_to_triangle(p, verts[tri.x], verts[tri.y], verts[tri.z]);
        Vec3 diff = p - closest;
        float dist_sq = diff.dot(diff);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
        }
    }

    return math::sqrt(min_dist_sq);
}

// R2 low-discrepancy sequence for even sampling (Plastic constant based)
fn float[2] r2_sequence(int n) @local {
    const double G = 1.32471795724474602596;  // Plastic constant
    const double A1 = 1.0 / G;
    const double A2 = 1.0 / (G * G);

    return {
        (float)((0.5 + A1 * n) % 1.0),
        (float)((0.5 + A2 * n) % 1.0)
    };
}

// Sample point on triangle using barycentric coordinates
fn Vec3 sample_triangle(Vec3 v0, Vec3 v1, Vec3 v2, float u, float v) @local {
    // Convert from unit square to triangle using sqrt mapping
    float su = math::sqrt(u);
    float bary_u = 1.0f - su;
    float bary_v = v * su;
    float bary_w = 1.0f - bary_u - bary_v;

    return v0 * bary_u + v1 * bary_v + v2 * bary_w;
}

// Compute Hb: Hausdorff distance from hull boundary to mesh boundary
// Optimized: limits samples and uses vertex-based sampling for speed
fn float compute_hausdorff(Mesh* mesh, Mesh* hull, int resolution) @local {
    if (hull.triangles.len() == 0 || mesh.triangles.len() == 0) {
        return 0;
    }

    Vec3[] hull_verts = hull.vertices.array_view();
    float max_dist = 0;

    // Fast path: sample hull vertices first (much faster than triangle sampling)
    int vertex_samples = math::min((int)hull.vertices.len(), resolution / 2);
    int step = math::max(1, (int)hull.vertices.len() / vertex_samples);

    for (usz i = 0; i < hull.vertices.len(); i += step) {
        float dist = point_to_mesh_distance(hull_verts[i], mesh);
        if (dist > max_dist) {
            max_dist = dist;
        }
    }

    // If we have budget left, sample some triangle centers
    int remaining_samples = resolution - vertex_samples;
    if (remaining_samples > 0 && hull.triangles.len() > 0) {
        int tri_step = math::max(1, (int)hull.triangles.len() / remaining_samples);

        for (usz i = 0; i < hull.triangles.len(); i += tri_step) {
            Triangle tri = hull.triangles[i];
            Vec3 center = (hull_verts[tri.x] + hull_verts[tri.y] + hull_verts[tri.z]) / 3.0f;

            float dist = point_to_mesh_distance(center, mesh);
            if (dist > max_dist) {
                max_dist = dist;
            }
        }
    }

    return max_dist;
}

// Combined concavity metric: max(Hb, k*Rv)
fn float compute_concavity(Mesh* mesh, Mesh* hull, CoacdParameters* params) @local {
    float rv = compute_rv(mesh, hull, params.k);
    float hb = compute_hausdorff(mesh, hull, params.resolution);
    return math::max(hb, rv);
}

// ----------------------------------------------------------------------------
// Cutting Plane Generation and Evaluation
// ----------------------------------------------------------------------------

// Generate axis-aligned cutting planes within mesh bounds
fn List{CutPlane} generate_cutting_planes(Aabb3 bounds, int nodes_per_axis) @local {
    List{CutPlane} planes;

    Vec3 size = bounds.size();

    for (int axis = 0; axis < 3; axis++) {
        float axis_min, axis_max;
        switch (axis) {
            case 0: axis_min = bounds.min.x; axis_max = bounds.max.x;
            case 1: axis_min = bounds.min.y; axis_max = bounds.max.y;
            case 2: axis_min = bounds.min.z; axis_max = bounds.max.z;
        }

        float range = axis_max - axis_min;
        if (range < 1e-6) continue;

        // Generate evenly spaced planes
        for (int i = 1; i <= nodes_per_axis; i++) {
            float t = (float)i / (float)(nodes_per_axis + 1);
            float pos = axis_min + range * t;
            planes.push({ .axis = axis, .position = pos });
        }
    }

    return planes;
}

// Split mesh by axis-aligned plane
fn void split_mesh_by_plane(Mesh* mesh, CutPlane cut, Mesh* pos_mesh, Mesh* neg_mesh) @local {
    Vec3 normal = {0, 0, 0};
    switch (cut.axis) {
        case 0: normal.x = 1;
        case 1: normal.y = 1;
        case 2: normal.z = 1;
    }

    Plane plane = create_plane(normal, cut.position);
    clip_mesh(mesh, plane, pos_mesh, neg_mesh);
}

// Fast cut evaluation: counts vertices on each side (no expensive concavity)
// Returns a score where lower is better (balanced cuts score lower)
fn float evaluate_cut_fast(Mesh* mesh, CutPlane cut) @local {
    Vec3 normal = {0, 0, 0};
    switch (cut.axis) {
        case 0: normal.x = 1;
        case 1: normal.y = 1;
        case 2: normal.z = 1;
    }

    int pos_count = 0;
    int neg_count = 0;

    foreach (v : mesh.vertices) {
        float dist;
        switch (cut.axis) {
            case 0: dist = v.x - cut.position;
            case 1: dist = v.y - cut.position;
            case 2: dist = v.z - cut.position;
        }

        if (dist > 0.001f) {
            pos_count++;
        } else if (dist < -0.001f) {
            neg_count++;
        }
    }

    // Invalid cut: one side is empty
    if (pos_count < 4 || neg_count < 4) {
        return float.max;
    }

    // Score: prefer balanced cuts (ratio close to 1)
    float ratio = (float)math::max(pos_count, neg_count) / (float)math::min(pos_count, neg_count);
    return ratio;
}

// ----------------------------------------------------------------------------
// MCTS (Monte Carlo Tree Search) for Finding Best Cut
// Based on CoACD-rs: https://github.com/Jondolf/CoACD-rs/blob/main/src/mcts.rs
// ----------------------------------------------------------------------------

// Compute total Rv cost for a split (used in MCTS evaluation)
// Returns MAX of both parts' Rv (matching CoACD-rs implementation)
fn float compute_total_rv(Mesh* pos_mesh, Mesh* pos_hull, Mesh* neg_mesh, Mesh* neg_hull, float k) @local {
    float pos_rv = compute_rv(pos_mesh, pos_hull, k);
    float neg_rv = compute_rv(neg_mesh, neg_hull, k);
    return math::max(pos_rv, neg_rv);
}

// Find concave region bounds: find the bounding box of hull points that are far from mesh
// This identifies where the "hole" is in the shape
struct ConcaveRegion @local {
    Vec3 min;
    Vec3 max;
    Vec3 center;
    bool valid;
}

fn ConcaveRegion find_concave_region(Mesh* mesh, Mesh* hull) @local {
    ConcaveRegion region = {
        .min = {float.max, float.max, float.max},
        .max = {-float.max, -float.max, -float.max},
        .center = {0, 0, 0},
        .valid = false
    };

    Vec3[] hull_verts = hull.vertices.array_view();
    float total_weight = 0;
    int concave_points = 0;

    // Sample points inside the hull (on triangle centers) to find the hole
    foreach (tri : hull.triangles) {
        Vec3 center = (hull_verts[tri.x] + hull_verts[tri.y] + hull_verts[tri.z]) / 3.0f;
        float dist = point_to_mesh_distance(center, mesh);

        if (dist > 0.05f) {  // Point is significantly inside the hole
            region.valid = true;
            concave_points++;

            // Expand bounds
            region.min.x = math::min(region.min.x, center.x);
            region.min.y = math::min(region.min.y, center.y);
            region.min.z = math::min(region.min.z, center.z);
            region.max.x = math::max(region.max.x, center.x);
            region.max.y = math::max(region.max.y, center.y);
            region.max.z = math::max(region.max.z, center.z);

            region.center += center * dist;
            total_weight += dist;
        }
    }

    if (total_weight > 0.001f) {
        region.center /= total_weight;
    } else {
        region.center = mesh.get_center();
    }

    // Debug output only for initial decomposition (not every iteration)
    // if (region.valid) {
    //     io::printfn("  Concave region: X=(%.2f to %.2f), points=%d",
    //                 region.min.x, region.max.x, concave_points);
    // }

    return region;
}

// Evaluate a cutting plane by computing Rv cost of resulting parts
// Prefer cuts along axes where the hole has significant extent (to go around the hole)
fn float evaluate_cut_rv(Mesh* mesh, Mesh* hull, CutPlane cut, float k, ConcaveRegion* concave) @local {
    Mesh pos_mesh;
    Mesh neg_mesh;
    split_mesh_by_plane(mesh, cut, &pos_mesh, &neg_mesh);

    // Both parts must be valid
    if (pos_mesh.vertices.len() < 4 || neg_mesh.vertices.len() < 4) {
        pos_mesh.free();
        neg_mesh.free();
        return float.max;
    }

    // Compute hulls
    Mesh pos_hull = quickhull(pos_mesh.vertices.array_view());
    Mesh neg_hull = quickhull(neg_mesh.vertices.array_view());

    if (pos_hull.vertices.len() < 4 || neg_hull.vertices.len() < 4) {
        pos_mesh.free();
        neg_mesh.free();
        pos_hull.free();
        neg_hull.free();
        return float.max;
    }

    // Compute total Rv cost
    float rv_cost = compute_total_rv(&pos_mesh, &pos_hull, &neg_mesh, &neg_hull, k);

    // If we found a concave region (hole), prefer cuts that go AROUND it
    if (concave.valid) {
        // Get hole extent in each axis
        float hole_x = concave.max.x - concave.min.x;
        float hole_y = concave.max.y - concave.min.y;
        float hole_z = concave.max.z - concave.min.z;

        // Get hole extent in the cut axis
        float hole_extent;
        switch (cut.axis) {
            case 0: hole_extent = hole_x;  // X-axis cut
            case 1: hole_extent = hole_y;  // Y-axis cut
            case 2: hole_extent = hole_z;  // Z-axis cut
        }

        // For a through-hole (like in a frame), the hole has:
        // - Large extent in 2 axes (X and Z for a vertical hole)
        // - Small extent in 1 axis (Y - the hole goes through)
        //
        // We want cuts along axes where the hole HAS extent (X and Z)
        // because those cuts can separate the walls around the hole.
        // Cuts along axes where the hole has NO extent (Y) just slice
        // through the frame and create parts that still have the hole.

        float max_hole_extent = math::max(hole_x, math::max(hole_y, hole_z));

        if (max_hole_extent > 0.1f) {
            // Penalize cuts along axes where hole has NO significant extent
            // These cuts don't help separate the concave region
            if (hole_extent < max_hole_extent * 0.3f) {
                // Strong penalty for cuts that don't help
                rv_cost *= 3.0f;
            } else {
                // For axes with hole extent, prefer cuts near the hole edges
                float hole_min, hole_max;
                switch (cut.axis) {
                    case 0: hole_min = concave.min.x; hole_max = concave.max.x;
                    case 1: hole_min = concave.min.y; hole_max = concave.max.y;
                    case 2: hole_min = concave.min.z; hole_max = concave.max.z;
                }

                float dist_to_min_edge = math::abs(cut.position - hole_min);
                float dist_to_max_edge = math::abs(cut.position - hole_max);
                float dist_to_edge = math::min(dist_to_min_edge, dist_to_max_edge);

                // Small penalty for cuts far from edges
                // Cuts right at the edge get no penalty
                rv_cost += dist_to_edge * rv_cost * 0.5f;
            }
        }
    }

    pos_mesh.free();
    neg_mesh.free();
    pos_hull.free();
    neg_hull.free();

    return rv_cost;
}

// MCTS Node for tree search
struct MCTSNode @local {
    CutPlane plane;
    float quality;           // Best cost found (lower is better)
    int visits;
    int parent_idx;          // Index in node array, -1 for root
    List{int} children;      // Indices of child nodes
    bool expanded;
}

fn void MCTSNode.free(&self) {
    self.children.free();
}

// MCTS Part: a mesh segment being considered for splitting
struct MCTSPart @local {
    Mesh mesh;
    Aabb3 aabb;
    float cost;
    int next_candidate;
    List{CutPlane} candidates;
}

fn void MCTSPart.free(&self) {
    self.mesh.free();
    self.candidates.free();
}

// Find best cutting plane using Monte Carlo Tree Search
fn CutPlane mcts_find_best_cut(Mesh* mesh, Mesh* hull, Aabb3 bounds, CoacdParameters* params) @local {
    List{CutPlane} candidates = generate_cutting_planes(bounds, params.mcts_nodes);
    defer candidates.free();

    if (candidates.len() == 0) {
        return { .axis = 0, .position = (bounds.min.x + bounds.max.x) * 0.5f };
    }

    // For complex meshes, skip expensive concave region detection
    ConcaveRegion concave;
    if (hull.triangles.len() < 200) {
        concave = find_concave_region(mesh, hull);
    } else {
        concave = { .valid = false };
    }

    // Find best plane by evaluating cost for each candidate
    CutPlane best_cut = candidates[0];
    float best_cost = float.max;

    // For large meshes, use fast evaluation (no hull computation)
    bool use_fast_eval = mesh.triangles.len() > 200;

    foreach (cut : candidates) {
        float cost;
        if (use_fast_eval) {
            cost = evaluate_cut_fast(mesh, cut);
        } else {
            cost = evaluate_cut_rv(mesh, hull, cut, params.k, &concave);
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_cut = cut;
        }
    }

    // Skip ternary refinement for large meshes (too slow)
    if (!use_fast_eval) {
        best_cut = ternary_refine_plane(mesh, hull, bounds, best_cut, best_cost, params, &concave);
    }

    return best_cut;
}

// Ternary search to refine plane position along its axis
fn CutPlane ternary_refine_plane(Mesh* mesh, Mesh* hull, Aabb3 bounds, CutPlane initial, float initial_cost, CoacdParameters* params, ConcaveRegion* concave) @local {
    CutPlane best = initial;
    float best_cost = initial_cost;

    // Get axis bounds
    float axis_min, axis_max;
    switch (initial.axis) {
        case 0: axis_min = bounds.min.x; axis_max = bounds.max.x;
        case 1: axis_min = bounds.min.y; axis_max = bounds.max.y;
        case 2: axis_min = bounds.min.z; axis_max = bounds.max.z;
    }

    // Search interval around the initial position
    float interval = (axis_max - axis_min) / (float)(params.mcts_nodes + 1);
    float left = math::max(axis_min + 0.01f, initial.position - interval);
    float right = math::min(axis_max - 0.01f, initial.position + interval);

    if (left >= right) {
        return best;
    }

    const float EPSILON = 0.0001f;
    const int MAX_ITER = 10;

    // Ternary search
    for (int iter = 0; iter < MAX_ITER && (right - left) > EPSILON; iter++) {
        float third = (right - left) / 3.0f;
        float mid1 = left + third;
        float mid2 = right - third;

        CutPlane plane1 = { .axis = initial.axis, .position = mid1 };
        CutPlane plane2 = { .axis = initial.axis, .position = mid2 };

        float cost1 = evaluate_cut_rv(mesh, hull, plane1, params.k, concave);
        float cost2 = evaluate_cut_rv(mesh, hull, plane2, params.k, concave);

        if (cost1 < cost2) {
            right = mid2;
            if (cost1 < best_cost) {
                best_cost = cost1;
                best = plane1;
            }
        } else {
            left = mid1;
            if (cost2 < best_cost) {
                best_cost = cost2;
                best = plane2;
            }
        }
    }

    return best;
}

// ----------------------------------------------------------------------------
// Main CoACD Decomposition Function
// ----------------------------------------------------------------------------

// Decompose a mesh into approximately convex parts
fn List{Mesh} coacd_decompose(Mesh* mesh, CoacdParameters params = COACD_MEDIUM) {
    List{Mesh} result;

    if (mesh.vertices.len() < 4 || mesh.triangles.len() < 2) {
        return result;
    }

    // Validate mesh to fix any invalid triangle indices
    mesh.validate();

    // For very complex meshes (>1000 tris), just use convex hull
    // Full decomposition is too slow for realtime
    if (mesh.triangles.len() > 1000) {
        io::printfn("CoACD: Very large mesh (%d tris, %d verts), using convex hull only",
                    mesh.triangles.len(), mesh.vertices.len());

        // For extremely large meshes, subsample vertices for hull
        Vec3[] verts = mesh.vertices.array_view();
        if (verts.len > 500) {
            // Subsample: limit to ~200 vertices
            int step = (int)(verts.len / 200);
            if (step < 1) step = 1;
            List{Vec3} sampled;
            for (usz i = 0; i < verts.len; i += step) {
                sampled.push(verts[i]);
            }
            io::printfn("  Subsampled to %d vertices for hull", sampled.len());
            io::printfn("  Computing quickhull...");
            Mesh hull = quickhull(sampled.array_view());
            io::printfn("  Quickhull complete: %d verts, %d tris", hull.vertices.len(), hull.triangles.len());
            sampled.free();
            if (hull.vertices.len() >= 4) {
                result.push(hull);
            }
        } else {
            Mesh hull = quickhull(verts);
            if (hull.vertices.len() >= 4) {
                result.push(hull);
            }
        }
        io::printfn("CoACD: Returning %d convex hulls (fast path)", result.len());
        return result;
    }

    // For moderately complex meshes, use simplified parameters
    if (mesh.triangles.len() > 200) {
        io::printfn("CoACD: Large mesh (%d tris), using fast parameters", mesh.triangles.len());
        params.concavity_threshold = 0.08;  // Slightly higher threshold
        params.mcts_nodes = 3;              // Fewer candidate planes
        params.mcts_iterations = 20;        // Fewer iterations
        params.mcts_max_depth = 2;          // Limit depth
        params.max_convex_hulls = 8;        // Reasonable limit
    }

    // Normalize threshold by mesh diagonal for scale invariance
    Aabb3 bounds = mesh.get_aabb();
    float diagonal = bounds.size().length();
    float abs_threshold = params.concavity_threshold * diagonal;

    io::printfn("CoACD: Starting decomposition (threshold=%.4f, abs=%.4f)",
                params.concavity_threshold, abs_threshold);

    // Processing queue
    List{CoacdPart} queue;
    defer {
        foreach (&part : queue) {
            part.free();
        }
        queue.free();
    }

    // Compute initial hull and concavity
    Mesh initial_hull = quickhull(mesh.vertices.array_view());
    if (initial_hull.vertices.len() < 4) {
        io::printfn("CoACD: Failed to compute initial hull");
        initial_hull.free();
        return result;
    }

    float initial_concavity = compute_concavity(mesh, &initial_hull, &params);
    io::printfn("CoACD: Initial concavity = %.4f (mesh: %d verts, %d tris)",
                initial_concavity, mesh.vertices.len(), mesh.triangles.len());

    // If already convex enough, return the hull
    if (initial_concavity <= abs_threshold) {
        io::printfn("CoACD: Mesh is already convex enough");
        result.push(initial_hull);
        return result;
    }

    // Create initial part
    Mesh mesh_copy;
    mesh_copy.vertices.push_all(mesh.vertices.array_view());
    mesh_copy.triangles.push_all(mesh.triangles.array_view());

    queue.push({
        .mesh = mesh_copy,
        .hull = initial_hull,
        .concavity = initial_concavity,
        .depth = 0
    });

    // Process queue: always split the part with highest concavity
    int iterations = 0;
    while (queue.len() > 0 && result.len() < params.max_convex_hulls && iterations < COACD_MAX_ITERATIONS) {
        iterations++;

        // Find part with highest concavity
        usz worst_idx = 0;
        float worst_concavity = 0;

        for (usz i = 0; i < queue.len(); i++) {
            if (queue[i].concavity > worst_concavity) {
                worst_concavity = queue[i].concavity;
                worst_idx = i;
            }
        }

        CoacdPart part = queue[worst_idx];
        queue.remove_at(worst_idx);

        // If concavity is acceptable, add hull to result
        if (part.concavity <= abs_threshold) {
            result.push(part.hull);
            part.mesh.free();
            continue;
        }

        // If max depth reached, accept current hull
        if (part.depth >= COACD_MAX_DEPTH) {
            result.push(part.hull);
            part.mesh.free();
            continue;
        }

        // Find best cutting plane using MCTS
        Aabb3 part_bounds = part.mesh.get_aabb();
        CutPlane best_cut = mcts_find_best_cut(&part.mesh, &part.hull, part_bounds, &params);

        // Apply the cut
        Mesh pos_mesh;
        Mesh neg_mesh;
        split_mesh_by_plane(&part.mesh, best_cut, &pos_mesh, &neg_mesh);

        // Validate split - need minimum triangles, not just vertices
        bool pos_valid = pos_mesh.vertices.len() >= 4 && pos_mesh.triangles.len() >= 4;
        bool neg_valid = neg_mesh.vertices.len() >= 4 && neg_mesh.triangles.len() >= 4;

        if (!pos_valid && !neg_valid) {
            // Can't split further - accept current hull
            result.push(part.hull);
            part.mesh.free();
            pos_mesh.free();
            neg_mesh.free();
            continue;
        }

        int new_depth = part.depth + 1;

        // Process valid parts
        if (pos_valid) {
            Mesh pos_hull = quickhull(pos_mesh.vertices.array_view());
            if (pos_hull.vertices.len() >= 4) {
                float pos_concavity = compute_concavity(&pos_mesh, &pos_hull, &params);

                // Check for progress (must improve by at least 1%)
                if (pos_concavity < part.concavity * 0.99f) {
                    queue.push({
                        .mesh = pos_mesh,
                        .hull = pos_hull,
                        .concavity = pos_concavity,
                        .depth = new_depth
                    });
                } else {
                    // No improvement - accept hull
                    result.push(pos_hull);
                    pos_mesh.free();
                }
            } else {
                pos_mesh.free();
                pos_hull.free();
            }
        } else {
            pos_mesh.free();
        }

        if (neg_valid) {
            Mesh neg_hull = quickhull(neg_mesh.vertices.array_view());
            if (neg_hull.vertices.len() >= 4) {
                float neg_concavity = compute_concavity(&neg_mesh, &neg_hull, &params);

                // Check for progress (must improve by at least 1%)
                if (neg_concavity < part.concavity * 0.99f) {
                    queue.push({
                        .mesh = neg_mesh,
                        .hull = neg_hull,
                        .concavity = neg_concavity,
                        .depth = new_depth
                    });
                } else {
                    // No improvement - accept hull
                    result.push(neg_hull);
                    neg_mesh.free();
                }
            } else {
                neg_mesh.free();
                neg_hull.free();
            }
        } else {
            neg_mesh.free();
        }

        // Clean up original part
        part.mesh.free();
        part.hull.free();
    }

    // If we hit iteration limit, log it
    if (iterations >= COACD_MAX_ITERATIONS) {
        io::printfn("CoACD: Hit iteration limit (%d), accepting remaining parts", COACD_MAX_ITERATIONS);
    }

    // Accept remaining queue items (up to hull limit)
    foreach (&part : queue) {
        if (result.len() < params.max_convex_hulls && part.hull.vertices.len() >= 4) {
            result.push(part.hull);
            part.mesh.free();
            part.hull = {};  // Prevent double-free in defer
        }
    }

    io::printfn("CoACD: Decomposition complete - %d convex hulls", result.len());

    // Debug: print bounds of each hull to verify they don't overlap incorrectly
    foreach (i, &hull : result) {
        Aabb3 hull_bounds = hull.get_aabb();
        io::printfn("  Hull %d: verts=%d, bounds min=(%.2f,%.2f,%.2f) max=(%.2f,%.2f,%.2f)",
            i, hull.vertices.len(),
            hull_bounds.min.x, hull_bounds.min.y, hull_bounds.min.z,
            hull_bounds.max.x, hull_bounds.max.y, hull_bounds.max.z);
    }

    return result;
}

// Convenience wrapper with default parameters
fn List{Mesh} decompose(Mesh* mesh, CoacdParameters params = COACD_MEDIUM) {
    return coacd_decompose(mesh, params);
}

// ----------------------------------------------------------------------------
// Inertia Tensor Helper (used by Mesh.inertia_tensor)
// ----------------------------------------------------------------------------

fn Matrix3f compute_inertia_contribution(Vec3 a, Vec3 b, Vec3 c, float density) @local {
    Matrix3f inertia_tensor;

    float detJ = a.dot(b.cross(c));

    float v_mult_diag = (density * detJ) / 60.0f;
    float v_mult_off_diag = (density * detJ) / 120.0f;

    // Diagonal elements (Moments of Inertia)
    inertia_tensor.m00 = v_mult_diag * (
        (a.y*a.y + a.y*b.y + b.y*b.y + a.y*c.y + b.y*c.y + c.y*c.y) +
        (a.z*a.z + a.z*b.z + b.z*b.z + a.z*c.z + b.z*c.z + c.z*c.z)
    );

    inertia_tensor.m11 = v_mult_diag * (
        (a.x*a.x + a.x*b.x + b.x*b.x + a.x*c.x + b.x*c.x + c.x*c.x) +
        (a.z*a.z + a.z*b.z + b.z*b.z + a.z*c.z + b.z*c.z + c.z*c.z)
    );

    inertia_tensor.m22 = v_mult_diag * (
        (a.x*a.x + a.x*b.x + b.x*b.x + a.x*c.x + b.x*c.x + c.x*c.x) +
        (a.y*a.y + a.y*b.y + b.y*b.y + a.y*c.y + b.y*c.y + c.y*c.y)
    );

    inertia_tensor.m01 = -v_mult_off_diag * (
        (2*a.x*a.y + b.x*a.y + c.x*a.y) + (a.x*b.y + 2*b.x*b.y + c.x*b.y) + (a.x*c.y + b.x*c.y + 2*c.x*c.y)
    );

    inertia_tensor.m02 = -v_mult_off_diag * (
        (2*a.x*a.z + b.x*a.z + c.x*a.z) + (a.x*b.z + 2*b.x*b.z + c.x*b.z) + (a.x*c.z + b.x*c.z + 2*c.x*c.z)
    );

    inertia_tensor.m12 = -v_mult_off_diag * (
        (2*a.y*a.z + b.y*a.z + c.y*a.z) + (a.y*b.z + 2*b.y*b.z + c.y*b.z) + (a.y*c.z + b.y*c.z + 2*c.y*c.z)
    );

    inertia_tensor.m10 = inertia_tensor.m01;
    inertia_tensor.m20 = inertia_tensor.m02;
    inertia_tensor.m21 = inertia_tensor.m12;

    return inertia_tensor;
}

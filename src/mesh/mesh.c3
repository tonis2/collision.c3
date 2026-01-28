module collision;
import std::math, std::collections::list, std::io;

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
    foreach (v : self.vertices) center += v;
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
    float total_volume = 0;
    Vec3 weighted_com = {0, 0, 0};

    foreach (triangle: self.triangles) {
        Vec3 v1 = self.vertices[triangle.x].xyz;
        Vec3 v2 = self.vertices[triangle.y].xyz;
        Vec3 v3 = self.vertices[triangle.z].xyz;

        // Signed volume of tetrahedron from origin to triangle
        float detJ = v1.dot(v2.cross(v3));
        total_volume += detJ;

        // Centroid of tetrahedron is at (v1 + v2 + v3) / 4 (origin is 4th vertex)
        // Weight by tetrahedron volume (detJ/6), but we factor out the /6
        weighted_com.x += detJ * (v1.x + v2.x + v3.x);
        weighted_com.y += detJ * (v1.y + v2.y + v3.y);
        weighted_com.z += detJ * (v1.z + v2.z + v3.z);
    }

    if (math::abs(total_volume) > VEC_EPSILON) {
        return weighted_com / (4.0f * total_volume);
    }
    return {0, 0, 0};
}

fn Matrix3f Mesh.inertia_tensor(&self, float mass) @dynamic {
    Matrix3f inertia = {};  // Start with zeros, not identity

    // Compute with density=1, then scale by actual density
    foreach (triangle: self.triangles) {
        Vec3 v1 = self.vertices[triangle.x].xyz;
        Vec3 v2 = self.vertices[triangle.y].xyz;
        Vec3 v3 = self.vertices[triangle.z].xyz;

        inertia += compute_inertia_contribution(v1, v2, v3, 1.0f);
    }

    // Scale inertia by actual density (mass / volume)
    double volume = self.get_volume();
    if (volume > VEC_EPSILON) {
        float density = mass / (float)volume;
        inertia.m00 *= density;
        inertia.m11 *= density;
        inertia.m22 *= density;
        inertia.m01 *= density;
        inertia.m02 *= density;
        inertia.m12 *= density;
        inertia.m10 *= density;
        inertia.m20 *= density;
        inertia.m21 *= density;
    }

    // Parallel axis theorem: shift from origin to center of mass
    // I_cm = I_origin - m * d^2  (for diagonal)
    // I_xy_cm = I_xy_origin - m * x * y  (for off-diagonal)
    Vec3 c = self.center_of_mass();

    inertia.m00 -= mass * (c.y * c.y + c.z * c.z);
    inertia.m11 -= mass * (c.x * c.x + c.z * c.z);
    inertia.m22 -= mass * (c.x * c.x + c.y * c.y);

    inertia.m01 -= mass * c.x * c.y;
    inertia.m02 -= mass * c.x * c.z;
    inertia.m12 -= mass * c.y * c.z;

    inertia.m10 = inertia.m01;
    inertia.m20 = inertia.m02;
    inertia.m21 = inertia.m12;

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

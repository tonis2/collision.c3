module collision;
import std::math, std::collections::list;


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
    Vec3 e1 = vertices[self.y] - vertices[self.x];
    Vec3 e2 = vertices[self.z] - vertices[self.x];
    return e1.cross(e2).normalize();
}

fn Vec3 Triangle.get_center(Triangle* self, Vec3[] vertices) {
    return (vertices[self.x] + vertices[self.y] + vertices[self.z]) / 3.0;
}

fn double Triangle.get_area(Triangle* self, Vec3[] vertices) {
    Vec3 e1 = vertices[self.y] - vertices[self.x];
    Vec3 e2 = vertices[self.z] - vertices[self.x];
    return e1.cross(e2).length() * 0.5;
}

// Mesh structure
struct Mesh (CollisionShape) {
    List{Vec3} vertices;
    List{Triangle} triangles;
}

fn Mesh create_mesh(Vec3[] vertices, uint[] indices) {
	Mesh mesh;
	mesh.vertices.push_all(vertices);
	mesh.set_indices(indices);
	return mesh;
}

fn void Mesh.free(&self) @dynamic {
    self.vertices.free();
    self.triangles.free();
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

fn double Mesh.get_volume(&self) {
    double volume = 0.0;
    
    foreach (tri : self.triangles) {
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
    
    foreach (tri : self.triangles) {
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
    Vec3 furthest_point;
    float max_distance = -float.max;

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

        inertia += compute_origin(v1, v2, v3, density);
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

// MeshClipper functions
fn void clip_triangle(Mesh* mesh, Triangle tri, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh) @local {
    Vec3 v0 = mesh.vertices[tri.x];
    Vec3 v1 = mesh.vertices[tri.y];
    Vec3 v2 = mesh.vertices[tri.z];
    
    double d0 = plane.distance(v0);
    double d1 = plane.distance(v1);
    double d2 = plane.distance(v2);
    
    const double EPS = 1e-6;
    
    int pos_count = (d0 > EPS ? 1 : 0) + (d1 > EPS ? 1 : 0) + (d2 > EPS ? 1 : 0);
    int neg_count = (d0 < -EPS ? 1 : 0) + (d1 < -EPS ? 1 : 0) + (d2 < -EPS ? 1 : 0);
    
    if (pos_count == 3) {
        pos_mesh.add_triangle(v0, v1, v2);
    } else if (neg_count == 3) {
        neg_mesh.add_triangle(v0, v1, v2);
    } else if (pos_count == 0 || neg_count == 0) {
        pos_mesh.add_triangle(v0, v1, v2);
        neg_mesh.add_triangle(v0, v1, v2);
    } else {
        // Simplified split - add to dominant side
        double avg_dist = (d0 + d1 + d2) / 3.0;
        if (avg_dist > 0) {
            pos_mesh.add_triangle(v0, v1, v2);
        } else {
            neg_mesh.add_triangle(v0, v1, v2);
        }
    }
}

fn void clip_mesh(Mesh* mesh, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh) @local {
    foreach (tri : mesh.triangles) {
        clip_triangle(mesh, tri, plane, pos_mesh, neg_mesh);
    }
}

// ConvexDecomposer
struct DecomposeParameters {
    int max_convex_hulls;
    int max_vertices_per_hull;
    double concavity_threshold;
    int max_depth;
}

const DecomposeParameters DEFAULT_PARAMETERS = {
	.max_convex_hulls = 32,
	.max_vertices_per_hull = 64,
	.concavity_threshold = 0.001,
	.max_depth = 10
};

struct MeshNode @local {
    Mesh mesh;
    int depth;
}

fn Plane find_best_split_plane(Mesh* mesh) @local {
    if (mesh.triangles.len() == 0) {
        return create_plane({1, 0, 0}, 0);
    }
    
    double max_area = 0;
    usz best_tri = 0;
    
    Vec3[] verts = mesh.vertices.array_view();
    for (usz i = 0; i < mesh.triangles.len(); i++) {
        double area = mesh.triangles[i].get_area(verts);
        if (area > max_area) {
            max_area = area;
            best_tri = i;
        }
    }
    
    Triangle tri = mesh.triangles[best_tri];
    Vec3 normal = tri.get_normal(verts);
    Vec3 center = tri.get_center(verts);
    
    return create_plane(normal, normal.dot(center));
}

fn List{Mesh} decompose(Mesh* mesh, DecomposeParameters params = DEFAULT_PARAMETERS) {
    List{Mesh} result;
    
    if (mesh.is_convex()) {
        result.push(*mesh);
        return result;
    }
    
    // Simple queue using List
    List{MeshNode} queue;
    defer queue.free();
    
    MeshNode start_node = { *mesh, 0 };
    queue.push(start_node);
    
    while (queue.len() > 0 && result.len() < params.max_convex_hulls) {
        MeshNode node = queue.get(0);
        queue.remove_at(0);
        
        if (node.depth >= params.max_depth || node.mesh.is_convex()) {
            result.push(node.mesh);
            continue;
        }
        
        Plane split_plane = find_best_split_plane(&node.mesh);
        Mesh pos_part;
        Mesh neg_part;
        clip_mesh(&node.mesh, split_plane, &pos_part, &neg_part);

        if (pos_part.vertices.len() > 0 && pos_part.triangles.len() > 0) {
            MeshNode new_node = { pos_part, node.depth + 1 };
            queue.push(new_node);
        } else {
            pos_part.free();
        }

        if (neg_part.vertices.len() > 0 && neg_part.triangles.len() > 0) {
            MeshNode new_node = { neg_part, node.depth + 1 };
            queue.push(new_node);
        } else {
            neg_part.free();
        }
        
        node.mesh.free();
    }
    
    // Clean up remaining queue items
    foreach (node : queue) {
        node.mesh.free();
    }
    
    return result;
}


fn Matrix3f compute_origin(Vec3 a, Vec3 b, Vec3 c, float density) @local {
    Matrix3f inertia_tensor;

    float detJ = a.dot(b.cross(c));

    float v_mult_diag = (density * detJ) / (float)60.0;
    float v_mult_off_diag = (density * detJ) / (float)120.0;

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
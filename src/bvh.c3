module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

const int MAX_TRIS_PER_LEAF = 8;

struct Triangle (CollisionShape) {
    Vec3 v0, v1, v2;
}

// Represents a node in the BVH tree
struct BVHNode {
    Aabb3 box;
    BVHNode* left;
    BVHNode* right;
    Vec4* positions; 
    usz count;
}

fn bool BVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void BVHNode.free(&self) {
    List{BVHNode*} lefts;
    List{BVHNode*} rights;

    defer {
        lefts.free();
        rights.free();
    }

    BVHNode* left = self.left;
    BVHNode* right = self.right;

    while (left != null) {
        lefts.push(left);
        left = left.left;
    }

    while (right != null) {
        rights.push(right);
        right = right.right;
    }

    foreach (value: lefts) mem::free(value);
    foreach (value: rights) mem::free(value);
}

struct MinMax @local {
    Number min;
    Number max;
}

fn void MinMax.expand(&self, Number value) {
    self.min = math::min(self.min, value);
    self.max = math::max(self.max, value);
}

fn void MinMax.expand_by_array(&self, Number[] values) {
    foreach (value: values) self.expand(value);
}

<*
 @require positions != null : `Has no positions attached`
*>
fn void BVHNode.build_recursive(&self, Vec4* positions, usz count) {
    for (usz i = 0; i < count; i += 3) {
        Vec3 v0 = positions[i].xyz;
        Vec3 v1 = positions[i + 1].xyz;
        Vec3 v2 = positions[i + 2].xyz;

        self.box.expand_by_point(v0);
        self.box.expand_by_point(v1);
        self.box.expand_by_point(v2);
    }

    if (count <= MAX_TRIS_PER_LEAF * 3) {
        self.positions = positions;
        self.count = count;
        return;
    }
    
    Vec3 extent = self.box.extent();
    uint axis = 0;

    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;

    Number split_pos = self.box.min[axis] + extent[axis] * 0.5;

    BVHNode left;
    BVHNode right;

    for (usz i = 0; i < count; i += 3) {
        Vec3 v0 = positions[i].xyz;
        Vec3 v1 = positions[i + 1].xyz;
        Vec3 v2 = positions[i + 2].xyz;
        Vec3 centroid = (v0 + v1 + v2) / 3.0;

        if (centroid[axis] < split_pos) {
            left.count += 3;
        } else {
            right.count += 3;
        }
    }

    // Handle cases where the split results in all triangles on one side.
    if (left.count == 0 || right.count == 0) {
        left.count = 0;
        right.count = 0;

        usz half = count / 2;

        for (usz i; i < count; i += 3) {
            if (i < half) { left.count += 3; }
            else { right.count += 3; }
        }
    }

    left.positions = positions;
    right.positions = positions + left.count;

    self.left = @clone(left);
    self.right = @clone(right);

    self.left.build_recursive(left.positions, left.count);
    self.right.build_recursive(right.positions, right.count);
}

fn HitInfo BVHNode.ray_intersects(&self, Ray ray) {
    HitInfo hit_info = {.collides = false};

    if (!self.box.intersect_ray(ray)) {
        return hit_info;
    }
    
    if (self.is_leaf()) {
        for (usz i = 0; i < self.count; i += 3) {
            Vec3 v0 = self.positions[i].xyz;
            Vec3 v1 = self.positions[i + 1].xyz;
            Vec3 v2 = self.positions[i + 2].xyz;

            Triangle triangle = {v0, v1, v2};
            HitInfo triangle_hit = triangle.intersect_ray(ray);
       
            if (triangle_hit.collides) {
                triangle_hit.positions = self.positions;
                return triangle_hit;
            };
        }
        
        return hit_info;
    }
    
    HitInfo left = self.left.ray_intersects(ray);
    HitInfo right = self.right.ray_intersects(ray);

    if (left.collides) return left;
    if (right.collides) return right;

    return hit_info;
}

fn HitInfo BVHNode.box_intersects(&self, Aabb3 box) {
    HitInfo hit_info = {.collides = false};

    if (!self.box.collides(box)) {
        return hit_info;
    }

    if (self.is_leaf()) {
        for (usz i; i < self.count; i += 3) {
            Vec3 v0 = self.positions[i].xyz;
            Vec3 v1 = self.positions[i + 1].xyz;
            Vec3 v2 = self.positions[i + 2].xyz;

            Triangle triangle = {v0, v1, v2};

            if (triangle.collides_aabb(box, full: false)) {
                return {.collides = true, .normal = triangle.v0.cross(triangle.v1), .positions = self.positions};
            }
        }

        return hit_info;
    }

    HitInfo left = self.left.box_intersects(box);
    HitInfo right = self.right.box_intersects(box);

    if (left.collides) return left;
    if (right.collides) return right;

    return hit_info;
}


// fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("v1 %s, v2 %s v3 %s", self.v0, self.v1, self.v2)!;

fn bool Triangle.collides_aabb(&self, Aabb3 box, bool full = true) @local {
    Vec3 center = box.center();
    Vec3 extent = box.extent();

    if (full) {
        Aabb3 aabb = self.get_aabb();
        if (!aabb.collides(box)) return false;
    }

    Vec3 t0 = self.v0 - center;
    Vec3 t1 = self.v1 - center;
    Vec3 t2 = self.v2 - center;

    // 2. Calculate triangle edges
    Vec3 e0 = t1 - t0;
    Vec3 e1 = t2 - t1;
    Vec3 e2 = t0 - t2;

    Vec3 normal = e0.cross(e1);

    if (!are_separated(normal, t0, t1, t2, extent)) return false;
 
    // Edge e0
    if (!are_separated({0, -e0.z, e0.y}, t0, t1, t2, extent)) return false;
    if (!are_separated({e0.z, 0, -e0.x}, t0, t1, t2, extent)) return false;
    if (!are_separated({-e0.y, e0.x, 0}, t0, t1, t2, extent)) return false;

    // Edge e1
    if (!are_separated({0, -e1.z, e1.y}, t0, t1, t2, extent)) return false;
    if (!are_separated({e1.z, 0, -e1.x}, t0, t1, t2, extent)) return false;
    if (!are_separated({-e1.y, e1.x, 0}, t0, t1, t2, extent)) return false;

    // Edge e2
    if (!are_separated({0, -e2.z, e2.y}, t0, t1, t2, extent)) return false;
    if (!are_separated({e2.z, 0, -e2.x}, t0, t1, t2, extent)) return false;
    if (!are_separated({-e2.y, e2.x, 0}, t0, t1, t2, extent)) return false;

    return true;
}

fn bool are_separated(Vec3 axis, Vec3 t0, Vec3 t1, Vec3 t2, Vec3 h) @local {
    Number p0 = t0.dot(axis);
    Number p1 = t1.dot(axis);
    Number p2 = t2.dot(axis);
    Number r = h.x * math::abs(axis.x) + h.y * math::abs(axis.y) + h.z * math::abs(axis.z);

    MinMax box;

    box.expand_by_array({p0, p1, p2});

    if (box.max < -r || box.min > r) {
        return false; // Separated
    }

    return true;
}

fn HitInfo Triangle.intersect_ray(&self, Ray ray) {
    HitInfo hit_info = {.collides = false};

    const Number EPSILON = 1e-6f;
    
    Vec3 edge1 = self.v1 - self.v0;
    Vec3 edge2 = self.v2 - self.v0;
    Vec3 h = ray.direction.cross(edge2);
    Number a = edge1.dot(h);

    if (math::abs(a) < EPSILON) return hit_info; // Ray is parallel to triangle

    Number f = 1.0f / a;
    Vec3 s = ray.origin - self.v0;
    Number u = f * s.dot(h);


    if (u < 0.0f || u > 1.0f) return hit_info;

    Vec3 q = s.cross(edge1);
    Number v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 1.0f) return hit_info;

    // Compute t to find the intersection point on the ray
    Number t = f * edge2.dot(q);

    if (t < EPSILON) return hit_info;
    
    return {.collides = true, .depth = t, .normal = edge1.cross(edge2).normalize()};
}

fn Vec3 Triangle.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 furthest_point = self.v0;
    Number max_distance = -Number.max;

    Number distance = self.v0.dot(direction);
    Number distance2 = self.v1.dot(direction);
    Number distance3 = self.v2.dot(direction);

    if (distance < distance2) {
        distance = distance2;
        furthest_point = self.v1;
    }

    if (distance < distance3) {
        distance = distance3;
        furthest_point = self.v2;
    }

    return furthest_point;
}

fn Matrix3f Triangle.inertia_tensor(&self, Number mass) @dynamic {
    Matrix3f inertia = MATRIX3F_IDENTITY;
    return inertia;
}

fn ShapeType Triangle.get_type(&self) @dynamic => ShapeType.TRIANGLE;

fn Aabb3 Triangle.get_aabb(&self) @dynamic {
    Aabb3 box = {
        .min = {math::min(self.v0.x, self.v1.x, self.v2.x), math::min(self.v0.y, self.v1.y, self.v2.y), math::min(self.v0.z, self.v1.z, self.v2.z)},
        .max = {math::max(self.v0.x, self.v1.x, self.v2.x), math::max(self.v0.y, self.v1.y, self.v2.y), math::max(self.v0.z, self.v1.z, self.v2.z)}
    };  
    return box;
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.v0, self.v1, self.v2)!;
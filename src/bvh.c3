module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

const int MAX_TRIS_PER_LEAF = 8;

struct Triangle {
    Vec3 v0, v1, v2;
}

// Represents a node in the BVH tree
struct BVHNode {
    Aabb3 box;
    BVHNode* left;
    BVHNode* right;
    ElasticArray{Triangle, MAX_TRIS_PER_LEAF} triangles;
}

fn bool BVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void BVHNode.free(&self) {
    mem::free(self.right);
    mem::free(self.left);
}

<*
 @require triangles.len != 0 : `Triangle list is empty`
*>
fn void BVHNode.build_recursive(&self, Triangle[] triangles) {

    foreach (triangle: triangles) {
        self.box.expand_by_point(triangle.v0);
        self.box.expand_by_point(triangle.v1);
        self.box.expand_by_point(triangle.v2);
    }

    if (triangles.len <= MAX_TRIS_PER_LEAF) {
        self.triangles.push_all(triangles);
        return;
    }
    
    Vec3 extent = self.box.max - self.box.min;
    uint axis = 0;

    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;

    Number split_pos = self.box.min[axis] + extent[axis] * 0.5;

    List{Triangle} left;
    List{Triangle} right;

    defer {
        left.free();
        right.free();
    }

    foreach (triangle: triangles) {
        Vec3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0;
        if (centroid[axis] < split_pos) {
            left.push(triangle);
        } else {
            right.push(triangle);
        }
    }

    // Handle cases where the split results in all triangles on one side.
    if (left.is_empty() || right.is_empty()) {
        uint half = triangles.len / 2;
        left.clear();
        right.clear();
        left.push_all(triangles[0..half]);
        right.push_all(triangles[half..triangles.len - 1]);
    }

    self.left = mem::new(BVHNode, (BVHNode){});
    self.right = mem::new(BVHNode, (BVHNode){});

    self.left.build_recursive(left.array_view());
    self.right.build_recursive(right.array_view());
}

fn bool BVHNode.ray_intersects(&self, Ray ray, HitInfo* nearest_info) {
    if (!self.box.intersect_ray(ray)) {
        return false;
    }
    
    if (self.is_leaf()) {
        foreach (triangle: self.triangles) {
            if (triangle.intersect_ray(ray, nearest_info)) {
                return true;
            }
        }
        
        return false;
    }
    
    bool hit_left = self.left.ray_intersects(ray, nearest_info);
    bool hit_right = self.right.ray_intersects(ray, nearest_info);

    return hit_left || hit_right;
}

fn bool BVHNode.box_intersects(&self, Aabb3 box, HitInfo* nearest_info) {
    if (!self.box.collides(box)) {
        return false;
    }

    if (self.is_leaf()) {
        bool hit = false;
        foreach (triangle: self.triangles) {
            if (triangle.collides_aabb(box)) {
                nearest_info.normal = triangle.v0.cross(triangle.v1);
                hit = true;
            }
        }
        return hit;
    }

    bool hit_left = self.left.box_intersects(box, nearest_info);
    bool hit_right = self.right.box_intersects(box, nearest_info);

    return hit_left || hit_right;
}


fn Aabb3 Triangle.get_aabb(&self) {
    Aabb3 box = {
        .min = {math::min(self.v0.x, self.v1.x, self.v2.x), math::min(self.v0.y, self.v1.y, self.v2.y), math::min(self.v0.z, self.v1.z, self.v2.z)},
        .max = {math::max(self.v0.x, self.v1.x, self.v2.x), math::max(self.v0.y, self.v1.y, self.v2.y), math::max(self.v0.z, self.v1.z, self.v2.z)}
    };  
    return box;
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("v1 %s, v2 %s v3 %s", self.v0, self.v1, self.v2)!;

fn bool Triangle.collides_aabb(&self, Aabb3 box) {
    Vec3 box_center = (box.min + box.max) * 0.5;
    Vec3 box_extents = box.max - box_center;
    Vec3[3] tri_verts = {
        self.v0 - box_center,
        self.v1 - box_center,
        self.v2 - box_center
    };

    for (int i = 0; i < 3; ++i) {
        if (math::max(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) < -box_extents[i] || math::min(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) > box_extents[i]) {
            return false; // Found a separating axis
        }
    }

    Vec3[3] tri_edges = {
        tri_verts[1] - tri_verts[0],
        tri_verts[2] - tri_verts[1],
        tri_verts[0] - tri_verts[2]
    };

    Vec3 tri_normal = tri_edges[0].cross(tri_edges[1]);
    Number tri_radius = math::abs(tri_verts[0].dot(tri_normal));
    Number box_radius = math::abs(box_extents.x * tri_normal.x) +
                       math::abs(box_extents.y * tri_normal.y) +
                       math::abs(box_extents.z * tri_normal.z);

    if (tri_radius > box_radius) {
        return false;
    }

    Vec3[3] axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    for (uint i = 0; i < 3; ++i) {
        for (uint j = 0; j < 3; ++j) {
            Vec3 axis = tri_edges[i].cross(axes[j]);
            Vec2 tri_interval = tri_verts[0..].get_interval(3, axis);

            Number box_radius_axis = box_extents.x * math::abs(axes[0].dot(axis)) +
                                    box_extents.y * math::abs(axes[1].dot(axis)) +
                                    box_extents.z * math::abs(axes[2].dot(axis));

            if (tri_interval.x > box_radius_axis || tri_interval.y < -box_radius_axis) {
                return false;
            }                         
        }
    } 

    return true;
}


fn bool Triangle.intersect_ray(&self, Ray ray, HitInfo* info) {
    const Number EPSILON = 1e-6f;
    
    Vec3 edge1 = self.v1 - self.v0;
    Vec3 edge2 = self.v2 - self.v0;
    Vec3 h = ray.direction.cross(edge2);
    Number a = edge1.dot(h);

    if (math::abs(a) < EPSILON) return false; // Ray is parallel to triangle

    Number f = 1.0f / a;
    Vec3 s = ray.origin - self.v0;
    Number u = f * s.dot(h);


    if (u < 0.0f || u > 1.0f) return false;

    Vec3 q = s.cross(edge1);
    Number v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 2) return false;

    // Compute t to find the intersection point on the ray
    Number t = f * edge2.dot(q);

    if (t > EPSILON) {
        info.depth = t;
        info.normal = edge1.cross(edge2).normalize();
        return true;
    } // ray intersection

    return false;
}
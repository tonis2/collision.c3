module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

const int MAX_TRIS_PER_LEAF = 8;

alias TriangleList = ElasticArray{Triangle, MAX_TRIS_PER_LEAF};

struct HitInfo {
    float t;
    Vec3f point;
}

// Represents a node in the BVH tree
struct BVHNode {
    Aabb3 box;
    BVHNode* left;
    BVHNode* right;
    TriangleList triangles;
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
    if (triangles.len <= MAX_TRIS_PER_LEAF) {
        Aabb3 box = triangles.calculate_aabb();
        self.box = box;
        self.triangles.add_array(triangles);
        return;
    }

    Aabb3 box = triangles.calculate_aabb();
    Vec3f extent = box.max - box.min;
    uint axis = 0;

    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    

    float split_pos = self.box.min[axis] + extent[axis] * 0.5;

    TriangleList left;
    TriangleList right;

    foreach (triangle: triangles) {
        Vec3f centroid = (triangle.v0 + triangle.v1 + triangle.v2) / 3.0;

        if (centroid[axis] < split_pos) {
            left.push(triangle);
        } else {
            right.push(triangle);
        }
    }

    // Handle cases where the split results in all triangles on one side.
    if (left.is_empty() || right.is_empty()) {
        uint half = triangles.len / 2;
        left.add_array(triangles[0..half]);
        right.add_array(triangles[half..triangles.len]);
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
        bool hit = false;
        foreach (triangle: self.triangles) {
            if (triangle.intersect_ray(ray)) {
                hit = true;
            }
        }
        return hit;
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
            if (box.collides(triangle.get_aabb())) {
                hit = true;
            }
        }
        return hit;
    }

    bool hit_left = self.left.box_intersects(box, nearest_info);
    bool hit_right = self.right.box_intersects(box, nearest_info);

    return hit_left || hit_right;
}



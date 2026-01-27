module collision;

import std::math;
import std::io;
import std::collections::list;

const int MAX_PARTS_PER_LEAF = 4;

// Info about a convex part for BVH building
struct PartInfo @local {
    usz index;      // Index into convex_parts list
    Vec3 centroid;
    Aabb3 aabb;
}

// BVH Node for convex parts collision
struct BVHNode {
    Aabb3 aabb;
    BVHNode* left;
    BVHNode* right;
    usz start;      // Start index in sorted parts array
    usz count;      // Number of parts in this node (leaf only)
}

fn bool BVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void BVHNode.free(&self) {
    if (self.left != null) {
        self.left.free();
        mem::free(self.left);
    }

    if (self.right != null) {
        self.right.free();
        mem::free(self.right);
    }
}

fn int compare_parts(PartInfo a, PartInfo b, usz* axis) @local {
    float val_a, val_b;

    if (*axis == 0) {
        val_a = a.centroid.x;
        val_b = b.centroid.x;
    } else if (*axis == 1) {
        val_a = a.centroid.y;
        val_b = b.centroid.y;
    } else {
        val_a = a.centroid.z;
        val_b = b.centroid.z;
    }

    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

fn BVHNode* build_bvh_recursive(PartInfo* parts, usz[] indices, usz start, usz count) @local {
    BVHNode* node = mem::new(BVHNode, { .start = start, .count = count, .aabb = NEW_AABB3 });

    // Compute AABB for all parts in this node
    for (usz i = start; i < start + count; i++) {
        node.aabb.expand_by_aabb(parts[indices[i]].aabb);
    }

    // If few enough parts, make a leaf
    if (count <= MAX_PARTS_PER_LEAF) {
        return node;
    }

    // Find split axis using centroid bounds
    Aabb3 centroid_aabb = NEW_AABB3;
    for (usz i = start; i < start + count; i++) {
        centroid_aabb.expand_by_point(parts[indices[i]].centroid);
    }

    usz split_axis = centroid_aabb.longest_axis();

    // Sort parts along split axis using simple bubble sort (parts count is usually small)
    for (usz i = start; i < start + count - 1; i++) {
        for (usz j = start; j < start + count - 1 - (i - start); j++) {
            if (compare_parts(parts[indices[j]], parts[indices[j + 1]], &split_axis) > 0) {
                usz tmp = indices[j];
                indices[j] = indices[j + 1];
                indices[j + 1] = tmp;
            }
        }
    }

    // Split at midpoint
    usz mid = start + count / 2;

    node.left = build_bvh_recursive(parts, indices, start, mid - start);
    node.right = build_bvh_recursive(parts, indices, mid, start + count - mid);
    node.count = 0;  // Internal nodes have count = 0

    return node;
}

// Query BVH for parts that potentially collide with given AABB
// Returns indices of potentially colliding parts
fn List{usz} BVHNode.query_aabb(&self, Aabb3 query_aabb, usz[] sorted_indices) {
    List{usz} result;
    self.query_aabb_recursive(query_aabb, sorted_indices, &result);
    return result;
}

fn void BVHNode.query_aabb_recursive(&self, Aabb3 query_aabb, usz[] sorted_indices, List{usz}* result) @local {
    // Early out if AABBs don't overlap
    if (!self.aabb.collides(query_aabb)) {
        return;
    }

    // If leaf, add all parts in this node
    if (self.is_leaf()) {
        for (usz i = self.start; i < self.start + self.count; i++) {
            result.push(sorted_indices[i]);
        }
        return;
    }

    // Recurse into children
    if (self.left != null) {
        self.left.query_aabb_recursive(query_aabb, sorted_indices, result);
    }
    if (self.right != null) {
        self.right.query_aabb_recursive(query_aabb, sorted_indices, result);
    }
}

// ============================================================================
// Triangle BVH for concave mesh collision (original implementation, restored)
// ============================================================================

const int MAX_TRIS_PER_LEAF = 8;
const float BVH_EPSILON @local = 0.00001;

struct TriangleInfo @local {
    usz v0, v1, v2;
    Vec3 centroid;
    Aabb3 aabb;
}

// BVH Node for triangle mesh collision
struct TriBVHNode {
    Aabb3 aabb;
    TriBVHNode* left;
    TriBVHNode* right;
    Vec3* positions;
    usz start;      // Start triangle index
    usz count;      // Number of triangles (leaf only)
}

fn bool TriBVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void TriBVHNode.free(&self) {
    if (self.left != null) {
        self.left.free();
        mem::free(self.left);
    }

    if (self.right != null) {
        self.right.free();
        mem::free(self.right);
    }
}

fn int compare_triangles(TriangleInfo a, TriangleInfo b, usz* split_axis) @local {
    float val_a, val_b;

    if (*split_axis == 0) {
        val_a = a.centroid.x;
        val_b = b.centroid.x;
    } else if (*split_axis == 1) {
        val_a = a.centroid.y;
        val_b = b.centroid.y;
    } else {
        val_a = a.centroid.z;
        val_b = b.centroid.z;
    }

    if (val_a < val_b) return -1;
    if (val_a > val_b) return 1;
    return 0;
}

// Create BVH tree from triangle mesh vertices
// vertices: array of Vec3 positions (every 3 = one triangle)
// count: total number of vertices (must be divisible by 3)
fn TriBVHNode* create_tri_bvh(Vec3* vertices, usz count) {
    if (vertices == null || count < 3) return null;

    usz triangle_count = count / 3;
    TriangleInfo* triangles = mem::new_array(TriangleInfo, triangle_count);

    for (usz i = 0; i < triangle_count; i++) {
        usz v_idx = i * 3;

        Vec3 v0 = vertices[v_idx];
        Vec3 v1 = vertices[v_idx + 1];
        Vec3 v2 = vertices[v_idx + 2];

        TriangleInfo triangle = {
            .v0 = v_idx,
            .v1 = v_idx + 1,
            .v2 = v_idx + 2,
            .centroid = (v0 + v1 + v2) / 3.0f,
            .aabb = NEW_AABB3
        };

        triangle.aabb.expand_by_point(v0);
        triangle.aabb.expand_by_point(v1);
        triangle.aabb.expand_by_point(v2);

        triangles[i] = triangle;
    }

    TriBVHNode* tree = build_tri_bvh_recursive(triangles, vertices, 0, triangle_count);

    // Reorder vertex buffer for cache locality
    Vec3* new_positions = mem::new_array(Vec3, count);

    for (usz i = 0; i < triangle_count; i++) {
        usz old_v0 = triangles[i].v0;
        usz old_v1 = triangles[i].v1;
        usz old_v2 = triangles[i].v2;

        usz index = i * 3;
        new_positions[index] = vertices[old_v0];
        new_positions[index + 1] = vertices[old_v1];
        new_positions[index + 2] = vertices[old_v2];
    }

    mem::copy(vertices, new_positions, Vec3.sizeof * count);
    mem::free(new_positions);
    mem::free(triangles);

    remap_tri_bvh(tree, vertices);

    return tree;
}

fn void remap_tri_bvh(TriBVHNode* node, Vec3* positions) @local {
    node.positions = positions;
    if (node.left != null) remap_tri_bvh(node.left, positions);
    if (node.right != null) remap_tri_bvh(node.right, positions);
}

fn TriBVHNode* build_tri_bvh_recursive(TriangleInfo* triangles, Vec3* vertices, usz start, usz count) @local {
    TriBVHNode* node = mem::new(TriBVHNode, { .start = start, .count = count, .aabb = NEW_AABB3 });

    for (usz i = start; i < start + count; i++) {
        node.aabb.expand_by_aabb(triangles[i].aabb);
    }

    if (count <= MAX_TRIS_PER_LEAF) {
        return node;
    }

    Aabb3 centroid_aabb = NEW_AABB3;
    for (usz i = start; i < start + count; i++) {
        centroid_aabb.expand_by_point(triangles[i].centroid);
    }

    usz split_axis = centroid_aabb.longest_axis();

    // Simple bubble sort for triangles
    for (usz i = start; i < start + count - 1; i++) {
        for (usz j = start; j < start + count - 1 - (i - start); j++) {
            if (compare_triangles(triangles[j], triangles[j + 1], &split_axis) > 0) {
                TriangleInfo tmp = triangles[j];
                triangles[j] = triangles[j + 1];
                triangles[j + 1] = tmp;
            }
        }
    }

    usz mid = start + count / 2;

    node.left = build_tri_bvh_recursive(triangles, vertices, start, mid - start);
    node.right = build_tri_bvh_recursive(triangles, vertices, mid, start + count - mid);
    node.count = 0;

    return node;
}

// Query triangle BVH with AABB, returns hit info with first colliding triangle
fn HitInfo TriBVHNode.box_intersects(&self, Aabb3 aabb) {
    HitInfo hit;

    if (!self.aabb.collides(aabb)) {
        return hit;
    }

    if (self.is_leaf()) {
        for (usz i = 0; i < self.count; i++) {
            usz offset = (self.start + i) * 3;

            Vec3 v0 = self.positions[offset];
            Vec3 v1 = self.positions[offset + 1];
            Vec3 v2 = self.positions[offset + 2];

            TriangleVerts triangle = { v0, v1, v2 };

            float overlap = triangle.collides_aabb(aabb);

            if (overlap > 0) {
                return {
                    .triangle = triangle,
                    .depth = overlap,
                    .collided = true,
                };
            }
        }
        return hit;
    }

    if (self.left != null) {
        HitInfo left_hit = self.left.box_intersects(aabb);
        if (left_hit.collided) return left_hit;
    }

    if (self.right != null) {
        HitInfo right_hit = self.right.box_intersects(aabb);
        if (right_hit.collided) return right_hit;
    }

    return hit;
}

// Query all overlapping triangles (for multiple contact points)
fn void TriBVHNode.query_all_triangles(&self, Aabb3 aabb, List{TriangleVerts}* results) {
    if (!self.aabb.collides(aabb)) {
        return;
    }

    if (self.is_leaf()) {
        for (usz i = 0; i < self.count; i++) {
            usz offset = (self.start + i) * 3;

            Vec3 v0 = self.positions[offset];
            Vec3 v1 = self.positions[offset + 1];
            Vec3 v2 = self.positions[offset + 2];

            TriangleVerts triangle = { v0, v1, v2 };

            if (triangle.collides_aabb(aabb) > 0) {
                results.push(triangle);
            }
        }
        return;
    }

    if (self.left != null) {
        self.left.query_all_triangles(aabb, results);
    }
    if (self.right != null) {
        self.right.query_all_triangles(aabb, results);
    }
}

// Ray intersection with triangle BVH
fn HitInfo TriBVHNode.ray_intersects(&self, Ray ray) {
    HitInfo hit_info = { .collided = false };

    if (self.aabb.intersect_ray(ray).collided == false) {
        return hit_info;
    }

    if (self.is_leaf()) {
        float closest_t = float.max;

        for (usz i = 0; i < self.count; i++) {
            usz offset = (self.start + i) * 3;

            Vec3 v0 = self.positions[offset];
            Vec3 v1 = self.positions[offset + 1];
            Vec3 v2 = self.positions[offset + 2];

            TriangleVerts triangle = { v0, v1, v2 };
            HitInfo hit = triangle.intersect_ray(ray);

            if (hit.collided && hit.depth < closest_t) {
                closest_t = hit.depth;
                hit_info = hit;
                hit_info.triangle = triangle;
            }
        }

        return hit_info;
    }

    HitInfo left_hit = { .collided = false };
    HitInfo right_hit = { .collided = false };

    if (self.left != null) {
        left_hit = self.left.ray_intersects(ray);
    }
    if (self.right != null) {
        right_hit = self.right.ray_intersects(ray);
    }

    if (left_hit.collided && right_hit.collided) {
        return left_hit.depth < right_hit.depth ? left_hit : right_hit;
    }
    if (left_hit.collided) return left_hit;
    if (right_hit.collided) return right_hit;

    return hit_info;
}

// ============================================================================
// Triangle collision utilities
// ============================================================================

alias TriangleVerts = Vec3[3];

fn HitInfo TriangleVerts.intersect_ray(&self, Ray ray) {
    HitInfo hit_info = { .collided = false };

    Vec3 edge1 = (*self)[1] - (*self)[0];
    Vec3 edge2 = (*self)[2] - (*self)[0];

    Vec3 h = ray.direction.cross(edge2);
    float a = edge1.dot(h);

    if (a > -BVH_EPSILON && a < BVH_EPSILON) return hit_info;

    float f = 1.0f / a;
    Vec3 s = ray.origin - (*self)[0];
    float u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f) return hit_info;

    Vec3 q = s.cross(edge1);
    float v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 1.0f) return hit_info;

    float t = f * edge2.dot(q);

    if (t < BVH_EPSILON) return hit_info;

    return { .collided = true, .depth = t, .normal = edge1.cross(edge2).normalize() };
}

fn float TriangleVerts.collides_aabb(&self, Aabb3 box) {
    Vec3 center = box.center();
    Vec3 extent = box.extent();

    Vec3 v0 = (*self)[0] - center;
    Vec3 v1 = (*self)[1] - center;
    Vec3 v2 = (*self)[2] - center;

    Vec3 f0 = v1 - v0;
    Vec3 f1 = v2 - v1;
    Vec3 f2 = v0 - v2;

    Vec3[] t = { v0, v1, v2 };

    Vec3[] axes = {
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    float result_depth = float.max;

    // Test box axes
    foreach (axis : axes) {
        MinMax triangle_mm = min_max_from_axis(axis, t);
        MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, axis);
        float depth = triangle_mm.overlap(box_mm);
        if (depth == -1) return -1;
        if (depth != 0) result_depth = math::min(depth, result_depth);
    }

    // Test triangle normal
    Vec3 normal = f0.cross(f1);
    {
        MinMax triangle_mm = min_max_from_axis(normal, t);
        MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, normal);
        float depth = triangle_mm.overlap(box_mm);
        if (depth == -1) return -1;
        if (depth != 0) result_depth = math::min(depth, result_depth);
    }

    // Test cross products of edges
    Vec3[] edges = { f0, f1, f2 };

    foreach (axis : axes) {
        foreach (edge : edges) {
            Vec3 new_axis = axis.cross(edge);

            if (new_axis == (Vec3){ 0, 0, 0 }) continue;

            MinMax triangle_mm = min_max_from_axis(new_axis, t);
            MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, new_axis);
            float depth = triangle_mm.overlap(box_mm);
            if (depth == -1) return -1;
            if (depth != 0) result_depth = math::min(depth, result_depth);
        }
    }

    return result_depth;
}

fn Vec3 TriangleVerts.get_normal(&self) {
    Vec3 edge1 = (*self)[1] - (*self)[0];
    Vec3 edge2 = (*self)[2] - (*self)[0];
    return edge1.cross(edge2).normalize();
}

// ============================================================================
// BVH Hit info structure (separate from helpers.c3 HitInfo)
// ============================================================================
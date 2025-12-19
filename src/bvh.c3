module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::time;
import std::sort;

const int MAX_TRIS_PER_LEAF = 8;
const Number EPSILON @local = 0.00001;

alias Triangle = Vec3[3];

struct TriangleInfo {
    usz v0, v1, v2;
    Vec3 centroid;
    Aabb3 aabb;
}

// Represents a node in the BVH tree
struct BVHNode (CollisionShape) {
    Aabb3 aabb;
    BVHNode* left;
    BVHNode* right;
    Vec3* positions;
    usz index;
}

fn bool BVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void BVHNode.free(&self) {
    if (self.left != null) {
        self.left.free();
        mem::free(self.left);
    };
    
    if (self.right != null) {
        self.right.free();
        mem::free(self.right);
    };

    // if (self.is_leaf()) {
    //     mem::free(self);
    // }
  
}

fn void BVHNode.remap(&self, Vec3* positions) @local {
    if (self.is_leaf()) {
        self.positions = positions;
        self.index *= 3;
        return;
    }

    if (self.left != null) self.left.remap(positions);
    if (self.right != null) self.right.remap(positions);
}

fn int compare(TriangleInfo a, TriangleInfo b, usz* split_index) {
    Number val_a, val_b;
    
    if (*split_index == 0) {
        val_a = a.centroid.x;
        val_b = b.centroid.x;
    } else if (*split_index == 1) {
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

<*
 @require vertices != null : `Has no vertices attached`
*>
fn BVHNode* create_bvh_tree(Vec3* vertices, usz count) {

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
            .centroid = (v0 + v1 + v2) / 3,
            .aabb = EMPTY_AABB3
        };

        triangle.aabb.expand_by_point(v0);
        triangle.aabb.expand_by_point(v1);
        triangle.aabb.expand_by_point(v2);
   
        triangles[i] = triangle;
    }

    BVHNode* tree = build_recursive(triangles, start: 0, count: triangle_count);

    // Reorder position buffer vertices, for fast cache locality
    Vec3* new_positions = mem::new_array(Vec3, count);

    // 2. Copy vertices from the old buffer to the new buffer in the new order
    for (usz i = 0; i < triangle_count; i++) {
        // Find where the vertices were originally
        usz old_v0 = triangles[i].v0;
        usz old_v1 = triangles[i].v1;
        usz old_v2 = triangles[i].v2;

        // Place them into the new buffer at the sequential index (i*3)
        usz index = i * 3;
        new_positions[index] = vertices[old_v0];
        new_positions[index + 1] = vertices[old_v1];
        new_positions[index + 2] = vertices[old_v2];
    }

    mem::copy(vertices, new_positions, Vec3.sizeof * count);

    mem::free(new_positions);
    mem::free(triangles);

    tree.remap(vertices);

    return tree;
}

<*
 @require triangles != null : `Has no triangles attached`
*>
fn BVHNode* build_recursive(TriangleInfo* triangles, usz start, usz count) @local {
    BVHNode* node = mem::new(BVHNode, {.index = 0, .aabb = EMPTY_AABB3 });

    for (usz i = start; i < start + count; i++) {
        node.aabb.expand_by_aabb(triangles[i].aabb);
    }

    if (count == 1) {
        node.index = start;
        return node;
    }

    Aabb3 centroid_aabb = EMPTY_AABB3;
    for (usz i = start; i < start + count; i++) {
        centroid_aabb.expand_by_point(triangles[i].centroid);
    }

    usz split_axis = centroid_aabb.longest_axis();

    sort::quicksort(triangles[start..start + count - 1], &compare, &split_axis);

    // 5. Find the midpoint and recurse
    usz mid = start + count / 2;
    
    node.left = build_recursive(triangles, start, mid - start);
    node.right = build_recursive(triangles, mid, start + count - mid);

    return node;
}

fn HitInfo BVHNode.ray_intersects(&self, Ray ray) {
    HitInfo hit_info = {.collided = false};
 
    if (self.aabb.intersect_ray(ray).collided == false) {
        return hit_info;
    }
    
    if (self.is_leaf()) {
        usz pos_index = self.index;
        Vec3 v0 = self.positions[pos_index].xyz;
        Vec3 v1 = self.positions[pos_index + 1].xyz;
        Vec3 v2 = self.positions[pos_index + 2].xyz;

        Triangle triangle = {v0, v1, v2};
        HitInfo hit = triangle.intersect_ray(ray);

        if (hit.collided) {
            hit.triangle = triangle;
            return hit;
        };
        
        return hit_info;
    }
    
    HitInfo left = self.left.ray_intersects(ray);
    HitInfo right = self.right.ray_intersects(ray);

    if (left.collided) return left;
    if (right.collided) return right;

    return hit_info;
}

fn HitInfo BVHNode.box_intersects(&self, Aabb3 aabb) {
    HitInfo hit_info = {.collided = false};

    if (!self.aabb.collides(aabb)) {
        return hit_info;
    }

    if (self.is_leaf()) {
        usz pos_index = self.index;
        Vec3 v0 = self.positions[pos_index].xyz;
        Vec3 v1 = self.positions[pos_index + 1].xyz;
        Vec3 v2 = self.positions[pos_index + 2].xyz;

        Triangle triangle = {v0, v1, v2};
        
       
        if (collision::check_convex_collision(&aabb, &self.aabb, epa: false).collided) {
            return {.collided = true, .normal = triangle[0].cross(triangle[1]), .triangle = triangle};
        }

        return hit_info;
    }

    HitInfo left = self.left.box_intersects(aabb);
    HitInfo right = self.right.box_intersects(aabb);

    if (left.collided) return left;
    if (right.collided) return right;

    return hit_info;
}

fn HitInfo Triangle.intersect_ray(self, Ray ray) {
    HitInfo hit_info = {.collided = false};

    Vec3 edge1 = self[1] - self[0];
    Vec3 edge2 = self[2] - self[0];

    Vec3 h = ray.direction.cross(edge2);
    Number a = edge1.dot(h);

    if (a > -EPSILON && a < EPSILON) return hit_info; // Ray is parallel to triangle

    Number f = 1.0f / a;
    Vec3 s = ray.origin - self[0];
    Number u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f) return hit_info;

    Vec3 q = s.cross(edge1);
    Number v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 1.0f) return hit_info;

    // Compute t to find the intersection point on the ray
    Number t = f * edge2.dot(q);

    if (t < EPSILON) return hit_info;
    
    return {.collided = true, .depth = t, .normal = edge1.cross(edge2).normalize()};
}

fn Vec3 Triangle.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 furthest_point;
    Number max_distance = -Number.max;

    foreach (point: *self) {
        Number distance = point.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = point;
        } 
    }

    return furthest_point;
}

fn Matrix3f Triangle.inertia_tensor(&self, Number mass) @dynamic {
    Matrix3f inertia = MATRIX3F_IDENTITY;
    return inertia;
}

fn ShapeType Triangle.get_type(&self) @dynamic => ShapeType.TRIANGLE;

fn Aabb3 Triangle.get_aabb(&self) @dynamic {
    Vec3[3] triangle = *self;
    return {
        .min = {math::min(triangle[0].x, triangle[1].x, triangle[2].x), math::min(triangle[0].y, triangle[1].y, triangle[2].y), math::min(triangle[0].z, triangle[1].z, triangle[2].z)},
        .max = {math::max(triangle[0].x, triangle[1].x, triangle[2].x), math::max(triangle[0].y, triangle[1].y, triangle[2].y), math::max(triangle[0].z, triangle[1].z, triangle[2].z)}
    };
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s\n%s", (*self)[0], (*self)[1], (*self)[2])!;


fn Vec3 BVHNode.furthest_point(&self, Vec3 direction) @dynamic => {0,0,0};
fn ShapeType BVHNode.get_type(&self) @dynamic => ShapeType.TRIANGLE;
fn Aabb3 BVHNode.get_aabb(&self) @dynamic => self.aabb;
fn Matrix3f BVHNode.inertia_tensor(&self, Number mass) @dynamic {
    Matrix3f inertia = MATRIX3F_IDENTITY;
    return inertia;
}

fn CollisionInfo check_concave_collision(Rigidbody* shape_1, Rigidbody* shape_2) {
    // Ball
    Rigidbody* convex_shape;

    // Plane
    Rigidbody* concave_shape;

    if (shape_1.collider.get_type() != TRIANGLE && shape_2.collider.get_type() == TRIANGLE) {
        convex_shape = shape_1;
        concave_shape = shape_2;
    } else {
        convex_shape = shape_2;
        concave_shape = shape_1;
    }

    // Shape 2 center of the universe, shape 1 player
    BVHNode* bvh_tree = (BVHNode*)concave_shape.shape;

    // Matrix4f matrix = concave_shape.collider.matrix();

    Vec3 local_pos = convex_shape.collider.translation;

    // io::printfn("%s", local_pos);

    // io::printfn("%s", shape_1.collider.translation);

    // Vec3 local_space = (matrix.inverse()!! * shape_1.collider.translation);
    //Vec3 local_space = shape_1.collider.translation;

    // HitInfo info = bvh_tree.ray_intersects({.origin = convex_shape.collider.translation, .direction = {0, 0, -1}});

    HitInfo info = bvh_tree.box_intersects(convex_shape.transformed_aabb);

    if (info.collided) {
        Triangle triangle = info.triangle;
        
        // io::printfn("%s", triangle);
        // foreach (i, value: triangle) {
        //     triangle[i] = value;
        // }

        TransformedShape shape = {
            ...convex_shape.collider,
            .shape = &triangle,
        };

        CollisionInfo collision = collision::check_convex_collision(&triangle, &convex_shape.collider, epa:true);

        // io::printfn("%s", collision.contact_points);

        return collision;
    }

    return {};
}
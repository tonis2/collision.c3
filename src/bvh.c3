module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::time;
import std::sort;

const int MAX_TRIS_PER_LEAF = 12;
const Number EPSILON @local = 0.00001;

struct MinMax @local {
    Number min;
    Number max;
}

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
    usz count;
    usz index;
}

fn void MinMax.expand(&self, Number value) {
    self.min = math::min(self.min, value);
    self.max = math::max(self.max, value);
}

fn void MinMax.expand_by_array(&self, Number[] values) {
    foreach (value: values) self.expand(value);
}

fn MinMax min_max_from_box(Vec3 center, Vec3 extent, Vec3 axis) {
    Number c = center.dot(axis);
    Number r = math::abs(axis.x) * extent.x + math::abs(axis.y) * extent.y + math::abs(axis.z) * extent.z;

    return {
        .min = c - r,
        .max = c + r
    };
}

fn MinMax min_max_from_axis(Vec3 axis, Vec3[] points) {
    MinMax corners = {
        .min = axis.dot(points[0]),
        .max = axis.dot(points[0])
    };

    foreach (point: points) corners.expand(axis.dot(point));

    return corners;
}

fn Number MinMax.overlap(&a, MinMax b) {
    if (a.max < b.min || b.max < a.min) {
        // Separated 
        return -1;
    }

    Number depth = math::min(a.max, b.max) - math::max(a.min, b.min);
    return depth;
}

fn usz? MinMax.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("min %s max %s",  self.min, self.max)!;
fn bool BVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void BVHNode.free(&self) @dynamic {
    if (self.left != null) {
        self.left.free();
        mem::free(self.left);
    };
    
    if (self.right != null) {
        self.right.free();
        mem::free(self.right);
    };
}

fn void BVHNode.remap(&self, Vec3* positions) @local {
    self.positions = positions;
    // if (self.is_leaf()) {
    //     self.index *= 3;
    //     return;
    // }

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
            .aabb = NEW_AABB3
        };

        triangle.aabb.expand_by_point(v0);
        triangle.aabb.expand_by_point(v1);
        triangle.aabb.expand_by_point(v2);

        assert(triangle.aabb != EMPTY_AABB3, "Triangle aabb, must not be empty");

        triangles[i] = triangle;
    }

    BVHNode* tree = build_recursive(triangles, start: 0, count: triangle_count);

    // Reorder position buffer vertices, for fast cache locality
    Vec3* new_positions = mem::new_array(Vec3, count);

    // // 2. Copy vertices from the old buffer to the new buffer in the new order
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
    BVHNode* node = mem::new(BVHNode, {.index = 0, .count = count, .index = start, .aabb = NEW_AABB3 });

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
	HitInfo hit;

    if (!self.aabb.collides(aabb)) {
        return hit;
    }

    if (self.is_leaf()) {
        for (usz i = 0; i < self.count; i++) {
            usz offset = self.index + (i * 3);
     
            Vec3 v0 = self.positions[offset];
            Vec3 v1 = self.positions[offset + 1];
            Vec3 v2 = self.positions[offset + 2];

            Triangle triangle = {v0, v1, v2};

            Number overlap = triangle.collides_aabb(aabb);

            if (overlap > 0) {
				return {
					.triangle = triangle,
					.depth = overlap,
					.collided = true,
				};
            }
        }
    }

    if (self.left != null) {
		HitInfo left_hit = self.left.box_intersects(aabb);
		if (left_hit.collided) return left_hit;
	};

    if (self.right != null) {
		HitInfo right_hit = self.right.box_intersects(aabb);
		if (right_hit.collided) return right_hit;
	};

	return hit;
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

fn Number Triangle.collides_aabb(&self, Aabb3 box, bool full = true) @local {
    Vec3 center = box.center();
    Vec3 extent = box.extent();

    // if (self.get_aabb().collides(box) == false) return 0;

    Vec3 v0 = (*self)[0] - center;
    Vec3 v1 = (*self)[1] - center;
    Vec3 v2 = (*self)[2] - center;

    // Compute the edge vectors of the triangle  (ABC)
    // That is, get the lines between the points as vectors
    Vec3 f0 = v1 - v0; // B - A
    Vec3 f1 = v2 - v1; // C - B
    Vec3 f2 = v0 - v2; // A - C

    Vec3[] t = {
        v0,
        v1,
        v2
    };

    Vec3[] axes = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    Number result_depth = float.max;

    foreach (axis: axes) {
        MinMax triangle_corners = min_max_from_axis(axis, t);
        MinMax box_corners = min_max_from_box({0,0,0}, extent, axis);
        Number depth = triangle_corners.overlap(box_corners);
        if (depth == -1) return -1;

        if (depth != 0) result_depth = math::min(depth, result_depth);
    }

    Vec3 normal = f0.cross(f1);

    {
        MinMax triangle_corners = min_max_from_axis(normal, t);
        MinMax box_corners = min_max_from_box({0,0,0}, extent, normal);
        Number depth = triangle_corners.overlap(box_corners);
        if (depth == -1) return -1;

        if (depth != 0) result_depth = math::min(depth, result_depth);
    }

    Vec3[] edges = {
        f0,
        f1,
        f2
    };

    foreach (axis: axes) {
        foreach (edge: edges) {
            Vec3 new_axis = axis.cross(edge);

            if (new_axis == {0,0,0}) continue;

            MinMax triangle_corners = min_max_from_axis(new_axis, edges);
            MinMax box_corners = min_max_from_box({0,0,0}, extent, new_axis);
            Number depth = triangle_corners.overlap(box_corners);
            if (depth == -1) return -1;
            if (depth != 0) result_depth = math::min(depth, result_depth);
        }
    }

    return result_depth;
}

fn Vec3 BVHNode.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 furthest_point;
    Number max_distance = -Number.max;

    for (usz i = 0; i < self.count; i++) {
        Vec3 point = self.positions[i];

        Number distance = point.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = point;
        }
    }

    return furthest_point;
}

fn ShapeType BVHNode.get_type(&self) @dynamic => ShapeType.TRIANGLE;
fn Aabb3 BVHNode.get_aabb(&self) @dynamic => self.aabb;
fn Matrix3f BVHNode.inertia_tensor(&self, Number mass) @dynamic => MATRIX3F_IDENTITY;

fn CollisionInfo check_concave_collision(Rigidbody* body_a, Rigidbody* body_b) {
    // Ball
    Rigidbody* convex_shape;

    // Plane
    Rigidbody* concave_shape;

    if (body_a.get_type() != TRIANGLE && body_b.get_type() == TRIANGLE) {
        convex_shape = body_a;
        concave_shape = body_b;
    } else {
        convex_shape = body_b;
        concave_shape = body_a;
    }

    // Shape 2 center of the universe, shape 1 player
    BVHNode* bvh_tree = (BVHNode*)concave_shape.shape;

    HitInfo triangle_hit = bvh_tree.box_intersects(convex_shape.transformed_aabb);

    if (triangle_hit.collided) {
        return check_convex_collision(convex_shape, &triangle_hit.triangle, epa:true);
    }

    return {.collided = false};
}
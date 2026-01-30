module collision;

import std::math;
import std::io;
import std::collections::list;

const int MAX_PARTS_PER_LEAF = 4;


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

    // For indexed mode (memory efficient)
    Vec3* vertices;
    Triangle* triangles;
    usz* sorted_indices;  // Sorted triangle indices for this subtree

    usz start;      // Start index in sorted_indices
    usz count;      // Number of triangles (leaf only)
    bool indexed;   // True if using indexed mode
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

// Free indexed BVH including the sorted_indices array (called on root only)
fn void TriBVHNode.free_indexed(&self) {
    if (self.indexed && self.sorted_indices != null) {
        mem::free(self.sorted_indices);
    }
    self.free();
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


// Query triangle BVH with AABB, returns hit info with first colliding triangle
fn HitInfo TriBVHNode.box_intersects(&self, Aabb3 aabb) {
    HitInfo hit;

    if (!self.aabb.collides(aabb)) {
        return hit;
    }

    if (self.is_leaf()) {
        for (usz i = 0; i < self.count; i++) {
            Vec3 v0, v1, v2;

            if (self.indexed) {
                // Indexed mode: look up triangle indices, then vertex positions
                usz tri_idx = self.sorted_indices[self.start + i];
                Triangle tri = self.triangles[tri_idx];
                v0 = self.vertices[tri.x];
                v1 = self.vertices[tri.y];
                v2 = self.vertices[tri.z];
            }

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
            Vec3 v0, v1, v2;

            if (self.indexed) {
                // Indexed mode: look up triangle indices, then vertex positions
                usz tri_idx = self.sorted_indices[self.start + i];
                Triangle tri = self.triangles[tri_idx];
                v0 = self.vertices[tri.x];
                v1 = self.vertices[tri.y];
                v2 = self.vertices[tri.z];
            }

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

// ============================================================================
// Indexed Triangle BVH (memory efficient - uses mesh vertices directly)
// ============================================================================

// Create indexed BVH tree from mesh vertices and triangles
// Does NOT copy vertex data - uses pointers to existing mesh data

<*
 @require vertices != null : `Vertices must have values`
 @require triangles != null : `Triangles must have values`
 @require tri_count > 0 : `Triangles must have values`
*>
fn TriBVHNode* create_indexed_tri_bvh(Vec3* vertices, Triangle* triangles, usz tri_count) {
    // Build triangle info for sorting
    TriangleInfo* tri_info = mem::new_array(TriangleInfo, tri_count);
    usz* sorted_indices = mem::new_array(usz, tri_count);

    for (usz i = 0; i < tri_count; i++) {
        Triangle tri = triangles[i];
        Vec3 v0 = vertices[tri.x];
        Vec3 v1 = vertices[tri.y];
        Vec3 v2 = vertices[tri.z];

        tri_info[i] = {
            .v0 = i,  // Store triangle index, not vertex index
            .v1 = 0,
            .v2 = 0,
            .centroid = (v0 + v1 + v2) / 3.0f,
            .aabb = NEW_AABB3
        };

        tri_info[i].aabb.expand_by_point(v0);
        tri_info[i].aabb.expand_by_point(v1);
        tri_info[i].aabb.expand_by_point(v2);

        sorted_indices[i] = i;
    }

    TriBVHNode* tree = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, 0, tri_count);

    mem::free(tri_info);

    // Propagate shared data to all nodes
    set_indexed_bvh_data(tree, vertices, triangles, sorted_indices);

    return tree;
}

fn void set_indexed_bvh_data(TriBVHNode* node, Vec3* vertices, Triangle* triangles, usz* sorted_indices) @local {
    node.vertices = vertices;
    node.triangles = triangles;
    node.sorted_indices = sorted_indices;
    node.indexed = true;
    if (node.left != null) set_indexed_bvh_data(node.left, vertices, triangles, sorted_indices);
    if (node.right != null) set_indexed_bvh_data(node.right, vertices, triangles, sorted_indices);
}

fn TriBVHNode* build_indexed_bvh_recursive(TriangleInfo* tri_info, usz* sorted_indices, Vec3* vertices, Triangle* triangles, usz start, usz count) @local {
    TriBVHNode* node = mem::new(TriBVHNode, { .start = start, .count = count, .aabb = NEW_AABB3, .indexed = true });

    for (usz i = start; i < start + count; i++) {
        node.aabb.expand_by_aabb(tri_info[sorted_indices[i]].aabb);
    }

    if (count <= MAX_TRIS_PER_LEAF) {
        return node;
    }

    Aabb3 centroid_aabb = NEW_AABB3;
    for (usz i = start; i < start + count; i++) {
        centroid_aabb.expand_by_point(tri_info[sorted_indices[i]].centroid);
    }

    usz split_axis = centroid_aabb.longest_axis();

    // Sort indices by centroid along split axis
    for (usz i = start; i < start + count - 1; i++) {
        for (usz j = start; j < start + count - 1 - (i - start); j++) {
            if (compare_triangles(tri_info[sorted_indices[j]], tri_info[sorted_indices[j + 1]], &split_axis) > 0) {
                usz tmp = sorted_indices[j];
                sorted_indices[j] = sorted_indices[j + 1];
                sorted_indices[j + 1] = tmp;
            }
        }
    }

    usz mid = start + count / 2;

    node.left = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, start, mid - start);
    node.right = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, mid, start + count - mid);
    node.count = 0;

    return node;
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
            Vec3 v0, v1, v2;

            if (self.indexed) {
                // Indexed mode: look up triangle indices, then vertex positions
                usz tri_idx = self.sorted_indices[self.start + i];
                Triangle tri = self.triangles[tri_idx];
                v0 = self.vertices[tri.x];
                v1 = self.vertices[tri.y];
                v2 = self.vertices[tri.z];
            }

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

const MAX_BVH_CONTACTS @local = 8;  // Max contacts per mesh pair

// Check collisions using triangle BVH from Mesh
// Queries BVH to find potentially colliding triangles, then runs GJK/EPA
fn void PhysicsWorld.check_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) {
    bool a_has_bvh = body_a.has_mesh_bvh();

    // Determine which body has the mesh BVH to query
    Rigidbody* mesh_body;
    Rigidbody* other_body;
    bool mesh_is_a;

    if (a_has_bvh) {
        mesh_body = body_a;
        other_body = body_b;
        mesh_is_a = true;
    } else {
        mesh_body = body_b;
        other_body = body_a;
        mesh_is_a = false;
    }

    Mesh* mesh = (Mesh*)mesh_body.collider.shape;

    // Transform other body's AABB to mesh body's local space for BVH query
    Aabb3 local_aabb = other_body.transformed_aabb.inverse_transform(
        mesh_body.collider.translation,
        mesh_body.collider.scale,
        mesh_body.collider.rotation
    );

    // Lazy init BVH cache (single triangle mesh reused for all checks)
    if (!self.bvh_cache_initialized) {
        self.tri_mesh_cache.vertices.push({0,0,0});
        self.tri_mesh_cache.vertices.push({0,0,0});
        self.tri_mesh_cache.vertices.push({0,0,0});
        self.tri_mesh_cache.triangles.push({0, 1, 2});
        self.bvh_cache_initialized = true;
    }

    // Query BVH for overlapping triangles (reuse cache list)
    self.bvh_query_cache.clear();
    mesh.bvh.query_all_triangles(local_aabb, &self.bvh_query_cache);

    // Reusable shape pointing to cached mesh
    TransformedShape tri_shape = {
        .translation = mesh_body.collider.translation,
        .scale = mesh_body.collider.scale,
        .rotation = mesh_body.collider.rotation,
        .shape = &self.tri_mesh_cache,
    };

    // Reset contact cache for this mesh pair
    self.bvh_contact_count = 0;

    // Pre-compute transform values for AABB calculation
    Vec3 mesh_translation = mesh_body.collider.translation;
    Vec3 mesh_scale = mesh_body.collider.scale;
    QuaternionValue mesh_rotation = mesh_body.collider.rotation;

    // Test each triangle with early AABB rejection
    foreach (tri : self.bvh_query_cache) {
        // Early AABB rejection: compute triangle world AABB
        Vec3 v0 = mesh_rotation * (tri[0] * mesh_scale) + mesh_translation;
        Vec3 v1 = mesh_rotation * (tri[1] * mesh_scale) + mesh_translation;
        Vec3 v2 = mesh_rotation * (tri[2] * mesh_scale) + mesh_translation;

        Aabb3 tri_aabb = {
            .min = {
                math::min(v0.x, math::min(v1.x, v2.x)),
                math::min(v0.y, math::min(v1.y, v2.y)),
                math::min(v0.z, math::min(v1.z, v2.z))
            },
            .max = {
                math::max(v0.x, math::max(v1.x, v2.x)),
                math::max(v0.y, math::max(v1.y, v2.y)),
                math::max(v0.z, math::max(v1.z, v2.z))
            }
        };

        // Skip GJK if triangle AABB doesn't overlap other body's AABB
        if (!tri_aabb.collides(other_body.transformed_aabb)) continue;

        // Update cached mesh vertices in-place (no allocation)
        self.tri_mesh_cache.vertices[0] = tri[0];
        self.tri_mesh_cache.vertices[1] = tri[1];
        self.tri_mesh_cache.vertices[2] = tri[2];

        // Check collision with correct argument order
        CollisionInfo info;
        if (mesh_is_a) {
            info = collision::check_convex_collision(&tri_shape, &other_body.collider, epa:true);
        } else {
            info = collision::check_convex_collision(&other_body.collider, &tri_shape, epa:true);
        }

        // Collect contacts, keeping deepest N (insertion sort by depth)
        if (info.collided && info.depth > 0) {
            Collision new_contact = {
                .a = pair.a,
                .b = pair.b,
                .normal = info.normal,
                .contact = info.contact_points,
                .depth = info.depth,
            };

            // Insert into sorted cache (deepest first)
            if (self.bvh_contact_count < MAX_BVH_CONTACTS) {
                // Cache not full - find insertion point
                usz insert_pos = self.bvh_contact_count;
                for (usz i = 0; i < self.bvh_contact_count; i++) {
                    if (info.depth > self.bvh_contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                // Shift elements down
                for (usz i = self.bvh_contact_count; i > insert_pos; i--) {
                    self.bvh_contact_cache[i] = self.bvh_contact_cache[i - 1];
                }
                self.bvh_contact_cache[insert_pos] = new_contact;
                self.bvh_contact_count++;
            } else if (info.depth > self.bvh_contact_cache[MAX_BVH_CONTACTS - 1].depth) {
                // Cache full but this contact is deeper than shallowest - replace
                usz insert_pos = MAX_BVH_CONTACTS - 1;
                for (usz i = 0; i < MAX_BVH_CONTACTS - 1; i++) {
                    if (info.depth > self.bvh_contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                // Shift elements down (dropping last)
                for (usz i = MAX_BVH_CONTACTS - 1; i > insert_pos; i--) {
                    self.bvh_contact_cache[i] = self.bvh_contact_cache[i - 1];
                }
                self.bvh_contact_cache[insert_pos] = new_contact;
            }
        }
    }

    // Push the best contacts to collision list
    for (usz i = 0; i < self.bvh_contact_count; i++) {
        self.collisions.push(self.bvh_contact_cache[i]);
    }
}


// Thread-safe version of check_bvh_collision that returns result instead of pushing to list
fn Collision? PhysicsWorld.test_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) @local {
    bool a_has_bvh = body_a.has_mesh_bvh();

    Rigidbody* mesh_body;
    Rigidbody* other_body;
    bool mesh_is_a;

    if (a_has_bvh) {
        mesh_body = body_a;
        other_body = body_b;
        mesh_is_a = true;
    } else {
        mesh_body = body_b;
        other_body = body_a;
        mesh_is_a = false;
    }

    Mesh* mesh = (Mesh*)mesh_body.collider.shape;

    Aabb3 local_aabb = other_body.transformed_aabb.inverse_transform(
        mesh_body.collider.translation,
        mesh_body.collider.scale,
        mesh_body.collider.rotation
    );

    List{TriangleVerts} triangles;
    defer triangles.free();
    mesh.bvh.query_all_triangles(local_aabb, &triangles);

    Vec3 best_normal;
    Vec3[2] best_contact;
    float max_depth = 0;

    foreach (tri : triangles) {
        Mesh tri_mesh;
        tri_mesh.vertices.push_all({tri[0], tri[1], tri[2]});
        tri_mesh.triangles.push({0, 1, 2});
        defer tri_mesh.free();

        TransformedShape tri_shape = {
            .translation = mesh_body.collider.translation,
            .scale = mesh_body.collider.scale,
            .rotation = mesh_body.collider.rotation,
            .shape = &tri_mesh,
        };

        CollisionInfo info;
        if (mesh_is_a) {
            info = collision::check_convex_collision(&tri_shape, &other_body.collider, epa:true);
        } else {
            info = collision::check_convex_collision(&other_body.collider, &tri_shape, epa:true);
        }

        if (info.collided && info.depth > max_depth) {
            max_depth = info.depth;
            best_normal = info.normal;
            best_contact = info.contact_points;
        }
    }

    if (max_depth > 0) {
        return {
            .a = pair.a,
            .b = pair.b,
            .normal = best_normal,
            .contact = best_contact,
            .depth = max_depth,
        };
    }

    return {};
}

// Check collisions using CoACD decomposed convex pieces
// Each piece is tested as a convex-convex collision (much faster than BVH triangles)
fn void PhysicsWorld.check_coacd_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) {
    bool a_has_compound = body_a.collider.get_type() == ShapeType.COMPOUND;
    bool b_has_compound = body_b.collider.get_type() == ShapeType.COMPOUND;

    // Determine which body has the compound shape
    Rigidbody* compound_body;
    Rigidbody* other_body;
    bool compound_is_a;

    if (a_has_compound) {
        compound_body = body_a;
        other_body = body_b;
        compound_is_a = true;
    } else if (b_has_compound) {
        compound_body = body_b;
        other_body = body_a;
        compound_is_a = false;
    } else {
        // Neither has compound - fall back to regular convex collision
        CollisionInfo info = collision::check_convex_collision(&body_a.collider, &body_b.collider, epa:true);
        if (info.collided) {
            self.collisions.push({
                .a = pair.a,
                .b = pair.b,
                .normal = info.normal,
                .contact = info.contact_points,
                .depth = info.depth,
            });
        }
        return;
    }

    CompoundShape* compound = (CompoundShape*)compound_body.collider.shape;

    // Reset contact cache
    self.bvh_contact_count = 0;

    // Test each convex piece against the other body
    foreach (&piece : compound.shapes) {
        // Create transformed shape for this piece
        // Piece transform is relative to compound body
        TransformedShape piece_shape = {
            .translation = compound_body.collider.translation + compound_body.collider.rotation * piece.offset,
            .scale = compound_body.collider.scale,
            .rotation = compound_body.collider.rotation,
            .shape = piece.shape,
        };

        // Early AABB rejection (use transformed AABB for world-space comparison)
        Aabb3 piece_aabb = piece_shape.get_transformed_aabb();
        if (!piece_aabb.collides(other_body.transformed_aabb)) continue;

        // Check collision
        CollisionInfo info;
        if (compound_is_a) {
            info = collision::check_convex_collision(&piece_shape, &other_body.collider, epa:true);
        } else {
            info = collision::check_convex_collision(&other_body.collider, &piece_shape, epa:true);
        }

        if (info.collided && info.depth > 0) {
            Collision new_contact = {
                .a = pair.a,
                .b = pair.b,
                .normal = info.normal,
                .contact = info.contact_points,
                .depth = info.depth,
            };

            // Insert into sorted cache (deepest first), same as BVH
            if (self.bvh_contact_count < MAX_BVH_CONTACTS) {
                usz insert_pos = self.bvh_contact_count;
                for (usz i = 0; i < self.bvh_contact_count; i++) {
                    if (info.depth > self.bvh_contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                for (usz i = self.bvh_contact_count; i > insert_pos; i--) {
                    self.bvh_contact_cache[i] = self.bvh_contact_cache[i - 1];
                }
                self.bvh_contact_cache[insert_pos] = new_contact;
                self.bvh_contact_count++;
            } else if (info.depth > self.bvh_contact_cache[MAX_BVH_CONTACTS - 1].depth) {
                usz insert_pos = MAX_BVH_CONTACTS - 1;
                for (usz i = 0; i < MAX_BVH_CONTACTS - 1; i++) {
                    if (info.depth > self.bvh_contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                for (usz i = MAX_BVH_CONTACTS - 1; i > insert_pos; i--) {
                    self.bvh_contact_cache[i] = self.bvh_contact_cache[i - 1];
                }
                self.bvh_contact_cache[insert_pos] = new_contact;
            }
        }
    }

    // Push the best contacts to collision list
    for (usz i = 0; i < self.bvh_contact_count; i++) {
        self.collisions.push(self.bvh_contact_cache[i]);
    }
}

// ============================================================================
// BVH Hit info structure (separate from helpers.c3 HitInfo)
// ============================================================================
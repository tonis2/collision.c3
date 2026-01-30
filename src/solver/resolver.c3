module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;
import std::thread::pool;
import std::thread;
import std::atomic;
import std::core::mem;

const EPSILON @local = 0.001f;
const MAX_COLLISION_TASKS @local = 1024;
const NUM_WORKER_THREADS @local = 8;

alias CollisionFilter = HashMap{Pair, bool};
alias RigidBodyMap = LinkedHashMap{usz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator{usz, Rigidbody};
alias UnionFindMap = HashMap{usz, usz};

// Task structure for parallel collision testing
struct CollisionTask {
    PhysicsWorld* world;
    Pair pair;
    Collision* result;       // Where to store result
    bool has_collision;      // Flag if collision was found
}

// Task structure for parallel integration
struct IntegrationTask {
    PhysicsWorld* world;
    Rigidbody** bodies;      // Array of body pointers to process
    usz count;               // Number of bodies in this batch
    float time;
}

// Task structure for island constraint solving
struct IslandSolveTask {
    PhysicsWorld* world;
    Constraint** constraints; // Constraints for this island
    usz count;               // Number of constraints
    uint iterations;         // Solver iterations
}

struct Constraint {
    Rigidbody* body_a;
    Rigidbody* body_b;

    float* impulse_cache;

    float min;
    float max;
    float bias;
    float effective_mass;
    float friction;
    float bounce;

    usz impulse_offset;

    Vec3 linear_a;
    Vec3 angular_a;

    Vec3 linear_b;
    Vec3 angular_b;
}

struct Collision {
    usz a;
    usz b;
    Vec3 normal;
    Vec3[2] contact;
    float depth;
    Vec3 impulse;
}

enum PhysicsCombine: inline uint {
    AVERAGE,
    MINIMUM,
    MAXIMUM,
    MULTIPLY
}

struct Material {
    float static_friction;
    float dynamic_friction;
    float restitution;
    uint friction_combine;
    uint restitution_combine;
}

struct Rigidbody {
    inline TransformedShape collider;
    usz id;
    usz island_id;

    float sleep_timer;

    bool sleeping;
    bool can_sleep;
    bool is_static;
    int impulse_dir;

    Aabb3 aabb;
    Aabb3 transformed_aabb;
    Material material;

    // Motion
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    float mass;
    float inverse_mass;
    float gravity_factor;
    float linear_dampening;
    float angular_dampening;
	float sleep_delta;

    Vec3 mass_center;
    //Vec3 inertia_diagonal;
    //QuaternionValue inertia_orientation;
    Matrix3f inertia;
}

struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{Constraint} constraints;
    List{GenericJoint3D} joints;
    SpatialHash3D spatial_map;
    CollisionFilter filters;
    UnionFindMap island_parent;
	float angular_dampening;
	float linear_dampening;
    Vec3 gravity;
    float sleep_timer;
	float sleep_delta;
    bool islands_dirty;

    // Threading support
    // Template param sets both queue size AND thread count
    ThreadPool{NUM_WORKER_THREADS} thread_pool;
    Mutex collision_mutex;
    bool threaded;           // Enable/disable threading
    bool thread_pool_initialized;

    // Reusable task lists (avoid per-frame allocations)
    List{CollisionTask} collision_tasks;
    List{Collision} collision_results;
    List{IslandSolveTask} island_tasks;
    List{IntegrationTask} integration_tasks;
    List{Rigidbody*} active_bodies;

    // BVH collision cache (avoid per-triangle allocations)
    List{TriangleVerts} bvh_query_cache;
    Mesh tri_mesh_cache;
    bool bvh_cache_initialized;
    // Contact sorting cache (keep deepest N contacts)
    Collision[8] bvh_contact_cache;
    usz bvh_contact_count;
}

fn void PhysicsWorld.free(&self) {
    if (self.thread_pool_initialized) {
        self.thread_pool.stop_and_destroy()!!;
        self.collision_mutex.destroy()!!;
    }
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.constraints.free();
    self.filters.free();
    self.joints.free();
    self.island_parent.free();
    // Free reusable task lists
    self.collision_tasks.free();
    self.collision_results.free();
    self.island_tasks.free();
    self.integration_tasks.free();
    self.active_bodies.free();
    // Free BVH cache
    self.bvh_query_cache.free();
    if (self.bvh_cache_initialized) {
        self.tri_mesh_cache.vertices.free();
        self.tri_mesh_cache.triangles.free();
    }
}

const PhysicsWorld DEFAULT_PHYSICS_WORLD = {
	.gravity = {0.0f, 0.0f, -9.8f},
	.spatial_map.cell_size = 2.0,
	.sleep_timer = 5.0,
	.linear_dampening = 0.9,
	.angular_dampening = 0.9,
	.sleep_delta = 0.1,
	.threaded = true,
};

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody*? PhysicsWorld.find_body(self, usz id) => self.bodies.get_ref(id);

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint) {

    // Add collision filter for joints, so they won't collide with eachother
    if (joint.collides == false) {
        self.filters.set({joint.body_a, joint.body_b}, true);
    }

    self.joints.push(joint);
    self.islands_dirty = true;
}

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
 @require body.collider.rotation != (QuaternionValue){0,0,0,0} : `Body rotation must not be 0`
 @require body.collider.scale != {0,0,0} : `Body scale must not be 0`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    body.is_static = body.mass == 0;
    body.inverse_mass = !body.is_static ? 1.0f / body.mass : 0;

    body.impulse_dir = (self.gravity * body.gravity_factor).z > 0 ? 1 : -1;

    // Compute AABB from collider
    body.aabb = body.collider.get_aabb();
	body.sleep_delta = self.sleep_delta;

    body.transformed_aabb = body.get_transformed_aabb();
    // Use AABB-based inertia - consistent for all shape types including mesh
    if (!body.is_static && &body.collider.shape.inertia_tensor) body.inertia = body.aabb.inertia_tensor(body.mass).inverse()!!;

    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self) {
    return self.aabb.transform(translation: self.collider.translation, scale: self.collider.scale, rotation: self.collider.rotation);
}

fn Matrix3f Rigidbody.get_inverse_world_inertia(self) {
    if (self.is_static) return MATRIX3F_IDENTITY;
    return self.collider.world_inertia_from_local(self.inertia);
}

fn void Rigidbody.apply_linear_impulse(&self, Vec3 impulse) => self.linear_velocity += impulse * self.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3 impulse) {
    Matrix3f inertia = self.get_inverse_world_inertia();
    self.angular_velocity += inertia * impulse;
}

fn void Rigidbody.apply_impulse(&self, Vec3 impulse, Vec3 point) {
    self.linear_velocity += impulse * self.inverse_mass;
    self.apply_angular_impulse(point.cross(impulse));
}

// fn Vec3 Rigidbody.get_intertia(self, Vec3 point, Vec3 normal) {
//     return (self.get_inverse_world_inertia() * point.cross(normal)).cross(point);
// }

fn float Rigidbody.get_intertia(self, Vec3 point, Vec3 normal) {
    Vec3 ang = point.cross(normal);
    Vec3 accel = self.get_inverse_world_inertia() * ang;
    return accel.dot(ang);
}

fn Vec3 Rigidbody.relative_velocity(self, Vec3 point) => self.linear_velocity + self.angular_velocity.cross(point);

fn bool Rigidbody.is_rotating(self) => self.angular_velocity.sq_magnitude() > self.sleep_delta;
fn bool Rigidbody.is_moving(self) => self.linear_velocity.sq_magnitude() > self.sleep_delta;
fn bool Rigidbody.is_sleeping(self) => self.sleeping;
fn void Rigidbody.stop_sleeping(&self) {
    self.sleeping = false;
    self.sleep_timer = 0;
}
fn bool Rigidbody.is_active(self) => self.sleeping == false && self.is_static == false;

// Union-Find: find root with path compression
fn usz PhysicsWorld.island_find(&self, usz id) {
    if (!self.island_parent.has_key(id)) {
        self.island_parent.set(id, id);
        return id;
    }

    usz parent = self.island_parent.get(id)!!;
    if (parent != id) {
        // Path compression: point directly to root
        usz root = self.island_find(parent);
        self.island_parent.set(id, root);
        return root;
    }
    return id;
}

// Union-Find: merge two bodies into same island
fn void PhysicsWorld.island_union(&self, usz a, usz b) {
    usz root_a = self.island_find(a);
    usz root_b = self.island_find(b);

    if (root_a != root_b) {
        // Use smaller id as root for consistency
        if (root_a < root_b) {
            self.island_parent.set(root_b, root_a);
        } else {
            self.island_parent.set(root_a, root_b);
        }
    }
}

// Compute islands from joints - call when islands_dirty is true
fn void PhysicsWorld.compute_islands(&self) {
    if (!self.islands_dirty) return;

    // Reset parent map
    self.island_parent.clear();

    // Each body starts as its own island
    RigidBodyIterator iter = self.bodies.value_iter();
    while (iter.next()) {
        Rigidbody* body = iter.get()!!;
        self.island_parent.set(body.id, body.id);
    }

    // Union bodies connected by joints
    foreach (&joint: self.joints) {
        // Only union non-static bodies
        Rigidbody*? a = self.find_body(joint.body_a);
        Rigidbody*? b = self.find_body(joint.body_b);

        if (try body_a = a) {
            if (try body_b = b) {
                if (!body_a.is_static && !body_b.is_static) {
                    self.island_union(joint.body_a, joint.body_b);
                }
            }
        }
    }

    // Assign island_id to each body
    RigidBodyIterator assign_iter = self.bodies.value_iter();
    while (assign_iter.next()) {
        Rigidbody* body = assign_iter.get()!!;
        body.island_id = self.island_find(body.id);
    }

    self.islands_dirty = false;
}

// Wake all bodies in the same island - O(n) where n is total bodies
fn void PhysicsWorld.wake_island(&self, Rigidbody* body) {
    if (!body.sleeping) return;

    usz target_island = body.island_id;

    RigidBodyIterator iter = self.bodies.value_iter();
    while (iter.next()) {
        Rigidbody* other = iter.get()!!;
        if (other.island_id == target_island && other.sleeping) {
            other.stop_sleeping();
        }
    }
}


fn bool Rigidbody.has_mesh_bvh(&self) {
    if (self.collider.get_type() == ShapeType.CONVEX) {
        Mesh* mesh = (Mesh*)self.collider.shape;
        return mesh.has_bvh();
    }
    return false;
}

fn bool Rigidbody.has_compound(&self) {
    return self.collider.get_type() == ShapeType.COMPOUND;
}

// Worker function for parallel collision testing (convex vs convex)
fn int collision_test_worker(void* arg) @local {
    CollisionTask* task = (CollisionTask*)arg;
    PhysicsWorld* world = task.world;

    Rigidbody* body_a = world.find_body(task.pair.a)!!;
    Rigidbody* body_b = world.find_body(task.pair.b)!!;

    // Check for BVH mesh collisions
    bool a_has_bvh = body_a.has_mesh_bvh();
    bool b_has_bvh = body_b.has_mesh_bvh();

    if (a_has_bvh || b_has_bvh) {
        // BVH collision - store result directly (thread-safe via separate result slots)
        Collision? collision = world.test_bvh_collision(body_a, body_b, task.pair);
        if (try c = collision) {
            *task.result = c;
            task.has_collision = true;
        }
    } else {
        // GJK collision test
        CollisionInfo info = collision::check_convex_collision(&body_a.collider, &body_b.collider, epa:true);

        if (info.collided) {
            *task.result = {
                .a = task.pair.a,
                .b = task.pair.b,
                .normal = info.normal,
                .contact = info.contact_points,
                .depth = info.depth,
            };
            task.has_collision = true;
        }
    }
    return 0;
}

// Worker function for parallel integration
fn int integration_worker(void* arg) @local {
    IntegrationTask* task = (IntegrationTask*)arg;
    PhysicsWorld* world = task.world;
    float time = task.time;

    for (usz i = 0; i < task.count; i++) {
        Rigidbody* body = task.bodies[i];

        // Apply linear velocity and translation
        body.collider.translation += body.linear_velocity * time;

        // Apply angular velocity as rotation
        Vec3 axis = body.angular_velocity * (time * 0.5);
        if (axis != {0,0,0}) {
            body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
        }

        // Apply gravity
        body.apply_linear_impulse(world.gravity * body.gravity_factor * body.mass * time);

        // Apply dampening friction
        body.angular_velocity *= math::pow(world.angular_dampening, time);
        body.linear_velocity *= math::pow(world.linear_dampening, time);

        // Clamp very small velocities to zero
        const float VELOCITY_CLAMP_SQ = 0.01f;
        if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.linear_velocity = {0, 0, 0};
        }
        if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.angular_velocity = {0, 0, 0};
        }
    }
    return 0;
}

// Worker function for island-parallel constraint solving
fn int island_solve_worker(void* arg) @local {
    IslandSolveTask* task = (IslandSolveTask*)arg;

    for (uint iter = 0; iter < task.iterations; iter++) {
        for (usz i = 0; i < task.count; i++) {
            Constraint* constraint = task.constraints[i];
            solve_constraint_internal(constraint);
        }
    }
    return 0;
}

// Internal constraint solver (no PhysicsWorld needed)
fn void solve_constraint_internal(Constraint* constraint) @local {
    Rigidbody* a = constraint.body_a;
    Rigidbody* b = constraint.body_b;

    float* cache = constraint.impulse_cache;

    // Total velocity difference (angular + linear)
    float relative_velocity = constraint.relative_velocity();

    // Calculate Lambda (The corrective impulse magnitude)
    float lambda = -(relative_velocity + constraint.bias + constraint.bounce) * constraint.effective_mass;

    if (constraint.friction != 0) {
        float max_friction = constraint.friction * cache[0];
        constraint.min = -max_friction;
        constraint.max = max_friction;
    }

    // Accumulate impulse with clamping
    float new_impulse = math::max(constraint.min, math::min(lambda, constraint.max));

    cache[constraint.impulse_offset] = new_impulse;

    // Apply impulse
    if (!a.is_static) {
        a.apply_linear_impulse(constraint.linear_a * new_impulse);
        a.apply_angular_impulse(constraint.angular_a * new_impulse);
    }

    if (!b.is_static) {
        b.apply_linear_impulse(constraint.linear_b * new_impulse);
        b.apply_angular_impulse(constraint.angular_b * new_impulse);
    }
}


<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 15) {
    if (self.threaded) {
        self.run_step_parallel(time, step_count);
    } else {
        self.run_step_sequential(time, step_count);
    }
}

// Sequential (single-threaded) physics step - original implementation
fn void PhysicsWorld.run_step_sequential(&self, float time, uint step_count) @local {
    // Recompute islands if joint topology changed
    self.compute_islands();

    // Create collisions
    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.find_body(pair.a)!!;
        Rigidbody* body_b = self.find_body(pair.b)!!;

        bool has_filter = self.filters.has_key({body_a.id, body_b.id}) == true;

        if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb)) {
            if (body_a.sleeping && body_b.is_active()) self.wake_island(body_a);
            if (body_b.sleeping && body_a.is_active()) self.wake_island(body_b);

            if (body_a.is_active() || body_b.is_active()) {
                bool a_has_compound = body_a.has_compound();
                bool b_has_compound = body_b.has_compound();

                if (a_has_compound || b_has_compound) {
                    self.check_coacd_collision(body_a, body_b, pair);
                } else {
                    CollisionInfo collision_info = collision::check_convex_collision(&body_a.collider, &body_b.collider, epa:true);

                    if (collision_info.collided) {
                        self.collisions.push({
                            .a = pair.a,
                            .b = pair.b,
                            .normal = collision_info.normal,
                            .contact = collision_info.contact_points,
                            .depth = collision_info.depth,
                        });
                    }
                }
            }
        }
    };

    foreach (&collision: self.collisions) {
        collision.create_constraint(self, time);
    }

    foreach (&joint: self.joints) {
        joint.create_constraint(self, time);
    }

    for (usz i; i < step_count; i++) {
        foreach (&contraint: self.constraints) {
            self.solve_constraint(contraint);
        }
    }

    self.collisions.clear();
    self.constraints.clear();

    self.integrate_bodies(time);
    self.update_sleep_state();
}

// Parallel (multi-threaded) physics step
fn void PhysicsWorld.run_step_parallel(&self, float time, uint step_count) @local {
    // Lazy init thread pool (can't do in create_world because workers get pointer before move)
    if (!self.thread_pool_initialized) {
        self.thread_pool.init()!!;
        self.collision_mutex.init()!!;
        self.thread_pool_initialized = true;
    }

    // Recompute islands if joint topology changed
    self.compute_islands();

    // === PHASE 1: Broad-phase - collect candidate pairs (sequential, fast) ===
    List{Pair} candidate_pairs;
    defer candidate_pairs.free();

    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.find_body(pair.a)!!;
        Rigidbody* body_b = self.find_body(pair.b)!!;

        bool has_filter = self.filters.has_key({body_a.id, body_b.id}) == true;

        if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb)) {
            // Wake sleeping bodies (must be sequential due to island wake)
            if (body_a.sleeping && body_b.is_active()) self.wake_island(body_a);
            if (body_b.sleeping && body_a.is_active()) self.wake_island(body_b);

            if (body_a.is_active() || body_b.is_active()) {
                candidate_pairs.push(pair);
            }
        }
    };

    // === PHASE 2: Narrow-phase collision testing ===
    // BVH collisions run sequentially (allocator not thread-safe)
    // Convex-only collisions can run in parallel
    usz num_pairs = candidate_pairs.len();

    if (num_pairs > 0) {
        // Separate BVH and convex-only pairs
        List{Pair} convex_pairs;
        List{Pair} bvh_pairs;
        defer convex_pairs.free();
        defer bvh_pairs.free();

        for (usz i = 0; i < num_pairs; i++) {
            Pair pair = candidate_pairs[i];
            Rigidbody* body_a = self.find_body(pair.a)!!;
            Rigidbody* body_b = self.find_body(pair.b)!!;

            if (body_a.has_compound() || body_b.has_compound()) {
                bvh_pairs.push(pair);
            } else {
                convex_pairs.push(pair);
            }
        }

        // Process convex pairs in parallel
        usz num_convex = convex_pairs.len();
        if (num_convex > 0) {
            self.collision_tasks.clear();
            self.collision_results.clear();

            for (usz i = 0; i < num_convex; i++) {
                self.collision_results.push({});
            }

            for (usz i = 0; i < num_convex; i++) {
                self.collision_tasks.push({
                    .world = self,
                    .pair = convex_pairs[i],
                    .result = &self.collision_results[i],
                    .has_collision = false,
                });
            }

            for (usz i = 0; i < num_convex; i++) {
                self.thread_pool.push(&collision_test_worker, &self.collision_tasks[i])!!;
            }

            self.thread_pool.join()!!;

            for (usz i = 0; i < num_convex; i++) {
                if (self.collision_tasks[i].has_collision) {
                    self.collisions.push(*self.collision_tasks[i].result);
                }
            }
        }

        // Process BVH pairs sequentially (thread-safe)
        foreach (pair : bvh_pairs) {
            Rigidbody* body_a = self.find_body(pair.a)!!;
            Rigidbody* body_b = self.find_body(pair.b)!!;
            self.check_coacd_collision(body_a, body_b, pair);
        }
    }

    // === PHASE 3: Create constraints (sequential, fast) ===
    foreach (&collision: self.collisions) {
        collision.create_constraint(self, time);
    }

    foreach (&joint: self.joints) {
        joint.create_constraint(self, time);
    }

    // === PHASE 4: Solve constraints - island-parallel ===
    self.solve_constraints_parallel(step_count);

    self.collisions.clear();
    self.constraints.clear();

    // === PHASE 5: Integration (parallel physics, sequential spatial map) ===
    self.integrate_bodies_parallel(time);

    // === PHASE 6: Sleep management (sequential) ===
    self.update_sleep_state();
}

// Island-parallel constraint solving
fn void PhysicsWorld.solve_constraints_parallel(&self, uint step_count) @local {
    if (self.constraints.len() == 0) return;

    // Group constraints by island
    HashMap{usz, List{Constraint*}} island_constraints;
    defer {
        island_constraints.@each(; usz id, List{Constraint*} list) {
            list.free();
        };
        island_constraints.free();
    }

    for (usz i = 0; i < self.constraints.len(); i++) {
        Constraint* constraint = &self.constraints[i];
        // Use body_a's island (both bodies should be in same island if connected)
        usz island_id = constraint.body_a.island_id;

        if (!island_constraints.has_key(island_id)) {
            island_constraints.set(island_id, {});
        }
        island_constraints.get_ref(island_id)!!.push(constraint);
    }

    // Count islands
    usz num_islands = island_constraints.len();

    if (num_islands <= 1) {
        // Single island or no islands - run sequential
        for (usz i; i < step_count; i++) {
            foreach (&contraint: self.constraints) {
                solve_constraint_internal(contraint);
            }
        }
        return;
    }

    // Create island solve tasks (reuse list)
    self.island_tasks.clear();

    island_constraints.@each(; usz id, List{Constraint*} list) {
        Constraint*[] arr = list.array_view();

        self.island_tasks.push({
            .world = self,
            .constraints = arr.ptr,
            .count = arr.len,
            .iterations = step_count,
        });
    };

    // Submit island solving tasks
    for (usz i = 0; i < num_islands; i++) {
        self.thread_pool.push(&island_solve_worker, &self.island_tasks[i])!!;
    }

    // Wait for all islands to be solved
    self.thread_pool.join()!!;
}

// Parallel integration - physics updates in parallel, spatial map sequential
fn void PhysicsWorld.integrate_bodies_parallel(&self, float time) @local {
    // Collect active bodies for parallel processing (reuse list)
    self.active_bodies.clear();

    RigidBodyIterator collect_iter = self.bodies.value_iter();
    while (collect_iter.next()) {
        Rigidbody* body = collect_iter.get()!!;
        if (body.is_active()) {
            self.active_bodies.push(body);
        }
    }

    usz num_active = self.active_bodies.len();
    if (num_active == 0) return;

    // Determine batch size for parallel work
    usz batch_size = (num_active + NUM_WORKER_THREADS - 1) / NUM_WORKER_THREADS;

    // Only parallelize if we have enough bodies
    if (num_active >= NUM_WORKER_THREADS * 2) {
        // Create integration tasks (reuse list)
        usz num_tasks = (num_active + batch_size - 1) / batch_size;
        self.integration_tasks.clear();

        Rigidbody** all_bodies_ptr = self.active_bodies.array_view().ptr;

        for (usz i = 0; i < num_tasks; i++) {
            usz start = i * batch_size;
            usz count = math::min(batch_size, num_active - start);

            self.integration_tasks.push({
                .world = self,
                .bodies = all_bodies_ptr + start,
                .count = count,
                .time = time,
            });
        }

        // Submit integration tasks
        for (usz i = 0; i < num_tasks; i++) {
            self.thread_pool.push(&integration_worker, &self.integration_tasks[i])!!;
        }

        // Wait for physics updates to complete
        self.thread_pool.join()!!;
    } else {
        // Too few bodies - just do physics updates inline
        foreach (body : self.active_bodies) {
            // Apply linear velocity and translation
            body.collider.translation += body.linear_velocity * time;

            // Apply angular velocity as rotation
            Vec3 axis = body.angular_velocity * (time * 0.5);
            if (axis != {0,0,0}) {
                body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
            }

            // Apply gravity
            body.apply_linear_impulse(self.gravity * body.gravity_factor * body.mass * time);

            // Apply dampening friction
            body.angular_velocity *= math::pow(self.angular_dampening, time);
            body.linear_velocity *= math::pow(self.linear_dampening, time);

            // Clamp very small velocities to zero
            const float VELOCITY_CLAMP_SQ = 0.01f;
            if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
                body.linear_velocity = {0, 0, 0};
            }
            if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
                body.angular_velocity = {0, 0, 0};
            }
        }
    }

    // Sequential: Update spatial map and sleep timers (requires shared data structure access)
    foreach (body : self.active_bodies) {
        bool rotating = body.is_rotating();
        bool moving = body.is_moving();

        if (rotating || moving) {
            Aabb3 new_aabb = body.get_transformed_aabb();
            self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
            body.transformed_aabb = new_aabb;
        }

        // Increment sleep timer when body is at rest
        if (body.can_sleep == true && moving == false && rotating == false) {
            body.sleep_timer += time;
        } else {
            body.sleep_timer = 0;
        }
    }
}

// Shared integration logic (sequential version)
fn void PhysicsWorld.integrate_bodies(&self, float time) @local {
    RigidBodyIterator rigid_bodies = self.bodies.value_iter();
    while (rigid_bodies.next()) {
        Rigidbody* body = rigid_bodies.get()!!;

        if (!body.is_active()) continue;

        // Apply linear velocity and translation
        body.collider.translation += body.linear_velocity * time;

        // Apply angular velocity as rotation
        Vec3 axis = body.angular_velocity * (time * 0.5);
        if (axis != {0,0,0}) {
            body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
        }

        // Apply gravity
        body.apply_linear_impulse(self.gravity * body.gravity_factor * body.mass * time);

        // Apply dampening friction
        body.angular_velocity *= math::pow(self.angular_dampening, time);
        body.linear_velocity *= math::pow(self.linear_dampening, time);

        // Clamp very small velocities to zero
        const float VELOCITY_CLAMP_SQ = 0.01f;
        if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.linear_velocity = {0, 0, 0};
        }
        if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.angular_velocity = {0, 0, 0};
        }

        bool rotating = body.is_rotating();
        bool moving = body.is_moving();

        // Update spatial map (must be sequential)
        if (rotating || moving) {
            Aabb3 new_aabb = body.get_transformed_aabb();
            self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
            body.transformed_aabb = new_aabb;
        }

        // Increment sleep timer when body is at rest
        if (body.can_sleep == true && moving == false && rotating == false) {
            body.sleep_timer += time;
        } else {
            body.sleep_timer = 0;
        }
    }
}

// Shared sleep state update logic
fn void PhysicsWorld.update_sleep_state(&self) @local {
    HashMap{usz, bool} island_can_sleep;
    HashMap{usz, bool} island_checked;
    defer island_can_sleep.free();
    defer island_checked.free();

    // First pass: determine which islands can sleep
    RigidBodyIterator sleep_check = self.bodies.value_iter();
    while (sleep_check.next()) {
        Rigidbody* body = sleep_check.get()!!;

        if (body.is_static) continue;

        usz island = body.island_id;

        if (!island_checked.has_key(island)) {
            island_checked.set(island, true);
            island_can_sleep.set(island, true);
        }

        if (!body.can_sleep || body.is_moving() || body.is_rotating() || body.sleep_timer < self.sleep_timer) {
            island_can_sleep.set(island, false);
        }
    }

    // Second pass: put entire islands to sleep
    RigidBodyIterator sleep_apply = self.bodies.value_iter();
    while (sleep_apply.next()) {
        Rigidbody* body = sleep_apply.get()!!;

        if (body.is_static || body.sleeping) continue;

        if (try can_sleep = island_can_sleep.get(body.island_id)) {
            if (can_sleep) {
                body.sleeping = true;
                body.sleep_timer = 0;
            }
        }
    }
}

<*
 @require constraint.body_a != constraint.body_b : `Collision bodies should not be same index`
*>
fn void PhysicsWorld.solve_constraint(&self, Constraint* constraint) @local {
    Rigidbody* a = constraint.body_a;
    Rigidbody* b = constraint.body_b;

    float* cache = constraint.impulse_cache;

    // Total velocity difference (angular + linear)
    float relative_velocity = constraint.relative_velocity();

    // Calculate Lambda (The corrective impulse magnitude)
    float lambda = -(relative_velocity + constraint.bias + constraint.bounce) * constraint.effective_mass;

    if (constraint.friction != 0) {
        float max_friction = constraint.friction * cache[0];
        constraint.min = -max_friction;
        constraint.max = max_friction;
    }

    // Accumulate impulse with clamping (for proper friction based on total normal force)
    float old_impulse = cache[constraint.impulse_offset];
    float new_impulse = math::max(constraint.min, math::min(lambda, constraint.max));
    // float impulse_delta = new_impulse - old_impulse;

    // Store accumulated impulse (friction constraints need total normal impulse from cache[0])
    cache[constraint.impulse_offset] = new_impulse;

    // Apply impulse delta (not the accumulated total)
    if (!a.is_static) {
        a.apply_linear_impulse(constraint.linear_a * new_impulse);
        a.apply_angular_impulse(constraint.angular_a * new_impulse);
    }

    if (!b.is_static) {
        b.apply_linear_impulse(constraint.linear_b * new_impulse);
        b.apply_angular_impulse(constraint.angular_b * new_impulse);
    }
}

fn float Constraint.relative_velocity(self) {
    Rigidbody* a = self.body_a;
    Rigidbody* b = self.body_b;
    return self.linear_a.dot(a.linear_velocity)
            + self.angular_a.dot(a.angular_velocity)
            + self.linear_b.dot(b.linear_velocity)
            + self.angular_b.dot(b.angular_velocity);
}

fn float combine_values(float a, float b, PhysicsCombine mode) @local {
    switch (mode) {
        case AVERAGE: return (a + b) * 0.5f;
        case MINIMUM: return math::min(a, b);
        case MAXIMUM: return math::max(a, b);
        case MULTIPLY:	return a * b;
		default: return a * b;
    }
}

fn void Collision.create_constraint(&collision, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(collision.a)!!;
    Rigidbody* b = world.find_body(collision.b)!!;
    Vec3 normal = collision.normal.normalize();

    float* impulse_cache = (float*)&collision.impulse;

    float inverse_mass_sum = a.inverse_mass + b.inverse_mass;

    // Combine material properties using the specified combine mode from GLTF materials
    PhysicsCombine friction_mode = (PhysicsCombine)math::max((int)a.material.friction_combine, (int)b.material.friction_combine);
    PhysicsCombine restitution_mode = (PhysicsCombine)math::max((int)a.material.restitution_combine, (int)b.material.restitution_combine);

    float restitution = combine_values(a.material.restitution, b.material.restitution, restitution_mode);
    float static_friction = combine_values(a.material.static_friction, b.material.static_friction, friction_mode);

    Vec3 rA = collision.contact[0] - a.collider.translation;
    Vec3 rB = collision.contact[1] - b.collider.translation;
 	float beta = 0.2f;
    float slop = 0.01f;
    int impulse_dir = math::min(a.impulse_dir, b.impulse_dir);

    float bias = (beta / dt) * math::max(collision.depth - slop, 0.0f) * impulse_dir;
    float normal_inertia;
    if (!a.is_static) normal_inertia += a.get_intertia(rA, -normal);
    if (!b.is_static) normal_inertia += b.get_intertia(rB, normal);

    Vec3 linear_a = -normal;
    Vec3 angular_a = rA.cross(-normal);

    Vec3 linear_b = normal;
    Vec3 angular_b = rB.cross(normal);

    Constraint normal_constraint = {
        .body_a = a,
        .body_b = b,
        .linear_a = linear_a,
        .angular_a = angular_a,
        .linear_b = linear_b,
        .angular_b = angular_b,
        .effective_mass = 1 / (inverse_mass_sum + normal_inertia),
        .min = 0,
        .max = float.max,
        .bias = bias,
        .impulse_cache = impulse_cache,
        .impulse_offset = 0,
    };

    float relative_velocity = normal_constraint.relative_velocity();

    // Only apply restitution above velocity threshold to prevent micro-bouncing at rest
    const float BOUNCE_THRESHOLD = 1.0f;
    if (math::abs(relative_velocity) > BOUNCE_THRESHOLD) {
        normal_constraint.bounce = relative_velocity * restitution;
    } else {
        normal_constraint.bounce = 0;
    }

    Vec3 helper = (math::abs(normal.y) > 0.99)
        ? {1.0f, 0.0f, 0.0f}
        : {0.0f, 1.0f, 0.0f};
  
    Vec3 tangent_1 = normal.cross(helper).normalize();
    Vec3 tangent_2 = normal.cross(tangent_1);

    float tangent_inertia;
    if (!a.is_static) tangent_inertia += a.get_intertia(rA, -tangent_1);
    if (!b.is_static) tangent_inertia += b.get_intertia(rB, tangent_1);

    float tangent_inertia_2;
    if (!a.is_static) tangent_inertia_2 += a.get_intertia(rA, -tangent_2);
    if (!b.is_static) tangent_inertia_2 += b.get_intertia(rB, tangent_2);

    // FRICTION
    Constraint tangent_constraint = {
        ...normal_constraint,
        .friction = static_friction,
        .linear_a = -tangent_1,
        .angular_a = rA.cross(-tangent_1),
        .linear_b = tangent_1,
        .angular_b = rB.cross(tangent_1),
        .bias = 0,
        .effective_mass = 1 / (inverse_mass_sum + tangent_inertia),
        .impulse_offset = 1,  
    };
  
    Constraint tangent_constraint_2 = {
        ...normal_constraint,
        .friction = static_friction,
        .linear_a = -tangent_2,
        .angular_a = rA.cross(-tangent_2),
        .linear_b = tangent_2,
        .angular_b = rB.cross(tangent_2),
        .bias = 0,
        .effective_mass = 1 / (inverse_mass_sum + tangent_inertia_2),
        .impulse_offset = 2,
    };

    world.constraints.push_all({normal_constraint, tangent_constraint, tangent_constraint_2});
}
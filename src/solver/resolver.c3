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
const MAX_CONTACTS_PER_PAIR @local = 8;  // Max contacts per collision pair

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

enum TriggerEventType : char {
    ENTER,
    EXIT
}

struct TriggerEvent {
    usz trigger_id;
    usz other_id;
    TriggerEventType type;
}

enum ContactEventType : char {
    START,   // First frame of contact
    END      // Contact ended
}

struct ContactEvent {
    usz body_a;
    usz body_b;
    ContactEventType type;
    Vec3 normal;        // Collision normal (only valid for START)
    Vec3 contact_point; // Contact point (only valid for START)
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
    bool is_trigger;
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

alias TriggerOverlapSet = HashMap{Pair, bool};
alias ContactOverlapSet = HashMap{Pair, bool};

struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{Constraint} constraints;
    List{GenericJoint3D} joints;
    SpatialHash3D spatial_map;
    CollisionFilter filters;
    UnionFindMap island_parent;
	bool islands_dirty;

    // Trigger system
    List{TriggerEvent} trigger_events;
    TriggerOverlapSet active_triggers;  // Currently overlapping trigger pairs

    // Contact event system
    List{ContactEvent} contact_events;
    ContactOverlapSet active_contacts;  // Currently contacting body pairs

    Vec3 gravity;
    float sleep_timer;
	float sleep_delta;
	float angular_dampening;
	float linear_dampening;

    // Threading support
    // Template param sets both queue size AND thread count
    ThreadPool{NUM_WORKER_THREADS} thread_pool;
    Mutex collision_mutex;
    bool thread_pool_initialized;

    // Reusable task lists (avoid per-frame allocations)
    List{CollisionTask} collision_tasks;
    List{Collision} collision_results;
    List{IslandSolveTask} island_tasks;
    List{IntegrationTask} integration_tasks;
    List{Rigidbody*} active_bodies;

    // Contact sorting cache (keep deepest N contacts per pair)
    Collision[MAX_CONTACTS_PER_PAIR] contact_cache;
    usz contact_cache_count;
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
    // Free trigger system
    self.trigger_events.free();
    self.active_triggers.free();
    // Free contact event system
    self.contact_events.free();
    self.active_contacts.free();
}

const PhysicsWorld DEFAULT_PHYSICS_WORLD = {
	.gravity = {0.0f, 0.0f, -9.8f},
	.spatial_map.cell_size = 2.0,
	.sleep_timer = 5.0,
	.linear_dampening = 0.9,
	.angular_dampening = 0.9,
	.sleep_delta = 0.1,
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

<*
 Add a trigger body - detects overlaps but doesn't generate collision responses.
 Use @get_trigger_events() to iterate trigger events after run_step().

 @require body.collider.shape != null : `Trigger must have collision shape`
 @require body.collider.rotation != (QuaternionValue){0,0,0,0} : `Trigger rotation must not be 0`
 @require body.collider.scale != {0,0,0} : `Trigger scale must not be 0`
*>
fn void? PhysicsWorld.add_trigger(&self, Rigidbody body) {
    body.is_trigger = true;
    body.is_static = true;  // Triggers don't move from physics
    body.inverse_mass = 0;
    body.mass = 0;

    // Compute AABB from collider
    body.aabb = body.collider.get_aabb();
    body.transformed_aabb = body.get_transformed_aabb();

    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

<*
 Iterate trigger events generated during the last run_step().
 Events include ENTER (overlap started) and EXIT (overlap ended).
*>
macro void PhysicsWorld.@get_trigger_events(&self; @callback(TriggerEvent event)) {
    foreach (event : self.trigger_events) {
        @callback(event);
    }
}

<*
 Iterate contact events generated during the last run_step().
 Events include START (first frame of contact) and END (contact ended).
 Unlike triggers, contacts also generate physics responses (impulses).
*>
macro void PhysicsWorld.@get_contact_events(&self; @callback(ContactEvent event)) {
    foreach (event : self.contact_events) {
        @callback(event);
    }
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

// Worker function for parallel collision testing (convex vs convex only)
// BVH and compound pairs are handled separately in the main loop
fn int collision_test_worker(void* arg) @local {
    CollisionTask* task = (CollisionTask*)arg;
    PhysicsWorld* world = task.world;

    Rigidbody* body_a = world.find_body(task.pair.a)!!;
    Rigidbody* body_b = world.find_body(task.pair.b)!!;

    // GJK/EPA collision test for convex shapes
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
    // Lazy init thread pool (can't do in create_world because workers get pointer before move)
    if (!self.thread_pool_initialized) {
        self.thread_pool.init()!!;
        self.collision_mutex.init()!!;
        self.thread_pool_initialized = true;
    }

    // Clear events from previous frame
    self.trigger_events.clear();
    self.contact_events.clear();

    // Recompute islands if joint topology changed
    self.compute_islands();

    // === PHASE 1: Broad-phase - collect candidate pairs (sequential, fast) ===
    List{Pair} candidate_pairs;
    TriggerOverlapSet current_triggers;  // Triggers overlapping this frame
    defer candidate_pairs.free();
    defer current_triggers.free();

    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.find_body(pair.a)!!;
        Rigidbody* body_b = self.find_body(pair.b)!!;

        bool has_filter = self.filters.has_key({body_a.id, body_b.id}) == true;

        if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb)) {
            // Check if either body is a trigger
            if (body_a.is_trigger || body_b.is_trigger) {
                // Normalize pair so trigger is always first
                Pair trigger_pair = body_a.is_trigger ? pair : {pair.b, pair.a};
                current_triggers.set(trigger_pair, true);

                // Generate ENTER event if this is a new overlap
                if (!self.active_triggers.has_key(trigger_pair)) {
                    self.trigger_events.push({
                        .trigger_id = trigger_pair.a,
                        .other_id = trigger_pair.b,
                        .type = TriggerEventType.ENTER
                    });
                }
            } else {
                // Regular collision pair
                // Wake sleeping bodies (must be sequential due to island wake)
                if (body_a.sleeping && body_b.is_active()) self.wake_island(body_a);
                if (body_b.sleeping && body_a.is_active()) self.wake_island(body_b);

                if (body_a.is_active() || body_b.is_active()) {
                    candidate_pairs.push(pair);
                }
            }
        }
    };

    // Generate EXIT events for triggers that are no longer overlapping
    self.active_triggers.@each(;Pair old_pair, bool active) {
        if (!current_triggers.has_key(old_pair)) {
            self.trigger_events.push({
                .trigger_id = old_pair.a,
                .other_id = old_pair.b,
                .type = TriggerEventType.EXIT
            });
        }
    };

    // Swap active triggers with current frame's triggers
    self.active_triggers.free();
    self.active_triggers = current_triggers;
    current_triggers = {};  // Prevent double-free in defer

    // === PHASE 2: Narrow-phase collision testing ===
    // BVH collisions run sequentially (allocator not thread-safe)
    // Convex-only collisions can run in parallel
    usz num_pairs = candidate_pairs.len();

    if (num_pairs > 0) {
        // Separate pairs by collision type
        List{Pair} convex_pairs;
        List{Pair} bvh_pairs;
        List{Pair} compound_pairs;
        defer convex_pairs.free();
        defer bvh_pairs.free();
        defer compound_pairs.free();

        for (usz i = 0; i < num_pairs; i++) {
            Pair pair = candidate_pairs[i];
            Rigidbody* body_a = self.find_body(pair.a)!!;
            Rigidbody* body_b = self.find_body(pair.b)!!;

            if (body_a.has_mesh_bvh() || body_b.has_mesh_bvh()) {
                bvh_pairs.push(pair);
            } else if (body_a.has_compound() || body_b.has_compound()) {
                compound_pairs.push(pair);
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

        foreach (pair : bvh_pairs) {
            Rigidbody* body_a = self.find_body(pair.a)!!;
            Rigidbody* body_b = self.find_body(pair.b)!!;
            Collision? collision = self.test_bvh_collision(body_a, body_b, pair);
            if (try c = collision) {
                self.collisions.push(c);
            }
        }

        foreach (pair : compound_pairs) {
            Rigidbody* body_a = self.find_body(pair.a)!!;
            Rigidbody* body_b = self.find_body(pair.b)!!;
            self.check_coacd_collision(body_a, body_b, pair);
        }
    }

    // === PHASE 2.5: Generate contact events ===
    ContactOverlapSet current_contacts;
    defer current_contacts.free();

    // Track all collisions this frame and generate START events
    foreach (&collision: self.collisions) {
        Pair contact_pair = { collision.a, collision.b };
        current_contacts.set(contact_pair, true);

        // Generate START event if this is a new contact
        if (!self.active_contacts.has_key(contact_pair)) {
            self.contact_events.push({
                .body_a = collision.a,
                .body_b = collision.b,
                .type = ContactEventType.START,
                .normal = collision.normal,
                .contact_point = collision.contact[0]
            });
        }
    }

    // Generate END events for contacts that ended
    self.active_contacts.@each(;Pair old_pair, bool active) {
        if (!current_contacts.has_key(old_pair)) {
            self.contact_events.push({
                .body_a = old_pair.a,
                .body_b = old_pair.b,
                .type = ContactEventType.END,
                .normal = {0, 0, 0},
                .contact_point = {0, 0, 0}
            });
        }
    };

    // Swap active contacts with current frame's contacts
    self.active_contacts.free();
    self.active_contacts = current_contacts;
    current_contacts = {};  // Prevent double-free in defer

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
    float normal_inertia = 0;
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


// Check collisions using CoACD decomposed convex pieces
// Each piece is tested as a convex-convex collision (much faster than BVH triangles)
// Note: At least one body must have a compound shape (pairs are pre-categorized)
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
    } else {
        // b must have compound (pairs are pre-categorized to ensure at least one has compound)
        compound_body = body_b;
        other_body = body_a;
        compound_is_a = false;
    }

    CompoundShape* compound = (CompoundShape*)compound_body.collider.shape;

    // Reset contact cache
    self.contact_cache_count = 0;

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
            if (self.contact_cache_count < MAX_CONTACTS_PER_PAIR) {
                usz insert_pos = self.contact_cache_count;
                for (usz i = 0; i < self.contact_cache_count; i++) {
                    if (info.depth > self.contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                for (usz i = self.contact_cache_count; i > insert_pos; i--) {
                    self.contact_cache[i] = self.contact_cache[i - 1];
                }
                self.contact_cache[insert_pos] = new_contact;
                self.contact_cache_count++;
            } else if (info.depth > self.contact_cache[MAX_CONTACTS_PER_PAIR - 1].depth) {
                usz insert_pos = MAX_CONTACTS_PER_PAIR - 1;
                for (usz i = 0; i < MAX_CONTACTS_PER_PAIR - 1; i++) {
                    if (info.depth > self.contact_cache[i].depth) {
                        insert_pos = i;
                        break;
                    }
                }
                for (usz i = MAX_CONTACTS_PER_PAIR - 1; i > insert_pos; i--) {
                    self.contact_cache[i] = self.contact_cache[i - 1];
                }
                self.contact_cache[insert_pos] = new_contact;
            }
        }
    }

    // Push the best contacts to collision list
    for (usz i = 0; i < self.contact_cache_count; i++) {
        self.collisions.push(self.contact_cache[i]);
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

    // Pre-compute transform values for AABB calculation
    Vec3 mesh_translation = mesh_body.collider.translation;
    Vec3 mesh_scale = mesh_body.collider.scale;
    QuaternionValue mesh_rotation = mesh_body.collider.rotation;

    Aabb3 local_aabb = other_body.transformed_aabb.inverse_transform(
        mesh_translation,
        mesh_scale,
        mesh_rotation
    );

    List{TriangleVerts} triangles;
    triangles.init(allocator: mem);
    defer triangles.free();
    mesh.bvh.query_all_triangles(local_aabb, &triangles);

    // Create reusable triangle mesh once (thread-local, no shared state)
    // Use heap allocator to avoid temp allocator issues on worker threads
    Mesh tri_mesh;
    tri_mesh.vertices.init(allocator: mem);
    tri_mesh.triangles.init(allocator: mem);
    tri_mesh.vertices.push({0,0,0});
    tri_mesh.vertices.push({0,0,0});
    tri_mesh.vertices.push({0,0,0});
    tri_mesh.triangles.push({0, 1, 2});
    defer tri_mesh.free();

    TransformedShape tri_shape = {
        .translation = mesh_translation,
        .scale = mesh_scale,
        .rotation = mesh_rotation,
        .shape = &tri_mesh,
    };

    Vec3 best_normal;
    Vec3[2] best_contact;
    float max_depth = 0;

    foreach (tri : triangles) {
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

        // Update mesh vertices in-place (no allocation)
        tri_mesh.vertices[0] = tri[0];
        tri_mesh.vertices[1] = tri[1];
        tri_mesh.vertices[2] = tri[2];

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
        // Clamp depth to prevent extreme correction forces from BVH triangle collisions
        float clamped_depth = math::min(max_depth, 1.0f);
        return {
            .a = pair.a,
            .b = pair.b,
            .normal = best_normal,
            .contact = best_contact,
            .depth = clamped_depth,
        };
    }

    return NOT_FOUND~;
}
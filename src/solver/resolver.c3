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

// Task structure for parallel XPBD predict/derive
struct XpbdPhaseTask {
    PhysicsWorld* world;
    Rigidbody** bodies;      // Array of body pointers to process
    usz count;               // Number of bodies in this batch
    float time;
}

// Task structure for parallel soft body XPBD solving
struct SoftBodySolveTask {
    SoftBody* body;
    PhysicsWorld* world;
    Vec3 gravity;
    float time;
    float ground_z;
    bool ground_enabled;
    float ground_friction;
}

// Task structure for island XPBD constraint solving
struct IslandSolveTask {
    PhysicsWorld* world;
    XpbdConstraint** constraints; // Constraints for this island
    usz count;               // Number of constraints
    uint iterations;         // Solver iterations
}

enum XpbdConstraintType : char {
    CONTACT_NORMAL,
    JOINT_POSITION,
    JOINT_ANGULAR,
}

struct XpbdConstraint {
    Rigidbody* body_a;
    Rigidbody* body_b;
    Vec3 normal;          // Constraint direction
    Vec3 r_a;             // Contact arm from body_a center (local space for joints, world-offset for contacts)
    Vec3 r_b;             // Contact arm from body_b center
    Vec3 r_a_local;       // Contact arm in body_a local space (for recomputing after position changes)
    Vec3 r_b_local;       // Contact arm in body_b local space
    float c_value;        // Constraint error (position-level)
    float compliance;     // 0 = rigid, higher = softer
    float lambda;         // Accumulated Lagrange multiplier
    XpbdConstraintType type;
    // Friction
    float friction;
    Vec3 tangent1;
    Vec3 tangent2;
    float lambda_tangent1;
    float lambda_tangent2;
    float lambda_normal;
    // Restitution
    float restitution;
    float pre_solve_relative_vel_n; // Normal relative velocity before solve (for restitution)
}

struct Collision {
    usz a;
    usz b;
    Vec3 normal;
    Vec3[2] contact;
    float depth;
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
    Matrix3f inertia;

    // XPBD: previous state for velocity derivation
    Vec3 prev_position;
    QuaternionValue prev_rotation;
}

alias TriggerOverlapSet = HashMap{Pair, bool};
alias ContactOverlapSet = HashMap{Pair, bool};

struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{XpbdConstraint} constraints;
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

    // Soft body system (XPBD)
    SoftBodyList soft_bodies;
    float ground_plane_z;
    bool ground_plane_enabled;

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
    List{XpbdPhaseTask} phase_tasks;
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
    self.phase_tasks.free();
    self.active_bodies.free();
    // Free trigger system
    self.trigger_events.free();
    self.active_triggers.free();
    // Free contact event system
    self.contact_events.free();
    self.active_contacts.free();
    // Free soft bodies
    foreach (&sb : self.soft_bodies) sb.free();
    self.soft_bodies.free();
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
 Add a soft body to the physics world.
 The soft body uses XPBD position-based dynamics, separate from rigidbody solver.
*>
fn void PhysicsWorld.add_softbody(&self, SoftBody sb) {
    self.soft_bodies.push(sb);
}

<*
 Remove a soft body by ID.
*>
fn void PhysicsWorld.remove_softbody(&self, usz id) {
    for (usz i = 0; i < self.soft_bodies.len(); i++) {
        if (self.soft_bodies[i].id == id) {
            self.soft_bodies[i].free();
            self.soft_bodies.remove_at(i);
            return;
        }
    }
}

<*
 Find a soft body by ID.
*>
fn SoftBody*? PhysicsWorld.find_softbody(&self, usz id) {
    foreach (&sb : self.soft_bodies) {
        if (sb.id == id) return sb;
    }
    return NOT_FOUND~;
}

<*
 Enable ground plane collision for soft bodies.
*>
fn void PhysicsWorld.enable_ground_plane(&self, float z = 0) {
    self.ground_plane_enabled = true;
    self.ground_plane_z = z;
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

// XPBD generalized inverse mass: w = 1/m + (r×n)^T · I^{-1} · (r×n)
fn float compute_generalized_inverse_mass(Rigidbody* body, Vec3 r, Vec3 n) {
    if (body.is_static) return 0;
    float w = body.inverse_mass;
    Vec3 rn = r.cross(n);
    Vec3 inertia_term = body.get_inverse_world_inertia() * rn;
    w += inertia_term.dot(rn);
    return w;
}

// XPBD positional correction: apply delta_lambda along direction n at contact arm r
fn void apply_positional_correction(Rigidbody* body, Vec3 n, Vec3 r, float delta_lambda) {
    if (body.is_static) return;

    // Linear correction
    body.collider.translation += n * (body.inverse_mass * delta_lambda);

    // Angular correction
    Vec3 rn = r.cross(n);
    Vec3 angular_correction = body.get_inverse_world_inertia() * rn * delta_lambda;
    float angle = angular_correction.length();
    if (angle > 1e-8) {
        Vec3 axis = angular_correction / angle;
        float half = angle * 0.5f;
        float s = math::sin(half);
        QuaternionValue dq = {axis.x * s, axis.y * s, axis.z * s, math::cos(half)};
        body.collider.rotation = (dq * body.collider.rotation).normalize();
    }
}

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

// Worker function for parallel XPBD predict phase
fn int predict_worker(void* arg) @local {
    XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
    PhysicsWorld* world = task.world;
    float dt = task.time;

    for (usz i = 0; i < task.count; i++) {
        Rigidbody* body = task.bodies[i];

        // Save previous state
        body.prev_position = body.collider.translation;
        body.prev_rotation = body.collider.rotation;

        // Apply gravity to velocity
        body.linear_velocity += world.gravity * body.gravity_factor * dt;

        // Advance position using current velocity
        body.collider.translation += body.linear_velocity * dt;

        // Advance rotation using angular velocity
        Vec3 axis = body.angular_velocity * (dt * 0.5f);
        if (axis != {0,0,0}) {
            body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
        }
    }
    return 0;
}

// Worker function for parallel XPBD derive phase
fn int derive_worker(void* arg) @local {
    XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
    PhysicsWorld* world = task.world;
    float dt = task.time;
    float inv_dt = 1.0f / dt;

    for (usz i = 0; i < task.count; i++) {
        Rigidbody* body = task.bodies[i];

        // Derive linear velocity from position change
        body.linear_velocity = (body.collider.translation - body.prev_position) * inv_dt;

        // Derive angular velocity: omega = 2 * (q * q_prev_conjugate).xyz / dt
        QuaternionValue dq = body.collider.rotation * body.prev_rotation.conjugate();
        // Ensure shortest path
        if (dq.l < 0) dq = {-dq.v.x, -dq.v.y, -dq.v.z, -dq.l};
        body.angular_velocity = dq.v.xyz * (2.0f * inv_dt);

        // Apply dampening
        body.angular_velocity *= math::pow(world.angular_dampening, dt);
        body.linear_velocity *= math::pow(world.linear_dampening, dt);

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

// Worker function for island-parallel XPBD constraint solving
fn int island_solve_worker(void* arg) @local {
    IslandSolveTask* task = (IslandSolveTask*)arg;

    for (uint iter = 0; iter < task.iterations; iter++) {
        for (usz i = 0; i < task.count; i++) {
            solve_xpbd_constraint(task.constraints[i], task.world);
        }
    }
    return 0;
}

// XPBD constraint solver — position-level corrections
fn void solve_xpbd_constraint(XpbdConstraint* c, PhysicsWorld* world) @local {
    Rigidbody* a = c.body_a;
    Rigidbody* b = c.body_b;

    switch (c.type) {
        case CONTACT_NORMAL:
            // Recompute contact arms in world space from local-space arms
            Vec3 cur_r_a = a.collider.rotation * c.r_a_local;
            Vec3 cur_r_b = b.collider.rotation * c.r_b_local;

            // Current contact points in world space
            Vec3 pa = a.collider.translation + cur_r_a;
            Vec3 pb = b.collider.translation + cur_r_b;

            // Separation along normal: positive = separated, negative = penetrating
            float separation = (pb - pa).dot(c.normal);
            float penetration = c.c_value - separation;

            if (penetration <= 0) return; // Not penetrating, skip

            // Update r_a/r_b for positional correction
            c.r_a = cur_r_a;
            c.r_b = cur_r_b;

            float w_a = compute_generalized_inverse_mass(a, c.r_a, c.normal);
            float w_b = compute_generalized_inverse_mass(b, c.r_b, c.normal);
            float w_sum = w_a + w_b;
            if (w_sum < 1e-10) return;

            float alpha = c.compliance;
            // XPBD: C = -penetration (non-penetration constraint: C >= 0)
            // delta_lambda = -(-pen + alpha * lambda) / (w + alpha) = (pen - alpha*lambda) / (w+alpha)
            float delta_lambda = (penetration - alpha * c.lambda) / (w_sum + alpha);

            // Clamp: lambda must be >= 0 (contact can only push, not pull)
            float new_lambda = c.lambda + delta_lambda;
            if (new_lambda < 0) {
                delta_lambda = -c.lambda;
                new_lambda = 0;
            }
            c.lambda = new_lambda;
            c.lambda_normal = new_lambda;

            // Apply positional corrections: push a along -normal, push b along +normal
            apply_positional_correction(a, -c.normal, c.r_a, delta_lambda);
            apply_positional_correction(b, c.normal, c.r_b, delta_lambda);

            // Friction: tangential position correction
            if (c.friction > 0 && math::abs(c.lambda_normal) > 1e-8) {
                // Compute tangential displacement
                Vec3 rel_disp = (a.collider.translation - a.prev_position) - (b.collider.translation - b.prev_position);
                float disp_t1 = rel_disp.dot(c.tangent1);
                float disp_t2 = rel_disp.dot(c.tangent2);

                float max_friction = c.friction * math::abs(c.lambda_normal);

                // Tangent 1 friction
                if (math::abs(disp_t1) > 1e-8) {
                    float w_t1_a = compute_generalized_inverse_mass(a, c.r_a, c.tangent1);
                    float w_t1_b = compute_generalized_inverse_mass(b, c.r_b, c.tangent1);
                    float w_t1 = w_t1_a + w_t1_b;
                    if (w_t1 > 1e-10) {
                        float dl_t1 = -disp_t1 / w_t1;
                        float new_lt1 = c.lambda_tangent1 + dl_t1;
                        new_lt1 = math::clamp(new_lt1, -max_friction, max_friction);
                        dl_t1 = new_lt1 - c.lambda_tangent1;
                        c.lambda_tangent1 = new_lt1;
                        apply_positional_correction(a, c.tangent1, c.r_a, dl_t1);
                        apply_positional_correction(b, -c.tangent1, c.r_b, dl_t1);
                    }
                }

                // Tangent 2 friction
                if (math::abs(disp_t2) > 1e-8) {
                    float w_t2_a = compute_generalized_inverse_mass(a, c.r_a, c.tangent2);
                    float w_t2_b = compute_generalized_inverse_mass(b, c.r_b, c.tangent2);
                    float w_t2 = w_t2_a + w_t2_b;
                    if (w_t2 > 1e-10) {
                        float dl_t2 = -disp_t2 / w_t2;
                        float new_lt2 = c.lambda_tangent2 + dl_t2;
                        new_lt2 = math::clamp(new_lt2, -max_friction, max_friction);
                        dl_t2 = new_lt2 - c.lambda_tangent2;
                        c.lambda_tangent2 = new_lt2;
                        apply_positional_correction(a, c.tangent2, c.r_a, dl_t2);
                        apply_positional_correction(b, -c.tangent2, c.r_b, dl_t2);
                    }
                }
            }

        case JOINT_POSITION:
            // Recompute anchor arms in world space (r_a/r_b store local-space anchors for joints)
            Vec3 j_r_a = a.collider.rotation * c.r_a;
            Vec3 j_r_b = b.collider.rotation * c.r_b;

            Vec3 world_a = a.collider.translation + j_r_a;
            Vec3 world_b = b.collider.translation + j_r_b;
            float pos_error = (world_b - world_a).dot(c.normal);

            float jw_a = compute_generalized_inverse_mass(a, j_r_a, c.normal);
            float jw_b = compute_generalized_inverse_mass(b, j_r_b, c.normal);
            float jw_sum = jw_a + jw_b;
            if (jw_sum < 1e-10) return;

            float j_alpha = c.compliance;
            float j_dl = -(pos_error + j_alpha * c.lambda) / (jw_sum + j_alpha);
            c.lambda += j_dl;

            apply_positional_correction(a, -c.normal, j_r_a, j_dl);
            apply_positional_correction(b, c.normal, j_r_b, j_dl);

        case JOINT_ANGULAR:
            // Angular constraint: correct orientation difference along axis
            Vec3 axis = c.normal; // axis in world space

            // Angular generalized inverse mass (no lever arm)
            float aw_a = 0;
            float aw_b = 0;
            if (!a.is_static) {
                Vec3 a_accel = a.get_inverse_world_inertia() * axis;
                aw_a = a_accel.dot(axis);
            }
            if (!b.is_static) {
                Vec3 b_accel = b.get_inverse_world_inertia() * axis;
                aw_b = b_accel.dot(axis);
            }
            float aw_sum = aw_a + aw_b;
            if (aw_sum < 1e-10) return;

            // Recompute angular error
            float ang_error = c.c_value;

            float a_alpha = c.compliance;
            float a_dl = -(ang_error + a_alpha * c.lambda) / (aw_sum + a_alpha);
            c.lambda += a_dl;

            // Apply angular corrections via quaternion
            if (!a.is_static) {
                Vec3 ang_corr = a.get_inverse_world_inertia() * axis * (-a_dl);
                float ang_mag = ang_corr.length();
                if (ang_mag > 1e-8) {
                    Vec3 ax = ang_corr / ang_mag;
                    float h = ang_mag * 0.5f;
                    float s = math::sin(h);
                    QuaternionValue dq = {ax.x * s, ax.y * s, ax.z * s, math::cos(h)};
                    a.collider.rotation = (dq * a.collider.rotation).normalize();
                }
            }
            if (!b.is_static) {
                Vec3 ang_corr = b.get_inverse_world_inertia() * axis * a_dl;
                float ang_mag = ang_corr.length();
                if (ang_mag > 1e-8) {
                    Vec3 ax = ang_corr / ang_mag;
                    float h = ang_mag * 0.5f;
                    float s = math::sin(h);
                    QuaternionValue dq = {ax.x * s, ax.y * s, ax.z * s, math::cos(h)};
                    b.collider.rotation = (dq * b.collider.rotation).normalize();
                }
            }
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

    // === PHASE 0: XPBD Predict — save prev state, apply gravity, advance positions ===
    self.xpbd_predict(time);

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

    // === PHASE 3: Create XPBD constraints (from collisions + joints) ===
    foreach (&collision: self.collisions) {
        collision.create_xpbd_constraint(self, time);
    }

    foreach (&joint: self.joints) {
        joint.create_xpbd_constraint(self, time);
    }

    // === PHASE 4: Project XPBD constraints — island-parallel ===
    self.xpbd_solve_constraints(step_count);

    self.collisions.clear();

    // === PHASE 5.5: XPBD soft body solve ===
    self.solve_soft_bodies(time);

    // === PHASE 6: Derive velocities, restitution, update spatial map ===
    self.xpbd_derive_and_update(time);

    // === PHASE 7: Sleep management (sequential) ===
    self.update_sleep_state();
}

// XPBD predict phase: save prev state, apply gravity, advance positions
fn void PhysicsWorld.xpbd_predict(&self, float dt) @local {
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

    usz batch_size = (num_active + NUM_WORKER_THREADS - 1) / NUM_WORKER_THREADS;

    if (num_active >= NUM_WORKER_THREADS * 2) {
        usz num_tasks = (num_active + batch_size - 1) / batch_size;
        self.phase_tasks.clear();

        Rigidbody** all_bodies_ptr = self.active_bodies.array_view().ptr;

        for (usz i = 0; i < num_tasks; i++) {
            usz start = i * batch_size;
            usz count = math::min(batch_size, num_active - start);
            self.phase_tasks.push({
                .world = self,
                .bodies = all_bodies_ptr + start,
                .count = count,
                .time = dt,
            });
        }

        for (usz i = 0; i < num_tasks; i++) {
            self.thread_pool.push(&predict_worker, &self.phase_tasks[i])!!;
        }
        self.thread_pool.join()!!;
    } else {
        foreach (body : self.active_bodies) {
            body.prev_position = body.collider.translation;
            body.prev_rotation = body.collider.rotation;
            body.linear_velocity += self.gravity * body.gravity_factor * dt;
            body.collider.translation += body.linear_velocity * dt;

            Vec3 axis = body.angular_velocity * (dt * 0.5f);
            if (axis != {0,0,0}) {
                body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
            }
        }
    }

    // Sequential: update spatial map AABBs after predict (must happen before broad phase)
    foreach (body : self.active_bodies) {
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
        body.transformed_aabb = new_aabb;
    }
}

// XPBD derive phase: compute velocities from position deltas, restitution, update spatial map
fn void PhysicsWorld.xpbd_derive_and_update(&self, float dt) @local {
    usz num_active = self.active_bodies.len();
    if (num_active == 0) return;

    float inv_dt = 1.0f / dt;

    // Derive velocities (parallelizable)
    usz batch_size = (num_active + NUM_WORKER_THREADS - 1) / NUM_WORKER_THREADS;

    if (num_active >= NUM_WORKER_THREADS * 2) {
        usz num_tasks = (num_active + batch_size - 1) / batch_size;
        self.phase_tasks.clear();

        Rigidbody** all_bodies_ptr = self.active_bodies.array_view().ptr;

        for (usz i = 0; i < num_tasks; i++) {
            usz start = i * batch_size;
            usz count = math::min(batch_size, num_active - start);
            self.phase_tasks.push({
                .world = self,
                .bodies = all_bodies_ptr + start,
                .count = count,
                .time = dt,
            });
        }

        for (usz i = 0; i < num_tasks; i++) {
            self.thread_pool.push(&derive_worker, &self.phase_tasks[i])!!;
        }
        self.thread_pool.join()!!;
    } else {
        foreach (body : self.active_bodies) {
            body.linear_velocity = (body.collider.translation - body.prev_position) * inv_dt;

            QuaternionValue dq = body.collider.rotation * body.prev_rotation.conjugate();
            if (dq.l < 0) dq = {-dq.v.x, -dq.v.y, -dq.v.z, -dq.l};
            body.angular_velocity = dq.v.xyz * (2.0f * inv_dt);

            body.angular_velocity *= math::pow(self.angular_dampening, dt);
            body.linear_velocity *= math::pow(self.linear_dampening, dt);

            const float VELOCITY_CLAMP_SQ = 0.01f;
            if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
                body.linear_velocity = {0, 0, 0};
            }
            if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
                body.angular_velocity = {0, 0, 0};
            }
        }
    }

    // Post-derive: velocity correction at contacts
    // For zero restitution: target_vn = 0 (inelastic, no bounce)
    // For restitution > 0: target_vn = -e * vn_pre (bounce proportional to impact speed)
    foreach (&c : self.constraints) {
        if (c.type != CONTACT_NORMAL) continue;
        if (math::abs(c.lambda_normal) < 1e-8) continue;

        Rigidbody* a = c.body_a;
        Rigidbody* b = c.body_b;

        // Current relative velocity at contact (after derive)
        Vec3 va = a.relative_velocity(c.r_a);
        Vec3 vb = b.relative_velocity(c.r_b);
        float vn = (va - vb).dot(c.normal);

        // Target normal velocity
        float target_vn = 0;
        float vn_pre = c.pre_solve_relative_vel_n;
        const float BOUNCE_THRESHOLD = 1.0f;
        if (c.restitution > 0.01f && math::abs(vn_pre) > BOUNCE_THRESHOLD) {
            target_vn = -c.restitution * vn_pre;
        }

        float dv = target_vn - vn;

        // Only apply if it would reduce separating velocity or add bounce
        // (don't add attraction at contacts)
        if (math::abs(dv) > 1e-6) {
            float w_a = compute_generalized_inverse_mass(a, c.r_a, c.normal);
            float w_b = compute_generalized_inverse_mass(b, c.r_b, c.normal);
            float w_sum = w_a + w_b;
            if (w_sum > 1e-10) {
                float impulse_mag = dv / w_sum;
                if (!a.is_static) {
                    a.apply_impulse(c.normal * impulse_mag, c.r_a);
                }
                if (!b.is_static) {
                    b.apply_impulse(-c.normal * impulse_mag, c.r_b);
                }
            }
        }
    }

    self.constraints.clear();

    // Sequential: update spatial map and sleep timers
    foreach (body : self.active_bodies) {
        bool rotating = body.is_rotating();
        bool moving = body.is_moving();

        if (rotating || moving) {
            Aabb3 new_aabb = body.get_transformed_aabb();
            self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
            body.transformed_aabb = new_aabb;
        }

        if (body.can_sleep == true && moving == false && rotating == false) {
            body.sleep_timer += dt;
        } else {
            body.sleep_timer = 0;
        }
    }
}

// Island-parallel XPBD constraint solving
fn void PhysicsWorld.xpbd_solve_constraints(&self, uint step_count) @local {
    if (self.constraints.len() == 0) return;

    // Group constraints by island
    HashMap{usz, List{XpbdConstraint*}} island_constraints;
    defer {
        island_constraints.@each(; usz id, List{XpbdConstraint*} list) {
            list.free();
        };
        island_constraints.free();
    }

    for (usz i = 0; i < self.constraints.len(); i++) {
        XpbdConstraint* constraint = &self.constraints[i];
        usz island_id = constraint.body_a.island_id;

        if (!island_constraints.has_key(island_id)) {
            island_constraints.set(island_id, {});
        }
        island_constraints.get_ref(island_id)!!.push(constraint);
    }

    usz num_islands = island_constraints.len();

    if (num_islands <= 1) {
        for (usz i; i < step_count; i++) {
            for (usz j = 0; j < self.constraints.len(); j++) {
                solve_xpbd_constraint(&self.constraints[j], self);
            }
        }
        return;
    }

    self.island_tasks.clear();

    island_constraints.@each(; usz id, List{XpbdConstraint*} list) {
        XpbdConstraint*[] arr = list.array_view();

        self.island_tasks.push({
            .world = self,
            .constraints = arr.ptr,
            .count = arr.len,
            .iterations = step_count,
        });
    };

    for (usz i = 0; i < num_islands; i++) {
        self.thread_pool.push(&island_solve_worker, &self.island_tasks[i])!!;
    }

    self.thread_pool.join()!!;
}

// Worker function for parallel soft body XPBD solving
fn int softbody_solve_worker(void* arg) @local {
    SoftBodySolveTask* task = (SoftBodySolveTask*)arg;
    SoftBody* sb = task.body;

    // Generate ground plane collisions
    if (task.ground_enabled) {
        sb.generate_ground_collisions(task.ground_z, task.ground_friction);
    }

    // Run XPBD solver (no rigidbody collisions in parallel — not thread-safe)
    sb.xpbd_solve(task.gravity, task.time, task.world);

    return 0;
}

// XPBD soft body solving phase
fn void PhysicsWorld.solve_soft_bodies(&self, float time) @local {
    if (self.soft_bodies.len() == 0) return;

    // Sequential: generate rigidbody collision constraints (needs shared body access)
    foreach (&sb : self.soft_bodies) {
        if (sb.sleeping) continue;

        RigidBodyIterator rb_iter = self.bodies.value_iter();
        while (rb_iter.next()) {
            Rigidbody* body = rb_iter.get()!!;
            sb.generate_rigidbody_collisions(body);
        }
    }

    // Count active soft bodies
    usz active_count = 0;
    foreach (&sb : self.soft_bodies) {
        if (!sb.sleeping) active_count++;
    }

    if (active_count == 0) return;

    // Parallel XPBD solving if multiple active soft bodies
    if (active_count >= 2) {
        List{SoftBodySolveTask} sb_tasks;
        defer sb_tasks.free();

        foreach (&sb : self.soft_bodies) {
            if (sb.sleeping) continue;
            sb_tasks.push({
                .body = sb,
                .world = self,
                .gravity = self.gravity,
                .time = time,
                .ground_z = self.ground_plane_z,
                .ground_enabled = self.ground_plane_enabled,
                .ground_friction = sb.material.static_friction,
            });
        }

        for (usz i = 0; i < sb_tasks.len(); i++) {
            self.thread_pool.push(&softbody_solve_worker, &sb_tasks[i])!!;
        }
        self.thread_pool.join()!!;
    } else {
        // Single soft body — solve inline
        foreach (&sb : self.soft_bodies) {
            if (sb.sleeping) continue;

            if (self.ground_plane_enabled) {
                sb.generate_ground_collisions(self.ground_plane_z,
                    sb.material.static_friction);
            }

            sb.xpbd_solve(self.gravity, time, self);
        }
    }

    // Sequential: manage soft body sleep
    foreach (&sb : self.soft_bodies) {
        if (sb.sleeping) continue;

        // Sleep management
        if (sb.can_sleep) {
            if (!sb.is_moving()) {
                sb.sleep_timer += time;
                if (sb.sleep_timer >= self.sleep_timer) {
                    sb.sleeping = true;
                }
            } else {
                sb.sleep_timer = 0;
            }
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

fn float combine_values(float a, float b, PhysicsCombine mode) @local {
    switch (mode) {
        case AVERAGE: return (a + b) * 0.5f;
        case MINIMUM: return math::min(a, b);
        case MAXIMUM: return math::max(a, b);
        case MULTIPLY:	return a * b;
		default: return a * b;
    }
}

fn void Collision.create_xpbd_constraint(&collision, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(collision.a)!!;
    Rigidbody* b = world.find_body(collision.b)!!;
    Vec3 normal = collision.normal.normalize();

    // Combine material properties
    PhysicsCombine friction_mode = (PhysicsCombine)math::max((int)a.material.friction_combine, (int)b.material.friction_combine);
    PhysicsCombine restitution_mode = (PhysicsCombine)math::max((int)a.material.restitution_combine, (int)b.material.restitution_combine);

    float restitution = combine_values(a.material.restitution, b.material.restitution, restitution_mode);
    float static_friction = combine_values(a.material.static_friction, b.material.static_friction, friction_mode);

    // Contact arms (world-space offset from body center)
    Vec3 rA = collision.contact[0] - a.collider.translation;
    Vec3 rB = collision.contact[1] - b.collider.translation;
    // Store in local space for recomputing after position changes
    Vec3 rA_local = a.collider.rotation.conjugate() * rA;
    Vec3 rB_local = b.collider.rotation.conjugate() * rB;

    // Compute tangent basis
    Vec3 helper = (math::abs(normal.y) > 0.99)
        ? {1.0f, 0.0f, 0.0f}
        : {0.0f, 1.0f, 0.0f};
    Vec3 tangent_1 = normal.cross(helper).normalize();
    Vec3 tangent_2 = normal.cross(tangent_1);

    // Pre-solve relative velocity for restitution (before XPBD changes positions)
    Vec3 va = a.relative_velocity(rA);
    Vec3 vb = b.relative_velocity(rB);
    float rel_vel_n = (va - vb).dot(normal);

    // Compliance = 0 for rigid contacts (perfectly stiff)
    float alpha = 0;

    XpbdConstraint constraint = {
        .body_a = a,
        .body_b = b,
        .normal = normal,
        .r_a = rA,
        .r_b = rB,
        .r_a_local = rA_local,
        .r_b_local = rB_local,
        .c_value = collision.depth,
        .compliance = alpha,
        .lambda = 0,
        .type = CONTACT_NORMAL,
        .friction = static_friction,
        .tangent1 = tangent_1,
        .tangent2 = tangent_2,
        .lambda_tangent1 = 0,
        .lambda_tangent2 = 0,
        .lambda_normal = 0,
        .restitution = restitution,
        .pre_solve_relative_vel_n = rel_vel_n,
    };

    world.constraints.push(constraint);
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
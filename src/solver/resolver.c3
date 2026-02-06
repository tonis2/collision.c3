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

// AVBD solver constants (module-level so visible across files)
const MAX_FORCE_ROWS = 3;
const PENALTY_MIN = 20000.0f;
const PENALTY_MAX = 1000000000.0f;
const PENETRATION_SLOP = 0.005f;
const AVBD_ALPHA = 0.95f;
const AVBD_BETA = 100000.0f;
const AVBD_GAMMA = 0.99f;
const MAX_ANGULAR_SPEED = 80.0f;

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

struct AvbdForce {
    Rigidbody* body_a;
    Rigidbody* body_b;
    usz num_rows;

    // Per-row state
    float[MAX_FORCE_ROWS] c_value;
    float[MAX_FORCE_ROWS] fmin;
    float[MAX_FORCE_ROWS] fmax;
    float[MAX_FORCE_ROWS] lambda;
    float[MAX_FORCE_ROWS] penalty;
    float[MAX_FORCE_ROWS] stiffness;

    // Contact-specific storage
    Vec3 rA_local;
    Vec3 rB_local;
    Vec3 normal;
    Vec3 tangent1;
    Vec3 tangent2;
    float[2] c0_tangent;

    // Joint-specific storage
    Vec3 anchor_a;
    Vec3 anchor_b;
    Vec3[MAX_FORCE_ROWS] basis;  // Axis per row

    float friction;
    bool is_contact;
    bool is_angular;  // Angular joint force (no linear Jacobian)
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
    Matrix3f inertia;           // Inverse inertia (for compatibility)

    // AVBD state
    Matrix3f inertia_local;     // Non-inverse local inertia tensor
    Vec3 initial_position;
    QuaternionValue initial_orientation;
    Vec3 inertial_position;
    QuaternionValue inertial_orientation;
    Vec3 prev_linear_velocity;
    Vec3 prev_angular_velocity;
}

alias TriggerOverlapSet = HashMap{Pair, bool};
alias ContactOverlapSet = HashMap{Pair, bool};

struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{AvbdForce} forces;
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
    ThreadPool{NUM_WORKER_THREADS} thread_pool;
    Mutex collision_mutex;
    bool thread_pool_initialized;

    // Reusable task lists (avoid per-frame allocations)
    List{CollisionTask} collision_tasks;
    List{Collision} collision_results;
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
    self.forces.free();
    self.filters.free();
    self.joints.free();
    self.island_parent.free();
    // Free reusable task lists
    self.collision_tasks.free();
    self.collision_results.free();
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
    // Store both non-inverse and inverse inertia for AVBD
    if (!body.is_static && &body.collider.shape.inertia_tensor) {
        body.inertia_local = body.aabb.inertia_tensor(body.mass);
        body.inertia = body.inertia_local.inverse()!!;
    }

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

fn Matrix3f Rigidbody.get_world_inertia(self) {
    if (self.is_static) return MATRIX3F_IDENTITY;
    return self.collider.world_inertia_from_local(self.inertia_local);
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

// Clamp angular velocity magnitude
fn void clamp_angular_velocity(Rigidbody* body) @local {
    float sq_mag = body.angular_velocity.sq_magnitude();
    if (sq_mag > MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED) {
        body.angular_velocity = body.angular_velocity * (MAX_ANGULAR_SPEED / math::sqrt(sq_mag));
    }
}

// Build tangent frame from a normal vector
fn void build_tangent_frame(Vec3 normal, Vec3* tangent1, Vec3* tangent2) @local {
    Vec3 helper = (math::abs(normal.x) > 0.9f)
        ? (Vec3){0.0f, 1.0f, 0.0f}
        : (Vec3){1.0f, 0.0f, 0.0f};
    *tangent1 = normal.cross(helper).normalize();
    *tangent2 = normal.cross(*tangent1).normalize();
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

// === AVBD: Create contact force from collision ===
fn void Collision.create_force(&collision, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(collision.a)!!;
    Rigidbody* b = world.find_body(collision.b)!!;

    // CRITICAL: Negate normal! C3 collision normal points A→B, AVBD needs B→A
    Vec3 normal = -collision.normal.normalize();

    // Compute body-local contact offsets
    Vec3 rA_local = a.collider.rotation.conjugate() * (collision.contact[0] - a.collider.translation);
    Vec3 rB_local = b.collider.rotation.conjugate() * (collision.contact[1] - b.collider.translation);

    // Build tangent frame
    Vec3 tangent1, tangent2;
    build_tangent_frame(normal, &tangent1, &tangent2);

    // Precompute C0 for Taylor series stabilization
    Vec3 delta = collision.contact[0] - collision.contact[1];
    float c0_t1 = delta.dot(tangent1);
    float c0_t2 = delta.dot(tangent2);

    // Combine friction
    PhysicsCombine friction_mode = (PhysicsCombine)math::max((int)a.material.friction_combine, (int)b.material.friction_combine);
    float static_friction = combine_values(a.material.static_friction, b.material.static_friction, friction_mode);

    AvbdForce force = {
        .body_a = a, .body_b = b, .num_rows = 3,
        .rA_local = rA_local, .rB_local = rB_local,
        .normal = normal,
        .tangent1 = tangent1,
        .tangent2 = tangent2,
        .c0_tangent = { c0_t1, c0_t2 },
        .friction = static_friction,
        .is_contact = true,
    };

    // Initialize per-row state
    for (usz i = 0; i < 3; i++) {
        force.lambda[i] = 0;
        force.penalty[i] = PENALTY_MIN;
        force.stiffness[i] = float.max;
    }
    // Normal: repulsion only (negative force = push apart)
    force.fmin[0] = -float.max;  force.fmax[0] = 0;
    // Friction: updated dynamically during solve
    force.fmin[1] = 0;  force.fmax[1] = 0;
    force.fmin[2] = 0;  force.fmax[2] = 0;

    // Initial constraint violations
    force.c_value[0] = delta.dot(normal) - PENETRATION_SLOP;
    force.c_value[1] = c0_t1;
    force.c_value[2] = c0_t2;

    world.forces.push(force);
}

// === AVBD: Recompute contact violations each iteration ===
fn void AvbdForce.update_contact_violations(&self, float alpha) @local {
    Vec3 world_rA = self.body_a.collider.rotation * self.rA_local;
    Vec3 world_rB = self.body_b.collider.rotation * self.rB_local;
    Vec3 pA = self.body_a.collider.translation + world_rA;
    Vec3 pB = self.body_b.collider.translation + world_rB;
    Vec3 delta = pA - pB;

    float separation = delta.dot(self.normal);
    float slip1 = delta.dot(self.tangent1);
    float slip2 = delta.dot(self.tangent2);

    // Taylor series stabilization bias
    float bias_t1 = (1.0f - alpha) * self.c0_tangent[0];
    float bias_t2 = (1.0f - alpha) * self.c0_tangent[1];

    self.c_value[0] = separation - PENETRATION_SLOP;
    self.c_value[1] = bias_t1 + slip1;
    self.c_value[2] = bias_t2 + slip2;

    // Update friction limits from normal force
    float friction_limit = self.friction * math::abs(self.lambda[0]);
    self.fmin[1] = -friction_limit;  self.fmax[1] = friction_limit;
    self.fmin[2] = -friction_limit;  self.fmax[2] = friction_limit;
    self.fmin[0] = -float.max;  self.fmax[0] = 0;
}

// === AVBD: Compute Jacobians for a body and force row ===
struct JacobianPair {
    Vec3 jl;
    Vec3 ja;
}

fn JacobianPair AvbdForce.compute_jacobian(&self, Rigidbody* body, usz row) @local {
    float sign = (body == self.body_a) ? 1.0f : -1.0f;

    if (self.is_contact) {
        Vec3 r_local = (body == self.body_a) ? self.rA_local : self.rB_local;
        Vec3 r_world = body.collider.rotation * r_local;

        Vec3 basis_vec;
        switch (row) {
            case 0: basis_vec = self.normal;
            case 1: basis_vec = self.tangent1;
            case 2: basis_vec = self.tangent2;
            default: basis_vec = self.normal;
        }

        return {
            .jl = basis_vec * sign,
            .ja = r_world.cross(basis_vec) * sign,
        };
    } else if (self.is_angular) {
        // Angular joint: no linear Jacobian
        return {
            .jl = {0, 0, 0},
            .ja = self.basis[row] * sign,
        };
    } else {
        // Linear joint
        Vec3 r_local = (body == self.body_a) ? self.anchor_a : self.anchor_b;
        Vec3 r_world = body.collider.rotation * r_local;

        return {
            .jl = self.basis[row] * sign,
            .ja = r_world.cross(self.basis[row]) * sign,
        };
    }
}

// === AVBD: Recompute joint violations each iteration ===
fn void AvbdForce.update_joint_violations(&self) @local {
    if (self.is_angular) {
        // Angular error
        QuaternionValue qa = self.body_a.collider.rotation;
        QuaternionValue qb = self.body_b.collider.rotation;
        QuaternionValue delta_q = qa.conjugate() * qb;
        float error_dir = delta_q.l >= 0 ? 1.0f : -1.0f;
        Vec3 error_local = delta_q.v.xyz * 2.0f * error_dir;
        Vec3 world_error = qa * error_local;

        for (usz i = 0; i < self.num_rows; i++) {
            self.c_value[i] = world_error.dot(self.basis[i]);
        }
    } else {
        // Linear position error
        Vec3 world_a = self.body_a.collider.translation + (self.body_a.collider.rotation * self.anchor_a);
        Vec3 world_b = self.body_b.collider.translation + (self.body_b.collider.rotation * self.anchor_b);
        Vec3 diff = world_a - world_b;

        for (usz i = 0; i < self.num_rows; i++) {
            self.c_value[i] = diff.dot(self.basis[i]);
        }
    }
}


// 6-vector and 6x6 matrix for block solve
struct Vec6 @local {
    Vec3 l;
    Vec3 a;
}

struct Mat66 @local {
    Matrix3f ll;
    Matrix3f la;
    Matrix3f al;
    Matrix3f aa;
}

// Solve 6x6 system via Schur complement
fn Vec6 solve6x6(Mat66 m, Vec6 b) @local {
    // Solve m.ll * X = m.la (3 column solves)
    Vec3 col0 = mat3_solve(m.ll, {m.la.m00, m.la.m10, m.la.m20});
    Vec3 col1 = mat3_solve(m.ll, {m.la.m01, m.la.m11, m.la.m21});
    Vec3 col2 = mat3_solve(m.ll, {m.la.m02, m.la.m12, m.la.m22});

    // AinvB = ll^{-1} * la as a Matrix3f
    Matrix3f ainv_b = {
        col0.x, col1.x, col2.x,
        col0.y, col1.y, col2.y,
        col0.z, col1.z, col2.z,
    };

    Vec3 x0 = mat3_solve(m.ll, b.l);

    // Schur complement: S = aa - al * ll^{-1} * la
    Matrix3f schur = m.aa - m.al * ainv_b;

    // Modified RHS: b.a - al * x0
    Vec3 rhs_s = b.a - m.al * x0;

    Vec3 y = mat3_solve(schur, rhs_s);
    Vec3 x = x0 - ainv_b * y;

    return { .l = x, .a = y };
}


<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 25) {
    // Lazy init thread pool
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
    TriggerOverlapSet current_triggers;
    defer candidate_pairs.free();
    defer current_triggers.free();

    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.find_body(pair.a)!!;
        Rigidbody* body_b = self.find_body(pair.b)!!;

        bool has_filter = self.filters.has_key({body_a.id, body_b.id}) == true;

        if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb)) {
            if (body_a.is_trigger || body_b.is_trigger) {
                Pair trigger_pair = body_a.is_trigger ? pair : (Pair){pair.b, pair.a};
                current_triggers.set(trigger_pair, true);

                if (!self.active_triggers.has_key(trigger_pair)) {
                    self.trigger_events.push({
                        .trigger_id = trigger_pair.a,
                        .other_id = trigger_pair.b,
                        .type = TriggerEventType.ENTER
                    });
                }
            } else {
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
    current_triggers = {};

    // === PHASE 2: Narrow-phase collision testing ===
    usz num_pairs = candidate_pairs.len();

    if (num_pairs > 0) {
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

    foreach (&collision: self.collisions) {
        Pair contact_pair = { collision.a, collision.b };
        current_contacts.set(contact_pair, true);

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

    self.active_contacts.free();
    self.active_contacts = current_contacts;
    current_contacts = {};

    // === PHASE 3: Create AVBD forces ===
    foreach (&collision: self.collisions) {
        collision.create_force(self, time);
    }

    foreach (&joint: self.joints) {
        joint.create_force(self, time);
    }

    // === PHASE 4: Predict body states ===
    self.predict_bodies(time);

    // === PHASE 5: AVBD solve ===
    self.avbd_solve(time, step_count);

    // === PHASE 6: Derive velocities + spatial map update ===
    self.derive_velocities(time);

    self.collisions.clear();
    self.forces.clear();

    // === PHASE 7: Sleep management ===
    self.update_sleep_state();
}

// === PHASE 4: Predict body states ===
fn void PhysicsWorld.predict_bodies(&self, float dt) @local {
    self.active_bodies.clear();

    RigidBodyIterator iter = self.bodies.value_iter();
    while (iter.next()) {
        Rigidbody* body = iter.get()!!;
        if (!body.is_active()) continue;

        self.active_bodies.push(body);

        // Save previous velocities for accelWeight
        body.prev_linear_velocity = body.linear_velocity;
        body.prev_angular_velocity = body.angular_velocity;
        clamp_angular_velocity(body);

        // Save initial state for velocity derivation
        body.initial_position = body.collider.translation;
        body.initial_orientation = body.collider.rotation;

        float dt2 = dt * dt;
        Vec3 grav = self.gravity * body.gravity_factor;

        // Inertial target (full gravity)
        body.inertial_position = body.collider.translation + body.linear_velocity * dt + grav * dt2;
        QuaternionValue omega = {
            body.angular_velocity.x * dt * 0.5f,
            body.angular_velocity.y * dt * 0.5f,
            body.angular_velocity.z * dt * 0.5f,
            0.0f
        };
        body.inertial_orientation = (body.collider.rotation + omega * body.collider.rotation).normalize();

        // accelWeight: prevents gravity double-counting for resting bodies
        float gravity_len = self.gravity.length();
        float accel_weight = 0.0f;
        if (gravity_len > 1e-5f) {
            Vec3 accel = (body.linear_velocity - body.prev_linear_velocity) / dt;
            float projected = accel.dot(self.gravity / gravity_len);
            accel_weight = math::clamp(projected / gravity_len, 0.0f, 1.0f);
        }

        // Predicted position uses accelWeight-scaled gravity
        body.collider.translation += body.linear_velocity * dt + grav * (accel_weight * dt2);
        body.collider.rotation = body.inertial_orientation;
    }
}

// === PHASE 5: AVBD block solve ===
fn void PhysicsWorld.avbd_solve(&self, float dt, uint iterations) @local {
    if (self.active_bodies.len() == 0 && self.forces.len() == 0) return;

    float inv_dt2 = 1.0f / (dt * dt);

    for (uint it = 0; it < iterations; it++) {
        float current_alpha = AVBD_ALPHA;

        // Recompute all constraint violations
        for (usz fi = 0; fi < self.forces.len(); fi++) {
            AvbdForce* force = &self.forces[fi];
            if (force.is_contact) {
                force.update_contact_violations(current_alpha);
            } else {
                force.update_joint_violations();
            }
        }

        // Per-body block solve
        foreach (body : self.active_bodies) {
            Matrix3f mass_mat = mat3_diagonal({body.mass, body.mass, body.mass});
            Matrix3f inertia_world = body.get_world_inertia();
            Matrix3f inv_inertia_world = body.get_inverse_world_inertia();

            // Initialize LHS and RHS with inertial terms
            Mat66 lhs = {
                .ll = mass_mat * inv_dt2,
                .aa = inertia_world * inv_dt2,
            };

            // RHS: inertial force pulling toward free-flight prediction
            Vec3 rhs_l = mass_mat * ((body.collider.translation - body.inertial_position) * inv_dt2);

            // Angular RHS: rotation error from inertial prediction
            QuaternionValue q_err = body.collider.rotation * body.inertial_orientation.conjugate();
            Vec3 rot_err = q_err.v.xyz * 2.0f;
            if (q_err.l < 0.0f) rot_err = -rot_err;
            Vec3 rhs_a = inertia_world * (rot_err * inv_dt2);

            // Accumulate force contributions
            for (usz fi = 0; fi < self.forces.len(); fi++) {
                AvbdForce* force = &self.forces[fi];

                // Skip forces not touching this body
                if (force.body_a != body && force.body_b != body) continue;

                for (usz row = 0; row < force.num_rows; row++) {
                    JacobianPair jp = force.compute_jacobian(body, row);

                    // Compute desired force
                    float lambda_warm = (force.stiffness[row] == float.max) ? force.lambda[row] : 0.0f;
                    float desired = force.penalty[row] * force.c_value[row] + lambda_warm;
                    float f = math::clamp(desired, force.fmin[row], force.fmax[row]);

                    // Add to RHS
                    rhs_l += jp.jl * f;
                    rhs_a += jp.ja * f;

                    // Add penalty-scaled outer products to LHS
                    float pen = force.penalty[row];
                    if (pen > 0.0f) {
                        lhs.ll += mat3_outer(jp.jl, jp.jl) * pen;
                        lhs.la += mat3_outer(jp.jl, jp.ja) * pen;
                        lhs.al += mat3_outer(jp.ja, jp.jl) * pen;
                        lhs.aa += mat3_outer(jp.ja, jp.ja) * pen;

                        // Gyroscopic stabilization (contacts only)
                        if (force.is_contact) {
                            Vec3 gyro = (jp.ja.cross(inv_inertia_world * jp.ja)).abs() * math::abs(f);
                            lhs.aa += mat3_diagonal(gyro);
                        }
                    }
                }
            }

            // Solve 6x6 system
            Vec6 dx = solve6x6(lhs, { .l = rhs_l, .a = rhs_a });

            // Apply correction
            body.collider.translation -= dx.l;
            QuaternionValue dq = {dx.a.x * 0.5f, dx.a.y * 0.5f, dx.a.z * 0.5f, 0.0f};
            body.collider.rotation = (body.collider.rotation - dq * body.collider.rotation).normalize();
        }

        // Dual update: lambda and penalty adaptation
        for (usz fi = 0; fi < self.forces.len(); fi++) {
            AvbdForce* force = &self.forces[fi];

            // Recompute violations after position update
            if (force.is_contact) {
                force.update_contact_violations(current_alpha);
            } else {
                force.update_joint_violations();
            }

            for (usz row = 0; row < force.num_rows; row++) {
                if (force.stiffness[row] != float.max) continue;  // only hard constraints

                float lambda_updated = math::clamp(
                    force.penalty[row] * force.c_value[row] + force.lambda[row],
                    force.fmin[row], force.fmax[row]);

                bool active = lambda_updated > force.fmin[row] && lambda_updated < force.fmax[row];

                force.lambda[row] = lambda_updated;

                if (active) {
                    force.penalty[row] = math::min(
                        force.penalty[row] + AVBD_BETA * math::abs(force.c_value[row]),
                        PENALTY_MAX);
                }
            }
        }
    }
}

// === PHASE 6: Derive velocities ===
fn void PhysicsWorld.derive_velocities(&self, float dt) @local {
    foreach (body : self.active_bodies) {
        // Derive velocities from position deltas
        body.linear_velocity = (body.collider.translation - body.initial_position) / dt;

        QuaternionValue delta_q = body.collider.rotation * body.initial_orientation.conjugate();
        body.angular_velocity = delta_q.v.xyz * (2.0f / dt);
        if (delta_q.l < 0.0f) body.angular_velocity = -body.angular_velocity;

        // Flat damping
        body.linear_velocity *= 0.90f;
        body.angular_velocity *= 0.85f;

        // Clamp angular velocity
        clamp_angular_velocity(body);

        // Clamp very small velocities to zero
        const float VELOCITY_CLAMP_SQ = 0.01f;
        if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.linear_velocity = {0, 0, 0};
        }
        if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.angular_velocity = {0, 0, 0};
        }
    }

    // Sequential: Update spatial map and sleep timers
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


// Check collisions using CoACD decomposed convex pieces
fn void PhysicsWorld.check_coacd_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) {
    bool a_has_compound = body_a.collider.get_type() == ShapeType.COMPOUND;
    bool b_has_compound = body_b.collider.get_type() == ShapeType.COMPOUND;

    Rigidbody* compound_body;
    Rigidbody* other_body;
    bool compound_is_a;

    if (a_has_compound) {
        compound_body = body_a;
        other_body = body_b;
        compound_is_a = true;
    } else {
        compound_body = body_b;
        other_body = body_a;
        compound_is_a = false;
    }

    CompoundShape* compound = (CompoundShape*)compound_body.collider.shape;

    // Reset contact cache
    self.contact_cache_count = 0;

    foreach (&piece : compound.shapes) {
        TransformedShape piece_shape = {
            .translation = compound_body.collider.translation + compound_body.collider.rotation * piece.offset,
            .scale = compound_body.collider.scale,
            .rotation = compound_body.collider.rotation,
            .shape = piece.shape,
        };

        Aabb3 piece_aabb = piece_shape.get_transformed_aabb();
        if (!piece_aabb.collides(other_body.transformed_aabb)) continue;

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

        if (!tri_aabb.collides(other_body.transformed_aabb)) continue;

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

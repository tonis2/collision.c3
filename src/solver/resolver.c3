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
const MAX_CONTACTS_PER_PAIR @local = 8;

// AVBD solver constants (matching reference solver.h)
const PENALTY_MIN = 1.0f;
const PENALTY_MAX = 1e10f;
const COLLISION_MARGIN = 0.01f;
const STICK_THRESH = 0.00001f;
const MAX_ANGULAR_SPEED = 80.0f;

alias CollisionFilter = HashMap{Pair, bool};
alias RigidBodyMap = LinkedHashMap{usz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator{usz, Rigidbody};
alias UnionFindMap = HashMap{usz, usz};

// Task structure for parallel collision testing
struct CollisionTask {
    PhysicsWorld* world;
    Pair pair;
    Collision* result;
    bool has_collision;
}

// === Contact manifold data (matching reference Manifold) ===

struct ManifoldContact {
    int feature_key;
    Vec3 rA;            // contact offset in A's local space
    Vec3 rB;            // contact offset in B's local space
    Vec3 c0;            // constraint at step start (in basis space)
    Vec3 penalty;       // per-axis penalty: normal, tangent1, tangent2
    Vec3 lambda;        // accumulated force
    bool stick;         // static friction active
}

struct ManifoldForce {
    Rigidbody* body_a;
    Rigidbody* body_b;
    ManifoldContact[8] contacts;
    Matrix3f basis;     // row0=normal(B->A), row1,2=tangents
    int num_contacts;
    float friction;
}

// === Joint data (matching reference Joint) ===

struct JointForce {
    Rigidbody* body_a;
    Rigidbody* body_b;
    Vec3 rA;            // anchor in A's local space
    Vec3 rB;            // anchor in B's local space
    Vec3 c0_lin;        // linear constraint at step start
    Vec3 c0_ang;        // angular constraint at step start
    Vec3 penalty_lin;
    Vec3 penalty_ang;
    Vec3 lambda_lin;
    Vec3 lambda_ang;
    float stiffness_lin;
    float stiffness_ang;
    float fracture;
    float torque_arm;
    bool broken;
}

// === Spring data (matching reference Spring) ===

struct SpringForce {
    Rigidbody* body_a;
    Rigidbody* body_b;
    Vec3 rA;
    Vec3 rB;
    float rest;
    float stiffness;
}

alias ManifoldCache = HashMap{Pair, ManifoldForce};

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
    START,
    END
}

struct ContactEvent {
    usz body_a;
    usz body_b;
    ContactEventType type;
    Vec3 normal;
    Vec3 contact_point;
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
	bool is_kinematic;

    Vec3 mass_center;
    Matrix3f inertia;           // Inverse inertia
    Matrix3f inertia_local;     // Non-inverse local inertia tensor (diagonal moment)

    // AVBD state
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
    List{ManifoldForce} manifolds;
    List{JointForce} joint_forces;
    List{SpringForce} spring_forces;
    List{GenericJoint3D} joints;
    SpatialHash3D spatial_map;
    CollisionFilter filters;
    UnionFindMap island_parent;
	bool islands_dirty;

    // Trigger system
    List{TriggerEvent} trigger_events;
    TriggerOverlapSet active_triggers;

    // Contact event system
    List{ContactEvent} contact_events;
    ContactOverlapSet active_contacts;

    Vec3 gravity;
    float sleep_timer;
	float sleep_delta;
	float angular_dampening;
	float linear_dampening;

    // AVBD solver parameters (matching reference solver.cpp:109-132)
    float avbd_alpha;
    float avbd_beta_lin;
    float avbd_beta_ang;
    float avbd_gamma;

    // Threading support
    ThreadPool{NUM_WORKER_THREADS} thread_pool;
    Mutex collision_mutex;
    bool thread_pool_initialized;

    // Reusable lists
    List{CollisionTask} collision_tasks;
    List{Collision} collision_results;
    List{Rigidbody*} active_bodies;

    // Contact sorting cache (keep deepest N contacts per pair)
    Collision[MAX_CONTACTS_PER_PAIR] contact_cache;
    usz contact_cache_count;

    // Manifold cache from previous frame (for warmstarting)
    ManifoldCache manifold_cache;
}

fn void PhysicsWorld.free(&self) {
    if (self.thread_pool_initialized) {
        self.thread_pool.stop_and_destroy()!!;
        self.collision_mutex.destroy()!!;
    }
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.manifolds.free();
    self.joint_forces.free();
    self.spring_forces.free();
    self.filters.free();
    self.joints.free();
    self.island_parent.free();
    self.collision_tasks.free();
    self.collision_results.free();
    self.active_bodies.free();
    self.trigger_events.free();
    self.active_triggers.free();
    self.contact_events.free();
    self.active_contacts.free();
    self.manifold_cache.free();
}

const PhysicsWorld DEFAULT_PHYSICS_WORLD = {
	.gravity = {0.0f, 0.0f, -9.8f},
	.spatial_map.cell_size = 2.0,
	.sleep_timer = 5.0,
	.linear_dampening = 0.9,
	.angular_dampening = 0.9,
	.sleep_delta = 0.1,
    .avbd_alpha = 0.99,
    .avbd_beta_lin = 10000.0,
    .avbd_beta_ang = 100.0,
    .avbd_gamma = 0.999,
};

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody*? PhysicsWorld.find_body(self, usz id) => self.bodies.get_ref(id);

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint) {
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

    body.aabb = body.collider.get_aabb();
	body.sleep_delta = self.sleep_delta;

    body.transformed_aabb = body.get_transformed_aabb();
    if (!body.is_static && &body.collider.shape.inertia_tensor) {
        body.inertia_local = body.aabb.inertia_tensor(body.mass);
        body.inertia = body.inertia_local.inverse()!!;
    }

    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

fn void PhysicsWorld.set_kinematic_target(&self, usz id, Vec3 position, QuaternionValue rotation)
{
	Rigidbody* body = self.find_body(id)!!;
	if (!body.is_kinematic) return;
}

<*
 @require body.collider.shape != null : `Trigger must have collision shape`
 @require body.collider.rotation != (QuaternionValue){0,0,0,0} : `Trigger rotation must not be 0`
 @require body.collider.scale != {0,0,0} : `Trigger scale must not be 0`
*>
fn void? PhysicsWorld.add_trigger(&self, Rigidbody body) {
    body.is_trigger = true;
    body.is_static = true;
    body.inverse_mass = 0;
    body.mass = 0;

    body.aabb = body.collider.get_aabb();
    body.transformed_aabb = body.get_transformed_aabb();

    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

macro void PhysicsWorld.@get_trigger_events(&self; @callback(TriggerEvent event)) {
    foreach (event : self.trigger_events) {
        @callback(event);
    }
}

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
        usz root = self.island_find(parent);
        self.island_parent.set(id, root);
        return root;
    }
    return id;
}

fn void PhysicsWorld.island_union(&self, usz a, usz b) {
    usz root_a = self.island_find(a);
    usz root_b = self.island_find(b);

    if (root_a != root_b) {
        if (root_a < root_b) {
            self.island_parent.set(root_b, root_a);
        } else {
            self.island_parent.set(root_a, root_b);
        }
    }
}

fn void PhysicsWorld.compute_islands(&self) {
    if (!self.islands_dirty) return;

    self.island_parent.clear();

    RigidBodyIterator iter = self.bodies.value_iter();
    while (iter.next()) {
        Rigidbody* body = iter.get()!!;
        self.island_parent.set(body.id, body.id);
    }

    foreach (&joint: self.joints) {
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

    RigidBodyIterator assign_iter = self.bodies.value_iter();
    while (assign_iter.next()) {
        Rigidbody* body = assign_iter.get()!!;
        body.island_id = self.island_find(body.id);
    }

    self.islands_dirty = false;
}

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
fn int collision_test_worker(void* arg) {
    CollisionTask* task = (CollisionTask*)arg;
    PhysicsWorld* world = task.world;

    Rigidbody* body_a = world.find_body(task.pair.a)!!;
    Rigidbody* body_b = world.find_body(task.pair.b)!!;

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

fn float combine_values(float a, float b, PhysicsCombine mode) {
    switch (mode) {
        case AVERAGE: return (a + b) * 0.5f;
        case MINIMUM: return math::min(a, b);
        case MAXIMUM: return math::max(a, b);
        case MULTIPLY:	return a * b;
		default: return a * b;
    }
}

fn bool is_manifold_shape(ShapeType t) @inline {
    return t == ShapeType.CONVEX || t == ShapeType.BOX;
}

// === Contact manifold generation ===
// Expand single EPA contact into multi-contact manifold
fn void PhysicsWorld.generate_manifold(&self, Collision* collision) {
    Rigidbody* a = self.find_body(collision.a)!!;
    Rigidbody* b = self.find_body(collision.b)!!;

    ShapeType a_type = a.collider.get_type();
    ShapeType b_type = b.collider.get_type();

    if (!is_manifold_shape(a_type) && !is_manifold_shape(b_type)) {
        self.collisions.push(*collision);
        return;
    }

    bool incident_is_a;
    bool a_candidate = is_manifold_shape(a_type) && !a.is_static;
    bool b_candidate = is_manifold_shape(b_type) && !b.is_static;

    if (!a_candidate && !b_candidate) {
        self.collisions.push(*collision);
        return;
    }

    if (a_candidate && b_candidate) {
        Vec3 ext_a = a.transformed_aabb.max - a.transformed_aabb.min;
        Vec3 ext_b = b.transformed_aabb.max - b.transformed_aabb.min;
        float sa_a = ext_a.x*ext_a.y + ext_a.y*ext_a.z + ext_a.z*ext_a.x;
        float sa_b = ext_b.x*ext_b.y + ext_b.y*ext_b.z + ext_b.z*ext_b.x;
        incident_is_a = sa_a <= sa_b;
    } else {
        incident_is_a = a_candidate;
    }

    Rigidbody* incident = incident_is_a ? a : b;
    ShapeType incident_type = incident_is_a ? a_type : b_type;

    Vec3[8] box_corners;
    Vec3[] local_verts;

    if (incident_type == ShapeType.BOX) {
        Aabb3* box = (Aabb3*)incident.collider.shape;
        box_corners = {
            box.min,
            { box.max.x, box.min.y, box.min.z },
            { box.min.x, box.max.y, box.min.z },
            { box.min.x, box.min.y, box.max.z },
            { box.max.x, box.max.y, box.min.z },
            { box.max.x, box.min.y, box.max.z },
            { box.min.x, box.max.y, box.max.z },
            box.max,
        };
        local_verts = &box_corners;
    } else {
        Mesh* mesh = (Mesh*)incident.collider.shape;
        local_verts = mesh.vertices.entries[:mesh.vertices.size];
    }

    Vec3 plane_point = incident_is_a ? collision.contact[1] : collision.contact[0];
    Vec3 plane_normal = incident_is_a ? -collision.normal : collision.normal;
    Vec3 orig_contact = incident_is_a ? collision.contact[0] : collision.contact[1];

    float depth_tol = collision.depth + 0.02f;
    const float DEDUP_DIST_SQ = 0.0001f;
    Vec3 inc_ext = incident.transformed_aabb.max - incident.transformed_aabb.min;
    float max_spread_sq = inc_ext.sq_magnitude();

    self.collisions.push(*collision);

    foreach (i, local_v : local_verts) {
        Vec3 v = incident.collider.rotation * (local_v * incident.collider.scale)
               + incident.collider.translation;

        float dist_to_orig = v.distance_sq(orig_contact);

        if (dist_to_orig < DEDUP_DIST_SQ || dist_to_orig > max_spread_sq) continue;

        float signed_dist = (v - plane_point).dot(plane_normal);
        if (signed_dist >= depth_tol) continue;

        float depth = math::max(0.0f, -signed_dist);
        Vec3 projected = v - plane_normal * signed_dist;

        Collision new_c = {
            .a = collision.a,
            .b = collision.b,
            .normal = collision.normal,
            .depth = depth,
        };

        if (incident_is_a) {
            new_c.contact = { v, projected };
        } else {
            new_c.contact = { projected, v };
        }

        self.collisions.push(new_c);
    }
}

// === Create manifold forces from collisions (matching reference Manifold::initialize) ===
fn void PhysicsWorld.create_manifold_forces(&self, float alpha, float gamma, float dt) {
    // Group collisions by body pair, create one ManifoldForce per pair
    HashMap{Pair, usz} pair_to_manifold;
    defer pair_to_manifold.free();

    foreach (&collision: self.collisions) {
        Rigidbody* a = self.find_body(collision.a)!!;
        Rigidbody* b = self.find_body(collision.b)!!;

        // Normal points A->B in collision system, AVBD needs B->A
        Vec3 normal = -collision.normal.normalize();

        // Compute body-local contact offsets
        Vec3 rA = a.collider.rotation.conjugate() * (collision.contact[0] - a.collider.translation);
        Vec3 rB = b.collider.rotation.conjugate() * (collision.contact[1] - b.collider.translation);

        // Feature key from quantized positions (for contact matching)
        int feature_key = (int)(rA.x * 1000) ^ ((int)(rA.y * 1000) << 10) ^ ((int)(rA.z * 1000) << 20);

        Pair pair = { collision.a, collision.b };

        usz manifold_idx;
        if (try idx = pair_to_manifold.get(pair)) {
            manifold_idx = idx;
        } else {
            // Combine friction (geometric mean, matching reference)
            float friction = math::sqrt(a.material.static_friction * b.material.static_friction);

            ManifoldForce mf = {
                .body_a = a,
                .body_b = b,
                .basis = orthonormal_basis(normal),
                .num_contacts = 0,
                .friction = friction,
            };
            self.manifolds.push(mf);
            manifold_idx = self.manifolds.len() - 1;
            pair_to_manifold.set(pair, manifold_idx);
        }

        ManifoldForce* mf = &self.manifolds[manifold_idx];
        if (mf.num_contacts >= 8) continue;

        // Initial penalty: must be comparable to M/dt² to resist gravity from first iteration.
        // With small substeps (dt << 1/60), PENALTY_MIN=1 is far too low.
        float eff_mass = math::max(a.mass, b.mass);
        float inv_dt2 = 1.0f / (dt * dt);
        float initial_penalty = math::max(PENALTY_MIN, eff_mass * inv_dt2);

        int ci = mf.num_contacts;
        mf.contacts[ci].feature_key = feature_key;
        mf.contacts[ci].rA = rA;
        mf.contacts[ci].rB = rB;
        mf.contacts[ci].penalty = { initial_penalty, PENALTY_MIN, PENALTY_MIN };
        mf.contacts[ci].lambda = { 0, 0, 0 };
        mf.contacts[ci].stick = false;
        mf.num_contacts++;
    }

    // Merge with previous frame's manifold cache (warmstart by feature key)
    for (usz mi = 0; mi < self.manifolds.len(); mi++) {
        ManifoldForce* mf = &self.manifolds[mi];
        Pair pair = { mf.body_a.id, mf.body_b.id };

        ManifoldForce*? cached = self.manifold_cache.get_ref(pair);

        if (try old = cached) {
            // Match contacts: first try feature key, then fall back to nearest
            bool[8] old_used = {};
            for (int i = 0; i < mf.num_contacts; i++) {
                int best_j = -1;
                float best_dist = 0.04f; // threshold: 0.2^2

                // Try exact feature key match first
                for (int j = 0; j < old.num_contacts; j++) {
                    if (old_used[j]) continue;
                    if (mf.contacts[i].feature_key == old.contacts[j].feature_key) {
                        best_j = j;
                        break;
                    }
                }

                // Fall back to nearest contact by body-local position
                if (best_j < 0) {
                    for (int j = 0; j < old.num_contacts; j++) {
                        if (old_used[j]) continue;
                        float d = mf.contacts[i].rA.distance_sq(old.contacts[j].rA);
                        if (d < best_dist) {
                            best_dist = d;
                            best_j = j;
                        }
                    }
                }

                if (best_j >= 0) {
                    old_used[best_j] = true;
                    Vec3 new_rA = mf.contacts[i].rA;
                    Vec3 new_rB = mf.contacts[i].rB;

                    mf.contacts[i].lambda = old.contacts[best_j].lambda;
                    mf.contacts[i].penalty = old.contacts[best_j].penalty;
                    mf.contacts[i].stick = old.contacts[best_j].stick;

                    if (!old.contacts[best_j].stick) {
                        mf.contacts[i].rA = new_rA;
                        mf.contacts[i].rB = new_rB;
                    }
                }
            }
        }

        // Compute C0 and warmstart (matching reference manifold.cpp:56-67)
        for (int i = 0; i < mf.num_contacts; i++) {
            Vec3 xA = quat_transform(mf.body_a.collider.translation, mf.body_a.collider.rotation, mf.contacts[i].rA);
            Vec3 xB = quat_transform(mf.body_b.collider.translation, mf.body_b.collider.rotation, mf.contacts[i].rB);
            mf.contacts[i].c0 = mf.basis * (xA - xB) + (Vec3){COLLISION_MARGIN, 0, 0};

            // Warmstart dual variables and penalty (Eq. 19)
            mf.contacts[i].lambda = mf.contacts[i].lambda * (alpha * gamma);
            mf.contacts[i].penalty = vec3_clamp(mf.contacts[i].penalty * gamma, PENALTY_MIN, PENALTY_MAX);
        }
    }
}

// === Manifold updatePrimal (matching reference manifold.cpp:72-125) ===
fn void ManifoldForce.update_primal(&self, Rigidbody* body, float alpha,
    Matrix3f* lhs_lin, Matrix3f* lhs_ang, Matrix3f* lhs_cross, Vec3* rhs_lin, Vec3* rhs_ang)
{
    Vec3 dq_a_lin = self.body_a.collider.translation - self.body_a.initial_position;
    Vec3 dq_a_ang = quat_sub(self.body_a.collider.rotation, self.body_a.initial_orientation);
    Vec3 dq_b_lin = self.body_b.collider.translation - self.body_b.initial_position;
    Vec3 dq_b_ang = quat_sub(self.body_b.collider.rotation, self.body_b.initial_orientation);

    for (int i = 0; i < self.num_contacts; i++) {
        Vec3 rA_world = quat_rotate(self.body_a.collider.rotation, self.contacts[i].rA);
        Vec3 rB_world = quat_rotate(self.body_b.collider.rotation, self.contacts[i].rB);

        // Jacobians (basis rows are normal, tangent1, tangent2)
        Matrix3f j_a_lin = self.basis;
        Matrix3f j_b_lin = -self.basis;

        // Angular Jacobians: cross(rWorld, jLin[row]) for each constraint row
        Vec3 j_a_lin_r0 = { self.basis.m00, self.basis.m01, self.basis.m02 };
        Vec3 j_a_lin_r1 = { self.basis.m10, self.basis.m11, self.basis.m12 };
        Vec3 j_a_lin_r2 = { self.basis.m20, self.basis.m21, self.basis.m22 };

        Matrix3f j_a_ang = {
            rA_world.cross(j_a_lin_r0).x, rA_world.cross(j_a_lin_r0).y, rA_world.cross(j_a_lin_r0).z,
            rA_world.cross(j_a_lin_r1).x, rA_world.cross(j_a_lin_r1).y, rA_world.cross(j_a_lin_r1).z,
            rA_world.cross(j_a_lin_r2).x, rA_world.cross(j_a_lin_r2).y, rA_world.cross(j_a_lin_r2).z,
        };

        Vec3 j_b_lin_r0 = { -self.basis.m00, -self.basis.m01, -self.basis.m02 };
        Vec3 j_b_lin_r1 = { -self.basis.m10, -self.basis.m11, -self.basis.m12 };
        Vec3 j_b_lin_r2 = { -self.basis.m20, -self.basis.m21, -self.basis.m22 };

        Matrix3f j_b_ang = {
            rB_world.cross(j_b_lin_r0).x, rB_world.cross(j_b_lin_r0).y, rB_world.cross(j_b_lin_r0).z,
            rB_world.cross(j_b_lin_r1).x, rB_world.cross(j_b_lin_r1).y, rB_world.cross(j_b_lin_r1).z,
            rB_world.cross(j_b_lin_r2).x, rB_world.cross(j_b_lin_r2).y, rB_world.cross(j_b_lin_r2).z,
        };

        // Penalty matrix K = diagonal(penalty)
        Matrix3f k = mat3_diagonal(self.contacts[i].penalty);

        // Taylor series constraint approximation (Sec 4)
        Vec3 c = self.contacts[i].c0 * (1.0f - alpha)
               + j_a_lin * dq_a_lin + j_b_lin * dq_b_lin
               + j_a_ang * dq_a_ang + j_b_ang * dq_b_ang;

        // Compute force
        Vec3 f = k * c + self.contacts[i].lambda;

        // Clamp normal force (repulsive only)
        f.x = math::min(f.x, 0.0f);

        // Clamp friction to Coulomb cone
        float bounds = math::abs(f.x) * self.friction;
        float friction_scale = math::sqrt(f.y * f.y + f.z * f.z);
        if (friction_scale > bounds && friction_scale > 0) {
            float scale = bounds / friction_scale;
            f.y *= scale;
            f.z *= scale;
        }

        // Choose Jacobian depending on input body
        Matrix3f j_lin = body == self.body_a ? j_a_lin : j_b_lin;
        Matrix3f j_ang = body == self.body_a ? j_a_ang : j_b_ang;

        // Stamp into LHS
        Matrix3f j_lin_t = j_lin.transpose();
        Matrix3f j_ang_t = j_ang.transpose();
        Matrix3f j_ang_tk = j_ang_t * k;

        *lhs_lin = *lhs_lin + j_lin_t * k * j_lin;
        *lhs_ang = *lhs_ang + j_ang_tk * j_ang;
        *lhs_cross = *lhs_cross + j_ang_tk * j_lin;

        // Stamp into RHS
        *rhs_lin = *rhs_lin + j_lin_t * f;
        *rhs_ang = *rhs_ang + j_ang_t * f;
    }
}

// === Manifold updateDual (matching reference manifold.cpp:127-176) ===
fn void ManifoldForce.update_dual(&self, float alpha, float beta_lin) {
    Vec3 dq_a_lin = self.body_a.collider.translation - self.body_a.initial_position;
    Vec3 dq_a_ang = quat_sub(self.body_a.collider.rotation, self.body_a.initial_orientation);
    Vec3 dq_b_lin = self.body_b.collider.translation - self.body_b.initial_position;
    Vec3 dq_b_ang = quat_sub(self.body_b.collider.rotation, self.body_b.initial_orientation);

    for (int i = 0; i < self.num_contacts; i++) {
        Vec3 rA_world = quat_rotate(self.body_a.collider.rotation, self.contacts[i].rA);
        Vec3 rB_world = quat_rotate(self.body_b.collider.rotation, self.contacts[i].rB);

        // Same Jacobians as updatePrimal
        Matrix3f j_a_lin = self.basis;
        Matrix3f j_b_lin = -self.basis;

        Vec3 j_a_lin_r0 = { self.basis.m00, self.basis.m01, self.basis.m02 };
        Vec3 j_a_lin_r1 = { self.basis.m10, self.basis.m11, self.basis.m12 };
        Vec3 j_a_lin_r2 = { self.basis.m20, self.basis.m21, self.basis.m22 };

        Matrix3f j_a_ang = {
            rA_world.cross(j_a_lin_r0).x, rA_world.cross(j_a_lin_r0).y, rA_world.cross(j_a_lin_r0).z,
            rA_world.cross(j_a_lin_r1).x, rA_world.cross(j_a_lin_r1).y, rA_world.cross(j_a_lin_r1).z,
            rA_world.cross(j_a_lin_r2).x, rA_world.cross(j_a_lin_r2).y, rA_world.cross(j_a_lin_r2).z,
        };

        Vec3 j_b_lin_r0 = { -self.basis.m00, -self.basis.m01, -self.basis.m02 };
        Vec3 j_b_lin_r1 = { -self.basis.m10, -self.basis.m11, -self.basis.m12 };
        Vec3 j_b_lin_r2 = { -self.basis.m20, -self.basis.m21, -self.basis.m22 };

        Matrix3f j_b_ang = {
            rB_world.cross(j_b_lin_r0).x, rB_world.cross(j_b_lin_r0).y, rB_world.cross(j_b_lin_r0).z,
            rB_world.cross(j_b_lin_r1).x, rB_world.cross(j_b_lin_r1).y, rB_world.cross(j_b_lin_r1).z,
            rB_world.cross(j_b_lin_r2).x, rB_world.cross(j_b_lin_r2).y, rB_world.cross(j_b_lin_r2).z,
        };

        Matrix3f k = mat3_diagonal(self.contacts[i].penalty);

        Vec3 c = self.contacts[i].c0 * (1.0f - alpha)
               + j_a_lin * dq_a_lin + j_b_lin * dq_b_lin
               + j_a_ang * dq_a_ang + j_b_ang * dq_b_ang;

        // Compute force
        Vec3 f = k * c + self.contacts[i].lambda;

        // Clamp normal force
        f.x = math::min(f.x, 0.0f);

        // Clamp friction cone
        float bounds = math::abs(f.x) * self.friction;
        float friction_scale = math::sqrt(f.y * f.y + f.z * f.z);
        if (friction_scale > bounds && friction_scale > 0) {
            float scale = bounds / friction_scale;
            f.y *= scale;
            f.z *= scale;
        }

        // Store updated force
        self.contacts[i].lambda = f;

        // Update penalty (Eq. 16)
        if (f.x < 0) {
            self.contacts[i].penalty.x = math::min(self.contacts[i].penalty.x + beta_lin * math::abs(c.x), PENALTY_MAX);
        }
        if (friction_scale <= bounds) {
            self.contacts[i].penalty.y = math::min(self.contacts[i].penalty.y + beta_lin * math::abs(c.y), PENALTY_MAX);
            self.contacts[i].penalty.z = math::min(self.contacts[i].penalty.z + beta_lin * math::abs(c.z), PENALTY_MAX);
            self.contacts[i].stick = math::sqrt(c.y * c.y + c.z * c.z) < STICK_THRESH;
        }
    }
}

// === Joint updatePrimal (matching reference joint.cpp:54-119) ===
fn void JointForce.update_primal(&self, Rigidbody* body, float alpha,
    Matrix3f* lhs_lin, Matrix3f* lhs_ang, Matrix3f* lhs_cross, Vec3* rhs_lin, Vec3* rhs_ang)
{
    // Linear constraint (ball-socket)
    if (self.penalty_lin.sq_magnitude() > 0) {
        Matrix3f k = mat3_diagonal(self.penalty_lin);
        Vec3 c = quat_transform(self.body_a.collider.translation, self.body_a.collider.rotation, self.rA)
               - quat_transform(self.body_b.collider.translation, self.body_b.collider.rotation, self.rB);

        // Stabilization for hard constraints
        if (self.stiffness_lin == float.max) {
            c = c - self.c0_lin * alpha;
        }

        Vec3 f = k * c + self.lambda_lin;

        // Choose Jacobian depending on input body
        Matrix3f j_lin;
        Matrix3f j_ang;
        Vec3 r;
        if (body == self.body_a) {
            j_lin = MATRIX3F_IDENTITY;
            r = quat_rotate(self.body_a.collider.rotation, self.rA);
            j_ang = mat3_skew(-r);
        } else {
            j_lin = -MATRIX3F_IDENTITY;
            r = quat_rotate(self.body_b.collider.rotation, self.rB);
            j_ang = mat3_skew(r);
        }

        // Stamp into LHS
        Matrix3f j_lin_t = j_lin.transpose();
        Matrix3f j_ang_t = j_ang.transpose();
        Matrix3f j_ang_tk = j_ang_t * k;

        *lhs_lin = *lhs_lin + j_lin_t * k * j_lin;
        *lhs_ang = *lhs_ang + j_ang_tk * j_ang;
        *lhs_cross = *lhs_cross + j_ang_tk * j_lin;

        // Geometric stiffness (higher order angular correction)
        Vec3 geo_r = body == self.body_a ? quat_rotate(self.body_a.collider.rotation, self.rA) : -quat_rotate(self.body_b.collider.rotation, self.rB);
        Matrix3f h = geometric_stiffness_ball_socket(0, geo_r) * f.x
                   + geometric_stiffness_ball_socket(1, geo_r) * f.y
                   + geometric_stiffness_ball_socket(2, geo_r) * f.z;
        *lhs_ang = *lhs_ang + mat3_diagonalize(h);

        // Stamp into RHS
        *rhs_lin = *rhs_lin + j_lin_t * f;
        *rhs_ang = *rhs_ang + j_ang_t * f;
    }

    // Angular constraint
    if (self.penalty_ang.sq_magnitude() > 0) {
        Matrix3f k = mat3_diagonal(self.penalty_ang);
        Vec3 c = quat_sub(self.body_a.collider.rotation, self.body_b.collider.rotation) * self.torque_arm;

        // Stabilization for hard constraints
        if (self.stiffness_ang == float.max) {
            c = c - self.c0_ang * alpha;
        }

        Vec3 f = k * c + self.lambda_ang;

        // Jacobian: +-I * torqueArm
        Matrix3f j_ang;
        if (body == self.body_a) {
            j_ang = MATRIX3F_IDENTITY * self.torque_arm;
        } else {
            j_ang = -MATRIX3F_IDENTITY * self.torque_arm;
        }

        // Stamp into LHS (angular only, no cross term)
        *lhs_ang = *lhs_ang + j_ang.transpose() * k * j_ang;

        // Stamp into RHS
        *rhs_ang = *rhs_ang + j_ang.transpose() * f;
    }
}

// === Joint updateDual (matching reference joint.cpp:121-178) ===
fn void JointForce.update_dual(&self, float alpha, float beta_lin, float beta_ang) {
    // Linear constraint
    if (self.penalty_lin.sq_magnitude() > 0) {
        Matrix3f k = mat3_diagonal(self.penalty_lin);
        Vec3 c = quat_transform(self.body_a.collider.translation, self.body_a.collider.rotation, self.rA)
               - quat_transform(self.body_b.collider.translation, self.body_b.collider.rotation, self.rB);

        if (self.stiffness_lin == float.max) {
            c = c - self.c0_lin * alpha;
            Vec3 f = k * c + self.lambda_lin;
            self.lambda_lin = f;
        }

        // Update penalty and clamp to material stiffness (Eq. 16)
        self.penalty_lin = vec3_min(self.penalty_lin + c.abs() * beta_lin, math::min(self.stiffness_lin, PENALTY_MAX));
    }

    // Angular constraint
    if (self.penalty_ang.sq_magnitude() > 0) {
        Matrix3f k = mat3_diagonal(self.penalty_ang);
        Vec3 c = quat_sub(self.body_a.collider.rotation, self.body_b.collider.rotation) * self.torque_arm;

        if (self.stiffness_ang == float.max) {
            c = c - self.c0_ang * alpha;
            Vec3 f = k * c + self.lambda_ang;
            self.lambda_ang = f;
        }

        self.penalty_ang = vec3_min(self.penalty_ang + c.abs() * beta_ang, math::min(self.stiffness_ang, PENALTY_MAX));
    }

    // Fracture test
    if (self.lambda_ang.sq_magnitude() > self.fracture * self.fracture) {
        self.penalty_lin = { 0, 0, 0 };
        self.penalty_ang = { 0, 0, 0 };
        self.lambda_lin = { 0, 0, 0 };
        self.lambda_ang = { 0, 0, 0 };
        self.broken = true;
    }
}

// === Spring updatePrimal (matching reference spring.cpp:25-67) ===
fn void SpringForce.update_primal(&self, Rigidbody* body, float alpha,
    Matrix3f* lhs_lin, Matrix3f* lhs_ang, Matrix3f* lhs_cross, Vec3* rhs_lin, Vec3* rhs_ang)
{
    Vec3 pA = quat_transform(self.body_a.collider.translation, self.body_a.collider.rotation, self.rA);
    Vec3 pB = quat_transform(self.body_b.collider.translation, self.body_b.collider.rotation, self.rB);
    Vec3 d = pA - pB;
    float d_len = d.length();
    if (d_len <= 1.0e-6f) return;

    Vec3 n = d / d_len;
    float c = d_len - self.rest;
    float f = self.stiffness * c;

    Vec3 r_world;
    Vec3 j_lin;
    Vec3 j_ang;
    if (body == self.body_a) {
        r_world = quat_rotate(self.body_a.collider.rotation, self.rA);
        j_lin = n;
        j_ang = r_world.cross(n);
    } else {
        r_world = quat_rotate(self.body_b.collider.rotation, self.rB);
        j_lin = -n;
        j_ang = -r_world.cross(n);
    }

    Vec3 fv = j_lin * f;
    Vec3 tau = j_ang * f;
    Matrix3f k_ll = mat3_outer(j_lin, j_lin) * self.stiffness;
    Matrix3f k_la = mat3_outer(j_ang, j_lin) * self.stiffness;
    Matrix3f k_aa = mat3_outer(j_ang, j_ang) * self.stiffness;

    *lhs_lin = *lhs_lin + k_ll;
    *lhs_ang = *lhs_ang + k_aa;
    *lhs_cross = *lhs_cross + k_la;
    *rhs_lin = *rhs_lin + fv;
    *rhs_ang = *rhs_ang + tau;
}

// Clamp angular velocity magnitude
fn void clamp_angular_velocity(Rigidbody* body) {
    float sq_mag = body.angular_velocity.sq_magnitude();
    if (sq_mag > MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED) {
        body.angular_velocity = body.angular_velocity * (MAX_ANGULAR_SPEED / math::sqrt(sq_mag));
    }
}


<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step_internal(&self, float time, uint step_count = 25) {
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

    // === PHASE 1: Broad-phase - collect candidate pairs ===
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

    // Generate EXIT events for triggers no longer overlapping
    self.active_triggers.@each(;Pair old_pair, bool active) {
        if (!current_triggers.has_key(old_pair)) {
            self.trigger_events.push({
                .trigger_id = old_pair.a,
                .other_id = old_pair.b,
                .type = TriggerEventType.EXIT
            });
        }
    };

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

    // === PHASE 2.5: Contact events ===
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

    // === PHASE 2.75: Expand contact manifolds ===
    {
        usz raw_count = self.collisions.len();
        for (usz i = 0; i < raw_count; i++) {
            Collision c = self.collisions[i];
            self.generate_manifold(&c);
        }
        if (raw_count > 0) {
            usz expanded_count = self.collisions.len() - raw_count;
            for (usz i = 0; i < expanded_count; i++) {
                self.collisions[i] = self.collisions[raw_count + i];
            }
            while (self.collisions.len() > expanded_count) {
                self.collisions.pop()!!;
            }
        }
    }

    // === PHASE 3: Create forces ===
    self.create_manifold_forces(self.avbd_alpha, self.avbd_gamma, time);

    foreach (&joint: self.joints) {
        joint.create_force(self, time);
    }

    // === PHASE 4: Predict body states ===
    self.predict_bodies(time);

    // === PHASE 5: AVBD solve ===
    self.avbd_solve(time, step_count);

    // === PHASE 6: Derive velocities + spatial map update ===
    self.derive_velocities(time);

    // === Save manifold cache for next frame ===
    self.manifold_cache.free();
    self.manifold_cache = {};
    for (usz mi = 0; mi < self.manifolds.len(); mi++) {
        ManifoldForce* mf = &self.manifolds[mi];
        Pair pair = { mf.body_a.id, mf.body_b.id };
        self.manifold_cache.set(pair, *mf);
    }

    // Save joint warm-start back to GenericJoint3D (1:1 correspondence by index)
    usz jf_count = math::min(self.joint_forces.len(), self.joints.len());
    for (usz ji = 0; ji < jf_count; ji++) {
        JointForce* jf = &self.joint_forces[ji];
        self.joints[ji].lambda_lin = jf.lambda_lin;
        self.joints[ji].lambda_ang = jf.lambda_ang;
        self.joints[ji].penalty_lin = jf.penalty_lin;
        self.joints[ji].penalty_ang = jf.penalty_ang;
    }

    // === PHASE 6.5: Sleep management ===
    self.update_sleep_state();
}

// Public entry point - runs physics then clears transient data
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 25) {
    self.run_step_internal(time, step_count);
    // Clear transient collision data (must be captured before this if needed for debug)
    self.collisions.clear();
    self.manifolds.clear();
    self.joint_forces.clear();
    self.spring_forces.clear();
}

// === PHASE 4: Predict body states ===
fn void PhysicsWorld.predict_bodies(&self, float dt) {
    self.active_bodies.clear();

    RigidBodyIterator iter = self.bodies.value_iter();
    while (iter.next()) {
        Rigidbody* body = iter.get()!!;

        // Save initial state for ALL bodies (needed by Taylor series in updatePrimal)
        body.initial_position = body.collider.translation;
        body.initial_orientation = body.collider.rotation;

        if (!body.is_active()) continue;

        self.active_bodies.push(body);

        body.prev_linear_velocity = body.linear_velocity;
        body.prev_angular_velocity = body.angular_velocity;
        clamp_angular_velocity(body);

        float dt2 = dt * dt;
        Vec3 grav = self.gravity * body.gravity_factor;

        // Inertial target (full gravity)
        body.inertial_position = body.collider.translation + body.linear_velocity * dt + grav * dt2;
        body.inertial_orientation = quat_add_vec3(body.collider.rotation, body.angular_velocity * dt);

        // Adaptive warmstart (from VBD paper, reference solver.cpp:174-177)
        // Reference: accelWeight = accel.z / gravity (both scalar)
        // Generalized: project accel onto gravity, normalize by |gravity|^2
        float gravity_sq = self.gravity.sq_magnitude();
        float accel_weight = 0.0f;
        if (gravity_sq > 1e-10f) {
            Vec3 accel = (body.linear_velocity - body.prev_linear_velocity) / dt;
            accel_weight = math::clamp(accel.dot(self.gravity) / gravity_sq, 0.0f, 1.0f);
            if (!math::is_finite(accel_weight)) accel_weight = 0.0f;
        }

        // Predicted position
        body.collider.translation += body.linear_velocity * dt + grav * (accel_weight * dt2);
        body.collider.rotation = quat_add_vec3(body.collider.rotation, body.angular_velocity * dt);
    }
}

// === PHASE 5: AVBD solve (matching reference solver.cpp:191-230) ===
fn void PhysicsWorld.avbd_solve(&self, float dt, uint iterations) {
    if (self.active_bodies.len() == 0) return;

    float inv_dt2 = 1.0f / (dt * dt);
    float alpha = self.avbd_alpha;

    for (uint it = 0; it < iterations; it++) {
        // === Primal update (per body) ===
        foreach (body : self.active_bodies) {
            // Initialize LHS and RHS with inertial terms (Eqs. 5, 6)
            Matrix3f m_lin = mat3_diagonal({body.mass, body.mass, body.mass});
            // Reference uses body-local diagonal inertia (NOT world-rotated)
            Matrix3f m_ang = body.inertia_local;

            Matrix3f lhs_lin = m_lin * inv_dt2;
            Matrix3f lhs_ang = m_ang * inv_dt2;
            Matrix3f lhs_cross = {};

            Vec3 rhs_lin = m_lin * inv_dt2 * (body.collider.translation - body.inertial_position);
            Vec3 rhs_ang = m_ang * inv_dt2 * quat_sub(body.collider.rotation, body.inertial_orientation);

            // Accumulate manifold contributions
            for (usz mi = 0; mi < self.manifolds.len(); mi++) {
                ManifoldForce* mf = &self.manifolds[mi];
                if (mf.body_a != body && mf.body_b != body) continue;
                mf.update_primal(body, alpha, &lhs_lin, &lhs_ang, &lhs_cross, &rhs_lin, &rhs_ang);
            }

            // Accumulate joint contributions
            for (usz ji = 0; ji < self.joint_forces.len(); ji++) {
                JointForce* jf = &self.joint_forces[ji];
                if (jf.body_a != body && jf.body_b != body) continue;
                jf.update_primal(body, alpha, &lhs_lin, &lhs_ang, &lhs_cross, &rhs_lin, &rhs_ang);
            }

            // Accumulate spring contributions
            for (usz si = 0; si < self.spring_forces.len(); si++) {
                SpringForce* sf = &self.spring_forces[si];
                if (sf.body_a != body && sf.body_b != body) continue;
                sf.update_primal(body, alpha, &lhs_lin, &lhs_ang, &lhs_cross, &rhs_lin, &rhs_ang);
            }

            // Solve 6x6 system with LDL^T (Eq. 4)
            Vec3 dx_lin;
            Vec3 dx_ang;
            solve_ldlt(lhs_lin, lhs_ang, lhs_cross, -rhs_lin, -rhs_ang, &dx_lin, &dx_ang);

            // Apply correction
            body.collider.translation = body.collider.translation + dx_lin;
            body.collider.rotation = quat_add_vec3(body.collider.rotation, dx_ang);
        }

        // === Dual update (per force) ===
        for (usz mi = 0; mi < self.manifolds.len(); mi++) {
            self.manifolds[mi].update_dual(alpha, self.avbd_beta_lin);
        }

        for (usz ji = 0; ji < self.joint_forces.len(); ji++) {
            self.joint_forces[ji].update_dual(alpha, self.avbd_beta_lin, self.avbd_beta_ang);
        }
        // Springs have no dual update
    }
}

// === PHASE 6: Derive velocities (simplified - no restitution/friction patches) ===
fn void PhysicsWorld.derive_velocities(&self, float dt) {
    foreach (body : self.active_bodies) {
        // BDF1 velocity derivation
        body.linear_velocity = (body.collider.translation - body.initial_position) / dt;
        body.angular_velocity = quat_sub(body.collider.rotation, body.initial_orientation) / dt;
    }

    foreach (body : self.active_bodies) {
        // Damping
        body.linear_velocity *= math::pow(self.linear_dampening, dt);
        body.angular_velocity *= math::pow(self.angular_dampening, dt);

        // Clamp angular velocity
        clamp_angular_velocity(body);

        // Clamp max linear speed
        const float MAX_LINEAR_SPEED = 50.0f;
        float sq_speed = body.linear_velocity.sq_magnitude();
        if (sq_speed > MAX_LINEAR_SPEED * MAX_LINEAR_SPEED) {
            body.linear_velocity *= MAX_LINEAR_SPEED / math::sqrt(sq_speed);
        }

        // Clamp very small velocities to zero
        const float VELOCITY_CLAMP_SQ = 0.001f;
        if (body.linear_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.linear_velocity = {0, 0, 0};
        }
        if (body.angular_velocity.sq_magnitude() < VELOCITY_CLAMP_SQ) {
            body.angular_velocity = {0, 0, 0};
        }
    }

    // Update spatial map and sleep timers
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
fn void PhysicsWorld.update_sleep_state(&self) {
    HashMap{usz, bool} island_can_sleep;
    HashMap{usz, bool} island_checked;
    defer island_can_sleep.free();
    defer island_checked.free();

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

// Thread-safe version of check_bvh_collision that returns result
fn Collision? PhysicsWorld.test_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) {
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

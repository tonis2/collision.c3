module collision;

import std::collections::fixedlist;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;
import std::thread;
import std::core::mem;
import std::sort;
import collision::dmath;

const MAX_CONTACTS_PER_PAIR @local = 8;

const float EPSILON @local = 1e-8f;
const float WEIGHT_EPSILON @local = 1e-10f;
const float MANIFOLD_DEPTH_TOL @local = 0.02f;
const float DEDUP_DIST_SQ @local = 0.0001f;
const float BOUNCE_THRESHOLD @local = 1.0f;
const float UNIFORM_SCALE_TOL @local = 0.001f;
const float DEGENERATE_DIST_SQ @local = 0.00001f * 0.00001f;
const float CLAMPED_DEPTH_MAX @local = 1.0f;

alias CollisionFilter = HashMap {Pair, bool};
alias RigidBodyMap = LinkedHashMap {sz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator {sz, Rigidbody};
alias UnionFindMap = HashMap {sz, sz};
alias ConstraintList = List {XpbdConstraint*};


// Task structure for parallel collision testing.
// Each task processes a contiguous slice of pairs — batching to at most one
// task per worker keeps thread pool overhead out of the hot path.
struct CollisionTask
{
	PhysicsWorld* world;
	Pair* pairs;
	Collision* results; // MAX_CONTACTS_PER_PAIR result slots per pair
	sz* counts; // Number of contacts written per pair
	sz count;
}

// Task structure for parallel XPBD predict/derive
struct XpbdPhaseTask
{
	PhysicsWorld* world;
	Rigidbody** bodies; // Array of body pointers to process
	sz count; // Number of bodies in this batch
	float time;
}

// Task structure for parallel soft body XPBD solving
struct SoftBodySolveTask
{
	SoftBody* body;
	PhysicsWorld* world;
	Vec3 gravity;
	float time;
	float ground_z;
	bool ground_enabled;
	float ground_friction;
}

// Task structure for island XPBD constraint solving
struct IslandSolveTask
{
	PhysicsWorld* world;
	XpbdConstraint** constraints; // Constraints for this island
	sz count; // Number of constraints
	uint iterations; // Solver iterations
}

enum XpbdConstraintType : char
{
	CONTACT_NORMAL,
	JOINT_POSITION,
	JOINT_ANGULAR,
}

struct XpbdConstraint
{
	Rigidbody* body_a;
	Rigidbody* body_b;
	Vec3 normal; // Constraint direction
	Vec3 r_a; // Contact arm from body_a center (local space for joints, world-offset for contacts)
	Vec3 r_b; // Contact arm from body_b center
	Vec3 r_a_local; // Contact arm in body_a local space (for recomputing after position changes)
	Vec3 r_b_local; // Contact arm in body_b local space
	float c_value; // Constraint error (position-level)
	float compliance; // 0 = rigid, higher = softer
	float lambda; // Accumulated Lagrange multiplier
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
	// Angular joint data (for recomputing error each iteration)
	QuaternionValue joint_orientation; // Rest orientation for angular constraints
}

struct Collision
{
	sz a;
	sz b;
	Vec3 normal;
	Vec3[2] contact;
	float depth;
	bool skip_expand; // Contact is already part of a manifold — don't vertex-expand it
}

enum TriggerEventType : char
{
	ENTER,
	EXIT
}

struct TriggerEvent
{
	sz trigger_id;
	sz other_id;
	TriggerEventType type;
}

enum ContactEventType : char
{
	START, // First frame of contact
	END // Contact ended
}

struct ContactEvent
{
	sz body_a;
	sz body_b;
	ContactEventType type;
	Vec3 normal; // Collision normal (only valid for START)
	Vec3 contact_point; // Contact point (only valid for START)
}

enum PhysicsCombine : uint
{
	AVERAGE,
	MINIMUM,
	MAXIMUM,
	MULTIPLY
}

struct Material
{
	float static_friction;
	float dynamic_friction;
	float restitution;
	PhysicsCombine friction_combine;
	PhysicsCombine restitution_combine;
}

struct Rigidbody
{
	inline TransformedShape collider;
	sz id;
	sz island_id;

	float sleep_timer;

	bool sleeping;
	bool can_sleep;
	bool is_static;
	bool is_trigger;
	bool is_kinematic;
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
	Matrix3f inertia; // Inverse local-space inertia tensor (set by add_body)

	// Conservative inner radius for swept CCD, set by add_body. A body whose
	// per-substep displacement exceeds this is swept against static geometry.
	// 0 disables CCD for this body.
	float ccd_radius;

	// XPBD: previous state for velocity derivation
	Vec3 prev_position;
	QuaternionValue prev_rotation;

	// Inverse of the rest-pose world matrix, captured at body creation. The
	// owner of the body (e.g. a scene/animation system) can compose
	// `current_world * inv_rest_world` to get a delta against rest without
	// keeping a separate rest-pose buffer per node. Cached at creation;
	// rebinding the rest pose requires recomputing this.
	Matrix4f inv_rest_world;
}

alias TriggerOverlapSet = HashMap {Pair, bool};
alias ContactOverlapSet = HashMap {Pair, bool};

struct PhysicsWorld
{
	RigidBodyMap bodies;
	List {Collision} collisions;
	List {Collision} manifold_scratch; // Raw EPA contacts awaiting manifold expansion (reused each substep)
	List {XpbdConstraint} constraints;
	List {GenericJoint3D} joints;
	SpatialHash3D spatial_map;
	CollisionFilter filters;
	UnionFindMap island_parent;
	bool islands_dirty;

	// Trigger system
	List {TriggerEvent} trigger_events;
	TriggerOverlapSet active_triggers; // Currently overlapping trigger pairs

	// Contact event system
	List {ContactEvent} contact_events;
	ContactOverlapSet active_contacts; // Currently contacting body pairs

	// Soft body system (XPBD)
	SoftBodyList soft_bodies;
	float ground_plane_z;
	bool ground_plane_enabled;

	// Swept CCD for fast-moving bodies (see apply_ccd)
	bool ccd_enabled;

	Vec3 gravity;
	float sleep_timer;
	float sleep_delta;
	float angular_dampening;
	float linear_dampening;

	// Threading support. Worker count comes from thread_count, resolved on
	// the first run_step: 0 = auto (CPU cores - 1, see default_worker_count).
	sz thread_count;
	WorkerPool thread_pool;

	// Reusable task lists (avoid per-frame allocations)
	List {CollisionTask} collision_tasks;
	List {Collision} collision_results;
	List {sz} collision_counts;
	List {IslandSolveTask} island_tasks;
	List {XpbdPhaseTask} phase_tasks;
	List {Rigidbody*} active_bodies;
}

fn void PhysicsWorld.free(&self)
{
	self.thread_pool.destroy();
	self.bodies.free();
	self.collisions.free();
	self.manifold_scratch.free();
	self.spatial_map.free();
	self.constraints.free();
	self.filters.free();
	self.joints.free();
	self.island_parent.free();
	// Free reusable task lists
	self.collision_tasks.free();
	self.collision_results.free();
	self.collision_counts.free();
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

// Library convention is z-up: soft-body ground plane, impulse_dir and the
// specialized capsule tests all assume gravity along -z.
const PhysicsWorld DEFAULT_PHYSICS_WORLD = {
	.gravity = { 0.0f, 0.0f, -9.8f },
	.spatial_map .cell_size = 2.0,
	.sleep_timer = 5.0,
	.linear_dampening = 0.9,
	.angular_dampening = 0.9,
	.sleep_delta = 0.2,
	.ccd_enabled = true,
};

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody*? PhysicsWorld.find_body(self, sz id) => self.bodies.get_ref(id);

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint)
{
	// Add collision filter for joints, so they won't collide with eachother
	if (!joint.collides)
	{
		self.filters.set({ joint.body_a, joint.body_b }, true);
	}

	self.joints.push(joint);
	self.islands_dirty = true;
}

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
 @require body.collider.scale != { 0, 0, 0 } : `Body scale must not be 0`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body)
{
	if (body.is_kinematic)
	{
		// Kinematic: externally driven, immovable for solver, always active
		body.is_static = true;
		body.inverse_mass = 0;
		body.mass = 0;
		body.can_sleep = false;
	}
	else
	{
		// Dynamic or static
		body.is_static = body.mass == 0;
		body.inverse_mass = !body.is_static ? 1.0f / body.mass : 0;
	}

	body.impulse_dir = (self.gravity * body.gravity_factor).z > 0 ? 1 : -1;

	// Compute AABB from collider
	body.aabb = body.collider.get_aabb();
	body.sleep_delta = self.sleep_delta;

	body.transformed_aabb = body.get_transformed_aabb();
	// Use AABB-based inertia - consistent for all shape types including mesh
	if (!body.is_static
		&& &body.collider.shape.inertia_tensor) body.inertia = body.aabb.inertia_tensor(body.mass).inverse()!!;

	// Conservative inner radius for swept CCD: half the smallest world-space extent
	if (!body.is_static)
	{
		Vec3 half_extent = body.aabb.extent() * body.collider.scale;
		body.ccd_radius = math::min(
			math::abs(half_extent.x),
			math::min(math::abs(half_extent.y), math::abs(half_extent.z))
		);
	}

	// Default to world dampening if not specified per-body
	if (body.linear_dampening == 0) body.linear_dampening = self.linear_dampening;
	if (body.angular_dampening == 0) body.angular_dampening = self.angular_dampening;

	// Initialize XPBD previous state so friction doesn't see phantom displacement
	body.prev_position = body.collider.translation;
	body.prev_rotation = body.collider.rotation;

	self.bodies.set(body.id, body);
	self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

<*
 Add a trigger body - detects overlaps but doesn't generate collision responses.
 Use @get_trigger_events() to iterate trigger events after run_step().
 @require body.collider.shape != null : `Trigger must have collision shape`
 @require body.collider.scale != { 0, 0, 0 } : `Trigger scale must not be 0`
*>
fn void? PhysicsWorld.add_trigger(&self, Rigidbody body)
{
	body.is_trigger = true;
	body.is_static = true; // Triggers don't move from physics
	body.inverse_mass = 0;
	body.mass = 0;

	// Compute AABB from collider
	body.aabb = body.collider.get_aabb();
	body.transformed_aabb = body.get_transformed_aabb();

	self.bodies.set(body.id, body);
	self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

<*
 Set the target position/rotation for a kinematic body. Must be called each frame
 before run_step(), with dt matching the frame delta. Computes velocity from the
 delta so dynamic bodies that collide with this kinematic get correct
 restitution/friction responses.
 @require dt > 0 : `dt must be positive`
*>
fn void PhysicsWorld.set_kinematic_target(&self, sz id, Vec3 position, QuaternionValue rotation, float dt = 1.0f / 60.0f)
{
	Rigidbody* body = self.find_body(id)!!;
	if (!body.is_kinematic) return;

	// Save previous state (for velocity derivation in constraint solver)
	body.prev_position = body.collider.translation;
	body.prev_rotation = body.collider.rotation;

	// Set new transform
	body.collider.translation = position;
	body.collider.rotation = rotation;

	// Derive velocity from delta — used by restitution/friction in the constraint solver
	float inv_dt = 1.0f / dt;
	body.linear_velocity = (position - body.prev_position) * inv_dt;

	QuaternionValue dq = rotation * body.prev_rotation.conjugate();
	if (dq.l < 0) dq = -dq;
	body.angular_velocity = dq.v.xyz * (2.0f * inv_dt);

	// Update spatial hash
	Aabb3 new_aabb = body.get_transformed_aabb();
	self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
	body.transformed_aabb = new_aabb;
}

<*
 Add a soft body to the physics world.
 The soft body uses XPBD position-based dynamics, separate from rigidbody solver.
*>
fn void PhysicsWorld.add_softbody(&self, SoftBody sb)
{
	self.soft_bodies.push(sb);
}

<*
 Remove a soft body by ID.
*>
fn void PhysicsWorld.remove_softbody(&self, sz id)
{
	for (sz i = 0; i < self.soft_bodies.len(); i++)
	{
		if (self.soft_bodies[i].id == id)
		{
			self.soft_bodies[i].free();
			self.soft_bodies.remove_at(i);
			return;
		}
	}
}

<*
 Find a soft body by ID.
*>
fn SoftBody*? PhysicsWorld.find_softbody(&self, sz id)
{
	foreach (&sb : self.soft_bodies)
	{
		if (sb.id == id) return sb;
	}
	return NOT_FOUND~;
}

<*
 Enable ground plane collision for soft bodies.
*>
fn void PhysicsWorld.enable_ground_plane(&self, float z = 0)
{
	self.ground_plane_enabled = true;
	self.ground_plane_z = z;
}

<*
 Iterate trigger events generated during the last run_step().
 Events include ENTER (overlap started) and EXIT (overlap ended).
*>
macro void PhysicsWorld.@get_trigger_events(&self; @callback(TriggerEvent event))
{
	foreach (event : self.trigger_events)
	{
		@callback(event);
	}
}

<*
 Iterate contact events generated during the last run_step().
 Events include START (first frame of contact) and END (contact ended).
 Unlike triggers, contacts also generate physics responses (impulses).
*>
macro void PhysicsWorld.@get_contact_events(&self; @callback(ContactEvent event))
{
	foreach (event : self.contact_events)
	{
		@callback(event);
	}
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self)
{
	return self.aabb.transform(
		translation: self.collider.translation,
		scale: self.collider.scale,
		rotation: self.collider.rotation
	);
}

fn Matrix3f Rigidbody.get_inverse_world_inertia(self)
{
	// Static/kinematic bodies have infinite inertia — inverse is zero,
	// so angular impulses are no-ops instead of unit-inertia rotations.
	if (self.is_static) return { };
	return self.collider.world_inertia_from_local(self.inertia);
}

fn void Rigidbody.apply_linear_impulse(&self, Vec3 impulse) => self.linear_velocity += impulse * self.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3 impulse)
{
	Matrix3f inertia = self.get_inverse_world_inertia();
	self.angular_velocity += inertia * impulse;
}

fn void Rigidbody.apply_impulse(&self, Vec3 impulse, Vec3 point)
{
	self.linear_velocity += impulse * self.inverse_mass;
	self.apply_angular_impulse(point.cross(impulse));
}

fn Vec3 Rigidbody.relative_velocity(self, Vec3 point) => self.linear_velocity + self.angular_velocity.cross(point);

fn bool Rigidbody.is_rotating(self) => self.angular_velocity.length_sq() > self.sleep_delta;
fn bool Rigidbody.is_moving(self) => self.linear_velocity.length_sq() > self.sleep_delta;
fn bool Rigidbody.is_sleeping(self) => self.sleeping;
fn void Rigidbody.stop_sleeping(&self)
{
	self.sleeping = false;
	self.sleep_timer = 0;
}
fn bool Rigidbody.is_active(self) => self.sleeping == false && self.is_static == false;

// XPBD generalized inverse mass: w = 1/m + (r×n)^T · I^{-1} · (r×n)
fn float compute_generalized_inverse_mass(Rigidbody* body, Vec3 r, Vec3 n)
{
	if (body.is_static) return 0;
	float w = body.inverse_mass;
	Vec3 rn = r.cross(n);
	Vec3 inertia_term = body.get_inverse_world_inertia() * rn;
	w += inertia_term.dot(rn);
	return w;
}

// Apply angular rotation from axis-angle correction vector.
// Builds the quaternion with dmath::sincos instead of
// quaternion::from_axis_angle — libm sin/cos differ across platforms, which
// would break lockstep determinism (see determinism.md).
fn void apply_angular_rotation(Rigidbody* body, Vec3 correction) @local
{
	float angle = correction.length();
	if (angle <= EPSILON) return;
	Vec3 axis = correction / angle;
	float[<2>] sc = dmath::sincos(angle * 0.5f);
	QuaternionValue dq = { ...(axis * sc[0]), sc[1] };
	body.collider.rotation = (dq * body.collider.rotation).normalize();
}

// XPBD positional correction: apply delta_lambda along direction n at contact arm r
fn void apply_positional_correction(Rigidbody* body, Vec3 n, Vec3 r, float delta_lambda)
{
	if (body.is_static) return;
	body.collider.translation += n * (body.inverse_mass * delta_lambda);
	apply_angular_rotation(body, body.get_inverse_world_inertia() * r.cross(n) * delta_lambda);
}

// Solve friction along a single tangent direction
fn void solve_tangent_friction(
	Rigidbody* a,
	Rigidbody* b,
	XpbdConstraint* c,
	Vec3 tangent,
	float* lambda_t,
	float displacement,
	float max_friction
) @local
{
	if (math::abs(displacement) <= EPSILON) return;
	float w = compute_generalized_inverse_mass(a, c.r_a, tangent) + compute_generalized_inverse_mass(b, c.r_b, tangent);
	if (w <= WEIGHT_EPSILON) return;
	float dl = -displacement / w;
	float new_l = math::clamp(*lambda_t + dl, -max_friction, max_friction);
	dl = new_l - *lambda_t;
	*lambda_t = new_l;
	apply_positional_correction(a, tangent, c.r_a, dl);
	apply_positional_correction(b, -tangent, c.r_b, dl);
}

// Union-Find: find root with path compression
fn sz UnionFindMap.find(&self, sz id)
{
	if (!self.has_key(id))
	{
		self.set(id, id);
		return id;
	}

	sz parent = self.get(id)!!;
	if (parent != id)
	{
		// Path compression: point directly to root
		sz root = self.find(parent);
		self.set(id, root);
		return root;
	}
	return id;
}

// Union-Find: merge two bodies into same island (smaller id becomes root)
fn void UnionFindMap.merge(&self, sz a, sz b)
{
	sz root_a = self.find(a);
	sz root_b = self.find(b);

	if (root_a != root_b)
	{
		if (root_a < root_b)
		{
			self.set(root_b, root_a);
		}
		else
		{
			self.set(root_a, root_b);
		}
	}
}

fn sz PhysicsWorld.island_find(&self, sz id) => self.island_parent.find(id);
fn void PhysicsWorld.island_union(&self, sz a, sz b) => self.island_parent.merge(a, b);

// Compute islands from joints - call when islands_dirty is true
fn void PhysicsWorld.compute_islands(&self)
{
	if (!self.islands_dirty) return;

	// Reset parent map
	self.island_parent.clear();

	// Each body starts as its own island
	RigidBodyIterator iter = self.bodies.value_iter();
	while (iter.next())
	{
		Rigidbody* body = iter.get()!!;
		self.island_parent.set(body.id, body.id);
	}

	// Union bodies connected by joints
	foreach (&joint : self.joints)
	{
		// Only union non-static bodies
		Rigidbody*? a = self.find_body(joint.body_a);
		Rigidbody*? b = self.find_body(joint.body_b);

		if (try body_a = a)
		{
			if (try body_b = b)
			{
				if (!body_a.is_static && !body_b.is_static)
				{
					self.island_union(joint.body_a, joint.body_b);
				}
			}
		}
	}

	// Assign island_id to each body
	RigidBodyIterator assign_iter = self.bodies.value_iter();
	while (assign_iter.next())
	{
		Rigidbody* body = assign_iter.get()!!;
		body.island_id = self.island_find(body.id);
	}

	self.islands_dirty = false;
}

// Wake all bodies in the same island - O(n) where n is total bodies
fn void PhysicsWorld.wake_island(&self, Rigidbody* body)
{
	if (!body.sleeping) return;

	sz target_island = body.island_id;

	RigidBodyIterator iter = self.bodies.value_iter();
	while (iter.next())
	{
		Rigidbody* other = iter.get()!!;
		if (other.island_id == target_island && other.sleeping)
		{
			other.stop_sleeping();
		}
	}
}


fn bool Rigidbody.has_mesh_bvh(&self)
{
	if (self.collider.get_type() == ShapeType.CONVEX)
	{
		Mesh* mesh = (Mesh*)self.collider.shape;
		return mesh.has_bvh();
	}
	return false;
}

fn bool Rigidbody.has_compound(&self)
{
	return self.collider.get_type() == ShapeType.COMPOUND;
}

// Worker function for parallel collision testing
fn int collision_test_worker(void* arg) @local
{
	CollisionTask* task = (CollisionTask*)arg;
	PhysicsWorld* world = task.world;

	for (sz i = 0; i < task.count; i++)
	{
		Pair pair = task.pairs[i];
		Rigidbody* body_a = world.find_body(pair.a)!!;
		Rigidbody* body_b = world.find_body(pair.b)!!;

		CollisionInfo info = collision::check_convex_collision(&body_a.collider, &body_b.collider, epa: true);

		if (info.collided)
		{
			task.results[i * MAX_CONTACTS_PER_PAIR] = {
				.a = pair.a,
				.b = pair.b,
				.normal = info.normal,
				.contact = info.contact_points,
				.depth = info.depth,
			};
			task.counts[i] = 1;
		}
	}
	return 0;
}

// Worker function for parallel BVH collision testing
fn int bvh_collision_test_worker(void* arg) @local
{
	CollisionTask* task = (CollisionTask*)arg;
	PhysicsWorld* world = task.world;

	for (sz i = 0; i < task.count; i++)
	{
		Pair pair = task.pairs[i];
		Rigidbody* body_a = world.find_body(pair.a)!!;
		Rigidbody* body_b = world.find_body(pair.b)!!;

		Collision[] slots = task.results[i * MAX_CONTACTS_PER_PAIR : MAX_CONTACTS_PER_PAIR];
		task.counts[i] = world.test_bvh_collision(body_a, body_b, pair, slots);
	}
	return 0;
}

// XPBD predict: save prev state, apply gravity, advance position and rotation
fn void predict_bodies(Rigidbody*[] bodies, Vec3 gravity, float dt) @local
{
	foreach (body : bodies)
	{
		// Save previous state
		body.prev_position = body.collider.translation;
		body.prev_rotation = body.collider.rotation;

		// Apply gravity to velocity
		body.linear_velocity += gravity * body.gravity_factor * dt;

		// Advance position using current velocity
		body.collider.translation += body.linear_velocity * dt;

		// Advance rotation using angular velocity
		Vec3 axis = body.angular_velocity * (dt * 0.5f);
		if (axis != { 0, 0, 0 })
		{
			body.collider.rotation = (body.collider.rotation
				+ (QuaternionValue) { ... axis, 0 } * body.collider.rotation).normalize();
		}
	}
}

// XPBD derive: recover velocities from position/rotation deltas, apply dampening
fn void derive_bodies(Rigidbody*[] bodies, float dt) @local
{
	float inv_dt = 1.0f / dt;

	foreach (body : bodies)
	{
		// Derive linear velocity from position change
		body.linear_velocity = (body.collider.translation - body.prev_position) * inv_dt;

		// Derive angular velocity: omega = 2 * (q * q_prev_conjugate).xyz / dt
		QuaternionValue dq = body.collider.rotation * body.prev_rotation.conjugate();
		// Ensure shortest path
		if (dq.l < 0) dq = -dq;
		body.angular_velocity = dq.v.xyz * (2.0f * inv_dt);

		// Apply per-body dampening (dmath::pow: libm pow is not
		// cross-platform deterministic)
		body.angular_velocity *= dmath::pow(body.angular_dampening, dt);
		body.linear_velocity *= dmath::pow(body.linear_dampening, dt);
	}
}

// Worker function for parallel XPBD predict phase
fn int predict_worker(void* arg) @local
{
	XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
	predict_bodies(task.bodies[:task.count], task.world.gravity, task.time);
	return 0;
}

// Worker function for parallel XPBD derive phase
fn int derive_worker(void* arg) @local
{
	XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
	derive_bodies(task.bodies[:task.count], task.time);
	return 0;
}

// Worker function for island-parallel XPBD constraint solving
fn int island_solve_worker(void* arg) @local
{
	IslandSolveTask* task = (IslandSolveTask*)arg;

	for (uint iter = 0; iter < task.iterations; iter++)
	{
		for (sz i = 0; i < task.count; i++)
		{
			solve_xpbd_constraint(task.constraints[i], task.world, iter, task.iterations);
		}
	}
	return 0;
}

// XPBD constraint solver — position-level corrections
fn void solve_xpbd_constraint(XpbdConstraint* c, PhysicsWorld* world, uint iteration, uint total_iterations) @local
{
	Rigidbody* a = c.body_a;
	Rigidbody* b = c.body_b;

	switch (c.type)
	{
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
			if (w_sum < WEIGHT_EPSILON) return;

			float alpha = c.compliance;
			// XPBD: C = -penetration (non-penetration constraint: C >= 0)
			// delta_lambda = -(-pen + alpha * lambda) / (w + alpha) = (pen - alpha*lambda) / (w+alpha)
			float delta_lambda = (penetration - alpha * c.lambda) / (w_sum + alpha);

			// Clamp: lambda must be >= 0 (contact can only push, not pull)
			float new_lambda = c.lambda + delta_lambda;
			if (new_lambda < 0)
			{
				delta_lambda = -c.lambda;
				new_lambda = 0;
			}
			c.lambda = new_lambda;
			c.lambda_normal = new_lambda;

			// Apply positional corrections: push a along -normal, push b along +normal
			apply_positional_correction(a, -c.normal, c.r_a, delta_lambda);
			apply_positional_correction(b, c.normal, c.r_b, delta_lambda);

			// Friction: tangential position correction
			if (c.friction > 0 && math::abs(c.lambda_normal) > EPSILON)
			{
				Vec3 prev_r_a = a.prev_rotation * c.r_a_local;
				Vec3 prev_r_b = b.prev_rotation * c.r_b_local;
				Vec3 disp_a = (a.collider.translation + cur_r_a) - (a.prev_position + prev_r_a);
				Vec3 disp_b = (b.collider.translation + cur_r_b) - (b.prev_position + prev_r_b);
				Vec3 rel_disp = disp_a - disp_b;
				float max_friction = c.friction * math::abs(c.lambda_normal);

				solve_tangent_friction(a, b, c, c.tangent1, &c.lambda_tangent1, rel_disp.dot(c.tangent1), max_friction);
				solve_tangent_friction(a, b, c, c.tangent2, &c.lambda_tangent2, rel_disp.dot(c.tangent2), max_friction);
			}

		case JOINT_POSITION:
			// Recompute anchor arms in world space (r_a/r_b store local-space anchors for joints)
			Vec3 joint_r_a = a.collider.rotation * c.r_a;
			Vec3 joint_r_b = b.collider.rotation * c.r_b;

			Vec3 world_a = a.collider.translation + joint_r_a;
			Vec3 world_b = b.collider.translation + joint_r_b;
			float pos_error = (world_b - world_a).dot(c.normal);

			float joint_weight_a = compute_generalized_inverse_mass(a, joint_r_a, c.normal);
			float joint_weight_b = compute_generalized_inverse_mass(b, joint_r_b, c.normal);
			float joint_weight_sum = joint_weight_a + joint_weight_b;
			if (joint_weight_sum < WEIGHT_EPSILON) return;

			// Progressive stiffness: ramp compliance from high (soft) to target over iterations
			// This lets weak forces propagate before stiff forces dominate
			float joint_compliance = c.compliance;
			if (joint_compliance > 0 && total_iterations > 1)
			{
				float progress = (float)(iteration + 1) / (float)total_iterations;
				// Start at 4x compliance (1/4 stiffness), ramp to target
				joint_compliance = c.compliance * (1.0f + 3.0f * (1.0f - progress));
			}
			float joint_delta_lambda = -(pos_error + joint_compliance * c.lambda)
				/ (joint_weight_sum + joint_compliance);
			c.lambda += joint_delta_lambda;

			apply_positional_correction(a, -c.normal, joint_r_a, joint_delta_lambda);
			apply_positional_correction(b, c.normal, joint_r_b, joint_delta_lambda);

		case JOINT_ANGULAR:
			// Angular constraint: correct orientation difference along axis
			// Recompute world-space axis from local-space axis (stored in r_a)
			Vec3 axis = a.collider.rotation * c.r_a;

			// Recompute angular error from current orientations
			QuaternionValue err_q = a.collider.rotation.conjugate() * b.collider.rotation
				* c.joint_orientation.conjugate();
			float err_dir = err_q.l >= 0 ? 1.0f : -1.0f;
			Vec3 err_local = err_q.v.xyz * 2 * err_dir;
			Vec3 world_err = a.collider.rotation * err_local;
			float ang_error = world_err.dot(axis) - c.c_value;

			// Angular generalized inverse mass (no lever arm)
			float angular_weight_a = 0;
			float angular_weight_b = 0;
			if (!a.is_static)
			{
				Vec3 a_accel = a.get_inverse_world_inertia() * axis;
				angular_weight_a = a_accel.dot(axis);
			}
			if (!b.is_static)
			{
				Vec3 b_accel = b.get_inverse_world_inertia() * axis;
				angular_weight_b = b_accel.dot(axis);
			}
			float angular_weight_sum = angular_weight_a + angular_weight_b;
			if (angular_weight_sum < WEIGHT_EPSILON) return;

			// Progressive stiffness: ramp compliance from high (soft) to target over iterations
			float angular_compliance = c.compliance;
			if (angular_compliance > 0 && total_iterations > 1)
			{
				float progress = (float)(iteration + 1) / (float)total_iterations;
				angular_compliance = c.compliance * (1.0f + 3.0f * (1.0f - progress));
			}
			float angular_delta_lambda = -(ang_error + angular_compliance * c.lambda)
				/ (angular_weight_sum + angular_compliance);
			c.lambda += angular_delta_lambda;

			// Apply angular corrections
			if (!a.is_static) apply_angular_rotation(a, a.get_inverse_world_inertia() * axis * (-angular_delta_lambda));
			if (!b.is_static) apply_angular_rotation(b, b.get_inverse_world_inertia() * axis * angular_delta_lambda);
	}
}


fn bool is_manifold_shape(TransformedShape* collider) @local @inline
{
	ShapeType t = collider.get_type();
	if (t == ShapeType.BOX) return true;
	if (t == ShapeType.CONVEX && &collider.shape.vertex_count) return collider.shape.vertex_count() < 24;
	return false;
}

// Expand a single EPA contact into a multi-contact manifold.
// For flat-on-flat contacts (box on floor), projects vertices of the incident body
// onto the reference body's contact plane and creates additional contacts.
fn void PhysicsWorld.generate_manifold(&self, Collision* collision)
{
	// Secondary contacts of a mesh/compound manifold are already spread out
	if (collision.skip_expand)
	{
		self.collisions.push(*collision);
		return;
	}

	Rigidbody* a = self.find_body(collision.a)!!;
	Rigidbody* b = self.find_body(collision.b)!!;

	if (!is_manifold_shape(&a.collider) && !is_manifold_shape(&b.collider))
	{
		self.collisions.push(*collision);
		return;
	}

	// Incident body: must be a dynamic manifold shape (never expand static ground)
	bool incident_is_a;
	bool a_candidate = is_manifold_shape(&a.collider) && !a.is_static;
	bool b_candidate = is_manifold_shape(&b.collider) && !b.is_static;

	if (!a_candidate && !b_candidate)
	{
		self.collisions.push(*collision);
		return;
	}

	if (a_candidate && b_candidate)
	{
		Vec3 ext_a = a.transformed_aabb.size();
		Vec3 ext_b = b.transformed_aabb.size();
		float sa_a = ext_a.x * ext_a.y + ext_a.y * ext_a.z + ext_a.z * ext_a.x;
		float sa_b = ext_b.x * ext_b.y + ext_b.y * ext_b.z + ext_b.z * ext_b.x;
		incident_is_a = sa_a <= sa_b;
	}
	else
	{
		incident_is_a = a_candidate;
	}

	Rigidbody* incident = incident_is_a ? a : b;
	ShapeType incident_type = incident.collider.get_type();

	Vec3[8] box_corners;
	Vec3[] local_verts;

	if (incident_type == ShapeType.BOX)
	{
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
	}
	else
	{
		Mesh* mesh = (Mesh*)incident.collider.shape;
		local_verts = mesh.vertices.entries[:mesh.vertices.size];
	}

	Vec3 plane_point = incident_is_a ? collision.contact[1] : collision.contact[0];
	Vec3 plane_normal = incident_is_a ? -collision.normal : collision.normal;
	Vec3 orig_contact = incident_is_a ? collision.contact[0] : collision.contact[1];

	float depth_tol = collision.depth + MANIFOLD_DEPTH_TOL;
	Vec3 inc_ext = incident.transformed_aabb.size();
	float max_spread_sq = inc_ext.length_sq();

	// Always include original EPA contact
	self.collisions.push(*collision);

	foreach (i, local_v : local_verts)
	{
		Vec3 v = incident.collider.rotation * (local_v * incident.collider.scale) + incident.collider.translation;

		float dist_to_orig = v.distance_sq(orig_contact);
		if (dist_to_orig < DEDUP_DIST_SQ || dist_to_orig > max_spread_sq) continue;

		float signed_dist = (v - plane_point).dot(plane_normal);
		if (signed_dist >= depth_tol) continue;

		float depth = math::max(0.0f, -signed_dist);
		Vec3 projected = v - plane_normal * signed_dist;

		Collision new_c = { .a = collision.a, .b = collision.b, .normal = collision.normal, .depth = depth, };

		if (incident_is_a)
		{
			new_c.contact = { v, projected };
		}
		else
		{
			new_c.contact = { projected, v };
		}

		self.collisions.push(new_c);
	}
}

fn void PhysicsWorld.dispatch_collision_tasks(&self, Pair[] pairs, ThreadFn worker_fn)
{
	sz count = pairs.len;
	if (count == 0) return;

	self.collision_results.clear();
	self.collision_counts.clear();
	for (sz i = 0; i < count; i++)
	{
		for (sz j = 0; j < MAX_CONTACTS_PER_PAIR; j++) self.collision_results.push({ });
		self.collision_counts.push(0);
	}

	Collision* results = self.collision_results.array_view().ptr;
	sz* counts = self.collision_counts.array_view().ptr;

	// Batch pairs into at most one task per worker
	self.collision_tasks.clear();
	sz workers = self.thread_pool.workers();
	sz batch_size = (count + workers - 1) / workers;

	for (sz start = 0; start < count; start += batch_size)
	{
		sz batch_count = math::min(batch_size, count - start);
		self.collision_tasks.push(
			{
				.world = self,
				.pairs = pairs.ptr + start,
				.results = results + start * MAX_CONTACTS_PER_PAIR,
				.counts = counts + start,
				.count = batch_count,
			}
		);
	}

	if (self.collision_tasks.len() == 1)
	{
		// Single batch — run inline, skip thread pool overhead
		worker_fn(&self.collision_tasks[0]);
	}
	else
	{
		for (sz i = 0; i < self.collision_tasks.len(); i++)
		{
			self.thread_pool.push(worker_fn, &self.collision_tasks[i]);
		}
		self.thread_pool.join();
	}

	for (sz i = 0; i < count; i++)
	{
		for (sz j = 0; j < self.collision_counts[i]; j++)
		{
			self.collisions.push(self.collision_results[i * MAX_CONTACTS_PER_PAIR + j]);
		}
	}
}

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 1, uint sub_steps = 1)
{
	// Lazy init thread pool (can't do in create_world because workers get pointer before move)
	if (!self.thread_pool.initialized)
	{
		sz workers = self.thread_count > 0 ? self.thread_count : default_worker_count();
		self.thread_pool.init(workers)!!;
	}

	self.trigger_events.clear();
	self.contact_events.clear();

	float sub_time = time / (float)sub_steps;
	for (uint i = 0; i < sub_steps; i++)
	{
		self.run_substep(sub_time, step_count);
	}
}

// Canonical orderings for lockstep determinism (see determinism.md): the
// Gauss-Seidel solver is order-dependent, so candidate pairs and the events
// handed to game logic are sorted by body id instead of inheriting hash-map
// iteration order.
fn int pair_id_cmp(Pair x, Pair y) @private
{
	if (x.a != y.a) return x.a < y.a ? -1 : 1;
	if (x.b != y.b) return x.b < y.b ? -1 : 1;
	return 0;
}

fn int trigger_event_cmp(TriggerEvent x, TriggerEvent y) @local
{
	if (x.trigger_id != y.trigger_id) return x.trigger_id < y.trigger_id ? -1 : 1;
	if (x.other_id != y.other_id) return x.other_id < y.other_id ? -1 : 1;
	if (x.type != y.type) return x.type.ordinal < y.type.ordinal ? -1 : 1;
	return 0;
}

fn int contact_event_cmp(ContactEvent x, ContactEvent y) @local
{
	if (x.body_a != y.body_a) return x.body_a < y.body_a ? -1 : 1;
	if (x.body_b != y.body_b) return x.body_b < y.body_b ? -1 : 1;
	if (x.type != y.type) return x.type.ordinal < y.type.ordinal ? -1 : 1;
	return 0;
}

fn void PhysicsWorld.run_substep(&self, float time, uint step_count)
{
	if (time <= 0.0f) return;

	// Events accumulate across substeps; remember where this substep's
	// slice starts so it can be canonically sorted below.
	sz trigger_events_start = self.trigger_events.len();
	sz contact_events_start = self.contact_events.len();

	// Recompute islands if joint topology changed
	self.compute_islands();

	// === Phase 1: Predict — save prev state, apply gravity, advance positions ===
	self.xpbd_predict(time);

	// === Phase 2: Broad-phase — collect candidate pairs ===
	List {Pair} candidate_pairs;
	TriggerOverlapSet current_triggers; // Triggers overlapping this frame
	defer candidate_pairs.free();
	defer current_triggers.free();

	self.spatial_map.@get_pairs(; Pair pair)
	{
		Rigidbody* body_a = self.find_body(pair.a)!!;
		Rigidbody* body_b = self.find_body(pair.b)!!;

		bool has_filter = self.filters.has_key({ body_a.id, body_b.id });

		if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb))
		{
			// Check if either body is a trigger
			if (body_a.is_trigger || body_b.is_trigger)
			{
				// Normalize pair so trigger is always first
				Pair trigger_pair = body_a.is_trigger ? pair : { pair.b, pair.a };
				current_triggers.set(trigger_pair, true);

				// Generate ENTER event if this is a new overlap
				if (!self.active_triggers.has_key(trigger_pair))
				{
					self.trigger_events.push(
						{ .trigger_id = trigger_pair.a, .other_id = trigger_pair.b, .type = TriggerEventType.ENTER }
					);
				}
			}
			else
			{
				// Regular collision pair
				// Kinematic bodies are always "active" for broad phase (externally driven)
				bool a_active = body_a.is_active() || body_a.is_kinematic;
				bool b_active = body_b.is_active() || body_b.is_kinematic;

				// Wake sleeping bodies (must be sequential due to island wake)
				if (body_a.sleeping && b_active) self.wake_island(body_a);
				if (body_b.sleeping && a_active) self.wake_island(body_b);

				if (a_active || b_active)
				{
					candidate_pairs.push(pair);
				}
			}
		}
	};

	// Generate EXIT events for triggers that are no longer overlapping
	self.active_triggers.@each(; Pair old_pair, bool active)
	{
		if (!current_triggers.has_key(old_pair))
		{
			self.trigger_events.push(
				{ .trigger_id = old_pair.a, .other_id = old_pair.b, .type = TriggerEventType.EXIT }
			);
		}
	};

	// Swap active triggers with current frame's triggers
	self.active_triggers.free();
	self.active_triggers = current_triggers;
	current_triggers = { }; // Prevent double-free in defer

	// Canonical order: pair order decides constraint order, and the
	// Gauss-Seidel solve is order-dependent
	if (candidate_pairs.len() > 1)
	{
		sort::quicksort(candidate_pairs.array_view(), &pair_id_cmp);
	}
	if (self.trigger_events.len() - trigger_events_start > 1)
	{
		sort::quicksort(self.trigger_events.array_view()[trigger_events_start..], &trigger_event_cmp);
	}

	// === Phase 3: Narrow-phase collision testing ===
	// Convex and BVH collisions run in parallel via thread pool
	sz num_pairs = candidate_pairs.len();

	if (num_pairs > 0)
	{
		// Separate pairs by collision type
		List {Pair} convex_pairs;
		List {Pair} bvh_pairs;
		List {Pair} compound_pairs;
		defer convex_pairs.free();
		defer bvh_pairs.free();
		defer compound_pairs.free();

		for (sz i = 0; i < num_pairs; i++)
		{
			Pair pair = candidate_pairs[i];
			Rigidbody* body_a = self.find_body(pair.a)!!;
			Rigidbody* body_b = self.find_body(pair.b)!!;

			if (body_a.has_mesh_bvh() || body_b.has_mesh_bvh())
			{
				bvh_pairs.push(pair);
			}
			else if (body_a.has_compound() || body_b.has_compound())
			{
				compound_pairs.push(pair);
			}
			else
			{
				convex_pairs.push(pair);
			}
		}

		// Process convex pairs in parallel
		self.dispatch_collision_tasks(convex_pairs.array_view(), &collision_test_worker);

		// Process BVH pairs in parallel
		self.dispatch_collision_tasks(bvh_pairs.array_view(), &bvh_collision_test_worker);

		foreach (pair : compound_pairs)
		{
			Rigidbody* body_a = self.find_body(pair.a)!!;
			Rigidbody* body_b = self.find_body(pair.b)!!;
			self.check_coacd_collision(body_a, body_b, pair);
		}
	}

	// === Phase 4: Generate contact events ===
	ContactOverlapSet current_contacts;
	defer current_contacts.free();

	// Track all collisions this frame and generate START events
	foreach (&collision : self.collisions)
	{
		Pair contact_pair = { collision.a, collision.b };
		current_contacts.set(contact_pair, true);

		// Generate START event if this is a new contact
		if (!self.active_contacts.has_key(contact_pair))
		{
			self.contact_events.push(
				{
					.body_a = collision.a,
					.body_b = collision.b,
					.type = ContactEventType.START,
					.normal = collision.normal,
					.contact_point = collision.contact[0]
				}
			);
		}
	}

	// Generate END events for contacts that ended
	self.active_contacts.@each(; Pair old_pair, bool active)
	{
		if (!current_contacts.has_key(old_pair))
		{
			self.contact_events.push(
				{
					.body_a = old_pair.a,
					.body_b = old_pair.b,
					.type = ContactEventType.END,
					.normal = { 0, 0, 0 },
					.contact_point = { 0, 0, 0 }
				}
			);
		}
	};

	// Swap active contacts with current frame's contacts
	self.active_contacts.free();
	self.active_contacts = current_contacts;
	current_contacts = { }; // Prevent double-free in defer

	// Canonical event order (END events come from hash-map iteration)
	if (self.contact_events.len() - contact_events_start > 1)
	{
		sort::quicksort(self.contact_events.array_view()[contact_events_start..], &contact_event_cmp);
	}

	// === Phase 4.5: Expand contact manifolds ===
	// EPA gives one contact per pair. For box-on-box or box-on-floor contacts,
	// expand into multiple contacts by projecting vertices near the contact plane.
	if (self.collisions.len() > 0)
	{
		@swap(self.collisions, self.manifold_scratch);
		self.collisions.clear();
		foreach (&collision : self.manifold_scratch)
		{
			self.generate_manifold(collision);
		}
		self.manifold_scratch.clear();
	}

	// === Phase 5: Create XPBD constraints (from collisions + joints) ===
	foreach (&collision : self.collisions)
	{
		collision.create_xpbd_constraint(self, time);
	}

	foreach (&joint : self.joints)
	{
		joint.create_xpbd_constraint(self, time);
	}

	// === Phase 6: Solve XPBD constraints — island-parallel ===
	self.xpbd_solve_constraints(step_count);

	self.collisions.clear();

	// === Phase 7: Soft body solve ===
	self.solve_soft_bodies(time);

	// === Phase 8: Derive velocities, restitution, update spatial map ===
	self.xpbd_derive_and_update(time);

	// === Phase 9: Sleep management ===
	self.update_sleep_state();
}

// Run an XPBD phase worker over the active bodies: batched onto the thread
// pool when there are enough bodies, otherwise inline as a single batch.
fn void PhysicsWorld.dispatch_phase_tasks(&self, float dt, ThreadFn worker_fn)
{
	sz num_active = self.active_bodies.len();
	Rigidbody** all_bodies_ptr = self.active_bodies.array_view().ptr;
	sz workers = self.thread_pool.workers();

	if (num_active < workers * 2)
	{
		XpbdPhaseTask task = { .world = self, .bodies = all_bodies_ptr, .count = num_active, .time = dt, };
		worker_fn(&task);
		return;
	}

	sz batch_size = (num_active + workers - 1) / workers;
	sz num_tasks = (num_active + batch_size - 1) / batch_size;
	self.phase_tasks.clear();

	for (sz i = 0; i < num_tasks; i++)
	{
		sz start = i * batch_size;
		sz count = math::min(batch_size, num_active - start);
		self.phase_tasks.push({ .world = self, .bodies = all_bodies_ptr + start, .count = count, .time = dt, });
	}

	for (sz i = 0; i < num_tasks; i++)
	{
		self.thread_pool.push(worker_fn, &self.phase_tasks[i]);
	}
	self.thread_pool.join();
}

<*
 Swept CCD against static geometry. A body whose predicted displacement this
 substep exceeds its own ccd_radius ray-casts its motion (center segment from
 prev_position to the predicted position) against static bodies — exact BVH
 triangle casts for meshes, world-AABB casts for other shapes — and is clamped
 just short of the first hit. The narrow phase then sees a regular overlap
 instead of the body skipping through, and the derive phase absorbs the lost
 velocity automatically.

 Limitations: rotation during the sweep is ignored, dynamic-vs-dynamic pairs
 are not swept, and kinematic movers are treated as stationary obstacles.
*>
fn void PhysicsWorld.apply_ccd(&self)
{
	foreach (body : self.active_bodies)
	{
		float radius = body.ccd_radius;
		if (radius <= 0) continue;

		Vec3 disp = body.collider.translation - body.prev_position;
		float disp_sq = disp.length_sq();
		if (disp_sq <= radius * radius) continue;

		// All hit fractions are along the motion segment: 0 = prev, 1 = predicted
		float best_fraction = float::max;

		RigidBodyIterator iter = self.bodies.value_iter();
		while (iter.next())
		{
			Rigidbody* obstacle = iter.get()!!;
			if (!obstacle.is_static || obstacle.is_trigger || obstacle.id == body.id) continue;

			if (obstacle.has_mesh_bvh())
			{
				// Cast in mesh-local space; the affine transform preserves
				// fractions along the segment.
				Mesh* mesh = (Mesh*)obstacle.collider.shape;
				Vec3 obstacle_scale = obstacle.collider.scale;
				Quaternionf inv_rot = obstacle.collider.rotation.conjugate();
				Vec3 inv_scale = { 1.0f / obstacle_scale.x, 1.0f / obstacle_scale.y, 1.0f / obstacle_scale.z };
				Vec3 local_start = (inv_rot * (body.prev_position - obstacle.collider.translation)) * inv_scale;
				Vec3 local_end = (inv_rot * (body.collider.translation - obstacle.collider.translation)) * inv_scale;

				HitInfo hit = mesh.bvh.ray_intersects({ .origin = local_start, .direction = local_end - local_start });
				if (hit.collided && hit.depth > 0 && hit.depth <= 1.0f && hit.depth < best_fraction)
				{
					best_fraction = hit.depth;
				}
			}
			else
			{
				// Conservative: cast against the obstacle's world AABB.
				// depth == 0 means we started inside — that's a resting
				// overlap for the narrow phase, not a CCD hit.
				HitInfo hit = obstacle.transformed_aabb.intersect_ray({ .origin = body.prev_position, .direction = disp });
				if (hit.collided && hit.depth > 0 && hit.depth <= 1.0f && hit.depth < best_fraction)
				{
					best_fraction = hit.depth;
				}
			}
		}

		if (best_fraction < float::max)
		{
			// Stop just short of the impact point so the next narrow phase
			// generates a regular contact at the surface
			float back_off = radius * 0.99f / math::sqrt(disp_sq);
			float clamped = math::max(0.0f, best_fraction - back_off);
			body.collider.translation = body.prev_position + disp * clamped;
		}
	}
}

// XPBD predict phase: save prev state, apply gravity, advance positions
fn void PhysicsWorld.xpbd_predict(&self, float dt)
{
	self.active_bodies.clear();

	RigidBodyIterator collect_iter = self.bodies.value_iter();
	while (collect_iter.next())
	{
		Rigidbody* body = collect_iter.get()!!;
		if (body.is_active())
		{
			self.active_bodies.push(body);
		}
	}

	if (self.active_bodies.len() == 0) return;

	self.dispatch_phase_tasks(dt, &predict_worker);

	// Clamp fast movers to their first time of impact before the broad phase
	if (self.ccd_enabled) self.apply_ccd();

	// Sequential: update spatial map AABBs after predict (must happen before broad phase)
	foreach (body : self.active_bodies)
	{
		Aabb3 new_aabb = body.get_transformed_aabb();
		self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
		body.transformed_aabb = new_aabb;
	}
}

// XPBD derive phase: compute velocities from position deltas, restitution, update spatial map
fn void PhysicsWorld.xpbd_derive_and_update(&self, float dt)
{
	if (self.active_bodies.len() == 0) return;

	// Derive velocities (parallelizable)
	self.dispatch_phase_tasks(dt, &derive_worker);

	// Post-derive: velocity correction at contacts
	// For zero restitution: target_vn = 0 (inelastic, no bounce)
	// For restitution > 0: target_vn = -e * vn_pre (bounce proportional to impact speed)
	foreach (&c : self.constraints)
	{
		if (c.type != CONTACT_NORMAL) continue;
		if (math::abs(c.lambda_normal) < EPSILON) continue;

		Rigidbody* a = c.body_a;
		Rigidbody* b = c.body_b;

		// Current relative velocity at contact (after derive)
		Vec3 va = a.relative_velocity(c.r_a);
		Vec3 vb = b.relative_velocity(c.r_b);
		float vn = (va - vb).dot(c.normal);

		// Target normal velocity
		float target_vn = 0;
		float vn_pre = c.pre_solve_relative_vel_n;
		if (c.restitution > 0.01f && math::abs(vn_pre) > BOUNCE_THRESHOLD)
		{
			target_vn = -c.restitution * vn_pre;
		}

		float dv = target_vn - vn;

		// Apply velocity correction to reach target (handles both residual approach and overshoot)
		if (math::abs(dv) > EPSILON)
		{
			float w_a = compute_generalized_inverse_mass(a, c.r_a, c.normal);
			float w_b = compute_generalized_inverse_mass(b, c.r_b, c.normal);
			float w_sum = w_a + w_b;
			if (w_sum > WEIGHT_EPSILON)
			{
				float impulse_mag = dv / w_sum;
				if (!a.is_static)
				{
					a.apply_impulse(c.normal * impulse_mag, c.r_a);
				}
				if (!b.is_static)
				{
					b.apply_impulse(-c.normal * impulse_mag, c.r_b);
				}
			}
		}
	}

	self.constraints.clear();

	// Sequential: update spatial map and sleep timers
	foreach (body : self.active_bodies)
	{
		bool rotating = body.is_rotating();
		bool moving = body.is_moving();

		if (rotating || moving)
		{
			Aabb3 new_aabb = body.get_transformed_aabb();
			self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
			body.transformed_aabb = new_aabb;
		}

		if (body.can_sleep && !moving && !rotating)
		{
			body.sleep_timer += dt;
		}
		else
		{
			body.sleep_timer = 0;
		}
	}
}


// Island-parallel XPBD constraint solving
fn void PhysicsWorld.xpbd_solve_constraints(&self, uint step_count)
{
	sz constraint_count = self.constraints.len();
	if (constraint_count == 0) return;

	// Tiny workloads aren't worth the grouping and thread pool overhead
	if (constraint_count <= 16)
	{
		for (uint i = 0; i < step_count; i++)
		{
			foreach (&constraint : self.constraints)
			{
				solve_xpbd_constraint(constraint, self, i, step_count);
			}
		}
		return;
	}

	// Islands must reflect *this substep's* constraint graph: two constraints
	// sharing a dynamic body have to solve on the same thread. Static bodies
	// are never written by the solver, so they don't merge islands.
	UnionFindMap solve_islands;
	defer solve_islands.free();

	foreach (&constraint : self.constraints)
	{
		if (!constraint.body_a.is_static && !constraint.body_b.is_static)
		{
			solve_islands.merge(constraint.body_a.id, constraint.body_b.id);
		}
	}

	// Pack islands into at most one bucket per worker. Islands touch disjoint
	// bodies, so one Gauss-Seidel sweep over a bucket's combined constraint
	// list is equivalent to solving its islands independently, and bounding
	// the task count keeps thread pool overhead out of the hot path.
	sz workers = self.thread_pool.workers();
	ConstraintList[MAX_WORKER_THREADS] bucket_storage;
	ConstraintList[] buckets = bucket_storage[:workers];
	defer foreach (&bucket : buckets) bucket.free();

	HashMap {sz, sz} island_bucket; // island root -> bucket index
	defer island_bucket.free();

	foreach (&constraint : self.constraints)
	{
		Rigidbody* anchor = !constraint.body_a.is_static ? constraint.body_a : constraint.body_b;
		sz root = solve_islands.find(anchor.id);

		sz bucket_index;
		if (try existing = island_bucket.get(root))
		{
			bucket_index = existing;
		}
		else
		{
			// New island: assign to the currently smallest bucket
			bucket_index = 0;
			for (sz i = 1; i < workers; i++)
			{
				if (buckets[i].len() < buckets[bucket_index].len()) bucket_index = i;
			}
			island_bucket.set(root, bucket_index);
		}
		buckets[bucket_index].push(constraint);
	}

	self.island_tasks.clear();

	foreach (&bucket : buckets)
	{
		if (bucket.len() == 0) continue;
		XpbdConstraint*[] arr = bucket.array_view();
		self.island_tasks.push({ .world = self, .constraints = arr.ptr, .count = arr.len, .iterations = step_count, });
	}

	if (self.island_tasks.len() == 1)
	{
		// Everything landed in one bucket — solve inline
		island_solve_worker(&self.island_tasks[0]);
		return;
	}

	for (sz i = 0; i < self.island_tasks.len(); i++)
	{
		self.thread_pool.push(&island_solve_worker, &self.island_tasks[i]);
	}

	self.thread_pool.join();
}

// Worker function for parallel soft body XPBD solving
fn int softbody_solve_worker(void* arg) @local
{
	SoftBodySolveTask* task = (SoftBodySolveTask*)arg;
	SoftBody* sb = task.body;

	// Generate ground plane collisions
	if (task.ground_enabled)
	{
		sb.generate_ground_collisions(task.ground_z, task.ground_friction);
	}

	// Run XPBD solver (no rigidbody collisions in parallel — not thread-safe)
	sb.xpbd_solve(task.gravity, task.time, task.world);

	return 0;
}

// XPBD soft body solving phase
fn void PhysicsWorld.solve_soft_bodies(&self, float time)
{
	if (self.soft_bodies.len() == 0) return;

	// Sequential: generate rigidbody collision constraints (needs shared body access)
	foreach (&sb : self.soft_bodies)
	{
		if (sb.sleeping) continue;

		RigidBodyIterator rb_iter = self.bodies.value_iter();
		while (rb_iter.next())
		{
			Rigidbody* body = rb_iter.get()!!;
			sb.generate_rigidbody_collisions(body);
		}
	}

	// Partition: soft bodies coupled to rigidbodies mutate those bodies while
	// solving, so they must run sequentially; uncoupled ones solve in parallel.
	List {SoftBodySolveTask} parallel_tasks;
	List {SoftBody*} coupled;
	defer parallel_tasks.free();
	defer coupled.free();

	foreach (&sb : self.soft_bodies)
	{
		if (sb.sleeping) continue;

		bool is_coupled = false;
		foreach (&cc : sb.collision_constraints)
		{
			if (cc.rigidbody_id != 0)
			{
				is_coupled = true;
				break;
			}
		}

		if (is_coupled)
		{
			coupled.push(sb);
			continue;
		}

		parallel_tasks.push(
			{
				.body = sb,
				.world = self,
				.gravity = self.gravity,
				.time = time,
				.ground_z = self.ground_plane_z,
				.ground_enabled = self.ground_plane_enabled,
				.ground_friction = sb.material.static_friction,
			}
		);
	}

	switch (parallel_tasks.len())
	{
		case 0:
			break;
		case 1:
			// Single soft body — solve inline, skip thread pool overhead
			softbody_solve_worker(&parallel_tasks[0]);
		default:
			for (sz i = 0; i < parallel_tasks.len(); i++)
			{
				self.thread_pool.push(&softbody_solve_worker, &parallel_tasks[i]);
			}
			self.thread_pool.join();
	}

	foreach (sb : coupled)
	{
		if (self.ground_plane_enabled)
		{
			sb.generate_ground_collisions(self.ground_plane_z, sb.material.static_friction);
		}
		sb.xpbd_solve(self.gravity, time, self);
	}

	// Sequential: manage soft body sleep
	foreach (&sb : self.soft_bodies)
	{
		if (sb.sleeping) continue;

		// Sleep management
		if (sb.can_sleep)
		{
			if (!sb.is_moving())
			{
				sb.sleep_timer += time;
				if (sb.sleep_timer >= self.sleep_timer)
				{
					sb.sleeping = true;
				}
			}
			else
			{
				sb.sleep_timer = 0;
			}
		}
	}
}


// Shared sleep state update logic
fn void PhysicsWorld.update_sleep_state(&self)
{
	HashMap {sz, bool} island_can_sleep;
	HashMap {sz, bool} island_checked;
	defer island_can_sleep.free();
	defer island_checked.free();

	// First pass: determine which islands can sleep
	RigidBodyIterator sleep_check = self.bodies.value_iter();
	while (sleep_check.next())
	{
		Rigidbody* body = sleep_check.get()!!;

		if (body.is_static) continue;

		sz island = body.island_id;

		if (!island_checked.has_key(island))
		{
			island_checked.set(island, true);
			island_can_sleep.set(island, true);
		}

		if (!body.can_sleep || body.is_moving() || body.is_rotating() || body.sleep_timer < self.sleep_timer)
		{
			island_can_sleep.set(island, false);
		}
	}

	// Second pass: put entire islands to sleep
	RigidBodyIterator sleep_apply = self.bodies.value_iter();
	while (sleep_apply.next())
	{
		Rigidbody* body = sleep_apply.get()!!;

		if (body.is_static || body.sleeping) continue;

		if (try can_sleep = island_can_sleep.get(body.island_id))
		{
			if (can_sleep)
			{
				body.sleeping = true;
				body.sleep_timer = 0;
			}
		}
	}
}

fn PhysicsCombine max_combine(PhysicsCombine a, PhysicsCombine b) @local => a.ordinal > b.ordinal ? a : b;

fn float combine_values(float a, float b, PhysicsCombine mode) @local
{
	switch (mode)
	{
		case AVERAGE:
			return (a + b) * 0.5f;
		case MINIMUM:
			return math::min(a, b);
		case MAXIMUM:
			return math::max(a, b);
		case MULTIPLY:
			return a * b;
	}
}

fn void Collision.create_xpbd_constraint(&collision, PhysicsWorld* world, float dt)
{
	Rigidbody* a = world.find_body(collision.a)!!;
	Rigidbody* b = world.find_body(collision.b)!!;
	Vec3 normal = collision.normal.normalize();

	// Combine material properties (higher-priority combine mode wins)
	PhysicsCombine friction_mode = max_combine(a.material.friction_combine, b.material.friction_combine);
	PhysicsCombine restitution_mode = max_combine(a.material.restitution_combine, b.material.restitution_combine);

	float restitution = combine_values(a.material.restitution, b.material.restitution, restitution_mode);
	float static_friction = combine_values(a.material.static_friction, b.material.static_friction, friction_mode);

	// Contact arms (world-space offset from body center)
	Vec3 rA = collision.contact[0] - a.collider.translation;
	Vec3 rB = collision.contact[1] - b.collider.translation;
	// Store in local space for recomputing after position changes
	Vec3 rA_local = a.collider.rotation.conjugate() * rA;
	Vec3 rB_local = b.collider.rotation.conjugate() * rB;

	// Compute tangent basis
	Vec3 helper = (math::abs(normal.y) > 0.99) ? { 1.0f, 0.0f, 0.0f } : { 0.0f, 1.0f, 0.0f };
	Vec3 tangent_1 = normal.cross(helper).normalize();
	Vec3 tangent_2 = normal.cross(tangent_1);

	// Pre-solve relative velocity for restitution (before XPBD changes positions)
	Vec3 va = a.relative_velocity(rA);
	Vec3 vb = b.relative_velocity(rB);
	float rel_vel_n = (va - vb).dot(normal);

	XpbdConstraint constraint = {
		.body_a = a,
		.body_b = b,
		.normal = normal,
		.r_a = rA,
		.r_b = rB,
		.r_a_local = rA_local,
		.r_b_local = rB_local,
		.c_value = 0, // Contact points are on body surfaces; separation already encodes penetration
		.compliance = 0,
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


// Insert a contact into a sorted cache (deepest contacts first, capacity
// MAX_CONTACTS_PER_PAIR). Near-duplicates — same contact point and normal —
// keep only the deeper entry, so manifolds spread across distinct points.
// Thread-safe: operates on a caller-owned cache. Returns the new count.
fn sz insert_contact_sorted(Collision[] cache, sz count, Collision contact) @local
{
	// Dedup against existing contacts
	for (sz i = 0; i < count; i++)
	{
		if (cache[i].contact[0].distance_sq(contact.contact[0]) < DEDUP_DIST_SQ
			&& cache[i].normal.dot(contact.normal) > 0.99f)
		{
			if (contact.depth <= cache[i].depth) return count; // Existing one is deeper
			// New one is deeper: remove the duplicate, then insert sorted below
			for (sz j = i; j + 1 < count; j++)
			{
				cache[j] = cache[j + 1];
			}
			count--;
			break;
		}
	}

	sz insert_pos = count;
	for (sz i = 0; i < count; i++)
	{
		if (contact.depth > cache[i].depth)
		{
			insert_pos = i;
			break;
		}
	}

	if (count < MAX_CONTACTS_PER_PAIR)
	{
		for (sz i = count; i > insert_pos; i--)
		{
			cache[i] = cache[i - 1];
		}
		cache[insert_pos] = contact;
		return count + 1;
	}

	if (insert_pos < MAX_CONTACTS_PER_PAIR)
	{
		for (sz i = MAX_CONTACTS_PER_PAIR - 1; i > insert_pos; i--)
		{
			cache[i] = cache[i - 1];
		}
		cache[insert_pos] = contact;
	}
	return count;
}

// Check collisions using CoACD decomposed convex pieces
// Each piece is tested as a convex-convex collision (much faster than BVH triangles)
// Note: At least one body must have a compound shape (pairs are pre-categorized)
fn void PhysicsWorld.check_coacd_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair)
{
	bool a_has_compound = body_a.collider.get_type() == ShapeType.COMPOUND;
	bool b_has_compound = body_b.collider.get_type() == ShapeType.COMPOUND;

	// Determine which body has the compound shape
	Rigidbody* compound_body;
	Rigidbody* other_body;
	bool compound_is_a;

	if (a_has_compound)
	{
		compound_body = body_a;
		other_body = body_b;
		compound_is_a = true;
	}
	else
	{
		// b must have compound (pairs are pre-categorized to ensure at least one has compound)
		compound_body = body_b;
		other_body = body_a;
		compound_is_a = false;
	}

	CompoundShape* compound = (CompoundShape*)compound_body.collider.shape;

	Collision[MAX_CONTACTS_PER_PAIR] contact_cache;
	sz contact_count = 0;

	// Test each convex piece against the other body
	foreach (&piece : compound.shapes)
	{
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
		if (compound_is_a)
		{
			info = collision::check_convex_collision(&piece_shape, &other_body.collider, epa: true);
		}
		else
		{
			info = collision::check_convex_collision(&other_body.collider, &piece_shape, epa: true);
		}

		if (info.collided && info.depth > 0)
		{
			contact_count = insert_contact_sorted(
				&contact_cache,
				contact_count,
				{
					.a = pair.a,
					.b = pair.b,
					.normal = info.normal,
					.contact = info.contact_points,
					.depth = info.depth,
				}
			);
		}
	}

	// Push the best contacts to the collision list; only the deepest one is
	// worth vertex-expanding — the rest already form a manifold.
	for (sz i = 0; i < contact_count; i++)
	{
		contact_cache[i].skip_expand = i > 0;
		self.collisions.push(contact_cache[i]);
	}
}

fn bool test_sphere_triangle_collision(
	Vec3 sphere_center,
	float radius,
	Vec3 v0,
	Vec3 v1,
	Vec3 v2,
	Vec3 mesh_normal,
	Vec3* out_normal,
	Vec3[2]* out_contact,
	float* out_depth
) @local
{
	Vec3 closest = closest_point_on_triangle(sphere_center, v0, v1, v2);
	Vec3 diff = sphere_center - closest;
	float dist_sq = diff.length_sq();

	if (dist_sq >= radius * radius) return false;

	if (dist_sq > DEGENERATE_DIST_SQ)
	{
		float dist = math::sqrt(dist_sq);
		if (diff.dot(mesh_normal) >= 0)
		{
			*out_normal = diff / dist;
			*out_depth = radius - dist;
		}
		else
		{
			// Center crossed to the back side of the one-sided surface —
			// push back out along the face normal instead of ejecting through.
			*out_normal = mesh_normal;
			*out_depth = radius + dist;
		}
	}
	else
	{
		*out_normal = mesh_normal;
		*out_depth = radius;
	}
	*out_contact = { closest, sphere_center - *out_normal * radius };
	return true;
}

fn bool test_capsule_triangle_collision(
	Vec3 capsule_p0,
	Vec3 capsule_p1,
	float radius,
	Vec3 v0,
	Vec3 v1,
	Vec3 v2,
	Vec3 mesh_normal,
	Vec3* out_normal,
	Vec3[2]* out_contact,
	float* out_depth
) @local
{
	float min_dist_sq = float::max;
	Vec3 closest_on_axis = { 0, 0, 0 };
	Vec3 closest_on_tri = { 0, 0, 0 };

	Vec3 cp = closest_point_on_triangle(capsule_p0, v0, v1, v2);
	float d = (capsule_p0 - cp).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = capsule_p0;
		closest_on_tri = cp;
	}

	cp = closest_point_on_triangle(capsule_p1, v0, v1, v2);
	d = (capsule_p1 - cp).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = capsule_p1;
		closest_on_tri = cp;
	}

	Vec3 seg_pt = closest_point_on_segment(v0, capsule_p0, capsule_p1);
	d = (seg_pt - v0).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = seg_pt;
		closest_on_tri = v0;
	}

	seg_pt = closest_point_on_segment(v1, capsule_p0, capsule_p1);
	d = (seg_pt - v1).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = seg_pt;
		closest_on_tri = v1;
	}

	seg_pt = closest_point_on_segment(v2, capsule_p0, capsule_p1);
	d = (seg_pt - v2).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = seg_pt;
		closest_on_tri = v2;
	}

	Vec3 c1, c2;
	closest_points_segments(capsule_p0, capsule_p1, v0, v1, &c1, &c2);
	d = (c1 - c2).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = c1;
		closest_on_tri = c2;
	}

	closest_points_segments(capsule_p0, capsule_p1, v1, v2, &c1, &c2);
	d = (c1 - c2).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = c1;
		closest_on_tri = c2;
	}

	closest_points_segments(capsule_p0, capsule_p1, v2, v0, &c1, &c2);
	d = (c1 - c2).length_sq();
	if (d < min_dist_sq)
	{
		min_dist_sq = d;
		closest_on_axis = c1;
		closest_on_tri = c2;
	}

	if (min_dist_sq >= radius * radius) return false;

	Vec3 diff = closest_on_axis - closest_on_tri;
	float dist_sq = diff.length_sq();

	if (dist_sq > DEGENERATE_DIST_SQ)
	{
		float dist = math::sqrt(dist_sq);
		if (diff.dot(mesh_normal) >= 0)
		{
			*out_normal = diff / dist;
			*out_depth = radius - dist;
		}
		else
		{
			// Axis crossed to the back side of the one-sided surface —
			// push back out along the face normal instead of ejecting through.
			*out_normal = mesh_normal;
			*out_depth = radius + dist;
		}
	}
	else
	{
		*out_normal = mesh_normal;
		*out_depth = radius;
	}
	*out_contact = { closest_on_tri, closest_on_axis - *out_normal * radius };
	return true;
}

<*
 Test a mesh-BVH pair and collect a contact manifold: up to MAX_CONTACTS_PER_PAIR
 deduplicated contacts across the overlapped triangles, deepest first.
 Thread-safe — writes only into the caller-provided out_contacts.
 Returns the number of contacts written.
 @require out_contacts.len >= MAX_CONTACTS_PER_PAIR : `Output buffer too small`
*>
fn sz PhysicsWorld.test_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair, Collision[] out_contacts)
{
	bool a_has_bvh = body_a.has_mesh_bvh();

	Rigidbody* mesh_body;
	Rigidbody* other_body;
	bool mesh_is_a;

	if (a_has_bvh)
	{
		mesh_body = body_a;
		other_body = body_b;
		mesh_is_a = true;
	}
	else
	{
		mesh_body = body_b;
		other_body = body_a;
		mesh_is_a = false;
	}

	Mesh* mesh = (Mesh*)mesh_body.collider.shape;

	// Pre-compute transform values for AABB calculation
	Vec3 mesh_translation = mesh_body.collider.translation;
	Vec3 mesh_scale = mesh_body.collider.scale;
	QuaternionValue mesh_rotation = mesh_body.collider.rotation;

	Aabb3 local_aabb = other_body.transformed_aabb.inverse_transform(mesh_translation, mesh_scale, mesh_rotation);

	// Check if we can use specialized collision tests (sphere/capsule vs triangle)
	ShapeType other_type = other_body.collider.get_type();
	Vec3 other_scale = other_body.collider.scale;
	bool uniform_scale = math::abs(other_scale.x - other_scale.y) < UNIFORM_SCALE_TOL
		&& math::abs(other_scale.y - other_scale.z) < UNIFORM_SCALE_TOL;

	bool use_sphere = uniform_scale && other_type == SPHERE;
	bool use_capsule = uniform_scale && other_type == CAPSULE;
	bool use_specialized = use_sphere || use_capsule;

	Vec3 sphere_center;
	Vec3 capsule_p0;
	Vec3 capsule_p1;
	float world_radius = 0;

	if (use_sphere)
	{
		Sphere* sphere = (Sphere*)other_body.collider.shape;
		sphere_center = other_body.collider.rotation * (sphere.center * other_scale) + other_body.collider.translation;
		world_radius = sphere.radius * other_scale.x;
	}
	else if (use_capsule)
	{
		Capsule* capsule = (Capsule*)other_body.collider.shape;
		// Capsule axis endpoints in local space: cap-sphere centers at ±height/2
		float half = capsule.height * 0.5f;
		Vec3 local_p0 = { 0, -half, 0 };
		Vec3 local_p1 = { 0, half, 0 };
		capsule_p0 = other_body.collider.rotation * (local_p0 * other_scale) + other_body.collider.translation;
		capsule_p1 = other_body.collider.rotation * (local_p1 * other_scale) + other_body.collider.translation;
		world_radius = math::max(capsule.radius_top, capsule.radius_bottom) * other_scale.x;
	}

	sz contact_count = 0;

	mesh.bvh.@foreach_triangle(local_aabb; TriangleVerts tri)
	{
		// Transform triangle to world space
		Vec3 v0 = mesh_rotation * (tri[0] * mesh_scale) + mesh_translation;
		Vec3 v1 = mesh_rotation * (tri[1] * mesh_scale) + mesh_translation;
		Vec3 v2 = mesh_rotation * (tri[2] * mesh_scale) + mesh_translation;

		Aabb3 tri_aabb = { .min = math::min(v0, math::min(v1, v2)), .max = math::max(v0, math::max(v1, v2)) };

		if (tri_aabb.collides(other_body.transformed_aabb))
		{
			// Face normal of the (one-sided) mesh surface; requires consistent winding
			Vec3 tri_normal = (v1 - v0).cross(v2 - v0).normalize();

			if (use_specialized)
			{
				Vec3 n;
				Vec3[2] contact;
				float depth;

				bool hit;
				if (use_sphere)
				{
					hit = test_sphere_triangle_collision(
						sphere_center,
						world_radius,
						v0,
						v1,
						v2,
						tri_normal,
						&n,
						&contact,
						&depth
					);
				}
				else
				{
					hit = test_capsule_triangle_collision(
						capsule_p0,
						capsule_p1,
						world_radius,
						v0,
						v1,
						v2,
						tri_normal,
						&n,
						&contact,
						&depth
					);
				}

				if (hit)
				{
					Collision c = { .a = pair.a, .b = pair.b, .depth = depth };
					if (mesh_is_a)
					{
						c.normal = n;
						c.contact = contact;
					}
					else
					{
						c.normal = -n;
						c.contact = { contact[1], contact[0] };
					}
					contact_count = insert_contact_sorted(out_contacts, contact_count, c);
				}
			}
			else
			{
				// GJK/EPA fallback for other shape types — triangle already in world space
				TriangleShape tri_shape = { .v = { v0, v1, v2 } };

				CollisionInfo info;
				if (mesh_is_a)
				{
					info = collision::check_convex_collision(&tri_shape, &other_body.collider, epa: true);
				}
				else
				{
					info = collision::check_convex_collision(&other_body.collider, &tri_shape, epa: true);
				}

				if (info.collided && info.depth > 0)
				{
					// One-sided surface: contacts must push the other body out
					// along the face normal. EPA resolving through the back side
					// means the body crossed the surface this step — rebuild the
					// contact against the triangle's plane instead of ejecting
					// the body through the mesh.
					Vec3 push_dir = mesh_is_a ? info.normal : -info.normal;
					if (push_dir.dot(tri_normal) < 0)
					{
						Vec3 deep = other_body.collider.furthest_point(-tri_normal);
						float plane_depth = (v0 - deep).dot(tri_normal);
						if (plane_depth > 0)
						{
							Vec3 on_plane = deep + tri_normal * plane_depth;
							Collision c = { .a = pair.a, .b = pair.b, .depth = plane_depth };
							if (mesh_is_a)
							{
								c.normal = tri_normal;
								c.contact = { on_plane, deep };
							}
							else
							{
								c.normal = -tri_normal;
								c.contact = { deep, on_plane };
							}
							contact_count = insert_contact_sorted(out_contacts, contact_count, c);
						}
					}
					else
					{
						contact_count = insert_contact_sorted(
							out_contacts,
							contact_count,
							{
								.a = pair.a,
								.b = pair.b,
								.normal = info.normal,
								.contact = info.contact_points,
								.depth = info.depth,
							}
						);
					}
				}
			}
		}
	};

	// Clamp depth to prevent extreme correction forces from BVH triangle
	// collisions. Only the deepest contact is vertex-expanded later — the
	// rest of the manifold is already spread across triangles.
	for (sz i = 0; i < contact_count; i++)
	{
		out_contacts[i].depth = math::min(out_contacts[i].depth, CLAMPED_DEPTH_MAX);
		out_contacts[i].skip_expand = i > 0;
	}

	return contact_count;
}

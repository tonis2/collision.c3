module collision;

import std::collections::fixedlist;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;
import std::thread;

import std::sort;
import collision::dmath;

const MAX_CONTACTS_PER_PAIR @private = 8;

const float EPSILON @private = 1e-8f;
const float WEIGHT_EPSILON @private = 1e-10f;
const float MANIFOLD_DEPTH_TOL @private = 0.02f;
const float DEDUP_DIST_SQ @private = 0.0001f;
const float BOUNCE_THRESHOLD @private = 1.0f;
const float UNIFORM_SCALE_TOL @private = 0.001f;
const float DEGENERATE_DIST_SQ @private = 0.00001f * 0.00001f;
const float CLAMPED_DEPTH_MAX @private = 1.0f;
const float TANGENT_HELPER_DOT @private = 0.99f; // |normal.y| above this: build the tangent from x instead of y
const float CCD_BACKOFF @private = 0.99f; // Fraction of ccd_radius a clamped body stops short of the hit
const float MIN_RESTITUTION @private = 0.01f; // Below this a contact is treated as inelastic
const sz SERIAL_CONSTRAINT_LIMIT @private = 16; // At most this many constraints solve inline
const sz PHASE_TASKS_PER_WORKER @private = 2; // Fewer bodies than workers * this run predict/derive inline
const float STICK_FRACTION @private = 0.98f; // A pair using less of its friction cone than this is held, not sliding
// Relative speed (units/s, and rad/s) below which a touching pair counts as resting
// for its friction anchor. Above a stack's contact jitter (~0.25), below a knock.
const float ANCHOR_REST_SPEED @private = 1.0f;

alias CollisionFilter = HashMap {Pair, bool};
alias RigidBodyMap = LinkedHashMap {sz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator {sz, Rigidbody};
alias UnionFindMap = HashMap {sz, sz};
alias ConstraintList = List {XpbdConstraint*};
alias PairList = List {Pair};


// Tests one body pair and writes up to MAX_CONTACTS_PER_PAIR contacts; returns how many.
alias PairTestFn = fn sz(PhysicsWorld*, Rigidbody*, Rigidbody*, Pair, Collision[]);

// Task structure for parallel collision testing.
// Each task processes a contiguous slice of pairs — batching to at most one
// task per worker keeps thread pool overhead out of the hot path.
struct CollisionTask
{
	PhysicsWorld* world;
	PairTestFn test; // Run on every pair in the slice
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
	GroundPlane ground_plane;
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

<*
 Body b's pose in body a's frame while static friction holds the pair. Friction
 pulls each contact back to where this pose puts it, so slip a substep leaves
 uncorrected is corrected by the next one instead of accumulating into creep.
 Re-taken from the current pose whenever the pair slides, loses its load, is
 knocked apart, or involves a shape that rolls (see update_friction_anchors).
*>
struct FrictionAnchor
{
	QuaternionValue rotation;
	Vec3 translation;
}

alias FrictionAnchorMap = HashMap {Pair, FrictionAnchor};

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
	float c_value; // Target for joint constraints (the limit, or 0): solved error is measured minus this
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
	// Where static friction holds b's contact point (`r_b_local`), in body_a's
	// local frame: the pair's friction anchor applied to it (see FrictionAnchor).
	Vec3 friction_anchor;
	// Contacts in the manifold that starts here (solved normals first, then
	// friction); 0 on the manifold's later members.
	ushort manifold_len;
	// Restitution
	float restitution;
	float pre_solve_relative_vel_n; // Normal relative velocity before solve (for restitution)
	// Joints: +1 = lower limit (lambda >= 0, only pushes apart), -1 = upper limit (lambda <= 0), 0 = bilateral
	int limit_side;
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
	ushort manifold_len; // Set by generate_manifold: contacts in the manifold starting here, 0 on its later members
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

	// Inverse body-frame inertia tensor, set by add_body. Full 3x3: it is only
	// diagonal for boxes, spheres, capsules and cylinders.
	Matrix3f inv_local_inertia;
	// Derived cache of R * inv_local_inertia * Rᵀ, valid while the collider rotation
	// equals `inv_world_rotation` (compared bitwise-by-value on every read, so any
	// writer of `collider.rotation` — solver, snapshot restore, the host — stays correct).
	Matrix3f inv_world_inertia;
	QuaternionValue inv_world_rotation;

	// Conservative inner radius for swept CCD, set by add_body. A body whose
	// per-substep displacement exceeds this is swept against static geometry.
	// 0 disables CCD for this body.
	float ccd_radius;

	// Per-substep scratch for packing constraints into worker buckets (see
	// xpbd_solve_constraints): an intrusive union-find over the constraint graph,
	// so grouping allocates nothing. Meaningless outside that call.
	Rigidbody* solve_parent; // null = root of its island
	int solve_bucket; // bucket of the island rooted here, -1 = unassigned

	// XPBD: previous state for velocity derivation
	Vec3 prev_position;
	QuaternionValue prev_rotation;
}

<*
 The plane soft bodies rest on: `dot(p, normal) < offset` is below the surface,
 and the penetration depth is the difference. A normal plus offset describes a
 ground perpendicular to any axis, not just one.
*>
struct GroundPlane
{
	Vec3 normal; // Unit, pointing away from the ground.
	float offset;
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
	TriggerOverlapSet triggers_next; // Double buffer: this substep's overlaps, swapped in by @retire_overlaps

	// Contact event system
	List {ContactEvent} contact_events;
	ContactOverlapSet active_contacts; // Currently contacting body pairs
	ContactOverlapSet contacts_next;

	// Static-friction anchors per touching pair, double-buffered: a substep reads
	// `friction_anchors` and writes the survivors to `anchors_next`.
	FrictionAnchorMap friction_anchors;
	FrictionAnchorMap anchors_next;

	// Soft body system (XPBD)
	SoftBodyList soft_bodies;
	GroundPlane ground_plane;
	bool ground_plane_enabled;

	// Swept CCD for fast-moving bodies (see apply_ccd)
	bool ccd_enabled;

	// Where this world's storage lives. Set by `init`; anything that creates a
	// collection which outlives a step reads it, falling back to the heap.
	Allocator allocator;

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

	// Per-substep scratch, seated on `allocator` and cleared (never re-grown) each
	// substep. Zero-valued lists would bind to the temp allocator and grow it forever.
	PairList candidate_pairs;
	PairList dormant_pairs; // Touching last step, all asleep now: carried, not narrow-phased
	PairList convex_pairs;
	PairList bvh_pairs;
	PairList field_pairs;
	PairList compound_pairs;
	ConstraintList[MAX_WORKER_THREADS] constraint_buckets;
	List {SoftBodySolveTask} soft_tasks;
	List {SoftBody*} coupled_soft;
	List {Rigidbody*} ccd_obstacles;

	// Non-static bodies sorted by (island_id, id), so waking an island is a
	// binary search plus a run instead of a scan of every body.
	List {Rigidbody*} island_members;
	bool members_dirty;
}

// Seats a collection on `allocator` unless it already has one.
macro void @seat(#collection, Allocator allocator) @private
{
	if (!#collection.is_initialized()) #collection.init(allocator);
}

<*
 For each pair in `active` that is missing from `next`, run the trailing body;
 then `next` becomes the new `active` and is left empty for reuse. Expiring an
 overlap set is the same diff-and-swap for triggers and contacts, at the step
 boundary and when a body is removed.
*>
macro void @retire_overlaps(#active, #next; @ended(Pair pair)) @private
{
	#active.@each(; Pair pair, bool flag)
	{
		if (!#next.has_key(pair)) @ended(pair);
	};
	@swap(#active, #next);
	#next.clear();
}

<*
 Whether a body's transform is still a number.

 A body that starts deep inside static geometry can be corrected into an
 infinity or NaN within a few substeps, and a non-finite AABB aborts the
 process inside the broadphase. The predict phase checks this first and resets
 such a body instead of propagating it.
*>
fn bool Rigidbody.transform_is_finite(&self)
{
	Vec3 at = self.collider.translation;
	QuaternionValue rot = self.collider.rotation;
	return is_finite3(at) && math::is_finite(rot.i) && math::is_finite(rot.j) && math::is_finite(rot.k)
		&& math::is_finite(rot.l);
}

<*
 Put a body back where it was before a nonsensical prediction and stop it. The
 overlap that caused it is still there, so keeping its velocity would repeat the
 failure every step.
*>
fn void Rigidbody.reset_to_previous(&self)
{
	self.collider.translation = self.prev_position;
	self.collider.rotation = self.prev_rotation;
	self.linear_velocity = { 0, 0, 0 };
	self.angular_velocity = { 0, 0, 0 };
}

// Remember the current transform for velocity derivation and friction.
fn void Rigidbody.save_previous(&self)
{
	self.prev_position = self.collider.translation;
	self.prev_rotation = self.collider.rotation;
}

<*
 Velocities implied by the move from the saved transform to the current one:
 linear from the position delta, angular as 2 * (q * q_prev⁻¹).xyz / dt on the
 shortest arc.
*>
fn void Rigidbody.derive_velocity(&self, float inv_dt)
{
	self.linear_velocity = (self.collider.translation - self.prev_position) * inv_dt;

	QuaternionValue dq = self.collider.rotation * self.prev_rotation.conjugate();
	if (dq.l < 0) dq = -dq;
	self.angular_velocity = dq.v.xyz * (2.0f * inv_dt);
}

<*
 The world's storage allocator: the one given to `init`, or the heap when the
 world was never `init`ed (a bare `DEFAULT_PHYSICS_WORLD`).
*>
fn Allocator PhysicsWorld.storage(&self)
{
	if (self.allocator.ptr == null) self.allocator = mem;
	return self.allocator;
}

<*
 Seat every collection that outlives a call or is reused per substep.

 A zero-valued `std::collections` container lazily binds to the *temp*
 allocator, which is harmless in a test and fatal in a host that steps across
 frames: the pool is released and the world keeps reading it, failing later
 inside a hash table. Idempotent.
*>
fn void PhysicsWorld.seat_storage(&self)
{
	Allocator a = self.storage();
	@seat(self.active_triggers, a);
	@seat(self.triggers_next, a);
	@seat(self.active_contacts, a);
	@seat(self.contacts_next, a);
	@seat(self.friction_anchors, a);
	@seat(self.anchors_next, a);
	@seat(self.candidate_pairs, a);
	@seat(self.dormant_pairs, a);
	@seat(self.convex_pairs, a);
	@seat(self.bvh_pairs, a);
	@seat(self.field_pairs, a);
	@seat(self.compound_pairs, a);
	@seat(self.soft_tasks, a);
	@seat(self.coupled_soft, a);
	@seat(self.ccd_obstacles, a);
	@seat(self.island_members, a);
}

<*
 Put the world's storage on a named allocator, once, before anything is added.
 `free` gives all of it back, so a world is init/free like everything else.
 Call once: initializing twice abandons the first set of allocations.
*>
fn void PhysicsWorld.init(&self, Allocator allocator)
{
	self.allocator = allocator;
	self.bodies.init(allocator);
	self.collisions.init(allocator);
	self.manifold_scratch.init(allocator);
	self.constraints.init(allocator);
	self.joints.init(allocator);
	self.spatial_map.init(allocator);
	self.filters.init(allocator);
	self.island_parent.init(allocator);
	self.trigger_events.init(allocator);
	self.contact_events.init(allocator);
	self.soft_bodies.init(allocator);
	self.collision_tasks.init(allocator);
	self.collision_results.init(allocator);
	self.collision_counts.init(allocator);
	self.island_tasks.init(allocator);
	self.phase_tasks.init(allocator);
	self.active_bodies.init(allocator);
	self.seat_storage();
}

<*
 A configured world with its storage already seated — `DEFAULT_PHYSICS_WORLD`
 followed by `init` in one call. `config` is copied before `init` runs and
 `init` only binds allocators, so everything set on the way in survives:
 gravity, `spatial_map.cell_size`, `thread_count`, `ccd_enabled`, sleep figures.

 @param config : `The settings to keep. Defaults to the library's own.`
 @param allocator : `Where the world's containers live. It has to outlive the world.`
*>
fn PhysicsWorld create_world(PhysicsWorld config = DEFAULT_PHYSICS_WORLD, Allocator allocator = mem)
{
	PhysicsWorld world = config;
	world.init(allocator);
	return world;
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
	self.collision_tasks.free();
	self.collision_results.free();
	self.collision_counts.free();
	self.island_tasks.free();
	self.phase_tasks.free();
	self.active_bodies.free();
	self.trigger_events.free();
	self.active_triggers.free();
	self.triggers_next.free();
	self.contact_events.free();
	self.active_contacts.free();
	self.contacts_next.free();
	self.friction_anchors.free();
	self.anchors_next.free();
	self.candidate_pairs.free();
	self.dormant_pairs.free();
	self.convex_pairs.free();
	self.bvh_pairs.free();
	self.field_pairs.free();
	self.compound_pairs.free();
	foreach (&bucket : self.constraint_buckets) bucket.free();
	self.soft_tasks.free();
	self.coupled_soft.free();
	self.ccd_obstacles.free();
	self.island_members.free();
	foreach (&sb : self.soft_bodies) sb.free();
	self.soft_bodies.free();
}

// The z-down gravity is a default, not an assumption: `impulse_dir` takes its
// sign along whatever gravity is set (`add_body`), the soft-body ground carries
// its own normal (`enable_ground_plane`), and the cylinder/capsule are defined
// about +Y with endpoints rotated by the collider's own orientation.
const PhysicsWorld DEFAULT_PHYSICS_WORLD = {
	.gravity = { 0.0f, 0.0f, -9.8f },
	.ground_plane = { .normal = { 0.0f, 0.0f, 1.0f } },
	.spatial_map .cell_size = 2.0,
	// Seconds under `sleep_delta` before a body sleeps. Long enough that
	// residual contact creep (linear_dampening is a ~9.5 s time constant) has
	// stopped, short enough not to be watched.
	.sleep_timer = 0.5,
	.linear_dampening = 0.9,
	.angular_dampening = 0.9,
	.sleep_delta = 0.2,
	.ccd_enabled = true,
};

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody*? PhysicsWorld.find_body(&self, sz id) => self.bodies.get_ref(id);

<*
 Visit every body. The trailing body runs inside the iteration, so `return`
 leaves the caller, and `continue`/`break` bind to a loop *around* the call,
 not to this one: guard with `if` instead.
*>
macro void PhysicsWorld.@each_body(&self; @body(Rigidbody* b))
{
	RigidBodyIterator iter = self.bodies.value_iter();
	while (iter.next())
	{
		Rigidbody* b = iter.get()!!;
		@body(b);
	}
}

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint)
{
	// Joints keep their bodies from colliding with each other
	if (!joint.collides)
	{
		self.filters.set({ joint.body_a, joint.body_b }, true);
	}

	self.joints.push(joint);
	self.islands_dirty = true;
}

// Refresh the local and world AABBs from the collider.
fn void Rigidbody.update_bounds(&self)
{
	self.aabb = self.collider.get_aabb();
	self.transformed_aabb = self.get_transformed_aabb();
}

<*
 Inverse body-frame inertia. The shape's own tensor, with the collider scale
 applied and no rotation (rotation is applied at solve time), when the shape
 provides one; otherwise a solid box over the scaled AABB.
*>
fn Matrix3f Rigidbody.compute_inv_local_inertia(&self)
{
	Matrix3f tensor;
	if (&self.collider.shape.inertia_tensor)
	{
		TransformedShape unrotated = self.collider;
		unrotated.rotation = { 0, 0, 0, 1 };
		tensor = unrotated.inertia_tensor(self.mass);
	}
	else
	{
		Aabb3 scaled = { self.aabb.min * self.collider.scale, self.aabb.max * self.collider.scale };
		tensor = scaled.inertia_tensor(self.mass);
	}
	return tensor.inverse() ?? (Matrix3f){};
}

<*
 Static geometry that never needs pairing with other static geometry.
 Kinematic bodies are marked static but move, and static triggers still need
 overlap events, so both are excluded.
*>
fn bool Rigidbody.is_fixed_geometry(&self) => self.is_static && !self.is_kinematic && !self.is_trigger;

// Store the body and register it with the broadphase; shared by bodies and triggers.
fn void PhysicsWorld.insert_body(&self, Rigidbody* body)
{
	self.bodies.set(body.id, *body);
	self.spatial_map.insert(body.transformed_aabb, body.id, is_static: body.is_fixed_geometry());
	self.members_dirty = true;
}

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
 @require body.collider.scale != { 0, 0, 0 } : `Body scale must not be 0`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body)
{
	if (body.is_kinematic)
	{
		// Externally driven: immovable for the solver, always active
		body.is_static = true;
		body.inverse_mass = 0;
		body.mass = 0;
		body.can_sleep = false;
	}
	else
	{
		body.is_static = body.mass == 0;
		body.inverse_mass = !body.is_static ? 1.0f / body.mass : 0;
	}

	// Whether the body's own gravity runs with the world's or against it (an
	// inverted `gravity_factor` flips the axes an angular joint limit pushes
	// along). A projection on gravity, not `.z`, so any gravity axis works; `>=`
	// so a zero factor counts as "with".
	body.impulse_dir = (self.gravity * body.gravity_factor).dot(self.gravity) >= 0 ? -1 : 1;

	body.update_bounds();
	body.sleep_delta = self.sleep_delta;

	if (!body.is_static)
	{
		body.inv_local_inertia = body.compute_inv_local_inertia();
		body.inv_world_rotation = { }; // Never a unit rotation: forces the first read to build the cache

		// Conservative inner radius for swept CCD: half the smallest world-space extent
		body.ccd_radius = math::abs(body.aabb.extent() * body.collider.scale).min();
	}

	// Default to world dampening if not specified per-body
	if (body.linear_dampening == 0) body.linear_dampening = self.linear_dampening;
	if (body.angular_dampening == 0) body.angular_dampening = self.angular_dampening;

	// So friction doesn't see phantom displacement on the first step
	body.save_previous();

	self.insert_body(&body);
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
	body.update_bounds();

	self.insert_body(&body);
}

<*
 Remove a rigid body, and every trace of it.

 Beyond dropping it from `bodies`:
 - the **spatial hash**, or a dead id keeps producing broadphase pairs;
 - the **island union-find**, marked dirty: the island may split in two;
 - the **active trigger and contact sets**, whose EXIT and END events are
   emitted here, so a body destroyed inside a trigger is still seen leaving it;
 - every **joint** naming it, plus the collision filter `add_constraint` paired
   with it.

 `constraints` needs no attention: `run_substep` rebuilds it every substep.

 The events land in `trigger_events` / `contact_events`, which `run_step` clears
 at the top, so a caller that removes bodies between steps drains them first.

 @return `false when no body has that id, so removing twice is not an error.`
*>
fn bool PhysicsWorld.remove_body(&self, sz id)
{
	Rigidbody*? found = self.bodies.get_ref(id);
	if (catch found) return false;

	self.spatial_map.remove(found.transformed_aabb, id);

	// The `*_next` buffers (seated) take the survivors; the retired pairs become events.
	self.seat_storage();
	self.active_triggers.@each(; Pair pair, bool active)
	{
		if (pair.a != id && pair.b != id) self.triggers_next.set(pair, true);
	};
	@retire_overlaps(self.active_triggers, self.triggers_next; Pair pair)
	{
		self.trigger_events.push({ .trigger_id = pair.a, .other_id = pair.b, .type = TriggerEventType.EXIT });
	};

	self.active_contacts.@each(; Pair pair, bool active)
	{
		if (pair.a != id && pair.b != id) self.contacts_next.set(pair, true);
	};
	@retire_overlaps(self.active_contacts, self.contacts_next; Pair pair)
	{
		self.contact_events.push({ .body_a = pair.a, .body_b = pair.b, .type = ContactEventType.END });
	};

	self.anchors_next.clear();
	self.friction_anchors.@each(; Pair pair, FrictionAnchor anchor)
	{
		if (pair.a != id && pair.b != id) self.anchors_next.set(pair, anchor);
	};
	@swap(self.friction_anchors, self.anchors_next);
	self.anchors_next.clear();

	// Backwards, so removing one does not skip the next.
	for (sz i = self.joints.len(); i > 0; i--)
	{
		GenericJoint3D joint = self.joints[i - 1];
		if (joint.body_a != id && joint.body_b != id) continue;
		self.filters.remove({ joint.body_a, joint.body_b });
		self.joints.remove_at(i - 1);
	}

	self.island_parent.remove(id);
	self.islands_dirty = true;
	self.members_dirty = true;

	self.bodies.remove(id);
	return true;
}

// Recompute a body's broadphase AABB and move it in the spatial hash.
fn void PhysicsWorld.refresh_broadphase(&self, Rigidbody* body)
{
	Aabb3 new_aabb = body.get_transformed_aabb();
	self.spatial_map.update(body.transformed_aabb, new_aabb, body.id);
	body.transformed_aabb = new_aabb;
}

<*
 Set the target position/rotation for a kinematic body or a trigger. Must be
 called each frame before run_step(), with dt matching the frame delta. Computes
 velocity from the delta so dynamic bodies that collide with this kinematic get
 correct restitution/friction responses.

 A trigger takes the same door but nothing integrates it, so its velocity is not
 derived; the spatial hash update is what makes a moved trigger overlap things.
 @require dt > 0 : `dt must be positive`
*>
fn void PhysicsWorld.set_kinematic_target(&self, sz id, Vec3 position, QuaternionValue rotation, float dt = 1.0f / 60.0f)
{
	Rigidbody* body = self.find_body(id)!!;
	if (!body.is_kinematic && !body.is_trigger) return;

	body.save_previous();
	body.collider.translation = position;
	body.collider.rotation = rotation;

	if (!body.is_trigger) body.derive_velocity(1.0f / dt);

	self.refresh_broadphase(body);
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
 Enable ground plane collision for soft bodies. The plane is a normal plus an
 offset, so a soft body can rest on a slope; the defaults are the z=0 plane.

 @param normal : `Which way is up, away from the ground. Need not be unit.`
 @param offset : `Where the surface sits along that normal.`
 @require normal.length_sq() > 0 : `The ground plane needs a direction`
*>
fn void PhysicsWorld.enable_ground_plane(&self, Vec3 normal = { 0, 0, 1 }, float offset = 0)
{
	self.ground_plane_enabled = true;
	self.ground_plane = { .normal = normal.normalize(), .offset = offset };
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

<*
 World-space inverse inertia, `R * inv_local_inertia * Rᵀ`. Static and kinematic
 bodies answer zero (infinite inertia), so angular impulses are no-ops.

 Cached per body and rebuilt only when the rotation differs from the one the
 cache was built for, so a body's tensor is computed once per rotation change
 instead of once per query. The cache is derived state: snapshot restore needs
 no special handling because a restored rotation simply misses the key.
*>
fn Matrix3f Rigidbody.get_inverse_world_inertia(&self)
{
	if (self.is_static) return { };
	if (self.inv_world_rotation.v != self.collider.rotation.v)
	{
		self.inv_world_inertia = self.collider.world_inertia_from_local(self.inv_local_inertia);
		self.inv_world_rotation = self.collider.rotation;
	}
	return self.inv_world_inertia;
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

fn Vec3 Rigidbody.relative_velocity(&self, Vec3 point) => self.linear_velocity + self.angular_velocity.cross(point);

// `sleep_delta` is a speed (units/s linear, rad/s angular), so it is squared
// before being compared against a squared length.
fn bool Rigidbody.is_rotating(&self) => self.angular_velocity.length_sq() > self.sleep_delta * self.sleep_delta;
fn bool Rigidbody.is_moving(&self) => self.linear_velocity.length_sq() > self.sleep_delta * self.sleep_delta;
fn bool Rigidbody.is_sleeping(&self) => self.sleeping;
fn void Rigidbody.stop_sleeping(&self)
{
	self.sleeping = false;
	self.sleep_timer = 0;
}
fn bool Rigidbody.is_active(&self) => self.sleeping == false && self.is_static == false;

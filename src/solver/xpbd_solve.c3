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

// ============================================================================
// Constraint math
// ============================================================================

// XPBD generalized inverse mass: w = 1/m + (r×n)^T · I^{-1} · (r×n)
fn float compute_generalized_inverse_mass(Rigidbody* body, Vec3 r, Vec3 n)
{
	if (body.is_static) return 0;
	Vec3 rn = r.cross(n);
	Vec3 inertia_term = body.get_inverse_world_inertia() * rn;
	return body.inverse_mass + inertia_term.dot(rn);
}

// Combined generalized inverse mass of a pair along `n`, each body at its own arm.
fn float pair_inverse_mass(Rigidbody* a, Vec3 r_a, Rigidbody* b, Vec3 r_b, Vec3 n) @private
{
	return compute_generalized_inverse_mass(a, r_a, n) + compute_generalized_inverse_mass(b, r_b, n);
}

// Rotate by an axis-angle vector (axis = direction, angle = length).
// Uses dmath's sincos: libm sin/cos differ across platforms and would break
// lockstep determinism (see determinism.md).
fn void apply_angular_rotation(Rigidbody* body, Vec3 correction) @private
{
	float angle = correction.length();
	if (angle <= EPSILON) return;
	QuaternionValue dq = dmath::quat_from_axis_angle(correction / angle, angle);
	body.collider.rotation = (dq * body.collider.rotation).normalize();
}

// XPBD positional correction: apply delta_lambda along direction n at contact arm r
fn void apply_positional_correction(Rigidbody* body, Vec3 n, Vec3 r, float delta_lambda)
{
	if (body.is_static) return;
	body.collider.translation += n * (body.inverse_mass * delta_lambda);
	apply_angular_rotation(body, body.get_inverse_world_inertia() * r.cross(n) * delta_lambda);
}

// Push `a` along -n and `b` along +n by delta_lambda, each at its own arm.
fn void apply_pair_correction(Rigidbody* a, Rigidbody* b, Vec3 n, Vec3 r_a, Vec3 r_b, float delta_lambda) @private
{
	apply_positional_correction(a, -n, r_a, delta_lambda);
	apply_positional_correction(b, n, r_b, delta_lambda);
}

<*
 Keep an accumulated multiplier on its allowed side of zero after an unclamped
 step: side +1 allows lambda >= 0, side -1 allows lambda <= 0, 0 is unrestricted.
 Returns the delta that was actually applied.
*>
fn float clamp_lambda_side(float* lambda, float previous, float delta, int side) @private
{
	if ((side > 0 && *lambda < 0) || (side < 0 && *lambda > 0))
	{
		*lambda = 0;
		return -previous;
	}
	return delta;
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
) @private
{
	if (math::abs(displacement) <= EPSILON) return;
	float w = pair_inverse_mass(a, c.r_a, b, c.r_b, tangent);
	if (w <= WEIGHT_EPSILON) return;
	float new_l = math::clamp(*lambda_t - displacement / w, -max_friction, max_friction);
	float dl = new_l - *lambda_t;
	*lambda_t = new_l;
	apply_pair_correction(a, b, -tangent, c.r_a, c.r_b, dl);
}

// Non-penetration. Contact arms are re-derived from the local-space arms because
// earlier constraints in the sweep may have moved the bodies. Returns whether the
// contact was penetrating, i.e. pushed, in this sweep.
fn bool solve_contact_normal(XpbdConstraint* c) @private
{
	Rigidbody* a = c.body_a;
	Rigidbody* b = c.body_b;

	Vec3 cur_r_a = a.collider.rotation * c.r_a_local;
	Vec3 cur_r_b = b.collider.rotation * c.r_b_local;
	Vec3 pa = a.collider.translation + cur_r_a;
	Vec3 pb = b.collider.translation + cur_r_b;

	// Positive separation = apart, negative = penetrating
	float separation = (pb - pa).dot(c.normal);
	float penetration = c.c_value - separation;
	if (penetration <= 0) return false;

	c.r_a = cur_r_a;
	c.r_b = cur_r_b;

	float w_sum = pair_inverse_mass(a, c.r_a, b, c.r_b, c.normal);
	if (w_sum < WEIGHT_EPSILON) return false;

	// Constraint C = -penetration (non-penetration: C >= 0); the contact can only push.
	float previous = c.lambda;
	float delta_lambda = xpbd_delta_lambda(-penetration, w_sum, c.compliance, &c.lambda);
	delta_lambda = clamp_lambda_side(&c.lambda, previous, delta_lambda, 1);
	c.lambda_normal = c.lambda;

	apply_pair_correction(a, b, c.normal, c.r_a, c.r_b, delta_lambda);
	return true;
}

<*
 Static friction: pull b's contact point back to where the pair's friction
 anchor holds it (`friction_anchor`, in a's frame), within the friction cone.
 The arms are re-derived: the normal pushes since they were last taken have
 rotated the bodies, and a stale arm measures slip at the wrong point.
*>
fn void solve_contact_friction(XpbdConstraint* c, float max_friction) @private
{
	if (max_friction <= EPSILON) return;
	Rigidbody* a = c.body_a;
	Rigidbody* b = c.body_b;

	c.r_a = a.collider.rotation * c.r_a_local;
	c.r_b = b.collider.rotation * c.r_b_local;
	Vec3 held = a.collider.translation + a.collider.rotation * c.friction_anchor;
	Vec3 slip = held - (b.collider.translation + c.r_b);

	solve_tangent_friction(a, b, c, c.tangent1, &c.lambda_tangent1, slip.dot(c.tangent1), max_friction);
	solve_tangent_friction(a, b, c, c.tangent2, &c.lambda_tangent2, slip.dot(c.tangent2), max_friction);
}

<*
 One manifold: every point's normal first, then friction. Solving each point's
 friction straight after its own normal lets the later points' pushes rotate the
 body and re-slip the earlier ones, which nothing corrects — a box creeps across
 a flat floor. The normal load is pooled over the points that carry any, since
 the first push of a sweep takes most of it and would starve the rest.

 Friction runs on the loaded points pushed in this sweep, as it always has: with
 more than one iteration, re-running it on every loaded point each iteration
 costs a rotation (and an inertia rebuild) per point for no visible gain.
*>
fn void solve_manifold(XpbdConstraint*[] run) @private
{
	// A lone contact (every terrain and mesh contact): no load to pool
	if (run.len == 1)
	{
		XpbdConstraint* c = run[0];
		if (solve_contact_normal(c) && c.lambda_normal > EPSILON) solve_contact_friction(c, c.friction * c.lambda_normal);
		return;
	}

	float load = 0;
	int touching = 0;
	ulong pushed = 0; // Bit k: run[k] carries load and was pushed in this sweep (manifolds are under 64 points)
	foreach (k, c : run)
	{
		bool hit = solve_contact_normal(c);
		if (c.lambda_normal > EPSILON)
		{
			load += c.lambda_normal;
			touching++;
			if (hit && k < 64) pushed |= 1UL << k;
		}
	}
	if (pushed == 0) return;

	// A point pushed by next to nothing carries no load of its own; given a share
	// of the pool its friction would act where the body barely touches.
	float share = load / (float)touching;
	foreach (k, c : run)
	{
		if (k < 64 && pushed & (1UL << k)) solve_contact_friction(c, c.friction * share);
	}
}

// Linear joint axis. `r_a`/`r_b` hold local-space anchors; the error is measured
// against `c_value` (0 for a locked axis, the limit for a limit constraint).
fn void solve_joint_position(XpbdConstraint* c, uint iteration, uint total_iterations) @private
{
	Rigidbody* a = c.body_a;
	Rigidbody* b = c.body_b;

	Vec3 arm_a = a.collider.rotation * c.r_a;
	Vec3 arm_b = b.collider.rotation * c.r_b;
	Vec3 world_a = a.collider.translation + arm_a;
	Vec3 world_b = b.collider.translation + arm_b;
	float pos_error = (world_b - world_a).dot(c.normal) - c.c_value;

	float weight = pair_inverse_mass(a, arm_a, b, arm_b, c.normal);
	if (weight < WEIGHT_EPSILON) return;

	float alpha = ramped_compliance(c.compliance, iteration, total_iterations);
	float previous = c.lambda;
	float delta_lambda = xpbd_delta_lambda(pos_error, weight, alpha, &c.lambda);
	delta_lambda = clamp_lambda_side(&c.lambda, previous, delta_lambda, c.limit_side);

	apply_pair_correction(a, b, c.normal, arm_a, arm_b, delta_lambda);
}

// Angular joint axis: `r_a` is the local-space axis, no lever arm.
fn void solve_joint_angular(XpbdConstraint* c, uint iteration, uint total_iterations) @private
{
	Rigidbody* a = c.body_a;
	Rigidbody* b = c.body_b;

	Vec3 axis = a.collider.rotation * c.r_a;
	Vec3 world_err = joint_angular_error(a.collider.rotation, b.collider.rotation, c.joint_orientation);
	float ang_error = world_err.dot(axis) - c.c_value;

	// Zero for static bodies, which makes both the weight and the rotation no-ops
	Matrix3f inv_inertia_a = a.get_inverse_world_inertia();
	Matrix3f inv_inertia_b = b.get_inverse_world_inertia();
	Vec3 accel_a = inv_inertia_a * axis;
	Vec3 accel_b = inv_inertia_b * axis;

	float weight = accel_a.dot(axis) + accel_b.dot(axis);
	if (weight < WEIGHT_EPSILON) return;

	float alpha = ramped_compliance(c.compliance, iteration, total_iterations);
	float previous = c.lambda;
	float delta_lambda = xpbd_delta_lambda(ang_error, weight, alpha, &c.lambda);
	delta_lambda = clamp_lambda_side(&c.lambda, previous, delta_lambda, c.limit_side);

	if (!a.is_static) apply_angular_rotation(a, accel_a * (-delta_lambda));
	if (!b.is_static) apply_angular_rotation(b, accel_b * delta_lambda);
}

<*
 One Gauss-Seidel sweep: contacts a manifold at a time, joints one by one. The
 constraints of a manifold are adjacent, which bucketing preserves.
*>
fn void solve_constraint_sweep(XpbdConstraint*[] cs, uint iteration, uint total_iterations) @private
{
	sz i = 0;
	while (i < cs.len)
	{
		XpbdConstraint* c = cs[i];
		switch (c.type)
		{
			case CONTACT_NORMAL:
				sz n = math::min(math::max((sz)c.manifold_len, (sz)1), cs.len - i);
				solve_manifold(cs[i:n]);
				i += n;
			case JOINT_POSITION:
				solve_joint_position(c, iteration, total_iterations);
				i++;
			case JOINT_ANGULAR:
				solve_joint_angular(c, iteration, total_iterations);
				i++;
		}
	}
}

// ============================================================================
// Islands
// ============================================================================

// Union-Find: find root with path compression (iterative)
fn sz UnionFindMap.find(&self, sz id)
{
	if (!self.has_key(id))
	{
		self.set(id, id);
		return id;
	}

	sz root = id;
	while (true)
	{
		sz parent = self.get(root)!!;
		if (parent == root) break;
		root = parent;
	}

	// Point everything on the path straight at the root
	sz node = id;
	while (node != root)
	{
		sz next = self.get(node)!!;
		self.set(node, root);
		node = next;
	}
	return root;
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

	self.island_parent.clear();

	// Each body starts as its own island
	self.@each_body(; Rigidbody* body)
	{
		self.island_parent.set(body.id, body.id);
	};

	// Union bodies connected by joints; static bodies never merge islands
	foreach (&joint : self.joints)
	{
		Rigidbody*? body_a = self.find_body(joint.body_a);
		Rigidbody*? body_b = self.find_body(joint.body_b);
		if (catch body_a, body_b) continue;
		if (!body_a.is_static && !body_b.is_static)
		{
			self.island_union(joint.body_a, joint.body_b);
		}
	}

	self.@each_body(; Rigidbody* body)
	{
		body.island_id = self.island_find(body.id);
	};

	self.islands_dirty = false;
	self.members_dirty = true;
}

fn int island_member_cmp(Rigidbody* x, Rigidbody* y) @private
{
	if (x.island_id != y.island_id) return x.island_id < y.island_id ? -1 : 1;
	if (x.id != y.id) return x.id < y.id ? -1 : 1;
	return 0;
}

fn void PhysicsWorld.rebuild_island_members(&self)
{
	self.island_members.clear();
	self.@each_body(; Rigidbody* body)
	{
		if (!body.is_static) self.island_members.push(body);
	};
	if (self.island_members.len() > 1) sort::quicksort(self.island_members.array_view(), &island_member_cmp);
	self.members_dirty = false;
}

// Wake every sleeping body in `body`'s island: a binary search for the island's
// run in the sorted member list, then a walk of just that run.
fn void PhysicsWorld.wake_island(&self, Rigidbody* body)
{
	if (!body.sleeping) return;
	if (self.members_dirty) self.rebuild_island_members();

	Rigidbody*[] members = self.island_members.array_view();
	sz lo = 0;
	sz hi = members.len;
	while (lo < hi)
	{
		sz mid = lo + (hi - lo) / 2;
		if (members[mid].island_id < body.island_id)
		{
			lo = mid + 1;
		}
		else
		{
			hi = mid;
		}
	}

	for (sz i = lo; i < members.len && members[i].island_id == body.island_id; i++)
	{
		if (members[i].sleeping) members[i].stop_sleeping();
	}
}

// ============================================================================
// Predict / derive
// ============================================================================

// XPBD predict: save prev state, apply gravity, advance position and rotation
fn void predict_bodies(Rigidbody*[] bodies, Vec3 gravity, float dt) @private
{
	foreach (body : bodies)
	{
		body.save_previous();
		body.linear_velocity += gravity * body.gravity_factor * dt;
		body.collider.translation += body.linear_velocity * dt;

		Vec3 axis = body.angular_velocity * (dt * 0.5f);
		if (axis != { 0, 0, 0 })
		{
			body.collider.rotation = (body.collider.rotation
				+ (QuaternionValue) { ... axis, 0 } * body.collider.rotation).normalize();
		}
	}
}

// XPBD derive: recover velocities from position/rotation deltas, apply dampening.
// dmath::pow because libm pow is not cross-platform deterministic; the factor is
// cached per distinct dampening value, since most bodies inherit the world's.
fn void derive_bodies(Rigidbody*[] bodies, float dt) @private
{
	float inv_dt = 1.0f / dt;
	float last_angular = float::nan;
	float last_linear = float::nan;
	float angular_factor = 1;
	float linear_factor = 1;

	foreach (body : bodies)
	{
		body.derive_velocity(inv_dt);

		if (body.angular_dampening != last_angular)
		{
			last_angular = body.angular_dampening;
			angular_factor = dmath::pow(last_angular, dt);
		}
		if (body.linear_dampening != last_linear)
		{
			last_linear = body.linear_dampening;
			linear_factor = dmath::pow(last_linear, dt);
		}
		body.angular_velocity *= angular_factor;
		body.linear_velocity *= linear_factor;
	}
}

// Worker function for parallel XPBD predict phase
fn int predict_worker(void* arg) @private
{
	XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
	predict_bodies(task.bodies[:task.count], task.world.gravity, task.time);
	return 0;
}

// Worker function for parallel XPBD derive phase
fn int derive_worker(void* arg) @private
{
	XpbdPhaseTask* task = (XpbdPhaseTask*)arg;
	derive_bodies(task.bodies[:task.count], task.time);
	return 0;
}

// Worker function for island-parallel XPBD constraint solving
fn int island_solve_worker(void* arg) @private
{
	IslandSolveTask* task = (IslandSolveTask*)arg;

	for (uint iter = 0; iter < task.iterations; iter++)
	{
		solve_constraint_sweep(task.constraints[:task.count], iter, task.iterations);
	}
	return 0;
}

// ============================================================================
// Stepping
// ============================================================================

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
	self.seat_storage();

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

fn int trigger_event_cmp(TriggerEvent x, TriggerEvent y) @private
{
	if (x.trigger_id != y.trigger_id) return x.trigger_id < y.trigger_id ? -1 : 1;
	if (x.other_id != y.other_id) return x.other_id < y.other_id ? -1 : 1;
	if (x.type != y.type) return x.type.ordinal < y.type.ordinal ? -1 : 1;
	return 0;
}

fn int contact_event_cmp(ContactEvent x, ContactEvent y) @private
{
	if (x.body_a != y.body_a) return x.body_a < y.body_a ? -1 : 1;
	if (x.body_b != y.body_b) return x.body_b < y.body_b ? -1 : 1;
	if (x.type != y.type) return x.type.ordinal < y.type.ordinal ? -1 : 1;
	return 0;
}

fn void PhysicsWorld.run_substep(&self, float time, uint step_count)
{
	if (time <= 0.0f) return;

	// Events accumulate across substeps; remember where this substep's slice
	// starts so it can be canonically sorted.
	sz trigger_events_start = self.trigger_events.len();
	sz contact_events_start = self.contact_events.len();

	self.compute_islands();
	self.xpbd_predict(time);
	self.broadphase(trigger_events_start);
	self.narrow_phase();
	self.update_contact_events(contact_events_start);
	self.expand_manifolds();
	self.build_constraints(time);
	self.xpbd_solve_constraints(step_count);
	self.update_friction_anchors(time);
	self.collisions.clear();
	self.solve_soft_bodies(time);
	self.xpbd_derive_and_update(time);
	self.update_sleep_state();
}

// Decide what one overlapping broadphase pair is: a trigger overlap, a pair for
// the narrow phase, or a dormant contact carried over from last step.
fn void PhysicsWorld.register_pair(&self, Pair pair)
{
	Rigidbody* body_a = self.find_body(pair.a)!!;
	Rigidbody* body_b = self.find_body(pair.b)!!;

	// The spatial hash already drops pairs of fixed geometry; this catches a
	// body whose flags were changed after it was added.
	bool skip_static = body_a.is_fixed_geometry() && body_b.is_fixed_geometry();

	bool has_filter = skip_static || self.filters.has_key({ body_a.id, body_b.id });
	if (has_filter || !body_a.transformed_aabb.collides(body_b.transformed_aabb)) return;

	if (body_a.is_trigger || body_b.is_trigger)
	{
		// Normalize so the trigger is always first
		Pair trigger_pair = body_a.is_trigger ? pair : { pair.b, pair.a };
		self.triggers_next.set(trigger_pair, true);

		if (!self.active_triggers.has_key(trigger_pair))
		{
			self.trigger_events.push(
				{ .trigger_id = trigger_pair.a, .other_id = trigger_pair.b, .type = TriggerEventType.ENTER }
			);
		}
		return;
	}

	// Kinematic bodies are always "active" for broad phase (externally driven)
	bool a_active = body_a.is_active() || body_a.is_kinematic;
	bool b_active = body_b.is_active() || body_b.is_kinematic;

	// Wake sleeping bodies (sequential: waking changes what later pairs see)
	if (body_a.sleeping && b_active) self.wake_island(body_a);
	if (body_b.sleeping && a_active) self.wake_island(body_b);

	if (a_active || b_active)
	{
		self.candidate_pairs.push(pair);
	}
	else if (self.active_contacts.has_key(pair))
	{
		// Both ends asleep and touching last step, AABBs still overlapping: the
		// contact is as true as it was. Carry it without solving it, or every
		// resting contact would report END the moment its bodies sleep.
		self.dormant_pairs.push(pair);
	}
}

// Collect candidate and dormant pairs, trigger overlaps and trigger events.
fn void PhysicsWorld.broadphase(&self, sz trigger_events_start)
{
	self.candidate_pairs.clear();
	self.dormant_pairs.clear();

	self.spatial_map.@get_pairs(; Pair pair)
	{
		self.register_pair(pair);
	};

	@retire_overlaps(self.active_triggers, self.triggers_next; Pair old_pair)
	{
		self.trigger_events.push(
			{ .trigger_id = old_pair.a, .other_id = old_pair.b, .type = TriggerEventType.EXIT }
		);
	};

	// Pair order decides constraint order, and the Gauss-Seidel solve depends on it
	if (self.candidate_pairs.len() > 1)
	{
		sort::quicksort(self.candidate_pairs.array_view(), &pair_id_cmp);
	}
	if (self.trigger_events.len() - trigger_events_start > 1)
	{
		sort::quicksort(self.trigger_events.array_view()[trigger_events_start..], &trigger_event_cmp);
	}
}

// Split the candidate pairs by the test that handles them.
fn void PhysicsWorld.classify_pairs(&self)
{
	self.convex_pairs.clear();
	self.bvh_pairs.clear();
	self.field_pairs.clear();
	self.compound_pairs.clear();

	foreach (pair : self.candidate_pairs)
	{
		Rigidbody* body_a = self.find_body(pair.a)!!;
		Rigidbody* body_b = self.find_body(pair.b)!!;

		// Terrain first: two heightfields must not fall through to GJK.
		if (body_a.has_heightfield() || body_b.has_heightfield())
		{
			self.field_pairs.push(pair);
		}
		else if (body_a.has_mesh_bvh() || body_b.has_mesh_bvh())
		{
			self.bvh_pairs.push(pair);
		}
		else if (body_a.has_compound() || body_b.has_compound())
		{
			self.compound_pairs.push(pair);
		}
		else
		{
			self.convex_pairs.push(pair);
		}
	}
}

// Narrow phase: convex, BVH and terrain pairs in one parallel dispatch, then
// compound pairs serially.
fn void PhysicsWorld.narrow_phase(&self)
{
	if (self.candidate_pairs.len() == 0) return;
	self.classify_pairs();

	// One dispatch (one barrier) for every parallel-safe pair kind. Terrain
	// pairs are a few hundred triangles per contact and touch no shared state.
	PairGroup[3] groups = {
		{ self.convex_pairs.array_view(), &test_convex_collision },
		{ self.bvh_pairs.array_view(), &PhysicsWorld.test_bvh_collision },
		{ self.field_pairs.array_view(), &PhysicsWorld.test_heightfield_collision },
	};
	self.dispatch_collision_tasks(&groups);

	foreach (pair : self.compound_pairs)
	{
		Rigidbody* body_a = self.find_body(pair.a)!!;
		Rigidbody* body_b = self.find_body(pair.b)!!;
		self.check_coacd_collision(body_a, body_b, pair);
	}
}

// START events for new contacts, END events for contacts that ended.
fn void PhysicsWorld.update_contact_events(&self, sz events_start)
{
	// Dormant contacts first, so the END sweep sees them as present. No START can
	// come of one: each was already in `active_contacts`, the START test itself.
	foreach (pair : self.dormant_pairs)
	{
		self.contacts_next.set(pair, true);
	}

	foreach (&collision : self.collisions)
	{
		Pair contact_pair = { collision.a, collision.b };
		self.contacts_next.set(contact_pair, true);

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

	@retire_overlaps(self.active_contacts, self.contacts_next; Pair old_pair)
	{
		self.contact_events.push(
			{ .body_a = old_pair.a, .body_b = old_pair.b, .type = ContactEventType.END }
		);
	};

	// END events come from hash-map iteration
	if (self.contact_events.len() - events_start > 1)
	{
		sort::quicksort(self.contact_events.array_view()[events_start..], &contact_event_cmp);
	}
}

// EPA gives one contact per pair; box and mesh contacts are expanded into a
// manifold by projecting vertices near the contact plane.
fn void PhysicsWorld.expand_manifolds(&self)
{
	if (self.collisions.len() == 0) return;

	@swap(self.collisions, self.manifold_scratch);
	self.collisions.clear();
	foreach (&collision : self.manifold_scratch)
	{
		self.generate_manifold(collision);
	}
	self.manifold_scratch.clear();
}

// Body b's pose in body a's frame.
fn FrictionAnchor relative_pose(Vec3 a_at, QuaternionValue a_rot, Vec3 b_at, QuaternionValue b_rot) @private
{
	QuaternionValue to_a = a_rot.conjugate();
	return { .rotation = to_a * b_rot, .translation = to_a * (b_at - a_at) };
}

// Shapes whose contact point travels over them as they move: an anchor would hold
// a rolling ball's old contact point and brake it, so theirs is re-taken every substep.
fn bool Rigidbody.rolls(&self)
{
	switch (self.collider.get_type())
	{
		case SPHERE:
		case CAPSULE:
		case CYLINDER:
			return true;
		default:
			return false;
	}
}

<*
 One XPBD constraint per contact, then one per joint axis.

 A pair's contacts are adjacent (pairs are narrow-phased in sorted order), so
 its bodies and its friction anchor are looked up once per pair. A pair without
 an anchor is held where it was at the start of the substep, which is plain
 per-substep static friction. The anchor used is carried into `anchors_next`;
 `update_friction_anchors` replaces it unless the pair stayed at rest.
*>
fn void PhysicsWorld.build_constraints(&self, float time)
{
	self.anchors_next.clear();
	Collision[] collisions = self.collisions.array_view();
	sz i = 0;
	while (i < collisions.len)
	{
		Pair pair = { collisions[i].a, collisions[i].b };
		sz end = i + 1;
		while (end < collisions.len && collisions[end].a == pair.a && collisions[end].b == pair.b) end++;

		Rigidbody* a = self.find_body(pair.a)!!;
		Rigidbody* b = self.find_body(pair.b)!!;
		bool anchored = !a.rolls() && !b.rolls();

		FrictionAnchor? kept = anchored ? self.friction_anchors.get(pair) : NOT_FOUND~;
		FrictionAnchor anchor = kept ?? relative_pose(a.prev_position, a.prev_rotation, b.prev_position, b.prev_rotation);
		if (anchored) self.anchors_next.set(pair, anchor);

		for (sz k = i; k < end; k++) collisions[k].create_xpbd_constraint(self, a, b, anchor);
		i = end;
	}

	foreach (&joint : self.joints)
	{
		joint.create_xpbd_constraint(self, time);
	}
}

// Run an XPBD phase worker over the active bodies: batched onto the thread
// pool when there are enough bodies, otherwise inline as a single batch.
fn void PhysicsWorld.dispatch_phase_tasks(&self, float dt, ThreadFn worker_fn)
{
	sz num_active = self.active_bodies.len();
	Rigidbody** all_bodies_ptr = self.active_bodies.array_view().ptr;
	sz workers = self.thread_pool.workers();

	if (num_active < workers * PHASE_TASKS_PER_WORKER)
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

 Obstacles are gathered once per call (and only if some body is fast), then
 rejected per body by the AABB of its motion segment before any ray is cast.

 Limitations: rotation during the sweep is ignored, dynamic-vs-dynamic pairs
 are not swept, and kinematic movers are treated as stationary obstacles.
*>
fn void PhysicsWorld.apply_ccd(&self)
{
	bool obstacles_ready = false;

	foreach (body : self.active_bodies)
	{
		float radius = body.ccd_radius;
		if (radius <= 0) continue;

		Vec3 disp = body.collider.translation - body.prev_position;
		float disp_sq = disp.length_sq();
		if (disp_sq <= radius * radius) continue;

		if (!obstacles_ready)
		{
			self.ccd_obstacles.clear();
			self.@each_body(; Rigidbody* obstacle)
			{
				if (obstacle.is_static && !obstacle.is_trigger) self.ccd_obstacles.push(obstacle);
			};
			obstacles_ready = true;
		}

		Aabb3 sweep = {
			.min = math::min(body.prev_position, body.collider.translation),
			.max = math::max(body.prev_position, body.collider.translation),
		};

		// All hit fractions are along the motion segment: 0 = prev, 1 = predicted
		float best_fraction = float::max;

		foreach (obstacle : self.ccd_obstacles)
		{
			if (!sweep.collides(obstacle.transformed_aabb)) continue;

			HitInfo hit;
			if (obstacle.has_mesh_bvh())
			{
				// Cast in mesh-local space; the affine transform preserves
				// fractions along the segment.
				Mesh* mesh = (Mesh*)obstacle.collider.shape;
				Vec3 local_start = obstacle.collider.to_local(body.prev_position);
				Vec3 local_end = obstacle.collider.to_local(body.collider.translation);
				hit = mesh.bvh.ray_intersects({ .origin = local_start, .direction = local_end - local_start });
			}
			else
			{
				// Conservative: cast against the obstacle's world AABB.
				// depth == 0 means we started inside — that's a resting
				// overlap for the narrow phase, not a CCD hit.
				hit = obstacle.transformed_aabb.intersect_ray({ .origin = body.prev_position, .direction = disp });
			}
			if (hit.collided && hit.depth > 0 && hit.depth <= 1.0f && hit.depth < best_fraction)
			{
				best_fraction = hit.depth;
			}
		}

		if (best_fraction < float::max)
		{
			// Stop just short of the impact point so the next narrow phase
			// generates a regular contact at the surface
			float back_off = radius * CCD_BACKOFF / math::sqrt(disp_sq);
			float clamped = math::max(0.0f, best_fraction - back_off);
			body.collider.translation = body.prev_position + disp * clamped;
		}
	}
}

// XPBD predict phase: save prev state, apply gravity, advance positions
fn void PhysicsWorld.xpbd_predict(&self, float dt)
{
	self.active_bodies.clear();
	self.@each_body(; Rigidbody* body)
	{
		if (body.is_active()) self.active_bodies.push(body);
	};

	if (self.active_bodies.len() == 0) return;

	self.dispatch_phase_tasks(dt, &predict_worker);

	// Clamp fast movers to their first time of impact before the broad phase
	if (self.ccd_enabled) self.apply_ccd();

	// Sequential: the spatial hash must hold the predicted AABBs before the
	// broad phase. A non-finite prediction is discarded here (see
	// `Rigidbody.transform_is_finite`) so the world keeps stepping.
	foreach (body : self.active_bodies)
	{
		if (!body.transform_is_finite())
		{
			body.reset_to_previous();
			continue;
		}
		self.refresh_broadphase(body);
	}
}

// Post-derive velocity correction at contacts: zero normal velocity for an
// inelastic contact, -e * pre-solve normal velocity for a bouncing one.
fn void PhysicsWorld.apply_contact_restitution(&self)
{
	foreach (&c : self.constraints)
	{
		if (c.type != CONTACT_NORMAL) continue;
		if (c.lambda_normal < EPSILON) continue;

		Rigidbody* a = c.body_a;
		Rigidbody* b = c.body_b;

		Vec3 va = a.relative_velocity(c.r_a);
		Vec3 vb = b.relative_velocity(c.r_b);
		float vn = (va - vb).dot(c.normal);

		float target_vn = 0;
		float vn_pre = c.pre_solve_relative_vel_n;
		if (c.restitution > MIN_RESTITUTION && math::abs(vn_pre) > BOUNCE_THRESHOLD)
		{
			target_vn = -c.restitution * vn_pre;
		}

		// Reaches the target from both residual approach and overshoot
		float dv = target_vn - vn;
		if (math::abs(dv) <= EPSILON) continue;

		float w_sum = pair_inverse_mass(a, c.r_a, b, c.r_b, c.normal);
		if (w_sum <= WEIGHT_EPSILON) continue;

		float impulse_mag = dv / w_sum;
		if (!a.is_static) a.apply_impulse(c.normal * impulse_mag, c.r_a);
		if (!b.is_static) b.apply_impulse(-c.normal * impulse_mag, c.r_b);
	}
}

// XPBD derive phase: compute velocities from position deltas, restitution, update spatial map
fn void PhysicsWorld.xpbd_derive_and_update(&self, float dt)
{
	if (self.active_bodies.len() == 0) return;

	self.dispatch_phase_tasks(dt, &derive_worker);
	self.apply_contact_restitution();
	self.constraints.clear();

	// Sequential: update spatial map and sleep timers
	foreach (body : self.active_bodies)
	{
		bool rotating = body.is_rotating();
		bool moving = body.is_moving();

		if (rotating || moving) self.refresh_broadphase(body);

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

// Root of `body`'s island in this substep's constraint graph, compressing the path.
fn Rigidbody* solve_root(Rigidbody* body) @private
{
	Rigidbody* root = body;
	while (root.solve_parent != null) root = root.solve_parent;
	while (body != root)
	{
		Rigidbody* next = body.solve_parent;
		body.solve_parent = root;
		body = next;
	}
	return root;
}

// Merge two islands; the smaller body id becomes the root, as in `UnionFindMap.merge`.
fn void solve_merge(Rigidbody* a, Rigidbody* b) @private
{
	Rigidbody* root_a = solve_root(a);
	Rigidbody* root_b = solve_root(b);
	if (root_a == root_b) return;
	if (root_a.id < root_b.id)
	{
		root_b.solve_parent = root_a;
	}
	else
	{
		root_a.solve_parent = root_b;
	}
}

// Island-parallel XPBD constraint solving
fn void PhysicsWorld.xpbd_solve_constraints(&self, uint step_count)
{
	sz constraint_count = self.constraints.len();
	if (constraint_count == 0) return;

	// Tiny workloads aren't worth the grouping and thread pool overhead
	if (constraint_count <= SERIAL_CONSTRAINT_LIMIT)
	{
		XpbdConstraint*[SERIAL_CONSTRAINT_LIMIT] serial;
		foreach (i, &constraint : self.constraints) serial[i] = constraint;
		for (uint i = 0; i < step_count; i++)
		{
			solve_constraint_sweep(serial[:constraint_count], i, step_count);
		}
		return;
	}

	// Islands must reflect *this substep's* constraint graph: two constraints
	// sharing a dynamic body have to solve on the same thread. Static bodies
	// are never written by the solver, so they don't merge islands.
	foreach (&constraint : self.constraints)
	{
		constraint.body_a.solve_parent = null;
		constraint.body_a.solve_bucket = -1;
		constraint.body_b.solve_parent = null;
		constraint.body_b.solve_bucket = -1;
	}
	foreach (&constraint : self.constraints)
	{
		if (!constraint.body_a.is_static && !constraint.body_b.is_static)
		{
			solve_merge(constraint.body_a, constraint.body_b);
		}
	}

	// Pack islands into at most one bucket per worker. Islands touch disjoint
	// bodies, so one Gauss-Seidel sweep over a bucket's combined constraint
	// list is equivalent to solving its islands independently, and bounding
	// the task count keeps thread pool overhead out of the hot path.
	sz workers = self.thread_pool.workers();
	Allocator allocator = self.storage();
	ConstraintList[] buckets = self.constraint_buckets[:workers];
	foreach (&bucket : buckets)
	{
		@seat(*bucket, allocator);
		bucket.clear();
	}

	foreach (&constraint : self.constraints)
	{
		Rigidbody* anchor = !constraint.body_a.is_static ? constraint.body_a : constraint.body_b;
		Rigidbody* root = solve_root(anchor);

		if (root.solve_bucket < 0)
		{
			// New island: assign to the currently smallest bucket
			sz smallest = 0;
			for (sz i = 1; i < workers; i++)
			{
				if (buckets[i].len() < buckets[smallest].len()) smallest = i;
			}
			root.solve_bucket = (int)smallest;
		}
		buckets[root.solve_bucket].push(constraint);
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
fn int softbody_solve_worker(void* arg) @private
{
	SoftBodySolveTask* task = (SoftBodySolveTask*)arg;
	SoftBody* sb = task.body;

	if (task.ground_enabled)
	{
		sb.generate_ground_collisions(task.ground_plane, task.ground_friction);
	}

	// Rigidbody collisions are generated serially beforehand — not thread-safe
	sb.xpbd_solve(task.gravity, task.time, task.world);

	return 0;
}

// XPBD soft body solving phase
fn void PhysicsWorld.solve_soft_bodies(&self, float time)
{
	if (self.soft_bodies.len() == 0) return;

	// Sequential: generate rigidbody collision constraints (needs shared body
	// access). The AABB reject here saves the call for the usual non-overlap.
	foreach (&sb : self.soft_bodies)
	{
		if (sb.sleeping) continue;

		self.@each_body(; Rigidbody* body)
		{
			if (sb.aabb.collides(body.transformed_aabb)) sb.generate_rigidbody_collisions(body);
		};
	}

	// Soft bodies coupled to rigidbodies mutate those bodies while solving, so
	// they run sequentially; uncoupled ones solve in parallel.
	self.soft_tasks.clear();
	self.coupled_soft.clear();

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
			self.coupled_soft.push(sb);
			continue;
		}

		self.soft_tasks.push(
			{
				.body = sb,
				.world = self,
				.gravity = self.gravity,
				.time = time,
				.ground_plane = self.ground_plane,
				.ground_enabled = self.ground_plane_enabled,
				.ground_friction = sb.material.static_friction,
			}
		);
	}

	switch (self.soft_tasks.len())
	{
		case 0:
			break;
		case 1:
			// Single soft body — solve inline, skip thread pool overhead
			softbody_solve_worker(&self.soft_tasks[0]);
		default:
			for (sz i = 0; i < self.soft_tasks.len(); i++)
			{
				self.thread_pool.push(&softbody_solve_worker, &self.soft_tasks[i]);
			}
			self.thread_pool.join();
	}

	foreach (sb : self.coupled_soft)
	{
		if (self.ground_plane_enabled)
		{
			sb.generate_ground_collisions(self.ground_plane, sb.material.static_friction);
		}
		sb.xpbd_solve(self.gravity, time, self);
	}

	foreach (&sb : self.soft_bodies)
	{
		if (sb.sleeping || !sb.can_sleep) continue;

		if (!sb.is_moving())
		{
			sb.sleep_timer += time;
			if (sb.sleep_timer >= self.sleep_timer) sb.sleeping = true;
		}
		else
		{
			sb.sleep_timer = 0;
		}
	}
}

// Put whole islands to sleep once every non-static member has been still long
// enough. Walks the island-sorted member list run by run, so it allocates nothing.
fn void PhysicsWorld.update_sleep_state(&self)
{
	if (self.members_dirty) self.rebuild_island_members();
	Rigidbody*[] members = self.island_members.array_view();

	sz start = 0;
	while (start < members.len)
	{
		sz island = members[start].island_id;
		bool can_sleep = true;
		sz end = start;
		while (end < members.len && members[end].island_id == island)
		{
			Rigidbody* body = members[end];
			if (!body.can_sleep || body.is_moving() || body.is_rotating() || body.sleep_timer < self.sleep_timer)
			{
				can_sleep = false;
			}
			end++;
		}

		if (can_sleep)
		{
			for (sz i = start; i < end; i++)
			{
				Rigidbody* body = members[i];
				if (body.sleeping) continue;
				body.sleeping = true;
				body.sleep_timer = 0;
				// `wake_island` does not clear residual creep, so zero it here or
				// the body resumes at that speed when touched.
				body.linear_velocity = { 0, 0, 0 };
				body.angular_velocity = { 0, 0, 0 };
			}
		}
		start = end;
	}
}

// ============================================================================
// Contact constraints
// ============================================================================

fn PhysicsCombine max_combine(PhysicsCombine a, PhysicsCombine b) @private => a.ordinal > b.ordinal ? a : b;

fn float combine_values(float a, float b, PhysicsCombine mode) @private
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

fn void Collision.create_xpbd_constraint(&collision, PhysicsWorld* world, Rigidbody* a, Rigidbody* b, FrictionAnchor anchor)
{
	Vec3 normal = collision.normal.normalize();

	// Combine material properties (higher-priority combine mode wins)
	PhysicsCombine friction_mode = max_combine(a.material.friction_combine, b.material.friction_combine);
	PhysicsCombine restitution_mode = max_combine(a.material.restitution_combine, b.material.restitution_combine);

	float restitution = combine_values(a.material.restitution, b.material.restitution, restitution_mode);
	float static_friction = combine_values(a.material.static_friction, b.material.static_friction, friction_mode);

	// Contact arms (world-space offset from body center), also kept in local
	// space so the solver can recompute them after the bodies move
	Vec3 rA = collision.contact[0] - a.collider.translation;
	Vec3 rB = collision.contact[1] - b.collider.translation;
	Vec3 rA_local = a.collider.rotation.conjugate() * rA;
	Vec3 rB_local = b.collider.rotation.conjugate() * rB;

	// Tangent basis
	Vec3 helper = (math::abs(normal.y) > TANGENT_HELPER_DOT) ? { 1.0f, 0.0f, 0.0f } : { 0.0f, 1.0f, 0.0f };
	Vec3 tangent_1 = normal.cross(helper).normalize();
	Vec3 tangent_2 = normal.cross(tangent_1);

	// Pre-solve relative velocity for restitution (before XPBD changes positions)
	Vec3 va = a.relative_velocity(rA);
	Vec3 vb = b.relative_velocity(rB);
	float rel_vel_n = (va - vb).dot(normal);

	world.constraints.push(
		{
			.body_a = a,
			.body_b = b,
			.normal = normal,
			.r_a = rA,
			.r_b = rB,
			.r_a_local = rA_local,
			.r_b_local = rB_local,
			.c_value = 0, // Contact points are on body surfaces; separation already encodes penetration
			.type = CONTACT_NORMAL,
			.friction = static_friction,
			.tangent1 = tangent_1,
			.tangent2 = tangent_2,
			.restitution = restitution,
			.pre_solve_relative_vel_n = rel_vel_n,
			.friction_anchor = anchor.translation + anchor.rotation * rB_local,
			.manifold_len = collision.manifold_len,
		}
	);
}

<*
 Carry each pair's friction anchor into the next substep only while static
 friction held a pair at rest: one that carried no load, used up its friction
 cone, or whose bodies moved apart faster than ANCHOR_REST_SPEED this substep is
 re-anchored at its current pose. Then `anchors_next` becomes the live set;
 pairs no longer in contact were never written to it and drop out.

 The cone is judged for the pair as a whole. Within a sweep the first point of a
 manifold corrects most of the slip and clamps at its share of the load while
 the pair is still well inside its total, so a per-point test would re-anchor a
 resting box every substep.

 The speed test keeps anchors to resting contact. A pair knocked apart (a crate
 spawned overlapping its support, say) would otherwise be pulled back towards
 the pose it was knocked out of, which turns the separation into a spin.
*>
fn void PhysicsWorld.update_friction_anchors(&self, float dt)
{
	float rest_step = ANCHOR_REST_SPEED * dt; // Relative motion per substep still counted as rest

	XpbdConstraint[] cs = self.constraints.array_view();
	sz i = 0;
	while (i < cs.len)
	{
		XpbdConstraint* c = &cs[i];
		if (c.type != CONTACT_NORMAL)
		{
			i++;
			continue;
		}

		Rigidbody* a = c.body_a;
		Rigidbody* b = c.body_b;
		float used = 0;
		float cone = 0;
		while (i < cs.len && cs[i].type == CONTACT_NORMAL && cs[i].body_a == a && cs[i].body_b == b)
		{
			XpbdConstraint* p = &cs[i];
			used += math::sqrt(p.lambda_tangent1 * p.lambda_tangent1 + p.lambda_tangent2 * p.lambda_tangent2);
			cone += p.friction * p.lambda_normal;
			i++;
		}

		if (a.rolls() || b.rolls()) continue;

		FrictionAnchor now = relative_pose(a.collider.translation, a.collider.rotation, b.collider.translation, b.collider.rotation);
		bool held = cone > EPSILON && used < STICK_FRACTION * cone;
		if (held)
		{
			FrictionAnchor before = relative_pose(a.prev_position, a.prev_rotation, b.prev_position, b.prev_rotation);
			QuaternionValue turn = now.rotation * before.rotation.conjugate();
			// |xyz| of a unit quaternion is sin(angle / 2): twice it is the angle, near rest
			float angle = 2.0f * turn.v.xyz.length();
			held = (now.translation - before.translation).length() <= rest_step && angle <= rest_step;
		}
		if (!held) self.anchors_next.set({ a.id, b.id }, now);
	}

	@swap(self.friction_anchors, self.anchors_next);
	self.anchors_next.clear();
}

module collision;

import std::collections::list;
import std::collections::map;
import std::math;
import std::io;


import collision::dmath;

// Lengths below this are treated as zero: the direction they would define is noise.
const float MIN_LENGTH @private = 1e-7f;
const float VOLUME_EPS @private = 1e-7f; // Volume error treated as satisfied
const float ANGLE_EPS @private = 1e-6f; // Dihedral error (radians) treated as satisfied
const float GRAD_SQ_EPS @private = 1e-10f; // Weighted squared gradient below which a step is skipped
const float FRICTION_SPEED_EPS @private = 1e-6f; // Tangential speed treated as zero
const float W_TOTAL_EPS @private = 1e-10f; // Combined inverse mass below which a contact is skipped
const float DEFAULT_BENDING_COMPLIANCE @private = 0.001f; // Bending is usually softer
// Particle speed (model units/s) under which a body counts as at rest
const float SLEEP_SPEED @private = 0.1f;
const float SLEEP_SPEED_SQ @private = SLEEP_SPEED * SLEEP_SPEED;

// XPBD constraint types for soft bodies. `lambda` is the accumulated Lagrange
// multiplier; it restarts from zero every substep.
struct DistanceConstraint
{
	sz p0; // Particle index 0
	sz p1; // Particle index 1
	float rest_length;
	float compliance; // Inverse stiffness (0 = infinitely stiff)
	float lambda;
}

struct VolumeConstraint
{
	float rest_volume;
	float compliance;
	float lambda;
}

struct BendingConstraint
{
	sz p0; // Shared edge vertex 0
	sz p1; // Shared edge vertex 1
	sz p2; // Triangle 1 opposite vertex
	sz p3; // Triangle 2 opposite vertex
	float rest_angle; // Rest dihedral angle
	float compliance;
	float lambda;
}

// Pull a particle toward a world-space point the host moves between substeps.
// One-sided: the target is immovable, so this is a spring to an external goal
// rather than a link between two particles (that is DistanceConstraint).
struct TargetConstraint
{
	sz particle; // Particle index
	Vec3 target; // World-space goal
	float compliance; // Inverse stiffness (0 = snap onto the target)
	float lambda;
}

// Cone limit: keeps (p_child - p_pivot) within max_angle of reference_dir.
// The reference direction is world space and owned by the host, which refreshes
// it every substep when the cone follows an animated bone.
struct SwingConstraint
{
	sz p_pivot; // Particle the cone is anchored at
	sz p_child; // Particle held inside the cone
	Vec3 reference_dir; // Cone axis, world space (need not be normalized)
	float max_angle; // Cone half-angle in radians; <= 0 pins the child to the axis
	float compliance; // Inverse stiffness (0 = hard clamp)
	float lambda;
}

// Collision constraint generated per-frame during soft-rigid narrow phase
struct SoftCollisionConstraint
{
	sz particle; // Particle index
	Vec3 normal; // Contact normal (pointing away from rigidbody)
	Vec3 contact_point; // Contact point on rigidbody surface
	sz rigidbody_id; // Rigidbody involved (0 = ground plane)
	float friction; // Combined friction coefficient
	Rigidbody* body; // Resolved from rigidbody_id at the start of each xpbd_solve; stale outside it
}

// Particle pinning: fix a particle to a world position or rigidbody
struct ParticlePin
{
	sz particle; // Particle index
	Vec3 local_offset; // Offset in rigidbody local space (or world pos if body_id == 0)
	sz body_id; // 0 = world-space pin, otherwise rigidbody id
	Rigidbody* body; // Resolved from body_id at the start of each xpbd_solve; stale outside it
}

alias SoftBodyList = List {SoftBody};

struct SoftBody
{
	sz id;

	// Per-particle data
	List {Vec3} positions;
	List {Vec3} prev_positions; // For XPBD velocity derivation
	List {Vec3} velocities;
	List {float} inverse_masses;

	// Optional per-particle overrides. Both are empty by default and stay
	// empty unless a caller fills them, which is what keeps the plain
	// mesh soft body behaving exactly as it always did.
	// particle_damping: fraction of velocity kept per *second*, in [0, 1].
	//   Set entries are applied dt-normalized (pow(d, sub_dt)) like the rigid
	//   path does; unset particles fall back to the whole-body `damping`
	//   with its original per-substep meaning.
	// particle_radius: collision radius of the particle, model units. Unset
	//   particles are points. Read by generate_capsule_collisions.
	List {float} particle_damping;
	List {float} particle_radius;

	// pow(particle_damping[i], sub_dt), refreshed at the start of each solve.
	List {float} damping_factors;

	// Surface mesh connectivity
	List {Triangle} triangles;

	// Constraints
	List {DistanceConstraint} distance_constraints;
	List {TargetConstraint} target_constraints;
	List {SwingConstraint} swing_constraints;
	List {VolumeConstraint} volume_constraints;
	List {BendingConstraint} bending_constraints;
	List {SoftCollisionConstraint} collision_constraints;
	List {ParticlePin} pins;

	// Scratch for the volume constraint's per-particle gradients. Owned here, not
	// local, because the solve runs on a worker thread with no temp allocator
	// (an unseated list would abort there); it also saves an alloc per substep.
	List {Vec3} volume_gradients;

	// Per-body properties
	Material material;
	float gravity_factor;
	float damping;

	// XPBD solver settings
	uint substeps;
	uint constraint_iterations;

	// Apply pins before each substep's prediction as well as after its constraint
	// sweep. Off by default; without it, anything hanging off a pin the host
	// moves between substeps trails it by one substep.
	bool pins_first;

	// AABB for broad phase
	Aabb3 aabb;

	// Sleep state
	float sleep_timer;
	bool sleeping;
	bool can_sleep;
}

fn void SoftBody.free(&self)
{
	self.positions.free();
	self.prev_positions.free();
	self.velocities.free();
	self.inverse_masses.free();
	self.particle_damping.free();
	self.particle_radius.free();
	self.damping_factors.free();
	self.triangles.free();
	self.distance_constraints.free();
	self.target_constraints.free();
	self.swing_constraints.free();
	self.volume_constraints.free();
	self.bending_constraints.free();
	self.collision_constraints.free();
	self.pins.free();
	self.volume_gradients.free();
}

<*
 Seat every list the body owns on `allocator`, including the ones filled later
 by the host (pins, targets, swings, per-substep collision constraints).

 A `List` never given an allocator takes the *temp* allocator on its first push,
 and a soft body routinely outlives the temp scope it was built in. Every
 constructor calls this.

 @param allocator : `Owns every list in the body; SoftBody.free returns them to it.`
 @param particles : `Expected particle count, used as the initial capacity.`
*>
fn void SoftBody.init(&self, Allocator allocator, sz particles = 16)
{
	sz slots = particles > 0 ? particles : 1;
	self.positions.init(allocator, slots);
	self.prev_positions.init(allocator, slots);
	self.velocities.init(allocator, slots);
	self.inverse_masses.init(allocator, slots);
	self.particle_damping.init(allocator, slots);
	self.particle_radius.init(allocator, slots);
	self.damping_factors.init(allocator, 1);
	self.triangles.init(allocator, 1);
	self.distance_constraints.init(allocator, slots);
	self.target_constraints.init(allocator, slots);
	self.swing_constraints.init(allocator, slots);
	self.volume_constraints.init(allocator, 1);
	self.bending_constraints.init(allocator, 1);
	self.collision_constraints.init(allocator, slots);
	self.pins.init(allocator, 1);
	self.volume_gradients.init(allocator, slots);
}

<*
 Solver defaults shared by every constructor. Call before `init`.
*>
fn void SoftBody.set_defaults(&self, sz id)
{
	self.id = id;
	self.gravity_factor = 1.0;
	self.damping = 0.99;
	self.substeps = 4;
	self.constraint_iterations = 8;
	self.can_sleep = true;
	self.material = { .static_friction = 0.5, .dynamic_friction = 0.3, .restitution = 0.2, };
}

<*
 Append a particle at rest at `p`. `inv_mass` 0 makes it immovable.
*>
fn void SoftBody.add_particle(&self, Vec3 p, float inv_mass)
{
	self.positions.push(p);
	self.prev_positions.push(p);
	self.velocities.push({ 0, 0, 0 });
	self.inverse_masses.push(inv_mass);
}

<*
 Join particles `a` and `b` with a distance constraint whose rest length is
 their current separation.
*>
fn void SoftBody.add_distance(&self, sz a, sz b, float compliance = 0.0)
{
	float rest_len = (self.positions[b] - self.positions[a]).length();
	self.distance_constraints.push({ .p0 = a, .p1 = b, .rest_length = rest_len, .compliance = compliance, .lambda = 0, });
}

<*
 Create a soft body from a mesh with the given particle mass.

 @param allocator : `Owns every list in the body; SoftBody.free returns them to it.`
*>
fn SoftBody create_softbody_from_mesh(Mesh* mesh, float total_mass, sz id, Allocator allocator = mem)
{
	SoftBody sb;
	sb.set_defaults(id);
	sb.init(allocator, mesh.vertices.len());

	float particle_mass = total_mass / (float)mesh.vertices.len();
	float inv_mass = particle_mass > 0 ? 1.0 / particle_mass : 0;

	foreach (v : mesh.vertices) sb.add_particle(v, inv_mass);
	foreach (tri : mesh.triangles) sb.triangles.push(tri);

	sb.extract_distance_constraints();
	sb.aabb = sb.compute_aabb();
	return sb;
}

<*
 Create a soft body that is a chain of world-space points: particle i joined to
 particle i+1 by a distance constraint whose rest length is the distance between
 the two points as given. No triangles, so no volume, bending or
 triangle-vs-rigidbody collision — use generate_capsule_collisions for contact.

 This is the shape a tail, a rope, a hair strand or a jiggle bone run has, and
 the only constructor that does not need a mesh. Solver defaults match
 create_softbody_from_mesh so a chain tunes the same way as any other body; the
 caller pins whichever particles the host drives (usually the root) with
 pin_particle.

 Every list the body holds is seated on `allocator` up front by `SoftBody.init`,
 which says there why leaving that to chance is not survivable.

 @param allocator : `Owns every list in the body; SoftBody.free returns them to it.`
 @param points : `Particle positions in world space, in chain order.`
 @param inverse_masses : `One per point; 0 makes that particle immovable.`
 @require points.len > 0 : `create_particle_chain needs at least one point`
 @require points.len == inverse_masses.len : `create_particle_chain needs one inverse mass per point`
*>
fn SoftBody create_particle_chain(Allocator allocator = mem, Vec3[] points, float[] inverse_masses, sz id)
{
	SoftBody sb;
	sb.set_defaults(id);
	sb.init(allocator, points.len);

	foreach (sz i, p : points) sb.add_particle(p, inverse_masses[i]);
	for (sz i = 1; i < points.len; i++) sb.add_distance(i - 1, i);

	sb.aabb = sb.compute_aabb();
	return sb;
}

<*
 Calls `@body(a, b)` once for each of the triangle's three edges, as particle indices.
*>
macro void Triangle.@each_edge(self; @body(a, b))
{
	@body((sz)self.x, (sz)self.y);
	@body((sz)self.y, (sz)self.z);
	@body((sz)self.z, (sz)self.x);
}

fn void SoftBody.extract_distance_constraints(&self)
{
	// Pair hashes and compares order-independently, so (a, b) and (b, a) are one edge.
	HashMap {Pair, bool} seen_edges;
	defer seen_edges.free();

	foreach (tri : self.triangles)
	{
		tri.@each_edge(; sz a, sz b)
		{
			if (!seen_edges.has_key({ a, b }))
			{
				seen_edges.set({ a, b }, true);
				self.add_distance(a, b);
			}
		};
	}
}

// The (up to two) triangles on one edge.
struct EdgeUse @local
{
	sz[2] tris;
	sz count;
}

// Create bending constraints for every edge shared by exactly two triangles.
fn void SoftBody.extract_bending_constraints(&self)
{
	HashMap {Pair, EdgeUse} edges;
	defer edges.free();

	for (sz ti = 0; ti < self.triangles.len(); ti++)
	{
		self.triangles[ti].@each_edge(; sz a, sz b)
		{
			EdgeUse* use = edges.get_ref({ a, b }) ?? null;
			if (!use)
			{
				edges.set({ a, b }, { .tris = { ti, 0 }, .count = 1 });
			}
			else if (use.count < 2)
			{
				use.tris[use.count++] = ti;
			}
		};
	}

	// No early `return` in the body: inside a macro body it returns from this
	// function, which would stop at the first edge that is not shared.
	edges.@each(; Pair key, EdgeUse use)
	{
		if (use.count == 2)
		{
			sz opposite0 = find_opposite_vertex(self.triangles[use.tris[0]], key.a, key.b);
			sz opposite1 = find_opposite_vertex(self.triangles[use.tris[1]], key.a, key.b);

			self.bending_constraints.push(
				{
					.p0 = key.a,
					.p1 = key.b,
					.p2 = opposite0,
					.p3 = opposite1,
					.rest_angle = self.compute_dihedral_angle(key.a, key.b, opposite0, opposite1),
					.compliance = DEFAULT_BENDING_COMPLIANCE,
					.lambda = 0,
				}
			);
		}
	};
}

fn sz find_opposite_vertex(Triangle tri, sz edge_a, sz edge_b) @local
{
	if (tri.x != edge_a && tri.x != edge_b) return tri.x;
	if (tri.y != edge_a && tri.y != edge_b) return tri.y;
	return tri.z;
}

// Dihedral angle of the hinge p0-p1 between faces (p0, p2, p1) and (p0, p1, p3).
struct Dihedral @local
{
	Vec3 n0; // Unit normal of the first face
	Vec3 n1; // Unit normal of the second face
	float angle; // Signed, in (-pi, pi]; 0 when degenerate
	bool valid; // False if the edge or either face has no area
}

fn Dihedral dihedral(Vec3 p0, Vec3 p1, Vec3 p2, Vec3 p3) @local
{
	Vec3 edge = p1 - p0;
	float edge_len = edge.length();
	Vec3 n0 = (p2 - p0).cross(edge);
	Vec3 n1 = edge.cross(p3 - p0);
	float n0_len = n0.length();
	float n1_len = n1.length();
	if (edge_len < MIN_LENGTH || n0_len < MIN_LENGTH || n1_len < MIN_LENGTH) return {};

	n0 /= n0_len;
	n1 /= n1_len;
	float cos_angle = math::clamp(n0.dot(n1), -1.0f, 1.0f);
	float sin_angle = n0.cross(n1).dot(edge / edge_len);

	// dmath::atan2: libm atan2 is not cross-platform deterministic
	return { .n0 = n0, .n1 = n1, .angle = dmath::atan2(sin_angle, cos_angle), .valid = true };
}

fn float SoftBody.compute_dihedral_angle(&self, sz e0, sz e1, sz opp0, sz opp1)
{
	return dihedral(self.positions[e0], self.positions[e1], self.positions[opp0], self.positions[opp1]).angle;
}

// Compute AABB from current particle positions
fn Aabb3 SoftBody.compute_aabb(&self)
{
	if (self.positions.len() == 0) return EMPTY_AABB3;

	Vec3 min_v = self.positions[0];
	Vec3 max_v = self.positions[0];

	for (sz i = 1; i < self.positions.len(); i++)
	{
		Vec3 p = self.positions[i];
		min_v = math::min(min_v, p);
		max_v = math::max(max_v, p);
	}

	return { min_v, max_v };
}

// Compute signed volume of the mesh (for volume preservation)
fn float SoftBody.compute_signed_volume(&self)
{
	float six_volume = 0;
	foreach (tri : self.triangles)
	{
		six_volume += self.positions[tri.x].dot(self.positions[tri.y].cross(self.positions[tri.z]));
	}
	return six_volume * (1.0f / 6.0f);
}

// ============================================================
// XPBD Solver
// ============================================================

<*
 Advance the body by `dt` in `substeps` XPBD substeps (Muller et al., "Detailed
 Rigid Body Simulation with Extended PBD"): predict, project the constraints,
 re-pin, derive velocities.

 Lagrange multipliers restart from zero every substep, as XPBD requires; they
 accumulate across the constraint iterations of one substep only.

 @param world : `Resolves rigid-body contacts and pins; null treats both as world-fixed.`
*>
fn void SoftBody.xpbd_solve(&self, Vec3 gravity, float dt, PhysicsWorld* world = null)
{
	if (self.sleeping) return;
	if (self.positions.len() == 0) return;

	float sub_dt = dt / (float)self.substeps;
	float inv_dt2 = 1.0f / (sub_dt * sub_dt);
	Vec3 gravity_step = gravity * self.gravity_factor * sub_dt;

	self.bind_bodies(world);
	self.cache_damping(sub_dt);

	for (uint sub = 0; sub < self.substeps; sub++)
	{
		self.reset_lambdas();

		// A pin the host moved since the last substep is where the constraints see it.
		if (self.pins_first) self.apply_pins();

		self.predict(gravity_step, sub_dt);

		for (uint iter = 0; iter < self.constraint_iterations; iter++)
		{
			self.project_constraints(sub_dt, inv_dt2);
		}

		self.apply_pins();
		self.derive_velocities(sub_dt);
	}

	self.collision_constraints.clear();
	foreach (&pin : self.pins) pin.body = null;
	self.aabb = self.compute_aabb();
}

macro void @reset_lambdas(#list) @local
{
	foreach (&c : #list) c.lambda = 0;
}

fn void SoftBody.reset_lambdas(&self)
{
	@reset_lambdas(self.distance_constraints);
	@reset_lambdas(self.target_constraints);
	@reset_lambdas(self.swing_constraints);
	@reset_lambdas(self.volume_constraints);
	@reset_lambdas(self.bending_constraints);
}

// Look up each contact's and pin's rigid body once. The pointers are valid only
// for this solve: the world's body map may move its storage when bodies change.
fn void SoftBody.bind_bodies(&self, PhysicsWorld* world)
{
	sz last_id = 0;
	Rigidbody* last = null;
	foreach (&cc : self.collision_constraints)
	{
		if (cc.rigidbody_id != last_id)
		{
			last_id = cc.rigidbody_id;
			last = lookup_body(world, last_id);
		}
		cc.body = last;
	}
	foreach (&pin : self.pins) pin.body = lookup_body(world, pin.body_id);
}

fn Rigidbody* lookup_body(PhysicsWorld* world, sz id) @local
{
	if (id == 0 || world == null) return null;
	return world.find_body(id) ?? null;
}

// Per-second particle damping as a per-substep factor: pow() once per frame, not per substep.
fn void SoftBody.cache_damping(&self, float sub_dt)
{
	self.damping_factors.clear();
	foreach (d : self.particle_damping) self.damping_factors.push(dmath::pow(d, sub_dt));
}

fn void SoftBody.predict(&self, Vec3 gravity_step, float sub_dt)
{
	for (sz i = 0; i < self.positions.len(); i++)
	{
		if (self.inverse_masses[i] == 0) continue;

		self.velocities[i] += gravity_step;
		self.prev_positions[i] = self.positions[i];
		self.positions[i] += self.velocities[i] * sub_dt;
	}
}

fn void SoftBody.project_constraints(&self, float sub_dt, float inv_dt2)
{
	foreach (&c : self.distance_constraints) self.solve_distance_constraint(c, inv_dt2);
	// Pull toward host-supplied world points
	foreach (&c : self.target_constraints) self.solve_target_constraint(c, inv_dt2);
	// Cone limits about host-supplied axes
	foreach (&c : self.swing_constraints) self.solve_swing_constraint(c, inv_dt2);
	foreach (&c : self.volume_constraints) self.solve_volume_constraint(c, inv_dt2);
	foreach (&c : self.bending_constraints) self.solve_bending_constraint(c, inv_dt2);
	// Generated during the broad/narrow phase
	foreach (&c : self.collision_constraints) self.solve_collision_constraint(c, sub_dt);
}

fn void SoftBody.apply_pins(&self)
{
	foreach (&pin : self.pins) self.place_pin(pin, pin.body);
}

fn void SoftBody.derive_velocities(&self, float sub_dt)
{
	// A per-particle entry is a per-second fraction (already normalized by the
	// substep length, so halving the substep does not double the drag); the
	// whole-body `damping` is per substep, for particles without an entry.
	for (sz i = 0; i < self.positions.len(); i++)
	{
		if (self.inverse_masses[i] == 0) continue;

		float keep = i < self.damping_factors.len() ? self.damping_factors[i] : self.damping;
		self.velocities[i] = (self.positions[i] - self.prev_positions[i]) / sub_dt * keep;
	}
}

// Step a distance-like constraint with violation `|diff| - rest`. Returns the
// multiplier change and the unit direction of `diff`, or 0 when `diff` has no direction.
fn float distance_step(Vec3 diff, float rest, float w_sum, float alpha, float* lambda, Vec3* dir) @local
{
	float dist = diff.length();
	if (dist < MIN_LENGTH) return 0;

	*dir = diff / dist;
	return collision::xpbd_delta_lambda(dist - rest, w_sum, alpha, lambda);
}

// XPBD distance constraint solver
fn void SoftBody.solve_distance_constraint(&self, DistanceConstraint* c, float inv_dt2)
{
	float w0 = self.inverse_masses[c.p0];
	float w1 = self.inverse_masses[c.p1];
	if (w0 + w1 == 0) return;

	Vec3 dir;
	float dl = distance_step(self.positions[c.p1] - self.positions[c.p0], c.rest_length, w0 + w1, c.compliance * inv_dt2, &c.lambda, &dir);

	if (w0 > 0) self.positions[c.p0] -= dir * (w0 * dl);
	if (w1 > 0) self.positions[c.p1] += dir * (w1 * dl);
}

// XPBD target constraint solver — a distance constraint to a fixed world point
// with rest length 0, so only the particle moves.
fn void SoftBody.solve_target_constraint(&self, TargetConstraint* c, float inv_dt2)
{
	float w = self.inverse_masses[c.particle];
	if (w == 0) return;

	Vec3 dir;
	float dl = distance_step(self.positions[c.particle] - c.target, 0, w, c.compliance * inv_dt2, &c.lambda, &dir);
	self.positions[c.particle] += dir * (w * dl);
}

// XPBD swing constraint solver — cone limit on the direction from pivot to child.
//
// The constraint is on the angle, C = angle(reference_dir, child - pivot) - max_angle,
// and it is one-sided: inside the cone there is nothing to solve. The correction
// is applied as a rotation of the pivot-to-child vector, so the distance between
// the two particles is preserved exactly however far the clamp has to turn; the
// resulting translation is split between the two particles by inverse mass, which
// leaves their centre of mass where it was.
fn void SoftBody.solve_swing_constraint(&self, SwingConstraint* c, float inv_dt2)
{
	float w_pivot = self.inverse_masses[c.p_pivot];
	float w_child = self.inverse_masses[c.p_child];
	float w_sum = w_pivot + w_child;
	if (w_sum == 0) return;

	Vec3 pivot = self.positions[c.p_pivot];
	Vec3 d = self.positions[c.p_child] - pivot;
	float len = d.length();
	if (len < MIN_LENGTH) return;

	float ref_len = c.reference_dir.length();
	if (ref_len < MIN_LENGTH) return;

	// Reads the angle over the whole range, not saturating past a quarter turn.
	Vec3 axis;
	float angle = angle_axis_between(c.reference_dir / ref_len, d / len, &axis);

	float constraint_value = angle - math::max(c.max_angle, 0.0f);
	if (constraint_value <= 0) return;

	// Turning about `axis` by +phi opens the angle, so the correction is negative.
	// Gradient magnitude of the angle w.r.t. either endpoint is 1/len.
	float len_sq = len * len;
	float dl = collision::xpbd_delta_lambda(constraint_value, w_sum / len_sq, c.compliance * inv_dt2, &c.lambda);

	float phi = w_sum * dl / len_sq;
	Vec3 delta = dmath::rotate_about_axis(d, axis, phi) - d;

	if (w_child > 0) self.positions[c.p_child] += delta * (w_child / w_sum);
	if (w_pivot > 0) self.positions[c.p_pivot] -= delta * (w_pivot / w_sum);
}

// XPBD volume constraint solver (pressure-based)
fn void SoftBody.solve_volume_constraint(&self, VolumeConstraint* c, float inv_dt2)
{
	// One pass over the triangles builds both six-times-the-volume and the
	// (six-times) gradient: d(v0 . v1xv2)/dv0 = v1xv2, so each cross serves both.
	// The scratch is the body's own — see `volume_gradients`.
	sz n = self.positions.len();
	self.volume_gradients.clear();
	self.volume_gradients.reserve(n);
	self.volume_gradients.size = n;
	foreach (&g : self.volume_gradients) *g = { 0, 0, 0 };

	float six_volume = 0;
	foreach (tri : self.triangles)
	{
		Vec3 v0 = self.positions[tri.x];
		Vec3 v1 = self.positions[tri.y];
		Vec3 v2 = self.positions[tri.z];

		Vec3 g0 = v1.cross(v2);
		six_volume += v0.dot(g0);
		self.volume_gradients[tri.x] += g0;
		self.volume_gradients[tri.y] += v2.cross(v0);
		self.volume_gradients[tri.z] += v0.cross(v1);
	}

	float constraint_value = six_volume * (1.0f / 6.0f) - c.rest_volume;
	if (math::abs(constraint_value) < VOLUME_EPS) return;

	float total_grad_sq = 0;
	for (sz i = 0; i < n; i++)
	{
		total_grad_sq += self.inverse_masses[i] * self.volume_gradients[i].dot(self.volume_gradients[i]);
	}
	total_grad_sq *= 1.0f / 36.0f;
	if (total_grad_sq < GRAD_SQ_EPS) return;

	float dl = collision::xpbd_delta_lambda(constraint_value, total_grad_sq, c.compliance * inv_dt2, &c.lambda);

	float step = dl * (1.0f / 6.0f);
	for (sz i = 0; i < n; i++)
	{
		float w = self.inverse_masses[i];
		if (w > 0) self.positions[i] += self.volume_gradients[i] * (w * step);
	}
}

// XPBD bending constraint solver (dihedral angle)
//
// The gradient is simplified: each opposite vertex moves along its face's unit
// normal and the edge vertices share the opposite push. That makes the step a
// distance in model units, not an exact angle gradient.
fn void SoftBody.solve_bending_constraint(&self, BendingConstraint* c, float inv_dt2)
{
	Dihedral h = dihedral(
		self.positions[c.p0], self.positions[c.p1], self.positions[c.p2], self.positions[c.p3]
	);
	if (!h.valid) return;

	// Wrap angle difference to [-pi, pi]
	float constraint_value = h.angle - c.rest_angle;
	if (constraint_value > math::PI)
	{
		constraint_value -= 2 * math::PI;
	}
	else if (constraint_value < -math::PI)
	{
		constraint_value += 2 * math::PI;
	}

	if (math::abs(constraint_value) < ANGLE_EPS) return;

	float w0 = self.inverse_masses[c.p0];
	float w1 = self.inverse_masses[c.p1];
	float w2 = self.inverse_masses[c.p2];
	float w3 = self.inverse_masses[c.p3];

	Vec3 g2 = h.n0;
	Vec3 g3 = h.n1;
	Vec3 g0 = -(g2 + g3) * 0.5;
	Vec3 g1 = g0;

	float grad_sq = w0 * g0.dot(g0) + w1 * g1.dot(g1) + w2 * g2.dot(g2) + w3 * g3.dot(g3);
	if (grad_sq < GRAD_SQ_EPS) return;

	float dl = collision::xpbd_delta_lambda(constraint_value, grad_sq, c.compliance * inv_dt2, &c.lambda);

	if (w0 > 0) self.positions[c.p0] += g0 * (w0 * dl);
	if (w1 > 0) self.positions[c.p1] += g1 * (w1 * dl);
	if (w2 > 0) self.positions[c.p2] += g2 * (w2 * dl);
	if (w3 > 0) self.positions[c.p3] += g3 * (w3 * dl);
}

fn void SoftBody.add_contact(&self, sz particle, Vec3 normal, Vec3 point, sz body_id, float friction)
{
	self.collision_constraints.push(
		{ .particle = particle, .normal = normal, .contact_point = point, .rigidbody_id = body_id, .friction = friction, }
	);
}

// Apply Coulomb friction to a particle's tangential velocity
fn void SoftBody.apply_particle_friction(
	&self,
	sz particle,
	Vec3 normal,
	float correction,
	float friction,
	float dt
)
{
	if (friction <= 0) return;
	Vec3 vel = self.velocities[particle];
	Vec3 vt = vel - normal * vel.dot(normal);
	float vt_mag = vt.length();
	if (vt_mag > FRICTION_SPEED_EPS)
	{
		float impulse = math::min(friction * math::abs(correction) / dt, vt_mag);
		self.velocities[particle] -= (vt / vt_mag) * impulse;
	}
}

// XPBD collision constraint solver — unified position-based coupling
fn void SoftBody.solve_collision_constraint(
	&self,
	SoftCollisionConstraint* c,
	float dt
)
{
	float w_particle = self.inverse_masses[c.particle];
	if (w_particle == 0) return;

	Vec3 p = self.positions[c.particle];
	float d = (p - c.contact_point).dot(c.normal);
	if (d >= 0) return;

	float correction = -d;

	// If coupled to a (non-static) rigid body, share the correction
	Rigidbody* body = c.body;
	if (body && !body.is_static)
	{
		Vec3 r = c.contact_point - body.collider.translation;
		float w_body = collision::compute_generalized_inverse_mass(body, r, c.normal);
		float w_total = w_particle + w_body;

		if (w_total > W_TOTAL_EPS)
		{
			self.positions[c.particle] += c.normal * (correction * (w_particle / w_total));
			collision::apply_positional_correction(body, -c.normal, r, correction * (w_body / w_total));
		}

		self.apply_particle_friction(c.particle, c.normal, correction, c.friction, dt);
		return;
	}

	self.positions[c.particle] += c.normal * correction;
	self.apply_particle_friction(c.particle, c.normal, correction, c.friction, dt);
}

// Place a pinned particle: on its rigid body if `body` is set, else at the world point.
fn void SoftBody.place_pin(&self, ParticlePin* pin, Rigidbody* body)
{
	if (body)
	{
		Vec3 world_pos = body.collider.rotation * pin.local_offset + body.collider.translation;
		self.positions[pin.particle] = world_pos;
		self.velocities[pin.particle] = body.relative_velocity(world_pos - body.collider.translation);
		return;
	}

	self.positions[pin.particle] = pin.local_offset;
	self.velocities[pin.particle] = { 0, 0, 0 };
}

// Apply one pin, looking its body up in `world` (xpbd_solve caches the lookup instead).
fn void SoftBody.apply_pin(&self, ParticlePin* pin, PhysicsWorld* world = null)
{
	self.place_pin(pin, lookup_body(world, pin.body_id));
}

// Pin a particle to a world position
fn void SoftBody.pin_particle(&self, sz particle, Vec3 world_pos)
{
	self.pins.push({ .particle = particle, .local_offset = world_pos, .body_id = 0, });
	self.inverse_masses[particle] = 0; // Make it immovable during solving
}

// Pin a particle to a rigidbody (follows body as it moves/rotates)
fn void SoftBody.pin_to_rigidbody(&self, sz particle, sz body_id, Vec3 local_offset)
{
	self.pins.push({ .particle = particle, .local_offset = local_offset, .body_id = body_id, });
	self.inverse_masses[particle] = 0;
}

// Unpin a particle and restore its mass
fn void SoftBody.unpin_particle(&self, sz particle, float mass)
{
	// Remove pin
	self.pins.remove_using_test(fn (pin, ctx) => pin.particle == *(sz*)ctx, &particle);
	// Restore inverse mass
	if (mass > 0)
	{
		self.inverse_masses[particle] = 1.0 / mass;
	}
}

// Spring a particle toward a world position. Unlike pin_particle this leaves the
// particle dynamic: it is pulled, not placed, and compliance sets how hard.
// Returns the constraint index, which set_target_at and the list both accept.
fn sz SoftBody.add_target(&self, sz particle, Vec3 world_pos, float compliance = 0.0)
{
	sz index = self.target_constraints.len();
	self.target_constraints.push(
		{ .particle = particle, .target = world_pos, .compliance = compliance, .lambda = 0, }
	);
	return index;
}

// Move every target belonging to a particle. This is the per-substep call: the
// host samples its animation and pushes the world point the particle chases.
fn void SoftBody.set_target(&self, sz particle, Vec3 world_pos)
{
	foreach (&tc : self.target_constraints)
	{
		if (tc.particle == particle) tc.target = world_pos;
	}
}

// Move one target by constraint index, for hosts that kept the index add_target
// returned and do not want the scan.
fn void SoftBody.set_target_at(&self, sz index, Vec3 world_pos)
{
	self.target_constraints.get_ref(index).target = world_pos;
}

// Limit the direction from p_pivot to p_child to a cone of half-angle max_angle
// about reference_dir. Returns the constraint index for set_swing_reference.
fn sz SoftBody.add_swing(&self, sz p_pivot, sz p_child, Vec3 reference_dir, float max_angle, float compliance = 0.0)
{
	sz index = self.swing_constraints.len();
	self.swing_constraints.push(
		{
			.p_pivot = p_pivot,
			.p_child = p_child,
			.reference_dir = reference_dir,
			.max_angle = max_angle,
			.compliance = compliance,
			.lambda = 0,
		}
	);
	return index;
}

// Re-aim a cone. The per-substep call for a limit that follows an animated bone.
fn void SoftBody.set_swing_reference(&self, sz index, Vec3 dir)
{
	self.swing_constraints.get_ref(index).reference_dir = dir;
}

macro void @set_compliance(#list, float compliance) @local
{
	foreach (&c : #list) c.compliance = compliance;
}

// Set distance constraint compliance (0 = rigid, higher = softer)
fn void SoftBody.set_distance_compliance(&self, float compliance) => @set_compliance(self.distance_constraints, compliance);

// Set volume constraint compliance
fn void SoftBody.set_volume_compliance(&self, float compliance) => @set_compliance(self.volume_constraints, compliance);

// Set bending constraint compliance
fn void SoftBody.set_bending_compliance(&self, float compliance) => @set_compliance(self.bending_constraints, compliance);

// Enable volume preservation
fn void SoftBody.enable_volume_preservation(&self, float compliance = 0.0)
{
	float rest_vol = self.compute_signed_volume();
	self.volume_constraints.push({ .rest_volume = rest_vol, .compliance = compliance, .lambda = 0, });
}

// Enable bending resistance
fn void SoftBody.enable_bending(&self, float compliance = DEFAULT_BENDING_COMPLIANCE)
{
	self.extract_bending_constraints();
	self.set_bending_compliance(compliance);
}

// Wake the soft body from sleep
fn void SoftBody.wake(&self)
{
	self.sleeping = false;
	self.sleep_timer = 0;
}

// Check if body has enough velocity to be considered moving
fn bool SoftBody.is_moving(&self)
{
	foreach (v : self.velocities)
	{
		if (v.length_sq() > SLEEP_SPEED_SQ) return true;
	}
	return false;
}

<*
 Generate ground plane collision constraints: `dot(p, normal) < offset` is below
 the surface, and the contact point is the particle projected back onto the plane.
*>
fn void SoftBody.generate_ground_collisions(&self, GroundPlane plane, float friction = 0.5)
{
	for (sz i = 0; i < self.positions.len(); i++)
	{
		if (self.inverse_masses[i] == 0) continue;

		Vec3 p = self.positions[i];
		float height = p.dot(plane.normal) - plane.offset;
		if (height < 0)
		{
			self.add_contact(i, plane.normal, p - plane.normal * height, 0, friction);
		}
	}
}

<*
 Generate collision constraints for particles inside world-space capsules.

 The point-based counterpart to generate_rigidbody_collisions, which walks
 triangles and so finds nothing at all on a body that has none — a particle
 chain among them. Capsule `i` is the segment seg_a[i] → seg_b[i] swept by
 radius[i]; no shape or transform is involved, the host poses the endpoints.
 Each particle is grown by its `particle_radius` entry, or treated as a point
 when the list is unset.

 Constraints land in `collision_constraints` with rigidbody_id 0, so the ordinary
 collision solve treats them as immovable-world contacts and does the pushout.
 Call this before xpbd_solve, as with the other generators: the list is cleared
 at the end of every solve.

 @require seg_a.len == seg_b.len : `generate_capsule_collisions needs matching segment endpoints`
 @require seg_a.len == radius.len : `generate_capsule_collisions needs one radius per segment`
*>
fn void SoftBody.generate_capsule_collisions(&self, Vec3[] seg_a, Vec3[] seg_b, float[] radius, float friction = 0.0f)
{
	if (seg_a.len == 0) return;

	for (sz i = 0; i < self.positions.len(); i++)
	{
		if (self.inverse_masses[i] == 0) continue;

		Vec3 p = self.positions[i];
		float p_radius = i < self.particle_radius.len() ? self.particle_radius[i] : 0;

		for (sz s = 0; s < seg_a.len; s++)
		{
			CapsuleHit hit = collision::capsule_push_point(p, seg_a[s], seg_b[s], radius[s], p_radius);
			if (!hit.hit) continue;

			self.add_contact(i, hit.normal, hit.point, 0, friction);
		}
	}
}

// Detect and generate collision constraints against a rigidbody
fn void SoftBody.generate_rigidbody_collisions(&self, Rigidbody* body)
{
	if (!self.aabb.collides(body.transformed_aabb)) return;

	float friction = (self.material.static_friction + body.material.static_friction) * 0.5f;

	foreach (tri : self.triangles)
	{
		// Triangle vertices are already in world space
		TriangleShape tri_shape = { .v = tri.verts(self.positions.array_view()) };
		if (!tri_shape.get_aabb().collides(body.transformed_aabb)) continue;

		CollisionInfo info = collision::check_convex_collision(&tri_shape, &body.collider, epa: true);
		if (!info.collided || info.depth <= 0) continue;

		// Constrain each vertex that has sunk into the body. The returned normal is
		// the one the triangle escapes along negated (as in the rigid path), so a
		// vertex inside the body projects POSITIVELY past the contact point; the
		// solver then pushes along -normal.
		sz[3] indices = { tri.x, tri.y, tri.z };
		foreach (idx : indices)
		{
			Vec3 p = self.positions[idx];
			float proj = (p - info.contact_points[1]).dot(info.normal);
			if (proj > 0)
			{
				self.add_contact(idx, -info.normal, p - info.normal * proj, body.id, friction);
			}
		}
	}
}

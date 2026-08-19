<*
 Particle chains — the shape a tail, a rope or a jiggle bone run has.

	`c3c test --test-filter collision_tests::chain`

 A chain is the one body a mesh constructor cannot make: links and no surface.
 What it is driven by is everything a host re-supplies each substep — a moving
 pin, spring targets, a re-aimed cone limit, posed collider capsules — and the
 checks below are mostly about what must *not* happen while all of that is
 running: the links do not stretch, the cone does not open past its limit, and
 two chains fed identical inputs stay equal to the last bit rather than to a
 tolerance. A bake replayed tomorrow has to produce the same numbers, and "close
 enough" is what drifts.

 `create_particle_chain` takes its allocator as the first argument, so every
 chain here is built on whatever `mem` resolves to at the call — which under
 `c3c test` is the runner's tracking allocator. That is what makes a missing
 `defer sb.free()` a failing test rather than nothing at all.
*>
module collision_tests::chain;

import collision;
import collision::dmath;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

// The angle between two directions, read the well-conditioned way.
macro float swing_angle(Vec3 v, Vec3 axis)
{
	Vec3 a = v.normalize();
	Vec3 b = axis.normalize();
	return dmath::atan2(a.cross(b).length(), a.dot(b));
}

// Equal to the last bit, not to a tolerance — a bake replayed tomorrow has to
// produce the same numbers, and "close enough" is what drifts.
macro bool same_bits(Vec3 a, Vec3 b)
{
	return bitcast(a.x, uint) == bitcast(b.x, uint) &&
		bitcast(a.y, uint) == bitcast(b.y, uint) &&
		bitcast(a.z, uint) == bitcast(b.z, uint);
}

// A four-particle chain hung off a pinned root with everything a host drives at
// once: a spring toward an animated point, a cone limit, per-particle damping
// and a collision radius.
macro SoftBody build_jiggle_chain(sz id)
{
	Vec3[4] points = { { 0, 0, 0 }, { 0, -0.5f, 0 }, { 0, -1.0f, 0 }, { 0, -1.5f, 0 } };
	float[4] weights = { 1, 1, 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], id);
	sb.substeps = 1;
	sb.pins_first = true;
	sb.pin_particle(0, points[0]);

	for (sz i = 0; i < 4; i++)
	{
		sb.particle_damping.push(0.2f);
		sb.particle_radius.push(0.05f);
	}

	for (sz i = 1; i < 4; i++)
	{
		sb.add_target(i, points[i], 0.0005f);
		sb.add_swing(i - 1, i, { 0, -1, 0 }, (float)(math::PI / 4.0), 0.0f);
	}

	return sb;
}

// A chain hung off a still pin is a pendulum: it has to end up straight below
// the pin, and it must not have stretched on the way there — not at the end,
// which any solver reaches eventually, but at every substep in between.
fn void test_particle_chain_settles_to_vertical() @test
{
	Vec3[4] points = { { 0, 0, 0 }, { 0.5f, 0, 0 }, { 1.0f, 0, 0 }, { 1.5f, 0, 0 } };
	float[4] weights = { 1, 1, 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 1);
	defer sb.free();

	// Three links, no surface: a chain is exactly the body a mesh constructor
	// cannot make.
	test::@check(sb.positions.len() == 4, "a 4-point chain made %d particles", sb.positions.len());
	test::@check(sb.prev_positions.len() == 4, "%d previous positions for 4 particles", sb.prev_positions.len());
	test::@check(sb.distance_constraints.len() == 3, "4 points make 3 links, got %d", sb.distance_constraints.len());
	test::@check(sb.triangles.len() == 0, "a chain has no surface, but it has %d triangles", sb.triangles.len());
	foreach (dc : sb.distance_constraints) test::@check(math::abs(dc.rest_length - 0.5f) < 1e-6f);

	sb.substeps = 1;
	sb.pin_particle(0, points[0]);

	Vec3 gravity = { 0, -9.81f, 0 };
	for (int i = 0; i < 2000; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 120.0f);

		for (sz p = 1; p < 4; p++)
		{
			float len = (sb.positions[p] - sb.positions[p - 1]).length();
			test::@check(math::abs(len - 0.5f) < 1e-3f, "segment %d measured %f on step %d", (int)p, len, i);
		}
	}

	test::@check(same_bits(sb.positions[0], (Vec3){ 0, 0, 0 }), "the pinned root drifted to %s", sb.positions[0]);
	for (sz p = 1; p < 4; p++)
	{
		Vec3 hang = sb.positions[p];
		test::@check(math::abs(hang.x) < 0.01f, "particle %d hangs out at x=%f", (int)p, hang.x);
		test::@check(math::abs(hang.z) < 0.01f, "particle %d hangs out at z=%f", (int)p, hang.z);
		test::@check(math::abs(hang.y + 0.5f * (float)p) < 0.01f, "particle %d hangs at y=%f", (int)p, hang.y);
	}
}

// A target is a spring to a point the host owns rather than a link to another
// particle, so with nothing else pulling the particle must arrive and stay.
fn void test_target_constraint_converges() @test
{
	Vec3[1] points = { { 0, 0, 0 } };
	float[1] weights = { 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 2);
	defer sb.free();
	sb.substeps = 1;

	test::@check(sb.distance_constraints.len() == 0); // one point is a chain of no links

	Vec3 target = { 1.0f, 2.0f, -0.5f };
	test::@check(sb.add_target(0, target, 0.0001f) == 0);

	for (int i = 0; i < 600; i++)
	{
		sb.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	}

	float miss = (sb.positions[0] - target).length();
	test::@check(miss < 1e-3f, "the particle settled %f from its target", miss);

	// Made rigid, gravity cannot pull it off; and the target is a thing the host
	// moves, so it has to follow.
	sb.target_constraints.get_ref(0).compliance = 0;
	Vec3 moved = { -2.0f, 0.5f, 3.0f };
	sb.set_target(0, moved);

	for (int i = 0; i < 120; i++)
	{
		sb.xpbd_solve({ 0, -9.81f, 0 }, 1.0f / 120.0f);
	}

	test::@check((sb.positions[0] - moved).length() < 1e-4f, "a rigid target left it at %s", sb.positions[0]);
}

// Pushing a point out of a capsule, including the one place where the direction
// is not implied by the geometry at all: dead on the axis.
fn void test_capsule_push_point_separates() @test
{
	Vec3 a = { 0, 0, 0 };
	Vec3 b = { 0, 2, 0 };

	// Inside and off-axis: straight out, to exactly the surface.
	CapsuleHit near = collision::capsule_push_point({ 0.25f, 1, 0 }, a, b, 1.0f);
	test::@check(near.hit);
	test::@check((near.normal - (Vec3){ 1, 0, 0 }).length() < 1e-5f, "normal was %s", near.normal);
	test::@check(math::abs(near.depth - 0.75f) < 1e-5f, "depth was %f", near.depth);
	test::@check((near.point - (Vec3){ 1, 1, 0 }).length() < 1e-5f, "surface point was %s", near.point);

	// A particle with a radius of its own is separated by that much more.
	CapsuleHit fat = collision::capsule_push_point({ 0.25f, 1, 0 }, a, b, 1.0f, 0.3f);
	test::@check(fat.hit);
	test::@check(math::abs(fat.depth - 1.05f) < 1e-5f, "depth was %f", fat.depth);
	test::@check((fat.point - (Vec3){ 1.3f, 1, 0 }).length() < 1e-5f, "surface point was %s", fat.point);

	// Outside is no hit, and neither is a capsule with no thickness to be in.
	test::@check(!collision::capsule_push_point({ 2, 1, 0 }, a, b, 1.0f).hit);
	test::@check(!collision::capsule_push_point({ 0, 1, 0 }, a, b, 0.0f).hit);

	// On the axis every direction is equally right and none is implied, so the
	// answer has to be a finite unit vector rather than a normalized zero.
	CapsuleHit axial = collision::capsule_push_point({ 0, 1, 0 }, a, b, 1.0f);
	test::@check(axial.hit);
	test::@check(math::abs(axial.normal.length() - 1.0f) < 1e-5f, "on-axis normal was %s", axial.normal);
	test::@check(math::abs(axial.normal.y) < 1e-5f, "on-axis normal ran along the axis: %s", axial.normal);
	test::@check(math::abs(axial.depth - 1.0f) < 1e-5f, "on-axis depth was %f", axial.depth);
	test::@check((axial.point - ((Vec3){ 0, 1, 0 } + axial.normal)).length() < 1e-5f);

	// A segment of no length is a sphere, and its centre is the same degeneracy
	// one level down — there is not even an axis left to be perpendicular to.
	CapsuleHit sphere = collision::capsule_push_point(a, a, a, 0.5f);
	test::@check(sphere.hit);
	test::@check(math::abs(sphere.normal.length() - 1.0f) < 1e-5f, "sphere-centre normal was %s", sphere.normal);
	test::@check(math::abs(sphere.depth - 0.5f) < 1e-5f, "sphere-centre depth was %f", sphere.depth);
}

// The chain-side wrapper: a body with no triangles gets no response at all from
// generate_rigidbody_collisions, so this is how a chain learns about a body.
fn void test_capsule_collisions_push_a_chain_out() @test
{
	Vec3[2] points = { { 0, 0, 0 }, { 0.2f, -0.4f, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 3);
	defer sb.free();
	sb.substeps = 1;
	sb.pin_particle(0, points[0]);
	sb.particle_radius.push(0);
	sb.particle_radius.push(0.05f);

	// A capsule hanging below the pin, thick enough to have swallowed particle 1.
	Vec3[1] seg_a = { { 0, -1, 0 } };
	Vec3[1] seg_b = { { 0, 0, 0 } };
	float[1] radii = { 0.3f };

	sb.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
	test::@check(sb.collision_constraints.len() == 1, "the buried particle produced %d constraints", (int)sb.collision_constraints.len());
	test::@check(sb.collision_constraints[0].particle == 1);
	sb.collision_constraints.clear();

	for (int i = 0; i < 400; i++)
	{
		sb.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
		sb.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	}

	Vec3 on_axis = collision::closest_point_on_segment(sb.positions[1], seg_a[0], seg_b[0]);
	float clearance = (sb.positions[1] - on_axis).length();
	test::@check(clearance > 0.35f - 1e-3f, "particle 1 settled %f from the axis, inside 0.35", clearance);

	// And the link it hangs from was not stretched to get it out.
	float len = (sb.positions[1] - sb.positions[0]).length();
	test::@check(math::abs(len - 0.4472136f) < 1e-3f, "the link measured %f", len);
}

// A cone limit is one-sided — inside it nothing happens — and it is a rotation,
// so however hard the pull, the link it clamps does not stretch to obey.
fn void test_swing_constraint_clamps_to_the_cone() @test
{
	Vec3[2] points = { { 0, 0, 0 }, { 0, -1, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 4);
	defer sb.free();
	sb.substeps = 1;
	sb.pin_particle(0, points[0]);

	float max_angle = (float)(math::PI / 6.0); // 30 degrees
	test::@check(sb.add_swing(0, 1, { 0, -1, 0 }, max_angle) == 0);

	// Gravity sideways: the chain wants to hang a quarter turn off the cone axis.
	Vec3 gravity = { 9.81f, 0, 0 };
	for (int i = 0; i < 1200; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 120.0f);

		float open = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, -1, 0 });
		test::@check(open <= max_angle + 1e-3f, "the cone opened to %f rad on step %d", open, i);

		float len = (sb.positions[1] - sb.positions[0]).length();
		test::@check(math::abs(len - 1.0f) < 1e-3f, "the link measured %f on step %d", len, i);
	}

	// Pressed against the wall of the cone, not idling somewhere inside it.
	float settled = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, -1, 0 });
	test::@check(math::abs(settled - max_angle) < 1e-2f, "it settled at %f rad, not against the wall", settled);

	// Re-aiming the cone is the per-substep call, and it carries the child with it.
	sb.set_swing_reference(0, { 0, 1, 0 });
	for (int i = 0; i < 1200; i++) sb.xpbd_solve(gravity, 1.0f / 120.0f);

	float flipped = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, 1, 0 });
	test::@check(flipped <= max_angle + 1e-2f, "the re-aimed cone left the child at %f rad", flipped);
}

// Pins land after the constraint sweep, so a pin the host moved between substeps
// is a substep late and the chain is solved against where it used to be. That is
// what pins_first is for; the default has to keep lagging.
fn void test_pins_first_removes_the_substep_lag() @test
{
	Vec3[2] points = { { 0, 0, 0 }, { 0, -1, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody lagging = collision::create_particle_chain(mem, points[..], weights[..], 5);
	SoftBody leading = collision::create_particle_chain(mem, points[..], weights[..], 6);
	defer lagging.free();
	defer leading.free();

	lagging.substeps = 1;
	leading.substeps = 1;
	leading.pins_first = true;
	test::@check(!lagging.pins_first); // off unless asked for

	lagging.pin_particle(0, points[0]);
	leading.pin_particle(0, points[0]);

	Vec3 moved = { 0.4f, 0, 0 };
	lagging.pins.get_ref(0).local_offset = moved;
	leading.pins.get_ref(0).local_offset = moved;

	lagging.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	leading.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);

	test::@check(same_bits(lagging.positions[0], moved) && same_bits(leading.positions[0], moved));

	float lead_len = (leading.positions[1] - leading.positions[0]).length();
	test::@check(math::abs(lead_len - 1.0f) < 1e-3f, "pins_first still left the link at %f", lead_len);

	float lag_len = (lagging.positions[1] - lagging.positions[0]).length();
	test::@check(math::abs(lag_len - 1.0f) > 0.05f, "the default stopped lagging: the link was %f", lag_len);
}

// Per-particle damping is a fraction per second, so the same second of
// simulation costs the same speed however it is cut into substeps. The
// whole-body `damping` is a fraction per substep and must stay one.
fn void test_particle_damping_is_substep_invariant() @test
{
	Vec3[1] points = { { 0, 0, 0 } };
	float[1] weights = { 1 };

	SoftBody coarse = collision::create_particle_chain(mem, points[..], weights[..], 8);
	SoftBody fine = collision::create_particle_chain(mem, points[..], weights[..], 9);
	defer coarse.free();
	defer fine.free();

	coarse.substeps = 1;
	fine.substeps = 1;
	coarse.particle_damping.push(0.5f);
	fine.particle_damping.push(0.5f);
	coarse.velocities[0] = (Vec3){ 1, 0, 0 };
	fine.velocities[0] = (Vec3){ 1, 0, 0 };

	coarse.xpbd_solve({ 0, 0, 0 }, 1.0f);
	for (int i = 0; i < 120; i++) fine.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);

	test::@check(math::abs(coarse.velocities[0].x - 0.5f) < 1e-4f, "one whole-second step kept %f", coarse.velocities[0].x);
	test::@check(math::abs(fine.velocities[0].x - 0.5f) < 1e-3f, "120 small steps kept %f", fine.velocities[0].x);

	// An empty list is the old path, unchanged: one raw multiply per substep.
	SoftBody legacy = collision::create_particle_chain(mem, points[..], weights[..], 10);
	defer legacy.free();
	legacy.substeps = 1;
	legacy.velocities[0] = (Vec3){ 1, 0, 0 };
	legacy.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	test::@check(legacy.velocities[0].x == 0.99f, "the whole-body damping path changed: %f", legacy.velocities[0].x);
}

// Two chains fed identical inputs must stay bit-equal for as long as they are
// driven — a bake is only reproducible if this holds every substep, not on
// average. Pins, targets, cones and capsules are all in play at once.
fn void test_chain_determinism_lockstep() @test
{
	SoftBody a = build_jiggle_chain(11);
	SoftBody b = build_jiggle_chain(12);
	defer a.free();
	defer b.free();

	Vec3 gravity = { 0, -9.81f, 0 };
	Vec3[1] seg_a = { { 0.35f, -0.9f, 0 } };
	Vec3[1] seg_b = { { 0.35f, -0.2f, 0 } };
	float[1] radii = { 0.25f };

	for (int step = 0; step < 500; step++)
	{
		// Everything a host re-supplies each substep: a moving pin, moving
		// targets, a re-aimed cone, posed collider segments.
		float t = (float)step / 120.0f;
		Vec3 sway = { dmath::sin(t * 6.0f) * 0.4f, 0, dmath::cos(t * 4.0f) * 0.2f };

		a.pins.get_ref(0).local_offset = sway;
		b.pins.get_ref(0).local_offset = sway;

		for (sz i = 1; i < 4; i++)
		{
			Vec3 target = sway + (Vec3){ 0, -0.5f * (float)i, 0 };
			a.set_target(i, target);
			b.set_target(i, target);
			a.set_swing_reference(i - 1, { sway.x * 0.5f, -1, 0 });
			b.set_swing_reference(i - 1, { sway.x * 0.5f, -1, 0 });
		}

		a.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
		b.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);

		a.xpbd_solve(gravity, 1.0f / 120.0f);
		b.xpbd_solve(gravity, 1.0f / 120.0f);

		for (sz i = 0; i < a.positions.len(); i++)
		{
			test::@check(same_bits(a.positions[i], b.positions[i]), "particle %d diverged on step %d: %s vs %s", (int)i, step, a.positions[i], b.positions[i]);
			test::@check(same_bits(a.velocities[i], b.velocities[i]), "particle %d's velocity diverged on step %d", (int)i, step);
		}
	}

	// And it actually moved, so the equality above is not two chains sitting still.
	test::@check((a.positions[3] - (Vec3){ 0, -1.5f, 0 }).length() > 0.05f, "the chain never left its rest pose");
}

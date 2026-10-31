<*
 Soft body behaviours that the solver cleanup touched: the bending gradient,
 the sleep threshold, per-substep multipliers, and the IK turn's bit-exactness.

	`c3c test --test-filter collision_tests::softbody_extra`
*>
module collision_tests::softbody_extra;

import collision;
import collision::dmath;
import collision::ik;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

// Two triangles sharing the edge p0-p1 (length `size`), flat at rest, then p3
// lifted `bend` radians about the edge. The edge is pinned; the tips are free.
fn SoftBody make_hinge(float size, float bend)
{
	SoftBody sb;
	sb.init(mem, 4);
	sb.add_particle({ 0, 0, 0 }, 0);
	sb.add_particle({ size, 0, 0 }, 0);
	sb.add_particle({ size * 0.5f, size, 0 }, 1);
	sb.add_particle({ size * 0.5f, -size, 0 }, 1);
	sb.triangles.push({ 0, 2, 1 });
	sb.triangles.push({ 0, 1, 3 });
	sb.enable_bending(0.0f);
	sb.positions[3] = { size * 0.5f, -size * math::cos(bend), size * math::sin(bend) };
	return sb;
}

fn float hinge_error(SoftBody* sb)
{
	BendingConstraint* c = sb.bending_constraints.get_ref(0);
	return sb.compute_dihedral_angle(c.p0, c.p1, c.p2, c.p3) - c.rest_angle;
}

fn void test_bending_gradient_is_size_independent_in_direction() @test
{
	float[3] sizes = { 1.0f, 2.0f, 4.0f };
	// A unit-normal gradient moves the tips by ~C/2 each, which turns the hinge
	// by ~C/size: one solve removes a size-dependent share, but never so little
	// that large triangles stall (they did when the normals were divided twice).
	float[3] max_left = { 0.2f, 0.8f, 0.9f };

	foreach (i, size : sizes)
	{
		SoftBody sb = make_hinge(size, 0.5f);
		defer sb.free();
		test::@check(sb.bending_constraints.len() == 1, "%d bending constraints", (int)sb.bending_constraints.len());

		float before = hinge_error(&sb);
		test::@check(math::abs(before) > 0.4f, "hinge not bent: %f", before);

		sb.solve_bending_constraint(sb.bending_constraints.get_ref(0), 3600.0f);
		float after = hinge_error(&sb);

		test::@check(before * after >= 0, "size %f overshot: %f -> %f", size, before, after);
		test::@check(math::abs(after) < math::abs(before) * max_left[i], "size %f barely corrected: %f -> %f", size, before, after);
	}
}

fn void test_is_moving_threshold_is_a_speed() @test
{
	SoftBody sb;
	sb.init(mem, 1);
	defer sb.free();
	sb.add_particle({ 0, 0, 0 }, 1);

	sb.velocities[0] = { 0.09f, 0, 0 };
	test::@check(!sb.is_moving(), "0.09 is below the 0.1 speed threshold");
	sb.velocities[0] = { 0.11f, 0, 0 };
	test::@check(sb.is_moving(), "0.11 is above the 0.1 speed threshold");
}

fn void test_multipliers_restart_each_substep() @test
{
	// A compliant, stretched link with one pinned end. If multipliers carried
	// across substeps the extra substeps would behave ever softer; reset per
	// substep, the lambda left after the solve is that of the last substep alone
	// and matches a one-substep solve of the same final state's scale.
	Vec3[2] points = { { 0, 0, 0 }, { 1, 0, 0 } };
	float[2] weights = { 0.0f, 1.0f };
	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 1);
	defer sb.free();
	sb.set_distance_compliance(0.01f);
	sb.gravity_factor = 0;
	sb.positions[1] = { 2, 0, 0 };
	sb.prev_positions[1] = { 2, 0, 0 };
	sb.xpbd_solve({ 0, 0, 0 }, 1.0f / 60.0f);

	DistanceConstraint* c = sb.distance_constraints.get_ref(0);
	float len = (sb.positions[1] - sb.positions[0]).length();
	test::@check(len < 2.0f && len > 1.0f, "link should relax toward rest: %f", len);
	// Within one substep the iterations accumulate toward -(C)/(w+alpha) at most.
	test::@check(c.lambda < 0.0f && c.lambda > -1.0f, "last-substep lambda out of range: %f", c.lambda);
}

fn void test_ik_turn_matches_deterministic_quaternion() @test
{
	// x -> y is a quarter turn about +z; the turn must be dmath's, bit for bit.
	Quaternionf q = ik::shortest_arc({ 1, 0, 0 }, { 0, 1, 0 });
	QuaternionValue expect = dmath::quat_from_axis_angle({ 0, 0, 1 }, dmath::atan2(1.0f, 0.0f));
	test::@check(q.v == expect.v, "ik turn differs from dmath axis-angle");
}

fn void test_bending_extracted_past_unshared_edges() @test
{
	// An open strip: three triangles in a row share two interior edges, and the
	// boundary edges (shared by one triangle) must not end the extraction.
	SoftBody sb;
	sb.init(mem, 5);
	defer sb.free();
	for (int i = 0; i < 5; i++) sb.add_particle({ (float)i, (float)(i % 2), 0 }, 1);
	sb.triangles.push({ 0, 1, 2 });
	sb.triangles.push({ 1, 3, 2 });
	sb.triangles.push({ 2, 3, 4 });
	sb.enable_bending();
	test::@check(sb.bending_constraints.len() == 2, "a 3-triangle strip has 2 hinges, got %d", (int)sb.bending_constraints.len());
}

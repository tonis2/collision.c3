<*
 Box against box by separating axes (`box_box_collision`): the contact it gives
 for a resting pair and for a gap inside the margin, and the depth EPA finds for
 the same pair turned every way.

	`c3c test --test-filter collision_tests::box_box`
*>
module collision_tests::box_box;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

const Aabb3 UNIT = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
const Aabb3 PLANK = { { -0.4f, -0.05f, -0.15f }, { 0.4f, 0.05f, 0.15f } };

fn TransformedShape place(Aabb3* box, Vec3 at, Quaternionf turn = { 0, 0, 0, 1 })
{
	return { .shape = box, .translation = at, .scale = { 1, 1, 1 }, .rotation = turn };
}

fn void a_box_resting_on_another_is_pushed_straight_up() @test
{
	TransformedShape a = place(&UNIT, { 0, 0, 0 });
	TransformedShape b = place(&UNIT, { 0.2f, 0.98f, -0.1f });
	CollisionInfo info = collision::box_box_collision(&a, &b, 0);

	test::@check(info.collided, "they overlap");
	test::@check(info.normal.distance((Vec3){ 0, 1, 0 }) < 1e-5f, "the normal is the face's: %s", info.normal);
	test::eq_approx(info.depth, 0.02f, delta: 1e-5);
	Vec3 between = info.contact_points[0] - info.contact_points[1];
	test::@check(between.distance(info.normal * info.depth) < 1e-5f, "the points are the depth apart along the normal: %s", between);
}

fn void a_gap_inside_the_margin_is_a_contact_with_negative_depth() @test
{
	TransformedShape a = place(&UNIT, { 0, 0, 0 });
	TransformedShape b = place(&UNIT, { 0, 1.003f, 0 });

	CollisionInfo near = collision::box_box_collision(&a, &b, 0.005f);
	test::@check(near.collided, "3 mm apart is inside a 5 mm margin");
	test::eq_approx(near.depth, -0.003f, delta: 1e-5);
	test::@check(!collision::box_box_collision(&a, &b, 0.002f).collided, "and outside a 2 mm one");
}

// A small fixed generator, so a failure names the same pair every run.
fn float next(uint* state)
{
	*state = *state * 1664525 + 1013904223;
	return (float)(*state >> 8) / (float)(1 << 24) * 2.0f - 1.0f;
}

fn Quaternionf turn(uint* state)
{
	Quaternionf q = { .v = { next(state), next(state), next(state), next(state) } };
	return q.normalize();
}

<*
 Random pairs turned every way, corners into faces and edges across edges: where
 EPA finds them overlapping, the separating axes find the same depth.
*>
fn void the_depth_is_the_one_epa_finds() @test
{
	uint state = 12345;
	int overlapping = 0;
	for (int trial = 0; trial < 400; trial++)
	{
		TransformedShape a = place(&UNIT, { 0, 0, 0 }, turn(&state));
		TransformedShape b = place(&PLANK, { next(&state) * 0.8f, next(&state) * 0.8f, next(&state) * 0.8f }, turn(&state));

		PreparedShape prep_a = a.prepare();
		PreparedShape prep_b = b.prepare();
		CollisionInfo epa = collision::check_convex_collision(&prep_a, &prep_b, epa: true);
		CollisionInfo sat = collision::box_box_collision(&a, &b, 0);
		if (!epa.collided || epa.depth < 1e-3f) continue;
		overlapping++;

		test::@check(sat.collided, "trial %d: EPA found %f deep, the axes found them apart", trial, epa.depth);
		test::@check(math::abs(sat.depth - epa.depth) < 2e-3f,
			"trial %d: %f deep by the axes, %f by EPA", trial, sat.depth, epa.depth);
	}
	test::@check(overlapping > 100, "only %d of the pairs overlapped", overlapping);
}

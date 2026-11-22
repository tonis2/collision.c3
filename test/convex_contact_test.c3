<*
 The contact between two convex colliders (`convex_contact`): GJK's closest
 points for a pair apart, a sphere's and a capsule's radius around a core, and
 EPA only for cores that overlap.

	`c3c test --test-filter collision_tests::convex_contact`
*>
module collision_tests::convex_contact;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

const Aabb3 UNIT = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
const Aabb3 PLANK = { { -0.4f, -0.05f, -0.15f }, { 0.4f, 0.05f, 0.15f } };
const float MARGIN = 0.005f;

fn TransformedShape place(CollisionShape shape, Vec3 at, Quaternionf turn = { 0, 0, 0, 1 })
{
	return { .shape = shape, .translation = at, .scale = { 1, 1, 1 }, .rotation = turn };
}

macro void @check_form(CollisionInfo info)
{
	Vec3 between = info.contact_points[0] - info.contact_points[1];
	test::@check(between.distance(info.normal * info.depth) < 1e-4f,
		"the points are the depth apart along the normal: %s against %s", between, info.normal * info.depth);
}

fn void a_stone_just_above_a_block_touches_it_through_the_margin() @test
{
	Sphere stone = { .radius = 0.04f };
	TransformedShape block = place(&UNIT, { 0, 0, 0 });
	TransformedShape above = place(&stone, { 0.1f, 0.543f, 0 });

	CollisionInfo info = collision::convex_contact(&block, &above, MARGIN);
	test::@check(info.collided, "3 mm apart is inside the margin");
	test::eq_approx(info.depth, -0.003f, delta: 1e-5);
	test::@check(info.normal.distance((Vec3){ 0, 1, 0 }) < 1e-4f, "the normal is the face's: %s", info.normal);
	test::@check(info.contact_points[0].distance((Vec3){ 0.1f, 0.5f, 0 }) < 1e-4f, "on the block's top: %s", info.contact_points[0]);
	@check_form(info);

	TransformedShape far = place(&stone, { 0.1f, 0.55f, 0 });
	test::@check(!collision::convex_contact(&block, &far, MARGIN).collided, "1 cm apart is not");
}

fn void a_stone_sunk_into_a_block_is_measured_from_its_centre() @test
{
	Sphere stone = { .radius = 0.04f };
	TransformedShape block = place(&UNIT, { 0, 0, 0 });

	TransformedShape sunk = place(&stone, { 0, 0.53f, 0.2f });
	CollisionInfo info = collision::convex_contact(&block, &sunk, MARGIN);
	test::@check(info.collided, "a centimetre deep");
	test::eq_approx(info.depth, 0.01f, delta: 1e-5);
	test::@check(info.normal.distance((Vec3){ 0, 1, 0 }) < 1e-4f, "pushed out of the top: %s", info.normal);
	@check_form(info);

	// Its centre inside the block: only EPA can say which way is out.
	TransformedShape inside = place(&stone, { 0, 0.48f, 0.2f });
	CollisionInfo deep = collision::convex_contact(&block, &inside, MARGIN);
	test::@check(deep.collided, "the centre is inside");
	test::eq_approx(deep.depth, 0.06f, delta: 1e-3);
	test::@check(deep.normal.distance((Vec3){ 0, 1, 0 }) < 1e-2f, "still out of the top: %s", deep.normal);
}

<*
 Two balls half inside each other. EPA on two round shapes never converges, and
 it reported them apart; their centres are the exact answer.
*>
fn void two_balls_deep_in_each_other_are_found() @test
{
	Sphere ball = { .radius = 0.4f };
	TransformedShape a = place(&ball, { 0, 0, 0 });
	TransformedShape b = place(&ball, { 0.3f, 0.2f, -0.1f });

	CollisionInfo info = collision::convex_contact(&a, &b, MARGIN);
	float apart = ((Vec3){ 0.3f, 0.2f, -0.1f }).length();
	test::@check(info.collided, "half inside each other");
	test::eq_approx(info.depth, (double)(0.8f - apart), delta: 1e-5);
	test::@check(info.normal.distance(((Vec3){ 0.3f, 0.2f, -0.1f }) / apart) < 1e-5f, "along the centres: %s", info.normal);
	@check_form(info);
}

fn void a_capsule_lying_on_a_block_rests_along_its_length() @test
{
	Capsule rod = { .height = 0.6f, .radius_top = 0.05f, .radius_bottom = 0.05f };
	// Its axis turned from y to x: lying down.
	Quaternionf lying = { .v = { 0, 0, (float)math::sin(math::PI / 4), (float)math::cos(math::PI / 4) } };
	TransformedShape block = place(&UNIT, { 0, 0, 0 });
	TransformedShape on_top = place(&rod, { 0.05f, 0.552f, 0 }, lying);

	CollisionInfo info = collision::convex_contact(&block, &on_top, MARGIN);
	test::@check(info.collided, "2 mm above");
	test::eq_approx(info.depth, -0.002f, delta: 1e-5);
	test::@check(info.normal.distance((Vec3){ 0, 1, 0 }) < 1e-4f, "the normal is the face's: %s", info.normal);
	@check_form(info);
}

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
 Two boxes through GJK, apart or overlapping, turned every way: the same
 contact the separating axes find, which is exact for boxes.
*>
fn void boxes_agree_with_the_separating_axes() @test
{
	uint state = 777;
	int apart = 0;
	int overlapping = 0;
	for (int trial = 0; trial < 600; trial++)
	{
		TransformedShape a = place(&UNIT, { 0, 0, 0 }, turn(&state));
		TransformedShape b = place(&PLANK, { next(&state) * 0.9f, next(&state) * 0.9f, next(&state) * 0.9f }, turn(&state));
		CollisionInfo exact = collision::box_box_collision(&a, &b, MARGIN);
		CollisionInfo gjk = collision::convex_contact(&a, &b, MARGIN);

		test::@check(exact.collided == gjk.collided || math::abs(exact.depth + MARGIN) < 1e-4f,
			"trial %d: the axes say %s, GJK %s", trial, exact.collided, gjk.collided);
		if (!exact.collided || !gjk.collided) continue;
		if (exact.depth < 0)
		{
			apart++;
		}
		else
		{
			overlapping++;
		}
		test::@check(math::abs(exact.depth - gjk.depth) < 2e-3f, "trial %d: %f deep by the axes, %f by GJK", trial, exact.depth, gjk.depth);
		@check_form(gjk);
	}
	test::@check(overlapping > 100 && apart > 0, "only %d pairs overlapped and %d were apart", overlapping, apart);
}

<*
 Boxes a hair apart, inside the margin, which is where every resting pile is:
 GJK's distance is the gap, with no EPA.
*>
fn void boxes_a_hair_apart_read_the_gap() @test
{
	uint state = 4242;
	for (int trial = 0; trial < 200; trial++)
	{
		TransformedShape a = place(&UNIT, { 0, 0, 0 });
		// On the top face, turned about the vertical only, `gap` above it.
		float gap = (next(&state) + 1) * 0.002f;
		float angle = next(&state) * 1.5f;
		Quaternionf spin = { .v = { 0, math::sin(angle), 0, math::cos(angle) } };
		TransformedShape b = place(&PLANK, { next(&state) * 0.3f, 0.5f + 0.05f + gap, next(&state) * 0.3f }, spin);
		CollisionInfo info = collision::convex_contact(&a, &b, MARGIN);
		test::@check(info.collided, "trial %d: %f apart", trial, gap);
		test::@check(math::abs(info.depth + gap) < 1e-4f, "trial %d: read %f for a gap of %f", trial, info.depth, gap);
		test::@check(info.normal.y > 0.999f, "trial %d: normal %s", trial, info.normal);
	}
}

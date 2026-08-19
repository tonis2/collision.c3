<*
 The IK solver: that a reachable target is reached without the links stretching
 to get there, that an unreachable one leaves the chain straight and says how
 far short it fell, and that the rotations it reports describe the points it
 moved.

	`c3c test --test-filter collision_tests::kinematics`

 The stretch half of the first check is the one that matters. A solver that let
 the segments grow would reach *any* target, so "the effector arrived" on its
 own is a check a broken solver passes.

 Named `kinematics` rather than `ik` on purpose: this module imports
 `collision::ik` and uses it unqualified everywhere, and a module of its own
 called `collision_tests::ik` would put a second meaning on that path.

 Nothing here allocates — the chains are stack arrays and the solver writes into
 them — so a leak reported against this file is a leak in `ik` itself.
*>
module collision_tests::kinematics;

import collision;
import collision::ik;
import std::core::test;
import std::math;
import std::math::quaternion;

alias Vec3 = float[<3>];

// A straight three-link chain along +X, each link one unit, anchored at the
// origin. Short enough to predict every answer by hand, which is the whole
// reason the solver takes points rather than a skeleton.
macro straight_chain() => (Vec3[4]){ { 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 }, { 3, 0, 0 } };

<*
 A target inside the chain's reach is reached, and the links keep their lengths.

 The second half is the one that matters. A solver that let the segments stretch
 would reach *any* target, so "the effector arrived" on its own is a check that
 a broken solver passes.
*>
fn void test_ik_reaches_and_keeps_its_lengths() @test
{
	Vec3[4] points = straight_chain();
	Quaternionf[3] turns;
	Vec3 target = { 1.0, 2.0, 0.5 };

	float gap = ik::solve_chain(points[..], turns[..], target, tolerance: 0.0005)!!;
	test::@check(gap <= 0.0005f, "the effector stopped %s away from a target it can reach", gap);

	Vec3[4] rest = straight_chain();
	for (usz i = 1; i < 4; i++)
	{
		float was = (rest[i] - rest[i - 1]).length();
		float now = (points[i] - points[i - 1]).length();
		test::@check(math::abs(now - was) < 0.0005f, "link %d stretched from %s to %s", i, was, now);
	}

	// The anchor is the one point a solve may never move: everything is measured
	// from it, and a chain that drifts has no fixed end to hang from.
	test::@check(points[0].length() < 0.0001f, "the anchor moved to %s", points[0]);
}

<*
 A target beyond the chain's length leaves it straight and pointing at the
 target, with the residual saying exactly how far short it fell.

 Not an error and not a fault: a person dragging past the limb's length wants to
 see it extend and stop, which is what every editor does, and the number is what
 they would want shown.
*>
fn void test_ik_out_of_reach_straightens() @test
{
	Vec3[4] points = straight_chain();
	Quaternionf[3] turns;
	Vec3 target = { 0, 9, 0 };

	float gap = ik::solve_chain(points[..], turns[..], target)!!;

	// Three unit links from the origin toward a target nine units away. The
	// residual converges long before the shape does — it is dominated by the
	// chain's overall direction, so the default sweep count settles it while the
	// links are still a few degrees from true.
	test::@check(math::abs(gap - 6.0f) < 0.01f, "residual %s, expected about 6", gap);

	// Given the budget to finish, the chain goes properly straight. This needs
	// more than the default: the whole chain has to swing a quarter turn to get
	// here, and `max_step` deliberately rations that across sweeps.
	points = straight_chain();
	ik::solve_chain(points[..], turns[..], target, passes: 64)!!;

	Vec3 aim = (target - points[0]).normalize();
	for (usz i = 1; i < 4; i++)
	{
		Vec3 link = (points[i] - points[i - 1]).normalize();
		test::@check(link.dot(aim) > 0.999f, "link %d is off the aim, dot %s", i, link.dot(aim));
	}
}

<*
 The reported turns describe what actually happened to the points.

 crig's whole conversion rests on this: it takes `turns` as accumulated world
 rotations about each pivot and rewrites a skeleton's local rotations from them.
 If the convention here were each pivot's *own* turn rather than the total, every
 caller below the first pivot would be wrong by its ancestors — and the points
 would still look right, so nothing else would catch it.
*>
fn void test_ik_turns_describe_the_points() @test
{
	Vec3[4] before = straight_chain();
	Vec3[4] points = straight_chain();
	Quaternionf[3] turns;

	ik::solve_chain(points[..], turns[..], (Vec3){ 0.5, 1.5, 1.0 }, tolerance: 0.0005)!!;

	for (usz pivot = 0; pivot < 3; pivot++)
	{
		// The pivot's own position is where the turn is taken about, and it is
		// already at its solved place — the turn below it did not move it.
		Vec3 arm = before[pivot + 1] - before[pivot];
		Vec3 want = points[pivot + 1] - points[pivot];
		Vec3 got = ik::turned(turns[pivot], arm);
		test::@check((got - want).length() < 0.001f,
			"pivot %d: its turn sends the arm to %s, but the point went to %s", pivot, got, want);
	}
}

<*
 One sweep cannot turn a pivot further than the clamp allows.

 What keeps a pointer that jumped across the screen from folding a limb through
 itself in a single frame.
*>
fn void test_ik_clamps_one_step() @test
{
	Vec3[2] points = { { 0, 0, 0 }, { 1, 0, 0 } };
	Quaternionf[1] turns;

	// Straight back the way it came — a half turn — allowed one sweep and a
	// tenth of a radian.
	ik::solve_chain(points[..], turns[..], (Vec3){ -1, 0, 0 }, passes: 1, max_step: 0.1)!!;

	float moved = (float)math::acos(math::clamp(points[1].normalize().dot((Vec3){ 1, 0, 0 }), -1.0f, 1.0f));
	test::@check(moved <= 0.1f + 0.001f, "one sweep turned the pivot %s radians", moved);
	test::@check(moved > 0.05f, "the clamp stopped the pivot turning at all");
}

// Both ways the caller can mis-size the buffers. A fault rather than a clamp,
// because a solve that quietly ran over a chain nobody meant returns numbers
// that look like an answer.
fn void test_ik_refuses_a_chain_that_is_not_one() @test
{
	Vec3[1] alone = { { 0, 0, 0 } };
	Quaternionf[1] turns;
	test::@check(@catch(ik::solve_chain(alone[..], turns[..], (Vec3){ 1, 0, 0 })) == ik::IK_CHAIN_SHAPE);

	Vec3[4] points = straight_chain();
	Quaternionf[2] short_turns;
	test::@check(@catch(ik::solve_chain(points[..], short_turns[..], (Vec3){ 1, 1, 1 })) == ik::IK_CHAIN_SHAPE);
}

// The antiparallel case has no shortest turn — every half-turn about a
// perpendicular is equally short — so the only thing to assert is that one is
// picked and that it is a half turn about something perpendicular.
fn void test_ik_shortest_arc_survives_a_reversal() @test
{
	Quaternionf half = ik::shortest_arc({ 1, 0, 0 }, { -1, 0, 0 });
	Vec3 sent = ik::turned(half, (Vec3){ 1, 0, 0 });
	test::@check((sent - (Vec3){ -1, 0, 0 }).length() < 0.001f, "a reversal came out as %s", sent);

	// And the two degenerate inputs that have no direction at all. Through
	// `equals` and a named constant, because `quaternion::IDENTITY` is generic
	// and needs a type to infer from, and a quaternion has no `==` at all.
	Quaternionf still = quaternion::IDENTITY;
	test::@check(ik::shortest_arc({ 0, 0, 0 }, { 1, 0, 0 }).equals(still), "a zero-length input turned something");
	test::@check(ik::shortest_arc({ 1, 0, 0 }, { 1, 0, 0 }).equals(still), "a vector onto itself turned something");
}

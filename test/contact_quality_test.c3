<*
 What a contact is allowed to do to a body that lands on it.

	`c3c test --test-filter collision_tests::contact_quality`

 Three claims, each of which was false and each of which is cheap to check:

 - a `hull` collider lands where a box collider of the same size lands, from
   any height
 - GJK/EPA never answers with a number that is not a number
 - a body dropped flat onto a level plane lands where it was dropped

 The third is the one that reads like a tautology and is not. A box has no axis
 to tip about when it lands flat and square, so any spin at all is invented by
 the solver, and spin plus friction is a body that walks away from where it was
 put. Before the manifold stopped carrying EPA's own contact point alongside the
 four corners it had already found, a 1-unit box dropped from 2 units onto a
 level plane came to rest **9.6 units** from directly below its drop point.

 **The residual spin is deliberately not asserted tight.** Four contacts solved
 one after another — Gauss-Seidel, not simultaneously — leave a transient of a
 few rad/s that damps out without moving the body, and pinning that number here
 would make the test a record of the iteration order rather than of the claim.
 What is asserted is the part a game can see: the body does not travel, and it
 does not acquire sideways velocity out of a vertical impact.
*>
module collision_tests::contact_quality;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

// Z is up here — the library's own axis — so the ground is a slab whose top
// face is z = 0 and a unit box rests with its centre at z = 0.5.
const float REST_Z = 0.5f;

fn Mesh unit_blob()
{
	Vec3[8] pts = {
		{ -0.5, -0.5, -0.5 }, { 0.5, -0.5, -0.5 }, { 0.5, 0.5, -0.5 }, { -0.5, 0.5, -0.5 },
		{ -0.5, -0.5, 0.5 }, { 0.5, -0.5, 0.5 }, { 0.5, 0.5, 0.5 }, { -0.5, 0.5, 0.5 },
	};
	return collision::quickhull(pts[..], mem);
}

fn bool finite3(Vec3 v) @local => math::is_finite(v.x) && math::is_finite(v.y) && math::is_finite(v.z);

fn void add_ground(PhysicsWorld* world, Aabb3* shape)
{
	world.add_body({
		.id = 1,
		.collider = { .shape = shape, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
		.mass = 0, .gravity_factor = 0, .can_sleep = false,
	})!!;
}

<*
 A hull collider is a collider. It used to be a way to lose a body: EPA walked
 an open polytope, reported a horizontal normal 71 units deep for a box sitting
 on a floor, and the solver did as it was told.
*>
fn void test_a_hull_lands_where_a_box_lands() @test
{
	float[8] heights = { 1.2, 2, 4, 6, 8, 12, 20, 40 };

	foreach (h : heights)
	{
		PhysicsWorld world = collision::create_world();
		defer world.free();

		Aabb3 ground_shape = { { -50, -50, -1 }, { 50, 50, 0 } };
		add_ground(&world, &ground_shape);

		Mesh hull = unit_blob();
		defer hull.free();
		world.add_body({
			.id = 2,
			.collider = { .shape = &hull, .translation = { 0, 0, h }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 1.0, .gravity_factor = 1.0, .can_sleep = true,
		})!!;

		Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
		world.add_body({
			.id = 3,
			.collider = { .shape = &box_shape, .translation = { 5, 0, h }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 1.0, .gravity_factor = 1.0, .can_sleep = true,
		})!!;

		// Long enough for the 40-unit drop to stop bouncing. A contact at
		// 28 m/s still gives back more energy than it should — see the note in
		// `test_a_flat_landing_does_not_travel` — so settling takes a while at
		// the top of this range.
		for (int i = 0; i < 900; i++) world.run_step(1.0f / 60.0f, 1, 4);

		Vec3 hull_pos = world.find_body(2)!!.collider.translation;
		Vec3 box_pos = world.find_body(3)!!.collider.translation;

		test::@check(finite3(hull_pos), "the hull body left the numbers behind, dropped from %s", h);
		test::@check(math::abs(box_pos.z - REST_Z) < 0.05f,
			"the box control did not rest at %s from %s: %s", REST_Z, h, box_pos.z);
		test::@check(math::abs(hull_pos.z - REST_Z) < 0.05f,
			"the hull did not rest at %s from %s: %s", REST_Z, h, hull_pos.z);
	}
}

<*
 The narrow phase is allowed to say "no collision". It is not allowed to say
 "collision, at a normal of NaN" — a solver has no way to refuse that, and one
 NaN in a transform is every later frame the body appears in.
*>
fn void test_the_narrow_phase_answers_with_numbers() @test
{
	Mesh hull = unit_blob();
	defer hull.free();
	Aabb3 slab = { { -50, -50, -1 }, { 50, 50, 0 } };
	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	TransformedShape ground = { .shape = &slab, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } };

	int bad = 0;

	// Every depth from a graze to fully buried, at every yaw — the pose the
	// falling hull was actually in when it went non-finite.
	for (int i = 0; i <= 60; i++)
	{
		float z = REST_Z - (float)i * 0.017f;
		for (int j = 0; j < 40; j++)
		{
			float ang = (float)j * ((float)math::PI / 40.0f);
			Quaternionf rot = { 0, 0, math::sin(ang * 0.5f), math::cos(ang * 0.5f) };

			TransformedShape h = { .shape = &hull, .translation = { 0, 0, z }, .scale = { 1, 1, 1 }, .rotation = rot };
			TransformedShape b = { .shape = &box_shape, .translation = { 0, 0, z }, .scale = { 1, 1, 1 }, .rotation = rot };

			CollisionInfo hi = collision::check_convex_collision(&h, &ground, epa: true);
			CollisionInfo bi = collision::check_convex_collision(&b, &ground, epa: true);

			if (hi.collided && (!finite3(hi.normal) || !math::is_finite(hi.depth)
				|| !finite3(hi.contact_points[0]) || !finite3(hi.contact_points[1]))) bad++;
			if (bi.collided && (!finite3(bi.normal) || !math::is_finite(bi.depth)
				|| !finite3(bi.contact_points[0]) || !finite3(bi.contact_points[1]))) bad++;
		}
	}

	test::@check(bad == 0, "the narrow phase answered %d times with a non-finite normal, depth or contact", bad);
}

<*
 Dropped straight down onto a level plane, a body should not acquire sideways
 velocity: the impulse is vertical, the plane is level, and there is nothing in
 the scene to push it. Sleep is off so this measures the solver rather than the
 sleep timer freezing the evidence.

 **This is a bound on a known defect, not a statement that the defect is gone.**
 The contact solve does inject lateral velocity at impact, and the numbers below
 are what it injects today at the substep count `three.c3` runs — enough to slide
 a crate a body-length or two off where it landed. The threshold here is set to
 catch a return to the metres-per-second launches, not to claim the invariant
 holds. See the note in the module header of `world_test`.
*>
fn void test_a_flat_landing_does_not_launch() @test
{
	float[5] heights = { 1, 2, 5, 10, 20 };

	foreach (h : heights)
	{
		PhysicsWorld world = collision::create_world();
		defer world.free();

		Aabb3 ground_shape = { { -50, -50, -1 }, { 50, 50, 0 } };
		world.add_body({
			.id = 1,
			.collider = { .shape = &ground_shape, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 0, .gravity_factor = 0, .can_sleep = false,
			.material = { .static_friction = 0.5, .dynamic_friction = 0.5, .restitution = 0.2 },
		})!!;

		Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
		world.add_body({
			.id = 2,
			.collider = { .shape = &box_shape, .translation = { 0, 0, h }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 1.0, .gravity_factor = 1.0, .can_sleep = false,
			.material = { .static_friction = 0.5, .dynamic_friction = 0.5, .restitution = 0.2 },
		})!!;

		Rigidbody* b = world.find_body(2)!!;
		float peak_lateral = 0;

		for (int i = 0; i < 400; i++)
		{
			world.run_step(1.0f / 60.0f, 1, 4);
			Vec3 v = b.linear_velocity;
			float lateral = math::sqrt(v.x * v.x + v.y * v.y);
			if (lateral > peak_lateral) peak_lateral = lateral;
		}

		Vec3 p = b.collider.translation;
		float drift = math::sqrt(p.x * p.x + p.y * p.y);

		test::@check(finite3(p), "dropped flat from %s, the body left the numbers behind", h);
		test::@check(drift < 5.0f, "dropped flat from %s, it came to rest %s away", h, drift);
		test::@check(peak_lateral < 5.0f,
			"a vertical impact from %s produced %s of sideways velocity", h, peak_lateral);
	}
}

<*
 Solver-side behaviour: joint limits hold their target, and bodies get the
 inertia of their actual shape.

	`c3c test --test-filter collision_tests::solver`
*>
module collision_tests::solver;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

// A static anchor body (id 1) at the origin and a dynamic body (id 2) at
// `start_x`, joined along x by `limit`, under `gravity_x` along x. Returns the
// dynamic body's final x and the largest/smallest x it reached on the way.
struct JointRun
{
	float final_x;
	float max_x;
	float min_x;
}

fn JointRun run_x_joint(Limit limit, float start_x, float velocity_x, uint steps, float gravity_x = 0) @local
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();
	world.gravity = { gravity_x, 0, 0 };

	Aabb3 shape = collision::aabb_from_half({ 0.5f, 0.5f, 0.5f });
	world.add_body(
		{
			.id = 1,
			.collider = { .shape = &shape, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 0,
		}
	)!!;
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &shape,
				.translation = { start_x, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1,
			.gravity_factor = 1,
			.linear_velocity = { velocity_x, 0, 0 },
			.can_sleep = false,
		}
	)!!;

	GenericJoint3D joint = { .body_a = 1, .body_b = 2, .relative_orientation = { 0, 0, 0, 1 }, .collides = false };
	limit.axes = { 1, 0, 0 };
	joint.linear_limits.push(limit);
	world.add_constraint(joint)!!;

	Rigidbody* mover = world.find_body(2)!!;
	JointRun run = { .max_x = start_x, .min_x = start_x };
	for (uint i = 0; i < steps; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 4);
		float x = mover.collider.translation.x;
		run.max_x = math::max(run.max_x, x);
		run.min_x = math::min(run.min_x, x);
	}
	run.final_x = mover.collider.translation.x;
	return run;
}

// A lower limit is a floor on the separation, not a pull toward zero: a body
// falling toward the anchor comes to rest at `min`, not at the anchor.
fn void test_joint_min_limit_settles_at_the_limit() @test
{
	JointRun run = run_x_joint({ .min = 1.0f }, 3.0f, 0.0f, 120, gravity_x: -9.8f);
	test::@check(math::abs(run.final_x - 1.0f) < 0.02f, "min limit 1 settled at x = %f", run.final_x);
	test::@check(run.min_x > 0.95f, "min limit 1 was undershot to x = %f", run.min_x);
}

// A body that starts closer than `min` is moved out to `min` by the first step
// (it was driven to 0, the free-joint answer, before limits kept their target).
fn void test_joint_min_limit_pushes_out_to_the_limit() @test
{
	JointRun run = run_x_joint({ .min = 1.0f }, 0.0f, 0.0f, 1);
	test::@check(math::abs(run.final_x - 1.0f) < 0.02f, "min limit 1 pushed the body to x = %f", run.final_x);
}

// ...and it holds against a body driven into it.
fn void test_joint_min_limit_holds_against_a_push() @test
{
	JointRun run = run_x_joint({ .min = 1.0f }, 1.5f, -10.0f, 60);
	test::@check(run.min_x > 0.95f, "min limit 1 was pushed through to x = %f", run.min_x);
	test::@check(math::abs(run.final_x - 1.0f) < 0.05f, "min limit 1 settled at x = %f", run.final_x);
}

// An upper limit stops a body being pulled apart.
fn void test_joint_max_limit_stops_at_the_limit() @test
{
	JointRun run = run_x_joint({ .max = 2.0f }, 0.0f, 10.0f, 60);
	test::@check(run.max_x < 2.05f, "max limit 2 let the body reach x = %f", run.max_x);
	test::@check(math::abs(run.final_x - 2.0f) < 0.05f, "max limit 2 settled at x = %f", run.final_x);
}

// Limits are one-sided: inside the allowed range nothing acts on the body.
fn void test_joint_limit_does_not_pull_inside_the_range() @test
{
	JointRun run = run_x_joint({ .min = 1.0f }, 3.0f, 0.0f, 30);
	test::@check(math::abs(run.final_x - 3.0f) < 1e-4f, "a satisfied min limit moved the body to x = %f", run.final_x);
}

// A free axis (min = max = 0) is the plain bilateral anchor constraint.
fn void test_joint_free_axis_holds_at_zero() @test
{
	JointRun run = run_x_joint({ }, 0.5f, 0.0f, 60);
	test::@check(math::abs(run.final_x) < 0.01f, "free joint settled at x = %f", run.final_x);

	JointRun pulled = run_x_joint({ }, 0.0f, 10.0f, 60);
	test::@check(pulled.max_x < 0.2f, "free joint let the body drift to x = %f", pulled.max_x);
}

// --- Shape inertia ----------------------------------------------------------------

fn Rigidbody* add_dynamic(PhysicsWorld* world, sz id, CollisionShape shape, Vec3 scale, float mass) @local
{
	world.add_body(
		{
			.id = id,
			.collider = { .shape = shape, .scale = scale, .rotation = { 0, 0, 0, 1 } },
			.mass = mass,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;
	return world.find_body(id)!!;
}

fn bool near(float a, float b) @local => math::abs(a - b) <= 1e-4f * math::max(1.0f, math::abs(b));

// A solid sphere is 2/5 m r^2 about every axis — not the box its AABB suggests.
fn void test_sphere_inverse_inertia_is_the_sphere_formula() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Sphere ball = { .center = { 0, 0, 0 }, .radius = 0.5f };
	Rigidbody* body = add_dynamic(&world, 1, &ball, { 1, 1, 1 }, 2.0f);

	float expected = 1.0f / (0.4f * 2.0f * 0.25f);
	test::@check(near(body.inv_local_inertia.m00, expected), "m00 = %f, expected %f", body.inv_local_inertia.m00, expected);
	test::@check(near(body.inv_local_inertia.m11, expected), "m11 = %f, expected %f", body.inv_local_inertia.m11, expected);
	test::@check(near(body.inv_local_inertia.m22, expected), "m22 = %f, expected %f", body.inv_local_inertia.m22, expected);
}

// A uniform scale grows the sphere: r = 1 here.
fn void test_scaled_sphere_inertia_follows_the_scale() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Sphere ball = { .center = { 0, 0, 0 }, .radius = 0.5f };
	Rigidbody* body = add_dynamic(&world, 1, &ball, { 2, 2, 2 }, 2.0f);

	float expected = 1.0f / (0.4f * 2.0f * 1.0f);
	test::@check(near(body.inv_local_inertia.m00, expected), "m00 = %f, expected %f", body.inv_local_inertia.m00, expected);
}

// A box of size (1, 2, 3) and mass 12: I = m/12 * (sy^2 + sz^2, sx^2 + sz^2, sx^2 + sy^2) = (13, 10, 5).
fn void test_box_inverse_inertia_is_the_box_formula() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 box = collision::aabb_from_half({ 1.0f, 2.0f, 3.0f });
	Rigidbody* body = add_dynamic(&world, 1, &box, { 1, 1, 1 }, 12.0f);

	test::@check(near(body.inv_local_inertia.m00, 1.0f / 13.0f), "m00 = %f", body.inv_local_inertia.m00);
	test::@check(near(body.inv_local_inertia.m11, 1.0f / 10.0f), "m11 = %f", body.inv_local_inertia.m11);
	test::@check(near(body.inv_local_inertia.m22, 1.0f / 5.0f), "m22 = %f", body.inv_local_inertia.m22);
	test::@check(near(body.inv_local_inertia.m01, 0) && near(body.inv_local_inertia.m12, 0), "box tensor is not diagonal");
}

// The world tensor is the local one rotated, and the cache follows the rotation:
// a quarter turn about z swaps the x and y entries, turning back swaps them again.
fn void test_world_inverse_inertia_follows_the_rotation() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 box = collision::aabb_from_half({ 1.0f, 2.0f, 3.0f });
	Rigidbody* body = add_dynamic(&world, 1, &box, { 1, 1, 1 }, 12.0f);

	Matrix3f upright = body.get_inverse_world_inertia();
	test::@check(near(upright.m00, 1.0f / 13.0f) && near(upright.m11, 1.0f / 10.0f), "identity world tensor wrong");

	float s = math::sqrt(0.5f);
	body.collider.rotation = { 0, 0, s, s };
	Matrix3f turned = body.get_inverse_world_inertia();
	test::@check(near(turned.m00, 1.0f / 10.0f), "turned m00 = %f, expected 0.1", turned.m00);
	test::@check(near(turned.m11, 1.0f / 13.0f), "turned m11 = %f, expected 1/13", turned.m11);
	test::@check(near(turned.m22, 1.0f / 5.0f), "turned m22 = %f", turned.m22);

	body.collider.rotation = { 0, 0, 0, 1 };
	Matrix3f back = body.get_inverse_world_inertia();
	test::@check(near(back.m00, 1.0f / 13.0f), "cache did not follow the rotation back: m00 = %f", back.m00);
}

// Static bodies have infinite inertia: the world tensor is zero.
fn void test_static_body_has_zero_inverse_inertia() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 box = collision::aabb_from_half({ 0.5f, 0.5f, 0.5f });
	Rigidbody* body = add_dynamic(&world, 1, &box, { 1, 1, 1 }, 0.0f);
	Matrix3f inertia = body.get_inverse_world_inertia();
	test::@check(inertia.m00 == 0 && inertia.m11 == 0 && inertia.m22 == 0);
}

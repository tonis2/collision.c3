<*
 Lockstep determinism: the same inputs must produce the same bits, however the
 work was divided and however the timeline was rewound.

	`c3c test --test-filter collision_tests::determinism`

 Three claims, each a different way for determinism to break. Two worlds fed
 identical operations stay bit-equal at every step. One worker or three makes no
 difference, because the islands are disjoint and the parallel solve is
 value-deterministic. And a snapshot restored replays the timeline it was taken
 from — which is the check that finds simulation state the snapshot forgot,
 since anything missing from it diverges immediately on replay.

 `test_dmath_accuracy` is the floor the other two stand on: dmath has to track
 libm to a few ulps while staying bit-identical across platforms, and only the
 first half of that is testable here. Bit-equality across platforms is checked
 by comparing golden state hashes between platform builds.

 `WorldSnapshot` is an allocation, and `defer snap.free()` is what the
 leak-tracked run holds the rollback test to.
*>
module collision_tests::determinism;

import collision;
import collision::dmath;
import collision_testutil;
import std::core::test;
import std::math;

// dmath must track libm closely (a few float ulps) while staying bit-identical
// across platforms. Accuracy is testable everywhere; bit-equality is checked
// by comparing golden state hashes across platform builds.
fn void test_dmath_accuracy() @test
{
	ulong seed = 12345;
	for (int i = 0; i < 10_000; i++)
	{
		seed = seed * 6364136223846793005u + 1442695040888963407u;
		float x = (float)((double)(seed >> 33) / 2147483648.0 * 40.0 - 20.0); // [-20, 20)

		test::@check(math::abs(dmath::sin(x) - (float)math::sin((double)x)) < 2e-6f);
		test::@check(math::abs(dmath::cos(x) - (float)math::cos((double)x)) < 2e-6f);

		// sincos must agree bit-exactly with the individual functions
		float[<2>] sc = dmath::sincos(x);
		test::@check(sc[0] == dmath::sin(x));
		test::@check(sc[1] == dmath::cos(x));

		seed = seed * 6364136223846793005u + 1442695040888963407u;
		float y = (float)((double)(seed >> 33) / 2147483648.0 * 40.0 - 20.0);
		if (x != 0 || y != 0)
		{
			test::@check(math::abs(dmath::atan2(y, x) - (float)math::atan2((double)y, (double)x)) < 2e-6f);
		}

		// pow over damping-style inputs: base (0, 2], exponent [-4, 4]
		float base = math::abs(x) * 0.1f + 1e-4f;
		float exponent = y * 0.2f;
		float want = (float)math::pow((double)base, (double)exponent);
		test::@check(math::abs(dmath::pow(base, exponent) - want) < math::abs(want) * 4e-6f + 1e-7f);
	}
}

// Mixed scene for the determinism tests: boxes and spheres stacking on a
// ground mesh — exercises manifolds, friction, islands, and sleep.
// Macro because fns inside a @test module may not take parameters.
macro void add_determinism_bodies(PhysicsWorld* world, Mesh* ground, Aabb3* box_shape, Sphere* sphere_shape)
{
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = ground,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	float[8] xs = { -1.2f, -0.6f, 0.0f, 0.6f, 1.2f, 0.2f, -0.4f, 0.8f };
	float[8] ys = { 0.3f, -0.5f, 0.1f, 0.7f, -0.2f, -0.8f, 0.5f, 0.0f };

	for (int i = 0; i < 8; i++)
	{
		bool is_box = i % 2 == 0;
		world.add_body(
			{
				.id = (sz)(10 + i),
				.collider = {
					.shape = is_box ? (CollisionShape)box_shape : (CollisionShape)sphere_shape,
					.translation = { xs[i], ys[i], 1.0f + 0.8f * (float)i },
					.scale = { 1, 1, 1 },
					.rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0f,
				.gravity_factor = 1.0f,
				.can_sleep = true,
				.material = { .static_friction = 0.5f, .dynamic_friction = 0.4f, .restitution = 0.3f },
			}
		)!!;
	}
}

// Two worlds fed identical operations must stay bit-equal at every step
fn void test_determinism_repeat() @test
{
	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();
	Aabb3 box_shape = { { -0.4f, -0.4f, -0.4f }, { 0.4f, 0.4f, 0.4f } };
	Sphere sphere_shape = { .center = { 0, 0, 0 }, .radius = 0.4f };

	PhysicsWorld world_a = collision::DEFAULT_PHYSICS_WORLD;
	PhysicsWorld world_b = collision::DEFAULT_PHYSICS_WORLD;
	defer world_a.free();
	defer world_b.free();

	add_determinism_bodies(&world_a, &ground, &box_shape, &sphere_shape);
	add_determinism_bodies(&world_b, &ground, &box_shape, &sphere_shape);

	for (int i = 0; i < 120; i++)
	{
		world_a.run_step(1.0f / 60.0f, step_count: 4);
		world_b.run_step(1.0f / 60.0f, step_count: 4);
		test::@check(world_a.state_hash() == world_b.state_hash());
	}
}

// Worker count must not change results: islands are disjoint, so the
// parallel solve is value-deterministic. Guards that invariant.
fn void test_determinism_thread_count() @test
{
	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();
	Aabb3 box_shape = { { -0.4f, -0.4f, -0.4f }, { 0.4f, 0.4f, 0.4f } };
	Sphere sphere_shape = { .center = { 0, 0, 0 }, .radius = 0.4f };

	PhysicsWorld world_a = collision::DEFAULT_PHYSICS_WORLD;
	PhysicsWorld world_b = collision::DEFAULT_PHYSICS_WORLD;
	world_a.thread_count = 1;
	world_b.thread_count = 3;
	defer world_a.free();
	defer world_b.free();

	add_determinism_bodies(&world_a, &ground, &box_shape, &sphere_shape);
	add_determinism_bodies(&world_b, &ground, &box_shape, &sphere_shape);

	for (int i = 0; i < 90; i++)
	{
		world_a.run_step(1.0f / 60.0f, step_count: 4);
		world_b.run_step(1.0f / 60.0f, step_count: 4);
	}

	test::@check(world_a.state_hash() == world_b.state_hash());
}

// Rollback: snapshot, simulate ahead, restore, re-simulate — both timelines
// must be bit-equal. Catches any simulation state the snapshot misses.
fn void test_snapshot_restore_rollback() @test
{
	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();
	Aabb3 box_shape = { { -0.4f, -0.4f, -0.4f }, { 0.4f, 0.4f, 0.4f } };
	Sphere sphere_shape = { .center = { 0, 0, 0 }, .radius = 0.4f };

	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();
	add_determinism_bodies(&world, &ground, &box_shape, &sphere_shape);

	for (int i = 0; i < 60; i++) world.run_step(1.0f / 60.0f, step_count: 4);

	WorldSnapshot snap = world.snapshot();
	defer snap.free();
	ulong hash_at_snapshot = world.state_hash();

	for (int i = 0; i < 30; i++) world.run_step(1.0f / 60.0f, step_count: 4);
	ulong hash_original = world.state_hash();
	test::@check(hash_original != hash_at_snapshot); // world must actually have advanced

	world.restore(&snap)!!;
	test::@check(world.state_hash() == hash_at_snapshot); // restore is bit-exact

	for (int i = 0; i < 30; i++) world.run_step(1.0f / 60.0f, step_count: 4);
	ulong hash_replayed = world.state_hash();

	test::@check(hash_original == hash_replayed); // replay reproduces the timeline
}

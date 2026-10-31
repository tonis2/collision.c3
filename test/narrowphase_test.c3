<*
 Narrow-phase regressions that sit between the shape tests and the world tests.

	`c3c test --test-filter collision_tests::narrowphase`
*>
module collision_tests::narrowphase;
import collision;
import std::core::test;
import std::math;

// Many bodies resting on one heightfield, on the default thread count.
//
// **Regression.** The heightfield pair test used to build its triangle list on
// the temp allocator, and collision workers are threads that never ran
// `@pool_init`, so the first batch handed to a worker panicked. It needed enough
// pairs to be split across workers, which is why a handful of bodies never saw it.
fn void test_heightfield_many_bodies_default_threads() @test
{
	uint side = 21;
	float[441] heights;
	for (uint j = 0; j < side; j++)
	{
		for (uint i = 0; i < side; i++) heights[j * side + i] = 0.1f * (float)i;
	}
	Heightmap field = collision::create_heightmap_from_data(&heights, side, side, 1.0f, { -10, 0, -10 });
	defer field.free();

	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.gravity = { 0, -9.8f, 0 };
	world.ground_plane = { .normal = { 0, 1, 0 }, .offset = -1000.0f };
	defer world.free();

	world.add_body(
		{
			.id = 1,
			.collider = { .shape = &field, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 0, .gravity_factor = 0, .can_sleep = false,
		}
	)!!;

	Sphere ball = { .center = { 0, 0, 0 }, .radius = 0.3f };
	int n = 20;
	for (int i = 0; i < n; i++)
	{
		float x = -4.0f + (float)(i % 5) * 1.5f;
		float z = -3.0f + (float)(i / 5) * 1.5f;
		world.add_body(
			{
				.id = (sz)(10 + i),
				.collider = {
					.shape = &ball, .translation = { x, 2.0f, z }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0f, .gravity_factor = 1.0f, .can_sleep = false,
			}
		)!!;
	}

	for (int i = 0; i < 180; i++) world.run_step(1.0f / 60.0f, step_count: 2);

	for (int i = 0; i < n; i++)
	{
		Rigidbody* b = world.find_body((sz)(10 + i))!!;
		test::@check(b.collider.translation.y > -1.0f, "ball %d fell through, y=%f", i, b.collider.translation.y);
	}
}

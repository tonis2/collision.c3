<*
 Performance baseline: one timed scenario per hot path, so a refactor can be
 measured against a number rather than a feeling.

	c3c test --test-filter collision_tests::bench -O3 2>&1 | grep BENCH

 Each test prints `BENCH <name> <ms> ms` and asserts something trivial (finite,
 above the floor) so it is also a valid test. Timings are only meaningful at
 -O3; a debug build is several times slower.
*>
module collision_tests::bench;

import collision;
import collision_testutil;
import std::core::test;
import std::collections::list;
import std::io;
import std::math;
import std::time;

alias Vec3 = float[<3>];

macro bool finite3(Vec3 v) => !math::is_nan(v.x + v.y + v.z) && math::abs(v.x + v.y + v.z) < 1e6f;

// Scattered, deterministic position on a grid so bodies do not start overlapped.
macro Vec3 grid_pos(int i, int per_row, float spacing, float z0, float layer)
{
	int col = i % per_row;
	int row = (i / per_row) % per_row;
	int lay = i / (per_row * per_row);
	return { ((float)col - (float)per_row * 0.5f) * spacing, ((float)row - (float)per_row * 0.5f) * spacing, z0 + (float)lay * layer };
}

fn void test_bench_box_pile() @test
{
	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	world.add_body(
		{
			.id = 1,
			.collider = { .shape = &ground, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 0, .gravity_factor = 0, .can_sleep = false,
		}
	)!!;

	Aabb3 box_shape = { { -0.4f, -0.4f, -0.4f }, { 0.4f, 0.4f, 0.4f } };
	Sphere sphere_shape = { .center = { 0, 0, 0 }, .radius = 0.4f };
	Capsule capsule_shape = { .height = 1, .radius_top = 0.3f, .radius_bottom = 0.3f };

	int n = 300;
	for (int i = 0; i < n; i++)
	{
		CollisionShape shape = (CollisionShape)&box_shape;
		if (i % 5 == 3) shape = (CollisionShape)&sphere_shape;
		if (i % 5 == 4) shape = (CollisionShape)&capsule_shape;
		world.add_body(
			{
				.id = (sz)(10 + i),
				.collider = { .shape = shape, .translation = grid_pos(i, 10, 1.3f, 1.0f, 1.2f), .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
				.mass = 1.0f, .gravity_factor = 1.0f, .can_sleep = true,
				.material = { .static_friction = 0.5f, .dynamic_friction = 0.4f, .restitution = 0.1f },
			}
		)!!;
	}

	Clock clock = clock::now();
	for (int i = 0; i < 120; i++) world.run_step(1.0f / 60.0f, step_count: 4);
	io::printfn("BENCH box_pile %d ms", clock.mark().to_ms());
	// The ground is finite (and the odd capsule slips through it), so a few
	// bodies leaving the pile is normal; most must be resting on it.
	int fallen = 0;
	for (int i = 0; i < n; i++)
	{
		Rigidbody* b = world.find_body((sz)(10 + i))!!;
		test::@check(finite3(b.collider.translation), "body %d went non-finite", i);
		if (b.collider.translation.z < -1.0f) fallen++;
	}
	test::@check(fallen < n / 10, "%d of %d bodies fell through or off the ground", fallen, n);
}

fn void test_bench_heightfield_drop() @test
{
	uint side = 41;
	float[1681] heights;
	for (uint j = 0; j < side; j++)
	{
		for (uint i = 0; i < side; i++)
		{
			heights[j * side + i] = 0.4f * math::sin((float)i * 0.4f) * math::cos((float)j * 0.3f);
		}
	}
	Heightmap field = collision::create_heightmap_from_data(&heights, side, side, 0.5f, { -10, 0, -10 });
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

	Aabb3 box_shape = { { -0.3f, -0.3f, -0.3f }, { 0.3f, 0.3f, 0.3f } };
	Sphere sphere_shape = { .center = { 0, 0, 0 }, .radius = 0.3f };
	int n = 100;
	for (int i = 0; i < n; i++)
	{
		Vec3 p = grid_pos(i, 10, 1.2f, 2.0f, 1.0f);
		world.add_body(
			{
				.id = (sz)(10 + i),
				.collider = {
					.shape = i % 2 == 0 ? (CollisionShape)&box_shape : (CollisionShape)&sphere_shape,
					.translation = { p.x, p.z, p.y }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0f, .gravity_factor = 1.0f, .can_sleep = true,
				.material = { .static_friction = 0.5f, .dynamic_friction = 0.4f, .restitution = 0.1f },
			}
		)!!;
	}

	Clock clock = clock::now();
	for (int i = 0; i < 240; i++) world.run_step(1.0f / 60.0f, step_count: 4);
	io::printfn("BENCH heightfield_drop %d ms", clock.mark().to_ms());

	for (int i = 0; i < n; i++)
	{
		Rigidbody* b = world.find_body((sz)(10 + i))!!;
		test::@check(finite3(b.collider.translation), "body %d went non-finite", i);
		test::@check(b.collider.translation.y > -5.0f, "body %d fell through, y=%f", i, b.collider.translation.y);
	}
}

fn void test_bench_softbody() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.enable_ground_plane();
	defer world.free();

	Aabb3 ground_box = { { -10, -10, -1 }, { 10, 10, 0 } };
	world.add_body(
		{
			.id = 100,
			.collider = { .shape = &ground_box, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 0, .gravity_factor = 0, .can_sleep = false,
		}
	)!!;

	Aabb3 box_shape = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	world.add_body(
		{
			.id = 101,
			.collider = { .shape = &box_shape, .translation = { 0, 0, 0.5f }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
			.mass = 2.0f, .gravity_factor = 1.0f, .can_sleep = false,
		}
	)!!;

	Mesh cube = collision_testutil::create_test_cube();
	defer cube.free();
	Mesh tetra = collision_testutil::create_test_tetrahedron();
	defer tetra.free();

	int count = 64;
	for (int i = 0; i < count; i++)
	{
		Mesh* src = i % 2 == 0 ? &cube : &tetra;
		Mesh moved = *src;
		// Work on a private copy of the vertices: the soft body copies positions at creation.
		float dx = ((float)(i % 8) - 3.5f) * 2.0f;
		float dy = ((float)((i / 8) % 8) - 3.5f) * 2.0f;
		for (sz v = 0; v < src.vertices.len(); v++)
		{
			src.vertices[v] += { dx, dy, 3.0f };
		}
		SoftBody sb = collision::create_softbody_from_mesh(src, 1.0, (sz)(200 + i));
		for (sz v = 0; v < src.vertices.len(); v++)
		{
			src.vertices[v] -= { dx, dy, 3.0f };
		}
		world.add_softbody(sb);
		(void)moved;
	}

	Clock clock = clock::now();
	for (int i = 0; i < 600; i++) world.run_step(1.0f / 60.0f);
	io::printfn("BENCH softbody %d ms", clock.mark().to_ms());

	for (int i = 0; i < count; i++)
	{
		SoftBody* found = world.find_softbody((sz)(200 + i))!!;
		foreach (p : found.positions)
		{
			test::@check(finite3(p), "soft body %d went non-finite", i);
			test::@check(p.z > -1.0f, "soft body %d fell through, z=%f", i, p.z);
		}
	}
}

fn void test_bench_decompose() @test
{
	// A U shape: three boxes, concave in the notch.
	Mesh mesh;
	mesh.init(mem, 24, 36);
	defer mesh.free();
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { -1.5, -0.5, 0 }, { -0.5, 0.5, 2 });
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { 0.5, -0.5, 0 }, { 1.5, 0.5, 2 });
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { -1.5, -0.5, -1 }, { 1.5, 0.5, 0 });
	mesh.aabb = mesh.get_aabb();

	Clock clock = clock::now();
	List {Mesh} pieces;
	for (int i = 0; i < 40; i++)
	{
		pieces = mesh.decompose(collision::COACD_FAST);
		if (i < 39)
		{
			foreach (&p : pieces) p.free();
			pieces.free();
		}
	}
	io::printfn("BENCH decompose %d ms", clock.mark().to_ms());

	test::@check(pieces.len() >= 2, "a U shape decomposed into %d pieces", pieces.len());
	foreach (&p : pieces)
	{
		test::@check(p.vertices.len() >= 4);
		p.free();
	}
	pieces.free();
}

fn void test_bench_joints_chain() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 link_shape = { { -0.5f, -0.1f, -0.1f }, { 0.5f, 0.1f, 0.1f } };
	int n = 40;
	for (int i = 0; i < n; i++)
	{
		world.add_body(
			{
				.id = (sz)(i + 1),
				.collider = { .shape = &link_shape, .translation = { (float)i * 1.0f, 0, 20 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
				.mass = i == 0 ? 0.0f : 1.0f,
				.gravity_factor = i == 0 ? 0.0f : 1.0f,
				.can_sleep = false,
			}
		)!!;
	}
	for (int i = 0; i + 1 < n; i++)
	{
		collision::GenericJoint3D joint = {
			.body_a = (sz)(i + 1),
			.body_b = (sz)(i + 2),
			.anchor_a = { 0.5f, 0, 0 },
			.anchor_b = { -0.5f, 0, 0 },
			.relative_orientation = { 0, 0, 0, 1 },
			.collides = false,
		};
		joint.linear_limits.push({ .axes = { 1, 0, 0 }, .stiffness = 100 });
		joint.linear_limits.push({ .axes = { 0, 1, 0 }, .stiffness = 100 });
		joint.linear_limits.push({ .axes = { 0, 0, 1 }, .stiffness = 100 });
		world.add_constraint(joint)!!;
	}

	Clock clock = clock::now();
	for (int i = 0; i < 1200; i++) world.run_step(1.0f / 60.0f, step_count: 10);
	io::printfn("BENCH joints_chain %d ms", clock.mark().to_ms());

	for (int i = 0; i < n; i++)
	{
		Rigidbody* b = world.find_body((sz)(i + 1))!!;
		test::@check(finite3(b.collider.translation), "link %d went non-finite", i);
	}
	Rigidbody* last = world.find_body((sz)n)!!;
	test::@check(last.collider.translation.z < 20.0f, "the free end did not swing down");
}

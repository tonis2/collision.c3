<*
 Soft bodies built from a mesh: what `create_softbody_from_mesh` derives from
 one, and what the XPBD solve then does to it under gravity, pins, ground and a
 volume constraint.

	`c3c test --test-filter collision_tests::softbody`

 Each of these owns two allocations at once — the `Mesh` it starts from and the
 `SoftBody` derived from it — and the soft body holds six lists of its own.
 `test_softbody_in_physics_world` is the one to watch: it hands the body to the
 world with `add_softbody` and does *not* free it, on the claim that the world
 took ownership. Under the leak-tracked run that claim is checked on every run
 rather than assumed.
*>
module collision_tests::softbody;

import collision;
import collision_testutil;
import std::core::test;

alias Vec3 = float[<3>];

fn void test_softbody_creation() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Should have 4 particles
	test::@check(sb.positions.len() == 4, "a tetrahedron made %d particles", sb.positions.len());
	test::@check(sb.velocities.len() == 4, "%d velocities for 4 particles", sb.velocities.len());
	test::@check(sb.inverse_masses.len() == 4, "%d inverse masses for 4 particles", sb.inverse_masses.len());

	// Tetrahedron has 6 edges
	test::@check(sb.distance_constraints.len() == 6, "a tetrahedron has 6 edges, got %d constraints", sb.distance_constraints.len());

	// Should have 4 triangles
	test::@check(sb.triangles.len() == 4, "a tetrahedron has 4 faces, got %d", sb.triangles.len());

	// All particles should have equal inverse mass
	float expected_inv_mass = 4.0f; // 1.0 total / 4 particles = 0.25 each, inv = 4.0
	test::@check(sb.inverse_masses[0] > 3.9f && sb.inverse_masses[0] < 4.1f,
		"1.0 over 4 particles is an inverse mass of 4, got %f", sb.inverse_masses[0]);
}

fn void test_softbody_cube_creation() @test
{
	Mesh mesh = collision_testutil::create_test_cube();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 2.0, 2);
	defer sb.free();

	// Should have 8 particles
	test::@check(sb.positions.len() == 8, "a cube made %d particles", sb.positions.len());

	// Cube has 12 triangles, 18 edges
	test::@check(sb.triangles.len() == 12, "a triangulated cube has 12 faces, got %d", sb.triangles.len());
	test::@check(sb.distance_constraints.len() == 18, "a cube has 18 edges, got %d constraints", sb.distance_constraints.len());

	// AABB should bound the cube
	Aabb3 aabb = sb.aabb;
	test::@check(aabb.min.x <= -0.49f && aabb.max.x >= 0.49f);
	test::@check(aabb.min.y <= -0.49f && aabb.max.y >= 0.49f);
	test::@check(aabb.min.z <= -0.49f && aabb.max.z >= 0.49f);
}

fn void test_softbody_gravity() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Record initial z positions
	float initial_z0 = sb.positions[0].z;
	float initial_z1 = sb.positions[1].z;

	// Simulate with gravity for several steps
	Vec3 gravity = { 0, 0, -9.8f };
	for (int i = 0; i < 10; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// All particles should have moved downward
	test::@check(sb.positions[0].z < initial_z0);
	test::@check(sb.positions[1].z < initial_z1);
}

fn void test_softbody_ground_collision() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Simulate with gravity for many steps — should settle on ground
	Vec3 gravity = { 0, 0, -9.8f };
	for (int i = 0; i < 300; i++)
	{
		sb.generate_ground_collisions({ .normal = { 0, 0, 1 } }, 0.5f);
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// No particle should be significantly below the ground plane
	foreach (p : sb.positions)
	{
		test::@check(p.z >= -0.01f);
	}
}

fn void test_softbody_pin() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Pin vertex 0 to its current position
	Vec3 pin_pos = sb.positions[0];
	sb.pin_particle(0, pin_pos);

	// Simulate
	Vec3 gravity = { 0, 0, -9.8f };
	for (int i = 0; i < 50; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// Pinned particle should not have moved
	Vec3 diff = sb.positions[0] - pin_pos;
	test::@check(diff.length_sq() < 0.001f);

	// Other particles should have moved downward from gravity
	test::@check(sb.positions[1].z < 0.0f);
}

fn void test_softbody_distance_constraint_stiffness() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Record initial distances between particles
	Vec3 p0_init = sb.positions[0];
	Vec3 p1_init = sb.positions[1];
	float initial_dist = (p1_init - p0_init).length();

	// Make constraints very stiff (compliance = 0)
	sb.set_distance_compliance(0.0f);

	// Simulate with gravity
	Vec3 gravity = { 0, 0, -9.8f };
	sb.generate_ground_collisions({ .normal = { 0, 0, 1 } }, 0.5f);
	for (int i = 0; i < 100; i++)
	{
		sb.generate_ground_collisions({ .normal = { 0, 0, 1 } }, 0.5f);
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// Distance between connected particles should be close to rest length
	float current_dist = (sb.positions[1] - sb.positions[0]).length();
	float error = current_dist - initial_dist;
	// Allow some tolerance due to solver iterations
	test::@check(error < 0.5f && error > -0.5f);
}

fn void test_softbody_volume_preservation() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Enable volume preservation
	float initial_volume = sb.compute_signed_volume();
	sb.enable_volume_preservation(0.0f);

	// Pin top vertex to prevent free-fall
	sb.pin_particle(0, sb.positions[0]);

	// Simulate
	Vec3 gravity = { 0, 0, -9.8f };
	for (int i = 0; i < 100; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// Volume should be roughly preserved
	float final_volume = sb.compute_signed_volume();
	float volume_ratio = final_volume / initial_volume;
	// Allow 50% deviation (volume preservation is approximate)
	test::@check(volume_ratio > 0.5f && volume_ratio < 1.5f);
}

fn void test_softbody_in_physics_world() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.enable_ground_plane();
	defer world.free();

	// Add a static rigidbody (ground box)
	Aabb3 ground_box = { { -10, -10, -1 }, { 10, 10, 0 } };
	world.add_body(
		{
			.id = 100,
			.collider = {
				.shape = &ground_box,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0, // Static
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Create and add a soft body above the ground
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	// Translate vertices up
	for (sz i = 0; i < mesh.vertices.len(); i++)
	{
		mesh.vertices[i].z += 5.0f;
	}

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 200);
	world.add_softbody(sb);

	// Simulate for many steps
	for (int i = 0; i < 300; i++)
	{
		world.run_step(1.0f / 60.0f);
	}

	// Soft body should have settled near the ground
	SoftBody* found = world.find_softbody(200)!!;
	foreach (p : found.positions)
	{
		test::@check(p.z >= -0.1f); // Should be above ground
		test::@check(p.z < 6.0f); // Should have fallen from initial position
	}
}

fn void test_softbody_sleep() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();
	sb.can_sleep = true;

	// All velocities are zero, should be considered not moving
	test::@check(!sb.is_moving());

	// Give it some velocity
	sb.velocities[0] = { 0, 0, 1.0f };
	test::@check(sb.is_moving());

	// Reset
	sb.velocities[0] = { 0, 0, 0 };
	test::@check(!sb.is_moving());
}

fn void test_softbody_bending() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Enable bending
	sb.enable_bending(0.001f);

	// Tetrahedron has 6 edges, each shared by 2 triangles
	// So we should have some bending constraints
	test::@check(sb.bending_constraints.len() > 0, "enabling bending produced no constraints");
}

fn void test_softbody_compliance() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Set high compliance (soft)
	sb.set_distance_compliance(0.1f);

	foreach (dc : sb.distance_constraints)
	{
		test::@check(dc.compliance > 0.09f && dc.compliance < 0.11f);
	}

	// Set zero compliance (rigid)
	sb.set_distance_compliance(0.0f);

	foreach (dc : sb.distance_constraints)
	{
		test::@check(dc.compliance == 0.0f);
	}
}

// The ground plane's whole point: a scalar height cannot describe this at all.
fn void test_ground_plane_can_tilt() @test
{
	Mesh mesh = collision_testutil::create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	Vec3 normal = ((Vec3){ 0, 1, 1 }).normalize();
	Vec3 gravity = normal * -9.8f;

	for (int i = 0; i < 300; i++)
	{
		sb.generate_ground_collisions({ .normal = normal, .offset = 0 }, 0.5f);
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	foreach (p : sb.positions)
	{
		test::@check(p.dot(normal) >= -0.01f);
	}
}

module collision_tests @test;

import collision;
import collision::dmath;
import pool_test_util;

import std::math;
import std::io;
import std::collections::list;

alias Vec2 = float[<2>];
alias Vec3 = float[<3>];
alias Vec4 = float[<4>];

alias Vertices = List {Vec3};
alias Triangles = List {Triangle};

fn void test_physics()
{
	Aabb3 aabb = { { -1.0, -1.0, -1.0 }, { 1.0, 1.0, 1.0 } };

	aabb = aabb.transform(translation: { 1, 5, 1 }, scale: { 3.0, 1.0, 1.0 }, rotation: { 0, 0, 0, 1 });

	assert (aabb.min == { -2, 4, 0 });
	assert (aabb.max == { 4, 6, 2 });

	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	Aabb3[] boxes = { { { 0, 0, 0 }, { 1, 1, 1 } }, { { 1, 1, 1 }, { 2.5, 2.5, 2.5 } }, { { 5, 5, 5 }, { 6, 6, 6 } }, };

	foreach (sz i, item : boxes) spatial_map.insert(item, i)!!;

	spatial_map.@get_pairs(; Pair pair)
	{
		assert (pair.a == 0 && pair.b == 1);
	};

	Vec3[] cube = {
		{ -1, 0, -1 },
		{ 1, 0, -1 },
		{ -1, 0, 1 },
		{ 1, 0, -1 },
		{ 1, 0, 1 },
		{ -1, 0, 1 },
		{ -1, -1, 0 },
		{ 1, -1, 0 },
		{ 0, 1, 0 },
		{ 3, 1, -1 },
		{ 3, 1, 1 },
		{ 1, 0, 1 }
	};

	Mesh shape_1;
	shape_1.vertices.push_all(cube);
	defer shape_1.free();

	TransformedShape transformed_shape_1 = {
		.shape = &shape_1,
		.translation = { -2.0, 0, 0 },
		.scale = { 1, 1, 1 },
		.rotation = { 0, 0, 0, 1 }
	};

	TransformedShape transformed_shape_2 = {
		.shape = &shape_1,
		.translation = { 2.0, 0, 0 },
		.scale = { 1, 1, 1 },
		.rotation = { 0, 0, 0, 1 }
	};

	TransformedShape transformed_shape_3 = {
		.shape = &shape_1,
		.translation = { 0.0, 0, 0 },
		.scale = { 1, 1, 1 },
		.rotation = { 0, 0, 0, 1 }
	};

	TransformedShape transformed_shape_4 = {
		.shape = &shape_1,
		.translation = { 0, 0, 0 },
		.scale = { 3, 3, 3 },
		.rotation = { 0, 0, 0, 1 }
	};

	TransformedShape transformed_shape_5 = {
		.shape = &shape_1,
		.translation = { -1.0, 0, 0 },
		.scale = { 1, 1, 1 },
		.rotation = { 0, 0.5, 0, 1 }
	};

	TransformedShape transformed_shape_6 = {
		.shape = &shape_1,
		.translation = { 1.0, 0, 0 },
		.scale = { 1, 1, 1 },
		.rotation = { 0, -0.5, 0, 1 }
	};

	assert (!collision::check_convex_collision(&transformed_shape_1, &transformed_shape_2).collided);
	assert (collision::check_convex_collision(&transformed_shape_5, &transformed_shape_6).collided);
	assert (collision::check_convex_collision(&transformed_shape_3, &transformed_shape_4).collided);


	// CollisionInfo epa_result = epa(collision_result.simplex, &shape_1, &shape_2);

	// {
	// Aabb3 aabb = {
	// {3.486510, 1693.368896, 2.556290},
	// {3.537758, 1698.706421, 2.556290}
	// };
	// }
}

fn void test_ray()
{
	Ray testRay = { { 0.0, 0.0, 0.0 }, (Vec3) { 0.0, 0.0, 1 }.normalize() };

	Sphere testSphere = { { 0, 0, 5 }, 1.0 };
	assert (testSphere.intersect(testRay) > 0);
}


fn void test_aabb()
{
	// Aabb3 box = collision::aabb_from_half({1,1,1});
	// Matrix3f intertia = box.inertia_tensor(3.0);

	// Matrix3f result = {
	// 1, 0, 0,
	// 0, 1, 0,
	// 0, 0, 1
	// };

	// assert(intertia == result);
}

fn void test_bvh()
{
	Vec3[] terrain = {
		{ -1, 0, -1 },
		{ 1, 0, -1 },
		{ -1, 0, 1 },
		{ 1, 0, -1 },
		{ 1, 0, 1 },
		{ -1, 0, 1 },
		{ -1, -1, 0 },
		{ 1, -1, 0 },
		{ 0, 1, 0 },
		{ 3, 1, -1 },
		{ 3, 1, 1 },
		{ 1, 0, 1 }
	};

	Vec3[] terrain2 = {
		{ 0, 0, 0 },
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 10, 10, 10 },
		{ 11, 10, 10 },
		{ 10, 11, 10 },
		{ -10, -10, -10 },
		{ -9, -10, -10 },
		{ -10, -9, -10 },
	};

	Triangle[] terrain_tris = { { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 }, { 9, 10, 11 } };
	Triangle[] terrain2_tris = { { 0, 1, 2 }, { 3, 4, 5 }, { 6, 7, 8 } };

	TriBVHNode* bvh = collision::create_indexed_tri_bvh(terrain.ptr, terrain_tris.ptr, terrain_tris.len);
	TriBVHNode* bvh2 = collision::create_indexed_tri_bvh(terrain2.ptr, terrain2_tris.ptr, terrain2_tris.len);

	defer
	{
		bvh.free_indexed();
		mem::free(bvh);

		bvh2.free_indexed();
		mem::free(bvh2);
	}

	TriangleVerts triangle = { { -1.0f, -1.0f, 0.0f }, { 1.0f, -1.0f, 0.0f }, { 0.0f, 1.0f, 0.0f } };

	Ray test_ray = { .origin = { 0.0f, 0.0f, 5.0f }, .direction = { 0, 0, -1 } };

	Aabb3 box = { .min = { -1.0f, -1.0f, -1.0f }, .max = { 1.0f, 1.0f, 1.0f } };

	Aabb3 box2 = { .min = { -0.1f, -0.1f, -0.1f }, .max = { 0.1f, 0.1f, 0.1f } };

	Aabb3 box3 = { .min = { 4.0f, 4.0f, 4.0f }, .max = { 6.0f, 6.0f, 6.0f } };

	assert (bvh2.box_intersects(box2).collided);
	assert (bvh2.box_intersects(box3).collided == false);

	assert (triangle.intersect_ray(test_ray).collided);
	// assert(bvh.ray_intersects(test_ray).collided);

	HitInfo result_2 = bvh.box_intersects(box);
	assert (result_2.collided == true);

	Mesh tri_1;
	tri_1.vertices.push_all({ { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.5f, 0.0f } });
	Mesh tri_2;
	tri_2.vertices.push_all({ { 1.1f, -2.0f, 0.0f }, { 1.1f, 2.0f, 0.0f }, { 1.1f, 0.0f, 2.0f } });
	Mesh tri_3;
	tri_3.vertices.push_all({ { 15.5f, 15.5f, 15.5f }, { -12.0f, 10.0f, 10.0f }, { 12.0f, 10.0f, 10.0f } });

	defer
	{
		tri_1.free();
		tri_2.free();
		tri_3.free();
	}

	assert (collision::check_convex_collision(&box, &tri_1).collided);
	assert (!collision::check_convex_collision(&box, &tri_2).collided);
	assert (!collision::check_convex_collision(&box, &tri_3).collided);
	// assert(collision::check_convex_collision(&box, &&result_2.triangle.get_aabb()).collided);
}

fn void test_remesh()
{
	Vertices vertices;
	Triangles triangles;

	vertices.push_all(
		{
			{ 0.0f, 0.0f, 0.0f },
			{ 1.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f },
			{ 1.0f, 0.0f, 0.0 },
			{ 1.0f, 1.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f }
		}
	);

	triangles.push_all({ { 0, 1, 2 }, { 3, 4, 5 } });

	collision::weld_remesh(vertices: &vertices, triangles: &triangles);

	assert (vertices.len() == 4);
	assert (triangles.len() == 2);
}

fn void test_spatial_hash_update()
{
	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	// Insert a box at position (0,0,0) to (1,1,1)
	Aabb3 box1_old = { { 0, 0, 0 }, { 0.9, 0.9, 0.9 } };
	spatial_map.insert(box1_old, 0)!!;

	// Insert another box that overlaps
	Aabb3 box2 = { { 0.5, 0.5, 0.5 }, { 1.5, 1.5, 1.5 } };
	spatial_map.insert(box2, 1)!!;

	// Verify they form a pair (both in cell 0,0,0)
	sz pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
		assert (pair.a == 0 && pair.b == 1);
	};
	assert (pair_count == 1);

	// Move box1 to a non-overlapping position
	Aabb3 box1_new = { { 5, 5, 5 }, { 5.9, 5.9, 5.9 } };
	spatial_map.update(box1_old, box1_new, 0)!!;

	// Verify no pairs after move
	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	assert (pair_count == 0);
}

fn void test_spatial_hash_update_same_cell()
{
	SpatialHash3D spatial_map = { .cell_size = 2.0 };
	defer spatial_map.free();

	// Insert a box
	Aabb3 box_old = { { 0.1, 0.1, 0.1 }, { 0.5, 0.5, 0.5 } };
	spatial_map.insert(box_old, 0)!!;

	// Move within the same cell (both positions map to cell 0,0,0)
	Aabb3 box_new = { { 0.2, 0.2, 0.2 }, { 0.6, 0.6, 0.6 } };
	spatial_map.update(box_old, box_new, 0)!!;

	// Should still be in the map
	sz count = 0;
	spatial_map.@get_nearby_objects({ 0.3, 0.3, 0.3 } ; sz id)
	{
		if (id == 0) count++;
	};
	assert (count >= 1);
}

fn void test_spatial_hash_update_cross_cells()
{
	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	// Insert a box spanning cells (0,0,0) and (1,0,0)
	Aabb3 box_old = { { 0.5, 0.5, 0.5 }, { 1.5, 0.9, 0.9 } };
	spatial_map.insert(box_old, 0)!!;

	// Insert a stationary box in cell (2,0,0)
	Aabb3 box_stationary = { { 2.1, 0.5, 0.5 }, { 2.9, 0.9, 0.9 } };
	spatial_map.insert(box_stationary, 1)!!;

	// Initially no collision (boxes don't share cells)
	sz pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	assert (pair_count == 0);

	// Move box to span cells (1,0,0) and (2,0,0) - now overlaps with stationary box's cell
	Aabb3 box_new = { { 1.5, 0.5, 0.5 }, { 2.5, 0.9, 0.9 } };
	spatial_map.update(box_old, box_new, 0)!!;

	// Should now have a pair
	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	assert (pair_count == 1);
}

fn void test_spatial_hash_multiple_updates()
{
	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	Aabb3 box = { { 0, 0, 0 }, { 0.9, 0.9, 0.9 } };
	spatial_map.insert(box, 0)!!;

	// Perform multiple sequential updates
	for (int i = 1; i <= 5; i++)
	{
		Aabb3 new_box = { { (float)i, 0, 0 }, { (float)i + 0.9, 0.9, 0.9 } };
		spatial_map.update(box, new_box, 0)!!;
		box = new_box;
	}

	// Verify the box is now at position (5,0,0)
	sz count = 0;
	spatial_map.@get_nearby_objects({ 5.5, 0.5, 0.5 } ; sz id)
	{
		if (id == 0) count++;
	};
	assert (count >= 1);

	// Verify it's NOT at the old position
	count = 0;
	spatial_map.@get_nearby_objects({ 0.5, 0.5, 0.5 } ; sz id)
	{
		if (id == 0) count++;
	};
	assert (count == 0);
}

// Helper: create a simple tetrahedron mesh for soft body testing
macro Mesh create_test_tetrahedron()
{
	Mesh mesh;
	// Tetrahedron vertices
	mesh.vertices.push_all(
		{
			{ 0.0f, 0.0f, 1.0f }, // Top
			{ -1.0f, -1.0f, 0.0f }, // Base left
			{ 1.0f, -1.0f, 0.0f }, // Base right
			{ 0.0f, 1.0f, 0.0f }, // Base back
		}
	);

	// 4 triangular faces
	mesh.triangles.push_all({ { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 }, });
	return mesh;
}

// Helper: create a simple cube mesh for soft body testing
macro Mesh create_test_cube()
{
	Mesh mesh;
	mesh.vertices.push_all(
		{
			{ -0.5f, -0.5f, -0.5f },
			{ 0.5f, -0.5f, -0.5f },
			{ 0.5f, 0.5f, -0.5f },
			{ -0.5f, 0.5f, -0.5f },
			{ -0.5f, -0.5f, 0.5f },
			{ 0.5f, -0.5f, 0.5f },
			{ 0.5f, 0.5f, 0.5f },
			{ -0.5f, 0.5f, 0.5f },
		}
	);
	mesh.triangles.push_all(
		{ // Front
			{ 0, 1, 2 },
			{ 0, 2, 3 }, // Back
			{ 5, 4, 7 },
			{ 5, 7, 6 }, // Top
			{ 3, 2, 6 },
			{ 3, 6, 7 }, // Bottom
			{ 4, 5, 1 },
			{ 4, 1, 0 }, // Right
			{ 1, 5, 6 },
			{ 1, 6, 2 }, // Left
			{ 4, 0, 3 },
			{ 4, 3, 7 },
		}
	);
	return mesh;
}

fn void test_softbody_creation()
{
	Mesh mesh = create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Should have 4 particles
	assert (sb.positions.len() == 4);
	assert (sb.velocities.len() == 4);
	assert (sb.inverse_masses.len() == 4);

	// Tetrahedron has 6 edges
	assert (sb.distance_constraints.len() == 6);

	// Should have 4 triangles
	assert (sb.triangles.len() == 4);

	// All particles should have equal inverse mass
	float expected_inv_mass = 4.0f; // 1.0 total / 4 particles = 0.25 each, inv = 4.0
	assert (sb.inverse_masses[0] > 3.9f && sb.inverse_masses[0] < 4.1f);
}

fn void test_softbody_cube_creation()
{
	Mesh mesh = create_test_cube();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 2.0, 2);
	defer sb.free();

	// Should have 8 particles
	assert (sb.positions.len() == 8);

	// Cube has 12 triangles, 18 edges
	assert (sb.triangles.len() == 12);
	assert (sb.distance_constraints.len() == 18);

	// AABB should bound the cube
	Aabb3 aabb = sb.aabb;
	assert (aabb.min.x <= -0.49f && aabb.max.x >= 0.49f);
	assert (aabb.min.y <= -0.49f && aabb.max.y >= 0.49f);
	assert (aabb.min.z <= -0.49f && aabb.max.z >= 0.49f);
}

fn void test_softbody_gravity()
{
	Mesh mesh = create_test_tetrahedron();
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
	assert (sb.positions[0].z < initial_z0);
	assert (sb.positions[1].z < initial_z1);
}

fn void test_softbody_ground_collision()
{
	Mesh mesh = create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Simulate with gravity for many steps — should settle on ground
	Vec3 gravity = { 0, 0, -9.8f };
	for (int i = 0; i < 300; i++)
	{
		sb.generate_ground_collisions(0.0f, 0.5f);
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// No particle should be significantly below the ground plane
	foreach (p : sb.positions)
	{
		assert (p.z >= -0.01f);
	}
}

fn void test_softbody_pin()
{
	Mesh mesh = create_test_tetrahedron();
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
	assert (diff.length_sq() < 0.001f);

	// Other particles should have moved downward from gravity
	assert (sb.positions[1].z < 0.0f);
}

fn void test_softbody_distance_constraint_stiffness()
{
	Mesh mesh = create_test_tetrahedron();
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
	sb.generate_ground_collisions(0.0f, 0.5f);
	for (int i = 0; i < 100; i++)
	{
		sb.generate_ground_collisions(0.0f, 0.5f);
		sb.xpbd_solve(gravity, 1.0f / 60.0f);
	}

	// Distance between connected particles should be close to rest length
	float current_dist = (sb.positions[1] - sb.positions[0]).length();
	float error = current_dist - initial_dist;
	// Allow some tolerance due to solver iterations
	assert (error < 0.5f && error > -0.5f);
}

fn void test_softbody_volume_preservation()
{
	Mesh mesh = create_test_tetrahedron();
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
	assert (volume_ratio > 0.5f && volume_ratio < 1.5f);
}

fn void test_softbody_in_physics_world()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.enable_ground_plane(0.0f);
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
	Mesh mesh = create_test_tetrahedron();
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
		assert (p.z >= -0.1f); // Should be above ground
		assert (p.z < 6.0f); // Should have fallen from initial position
	}
}

fn void test_softbody_sleep()
{
	Mesh mesh = create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();
	sb.can_sleep = true;

	// All velocities are zero, should be considered not moving
	assert (!sb.is_moving());

	// Give it some velocity
	sb.velocities[0] = { 0, 0, 1.0f };
	assert (sb.is_moving());

	// Reset
	sb.velocities[0] = { 0, 0, 0 };
	assert (!sb.is_moving());
}

fn void test_softbody_bending()
{
	Mesh mesh = create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Enable bending
	sb.enable_bending(0.001f);

	// Tetrahedron has 6 edges, each shared by 2 triangles
	// So we should have some bending constraints
	assert (sb.bending_constraints.len() > 0);
}

fn void test_softbody_compliance()
{
	Mesh mesh = create_test_tetrahedron();
	defer mesh.free();

	SoftBody sb = collision::create_softbody_from_mesh(&mesh, 1.0, 1);
	defer sb.free();

	// Set high compliance (soft)
	sb.set_distance_compliance(0.1f);

	foreach (dc : sb.distance_constraints)
	{
		assert (dc.compliance > 0.09f && dc.compliance < 0.11f);
	}

	// Set zero compliance (rigid)
	sb.set_distance_compliance(0.0f);

	foreach (dc : sb.distance_constraints)
	{
		assert (dc.compliance == 0.0f);
	}
}

fn void test_xpbd_rigid_contact()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	// Static ground plane (large box at z=-1 to z=0)
	Aabb3 ground_shape = { { -50, -50, -1 }, { 50, 50, 0 } };
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground_shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0, // Static
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Dynamic box starting above ground
	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = true,
		}
	)!!;

	// Simulate for 300 steps (5 seconds at 60fps)
	for (int i = 0; i < 300; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 20);
	}

	// Box should have settled on ground — z position should be near 0.5 (half-height above ground)
	Rigidbody* box = world.find_body(2)!!;
	float z = box.collider.translation.z;

	assert (z > -0.1f); // Not fallen through ground
	assert (z < 2.0f); // Has fallen from initial position of 5
}

fn void test_worker_pool()
{
	WorkerPool pool;
	pool.init(3)!!;
	defer pool.destroy();
	assert (pool.workers() == 3);

	// More tasks than queue capacity (workers * 4) to exercise blocking push,
	// several rounds to exercise join + reuse
	int[64] slots;
	for (int round = 0; round < 4; round++)
	{
		for (int i = 0; i < 64; i++) slots[i] = 0;
		for (int i = 0; i < 64; i++) pool.push(&pool_test_util::slot_worker, &slots[i]);
		pool.join();
		for (int i = 0; i < 64; i++) assert (slots[i] == 1);
	}
}

fn void test_world_thread_count()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.thread_count = 2;
	defer world.free();

	Aabb3 box_shape = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
		}
	)!!;

	world.run_step(1.0f / 60.0f);
	assert (world.thread_pool.workers() == 2);

	// Default (thread_count = 0) resolves to the machine's core count
	assert (collision::default_worker_count() >= 1);
}

// Helper: flat two-triangle ground mesh at z=0 spanning ±10, with BVH
macro Mesh create_test_ground_mesh()
{
	Mesh mesh;
	mesh.vertices.push_all(
		{
			{ -10.0f, -10.0f, 0.0f },
			{ 10.0f, -10.0f, 0.0f },
			{ -10.0f, 10.0f, 0.0f },
			{ 10.0f, 10.0f, 0.0f },
		}
	);
	// Consistent winding — face normals point up (+z)
	mesh.triangles.push_all({ { 0, 1, 2 }, { 1, 3, 2 } });
	mesh.aabb = mesh.get_aabb();
	mesh.build_bvh();
	return mesh;
}

fn void test_ccd_fast_projectile()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Mesh ground = create_test_ground_mesh();
	defer ground.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Small, very fast sphere: moves 2.5 units per step — far more than its
	// 0.1 radius — and would pass straight through the ground without CCD.
	Sphere bullet = { .center = { 0, 0, 0 }, .radius = 0.1f };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &bullet,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0.1f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
			.linear_velocity = { 0, 0, -150.0f },
		}
	)!!;

	for (int i = 0; i < 60; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 8);
	}

	// Projectile must have been stopped by the ground, not tunneled through
	float z = world.find_body(2)!!.collider.translation.z;
	assert (z > -0.5f);
	assert (z < 1.0f);
}

fn void test_mesh_contact_manifold()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Mesh ground = create_test_ground_mesh();
	defer ground.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Box dropped across the shared edge of the two triangles — needs the
	// multi-contact manifold to settle flat instead of rocking or sinking.
	Aabb3 box_shape = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 2 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
			.material = { .static_friction = 0.5f, .dynamic_friction = 0.4f },
		}
	)!!;

	for (int i = 0; i < 180; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 8);
	}

	// Box should rest on the surface: center near half its height
	float z = world.find_body(2)!!.collider.translation.z;
	assert (z > 0.3f);
	assert (z < 0.8f);
}

fn void test_xpbd_joint()
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

	// Body A — static anchor
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Body B — dynamic, connected by joint
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 2, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = false,
		}
	)!!;

	// Joint connecting body_a anchor to body_b anchor
	collision::GenericJoint3D joint = {
		.body_a = 1,
		.body_b = 2,
		.anchor_a = { 1, 0, 0 }, // Right edge of A
		.anchor_b = { -1, 0, 0 }, // Left edge of B
		.relative_orientation = { 0, 0, 0, 1 },
		.collides = false,
	};
	joint.linear_limits.push({ .axes = { 1, 0, 0 }, .stiffness = 100 });
	joint.linear_limits.push({ .axes = { 0, 1, 0 }, .stiffness = 100 });
	joint.linear_limits.push({ .axes = { 0, 0, 1 }, .stiffness = 100 });
	world.add_constraint(joint)!!;

	// Record initial distance between anchor points
	float initial_dist = 2.0f; // |{2,0,5} - {0,0,5}| = 2, anchors are at x=1 and x=-1+2=1

	// Simulate
	for (int i = 0; i < 200; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 20);
	}

	// Bodies should still be roughly connected — check distance between anchor points
	Rigidbody* body_a = world.find_body(1)!!;
	Rigidbody* body_b = world.find_body(2)!!;

	Vec3 anchor_world_a = body_a.collider.translation + body_a.collider.rotation * (Vec3) { 1, 0, 0 };
	Vec3 anchor_world_b = body_b.collider.translation + body_b.collider.rotation * (Vec3) { -1, 0, 0 };

	float dist = (anchor_world_b - anchor_world_a).length();

	// Joint should keep bodies connected (allow some tolerance for XPBD softness)
	assert (dist < 2.0f); // Should be much closer than free-falling apart

	// Body B should have moved downward from gravity
	assert (body_b.collider.translation.z < 5.0f);
}

// === Lockstep determinism (see determinism.md) ===

// dmath must track libm closely (a few float ulps) while staying bit-identical
// across platforms. Accuracy is testable everywhere; bit-equality is checked
// by comparing golden state hashes across platform builds.
fn void test_dmath_accuracy()
{
	ulong seed = 12345;
	for (int i = 0; i < 10_000; i++)
	{
		seed = seed * 6364136223846793005u + 1442695040888963407u;
		float x = (float)((double)(seed >> 33) / 2147483648.0 * 40.0 - 20.0); // [-20, 20)

		assert (math::abs(dmath::sin(x) - (float)math::sin((double)x)) < 2e-6f);
		assert (math::abs(dmath::cos(x) - (float)math::cos((double)x)) < 2e-6f);

		// sincos must agree bit-exactly with the individual functions
		float[<2>] sc = dmath::sincos(x);
		assert (sc[0] == dmath::sin(x));
		assert (sc[1] == dmath::cos(x));

		seed = seed * 6364136223846793005u + 1442695040888963407u;
		float y = (float)((double)(seed >> 33) / 2147483648.0 * 40.0 - 20.0);
		if (x != 0 || y != 0)
		{
			assert (math::abs(dmath::atan2(y, x) - (float)math::atan2((double)y, (double)x)) < 2e-6f);
		}

		// pow over damping-style inputs: base (0, 2], exponent [-4, 4]
		float base = math::abs(x) * 0.1f + 1e-4f;
		float exponent = y * 0.2f;
		float want = (float)math::pow((double)base, (double)exponent);
		assert (math::abs(dmath::pow(base, exponent) - want) < math::abs(want) * 4e-6f + 1e-7f);
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
fn void test_determinism_repeat()
{
	Mesh ground = create_test_ground_mesh();
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
		assert (world_a.state_hash() == world_b.state_hash());
	}
}

// Worker count must not change results: islands are disjoint, so the
// parallel solve is value-deterministic. Guards that invariant.
fn void test_determinism_thread_count()
{
	Mesh ground = create_test_ground_mesh();
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

	assert (world_a.state_hash() == world_b.state_hash());
}

// Rollback: snapshot, simulate ahead, restore, re-simulate — both timelines
// must be bit-equal. Catches any simulation state the snapshot misses.
fn void test_snapshot_restore_rollback()
{
	Mesh ground = create_test_ground_mesh();
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
	assert (hash_original != hash_at_snapshot); // world must actually have advanced

	world.restore(&snap)!!;
	assert (world.state_hash() == hash_at_snapshot); // restore is bit-exact

	for (int i = 0; i < 30; i++) world.run_step(1.0f / 60.0f, step_count: 4);
	ulong hash_replayed = world.state_hash();

	assert (hash_original == hash_replayed); // replay reproduces the timeline
}

module pool_test_util;

fn int slot_worker(void* arg)
{
	*(int*)arg += 1;
	return 0;
}

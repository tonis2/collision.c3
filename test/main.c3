module collision_tests @test;

import collision;
import collision::dmath;
import collision::ik;
import pool_test_util;

import std::math;
import std::math::quaternion;
import std::io;
import std::collections::list;

alias Vec2 = float[<2>];
alias Vec3 = float[<3>];
alias Vec4 = float[<4>];

alias Vertices = List {Vec3};
alias Triangles = List {Triangle};
// `Vertices` above is ambiguous with collision's own, which aliases a private
// Vec3 — so the voxel tests below name their point list separately.
alias BoxPoints = List {Vec3};

fn void test_physics()
{
	Aabb3 aabb = { { -1.0, -1.0, -1.0 }, { 1.0, 1.0, 1.0 } };

	aabb = aabb.transform(translation: { 1, 5, 1 }, scale: { 3.0, 1.0, 1.0 }, rotation: { 0, 0, 0, 1 });

	assert (aabb.min == { -2, 4, 0 });
	assert (aabb.max == { 4, 6, 2 });

	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	Aabb3[] boxes = { { { 0, 0, 0 }, { 1, 1, 1 } }, { { 1, 1, 1 }, { 2.5, 2.5, 2.5 } }, { { 5, 5, 5 }, { 6, 6, 6 } }, };

	foreach (sz i, item : boxes) spatial_map.insert(item, i);

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
	BoxPoints vertices;
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
	spatial_map.insert(box1_old, 0);

	// Insert another box that overlaps
	Aabb3 box2 = { { 0.5, 0.5, 0.5 }, { 1.5, 1.5, 1.5 } };
	spatial_map.insert(box2, 1);

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
	spatial_map.update(box1_old, box1_new, 0);

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
	spatial_map.insert(box_old, 0);

	// Move within the same cell (both positions map to cell 0,0,0)
	Aabb3 box_new = { { 0.2, 0.2, 0.2 }, { 0.6, 0.6, 0.6 } };
	spatial_map.update(box_old, box_new, 0);

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
	spatial_map.insert(box_old, 0);

	// Insert a stationary box in cell (2,0,0)
	Aabb3 box_stationary = { { 2.1, 0.5, 0.5 }, { 2.9, 0.9, 0.9 } };
	spatial_map.insert(box_stationary, 1);

	// Initially no collision (boxes don't share cells)
	sz pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	assert (pair_count == 0);

	// Move box to span cells (1,0,0) and (2,0,0) - now overlaps with stationary box's cell
	Aabb3 box_new = { { 1.5, 0.5, 0.5 }, { 2.5, 0.9, 0.9 } };
	spatial_map.update(box_old, box_new, 0);

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
	spatial_map.insert(box, 0);

	// Perform multiple sequential updates
	for (int i = 1; i <= 5; i++)
	{
		Aabb3 new_box = { { (float)i, 0, 0 }, { (float)i + 0.9, 0.9, 0.9 } };
		spatial_map.update(box, new_box, 0);
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

fn void test_spatial_hash_distinct_cells_no_false_pairs()
{
	// Regression: the old float-linear cell hash merged distinct cells —
	// grid cells (5,3,7), (5,4,-16) and (4,26,7) all shared one key — which
	// produced false candidate pairs between distant bodies.
	SpatialHash3D spatial_map = { .cell_size = 2.0 };
	defer spatial_map.free();

	// World positions landing exactly in those previously-colliding cells
	spatial_map.insert({ { 10.1, 6.1, 14.1 }, { 10.9, 6.9, 14.9 } }, 0); // cell (5,3,7)
	spatial_map.insert({ { 10.1, 8.1, -31.9 }, { 10.9, 8.9, -31.1 } }, 1); // cell (5,4,-16)
	spatial_map.insert({ { 8.1, 52.1, 14.1 }, { 8.9, 52.9, 14.9 } }, 2); // cell (4,26,7)

	sz pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	assert (pair_count == 0);
}

fn void test_spatial_hash_update_wide_body()
{
	// Regression: with the old hash two cells of one body's own range could
	// share a bucket (e.g. (0,1,0) and (0,0,23)); update() then removed both
	// copies of the id and the body vanished from a cell it still occupied.
	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	// Wide body spanning cells x 0, y 0..1, z 0..23
	Aabb3 wide_old = { { 0.1, 0.1, 0.1 }, { 0.9, 1.9, 23.9 } };
	spatial_map.insert(wide_old, 0);

	// Small box overlapping the far end, cell (0,0,23)
	spatial_map.insert({ { 0.2, 0.2, 23.2 }, { 0.8, 0.8, 23.8 } }, 1);

	sz pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		assert (pair.a == 0 && pair.b == 1);
		pair_count++;
	};
	assert (pair_count == 1);

	// Shrink the wide body's y range so cells at y 1 are vacated;
	// cell (0,0,23) stays occupied and the pair must survive.
	Aabb3 wide_new = { { 0.1, 0.1, 0.1 }, { 0.9, 0.9, 23.9 } };
	spatial_map.update(wide_old, wide_new, 0);

	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		assert (pair.a == 0 && pair.b == 1);
		pair_count++;
	};
	assert (pair_count == 1);
}

fn void test_spatial_hash_nearby_objects_cell_size()
{
	// Regression: @get_nearby_objects used to re-divide grid coordinates by
	// cell_size, querying the wrong cells for any cell_size != 1.
	SpatialHash3D spatial_map = { .cell_size = 2.0 };
	defer spatial_map.free();

	spatial_map.insert({ { 20.1, 0.1, 0.1 }, { 20.9, 0.9, 0.9 } }, 7); // cell (10,0,0)

	sz count = 0;
	spatial_map.@get_nearby_objects({ 21.0, 0.5, 0.5 } ; sz id)
	{
		if (id == 7) count++;
	};
	assert (count == 1);
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

fn void test_capsule_cylinder_shape_conventions()
{
	// Both shapes are centered at the origin with their axis along +y;
	// capsule height covers the cylindrical section only (KHR_implicit_shapes).
	Capsule capsule = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };

	Vec3 cap_down = capsule.furthest_point({ 0, -1, 0 });
	Vec3 cap_up = capsule.furthest_point({ 0, 1, 0 });
	assert (math::abs(cap_down.y + 1.5f) < 1e-5f); // bottom cap: -h/2 - r
	assert (math::abs(cap_up.y - 1.5f) < 1e-5f); // top cap: +h/2 + r

	Aabb3 cap_aabb = capsule.get_aabb();
	assert (math::abs(cap_aabb.min.y + 1.5f) < 1e-5f && math::abs(cap_aabb.max.y - 1.5f) < 1e-5f);
	assert (math::abs(cap_aabb.min.x + 0.5f) < 1e-5f && math::abs(cap_aabb.max.x - 0.5f) < 1e-5f);

	Cylinder cylinder = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };

	Vec3 cyl_side = cylinder.furthest_point({ 1, 0, 0 });
	assert (math::abs(cyl_side.x - 0.5f) < 1e-5f); // rim radius
	assert (math::abs(math::abs(cyl_side.y) - 1.0f) < 1e-5f); // rim sits on a cap plane

	Aabb3 cyl_aabb = cylinder.get_aabb();
	assert (math::abs(cyl_aabb.min.y + 1.0f) < 1e-5f && math::abs(cyl_aabb.max.y - 1.0f) < 1e-5f);
}

fn void test_capsule_rests_on_box()
{
	// z-up world; the capsule axis (+y) is horizontal, so it rests on its side
	// at ground_top + radius.
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 ground_shape = collision::aabb_from_half({ 10, 10, 1 });
	world.add_body(body: {
		.id = 1,
		.collider = { .shape = &ground_shape, .translation = { 0, 0, -0.5f }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	Capsule capsule_shape = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };
	world.add_body(body: {
		.id = 2,
		.mass = 1, .gravity_factor = 1,
		.collider = { .shape = &capsule_shape, .translation = { 0, 0, 1 }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	for (int i = 0; i < 240; i++) world.run_step(1.0f / 60.0f, step_count: 4);

	Rigidbody* capsule_body = world.find_body(2)!!;
	assert (math::abs(capsule_body.collider.translation.z - 0.5f) < 0.05f,
		"capsule should rest at ground + radius, got z=%f", capsule_body.collider.translation.z);
}

fn void test_box_rests_on_thin_box()
{
	// Paper-thin static slab (like glTF exporter ground planes): a dropped box
	// must neither tunnel through nor get ejected.
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 slab_shape = collision::aabb_from_half({ 15, 15, 0.001f });
	world.add_body(body: {
		.id = 1,
		.collider = { .shape = &slab_shape, .translation = { 0, 0, 0 }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	Aabb3 box_shape = collision::aabb_from_half({ 1, 1, 1 });
	world.add_body(body: {
		.id = 2,
		.mass = 1, .gravity_factor = 1,
		.collider = { .shape = &box_shape, .translation = { 0, 0, 1.2f }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	for (int i = 0; i < 300; i++) world.run_step(1.0f / 60.0f, step_count: 4);

	Rigidbody* box_body = world.find_body(2)!!;
	assert (box_body.collider.translation.z > 0.4f && box_body.collider.translation.z < 0.6f,
		"box should rest on the thin slab, got z=%f", box_body.collider.translation.z);
}

// ============================================================================
// Solid voxelization and distance fields
// ============================================================================

// A closed axis-aligned box, twelve triangles, appended to the lists given.
macro void push_box(BoxPoints* vertices, Triangles* triangles, Vec3 low, Vec3 high)
{
	uint first = (uint)vertices.len();
	vertices.push_all({
		{ low.x, low.y, low.z },   { high.x, low.y, low.z },
		{ high.x, high.y, low.z }, { low.x, high.y, low.z },
		{ low.x, low.y, high.z },  { high.x, low.y, high.z },
		{ high.x, high.y, high.z }, { low.x, high.y, high.z },
	});

	uint[36] faces = {
		0, 1, 2,  0, 2, 3,   // -z
		4, 6, 5,  4, 7, 6,   // +z
		0, 5, 1,  0, 4, 5,   // -y
		3, 2, 6,  3, 6, 7,   // +y
		0, 3, 7,  0, 7, 4,   // -x
		1, 5, 6,  1, 6, 2,   // +x
	};
	for (usz i = 0; i < 36; i += 3)
	{
		triangles.push({ first + faces[i], first + faces[i + 1], first + faces[i + 2] });
	}
}

// A box is the one shape whose inside is known in closed form, so it is the
// only place the fill can be checked against something other than itself.
fn void test_voxel_grid_fills_a_box()
{
	BoxPoints vertices;
	Triangles triangles;
	defer
	{
		vertices.free();
		triangles.free();
	}

	push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();

	assert(grid.usable());

	float filled = (float)grid.cells.len * grid.cell * grid.cell * grid.cell;
	float actual = 1.0f * 0.5f * 0.5f;
	assert(math::abs(filled - actual) / actual < 0.05f, "filled volume is not the box's");

	// The middle is inside; a point well outside the box is not, and does not
	// find a cell within the shell search either.
	assert(grid.nearest_cell({ 0, 0, 0 }) >= 0);
	assert(grid.nearest_cell({ 0, 5, 0 }) < 0);
}

// The property everything else rests on: distance is measured *through* solid
// space, so two bodies with air between them are unreachable from each other
// however short the straight line is.
fn void test_voxel_field_stops_at_a_gap()
{
	BoxPoints vertices;
	Triangles triangles;
	defer
	{
		vertices.free();
		triangles.free();
	}

	push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { -0.2, 0.25, 0.25 });
	push_box(&vertices, &triangles, { 0.2, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	assert(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	assert(grid.distance_from_segment({ -0.35f, 0, 0 }, { -0.35f, 0, 0 }, field));

	usz reached_left = 0;
	usz reached_right = 0;
	foreach (usz entry, distance : field)
	{
		if (distance >= collision::VOXEL_UNREACHED) continue;
		if (grid.center(grid.cells[entry]).x < 0.0f) { reached_left++; } else { reached_right++; }
	}

	assert(reached_left > 0, "the seeded body was not filled");
	assert(reached_right == 0, "a path crossed the gap");

	// A seed that touches nothing says so, rather than handing back a field of
	// infinities the caller cannot tell from a body that is merely far away.
	assert(!grid.distance_from_segment({ 0, 5, 0 }, { 0, 5, 0 }, field));
}

// `nearest_solved` asks a different question from `nearest_cell`: not where the
// mesh is, but where the part of it this field reached is. The two disagree
// exactly where a body comes apart — which is the case it exists for, since a
// point mapped into an unreachable piece is inside the mesh and still has
// nothing any field can tell it.
fn void test_voxel_nearest_solved_skips_the_unreached_body()
{
	BoxPoints vertices;
	Triangles triangles;
	defer
	{
		vertices.free();
		triangles.free();
	}

	push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { -0.2, 0.25, 0.25 });
	push_box(&vertices, &triangles, { 0.2, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	assert(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	// Seeded in the left body, so the right one is inside the mesh and unreached.
	assert(grid.distance_from_segment({ -0.35f, 0, 0 }, { -0.35f, 0, 0 }, field));

	Vec3 in_the_right_body = { 0.35f, 0, 0 };

	int mapped = grid.nearest_cell(in_the_right_body);
	assert(mapped >= 0, "the right body did not voxelize, so there is nothing to tell apart");
	assert(field[mapped] >= collision::VOXEL_UNREACHED, "the seed crossed the gap");

	// Deep inside the unreached body, the left one is many cells away: the
	// default shell search must come back empty rather than reach across.
	assert(grid.nearest_solved(field, in_the_right_body) < 0,
		"the default reach borrowed an answer from across the gap");

	// Widened far enough to cross, it finds the reached body — and what it finds
	// is on the reached side, which is the whole point of not using nearest_cell.
	int borrowed = grid.nearest_solved(field, in_the_right_body, 64);
	assert(borrowed >= 0, "a wide search still found nothing the field had reached");
	assert(field[borrowed] < collision::VOXEL_UNREACHED, "it borrowed from an unreached cell");
	assert(grid.center(grid.cells[borrowed]).x < 0.0f, "it borrowed from the wrong body");

	// Inside the reached body it agrees with nearest_cell, since there the two
	// questions have the same answer.
	Vec3 in_the_left_body = { -0.35f, 0, 0 };
	assert(grid.nearest_solved(field, in_the_left_body) == grid.nearest_cell(in_the_left_body),
		"the two disagreed where the field had reached");

	// A field of the wrong length is a caller error, not a shape to search.
	float[] wrong = mem::new_array(float, 3);
	defer mem::free(wrong);
	assert(grid.nearest_solved(wrong, in_the_left_body) < 0, "a mis-sized field was searched anyway");
}

// Distance grows with the path, and reading between cells does not step.
fn void test_voxel_field_measures_and_interpolates()
{
	BoxPoints vertices;
	Triangles triangles;
	defer
	{
		vertices.free();
		triangles.free();
	}

	push_box(&vertices, &triangles, { -1.0, -0.1, -0.1 }, { 1.0, 0.1, 0.1 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	assert(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	assert(grid.distance_from_segment({ -0.9f, 0, 0 }, { -0.9f, 0, 0 }, field));

	float near = grid.sample(field, { -0.5f, 0, 0 });
	float far = grid.sample(field, { 0.9f, 0, 0 });

	assert(near < far, "distance did not grow along the bar");
	// The 26-neighbour steps overestimate a straight run by a few percent, so
	// this is a bound rather than an equality.
	assert(math::abs(far - 1.8f) < 0.2f, "distance along the bar is not its length");

	// Two points inside one cell must not read the same value.
	float a = grid.sample(field, { 0.0f, 0, 0 });
	float b = grid.sample(field, { 0.0f + grid.cell * 0.4f, 0, 0 });
	assert(a != b, "the field is stepping at the grid resolution");
}

// The one-lookup half: point-in-solid, and how far a point in mid-air is
// allowed to reach for a body.
fn void test_voxel_grid_answers_inside_and_reaches()
{
	BoxPoints vertices;
	Triangles triangles;
	defer
	{
		vertices.free();
		triangles.free();
	}

	// Two bars with a wide gap between them, so "the nearest body" is a real
	// question and not the one the point is already standing in.
	push_box(&vertices, &triangles, { -1.0, -0.15, -0.15 }, { -0.4, 0.15, 0.15 });
	push_box(&vertices, &triangles, { 0.4, -0.15, -0.15 }, { 1.0, 0.15, 0.15 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	assert(grid.usable());

	assert(grid.inside({ -0.7f, 0, 0 }), "the middle of a bar is not inside");
	assert(!grid.inside({ 0, 0, 0 }), "the gap between the bars is inside");
	assert(!grid.inside({ 0, 9, 0 }), "a point nowhere near the mesh is inside");

	// Safe on a grid that was never built — the reason this tests its ranges
	// rather than leaning on the padding the fill guarantees.
	VoxelGrid nothing;
	assert(!nothing.inside({ 0, 0, 0 }));

	// A point in the gap is further from either bar than the default search
	// goes, and finds one once it is allowed to look.
	assert(grid.nearest_cell({ 0, 0, 0 }) < 0, "the default reach crossed the gap");

	int found = grid.nearest_cell({ 0, 0, 0 }, 32);
	assert(found >= 0, "a wide search still found no body");
	// It came back with the near bar's end, not with whatever it scanned first.
	assert(math::abs(grid.entry_center(found).x) > 0.3f, "the nearest cell is not on a bar");
}

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
fn void test_ik_reaches_and_keeps_its_lengths()
{
	Vec3[4] points = straight_chain();
	Quaternionf[3] turns;
	Vec3 target = { 1.0, 2.0, 0.5 };

	float gap = ik::solve_chain(points[..], turns[..], target, tolerance: 0.0005)!!;
	assert(gap <= 0.0005f, "the effector stopped %s away from a target it can reach", gap);

	Vec3[4] rest = straight_chain();
	for (usz i = 1; i < 4; i++)
	{
		float was = (rest[i] - rest[i - 1]).length();
		float now = (points[i] - points[i - 1]).length();
		assert(math::abs(now - was) < 0.0005f, "link %d stretched from %s to %s", i, was, now);
	}

	// The anchor is the one point a solve may never move: everything is measured
	// from it, and a chain that drifts has no fixed end to hang from.
	assert(points[0].length() < 0.0001f, "the anchor moved to %s", points[0]);
}

<*
 A target beyond the chain's length leaves it straight and pointing at the
 target, with the residual saying exactly how far short it fell.

 Not an error and not a fault: a person dragging past the limb's length wants to
 see it extend and stop, which is what every editor does, and the number is what
 they would want shown.
*>
fn void test_ik_out_of_reach_straightens()
{
	Vec3[4] points = straight_chain();
	Quaternionf[3] turns;
	Vec3 target = { 0, 9, 0 };

	float gap = ik::solve_chain(points[..], turns[..], target)!!;

	// Three unit links from the origin toward a target nine units away. The
	// residual converges long before the shape does — it is dominated by the
	// chain's overall direction, so the default sweep count settles it while the
	// links are still a few degrees from true.
	assert(math::abs(gap - 6.0f) < 0.01f, "residual %s, expected about 6", gap);

	// Given the budget to finish, the chain goes properly straight. This needs
	// more than the default: the whole chain has to swing a quarter turn to get
	// here, and `max_step` deliberately rations that across sweeps.
	points = straight_chain();
	ik::solve_chain(points[..], turns[..], target, passes: 64)!!;

	Vec3 aim = (target - points[0]).normalize();
	for (usz i = 1; i < 4; i++)
	{
		Vec3 link = (points[i] - points[i - 1]).normalize();
		assert(link.dot(aim) > 0.999f, "link %d is off the aim, dot %s", i, link.dot(aim));
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
fn void test_ik_turns_describe_the_points()
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
		assert((got - want).length() < 0.001f,
			"pivot %d: its turn sends the arm to %s, but the point went to %s", pivot, got, want);
	}
}

<*
 One sweep cannot turn a pivot further than the clamp allows.

 What keeps a pointer that jumped across the screen from folding a limb through
 itself in a single frame.
*>
fn void test_ik_clamps_one_step()
{
	Vec3[2] points = { { 0, 0, 0 }, { 1, 0, 0 } };
	Quaternionf[1] turns;

	// Straight back the way it came — a half turn — allowed one sweep and a
	// tenth of a radian.
	ik::solve_chain(points[..], turns[..], (Vec3){ -1, 0, 0 }, passes: 1, max_step: 0.1)!!;

	float moved = (float)math::acos(math::clamp(points[1].normalize().dot((Vec3){ 1, 0, 0 }), -1.0f, 1.0f));
	assert(moved <= 0.1f + 0.001f, "one sweep turned the pivot %s radians", moved);
	assert(moved > 0.05f, "the clamp stopped the pivot turning at all");
}

// Both ways the caller can mis-size the buffers. A fault rather than a clamp,
// because a solve that quietly ran over a chain nobody meant returns numbers
// that look like an answer.
fn void test_ik_refuses_a_chain_that_is_not_one()
{
	Vec3[1] alone = { { 0, 0, 0 } };
	Quaternionf[1] turns;
	assert(@catch(ik::solve_chain(alone[..], turns[..], (Vec3){ 1, 0, 0 })) == ik::IK_CHAIN_SHAPE);

	Vec3[4] points = straight_chain();
	Quaternionf[2] short_turns;
	assert(@catch(ik::solve_chain(points[..], short_turns[..], (Vec3){ 1, 1, 1 })) == ik::IK_CHAIN_SHAPE);
}

// The antiparallel case has no shortest turn — every half-turn about a
// perpendicular is equally short — so the only thing to assert is that one is
// picked and that it is a half turn about something perpendicular.
fn void test_ik_shortest_arc_survives_a_reversal()
{
	Quaternionf half = ik::shortest_arc({ 1, 0, 0 }, { -1, 0, 0 });
	Vec3 sent = ik::turned(half, (Vec3){ 1, 0, 0 });
	assert((sent - (Vec3){ -1, 0, 0 }).length() < 0.001f, "a reversal came out as %s", sent);

	// And the two degenerate inputs that have no direction at all. Through
	// `equals` and a named constant, because `quaternion::IDENTITY` is generic
	// and needs a type to infer from, and a quaternion has no `==` at all.
	Quaternionf still = quaternion::IDENTITY;
	assert(ik::shortest_arc({ 0, 0, 0 }, { 1, 0, 0 }).equals(still), "a zero-length input turned something");
	assert(ik::shortest_arc({ 1, 0, 0 }, { 1, 0, 0 }).equals(still), "a vector onto itself turned something");
}


// ============================================================================
// Particle chains — the shape a tail, a rope or a jiggle bone run has
// ============================================================================

// The angle between two directions, read the well-conditioned way.
macro float swing_angle(Vec3 v, Vec3 axis)
{
	Vec3 a = v.normalize();
	Vec3 b = axis.normalize();
	return dmath::atan2(a.cross(b).length(), a.dot(b));
}

// Equal to the last bit, not to a tolerance — a bake replayed tomorrow has to
// produce the same numbers, and "close enough" is what drifts.
macro bool same_bits(Vec3 a, Vec3 b)
{
	return bitcast(a.x, uint) == bitcast(b.x, uint) &&
		bitcast(a.y, uint) == bitcast(b.y, uint) &&
		bitcast(a.z, uint) == bitcast(b.z, uint);
}

// A four-particle chain hung off a pinned root with everything a host drives at
// once: a spring toward an animated point, a cone limit, per-particle damping
// and a collision radius.
macro SoftBody build_jiggle_chain(sz id)
{
	Vec3[4] points = { { 0, 0, 0 }, { 0, -0.5f, 0 }, { 0, -1.0f, 0 }, { 0, -1.5f, 0 } };
	float[4] weights = { 1, 1, 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], id);
	sb.substeps = 1;
	sb.pins_first = true;
	sb.pin_particle(0, points[0]);

	for (sz i = 0; i < 4; i++)
	{
		sb.particle_damping.push(0.2f);
		sb.particle_radius.push(0.05f);
	}

	for (sz i = 1; i < 4; i++)
	{
		sb.add_target(i, points[i], 0.0005f);
		sb.add_swing(i - 1, i, { 0, -1, 0 }, (float)(math::PI / 4.0), 0.0f);
	}

	return sb;
}

// A chain hung off a still pin is a pendulum: it has to end up straight below
// the pin, and it must not have stretched on the way there — not at the end,
// which any solver reaches eventually, but at every substep in between.
fn void test_particle_chain_settles_to_vertical()
{
	Vec3[4] points = { { 0, 0, 0 }, { 0.5f, 0, 0 }, { 1.0f, 0, 0 }, { 1.5f, 0, 0 } };
	float[4] weights = { 1, 1, 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 1);
	defer sb.free();

	// Three links, no surface: a chain is exactly the body a mesh constructor
	// cannot make.
	assert(sb.positions.len() == 4);
	assert(sb.prev_positions.len() == 4);
	assert(sb.distance_constraints.len() == 3);
	assert(sb.triangles.len() == 0);
	foreach (dc : sb.distance_constraints) assert(math::abs(dc.rest_length - 0.5f) < 1e-6f);

	sb.substeps = 1;
	sb.pin_particle(0, points[0]);

	Vec3 gravity = { 0, -9.81f, 0 };
	for (int i = 0; i < 2000; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 120.0f);

		for (sz p = 1; p < 4; p++)
		{
			float len = (sb.positions[p] - sb.positions[p - 1]).length();
			assert(math::abs(len - 0.5f) < 1e-3f, "segment %d measured %f on step %d", (int)p, len, i);
		}
	}

	assert(same_bits(sb.positions[0], (Vec3){ 0, 0, 0 }), "the pinned root drifted to %s", sb.positions[0]);
	for (sz p = 1; p < 4; p++)
	{
		Vec3 hang = sb.positions[p];
		assert(math::abs(hang.x) < 0.01f, "particle %d hangs out at x=%f", (int)p, hang.x);
		assert(math::abs(hang.z) < 0.01f, "particle %d hangs out at z=%f", (int)p, hang.z);
		assert(math::abs(hang.y + 0.5f * (float)p) < 0.01f, "particle %d hangs at y=%f", (int)p, hang.y);
	}
}

// A target is a spring to a point the host owns rather than a link to another
// particle, so with nothing else pulling the particle must arrive and stay.
fn void test_target_constraint_converges()
{
	Vec3[1] points = { { 0, 0, 0 } };
	float[1] weights = { 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 2);
	defer sb.free();
	sb.substeps = 1;

	assert(sb.distance_constraints.len() == 0); // one point is a chain of no links

	Vec3 target = { 1.0f, 2.0f, -0.5f };
	assert(sb.add_target(0, target, 0.0001f) == 0);

	for (int i = 0; i < 600; i++)
	{
		sb.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	}

	float miss = (sb.positions[0] - target).length();
	assert(miss < 1e-3f, "the particle settled %f from its target", miss);

	// Made rigid, gravity cannot pull it off; and the target is a thing the host
	// moves, so it has to follow.
	sb.target_constraints.get_ref(0).compliance = 0;
	Vec3 moved = { -2.0f, 0.5f, 3.0f };
	sb.set_target(0, moved);

	for (int i = 0; i < 120; i++)
	{
		sb.xpbd_solve({ 0, -9.81f, 0 }, 1.0f / 120.0f);
	}

	assert((sb.positions[0] - moved).length() < 1e-4f, "a rigid target left it at %s", sb.positions[0]);
}

// Pushing a point out of a capsule, including the one place where the direction
// is not implied by the geometry at all: dead on the axis.
fn void test_capsule_push_point_separates()
{
	Vec3 a = { 0, 0, 0 };
	Vec3 b = { 0, 2, 0 };

	// Inside and off-axis: straight out, to exactly the surface.
	CapsuleHit near = collision::capsule_push_point({ 0.25f, 1, 0 }, a, b, 1.0f);
	assert(near.hit);
	assert((near.normal - (Vec3){ 1, 0, 0 }).length() < 1e-5f, "normal was %s", near.normal);
	assert(math::abs(near.depth - 0.75f) < 1e-5f, "depth was %f", near.depth);
	assert((near.point - (Vec3){ 1, 1, 0 }).length() < 1e-5f, "surface point was %s", near.point);

	// A particle with a radius of its own is separated by that much more.
	CapsuleHit fat = collision::capsule_push_point({ 0.25f, 1, 0 }, a, b, 1.0f, 0.3f);
	assert(fat.hit);
	assert(math::abs(fat.depth - 1.05f) < 1e-5f, "depth was %f", fat.depth);
	assert((fat.point - (Vec3){ 1.3f, 1, 0 }).length() < 1e-5f, "surface point was %s", fat.point);

	// Outside is no hit, and neither is a capsule with no thickness to be in.
	assert(!collision::capsule_push_point({ 2, 1, 0 }, a, b, 1.0f).hit);
	assert(!collision::capsule_push_point({ 0, 1, 0 }, a, b, 0.0f).hit);

	// On the axis every direction is equally right and none is implied, so the
	// answer has to be a finite unit vector rather than a normalized zero.
	CapsuleHit axial = collision::capsule_push_point({ 0, 1, 0 }, a, b, 1.0f);
	assert(axial.hit);
	assert(math::abs(axial.normal.length() - 1.0f) < 1e-5f, "on-axis normal was %s", axial.normal);
	assert(math::abs(axial.normal.y) < 1e-5f, "on-axis normal ran along the axis: %s", axial.normal);
	assert(math::abs(axial.depth - 1.0f) < 1e-5f, "on-axis depth was %f", axial.depth);
	assert((axial.point - ((Vec3){ 0, 1, 0 } + axial.normal)).length() < 1e-5f);

	// A segment of no length is a sphere, and its centre is the same degeneracy
	// one level down — there is not even an axis left to be perpendicular to.
	CapsuleHit sphere = collision::capsule_push_point(a, a, a, 0.5f);
	assert(sphere.hit);
	assert(math::abs(sphere.normal.length() - 1.0f) < 1e-5f, "sphere-centre normal was %s", sphere.normal);
	assert(math::abs(sphere.depth - 0.5f) < 1e-5f, "sphere-centre depth was %f", sphere.depth);
}

// The chain-side wrapper: a body with no triangles gets no response at all from
// generate_rigidbody_collisions, so this is how a chain learns about a body.
fn void test_capsule_collisions_push_a_chain_out()
{
	Vec3[2] points = { { 0, 0, 0 }, { 0.2f, -0.4f, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 3);
	defer sb.free();
	sb.substeps = 1;
	sb.pin_particle(0, points[0]);
	sb.particle_radius.push(0);
	sb.particle_radius.push(0.05f);

	// A capsule hanging below the pin, thick enough to have swallowed particle 1.
	Vec3[1] seg_a = { { 0, -1, 0 } };
	Vec3[1] seg_b = { { 0, 0, 0 } };
	float[1] radii = { 0.3f };

	sb.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
	assert(sb.collision_constraints.len() == 1, "the buried particle produced %d constraints", (int)sb.collision_constraints.len());
	assert(sb.collision_constraints[0].particle == 1);
	sb.collision_constraints.clear();

	for (int i = 0; i < 400; i++)
	{
		sb.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
		sb.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	}

	Vec3 on_axis = collision::closest_point_on_segment(sb.positions[1], seg_a[0], seg_b[0]);
	float clearance = (sb.positions[1] - on_axis).length();
	assert(clearance > 0.35f - 1e-3f, "particle 1 settled %f from the axis, inside 0.35", clearance);

	// And the link it hangs from was not stretched to get it out.
	float len = (sb.positions[1] - sb.positions[0]).length();
	assert(math::abs(len - 0.4472136f) < 1e-3f, "the link measured %f", len);
}

// A cone limit is one-sided — inside it nothing happens — and it is a rotation,
// so however hard the pull, the link it clamps does not stretch to obey.
fn void test_swing_constraint_clamps_to_the_cone()
{
	Vec3[2] points = { { 0, 0, 0 }, { 0, -1, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], 4);
	defer sb.free();
	sb.substeps = 1;
	sb.pin_particle(0, points[0]);

	float max_angle = (float)(math::PI / 6.0); // 30 degrees
	assert(sb.add_swing(0, 1, { 0, -1, 0 }, max_angle) == 0);

	// Gravity sideways: the chain wants to hang a quarter turn off the cone axis.
	Vec3 gravity = { 9.81f, 0, 0 };
	for (int i = 0; i < 1200; i++)
	{
		sb.xpbd_solve(gravity, 1.0f / 120.0f);

		float open = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, -1, 0 });
		assert(open <= max_angle + 1e-3f, "the cone opened to %f rad on step %d", open, i);

		float len = (sb.positions[1] - sb.positions[0]).length();
		assert(math::abs(len - 1.0f) < 1e-3f, "the link measured %f on step %d", len, i);
	}

	// Pressed against the wall of the cone, not idling somewhere inside it.
	float settled = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, -1, 0 });
	assert(math::abs(settled - max_angle) < 1e-2f, "it settled at %f rad, not against the wall", settled);

	// Re-aiming the cone is the per-substep call, and it carries the child with it.
	sb.set_swing_reference(0, { 0, 1, 0 });
	for (int i = 0; i < 1200; i++) sb.xpbd_solve(gravity, 1.0f / 120.0f);

	float flipped = swing_angle(sb.positions[1] - sb.positions[0], (Vec3){ 0, 1, 0 });
	assert(flipped <= max_angle + 1e-2f, "the re-aimed cone left the child at %f rad", flipped);
}

// Pins land after the constraint sweep, so a pin the host moved between substeps
// is a substep late and the chain is solved against where it used to be. That is
// what pins_first is for; the default has to keep lagging.
fn void test_pins_first_removes_the_substep_lag()
{
	Vec3[2] points = { { 0, 0, 0 }, { 0, -1, 0 } };
	float[2] weights = { 1, 1 };

	SoftBody lagging = collision::create_particle_chain(mem, points[..], weights[..], 5);
	SoftBody leading = collision::create_particle_chain(mem, points[..], weights[..], 6);
	defer lagging.free();
	defer leading.free();

	lagging.substeps = 1;
	leading.substeps = 1;
	leading.pins_first = true;
	assert(!lagging.pins_first); // off unless asked for

	lagging.pin_particle(0, points[0]);
	leading.pin_particle(0, points[0]);

	Vec3 moved = { 0.4f, 0, 0 };
	lagging.pins.get_ref(0).local_offset = moved;
	leading.pins.get_ref(0).local_offset = moved;

	lagging.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	leading.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);

	assert(same_bits(lagging.positions[0], moved) && same_bits(leading.positions[0], moved));

	float lead_len = (leading.positions[1] - leading.positions[0]).length();
	assert(math::abs(lead_len - 1.0f) < 1e-3f, "pins_first still left the link at %f", lead_len);

	float lag_len = (lagging.positions[1] - lagging.positions[0]).length();
	assert(math::abs(lag_len - 1.0f) > 0.05f, "the default stopped lagging: the link was %f", lag_len);
}

// Per-particle damping is a fraction per second, so the same second of
// simulation costs the same speed however it is cut into substeps. The
// whole-body `damping` is a fraction per substep and must stay one.
fn void test_particle_damping_is_substep_invariant()
{
	Vec3[1] points = { { 0, 0, 0 } };
	float[1] weights = { 1 };

	SoftBody coarse = collision::create_particle_chain(mem, points[..], weights[..], 8);
	SoftBody fine = collision::create_particle_chain(mem, points[..], weights[..], 9);
	defer coarse.free();
	defer fine.free();

	coarse.substeps = 1;
	fine.substeps = 1;
	coarse.particle_damping.push(0.5f);
	fine.particle_damping.push(0.5f);
	coarse.velocities[0] = (Vec3){ 1, 0, 0 };
	fine.velocities[0] = (Vec3){ 1, 0, 0 };

	coarse.xpbd_solve({ 0, 0, 0 }, 1.0f);
	for (int i = 0; i < 120; i++) fine.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);

	assert(math::abs(coarse.velocities[0].x - 0.5f) < 1e-4f, "one whole-second step kept %f", coarse.velocities[0].x);
	assert(math::abs(fine.velocities[0].x - 0.5f) < 1e-3f, "120 small steps kept %f", fine.velocities[0].x);

	// An empty list is the old path, unchanged: one raw multiply per substep.
	SoftBody legacy = collision::create_particle_chain(mem, points[..], weights[..], 10);
	defer legacy.free();
	legacy.substeps = 1;
	legacy.velocities[0] = (Vec3){ 1, 0, 0 };
	legacy.xpbd_solve({ 0, 0, 0 }, 1.0f / 120.0f);
	assert(legacy.velocities[0].x == 0.99f, "the whole-body damping path changed: %f", legacy.velocities[0].x);
}

// Two chains fed identical inputs must stay bit-equal for as long as they are
// driven — a bake is only reproducible if this holds every substep, not on
// average. Pins, targets, cones and capsules are all in play at once.
fn void test_chain_determinism_lockstep()
{
	SoftBody a = build_jiggle_chain(11);
	SoftBody b = build_jiggle_chain(12);
	defer a.free();
	defer b.free();

	Vec3 gravity = { 0, -9.81f, 0 };
	Vec3[1] seg_a = { { 0.35f, -0.9f, 0 } };
	Vec3[1] seg_b = { { 0.35f, -0.2f, 0 } };
	float[1] radii = { 0.25f };

	for (int step = 0; step < 500; step++)
	{
		// Everything a host re-supplies each substep: a moving pin, moving
		// targets, a re-aimed cone, posed collider segments.
		float t = (float)step / 120.0f;
		Vec3 sway = { dmath::sin(t * 6.0f) * 0.4f, 0, dmath::cos(t * 4.0f) * 0.2f };

		a.pins.get_ref(0).local_offset = sway;
		b.pins.get_ref(0).local_offset = sway;

		for (sz i = 1; i < 4; i++)
		{
			Vec3 target = sway + (Vec3){ 0, -0.5f * (float)i, 0 };
			a.set_target(i, target);
			b.set_target(i, target);
			a.set_swing_reference(i - 1, { sway.x * 0.5f, -1, 0 });
			b.set_swing_reference(i - 1, { sway.x * 0.5f, -1, 0 });
		}

		a.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
		b.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);

		a.xpbd_solve(gravity, 1.0f / 120.0f);
		b.xpbd_solve(gravity, 1.0f / 120.0f);

		for (sz i = 0; i < a.positions.len(); i++)
		{
			assert(same_bits(a.positions[i], b.positions[i]), "particle %d diverged on step %d: %s vs %s", (int)i, step, a.positions[i], b.positions[i]);
			assert(same_bits(a.velocities[i], b.velocities[i]), "particle %d's velocity diverged on step %d", (int)i, step);
		}
	}

	// And it actually moved, so the equality above is not two chains sitting still.
	assert((a.positions[3] - (Vec3){ 0, -1.5f, 0 }).length() > 0.05f, "the chain never left its rest pose");
}


module pool_test_util;

fn int slot_worker(void* arg)
{
	*(int*)arg += 1;
	return 0;
}

<*
 The broadphase hash: that a move is a move rather than a copy, that distinct
 cells stay distinct, and that a query at a cell size other than 1 looks in the
 right place.

	`c3c test --test-filter collision_tests::spatial`

 Two of these are regressions with the bug written into the comment, and both
 were bugs about *keys* — cells that collided in the table, and coordinates
 divided twice. Neither shows up as a wrong answer often enough to notice; both
 show up here immediately.

 Every test frees its map, which the leak-tracked run is what enforces: a
 `SpatialHash3D` that grows a bucket per cell and never gives one back is the
 leak this library is most likely to have.
*>
module collision_tests::spatial;

import collision;
import std::core::test;

fn void test_spatial_hash_update() @test
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
		test::@check(pair.a == 0 && pair.b == 1);
	};
	test::@check(pair_count == 1);

	// Move box1 to a non-overlapping position
	Aabb3 box1_new = { { 5, 5, 5 }, { 5.9, 5.9, 5.9 } };
	spatial_map.update(box1_old, box1_new, 0);

	// Verify no pairs after move
	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	test::@check(pair_count == 0);
}

fn void test_spatial_hash_update_same_cell() @test
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
	test::@check(count >= 1);
}

fn void test_spatial_hash_update_cross_cells() @test
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
	test::@check(pair_count == 0);

	// Move box to span cells (1,0,0) and (2,0,0) - now overlaps with stationary box's cell
	Aabb3 box_new = { { 1.5, 0.5, 0.5 }, { 2.5, 0.9, 0.9 } };
	spatial_map.update(box_old, box_new, 0);

	// Should now have a pair
	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		pair_count++;
	};
	test::@check(pair_count == 1);
}

fn void test_spatial_hash_multiple_updates() @test
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
	test::@check(count >= 1);

	// Verify it's NOT at the old position
	count = 0;
	spatial_map.@get_nearby_objects({ 0.5, 0.5, 0.5 } ; sz id)
	{
		if (id == 0) count++;
	};
	test::@check(count == 0);
}

fn void test_spatial_hash_distinct_cells_no_false_pairs() @test
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
	test::@check(pair_count == 0);
}

fn void test_spatial_hash_update_wide_body() @test
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
		test::@check(pair.a == 0 && pair.b == 1);
		pair_count++;
	};
	test::@check(pair_count == 1);

	// Shrink the wide body's y range so cells at y 1 are vacated;
	// cell (0,0,23) stays occupied and the pair must survive.
	Aabb3 wide_new = { { 0.1, 0.1, 0.1 }, { 0.9, 0.9, 23.9 } };
	spatial_map.update(wide_old, wide_new, 0);

	pair_count = 0;
	spatial_map.@get_pairs(; Pair pair)
	{
		test::@check(pair.a == 0 && pair.b == 1);
		pair_count++;
	};
	test::@check(pair_count == 1);
}

fn void test_spatial_hash_nearby_objects_cell_size() @test
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
	test::@check(count == 1);
}

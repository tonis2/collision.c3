<*
 Solid voxelization and the distance fields read off it.

	`c3c test --test-filter collision_tests::voxel`

 The property everything here rests on is that distance is measured *through*
 solid space: two bodies with air between them are unreachable from each other
 however short the straight line is. A field that leaked across a gap would look
 entirely reasonable in every other respect, which is why the gap scene appears
 in three of these tests rather than one.

 Every test holds four allocations at once — a vertex list, a triangle list, the
 grid, and a `float[]` field sized from the grid — and the field is a raw
 `mem::new_array`, so it is the one with no destructor to forget on your behalf.
*>
module collision_tests::voxel;

import collision;
import collision_testutil;
import std::core::test;
import std::math;
import std::collections::list;

alias Vec3 = float[<3>];
alias BoxPoints = List {Vec3};
alias Triangles = List {Triangle};

// A box is the one shape whose inside is known in closed form, so it is the
// only place the fill can be checked against something other than itself.
fn void test_voxel_grid_fills_a_box() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 8);
	triangles.init(mem, 12);
	defer
	{
		vertices.free();
		triangles.free();
	}

	collision_testutil::push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();

	test::@check(grid.usable());

	float filled = (float)grid.cells.len * grid.cell * grid.cell * grid.cell;
	float actual = 1.0f * 0.5f * 0.5f;
	test::@check(math::abs(filled - actual) / actual < 0.05f, "filled volume is not the box's");

	// The middle is inside; a point well outside the box is not, and does not
	// find a cell within the shell search either.
	test::@check(grid.nearest_cell({ 0, 0, 0 }) >= 0);
	test::@check(grid.nearest_cell({ 0, 5, 0 }) < 0);
}

// The property everything else rests on: distance is measured *through* solid
// space, so two bodies with air between them are unreachable from each other
// however short the straight line is.
fn void test_voxel_field_stops_at_a_gap() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 8);
	triangles.init(mem, 12);
	defer
	{
		vertices.free();
		triangles.free();
	}

	collision_testutil::push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { -0.2, 0.25, 0.25 });
	collision_testutil::push_box(&vertices, &triangles, { 0.2, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	test::@check(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	test::@check(grid.distance_from_segment({ -0.35f, 0, 0 }, { -0.35f, 0, 0 }, field));

	usz reached_left = 0;
	usz reached_right = 0;
	foreach (usz entry, distance : field)
	{
		if (distance >= collision::VOXEL_UNREACHED) continue;
		if (grid.center(grid.cells[entry]).x < 0.0f) { reached_left++; } else { reached_right++; }
	}

	test::@check(reached_left > 0, "the seeded body was not filled");
	test::@check(reached_right == 0, "a path crossed the gap");

	// A seed that touches nothing says so, rather than handing back a field of
	// infinities the caller cannot tell from a body that is merely far away.
	test::@check(!grid.distance_from_segment({ 0, 5, 0 }, { 0, 5, 0 }, field));
}

// `nearest_solved` asks a different question from `nearest_cell`: not where the
// mesh is, but where the part of it this field reached is. The two disagree
// exactly where a body comes apart — which is the case it exists for, since a
// point mapped into an unreachable piece is inside the mesh and still has
// nothing any field can tell it.
fn void test_voxel_nearest_solved_skips_the_unreached_body() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 8);
	triangles.init(mem, 12);
	defer
	{
		vertices.free();
		triangles.free();
	}

	collision_testutil::push_box(&vertices, &triangles, { -0.5, -0.25, -0.25 }, { -0.2, 0.25, 0.25 });
	collision_testutil::push_box(&vertices, &triangles, { 0.2, -0.25, -0.25 }, { 0.5, 0.25, 0.25 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	test::@check(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	// Seeded in the left body, so the right one is inside the mesh and unreached.
	test::@check(grid.distance_from_segment({ -0.35f, 0, 0 }, { -0.35f, 0, 0 }, field));

	Vec3 in_the_right_body = { 0.35f, 0, 0 };

	int mapped = grid.nearest_cell(in_the_right_body);
	test::@check(mapped >= 0, "the right body did not voxelize, so there is nothing to tell apart");
	test::@check(field[mapped] >= collision::VOXEL_UNREACHED, "the seed crossed the gap");

	// Deep inside the unreached body, the left one is many cells away: the
	// default shell search must come back empty rather than reach across.
	test::@check(grid.nearest_solved(field, in_the_right_body) < 0,
		"the default reach borrowed an answer from across the gap");

	// Widened far enough to cross, it finds the reached body — and what it finds
	// is on the reached side, which is the whole point of not using nearest_cell.
	int borrowed = grid.nearest_solved(field, in_the_right_body, 64);
	test::@check(borrowed >= 0, "a wide search still found nothing the field had reached");
	test::@check(field[borrowed] < collision::VOXEL_UNREACHED, "it borrowed from an unreached cell");
	test::@check(grid.center(grid.cells[borrowed]).x < 0.0f, "it borrowed from the wrong body");

	// Inside the reached body it agrees with nearest_cell, since there the two
	// questions have the same answer.
	Vec3 in_the_left_body = { -0.35f, 0, 0 };
	test::@check(grid.nearest_solved(field, in_the_left_body) == grid.nearest_cell(in_the_left_body),
		"the two disagreed where the field had reached");

	// A field of the wrong length is a caller error, not a shape to search.
	float[] wrong = mem::new_array(float, 3);
	defer mem::free(wrong);
	test::@check(grid.nearest_solved(wrong, in_the_left_body) < 0, "a mis-sized field was searched anyway");
}

// Distance grows with the path, and reading between cells does not step.
fn void test_voxel_field_measures_and_interpolates() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 8);
	triangles.init(mem, 12);
	defer
	{
		vertices.free();
		triangles.free();
	}

	collision_testutil::push_box(&vertices, &triangles, { -1.0, -0.1, -0.1 }, { 1.0, 0.1, 0.1 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	test::@check(grid.usable());

	float[] field = mem::new_array(float, (sz)grid.cells.len);
	defer mem::free(field);

	test::@check(grid.distance_from_segment({ -0.9f, 0, 0 }, { -0.9f, 0, 0 }, field));

	float near = grid.sample(field, { -0.5f, 0, 0 });
	float far = grid.sample(field, { 0.9f, 0, 0 });

	test::@check(near < far, "distance did not grow along the bar");
	// The 26-neighbour steps overestimate a straight run by a few percent, so
	// this is a bound rather than an equality.
	test::@check(math::abs(far - 1.8f) < 0.2f, "distance along the bar is not its length");

	// Two points inside one cell must not read the same value.
	float a = grid.sample(field, { 0.0f, 0, 0 });
	float b = grid.sample(field, { 0.0f + grid.cell * 0.4f, 0, 0 });
	test::@check(a != b, "the field is stepping at the grid resolution");
}

// The one-lookup half: point-in-solid, and how far a point in mid-air is
// allowed to reach for a body.
fn void test_voxel_grid_answers_inside_and_reaches() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 8);
	triangles.init(mem, 12);
	defer
	{
		vertices.free();
		triangles.free();
	}

	// Two bars with a wide gap between them, so "the nearest body" is a real
	// question and not the one the point is already standing in.
	collision_testutil::push_box(&vertices, &triangles, { -1.0, -0.15, -0.15 }, { -0.4, 0.15, 0.15 });
	collision_testutil::push_box(&vertices, &triangles, { 0.4, -0.15, -0.15 }, { 1.0, 0.15, 0.15 });

	VoxelGrid grid = collision::create_voxel_grid(vertices.array_view(), triangles.array_view());
	defer grid.free();
	test::@check(grid.usable());

	test::@check(grid.inside({ -0.7f, 0, 0 }), "the middle of a bar is not inside");
	test::@check(!grid.inside({ 0, 0, 0 }), "the gap between the bars is inside");
	test::@check(!grid.inside({ 0, 9, 0 }), "a point nowhere near the mesh is inside");

	// Safe on a grid that was never built — the reason this tests its ranges
	// rather than leaning on the padding the fill guarantees.
	VoxelGrid nothing;
	test::@check(!nothing.inside({ 0, 0, 0 }));

	// A point in the gap is further from either bar than the default search
	// goes, and finds one once it is allowed to look.
	test::@check(grid.nearest_cell({ 0, 0, 0 }) < 0, "the default reach crossed the gap");

	int found = grid.nearest_cell({ 0, 0, 0 }, 32);
	test::@check(found >= 0, "a wide search still found no body");
	// It came back with the near bar's end, not with whatever it scanned first.
	test::@check(math::abs(grid.entry_center(found).x) > 0.3f, "the nearest cell is not on a bar");
}

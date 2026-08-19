<*
 The shapes themselves: what a box does under a transform, what a ray hits, what
 the convex test says about two of them, and the conventions a capsule and a
 cylinder are built to.

	`c3c test --test-filter collision_tests::shapes`

 Leak tracking is on unless you ask for it off. The runner swaps the thread
 allocator for a tracking one around each `@test` fn and reports whatever the
 test did not give back, so a `Mesh` built here and not freed is a failure with
 a report rather than a slow afternoon with valgrind. `--test-noleak` turns it
 off, which is worth doing while iterating and not worth committing.
*>
module collision_tests::shapes;

import collision;
import std::core::test;
import std::math;
import std::collections::list;

alias Vec3 = float[<3>];
alias BoxPoints = List {Vec3};
alias Triangles = List {Triangle};

fn void test_physics() @test
{
	Aabb3 aabb = { { -1.0, -1.0, -1.0 }, { 1.0, 1.0, 1.0 } };

	aabb = aabb.transform(translation: { 1, 5, 1 }, scale: { 3.0, 1.0, 1.0 }, rotation: { 0, 0, 0, 1 });

	test::@check(aabb.min == (Vec3){ -2, 4, 0 }, "the transformed box starts at %s", aabb.min);
	test::@check(aabb.max == (Vec3){ 4, 6, 2 }, "the transformed box ends at %s", aabb.max);

	SpatialHash3D spatial_map = { .cell_size = 1.0 };
	defer spatial_map.free();

	Aabb3[] boxes = { { { 0, 0, 0 }, { 1, 1, 1 } }, { { 1, 1, 1 }, { 2.5, 2.5, 2.5 } }, { { 5, 5, 5 }, { 6, 6, 6 } }, };

	foreach (sz i, item : boxes) spatial_map.insert(item, i);

	spatial_map.@get_pairs(; Pair pair)
	{
		test::@check(pair.a == 0 && pair.b == 1);
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
	shape_1.init(mem, 12, 1);
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

	test::@check(!collision::check_convex_collision(&transformed_shape_1, &transformed_shape_2).collided);
	test::@check(collision::check_convex_collision(&transformed_shape_5, &transformed_shape_6).collided);
	test::@check(collision::check_convex_collision(&transformed_shape_3, &transformed_shape_4).collided);

	// CollisionInfo epa_result = epa(collision_result.simplex, &shape_1, &shape_2);

	// {
	// Aabb3 aabb = {
	// {3.486510, 1693.368896, 2.556290},
	// {3.537758, 1698.706421, 2.556290}
	// };
	// }
}

fn void test_ray() @test
{
	Ray testRay = { { 0.0, 0.0, 0.0 }, (Vec3) { 0.0, 0.0, 1 }.normalize() };

	Sphere testSphere = { { 0, 0, 5 }, 1.0 };
	test::@check(testSphere.intersect(testRay) > 0);
}

fn void test_aabb() @test
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

fn void test_remesh() @test
{
	BoxPoints vertices;
	Triangles triangles;
	vertices.init(mem, 6);
	triangles.init(mem, 2);
	defer
	{
		vertices.free();
		triangles.free();
	}

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

	test::@check(vertices.len() == 4, "the weld left %d of 6 vertices, expected 4", vertices.len());
	test::@check(triangles.len() == 2, "the weld left %d triangles, expected 2", triangles.len());
}

fn void test_capsule_cylinder_shape_conventions() @test
{
	// Both shapes are centered at the origin with their axis along +y;
	// capsule height covers the cylindrical section only (KHR_implicit_shapes).
	Capsule capsule = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };

	Vec3 cap_down = capsule.furthest_point({ 0, -1, 0 });
	Vec3 cap_up = capsule.furthest_point({ 0, 1, 0 });
	test::@check(math::abs(cap_down.y + 1.5f) < 1e-5f); // bottom cap: -h/2 - r
	test::@check(math::abs(cap_up.y - 1.5f) < 1e-5f); // top cap: +h/2 + r

	Aabb3 cap_aabb = capsule.get_aabb();
	test::@check(math::abs(cap_aabb.min.y + 1.5f) < 1e-5f && math::abs(cap_aabb.max.y - 1.5f) < 1e-5f);
	test::@check(math::abs(cap_aabb.min.x + 0.5f) < 1e-5f && math::abs(cap_aabb.max.x - 0.5f) < 1e-5f);

	Cylinder cylinder = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };

	Vec3 cyl_side = cylinder.furthest_point({ 1, 0, 0 });
	test::@check(math::abs(cyl_side.x - 0.5f) < 1e-5f); // rim radius
	test::@check(math::abs(math::abs(cyl_side.y) - 1.0f) < 1e-5f); // rim sits on a cap plane

	Aabb3 cyl_aabb = cylinder.get_aabb();
	test::@check(math::abs(cyl_aabb.min.y + 1.0f) < 1e-5f && math::abs(cyl_aabb.max.y - 1.0f) < 1e-5f);
}

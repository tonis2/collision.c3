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

// ---------------------------------------------------------------------------
// `Aabb3.transform` against the algorithm it replaced.
// ---------------------------------------------------------------------------

// The eight corners, the long way round — what `Aabb3.transform` used to do.
fn Aabb3 by_corners(Aabb3 box, Vec3 translation, Vec3 scale, Quaternionf rotation) @local
{
	Vec3[8] corners = {
		box.min,
		{ box.max.x, box.min.y, box.min.z },
		{ box.min.x, box.max.y, box.min.z },
		{ box.min.x, box.min.y, box.max.z },
		{ box.max.x, box.max.y, box.min.z },
		{ box.max.x, box.min.y, box.max.z },
		{ box.min.x, box.max.y, box.max.z },
		box.max
	};

	Aabb3 out = collision::NEW_AABB3;
	foreach (corner : corners) out.expand_by_point(rotation * (corner * scale) + translation);
	return out;
}

// And the same the other way round, for `inverse_transform`.
fn Aabb3 by_corners_inverse(Aabb3 box, Vec3 translation, Vec3 scale, Quaternionf rotation) @local
{
	Quaternionf inv_rot = rotation.conjugate();
	Vec3 inv_scale = { 1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z };

	Vec3[8] corners = {
		box.min,
		{ box.max.x, box.min.y, box.min.z },
		{ box.min.x, box.max.y, box.min.z },
		{ box.min.x, box.min.y, box.max.z },
		{ box.max.x, box.max.y, box.min.z },
		{ box.max.x, box.min.y, box.max.z },
		{ box.min.x, box.max.y, box.max.z },
		box.max
	};

	Aabb3 out = collision::NEW_AABB3;
	foreach (corner : corners) out.expand_by_point((inv_rot * (corner - translation)) * inv_scale);
	return out;
}

// A fixed sequence, so a failure is reproducible rather than a bad afternoon.
uint transform_seed @local = 0x9e3779b9;

fn float next_float(float lo, float hi) @local
{
	transform_seed = transform_seed * 1664525 + 1013904223;
	return lo + (hi - lo) * ((float)(transform_seed >> 8) / (float)(1u << 24));
}

<*
 The closed form and the eight corners agree, everywhere.

 `Aabb3.transform` used to rotate all eight corners of the box and take their
 minimum and maximum; it now transforms the centre and the extent, which is the
 same answer in a fraction of the arithmetic. "The same answer" is the claim
 worth pinning, so this keeps the old algorithm alongside as the reference and
 runs both over a spread of transforms that includes the awkward ones:
 non-uniform scale, *negative* scale, and quaternions that are not unit length —
 the last because `q * v * q.conjugate()` scales its result by `|q|²` and a
 rotation matrix built by normalising first would quietly disagree.

 The tolerance is relative to the size of the box produced. These are floats and
 the two orders of operations are genuinely different, so bit equality is not on
 offer; a box within a millionth of its own extent of the reference is the same
 box.
*>
fn void test_transform_matches_eight_corners() @test
{
	transform_seed = 0x9e3779b9;

	uint checked = 0;
	for (uint i = 0; i < 512; i++)
	{
		Vec3 lo = { next_float(-3, 0), next_float(-3, 0), next_float(-3, 0) };
		Vec3 span = { next_float(0.1f, 4), next_float(0.1f, 4), next_float(0.1f, 4) };
		Aabb3 box = { .min = lo, .max = lo + span };

		Vec3 translation = { next_float(-5, 5), next_float(-5, 5), next_float(-5, 5) };
		Vec3 scale = { next_float(0.2f, 3), next_float(0.2f, 3), next_float(0.2f, 3) };
		// A third of them mirrored on one axis: taking `abs` of the product of
		// row and scale is what keeps a negative scale from inverting the box.
		if (i % 3 == 0) scale.x = -scale.x;

		Quaternionf rotation = {
			next_float(-1, 1), next_float(-1, 1), next_float(-1, 1), next_float(-1, 1)
		};
		// Two thirds unit length, one third not, on purpose.
		if (i % 3 != 0) rotation = rotation.normalize();

		Aabb3 want = by_corners(box, translation, scale, rotation);
		Aabb3 got = box.transform(translation, scale, rotation);
		float slack = 1e-6f * (1.0f + want.size().length());

		test::@check((got.min - want.min).length() < slack, "forward min %s, wanted %s", got.min, want.min);
		test::@check((got.max - want.max).length() < slack, "forward max %s, wanted %s", got.max, want.max);

		Aabb3 want_back = by_corners_inverse(want, translation, scale, rotation);
		Aabb3 got_back = want.inverse_transform(translation, scale, rotation);
		float slack_back = 1e-6f * (1.0f + want_back.size().length());

		test::@check((got_back.min - want_back.min).length() < slack_back, "inverse min %s, wanted %s", got_back.min, want_back.min);
		test::@check((got_back.max - want_back.max).length() < slack_back, "inverse max %s, wanted %s", got_back.max, want_back.max);

		checked++;
	}

	test::@check(checked == 512);
}

<*
 `skip_rotation` skips the rotation.

 It was a parameter the old body accepted and never read, so passing it did
 nothing at all; it now means what it says. Nothing in this library passed it,
 which is why the change is a fix rather than a break.
*>
fn void test_transform_can_skip_the_rotation() @test
{
	Aabb3 box = { { -1, -2, -3 }, { 1, 2, 3 } };
	// A quarter turn about y: (0, sin 45°, 0, cos 45°).
	Quaternionf quarter_turn = { 0, 0.70710678f, 0, 0.70710678f };

	Aabb3 turned = box.transform(scale: { 2, 1, 1 }, rotation: quarter_turn);
	// The turn swaps the x and z half-extents, and x was scaled before it.
	test::@check(math::abs(turned.max.x - 3.0f) < 1e-5f, "turned x %s", turned.max.x);
	test::@check(math::abs(turned.max.z - 2.0f) < 1e-5f, "turned z %s", turned.max.z);

	Aabb3 straight = box.transform(scale: { 2, 1, 1 }, rotation: quarter_turn, skip_rotation: true);
	test::@check(math::abs(straight.max.x - 2.0f) < 1e-5f, "straight x %s", straight.max.x);
	test::@check(math::abs(straight.max.z - 3.0f) < 1e-5f, "straight z %s", straight.max.z);

	// The translation still lands, which is the half `skip_rotation` keeps.
	Aabb3 moved = box.transform(translation: { 10, 0, 0 }, skip_rotation: true);
	test::@check(math::abs(moved.min.x - 9.0f) < 1e-5f && math::abs(moved.max.x - 11.0f) < 1e-5f);
}

<*
 The triangle BVH: that a box query finds what overlaps it and refuses what does
 not, and that a ray meets a triangle it points at.

	`c3c test --test-filter collision_tests::bvh`

 `create_indexed_tri_bvh` hands back a pointer the caller owns, and freeing it
 takes two calls — `free_indexed()` for the nodes and `mem::free()` for the root
 — which is exactly the shape of ownership that goes wrong silently. Under the
 leak-tracked run it does not: forget either and the test fails with a report.
*>
module collision_tests::bvh;

import collision;
import std::core::test;

alias Vec3 = float[<3>];

fn void test_bvh() @test
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

	test::@check(bvh2.box_intersects(box2).collided);
	test::@check(bvh2.box_intersects(box3).collided == false);

	test::@check(triangle.intersect_ray(test_ray).collided);
	// assert(bvh.ray_intersects(test_ray).collided);

	HitInfo result_2 = bvh.box_intersects(box);
	test::@check(result_2.collided == true);

	Mesh tri_1;
	tri_1.init(mem, 3, 1);
	tri_1.vertices.push_all({ { 0.0f, 0.0f, 0.0f }, { 0.5f, 0.0f, 0.0f }, { 0.0f, 0.5f, 0.0f } });
	Mesh tri_2;
	tri_2.init(mem, 3, 1);
	tri_2.vertices.push_all({ { 1.1f, -2.0f, 0.0f }, { 1.1f, 2.0f, 0.0f }, { 1.1f, 0.0f, 2.0f } });
	Mesh tri_3;
	tri_3.init(mem, 3, 1);
	tri_3.vertices.push_all({ { 15.5f, 15.5f, 15.5f }, { -12.0f, 10.0f, 10.0f }, { 12.0f, 10.0f, 10.0f } });

	defer
	{
		tri_1.free();
		tri_2.free();
		tri_3.free();
	}

	test::@check(collision::check_convex_collision(&box, &tri_1).collided);
	test::@check(!collision::check_convex_collision(&box, &tri_2).collided);
	test::@check(!collision::check_convex_collision(&box, &tri_3).collided);
	// assert(collision::check_convex_collision(&box, &&result_2.triangle.get_aabb()).collided);
}

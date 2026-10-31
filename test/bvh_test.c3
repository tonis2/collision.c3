<*
 The triangle BVH: that a box query finds what overlaps it and refuses what does
 not, and that a ray meets a triangle it points at.

	`c3c test --test-filter collision_tests::bvh`

 `create_tri_bvh` hands back a tree the caller owns; one `free()` releases the
 nodes, the triangle order and the tree. Under the leak-tracked run a missed
 free fails the test with a report.
*>
module collision_tests::bvh;

import collision;
import std::collections::list;
import std::core::test;
import std::math;

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

	TriBVH* bvh = collision::create_tri_bvh(terrain.ptr, terrain_tris.ptr, terrain_tris.len);
	TriBVH* bvh2 = collision::create_tri_bvh(terrain2.ptr, terrain2_tris.ptr, terrain2_tris.len);

	defer
	{
		bvh.free();
		bvh2.free();
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

// A grid of unit quads, enough triangles for several levels of tree.
fn void test_bvh_ray_closest_hit_and_box_query() @test
{
	List {Vec3} vlist;
	List {Triangle} tlist;
	vlist.init(mem);
	tlist.init(mem);
	defer
	{
		vlist.free();
		tlist.free();
	}
	// Three stacked 8x8 sheets (128 triangles each) at z = 0, 1, 2.
	for (int layer = 0; layer < 3; layer++)
	{
		for (int gx = 0; gx < 8; gx++)
		{
			for (int gy = 0; gy < 8; gy++)
			{
				uint base = (uint)vlist.len();
				float x = (float)gx;
				float y = (float)gy;
				float z = (float)layer;
				vlist.push_all({ { x, y, z }, { x + 1, y, z }, { x + 1, y + 1, z }, { x, y + 1, z } });
				tlist.push({ base, base + 1, base + 2 });
				tlist.push({ base, base + 2, base + 3 });
			}
		}
	}

	TriBVH* bvh = collision::create_tri_bvh(vlist.entries, tlist.entries, tlist.len());
	defer bvh.free();

	// From above, the closest sheet is the top one, z = 2: t = 10 - 2.
	HitInfo hit = bvh.ray_intersects({ .origin = { 3.3, 4.4, 10 }, .direction = { 0, 0, -1 } });
	test::@check(hit.collided);
	test::@check(math::abs(hit.depth - 8.0f) < 1e-4f, "closest sheet at t=8, got %f", hit.depth);
	test::@check(hit.normal.z > 0.99f || hit.normal.z < -0.99f);

	// From below, the bottom sheet.
	hit = bvh.ray_intersects({ .origin = { 3.3, 4.4, -5 }, .direction = { 0, 0, 1 } });
	test::@check(hit.collided && math::abs(hit.depth - 5.0f) < 1e-4f, "bottom sheet at t=5, got %f", hit.depth);

	// Sideways through the stack, between the sheets: nothing to hit.
	hit = bvh.ray_intersects({ .origin = { -3, 4.4, 0.5 }, .direction = { 1, 0, 0 } });
	test::@check(!hit.collided);

	// Missing the whole tree.
	hit = bvh.ray_intersects({ .origin = { 30, 30, 10 }, .direction = { 0, 0, -1 } });
	test::@check(!hit.collided);

	// Box query: the overlap count matches brute force, via @foreach_triangle.
	Aabb3 query = { { 2.2, 2.2, 0.9 }, { 3.7, 3.7, 1.1 } };
	sz via_walk = 0;
	bvh.@foreach_triangle(query; TriangleVerts tri) { via_walk++; };
	sz brute = 0;
	foreach (t : tlist)
	{
		TriangleVerts tv = { vlist[t.x], vlist[t.y], vlist[t.z] };
		if (tv.overlaps_aabb(query)) brute++;
	}
	test::@check(via_walk == brute && brute > 0, "walk %d vs brute %d", via_walk, brute);
	test::@check(bvh.box_intersects(query).collided);
	test::@check(!bvh.box_intersects({ { 2.2, 2.2, 5 }, { 3.7, 3.7, 6 } }).collided);
}

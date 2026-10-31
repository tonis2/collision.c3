<*
 Mesh geometry plumbing: decomposition leaves its input alone, the bounding box
 follows the vertices, and welding merges what it should.

	`c3c test --test-filter collision_tests::geometry`
*>
module collision_tests::geometry;

import collision;
import collision_testutil;
import std::collections::list;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

fn Mesh u_shape()
{
	Mesh mesh;
	mesh.init(mem, 24, 36);
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { -1.5, -0.5, 0 }, { -0.5, 0.5, 2 });
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { 0.5, -0.5, 0 }, { 1.5, 0.5, 2 });
	collision_testutil::push_box(&mesh.vertices, &mesh.triangles, { -1.5, -0.5, -1 }, { 1.5, 0.5, 0 });
	return mesh;
}

// decompose used to validate and weld the caller's own mesh, which moved its
// vertices and left its BVH pointing at the old arrays.
fn void test_decompose_leaves_input_untouched() @test
{
	Mesh mesh = u_shape();
	defer mesh.free();
	// Add a duplicate vertex and a degenerate-looking weld candidate the copy would change.
	mesh.vertices.push(mesh.vertices[0]);
	mesh.triangles.push({ 0, 1, (uint)mesh.vertices.len() - 1 });
	mesh.build_bvh();

	Vec3[] before_v = mesh.vertices.to_tarray();
	Triangle[] before_t = mesh.triangles.to_tarray();
	Vec3* vertex_ptr = mesh.vertices.entries;
	Triangle* tri_ptr = mesh.triangles.entries;
	TriBVH* bvh = mesh.bvh;
	Aabb3 box_before = mesh.get_aabb();

	MeshList pieces = mesh.decompose(collision::COACD_FAST);
	defer pieces.release();

	test::@check(pieces.len() >= 2, "decomposed into %d pieces", pieces.len());
	test::@check(mesh.vertices.len() == before_v.len, "vertex count unchanged");
	test::@check(mesh.triangles.len() == before_t.len, "triangle count unchanged");
	test::@check(mesh.vertices.entries == vertex_ptr && mesh.triangles.entries == tri_ptr, "storage not reallocated");
	test::@check(mesh.bvh == bvh, "BVH untouched");
	foreach (i, v : before_v) test::@check(mesh.vertices[i] == v, "vertex %d changed", i);
	foreach (i, t : before_t) test::@check(mesh.triangles[i] == t, "triangle %d changed", i);
	test::@check(mesh.get_aabb() == box_before);

	// And the BVH still answers.
	test::@check(mesh.bvh.ray_intersects({ .origin = { -1, 0, 10 }, .direction = { 0, 0, -1 } }).collided);
}

fn void test_mesh_aabb_follows_vertices() @test
{
	Mesh mesh;
	mesh.init(mem);
	defer mesh.free();

	mesh.add_triangle({ 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 });
	Aabb3 first = mesh.get_aabb();
	test::@check(first.max == (Vec3){ 1, 1, 0 });

	mesh.add_triangle({ 5, 0, 0 }, { 6, 0, 0 }, { 5, 3, 2 });
	Aabb3 second = mesh.get_aabb();
	test::@check(second.max == (Vec3){ 6, 3, 2 }, "box grew with the new triangle");
	test::@check(second.min == (Vec3){ 0, 0, 0 });
}

fn void test_add_fan() @test
{
	Mesh mesh;
	mesh.init(mem);
	defer mesh.free();

	Vec3[4] quad = { { 0, 0, 0 }, { 1, 0, 0 }, { 1, 1, 0 }, { 0, 1, 0 } };
	mesh.add_fan(quad[:4]);
	test::@check(mesh.triangles.len() == 2);
	mesh.add_fan(quad[:2]);
	test::@check(mesh.triangles.len() == 2, "fewer than three points adds nothing");
}

// Vertices within the weld radius merge into the earliest one; farther ones stay.
fn void test_weld_radius_and_grid_matches_brute_force() @test
{
	List {Vec3} verts;
	List {Triangle} tris;
	verts.init(mem);
	tris.init(mem);
	defer
	{
		verts.free();
		tris.free();
	}

	// Pseudo-random cloud dense enough that many pairs fall inside the radius,
	// with a few exact duplicates and points straddling cell borders.
	uint state = 12345;
	for (int i = 0; i < 400; i++)
	{
		state = state * 1664525u + 1013904223u;
		float x = (float)(state >> 8) / 16777216.0f * 1.5f;
		state = state * 1664525u + 1013904223u;
		float y = (float)(state >> 8) / 16777216.0f * 1.5f;
		state = state * 1664525u + 1013904223u;
		float z = (float)(state >> 8) / 16777216.0f * 1.5f;
		verts.push({ x, y, z });
	}
	for (int i = 0; i + 2 < 400; i += 3) tris.push({ (uint)i, (uint)i + 1, (uint)i + 2 });

	// Reference: linear scan with the same radius.
	const float RADIUS = 0.0707107f;
	List {Vec3} expected;
	expected.init(mem);
	defer expected.free();
	foreach (p : verts)
	{
		bool merged = false;
		foreach (q : expected)
		{
			if (p.distance_sq(q) <= RADIUS * RADIUS)
			{
				merged = true;
				break;
			}
		}
		if (!merged) expected.push(p);
	}

	collision::weld_remesh(&verts, &tris);
	test::@check(verts.len() == expected.len(), "welded to %d vertices, brute force gives %d", verts.len(), expected.len());
	foreach (i, q : expected) test::@check(verts[i] == q, "kept vertex %d differs", i);
	foreach (t : tris) test::@check(t.x != t.y && t.x != t.z && t.y != t.z, "degenerate triangle left in");

	// Two points just inside and just outside the radius.
	List {Vec3} pair;
	List {Triangle} pair_tris;
	pair.init(mem);
	pair_tris.init(mem);
	defer
	{
		pair.free();
		pair_tris.free();
	}
	pair.push_all({ { 0, 0, 0 }, { 0.06, 0, 0 }, { 0.5, 0, 0 }, { 0.5, 0.09, 0 } });
	pair_tris.push({ 0, 2, 3 });
	collision::weld_remesh(&pair, &pair_tris);
	test::@check(pair.len() == 3, "0.06 apart welds, 0.09 apart does not; got %d", pair.len());
}

<*
 quickhull: that what comes back is a closed convex surface, wound outwards,
 with every input point inside it.

	`c3c test --test-filter collision_tests::hull`

 Four properties, checked by the macros at the top rather than against expected
 output, because a hull has no single right triangulation to compare to. Closed:
 every directed edge appears exactly once each way, so a hole, a repeated face
 and an edge shared by three faces all show up. Euler: V - E + F is 2. Wound
 outwards: the centroid of a closed convex surface is inside it, so every face
 turns its back on it. Convex: no input point lies outside any face by more than
 quickhull's own tolerance.

 The planes in `hull_worst_outside` are rebuilt in double on purpose — a thin
 triangle's float cross product has no direction left worth reading, and the
 check would measure its own noise instead of the hull. `test_hull_thin_faces`
 is the case that argument comes from: in float the surface still comes out
 closed with the right Euler number and is wrong anyway, one face wound inwards
 and a point of the hull's own sitting 0.85 outside it.

 Each `quickhull` result is a `Mesh` with two lists in it, and the degenerate
 test holds five at once.
*>
module collision_tests::hull;

import collision;
import std::core::test;
import std::math;
import std::collections::list;

alias Vec3 = float[<3>];
alias BoxPoints = List {Vec3};

// Every directed edge of a closed triangle mesh appears exactly once, and the
// same edge walked backwards appears exactly once as well. A hole, a repeated
// face and an edge shared by three faces all show up here.
macro bool hull_is_closed(Mesh* hull)
{
	if (hull.triangles.len() == 0) return false;

	foreach (a : hull.triangles)
	{
		uint[3] left = { a.x, a.y, a.z };
		for (int e = 0; e < 3; e++)
		{
			uint from = left[e];
			uint to = left[(e + 1) % 3];
			int forward = 0;
			int backward = 0;
			foreach (b : hull.triangles)
			{
				uint[3] right = { b.x, b.y, b.z };
				for (int f = 0; f < 3; f++)
				{
					if (right[f] == from && right[(f + 1) % 3] == to) forward++;
					if (right[f] == to && right[(f + 1) % 3] == from) backward++;
				}
			}
			if (forward != 1 || backward != 1) return false;
		}
	}
	return true;
}

// V - E + F, with E read off a closed triangle mesh as 3F / 2. Two for a sphere.
macro int hull_euler(Mesh* hull)
{
	int faces = (int)hull.triangles.len();
	return (int)hull.vertices.len() - faces * 3 / 2 + faces;
}

// The largest distance any of `points` sits outside any face of the hull.
// The planes are rebuilt in double on purpose: a thin triangle's float cross
// product has no direction left worth reading, and the check would end up
// measuring its own noise instead of the hull.
macro double hull_worst_outside(Mesh* hull, Vec3[] points)
{
	double worst = 0;
	foreach (t : hull.triangles)
	{
		double[<3>] a = { hull.vertices[t.x].x, hull.vertices[t.x].y, hull.vertices[t.x].z };
		double[<3>] b = { hull.vertices[t.y].x, hull.vertices[t.y].y, hull.vertices[t.y].z };
		double[<3>] c = { hull.vertices[t.z].x, hull.vertices[t.z].y, hull.vertices[t.z].z };
		double[<3>] normal = (b - a).cross(c - a);
		double len = normal.length();
		if (len <= 0) continue; // zero area, no plane to be outside of

		foreach (p : points)
		{
			double[<3>] q = { p.x, p.y, p.z };
			double dist = normal.dot(q - a) / len;
			if (dist > worst) worst = dist;
		}
	}
	return worst;
}

// Windings agree and point outwards. The centroid of a closed convex surface is
// inside it, so every face has to turn its back on it.
macro bool hull_faces_point_outward(Mesh* hull)
{
	if (hull.vertices.len() == 0) return false;

	Vec3 centre = { 0, 0, 0 };
	foreach (v : hull.vertices) centre += v;
	centre = centre / (float)hull.vertices.len();
	double[<3>] inside = { centre.x, centre.y, centre.z };

	foreach (t : hull.triangles)
	{
		double[<3>] a = { hull.vertices[t.x].x, hull.vertices[t.x].y, hull.vertices[t.x].z };
		double[<3>] b = { hull.vertices[t.y].x, hull.vertices[t.y].y, hull.vertices[t.y].z };
		double[<3>] c = { hull.vertices[t.z].x, hull.vertices[t.z].y, hull.vertices[t.z].z };
		double[<3>] normal = (b - a).cross(c - a);
		double len = normal.length();
		if (len <= 0) continue; // zero area, no orientation to check

		if (normal.dot(inside - a) / len >= 0) return false;
	}
	return true;
}

// The same yardstick quick_hull.c3 measures with: an epsilon scaled to how far
// the input reaches from the origin.
macro double hull_tolerance_of(Vec3[] points)
{
	Vec3 extent = { 0, 0, 0 };
	foreach (p : points)
	{
		extent.x = math::max(extent.x, math::abs(p.x));
		extent.y = math::max(extent.y, math::abs(p.y));
		extent.z = math::max(extent.z, math::abs(p.z));
	}
	return collision::HULL_EPSILON * (double)(extent.x + extent.y + extent.z);
}

// xorshift32, so the cloud below is the same cloud on every machine and run
macro float hull_random(uint* state)
{
	uint x = *state;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*state = x;
	return (float)(x % 20001) / 10000.0f - 1.0f;
}

// Shapes whose hull is known before the algorithm runs. The lattice is the
// interesting one: its corners are the whole answer and every other point is
// exactly on a face, exactly on an edge or inside, so the run spends its time
// deciding what to do with points that are exactly coplanar with a face.
fn void test_hull_known_shapes() @test
{
	Vec3[4] tetra = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
	Mesh tetra_hull = collision::quickhull(tetra[..]);
	defer tetra_hull.free();

	test::@check(tetra_hull.vertices.len() == 4 && tetra_hull.triangles.len() == 4,
		"a tetrahedron is 4 points and 4 faces, got %d and %d",
		(int)tetra_hull.vertices.len(), (int)tetra_hull.triangles.len());
	test::@check(hull_is_closed(&tetra_hull), "the tetrahedron hull is not closed");
	test::@check(hull_euler(&tetra_hull) == 2, "tetrahedron V - E + F = %d", hull_euler(&tetra_hull));
	test::@check(hull_faces_point_outward(&tetra_hull), "a tetrahedron face points inwards");

	BoxPoints lattice;
	lattice.init(mem, 256);
	defer lattice.free();
	for (int x = 0; x <= 4; x++)
	{
		for (int y = 0; y <= 4; y++)
		{
			for (int z = 0; z <= 4; z++)
			{
				lattice.push({ (float)x * 0.5f - 1, (float)y * 0.5f - 1, (float)z * 0.5f - 1 });
			}
		}
	}
	// And every corner three times over, because duplicated points are what a
	// glTF position stream is made of
	Vec3[8] corners = {
		{ -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 }, { -1, 1, -1 },
		{ -1, -1, 1 }, { 1, -1, 1 }, { 1, 1, 1 }, { -1, 1, 1 },
	};
	for (int i = 0; i < 3; i++)
	{
		foreach (corner : corners) lattice.push(corner);
	}

	Mesh box_hull = collision::quickhull(lattice.array_view());
	defer box_hull.free();

	test::@check(box_hull.vertices.len() == 8, "a box is 8 corners, got %d", (int)box_hull.vertices.len());
	test::@check(box_hull.triangles.len() == 12, "a triangulated box is 12 faces, got %d", (int)box_hull.triangles.len());
	test::@check(hull_is_closed(&box_hull), "the box hull is not closed");
	test::@check(hull_euler(&box_hull) == 2, "box V - E + F = %d", hull_euler(&box_hull));
	test::@check(hull_faces_point_outward(&box_hull), "a box face points inwards");
	test::@check(hull_worst_outside(&box_hull, lattice.array_view()) <= hull_tolerance_of(lattice.array_view()),
		"a lattice point is %g outside the box hull", hull_worst_outside(&box_hull, lattice.array_view()));
}

// Points on a sphere are all on their own hull, so nothing may be quietly
// dropped, and dense enough that neighbouring faces are nearly coplanar
// everywhere - which is where an implementation that stops peeling faces at its
// tolerance leaves the new vertex sitting outside the ones it left behind.
fn void test_hull_sphere_cloud() @test
{
	uint seed = 0x2545f491;
	BoxPoints cloud;
	cloud.init(mem, 512);
	defer cloud.free();

	while (cloud.len() < 700)
	{
		Vec3 v = { hull_random(&seed), hull_random(&seed), hull_random(&seed) };
		float len = v.length();
		if (len < 0.25f) continue; // near the centre the direction is all rounding
		cloud.push(v / len);
	}

	Mesh hull = collision::quickhull(cloud.array_view());
	defer hull.free();

	test::@check(hull.vertices.len() == cloud.len(), "%d of %d points on the sphere missed the hull",
		(int)(cloud.len() - hull.vertices.len()), (int)cloud.len());
	test::@check(hull_is_closed(&hull), "the sphere hull is not closed");
	test::@check(hull_euler(&hull) == 2, "sphere V - E + F = %d", hull_euler(&hull));
	test::@check(hull_faces_point_outward(&hull), "a sphere hull face points inwards");
	test::@check(hull_worst_outside(&hull, cloud.array_view()) <= hull_tolerance_of(cloud.array_view()),
		"a point of the sphere is %g outside its own hull", hull_worst_outside(&hull, cloud.array_view()));
}

// Input with no volume to enclose has no hull. What matters is that it says so
// promptly rather than hanging or handing back a torn surface.
fn void test_hull_degenerate_input() @test
{
	Vec3[3] too_few = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 1, 0 } };
	Vec3[6] identical = { { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 }, { 1, 2, 3 } };
	Vec3[5] collinear = { { 0, 0, 0 }, { 1, 1, 1 }, { 2, 2, 2 }, { 3, 3, 3 }, { -1, -1, -1 } };
	Vec3[9] coplanar = {
		{ 0, 0, 0 }, { 1, 0, 0 }, { 2, 0, 0 },
		{ 0, 1, 0 }, { 1, 1, 0 }, { 2, 1, 0 },
		{ 0, 2, 0 }, { 1, 2, 0 }, { 2, 2, 0 },
	};
	// Not quite coplanar, and the same answer: the z jitter is four orders
	// below the tolerance, so a simplex built on it would be a sliver whose
	// plane is nothing but rounding, and every decision taken afterwards would
	// be taken against that plane.
	Vec3[9] nearly_coplanar = {
		{ 0, 0, 1e-9f }, { 1, 0, -1e-9f }, { 2, 0, 1e-9f },
		{ 0, 1, -1e-9f }, { 1, 1, 1e-9f }, { 2, 1, -1e-9f },
		{ 0, 2, 1e-9f }, { 1, 2, -1e-9f }, { 2, 2, 1e-9f },
	};

	Mesh few_hull = collision::quickhull(too_few[..]);
	Mesh same_hull = collision::quickhull(identical[..]);
	Mesh line_hull = collision::quickhull(collinear[..]);
	Mesh flat_hull = collision::quickhull(coplanar[..]);
	Mesh nearly_flat_hull = collision::quickhull(nearly_coplanar[..]);
	defer
	{
		few_hull.free();
		same_hull.free();
		line_hull.free();
		flat_hull.free();
		nearly_flat_hull.free();
	}

	test::@check(few_hull.triangles.len() == 0, "three points made %d faces", (int)few_hull.triangles.len());
	test::@check(same_hull.triangles.len() == 0, "one point six times made %d faces", (int)same_hull.triangles.len());
	test::@check(line_hull.triangles.len() == 0, "a line made %d faces", (int)line_hull.triangles.len());
	test::@check(flat_hull.triangles.len() == 0, "a plane made %d faces", (int)flat_hull.triangles.len());
	test::@check(nearly_flat_hull.triangles.len() == 0, "a plane with 1e-9 of jitter made %d faces",
		(int)nearly_flat_hull.triangles.len());
}

// Fourteen points lifted out of truck.glb's third primitive - the smallest
// subset of it that still breaks a hull built by testing each face's plane on
// its own. Eight of them share a y exactly, and that is the whole story: a
// point near that plane reads as outside two faces that do not touch, coning it
// to both tears the surface, and every later pass reports one horizon edge per
// face along the tear. On the full 151-point primitive the run never came back;
// on these fourteen the old iteration cap cut it off at 2696 faces for 14
// points. The answer is 24.
fn void test_hull_truck_panel() @test
{
	Vec3[14] panel = {
		{ 1.10368001f, -1.10000002f, -1.67109001f },
		{ 1.40384996f, 1.00699997f, -2.30337f },
		{ 1.58576f, 1.00699997f, -1.69965005f },
		{ 1.38843f, 0.948902011f, -2.35454011f },
		{ 1.60117996f, 0.948902011f, -1.64847004f },
		{ 1.23666f, 1.09000003f, -2.35390997f },
		{ 1.10368001f, 1.09000003f, -2.35390997f },
		{ 1.27451003f, 1.09000003f, -2.32679009f },
		{ 1.48608994f, 1.09000003f, -1.70337999f },
		{ 1.45035994f, 1.09000003f, -1.67109001f },
		{ 1.08229005f, 1.09000003f, -1.69247997f },
		{ 1.10368001f, 1.09000003f, -1.67109001f },
		{ 1.40384996f, -1.00699997f, -2.30337f },
		{ 1.38843f, -0.948902011f, -2.35454011f },
	};

	Mesh hull = collision::quickhull(panel[..]);
	defer hull.free();

	test::@check(hull.triangles.len() > 0 && hull.triangles.len() <= 40,
		"14 points cannot make %d faces", (int)hull.triangles.len());
	test::@check(hull_is_closed(&hull), "the panel hull is not closed");
	test::@check(hull_euler(&hull) == 2, "panel V - E + F = %d", hull_euler(&hull));
	test::@check(hull_faces_point_outward(&hull), "a panel hull face points inwards");
	test::@check(hull_worst_outside(&hull, panel[..]) <= hull_tolerance_of(panel[..]),
		"a panel point is %g outside its own hull", hull_worst_outside(&hull, panel[..]));
}

// Fifteen points out of truck.glb's first primitive, reduced from the 828 it
// ships with. Between them they make triangles thin enough that a float cross
// product has no direction left in it. With the planes in float the surface
// still comes out closed with the right Euler number, and is wrong anyway: one
// face ends up wound inwards and a point of the hull's own sits 0.85 outside
// it. Nothing but a convexity or winding check would notice.
fn void test_hull_thin_faces() @test
{
	Vec3[15] rim = {
		{ 0.30250001f, -0.800000012f, -0.30250001f },
		{ 2.60208997e-18f, -0.800000012f, -0.4278f },
		{ 0.30250001f, -0.800000012f, 0.30250001f },
		{ 0.4278f, -0.800000012f, 6.93889018e-18f },
		{ 0.413787991f, 1.02477002f, 6.93889018e-18f },
		{ -0.413222998f, 0.987999976f, -0.110722996f },
		{ -0.358350992f, 1.02477002f, -0.206893995f },
		{ 2.60208997e-18f, 0.987999976f, -0.4278f },
		{ 0.110722996f, 0.987999976f, -0.413222998f },
		{ -0.0636605024f, 1.05799997f, 0.237584993f },
		{ -0.292591989f, -1.02477002f, 0.292591989f },
		{ -0.206893995f, -1.02477002f, -0.358350992f },
		{ 2.60208997e-18f, -0.987999976f, -0.4278f },
		{ 0.110722996f, -0.987999976f, -0.413222998f },
		{ 2.60208997e-18f, -0.987999976f, 0.4278f },
	};

	Mesh hull = collision::quickhull(rim[..]);
	defer hull.free();

	test::@check(hull_is_closed(&hull), "the rim hull is not closed");
	test::@check(hull_euler(&hull) == 2, "rim V - E + F = %d", hull_euler(&hull));
	test::@check(hull_faces_point_outward(&hull), "a rim hull face points inwards");
	test::@check(hull_worst_outside(&hull, rim[..]) <= hull_tolerance_of(rim[..]),
		"a rim point is %g outside its own hull", hull_worst_outside(&hull, rim[..]));
}

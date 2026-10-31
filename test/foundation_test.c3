<*
 The shared helpers the rest of the library builds on: triangle vocabulary,
 transforms, prepared shapes, shape inertia, vector helpers, XPBD scalars.

	`c3c test --test-filter collision_tests::foundation`
*>
module collision_tests::foundation;

import collision;
import collision_testutil;
import std::collections::list;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

// Deterministic pseudo-random floats in [0, 1) so the tests never touch libm or a seed source.
struct Lcg
{
	uint state;
}

fn float Lcg.next(&self)
{
	self.state = self.state * 1664525u + 1013904223u;
	return (float)(self.state >> 8) / 16777216.0f;
}

fn Vec3 Lcg.vec(&self, float lo, float hi)
{
	return { lo + (hi - lo) * self.next(), lo + (hi - lo) * self.next(), lo + (hi - lo) * self.next() };
}

fn Quaternionf Lcg.rotation(&self)
{
	Vec3 axis = self.vec(-1, 1).normalize();
	return collision::dmath::quat_from_axis_angle(axis, self.next() * 6.0f);
}

fn bool close(Vec3 a, Vec3 b, float eps) @local => a.distance(b) <= eps;

// --- Triangle vocabulary -----------------------------------------------------

fn void test_triangle_verts() @test
{
	TriangleVerts t = { { 0, 0, 0 }, { 1, 0, 0 }, { 0, 0, -1 } };
	test::@check(close(t.normal(), { 0, 1, 0 }, 1e-6), "normal %s", t.normal());
	test::@check(math::abs(t.area() - 0.5f) < 1e-6, "area %f", t.area());
	test::@check(close(t.centroid(), { 1.0f / 3.0f, 0, -1.0f / 3.0f }, 1e-6));
	Aabb3 box = t.aabb();
	test::@check(box.min == (Vec3){ 0, 0, -1 } && box.max == (Vec3){ 1, 0, 0 }, "aabb %s", box);

	test::@check(close(t.closest_point({ 0.25f, 3, -0.25f }), { 0.25f, 0, -0.25f }, 1e-6), "interior");
	test::@check(close(t.closest_point({ -2, 1, 2 }), { 0, 0, 0 }, 1e-6), "vertex region");
	test::@check(close(t.closest_point({ 2, 0, 1 }), { 1, 0, 0 }, 1e-6), "vertex b region");
	test::@check(close(t.closest_point({ 0.5f, 0, 1 }), { 0.5f, 0, 0 }, 1e-6), "edge region");
}

fn void test_index_triangle_matches_verts() @test
{
	Vec3[4] verts = { { 0, 0, 0 }, { 2, 0, 0 }, { 0, 3, 0 }, { 0, 0, 4 } };
	Triangle tri = { 0, 1, 2 };
	test::@check(tri.in_range(4) && !((Triangle){ 0, 1, 4 }).in_range(4));
	test::@check(tri.verts(&verts)[1] == (Vec3){ 2, 0, 0 });
	test::@check(close(tri.get_normal(&verts), { 0, 0, 1 }, 1e-6));
	test::@check(math::abs(tri.get_area(&verts) - 3.0) < 1e-6);
	test::@check(close(tri.get_center(&verts), { 2.0f / 3.0f, 1, 0 }, 1e-6));
}

fn void test_mesh_foreach_tri_and_volume() @test
{
	Mesh mesh = collision_testutil::create_test_cube();
	defer mesh.free();

	sz count = 0;
	mesh.@foreach_tri(; Vec3 a, Vec3 b, Vec3 c)
	{
		count++;
	};
	test::@check(count == 12, "visited %d", count);
	test::@check(math::abs(mesh.get_volume() - 1.0) < 1e-5, "volume %f", mesh.get_volume());
	test::@check(close(mesh.center_of_mass(), { 0, 0, 0 }, 1e-5), "com %s", mesh.center_of_mass());

	Matrix3f inertia = mesh.inertia_tensor(6.0f);
	test::@check(math::abs(inertia.m00 - 1.0f) < 1e-4 && math::abs(inertia.m11 - 1.0f) < 1e-4, "cube inertia m/6 * 2 * a^2 = %f", inertia.m00);

	mesh.ensure_bvh();
	test::@check(mesh.has_bvh());
}

fn void test_heightmap_foreach_matches_query() @test
{
	uint side = 5;
	float[25] heights;
	for (uint i = 0; i < 25; i++) heights[i] = 0.1f * (float)(i % 7);
	Heightmap field = collision::create_heightmap_from_data(&heights, side, side, 1.0f, { -2, 0, -2 });
	defer field.free();

	Aabb3 q = { { -1.2f, -5, -0.7f }, { 0.9f, 5, 1.4f } };
	List {TriangleVerts} listed;
	listed.init(mem);
	defer listed.free();
	field.query_triangles(q, &listed);

	sz i = 0;
	bool same = true;
	field.@foreach_triangle(q; TriangleVerts tri)
	{
		if (i >= listed.len() || tri != listed[i]) same = false;
		i++;
	};
	test::@check(same && i == listed.len() && i > 0, "visited %d of %d", i, listed.len());
	test::@check(close(listed[0].normal(), listed[0].normal(), 1e-6) && listed[0].normal().y > 0, "faces up");
}

// --- Aabb3 -------------------------------------------------------------------

fn void test_aabb_additions() @test
{
	Aabb3 box = { { -1, -2, -3 }, { 1, 2, 3 } };
	test::@check(math::abs(box.surface_area() - 2 * (8 + 24 + 12)) < 1e-5);

	Vec3[8] c = box.corners();
	test::@check(c[0] == box.min && c[7] == box.max);
	test::@check(c[1] == (Vec3){ 1, -2, -3 } && c[2] == (Vec3){ -1, 2, -3 } && c[3] == (Vec3){ -1, -2, 3 });
	test::@check(c[4] == (Vec3){ 1, 2, -3 } && c[5] == (Vec3){ 1, -2, 3 } && c[6] == (Vec3){ -1, 2, 3 });

	Vec3[3] pts = { { 1, 5, 2 }, { -1, 0, 7 }, { 3, 3, 3 } };
	Aabb3 b = collision::aabb_of_points(&pts);
	test::@check(b.min == (Vec3){ -1, 0, 2 } && b.max == (Vec3){ 3, 5, 7 }, "%s", b);

	Aabb3 child = box.child_bounds(0b101);
	test::@check(child.min == (Vec3){ 0, -2, 0 } && child.max == (Vec3){ 1, 0, 3 }, "child %s", child);

	Aabb3 inv = { { 2, 2, 2 }, { -2, -2, -2 } };
	inv.fix();
	test::@check(inv.min == (Vec3){ -2, -2, -2 } && inv.max == (Vec3){ 2, 2, 2 });
	test::@check(box.collides({ { 1, 2, 3 }, { 4, 4, 4 } }) && !box.collides({ { 1.1f, 0, 0 }, { 2, 1, 1 } }));
	test::@check(box.contains_point({ 1, 2, 3 }) && !box.contains_point({ 1.01f, 0, 0 }));
}

// --- Vector helpers ------------------------------------------------------------

fn void test_vector_helpers() @test
{
	Lcg rng = { 7 };
	for (int i = 0; i < 50; i++)
	{
		Vec3 u = rng.vec(-1, 1).normalize();
		Vec3 p = collision::any_perpendicular(u);
		test::@check(math::abs(p.length() - 1.0f) < 1e-5 && math::abs(p.dot(u)) < 1e-5, "perpendicular of %s is %s", u, p);
	}

	Vec3 axis;
	float angle = collision::angle_axis_between({ 1, 0, 0 }, { 0, 1, 0 }, &axis);
	test::@check(math::abs(angle - math::PI / 2) < 1e-5 && close(axis, { 0, 0, 1 }, 1e-6), "quarter turn %f %s", angle, axis);
	angle = collision::angle_axis_between({ 1, 0, 0 }, { -1, 0, 0 }, &axis);
	test::@check(math::abs(angle - math::PI) < 1e-4 && math::abs(axis.dot((Vec3){ 1, 0, 0 })) < 1e-5, "antiparallel %f %s", angle, axis);
	angle = collision::angle_axis_between({ 0, 1, 0 }, { 0, 1, 0 }, &axis);
	test::@check(angle == 0, "parallel");

	Vec3[3] pts = { { 0, 0, 0 }, { 1, 2, 0 }, { 0, 1, 5 } };
	test::@check(collision::support_of(&pts, { 0, 1, 0 }) == (Vec3){ 1, 2, 0 });
	test::@check(collision::support_of(&pts, { 0, 0, 1 }) == (Vec3){ 0, 1, 5 });
	test::@check(collision::support_of({}, { 0, 0, 1 }) == (Vec3){ 0, 0, 0 });

	test::@check(((Vec3){ 0, 0, 0 }).near({ 0.003f, 0.004f, 0 }, 0.006f) && !((Vec3){ 0, 0, 0 }).near({ 0.003f, 0.004f, 0 }, 0.004f));
	test::@check(collision::is_finite3({ 1, 2, 3 }) && !collision::is_finite3({ 1, float::nan, 3 }));
}

fn void test_contact_hit_keep_deeper() @test
{
	ContactHit best;
	best.keep_deeper({ .hit = false, .depth = 9 });
	test::@check(!best.hit);
	best.keep_deeper({ .hit = true, .depth = 1, .normal = { 0, 1, 0 } });
	best.keep_deeper({ .hit = true, .depth = 0.5f, .normal = { 1, 0, 0 } });
	test::@check(best.hit && best.depth == 1 && best.normal == (Vec3){ 0, 1, 0 });
	best.keep_deeper({ .hit = true, .depth = 2, .normal = { 0, 0, 1 } });
	test::@check(best.depth == 2 && best.normal == (Vec3){ 0, 0, 1 });
}

// Formulas as they were written inline in softbody.c3 / resolver.c3.
fn void test_dmath_axis_angle_matches_inline() @test
{
	Lcg rng = { 11 };
	for (int i = 0; i < 50; i++)
	{
		Vec3 axis = rng.vec(-1, 1).normalize();
		Vec3 v = rng.vec(-3, 3);
		float angle = rng.next() * 3.0f;

		float[<2>] sc = collision::dmath::sincos(angle);
		Vec3 expect = v * sc[1] + axis.cross(v) * sc[0] + axis * (axis.dot(v) * (1.0f - sc[1]));
		test::@check(collision::dmath::rotate_about_axis(v, axis, angle) == expect, "rodrigues differs");

		float[<2>] h = collision::dmath::sincos(angle * 0.5f);
		QuaternionValue dq = { ...(axis * h[0]), h[1] };
		test::@check(collision::dmath::quat_from_axis_angle(axis, angle).v == dq.v, "quaternion differs");
	}
}

// --- XPBD scalars --------------------------------------------------------------

fn void test_xpbd_helpers_match_inline() @test
{
	Lcg rng = { 3 };
	for (int i = 0; i < 100; i++)
	{
		float c = rng.next() - 0.5f;
		float w = rng.next() + 0.1f;
		float alpha = rng.next() * 0.01f;
		float lambda = rng.next() - 0.5f;

		float expect_dl = -(c + alpha * lambda) / (w + alpha);
		float expect_lambda = lambda + expect_dl;
		float dl = collision::xpbd_delta_lambda(c, w, alpha, &lambda);
		test::@check(dl == expect_dl && lambda == expect_lambda);

		uint total = 1 + (uint)(rng.next() * 8.0f);
		uint it = (uint)(rng.next() * (float)total);
		float base = i % 5 == 0 ? 0.0f : rng.next();

		float compliance = base;
		if (compliance > 0 && total > 1)
		{
			float progress = (float)(it + 1) / (float)total;
			compliance = base * (1.0f + 3.0f * (1.0f - progress));
		}
		test::@check(collision::ramped_compliance(base, it, total) == compliance);

		float k = i % 4 == 0 ? 0.0f : rng.next() * 100.0f;
		float dt = 1.0f / 240.0f;
		float expect = k > 0 ? (1.0f / k) / (dt * dt) : 0.25f;
		test::@check(collision::stiffness_to_compliance(k, dt, 0.25f) == expect);
	}
}

// --- Transforms and prepared shapes -----------------------------------------------

fn void test_transformed_shape_roundtrip() @test
{
	Lcg rng = { 5 };
	Aabb3 box = { { -1, -1, -1 }, { 1, 1, 1 } };
	for (int i = 0; i < 20; i++)
	{
		TransformedShape t = { .translation = rng.vec(-5, 5), .scale = rng.vec(0.5f, 2.0f), .rotation = rng.rotation(), .shape = &box };
		Vec3 p = rng.vec(-3, 3);
		test::@check(close(t.to_local(t.to_world(p)), p, 1e-4), "point roundtrip");
		test::@check(close(t.dir_to_local(t.dir_to_world(p)), p, 1e-4), "direction roundtrip");
	}
}

fn void test_prepared_shape_matches_transformed() @test
{
	Lcg rng = { 42 };
	Aabb3 box = { { -0.5f, -1, -0.25f }, { 0.5f, 1, 0.25f } };
	Cylinder cyl = { .height = 1.5f, .radius_top = 0.5f, .radius_bottom = 0.4f };
	Sphere sphere = { .center = { 0, 0, 0 }, .radius = 0.7f };
	CollisionShape[3] shapes = { &box, &cyl, &sphere };

	for (int i = 0; i < 90; i++)
	{
		TransformedShape t = {
			.translation = rng.vec(-3, 3), .scale = rng.vec(0.5f, 2.0f), .rotation = rng.rotation(), .shape = shapes[i % 3]
		};
		PreparedShape prepared = t.prepare();
		Vec3 dir = rng.vec(-1, 1);
		Vec3 a = t.furthest_point(dir);
		Vec3 b = prepared.furthest_point(dir);
		test::@check(close(a, b, 1e-4), "support differs: %s vs %s", a, b);
		test::@check(prepared.get_type() == t.get_type());
	}

	// Un-normalised rotation: both scale by |q|^2 the same way.
	TransformedShape odd = { .translation = { 1, 2, 3 }, .scale = { 1, 2, 1 }, .rotation = { 0.1f, 0.5f, 0.2f, 1.3f }, .shape = &box };
	PreparedShape p_odd = odd.prepare();
	test::@check(close(odd.furthest_point({ 0.3f, -0.8f, 0.5f }), p_odd.furthest_point({ 0.3f, -0.8f, 0.5f }), 1e-4));
}

fn void test_prepared_shape_gives_same_collision() @test
{
	Lcg rng = { 99 };
	Aabb3 box = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	Sphere sphere = { .center = { 0, 0, 0 }, .radius = 0.6f };
	int colliding = 0;

	for (int i = 0; i < 60; i++)
	{
		TransformedShape a = { .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = rng.rotation(), .shape = &box };
		TransformedShape b = {
			.translation = rng.vec(-1.6f, 1.6f), .scale = { 1, 1.2f, 1 }, .rotation = rng.rotation(),
			.shape = i % 2 == 0 ? (CollisionShape)&box : (CollisionShape)&sphere,
		};
		PreparedShape pa = a.prepare();
		PreparedShape pb = b.prepare();

		CollisionInfo plain = collision::check_convex_collision(&a, &b, epa: true);
		CollisionInfo prepared = collision::check_convex_collision(&pa, &pb, epa: true);
		test::@check(plain.collided == prepared.collided, "case %d: collided %s vs %s", i, plain.collided, prepared.collided);
		if (plain.collided)
		{
			colliding++;
			test::@check(math::abs(plain.depth - prepared.depth) < 1e-3, "case %d: depth %f vs %f", i, plain.depth, prepared.depth);
		}
	}
	test::@check(colliding > 5, "only %d overlapping cases, test is not exercising EPA", colliding);
}

// --- Shape inertia ----------------------------------------------------------------

fn void test_shape_inertia_formulas() @test
{
	Sphere sphere = { .center = { 0, 0, 0 }, .radius = 2.0f };
	Matrix3f s = sphere.inertia_tensor(5.0f);
	float expect = 0.4f * 5.0f * 4.0f;
	test::@check(math::abs(s.m00 - expect) < 1e-5 && math::abs(s.m11 - expect) < 1e-5 && math::abs(s.m22 - expect) < 1e-5, "sphere %f", s.m00);

	// Solid cylinder along y: axial 1/2 m r^2, transverse 1/12 m (3 r^2 + h^2)
	Cylinder cyl = { .height = 3.0f, .radius_top = 1.0f, .radius_bottom = 1.0f };
	Matrix3f c = cyl.inertia_tensor(12.0f);
	test::@check(math::abs(c.m11 - 6.0f) < 1e-5, "cylinder axial %f", c.m11);
	test::@check(math::abs(c.m00 - 12.0f) < 1e-4 && math::abs(c.m22 - 12.0f) < 1e-4, "cylinder transverse %f", c.m00);

	// A capsule with no cylindrical section is a sphere.
	Capsule ball = { .height = 0, .radius_top = 1.0f, .radius_bottom = 1.0f };
	Matrix3f b = ball.inertia_tensor(5.0f);
	test::@check(math::abs(b.m00 - 2.0f) < 1e-5 && math::abs(b.m11 - 2.0f) < 1e-5 && math::abs(b.m22 - 2.0f) < 1e-5, "capsule ball %f", b.m00);

	// A capsule is heavier to turn than the cylinder alone and lighter than the enclosing cylinder.
	Capsule cap = { .height = 2.0f, .radius_top = 0.5f, .radius_bottom = 0.5f };
	Matrix3f k = cap.inertia_tensor(3.0f);
	Cylinder enclosing = { .height = 3.0f, .radius_top = 0.5f, .radius_bottom = 0.5f };
	Matrix3f e = enclosing.inertia_tensor(3.0f);
	test::@check(k.m00 < e.m00 && k.m11 < e.m11 && k.m00 == k.m22, "capsule %f axial %f", k.m00, k.m11);
	// Hand value: m_cyl = 3 * 0.5 / (0.5 + 1/6... ) via volumes r^2 h : 4/3 r^3
	float v_cyl = 0.25f * 2.0f;
	float v_cap = 4.0f / 3.0f * 0.125f;
	float m_cyl = 3.0f * v_cyl / (v_cyl + v_cap);
	float m_cap = 3.0f - m_cyl;
	float axial = 0.5f * m_cyl * 0.25f + 0.4f * m_cap * 0.25f;
	test::@check(math::abs(k.m11 - axial) < 1e-5, "capsule axial %f vs %f", k.m11, axial);
}

// A box with local inertia diag(a, b, c) turned 30 degrees about z: the world tensor is
// R diag R^T. Its xy product is sin*cos*(a - b) with a positive sign, and R e_x is an
// eigenvector with eigenvalue a. R^T diag R would give the opposite xy sign.
fn void test_rotated_box_world_inertia() @test
{
	Aabb3 box = { { -1, -0.5f, -0.25f }, { 1, 0.5f, 0.25f } };
	Matrix3f local = box.inertia_tensor(12.0f);
	float a = local.m00;
	float b = local.m11;
	float c = local.m22;
	test::@check(a != b && b != c);

	float angle = math::PI / 6.0f;
	QuaternionValue rot = collision::dmath::quat_from_axis_angle({ 0, 0, 1 }, angle);
	float s = math::sin(angle);
	float co = math::cos(angle);

	TransformedShape t = { .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = rot, .shape = &box };
	Matrix3f from_shape = t.inertia_tensor(12.0f);
	Matrix3f from_local = t.world_inertia_from_local(local);

	Matrix3f[2] worlds = { from_shape, from_local };
	foreach (w : worlds)
	{
		test::@check(math::abs(w.m00 - (a * co * co + b * s * s)) < 1e-4, "xx %f", w.m00);
		test::@check(math::abs(w.m11 - (a * s * s + b * co * co)) < 1e-4, "yy %f", w.m11);
		test::@check(math::abs(w.m22 - c) < 1e-4, "zz %f", w.m22);
		test::@check(math::abs(w.m01 - s * co * (a - b)) < 1e-4, "xy %f expected %f", w.m01, s * co * (a - b));
		test::@check(math::abs(w.m10 - w.m01) < 1e-5);

		Vec3 ex = { co, s, 0 };
		test::@check(close(w * ex, ex * a, 1e-4), "R e_x is not an eigenvector: %s", w * ex);
	}
}

fn void test_transformed_matrix_places_translation() @test
{
	Aabb3 box = { { -1, -1, -1 }, { 1, 1, 1 } };
	TransformedShape t = { .translation = { 1, 2, 3 }, .scale = { 2, 2, 2 }, .rotation = { 0, 0, 0, 1 }, .shape = &box };
	Vec3 world = t.to_world({ 1, 0, 0 });
	test::@check(close(world, { 3, 2, 3 }, 1e-6));
	Matrix4f m = t.matrix();
	float[<4>] r = m * (float[<4>]){ 1, 0, 0, 1 };
	test::@check(close(r.xyz, world, 1e-6), "matrix %s", r);
}

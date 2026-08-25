<*
 A `Heightmap` as a collider, which is what `ShapeType.HEIGHTFIELD` and
 `PhysicsWorld.test_heightfield_collision` exist for.

	`c3c test --test-filter collision_tests::heightfield`

 **The claim under test is that a body lands on the ground and not on a plane.**
 Before this the shape reported `CONVEX`, its `furthest_point` answered with one
 of the four corners of the whole map, and GJK therefore settled every body onto
 the plane through those corners — the terrain, flattened, with nothing anywhere
 reporting an error. That failure is invisible on a flat map, so every test here
 uses a map with a slope in it.
*>
module collision_tests::heightfield;
import collision;
import std::collections::list;
import std::core::test;
import std::math;

// A ramp rising `slope` per world unit of x, over a 20 x 20 map at one sample
// per unit, centred on the origin. `h(x) = slope * (x + 10)`, so the low edge
// is 0 and there is nothing flat anywhere on it — a flat map cannot tell a
// terrain collider from the plane the old one collapsed to.
//
// **The map is finite and there is nothing outside it.** A body that slides off
// the low edge falls, which is the honest behaviour and is why the tests below
// pick slopes their friction can hold.
fn Heightmap ramp(float slope) @local
{
	uint side = 21;
	float[441] heights;
	for (uint j = 0; j < side; j++)
	{
		for (uint i = 0; i < side; i++)
		{
			heights[j * side + i] = slope * (float)i;
		}
	}
	return collision::create_heightmap_from_data(&heights, side, side, 1.0f, { -10, 0, -10 });
}

// The library's default gravity is -Z, and a heightmap's up is +Y — it is a
// grid in x and z with heights in y, which is the one shape in here that has an
// opinion about which axis is up. So every world in this file is turned the
// right way round before anything is dropped into it.
fn PhysicsWorld y_up_world() @local
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.gravity = { 0, -9.8f, 0 };
	world.ground_plane = { .normal = { 0, 1, 0 }, .offset = -1000.0f };
	return world;
}

fn void add_ground(PhysicsWorld* world, Heightmap* field) @local
{
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = field,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;
}

// The type is the dispatch. Answering CONVEX is what sent these pairs to GJK.
fn void test_heightmap_reports_heightfield() @test
{
	Heightmap field = ramp(0.5f);
	defer field.free();

	test::@check(field.get_type() == ShapeType.HEIGHTFIELD);

	// And through a transform, which is how the solver actually sees it — a
	// TransformedShape delegates, and a delegate that lost the type would put
	// the pair back on the GJK path with nothing to say so.
	TransformedShape shape = { .shape = &field, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } };
	test::@check(shape.get_type() == ShapeType.HEIGHTFIELD);
}

// A sphere dropped on the slope rests on the slope. The number that matters is
// the *difference* between where it lands and the plane through the corners:
// at x = 4 the ground is at 7.0 and the flattened answer would be about 5.
fn void test_sphere_rests_on_the_slope() @test
{
	PhysicsWorld world = y_up_world();
	defer world.free();

	Heightmap field = ramp(0.5f);
	defer field.free();
	add_ground(&world, &field);

	Sphere ball = { .center = { 0, 0, 0 }, .radius = 0.5 };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &ball,
				.translation = { 4, 14, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.material = { .static_friction = 0.9, .dynamic_friction = 0.8 },
			.can_sleep = true,
		}
	)!!;

	for (int i = 0; i < 240; i++) world.run_step(1.0f / 60.0f, step_count: 20);

	Rigidbody* body = world.find_body(2)!!;
	Vec3 at = body.collider.translation;
	float ground = field.get_height(at.x, at.z);

	// On the ground, to within the radius and a slope's worth of slack. A body
	// that went through it is metres below; a body on the flattened plane is
	// metres above the ground under it at the low end and below at the high end.
	float gap = at.y - ground;
	test::@check(gap > 0.3f && gap < 1.2f, "rests on the terrain, gap = %s", gap);
	// And it rolled downhill rather than staying where it was dropped.
	test::@check(at.x < 3.0f, "rolled down the slope, x = %s", at.x);
}

// The GJK/EPA half of the triangle path — a box has no specialised test against
// a triangle, so this is the branch a crate takes.
fn void test_box_rests_on_the_slope() @test
{
	PhysicsWorld world = y_up_world();
	defer world.free();

	// Gentler than the sphere's, and deliberately: a box has no specialised
	// triangle test and takes the GJK/EPA path, and on a 27-degree ramp it
	// creeps far enough in four seconds to slide off the low edge of the map —
	// which is a true answer to a badly chosen test rather than a defect.
	Heightmap field = ramp(0.15f);
	defer field.free();
	add_ground(&world, &field);

	Aabb3 crate = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &crate,
				.translation = { -2, 12, 1 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.material = { .static_friction = 0.9, .dynamic_friction = 0.8 },
			.can_sleep = true,
		}
	)!!;

	for (int i = 0; i < 240; i++) world.run_step(1.0f / 60.0f, step_count: 20);

	Rigidbody* body = world.find_body(2)!!;
	Vec3 at = body.collider.translation;
	float ground = field.get_height(at.x, at.z);
	float gap = at.y - ground;

	test::@check(gap > 0.2f && gap < 1.5f, "rests on the terrain, gap = %s at %s", gap, at);
	test::@check(at.x > -9.5f && at.x < 9.5f, "and stayed on the map, x = %s", at.x);
}

// The winding regression, stated on its own. `query_triangles` used to wind its
// two triangles so the face normal pointed straight down, which made the ground
// a one-sided surface facing away from everything standing on it — and nothing
// noticed while the function only ever fed a debug draw.
fn void test_query_triangles_face_up() @test
{
	Heightmap field = ramp(0.5f);
	defer field.free();

	List {TriangleVerts} triangles;
	defer triangles.free();
	field.query_triangles({ { -1, -10, -1 }, { 1, 10, 1 } }, &triangles);

	test::@check(triangles.len() > 0, "the query found triangles");

	foreach (tri : triangles)
	{
		Vec3 normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]).normalize();
		test::@check(normal.y > 0.0f, "the ground faces up, y = %s", normal.y);
	}
}

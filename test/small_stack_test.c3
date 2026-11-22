<*
 Boxes 8 cm across, the size of a block picked up in VR: stacked, they must
 stand as boxes a metre across do. Each scene steps the way three.c3 does
 (`run_step(1/60, 1, 4)`) on one thread.

 These guard four separate failures, all of which only showed below about a
 metre, because each was a tolerance that does not scale with the shapes:
 - GJK compared an unnormalised direction with an absolute tolerance, so small
   boxes overlapping by millimetres read as apart;
 - a pair apart when the narrow phase looked got no contact, though the solver
   then pressed it together (the box under a stack is pushed up into the one
   above), so a stack sank and was kicked out on alternate substeps;
 - the manifold projected a box's own face corners onto its own face and glued
   them to the other body, so a box tilted on another one stood on points under
   it instead of rocking flat;
 - one normal pass over a manifold always left the tilt leaning the same way.

	`c3c test --test-filter collision_tests::small_stack`
*>
module collision_tests::small_stack;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

const float SIDE = 0.08f;
const float TABLE_TOP = 0.76f;
const Aabb3 FLOOR = { { -20, -1, -20 }, { 20, 0, 20 } };
const Aabb3 TABLE = { { -0.6f, 0, -1.0f }, { 0.6f, 0.76f, -0.4f } };
const Aabb3 CUBE = { { -0.04f, -0.04f, -0.04f }, { 0.04f, 0.04f, 0.04f } };

// A y-up world with a floor and a table, as examples/vr_controllers.js has.
fn PhysicsWorld* make_world()
{
	PhysicsWorld* world = mem::new(PhysicsWorld);
	*world = collision::DEFAULT_PHYSICS_WORLD;
	world.init(mem);
	world.gravity = { 0, -9.8f, 0 };
	world.thread_count = 1;
	world.add_body({ .id = 1, .collider = { .shape = &FLOOR, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
		.material = { .static_friction = 0.8f, .dynamic_friction = 0.8f, .restitution = 0.2f } })!!;
	world.add_body({ .id = 2, .collider = { .shape = &TABLE, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
		.material = { .static_friction = 0.8f, .dynamic_friction = 0.8f, .restitution = 0.2f } })!!;
	return world;
}

fn void free_world(PhysicsWorld* world)
{
	world.free();
	mem::free(world);
}

// A 0.3 kg cube, as the example's.
fn void add_cube(PhysicsWorld* world, sz id, Vec3 at, QuaternionValue rotation = { 0, 0, 0, 1 })
{
	world.add_body({
		.id = id, .mass = 0.3f, .gravity_factor = 1, .can_sleep = true,
		.collider = { .shape = &CUBE, .translation = at, .scale = { 1, 1, 1 }, .rotation = rotation },
		.material = { .static_friction = 0.7f, .dynamic_friction = 0.7f, .restitution = 0.2f },
	})!!;
}

// Where the `level`th cube of a stack on the table rests.
fn Vec3 level_at(int level) => { 0, TABLE_TOP + SIDE * 0.5f + level * SIDE, -0.65f };

fn void step(PhysicsWorld* world, int frames)
{
	for (int f = 0; f < frames; f++) world.run_step(1.0f / 60.0f, 1, 4);
}

fn float tilt_degrees(Rigidbody* body)
{
	Vec3 up = body.collider.rotation * (Vec3){ 0, 1, 0 };
	return math::acos(math::clamp(up.y, -1.0f, 1.0f)) * 180.0f / (float)math::PI;
}

fn void gjk_sees_a_millimetre_overlap_between_small_boxes() @test
{
	// One cube 1.7 mm into another, off centre by half a millimetre and turned a
	// quarter of a degree, as a stack is mid-substep. The raw tolerance read
	// this as apart (found by search: it misses overlaps up to about 2 mm).
	Vec3 axis = { 0.001428f, -0.000312f, -0.001834f };
	QuaternionValue turn = { ...axis, math::sqrt(1 - axis.length_sq()) };
	Aabb3 box = CUBE;
	TransformedShape lower = { .shape = &box, .translation = { 0, 0, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } };
	TransformedShape upper = { .shape = &box, .translation = { -0.000599f, 0.078330f, -0.000567f }, .scale = { 1, 1, 1 }, .rotation = turn };

	CollisionInfo info = collision::check_convex_collision(&lower, &upper, epa: true);
	test::@check(info.collided, "a 1.7 mm overlap between 8 cm boxes read as apart");
	test::@check(info.depth > 0.001f && info.depth < 0.0025f, "the overlap measured %f", info.depth);
}

fn void small_cubes_stack_and_sleep() @test
{
	PhysicsWorld* world = make_world();
	defer free_world(world);
	for (int k = 0; k < 3; k++) add_cube(world, (sz)(10 + k), level_at(k));

	step(world, 600);
	for (int k = 0; k < 3; k++)
	{
		Rigidbody* cube = world.find_body((sz)(10 + k))!!;
		float moved = (cube.collider.translation - level_at(k)).length();
		test::@check(moved < 0.005f, "cube %d of a 3-stack moved %f in 10 s", k, moved);
		test::@check(cube.sleeping, "cube %d of a 3-stack never slept", k);
	}
}

// Welds `upper` on top of `lower`, as a castle's blocks are.
fn void weld(PhysicsWorld* world, sz id, sz lower, sz upper)
{
	GenericJoint3D joint = {
		.id = id, .body_a = lower, .body_b = upper,
		.anchor_a = { 0, SIDE * 0.5f, 0 }, .anchor_b = { 0, -SIDE * 0.5f, 0 },
		.relative_orientation = { 0, 0, 0, 1 },
	};
	joint.linear_limits.push({ .axes = { 1, 0, 0 } });
	joint.linear_limits.push({ .axes = { 0, 1, 0 } });
	joint.linear_limits.push({ .axes = { 0, 0, 1 } });
	joint.angular_limits.push({ .axes = { 1, 0, 0 } });
	joint.angular_limits.push({ .axes = { 0, 1, 0 } });
	joint.angular_limits.push({ .axes = { 0, 0, 1 } });
	world.add_constraint(joint)!!;
}

// A small hop for a sleeping body, which wakes its island.
fn void hop(PhysicsWorld* world, sz id)
{
	Rigidbody* body = world.find_body(id)!!;
	world.wake_island(body);
	body.linear_velocity = { 0, 0.3f, 0 };
}

fn void touching_islands_that_settle_apart_still_sleep() @test
{
	// Two welded pairs side by side, touching: two islands, as a castle's tower
	// and the wall against it are. Hopped a quarter second apart, one came to rest
	// first and fell asleep, and the other, still awake, woke it; then the other
	// fell asleep and was woken in turn, for ever.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	add_cube(world, 10, level_at(0));
	add_cube(world, 11, level_at(1));
	add_cube(world, 12, level_at(0) + (Vec3){ SIDE, 0, 0 });
	add_cube(world, 13, level_at(1) + (Vec3){ SIDE, 0, 0 });
	weld(world, 20, 10, 11);
	weld(world, 21, 12, 13);
	step(world, 120);
	hop(world, 10);
	step(world, 15);
	hop(world, 12);

	step(world, 180);
	int asleep = 0;
	for (int f = 0; f < 60; f++)
	{
		step(world, 1);
		bool all = true;
		for (sz id = 10; id <= 13; id++) all = all && world.find_body(id)!!.sleeping;
		if (all) asleep++;
	}
	test::@check(asleep == 60, "all four asleep on %d of the last 60 frames", asleep);
}

fn void a_hand_placed_stack_stands() @test
{
	// One cube every half second, dropped 1 cm onto the top, a few millimetres
	// off centre and turned a little: a stack built in VR. Before the fixes not
	// even the second cube stayed up, and cubes left at over 2 m/s.
	float[6] dx = { 0.004f, -0.003f, 0.005f, -0.005f, 0.002f, -0.004f };
	float[6] dz = { -0.002f, 0.004f, 0.003f, -0.004f, -0.005f, 0.002f };
	float[6] yaw = { 8, -5, 12, -10, 4, -7 };

	PhysicsWorld* world = make_world();
	defer free_world(world);
	float fastest = 0;
	for (int k = 0; k < 6; k++)
	{
		float half = yaw[k] * (float)math::PI / 360.0f;
		add_cube(world, (sz)(10 + k), level_at(k) + (Vec3){ dx[k], 0.01f, dz[k] }, { 0, math::sin(half), 0, math::cos(half) });
		for (int f = 0; f < 30; f++)
		{
			step(world, 1);
			for (int j = 0; j <= k; j++) fastest = math::max(fastest, world.find_body((sz)(10 + j))!!.linear_velocity.length());
		}
	}
	step(world, 300);

	// A 1 cm drop lands at 0.44 m/s; anything much faster was thrown.
	test::@check(fastest < 0.6f, "a cube of the stack reached %f m/s", fastest);
	for (int k = 0; k < 6; k++)
	{
		Rigidbody* cube = world.find_body((sz)(10 + k))!!;
		float off = math::abs(cube.collider.translation.y - level_at(k).y);
		test::@check(off < 0.005f, "cube %d of 6 ended %f from its level", k, off);
		test::@check(tilt_degrees(cube) < 2, "cube %d of 6 is tilted %f degrees", k, tilt_degrees(cube));
	}
}

fn void a_tilted_box_rocks_flat_on_another() @test
{
	// A metre box set down on another at 8 degrees, resting on its low edge.
	// It falls flat in about a third of a second; standing on points glued
	// under it, it was still at 7 degrees after a second.
	Aabb3 unit = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	PhysicsWorld* world = make_world();
	defer free_world(world);
	world.add_body({ .id = 10, .mass = 1, .gravity_factor = 1,
		.collider = { .shape = &unit, .translation = { 0, 0.5f, 3 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
		.material = { .static_friction = 0.6f, .dynamic_friction = 0.6f } })!!;
	float half = 8 * (float)math::PI / 360.0f;
	// Lifted so the low edge just touches: half a box times (cos + sin) of the tilt.
	float lift = 0.5f * (math::cos(2 * half) + math::sin(2 * half));
	world.add_body({ .id = 11, .mass = 1, .gravity_factor = 1,
		.collider = { .shape = &unit, .translation = { 0, 1 + lift, 3 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, math::sin(half), math::cos(half) } },
		.material = { .static_friction = 0.6f, .dynamic_friction = 0.6f } })!!;

	step(world, 60);
	float tilt = tilt_degrees(world.find_body(11)!!);
	test::@check(tilt < 1, "the box is still tilted %f degrees after a second", tilt);
}

fn void a_contact_inside_the_margin_is_not_a_touch() @test
{
	// 3 mm above the table, held there: close enough for a speculative contact,
	// too far to report one.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	Vec3 at = level_at(0) + (Vec3){ 0, 0.003f, 0 };
	world.add_body({
		.id = 10, .mass = 0.3f, .gravity_factor = 0, .can_sleep = false,
		.collider = { .shape = &CUBE, .translation = at, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
	})!!;

	int starts = 0;
	for (int f = 0; f < 10; f++)
	{
		step(world, 1);
		foreach (event : world.contact_events)
		{
			if (event.type == ContactEventType.START) starts++;
		}
	}
	test::eq(starts, 0);
	float moved = (world.find_body(10)!!.collider.translation - at).length();
	test::@check(moved < 0.0001f, "a contact that never touched moved the cube %f", moved);
}

fn void a_trigger_does_not_reach_across_the_margin() @test
{
	// The broadphase pairs boxes up to the contact margin apart; a trigger
	// reports overlap, so a box 3 mm off it is not inside.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	Aabb3 zone = { { -0.1f, 0, -0.1f }, { 0.1f, 0.2f, 0.1f } };
	world.add_trigger({ .id = 20, .collider = { .shape = &zone, .translation = { 2, 1, 2 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } } })!!;
	world.add_body({
		.id = 10, .mass = 0.3f, .gravity_factor = 0, .can_sleep = false,
		.collider = { .shape = &CUBE, .translation = { 2.143f, 1.1f, 2 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
	})!!;

	step(world, 1);
	test::eq(world.trigger_events.len(), 0);
}

fn void the_margin_can_change_with_bodies_in_the_world() @test
{
	// Bodies are filed in the broadphase under boxes grown by half the margin
	// and taken out under the same boxes. A cube by a cell boundary (cells are 2
	// units) is filed in more cells under a wide margin than a narrow one, so a
	// margin narrowed underneath it would leave cells no removal finds.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	for (int k = 0; k < 3; k++) add_cube(world, (sz)(10 + k), level_at(k));
	world.set_contact_margin(0.2f);
	add_cube(world, 20, { 1.95f, SIDE * 0.5f, 0 });
	step(world, 10);

	world.set_contact_margin(0);
	step(world, 10);
	for (int k = 0; k < 3; k++) world.remove_body((sz)(10 + k));
	world.remove_body(20);
	world.remove_body(1);
	world.remove_body(2);
	test::@check(world.spatial_map.table.len() == 0, "%d broadphase cells outlived the bodies", world.spatial_map.table.len());
	test::@check(world.spatial_map.large.len() == 0, "%d large boxes outlived the bodies", world.spatial_map.large.len());
}

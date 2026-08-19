<*
 `PhysicsWorld` end to end: contacts, CCD, manifolds, joints, the gravity axis,
 and what `remove_body` leaves behind.

	`c3c test --test-filter collision_tests::world`

 This is where the leaks live if there are any. A world owns bodies, joints, an
 island map, a filter set, a trigger set, a contact set, a broadphase and a
 thread pool, and `add_body`/`add_constraint`/`remove_body` each touch several
 of them. `test_remove_body_leaves_nothing_behind` states the ownership claim
 as an assertion — every one of those containers is empty after a hundred
 bodies and ninety-nine joints have come and gone — and the leak-tracked run
 states the other half of it, that the memory behind them went back too.

 Neither check is redundant. A container emptied without freeing its storage
 passes the first and fails the second; a `free()` that misses a container
 entirely passes the second, because the world's own free eventually gets it,
 and fails the first.
*>
module collision_tests::world;

import collision;
import collision_testutil;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

fn void test_xpbd_rigid_contact() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	// Static ground plane (large box at z=-1 to z=0)
	Aabb3 ground_shape = { { -50, -50, -1 }, { 50, 50, 0 } };
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground_shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0, // Static
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Dynamic box starting above ground
	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = true,
		}
	)!!;

	// Simulate for 300 steps (5 seconds at 60fps)
	for (int i = 0; i < 300; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 20);
	}

	// Box should have settled on ground — z position should be near 0.5 (half-height above ground)
	Rigidbody* box = world.find_body(2)!!;
	float z = box.collider.translation.z;

	test::@check(z > -0.1f); // Not fallen through ground
	test::@check(z < 2.0f); // Has fallen from initial position of 5
}

fn void test_world_thread_count() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.thread_count = 2;
	defer world.free();

	Aabb3 box_shape = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
		}
	)!!;

	world.run_step(1.0f / 60.0f);
	test::@check(world.thread_pool.workers() == 2);

	// Default (thread_count = 0) resolves to the machine's core count
	test::@check(collision::default_worker_count() >= 1);
}

fn void test_ccd_fast_projectile() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Small, very fast sphere: moves 2.5 units per step — far more than its
	// 0.1 radius — and would pass straight through the ground without CCD.
	Sphere bullet = { .center = { 0, 0, 0 }, .radius = 0.1f };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &bullet,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0.1f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
			.linear_velocity = { 0, 0, -150.0f },
		}
	)!!;

	for (int i = 0; i < 60; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 8);
	}

	// Projectile must have been stopped by the ground, not tunneled through
	float z = world.find_body(2)!!.collider.translation.z;
	test::@check(z > -0.5f);
	test::@check(z < 1.0f);
}

fn void test_mesh_contact_manifold() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Mesh ground = collision_testutil::create_test_ground_mesh();
	defer ground.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &ground,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Box dropped across the shared edge of the two triangles — needs the
	// multi-contact manifold to settle flat instead of rocking or sinking.
	Aabb3 box_shape = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 2 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0f,
			.gravity_factor = 1.0f,
			.can_sleep = false,
			.material = { .static_friction = 0.5f, .dynamic_friction = 0.4f },
		}
	)!!;

	for (int i = 0; i < 180; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 8);
	}

	// Box should rest on the surface: center near half its height
	float z = world.find_body(2)!!.collider.translation.z;
	test::@check(z > 0.3f);
	test::@check(z < 0.8f);
}

fn void test_xpbd_joint() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

	// Body A — static anchor
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	// Body B — dynamic, connected by joint
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 2, 0, 5 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = false,
		}
	)!!;

	// Joint connecting body_a anchor to body_b anchor
	collision::GenericJoint3D joint = {
		.body_a = 1,
		.body_b = 2,
		.anchor_a = { 1, 0, 0 }, // Right edge of A
		.anchor_b = { -1, 0, 0 }, // Left edge of B
		.relative_orientation = { 0, 0, 0, 1 },
		.collides = false,
	};
	joint.linear_limits.push({ .axes = { 1, 0, 0 }, .stiffness = 100 });
	joint.linear_limits.push({ .axes = { 0, 1, 0 }, .stiffness = 100 });
	joint.linear_limits.push({ .axes = { 0, 0, 1 }, .stiffness = 100 });
	world.add_constraint(joint)!!;

	// Record initial distance between anchor points
	float initial_dist = 2.0f; // |{2,0,5} - {0,0,5}| = 2, anchors are at x=1 and x=-1+2=1

	// Simulate
	for (int i = 0; i < 200; i++)
	{
		world.run_step(1.0f / 60.0f, step_count: 20);
	}

	// Bodies should still be roughly connected — check distance between anchor points
	Rigidbody* body_a = world.find_body(1)!!;
	Rigidbody* body_b = world.find_body(2)!!;

	Vec3 anchor_world_a = body_a.collider.translation + body_a.collider.rotation * (Vec3) { 1, 0, 0 };
	Vec3 anchor_world_b = body_b.collider.translation + body_b.collider.rotation * (Vec3) { -1, 0, 0 };

	float dist = (anchor_world_b - anchor_world_a).length();

	// Joint should keep bodies connected (allow some tolerance for XPBD softness)
	test::@check(dist < 2.0f); // Should be much closer than free-falling apart

	// Body B should have moved downward from gravity
	test::@check(body_b.collider.translation.z < 5.0f);
}

fn void test_capsule_rests_on_box() @test
{
	// z-up world; the capsule axis (+y) is horizontal, so it rests on its side
	// at ground_top + radius.
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 ground_shape = collision::aabb_from_half({ 10, 10, 1 });
	world.add_body(body: {
		.id = 1,
		.collider = { .shape = &ground_shape, .translation = { 0, 0, -0.5f }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	Capsule capsule_shape = { .height = 2, .radius_top = 0.5f, .radius_bottom = 0.5f };
	world.add_body(body: {
		.id = 2,
		.mass = 1, .gravity_factor = 1,
		.collider = { .shape = &capsule_shape, .translation = { 0, 0, 1 }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	for (int i = 0; i < 240; i++) world.run_step(1.0f / 60.0f, step_count: 4);

	Rigidbody* capsule_body = world.find_body(2)!!;
	test::@check(math::abs(capsule_body.collider.translation.z - 0.5f) < 0.05f,
		"capsule should rest at ground + radius, got z=%f", capsule_body.collider.translation.z);
}

fn void test_box_rests_on_thin_box() @test
{
	// Paper-thin static slab (like glTF exporter ground planes): a dropped box
	// must neither tunnel through nor get ejected.
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 slab_shape = collision::aabb_from_half({ 15, 15, 0.001f });
	world.add_body(body: {
		.id = 1,
		.collider = { .shape = &slab_shape, .translation = { 0, 0, 0 }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	Aabb3 box_shape = collision::aabb_from_half({ 1, 1, 1 });
	world.add_body(body: {
		.id = 2,
		.mass = 1, .gravity_factor = 1,
		.collider = { .shape = &box_shape, .translation = { 0, 0, 1.2f }, .rotation = { 0, 0, 0, 1 }, .scale = { 1, 1, 1 } },
		.material = { .static_friction = 0.5f, .dynamic_friction = 0.5f, .restitution = 0 },
	})!!;

	for (int i = 0; i < 300; i++) world.run_step(1.0f / 60.0f, step_count: 4);

	Rigidbody* box_body = world.find_body(2)!!;
	test::@check(box_body.collider.translation.z > 0.4f && box_body.collider.translation.z < 0.6f,
		"box should rest on the thin slab, got z=%f", box_body.collider.translation.z);
}

// ---------------------------------------------------------------------------
// The axis, and what it costs to move it — `game.md` G5/S2, S2a, S2b.
//
// The claim these hold up is that gravity's direction is a *setting* and not a
// convention: nothing in the solver reads a fixed component of it any more.

// Runs a box onto a floor and answers with the rest position measured along
// `up`. A macro because a fn in a `@test` module may take no parameters.
macro float @box_rest_along(gravity, floor_shape, box_shape, start, up)
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.gravity = gravity;
	defer world.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = floor_shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = box_shape,
				.translation = start,
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = false,
		}
	)!!;

	for (int i = 0; i < 300; i++) world.run_step(1.0f / 60.0f, 1, 4);

	return world.find_body(2)!!.collider.translation.dot(up);
}

// The whole scene rotated so that down is -y instead of -z. The rest height
// above the floor is the physics; the axis it happens to be measured along is
// not. A suite that passes at one orientation and fails at the other is testing
// the orientation, which is the cheapest possible thing to find out.
fn void test_gravity_is_axis_agnostic() @test
{
	// An untyped macro parameter cannot take a brace list, so the vectors are
	// named rather than written at the call.
	Aabb3 floor_z = { { -50, -50, -1 }, { 50, 50, 0 } };
	Aabb3 box_z = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	Vec3 gravity_z = { 0, 0, -9.8f };
	Vec3 start_z = { 0, 0, 5 };
	Vec3 up_z = { 0, 0, 1 };
	float rest_z = @box_rest_along(gravity_z, &floor_z, &box_z, start_z, up_z);

	Aabb3 floor_y = { { -50, -1, -50 }, { 50, 0, 50 } };
	Aabb3 box_y = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	Vec3 gravity_y = { 0, -9.8f, 0 };
	Vec3 start_y = { 0, 5, 0 };
	Vec3 up_y = { 0, 1, 0 };
	float rest_y = @box_rest_along(gravity_y, &floor_y, &box_y, start_y, up_y);

	test::@check(math::abs(rest_z - rest_y) < 0.01f);
	// And it actually came to rest on the floor rather than agreeing at
	// whatever height two identically broken runs both fell to.
	test::@check(math::abs(rest_z - 0.5f) < 0.1f);
}

macro int @impulse_dir_for(gravity, factor, shape)
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	world.gravity = gravity;
	defer world.free();

	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = factor,
		}
	)!!;

	return world.find_body(1)!!.impulse_dir;
}

fn void test_impulse_dir_follows_gravity() @test
{
	Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	Vec3 z_down = { 0, 0, -9.8f };
	Vec3 y_down = { 0, -9.8f, 0 };

	// The z-down default, unchanged. This is exactly what the old `.z > 0`
	// form produced for every factor, zero included — which is what makes the
	// change a restatement rather than a new behaviour.
	test::@check(@impulse_dir_for(z_down, 1.0f, &shape) == -1);
	test::@check(@impulse_dir_for(z_down, 0.0f, &shape) == -1);
	test::@check(@impulse_dir_for(z_down, -1.0f, &shape) == 1);

	// Y-down gravity. The component form got the first two right by accident —
	// `.z` is 0, and 0 is not greater than 0. The third is the one it got
	// wrong: a body with inverted gravity under gravity that is not along z.
	test::@check(@impulse_dir_for(y_down, 1.0f, &shape) == -1);
	test::@check(@impulse_dir_for(y_down, 0.0f, &shape) == -1);
	test::@check(@impulse_dir_for(y_down, -1.0f, &shape) == 1);
}

// ---------------------------------------------------------------------------
// `remove_body` — G5/S5. Written as a cycle for the reason G2's unload test is
// a cycle: a leak in any one of these four is invisible until a long-running
// game slows down for no reason anybody can point at.
fn void test_remove_body_leaves_nothing_behind() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

	for (sz i = 1; i <= 100; i++)
	{
		world.add_body(
			{
				.id = i,
				.collider = {
					.shape = &shape,
					.translation = { (float)i * 1.5f, 0, 0 },
					.scale = { 1, 1, 1 },
					.rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0,
				.gravity_factor = 1.0,
				.can_sleep = false,
			}
		)!!;
	}

	// Joints as well, so the island map and the filter set are not empty to
	// begin with — an assertion that they end empty proves nothing otherwise.
	for (sz i = 1; i < 100; i++)
	{
		world.add_constraint({ .body_a = i, .body_b = i + 1, .collides = false })!!;
	}

	world.run_step(1.0f / 60.0f, 1, 1);
	test::@check(world.bodies.len() == 100, "%d of the 100 bodies were added", world.bodies.len());
	test::@check(world.island_parent.len() > 0, "the step left no islands, so emptying them proves nothing");

	for (sz i = 1; i <= 100; i++) test::@check(world.remove_body(i), "body %d would not remove", i);

	test::@check(world.bodies.len() == 0, "%d bodies outlived their removal", world.bodies.len());
	test::@check(world.joints.len() == 0, "%d joints still name a body that is gone", world.joints.len());
	test::@check(world.filters.len() == 0, "%d collision filters outlived their joints", world.filters.len());
	test::@check(world.island_parent.len() == 0, "%d island entries outlived their bodies", world.island_parent.len());
	test::@check(world.active_triggers.len() == 0, "%d trigger overlaps outlived their bodies", world.active_triggers.len());
	test::@check(world.active_contacts.len() == 0, "%d contact overlaps outlived their bodies", world.active_contacts.len());
	// The one most likely to be forgotten, and the one whose cost is a
	// broadphase that never stops growing.
	test::@check(world.spatial_map.table.len() == 0, "%d broadphase cells still hold a dead id", world.spatial_map.table.len());

	// Removing what is not there is an answer, not an error.
	test::@check(!world.remove_body(1), "removing a body twice reported success");
}

// A body destroyed inside a trigger must still be seen to leave it.
fn void test_remove_body_emits_exit_events() @test
{
	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
	defer world.free();

	Aabb3 trigger_shape = { { -2, -2, -2 }, { 2, 2, 2 } };
	Aabb3 box_shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

	world.add_trigger(
		{
			.id = 1,
			.collider = {
				.shape = &trigger_shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
		}
	)!!;
	world.add_body(
		{
			.id = 2,
			.collider = {
				.shape = &box_shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 0,
			.can_sleep = false,
		}
	)!!;

	world.run_step(1.0f / 60.0f, 1, 1);
	test::@check(world.active_triggers.len() == 1, "%d trigger overlaps after the step, expected 1", world.active_triggers.len());

	world.remove_body(2);

	int exits = 0;
	world.@get_trigger_events(; TriggerEvent event)
	{
		if (event.type == TriggerEventType.EXIT && event.other_id == 2) exits++;
	};

	test::@check(exits == 1, "%d EXIT events for a body destroyed inside a trigger, expected 1", exits);
	test::@check(world.active_triggers.len() == 0, "%d trigger overlaps outlived the removed body", world.active_triggers.len());
}

<*
 Allocate and give back, in cycles, with every container explicitly seated on
 the heap so the runner can see it.

	`c3c test --test-filter collision_tests::lifetime`

 ## Why this file exists at all

 The runner's leak check swaps `allocator::thread_allocator` — what `mem`
 resolves to — for a `TrackingAllocator` around each `@test` fn, and reports
 whatever is still outstanding when the fn returns. It therefore sees exactly
 the allocations that go through `mem`, and nothing else.

 **Most of this library's containers do not go through `mem` unless somebody
 says so.** A zero-valued `List` or `HashMap` binds itself to `tmem` — the temp
 allocator — on its first growth (`list.c3`'s `list_ensure_capacity`,
 `hashmap.c3`'s `set`). `PhysicsWorld` is normally written as

	PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;

 which is a const struct of zeroes with a few settings in it, so every one of
 its nineteen containers takes that path. The rest of the suite is written that
 way, and under `c3c test` it passes leak-clean while telling you almost
 nothing: the temp pool the runner wraps each test in swallows the lot and
 releases it at the end regardless of whether the library gave it back.

 So each test below calls `init(mem)` — or the constructor that takes an
 allocator — before anything is added, and only then cycles. That is the whole
 difference between a leak check that runs and a leak check that counts.

 It is also what a host has to do for its own reasons: a world built in a frame
 scope and stepped in the next one reads a pool that has been released, which
 surfaces as an unwrapped `NOT_FOUND` deep inside the solver rather than
 anywhere near the cause. `PhysicsWorld.init`, `SpatialHash3D.cell` and
 `run_step`'s overlap sets all say so in their own comments.

 ## What a cycle catches that a single pass does not

 Every test here runs its build/tear-down many times over. One pass proves the
 total is zero at the end; a hundred proves nothing accumulates in between —
 which is the shape of the leak that matters, because a frame that keeps one
 cell of a hash table is invisible for an afternoon and fatal by the evening.

 ## What this cost, and what it bought

 An audit of `src/` found no allocation without a matching free — every `free`
 walks everything its type owns. What it did find was four places where storage
 that outlives its call bound itself to `tmem` because nobody named an
 allocator, each one a blind spot in the leak check as well as a dangling
 pointer in a host. All four now take an allocator, defaulting to `mem`:

 - `create_softbody_from_mesh`, which seated none of the body's fourteen lists
   where `create_particle_chain` seated all of them. Both now call
   `SoftBody.init`, which exists as one function precisely because it was once
   two copies and only one of them got fixed.
 - `Mesh`, which had no `init` at all, so `create_mesh`, `quickhull` and every
   `Mesh mesh;` in this suite bound on first push — and `build_bvh` stores
   `vertices.entries` in every node of the tree, so the tree pointed into
   whatever the list happened to bind to.
 - `PhysicsWorld.remove_body`, which rebuilt `active_triggers` and
   `active_contacts` into zero-valued maps where `run_step` seats the
   equivalents and says in a comment why it must. `restore` re-filled them the
   same way.
 - `PhysicsWorld.snapshot`, which seated none of its four lists nor the two
   inside each soft-body entry — the one structure most likely to be taken in
   a frame scope and read outside one, since that is what rollback is.

 The difference is measurable: three deliberately-leaked objects — a soft body
 from a mesh, a hull, and a snapshot — passed leak-clean before those fixes and
 fail with a report after them.
*>
module collision_tests::lifetime;

import collision;
import collision_testutil;
import std::core::test;

alias Vec3 = float[<3>];

<*
 `create_world` is the one-call form, and the two things it has to get right are
 that the storage is seated and that the settings survive being copied through
 it. A `create_world` that quietly reset `gravity` or `cell_size` to the
 constant's values would pass every other test in this suite.
*>
fn void test_create_world_seats_storage_and_keeps_the_config() @test
{
	PhysicsWorld config = collision::DEFAULT_PHYSICS_WORLD;
	config.gravity = { 0, -9.8f, 0 };
	config.spatial_map.cell_size = 3.5f;
	config.thread_count = 2;
	config.ccd_enabled = false;

	PhysicsWorld world = collision::create_world(config);
	defer world.free();

	test::@check(world.allocator.ptr == mem.ptr, "create_world did not seat the storage");
	test::@check(world.spatial_map.allocator.ptr == mem.ptr, "create_world did not seat the broadphase");
	test::@check(world.bodies.is_initialized(), "the body map came back uninitialized");

	test::@check(world.gravity == (Vec3){ 0, -9.8f, 0 }, "gravity came through as %s", world.gravity);
	test::@check(world.spatial_map.cell_size == 3.5f, "cell_size came through as %f", world.spatial_map.cell_size);
	test::@check(world.thread_count == 2, "thread_count came through as %d", world.thread_count);
	test::@check(!world.ccd_enabled, "ccd_enabled came through set");

	// And the default form is the constant's settings, seated.
	PhysicsWorld plain = collision::create_world();
	defer plain.free();
	test::@check(plain.gravity == collision::DEFAULT_PHYSICS_WORLD.gravity,
		"the no-argument form did not take the default gravity: %s", plain.gravity);
	test::@check(plain.spatial_map.cell_size == collision::DEFAULT_PHYSICS_WORLD.spatial_map.cell_size,
		"the no-argument form did not take the default cell_size: %f", plain.spatial_map.cell_size);
	test::@check(plain.allocator.ptr == mem.ptr, "the no-argument form did not seat the storage");
}

// The claim `PhysicsWorld.init` makes: it seats the storage, and `free` gives
// all of it back. Both halves are checked here — the binding directly, and the
// return by the runner.
fn void test_world_init_seats_every_container() @test
{
	PhysicsWorld world = collision::create_world();
	defer world.free();

	test::@check(world.allocator.ptr == mem.ptr, "the world kept an allocator that is not the one it was given");
	test::@check(world.spatial_map.allocator.ptr == mem.ptr, "the broadphase did not take the world's allocator");

	Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	world.add_body(
		{
			.id = 1,
			.collider = {
				.shape = &shape,
				.translation = { 0, 0, 0 },
				.scale = { 1, 1, 1 },
				.rotation = { 0, 0, 0, 1 },
			},
			.mass = 1.0,
			.gravity_factor = 1.0,
			.can_sleep = false,
		}
	)!!;

	// A list that grew after `init` is still on the allocator `init` gave it —
	// which is the thing a lazily-bound container silently is not.
	test::@check(world.bodies.allocator.ptr == mem.ptr, "the body map re-bound itself on first insert");
}

<*
 A hundred worlds built, filled, stepped, emptied and freed.

 Everything a world owns is in play: bodies, joints and the collision filters
 that come with them, the island union-find, the broadphase, the trigger and
 contact overlap sets, and the task lists the parallel solve reuses. A step is
 run before the teardown so that none of them are empty when it starts —
 tearing down what was never built proves nothing.
*>
fn void test_world_cycle_returns_everything() @test
{
	for (int cycle = 0; cycle < 100; cycle++)
	{
		PhysicsWorld world = collision::create_world();

		Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

		for (sz i = 1; i <= 12; i++)
		{
			world.add_body(
				{
					.id = i,
					.collider = {
						.shape = &shape,
						.translation = { (float)i * 0.9f, 0, 1 },
						.scale = { 1, 1, 1 },
						.rotation = { 0, 0, 0, 1 },
					},
					.mass = 1.0,
					.gravity_factor = 1.0,
					.can_sleep = false,
				}
			)!!;
		}

		// Joints, so `joints` and `filters` are not empty either.
		for (sz i = 1; i < 12; i++)
		{
			world.add_constraint({ .body_a = i, .body_b = i + 1, .collides = false })!!;
		}

		world.run_step(1.0f / 60.0f, 1, 1);
		world.free();
	}
}

<*
 The same cycle taken apart from the other end: every body removed one at a
 time before the world is freed, so `remove_body` does the work rather than
 `free`.

 Both orders have to give the same memory back. A `free` that mops up after a
 `remove_body` which forgot something looks identical to one that had nothing
 to mop up, and only the removal order tells them apart.
*>
fn void test_remove_body_cycle_returns_everything() @test
{
	for (int cycle = 0; cycle < 50; cycle++)
	{
		PhysicsWorld world = collision::create_world();

		Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };

		for (sz i = 1; i <= 12; i++)
		{
			world.add_body(
				{
					.id = i,
					.collider = {
						.shape = &shape,
						.translation = { (float)i * 0.9f, 0, 1 },
						.scale = { 1, 1, 1 },
						.rotation = { 0, 0, 0, 1 },
					},
					.mass = 1.0,
					.gravity_factor = 1.0,
					.can_sleep = false,
				}
			)!!;
		}
		for (sz i = 1; i < 12; i++)
		{
			world.add_constraint({ .body_a = i, .body_b = i + 1, .collides = false })!!;
		}

		world.run_step(1.0f / 60.0f, 1, 1);

		for (sz i = 1; i <= 12; i++) world.remove_body(i);

		// The broadphase is the one that grows a bucket per cell and has to be
		// asked to give them back — `remove_body` is the only caller of
		// `SpatialHash3D.remove` there is.
		test::@check(world.spatial_map.table.len() == 0,
			"%d broadphase cells outlived the bodies that made them", world.spatial_map.table.len());

		world.free();
	}
}

<*
 Snapshots, taken and freed in a cycle.

 A `WorldSnapshot` owns four lists of its own, and each soft body in it owns two
 more — nested ownership, which is where a `free` that walks the outer list and
 forgets the inner one hides. Rollback is also the operation most likely to be
 run every frame by a netcode host, so a snapshot that keeps one list is a leak
 measured in frames.
*>
fn void test_snapshot_cycle_returns_everything() @test
{
	PhysicsWorld world = collision::create_world();
	defer world.free();

	Aabb3 shape = { { -0.5, -0.5, -0.5 }, { 0.5, 0.5, 0.5 } };
	for (sz i = 1; i <= 8; i++)
	{
		world.add_body(
			{
				.id = i,
				.collider = {
					.shape = &shape,
					.translation = { (float)i * 0.9f, 0, 1 },
					.scale = { 1, 1, 1 },
					.rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0,
				.gravity_factor = 1.0,
				.can_sleep = false,
			}
		)!!;
	}

	Vec3[4] points = { { 0, 0, 3 }, { 0, 0, 2.5f }, { 0, 0, 2.0f }, { 0, 0, 1.5f } };
	float[4] weights = { 1, 1, 1, 1 };
	world.add_softbody(collision::create_particle_chain(mem, points[..], weights[..], 99));

	for (int cycle = 0; cycle < 100; cycle++)
	{
		world.run_step(1.0f / 60.0f, 1, 1);
		WorldSnapshot snap = world.snapshot();
		world.restore(&snap)!!;
		snap.free();
	}
}

<*
 A chain built and freed a hundred times, on `mem` because that is the argument
 `create_particle_chain` takes.

 Fourteen lists per body, and the host fills four of them after construction —
 pins, targets, swings and the per-substep collision constraints — so this is
 also the check that the four filled late are freed like the ten filled early.
*>
fn void test_particle_chain_cycle_returns_everything() @test
{
	Vec3[4] points = { { 0, 0, 0 }, { 0, -0.5f, 0 }, { 0, -1.0f, 0 }, { 0, -1.5f, 0 } };
	float[4] weights = { 1, 1, 1, 1 };

	Vec3[1] seg_a = { { 0.35f, -0.9f, 0 } };
	Vec3[1] seg_b = { { 0.35f, -0.2f, 0 } };
	float[1] radii = { 0.25f };

	for (int cycle = 0; cycle < 100; cycle++)
	{
		SoftBody sb = collision::create_particle_chain(mem, points[..], weights[..], (sz)cycle);
		sb.substeps = 1;
		sb.pin_particle(0, points[0]);

		// One per particle: both lists are read with a bounds guard, so a short
		// one is silently ignored rather than caught.
		for (sz i = 0; i < 4; i++)
		{
			sb.particle_damping.push(0.2f);
			sb.particle_radius.push(0.05f);
		}
		for (sz i = 1; i < 4; i++)
		{
			sb.add_target(i, points[i], 0.0005f);
			sb.add_swing(i - 1, i, { 0, -1, 0 }, 0.7f, 0.0f);
		}

		sb.generate_capsule_collisions(seg_a[..], seg_b[..], radii[..]);
		sb.xpbd_solve({ 0, -9.81f, 0 }, 1.0f / 120.0f);

		sb.free();
	}
}

<*
 Meshes with a BVH on them, built and freed in a cycle.

 `Mesh.free` has three things to give back and one of them takes two calls:
 `free_indexed()` releases the sorted-index array the tree shares, and
 `mem::free()` releases the root node the builder returned. A tree freed without
 its root leaks one node per mesh, which is small enough to survive a long time
 and large enough to matter in a level with a thousand colliders.
*>
fn void test_mesh_bvh_cycle_returns_everything() @test
{
	for (int cycle = 0; cycle < 200; cycle++)
	{
		// A cube rather than the ground fixture: that one builds its tree in the
		// constructor, and this test is about `build_bvh` being freed, so it
		// has to be the caller that builds it.
		Mesh mesh = collision_testutil::create_test_cube();
		mesh.build_bvh();
		test::@check(mesh.has_bvh(), "the mesh did not build a tree to free");
		mesh.free();
	}
}

<*
 The broadphase on its own: inserted, moved across cells, emptied by `remove`,
 and freed — repeatedly.

 `remove` is the newest of the four and the one with a free in it: a cell whose
 last id leaves is freed and dropped from the table, rather than left as an
 empty list nobody will ever look in again. The `table.len() == 0` check is what
 says the drop happened; the leak check is what says the list went with it.
*>
fn void test_spatial_hash_cycle_returns_everything() @test
{
	for (int cycle = 0; cycle < 50; cycle++)
	{
		SpatialHash3D map = { .cell_size = 1.0 };
		map.init(mem);

		Aabb3[32] boxes;
		for (sz i = 0; i < 32; i++)
		{
			float x = (float)i * 1.5f;
			boxes[i] = { { x, 0, 0 }, { x + 0.9f, 0.9f, 0.9f } };
			map.insert(boxes[i], i);
		}

		// Moved far enough to change cells, which is the path that frees the
		// vacated ones.
		for (sz i = 0; i < 32; i++)
		{
			Aabb3 moved = { { boxes[i].min.x, 5, 5 }, { boxes[i].max.x, 5.9f, 5.9f } };
			map.update(boxes[i], moved, i);
			boxes[i] = moved;
		}

		map.@get_pairs(; Pair pair) {};

		for (sz i = 0; i < 32; i++) map.remove(boxes[i], i);

		test::@check(map.table.len() == 0, "%d cells survived every id leaving them", map.table.len());

		map.free();
	}
}

<*
 `remove_body` rebuilds `active_triggers` and `active_contacts` rather than
 erasing from them mid-walk, and the rebuilt pair has to be seated where the
 old one was.

 This file's header already names `remove_body` among the four places that had
 to take an allocator, and `restore`'s comment says `remove_body` seats its
 overlap sets for the same reason `run_step` does. Neither was true of the code:
 the two replacements went in zero-valued, bound themselves to the temp pool on
 the first surviving pair, and were read from a released pool by the next step's
 broadphase. It reached a host as an out-of-bounds read inside
 `HashMap.get_ref`, several frames after the removal that caused it.

 Nothing above catches it, because catching it needs a *survivor*. The two
 `remove_body` tests in `world_test` each remove the only pair there is, so the
 replacements are never written to, never bind to anything, and stay honest by
 never being used. This keeps two pairs alive across the removal.

 The binding is checked before anything reads the table on purpose: a map that
 bound itself to the pool is a probe loop over freed memory with no empty slot
 to terminate at, so a test that only stepped and looked at the answer would
 hang the suite instead of failing it.
*>
fn void test_remove_body_seats_the_sets_it_rebuilds() @test
{
	PhysicsWorld world = collision::create_world();
	defer world.free();

	Aabb3 trigger_shape = { { -4, -4, -4 }, { 4, 4, 4 } };
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

	// Three inside the trigger and far enough apart to touch nothing else, so
	// every overlap is a trigger pair and none of them settles or sleeps.
	for (sz i = 2; i <= 4; i++)
	{
		world.add_body(
			{
				.id = i,
				.collider = {
					.shape = &box_shape,
					.translation = { (float)(i - 3) * 2.0f, 0, 0 },
					.scale = { 1, 1, 1 },
					.rotation = { 0, 0, 0, 1 },
				},
				.mass = 1.0,
				.gravity_factor = 0,
				.can_sleep = false,
			}
		)!!;
	}

	world.run_step(1.0f / 60.0f, 1, 1);
	test::@check(world.active_triggers.len() == 3,
		"the three bodies did not all overlap the trigger: %d", world.active_triggers.len());

	// The removal, and nothing else, inside a scope that takes the temp pool
	// away on the way out — so a map that bound itself there is definitively
	// gone rather than merely unlucky.
	@pool()
	{
		test::@check(world.remove_body(3), "the middle body would not remove");
	};

	test::@check(world.active_triggers.len() == 2,
		"the survivors did not survive: %d left", world.active_triggers.len());

	test::@check(world.active_triggers.allocator.ptr == mem.ptr,
		"remove_body rebuilt active_triggers into a map it does not own");
	test::@check(world.active_contacts.allocator.ptr == mem.ptr,
		"remove_body rebuilt active_contacts into a map it does not own");

	// And the survivors read back — `run_substep` asks `has_key` of this set
	// for every overlap it finds, which is where the host crashed.
	for (int i = 0; i < 4; i++) world.run_step(1.0f / 60.0f, 1, 1);

	int survivors = 0;
	world.active_triggers.@each(; Pair pair, bool active)
	{
		test::@check(pair.a != 3 && pair.b != 3, "a removed body is still in the overlap set");
		survivors++;
	};
	test::@check(survivors == 2, "the surviving pairs did not survive the steps: %d", survivors);
}

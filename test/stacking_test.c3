<*
 Resting contact: boxes left alone must stay where they were put. Each scene
 steps the way a 60 Hz host does (`run_step(1/60, 1, 4)`) on one thread, so the
 numbers are deterministic.

 These guard three separate failures:
 - per-point friction ordering, which let a lone box creep across a flat floor;
 - friction that only cancelled one substep's slip, so stacks and boxes on a
   slope crept by the remainder until they fell;
 - EPA picking a full-width face for boxes that touch exactly, which launched
   the top of a freshly spawned stack.
*>
module collision_tests::stacking;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

const Aabb3 UNIT_BOX = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
const Aabb3 FLOOR = { { -20, -1, -20 }, { 20, 0, 20 } };

struct Scene
{
	PhysicsWorld world;
	Vec3[10] start;
	int count;
}

<*
 A y-up world with a 40 x 40 floor tilted `slope_deg` about z, and `slots`
 boxes placed in the floor's frame (so they sit on the slope as on flat ground).
*>
fn Scene* make_scene(Vec3[] slots, float slope_deg, bool can_sleep)
{
	Scene* scene = mem::new(Scene);
	scene.world = collision::DEFAULT_PHYSICS_WORLD;
	scene.world.gravity = { 0, -9.81f, 0 };
	scene.world.thread_count = 1;

	float half = slope_deg * (float)math::PI / 360.0f;
	QuaternionValue tilt = { 0, 0, math::sin(half), math::cos(half) };
	scene.world.add_body(
		{
			.id = 1,
			.collider = { .shape = &FLOOR, .scale = { 1, 1, 1 }, .rotation = tilt },
			.material = { .static_friction = 0.7f, .dynamic_friction = 0.7f, .restitution = 0.05f },
		}
	)!!;

	scene.count = (int)slots.len;
	foreach (k, slot : slots)
	{
		Vec3 at = tilt * slot;
		scene.start[k] = at;
		scene.world.add_body(
			{
				.id = (sz)(10 + k),
				.mass = 1,
				.gravity_factor = 1,
				.can_sleep = can_sleep,
				.collider = { .shape = &UNIT_BOX, .translation = at, .scale = { 1, 1, 1 }, .rotation = tilt },
				.material = { .static_friction = 0.6f, .dynamic_friction = 0.6f, .restitution = 0.05f },
			}
		)!!;
	}
	return scene;
}

fn void Scene.free(&self)
{
	self.world.free();
	mem::free(self);
}

fn void Scene.step(&self, int frames)
{
	for (int i = 0; i < frames; i++) self.world.run_step(1.0f / 60.0f, 1, 4);
}

fn Rigidbody* Scene.box(&self, int k) => self.world.find_body((sz)(10 + k))!!;

// The furthest any box has moved from where it was placed.
fn float Scene.max_offset(&self)
{
	float worst = 0;
	for (int k = 0; k < self.count; k++)
	{
		worst = math::max(worst, (self.box(k).collider.translation - self.start[k]).length());
	}
	return worst;
}

fn int Scene.asleep(&self)
{
	int n = 0;
	for (int k = 0; k < self.count; k++)
	{
		if (self.box(k).sleeping) n++;
	}
	return n;
}

// The pyramid from three.c3's examples/alley.js: rows of 4, 3, 2, 1, each row
// straddling the gaps of the one below.
fn Vec3[10] alley_slots()
{
	Vec3[10] slots;
	int n = 0;
	for (int row = 0; row < 4; row++)
	{
		for (int i = 0; i < 4 - row; i++) slots[n++] = { -1.53f + row * 0.51f + i * 1.02f, 0.5f + row * 1.01f, -3 };
	}
	return slots;
}

fn void lone_box_does_not_creep_on_a_flat_floor() @test
{
	Vec3[1] slots = { { 0, 0.5f, 0 } };
	Scene* scene = make_scene(&slots, 0, can_sleep: false);
	defer scene.free();

	scene.step(1200);
	test::@check(scene.max_offset() < 0.001f, "a lone box crept %f in 20 s", scene.max_offset());
}

fn void alley_pyramid_settles_and_sleeps() @test
{
	Vec3[10] slots = alley_slots();
	Scene* scene = make_scene(&slots, 0, can_sleep: true);
	defer scene.free();

	scene.step(120);
	test::@check(scene.asleep() == 10, "only %d of 10 crates asleep after 2 s", scene.asleep());
	test::@check(scene.max_offset() < 0.05f, "the pyramid moved %f while settling", scene.max_offset());
}

fn void alley_pyramid_holds_while_awake() @test
{
	Vec3[10] slots = alley_slots();
	Scene* scene = make_scene(&slots, 0, can_sleep: false);
	defer scene.free();

	// It settles by about 10 cm and stays there. Before friction anchors it crept
	// apart and collapsed (3.8 m by 20 s).
	scene.step(1200);
	test::@check(scene.max_offset() < 0.15f, "an awake pyramid drifted %f in 20 s", scene.max_offset());
}

fn void lone_box_holds_on_a_slope() @test
{
	Vec3[1] slots = { { 0, 0.5f, 0 } };
	Scene* scene = make_scene(&slots, 15, can_sleep: false);
	defer scene.free();

	scene.step(1200);
	test::@check(scene.max_offset() < 0.005f, "a box on 15 degrees slid %f in 20 s", scene.max_offset());
}

fn void tower_stands_on_a_slope() @test
{
	// Friction alone holds this: before anchors the boxes crept apart and it fell
	// at 8 degrees. Steeper towers (10+ degrees, still under the 18.4 degree
	// tipping angle of a 1 x 3 stack) slowly lean over, which is the one-iteration
	// normal solve, not friction.
	Vec3[3] slots = { { 0, 0.5f, 0 }, { 0, 1.51f, 0 }, { 0, 2.52f, 0 } };
	Scene* scene = make_scene(&slots, 8, can_sleep: false);
	defer scene.free();

	scene.step(1200);
	test::@check(scene.max_offset() < 0.05f, "a 3-box tower on 8 degrees moved %f in 20 s", scene.max_offset());
}

fn void exactly_touching_tower_is_not_launched() @test
{
	// Spawned with no gap at all, as a host stacking crates edge to edge does.
	// 25 degrees is past the tipping angle: it may fall, but not fly.
	Vec3[3] slots = { { 0, 0.5f, 0 }, { 0, 1.5f, 0 }, { 0, 2.5f, 0 } };
	Scene* scene = make_scene(&slots, 25, can_sleep: false);
	defer scene.free();

	float fastest = 0;
	for (int f = 0; f < 60; f++)
	{
		scene.step(1);
		for (int k = 0; k < 3; k++) fastest = math::max(fastest, scene.box(k).linear_velocity.length());
	}
	test::@check(fastest < 5.0f, "a box left the tower at %f m/s", fastest);
}

fn void epa_reports_touching_boxes_as_touching() @test
{
	// Two unit boxes face to face, rotated off the axes: the origin of their
	// Minkowski difference lies on a face, which once gave a full-width depth.
	float half = 25.0f * (float)math::PI / 360.0f;
	QuaternionValue tilt = { 0, 0, math::sin(half), math::cos(half) };
	Aabb3 box = UNIT_BOX;
	TransformedShape lower = { .shape = &box, .translation = tilt * (Vec3){ 0, 0.5f, 0 }, .scale = { 1, 1, 1 }, .rotation = tilt };
	TransformedShape upper = { .shape = &box, .translation = tilt * (Vec3){ 0, 1.5f, 0 }, .scale = { 1, 1, 1 }, .rotation = tilt };

	CollisionInfo info = collision::check_convex_collision(&lower, &upper, epa: true);
	if (!info.collided) return; // Touching may also read as apart; only a wrong depth is a failure
	test::@check(info.depth < 0.01f, "touching boxes reported depth %f", info.depth);
	Vec3 up = tilt * (Vec3){ 0, 1, 0 };
	test::@check(math::abs(info.normal.dot(up)) > 0.99f, "touching boxes reported normal %s", info.normal);
}

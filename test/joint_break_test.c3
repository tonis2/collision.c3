<*
 Joints that measure their load and break past a rating
 (`GenericJoint3D.break_force` / `break_torque`).

 A weight hanging still carries exactly its weight, so these check the
 measurement against m * g and m * g * d, and that it does not change with the
 substep count, before trusting it to break anything.

	`c3c test --test-filter collision_tests::joint_break`
*>
module collision_tests::joint_break;

import collision;
import std::core::test;
import std::math;

alias Vec3 = float[<3>];

const Aabb3 ANCHOR = { { -0.5f, -0.5f, -0.5f }, { 0.5f, 0.5f, 0.5f } };
const Aabb3 WEIGHT = { { -0.25f, -0.25f, -0.25f }, { 0.25f, 0.25f, 0.25f } };
// A beam a metre long, its end at its local origin's -x side.
const Aabb3 BEAM = { { -0.5f, -0.05f, -0.05f }, { 0.5f, 0.05f, 0.05f } };
const float G = 9.8f;

fn PhysicsWorld* make_world()
{
	PhysicsWorld* world = mem::new(PhysicsWorld);
	*world = collision::DEFAULT_PHYSICS_WORLD;
	world.init(mem);
	world.gravity = { 0, -G, 0 };
	world.thread_count = 1;
	world.add_body({ .id = 1, .collider = { .shape = &ANCHOR, .translation = { 0, 5, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } } })!!;
	return world;
}

fn void free_world(PhysicsWorld* world)
{
	world.free();
	mem::free(world);
}

// A 2 kg weight hanging 1 m under the anchor's centre, from a point joint at the
// anchor's underside.
fn void hang(PhysicsWorld* world, float break_force = 0, bool can_sleep = false)
{
	world.add_body({
		.id = 2, .mass = 2, .gravity_factor = 1, .can_sleep = can_sleep,
		.collider = { .shape = &WEIGHT, .translation = { 0, 4, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
	})!!;
	GenericJoint3D joint = {
		.id = 7, .body_a = 1, .body_b = 2,
		.anchor_a = { 0, -0.5f, 0 }, .anchor_b = { 0, 0.5f, 0 },
		.relative_orientation = { 0, 0, 0, 1 },
		.break_force = break_force,
	};
	joint.linear_limits.push({ .axes = { 1, 0, 0 } });
	joint.linear_limits.push({ .axes = { 0, 1, 0 } });
	joint.linear_limits.push({ .axes = { 0, 0, 1 } });
	world.add_constraint(joint)!!;
}

// A 1 kg beam welded by one end to the anchor's +x face, sticking out sideways:
// the weld carries its weight and the moment of it, m * g * 0.5 m.
fn void cantilever(PhysicsWorld* world, float break_force = 0, float break_torque = 0)
{
	world.add_body({
		.id = 2, .mass = 1, .gravity_factor = 1,
		.collider = { .shape = &BEAM, .translation = { 1, 5, 0 }, .scale = { 1, 1, 1 }, .rotation = { 0, 0, 0, 1 } },
	})!!;
	GenericJoint3D joint = {
		.id = 7, .body_a = 1, .body_b = 2,
		.anchor_a = { 0.5f, 0, 0 }, .anchor_b = { -0.5f, 0, 0 },
		.relative_orientation = { 0, 0, 0, 1 },
		.break_force = break_force, .break_torque = break_torque,
	};
	joint.linear_limits.push({ .axes = { 1, 0, 0 } });
	joint.linear_limits.push({ .axes = { 0, 1, 0 } });
	joint.linear_limits.push({ .axes = { 0, 0, 1 } });
	joint.angular_limits.push({ .axes = { 1, 0, 0 } });
	joint.angular_limits.push({ .axes = { 0, 1, 0 } });
	joint.angular_limits.push({ .axes = { 0, 0, 1 } });
	world.add_constraint(joint)!!;
}

fn void step(PhysicsWorld* world, int frames, uint substeps = 4)
{
	for (int f = 0; f < frames; f++) world.run_step(1.0f / 60.0f, 1, substeps);
}

fn void a_hanging_weight_carries_its_weight_at_any_substep_count() @test
{
	foreach (substeps : (uint[]){ 4, 8, 16 })
	{
		PhysicsWorld* world = make_world();
		defer free_world(world);
		hang(world);
		step(world, 60, substeps);

		GenericJoint3D joint = world.joints[0];
		test::@check(math::abs(joint.force - 2 * G) < 0.02f * 2 * G,
			"at %d substeps the joint carried %f N, not 2 kg's %f", substeps, joint.force, 2 * G);
		test::@check(joint.torque < 0.01f, "a point joint carried %f N m", joint.torque);
	}
}

fn void a_weld_carries_the_moment_of_what_it_holds_out() @test
{
	PhysicsWorld* world = make_world();
	defer free_world(world);
	cantilever(world);
	step(world, 60);

	GenericJoint3D joint = world.joints[0];
	test::@check(math::abs(joint.force - G) < 0.02f * G, "the weld carried %f N, not %f", joint.force, G);
	test::@check(math::abs(joint.torque - G * 0.5f) < 0.02f * G * 0.5f,
		"the weld carried %f N m, not %f", joint.torque, G * 0.5f);
}

fn void a_sleeping_weight_still_reads_its_weight() @test
{
	// Asleep, the joint is not solved; it keeps the load it carried last.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	hang(world, can_sleep: true);
	step(world, 600);

	test::@check(world.find_body(2)!!.sleeping, "the weight never slept");
	// The joint's constraints are built every substep, asleep or not; they
	// piled up while everything slept, and the solve over them grew each frame.
	test::eq(world.constraints.len(), 0);
	// Nor solved: a sleeping island's welds cost a castle nothing while it stands.
	test::@check(world.joints[0].resting, "the joint of a sleeping weight was still solved");
	test::eq(world.joints[0].constraint_count, 0);
	float force = world.joints[0].force;
	test::@check(math::abs(force - 2 * G) < 0.02f * 2 * G, "asleep, the joint reads %f N", force);
}

fn void a_joint_breaks_past_its_rating_and_says_so() @test
{
	PhysicsWorld* world = make_world();
	defer free_world(world);
	hang(world, break_force: 15);

	step(world, 1);
	test::eq(world.joints.len(), 0);
	test::eq(world.joint_breaks.len(), 1);
	JointBreak broke = world.joint_breaks[0];
	test::eq(broke.joint_id, 7);
	test::eq(broke.body_a, 1);
	test::eq(broke.body_b, 2);
	test::@check(broke.force > 15, "it broke carrying %f N", broke.force);

	step(world, 30);
	test::eq(world.joint_breaks.len(), 0);
	float y = world.find_body(2)!!.collider.translation.y;
	test::@check(y < 3, "the weight is still up at %f after it was let go", y);
}

fn void a_joint_under_its_rating_holds() @test
{
	PhysicsWorld* world = make_world();
	defer free_world(world);
	hang(world, break_force: 25);
	for (int f = 0; f < 120; f++)
	{
		step(world, 1);
		test::eq(world.joint_breaks.len(), 0);
	}
	test::eq(world.joints.len(), 1);
}

fn void a_weld_breaks_on_torque_alone() @test
{
	// 4.9 N m against a rating of 4, and no force rating at all.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	cantilever(world, break_torque: 4);
	step(world, 1);
	test::eq(world.joints.len(), 0);
	test::eq(world.joint_breaks.len(), 1);
	test::@check(world.joint_breaks[0].torque > 4, "it broke carrying %f N m", world.joint_breaks[0].torque);
}

fn void a_broken_joint_stops_keeping_its_bodies_apart() @test
{
	// Two joints on one pair, both keeping it from colliding. The pair collides
	// again only when the last of them is gone. The first in the list takes the
	// load, so it is the one a 1 N rating breaks.
	PhysicsWorld* world = make_world();
	defer free_world(world);
	hang(world, break_force: 1);
	GenericJoint3D spare = world.joints[0];
	spare.id = 8;
	spare.break_force = 0;
	world.add_constraint(spare)!!;

	step(world, 1);
	test::eq(world.joint_breaks.len(), 1);
	test::eq(world.joint_breaks[0].joint_id, 7);
	test::@check(world.filters.has_key({ 1, 2 }), "the pair collides while a joint still keeps it apart");

	world.joints[0].break_force = 1;
	step(world, 1);
	test::eq(world.joint_breaks.len(), 1);
	test::eq(world.joint_breaks[0].joint_id, 8);
	test::@check(!world.filters.has_key({ 1, 2 }), "the pair is still kept apart with no joint left");
}

module collision;

import std::collections::fixedlist;
import std::io;
import std::math;
import std::collections::list;
import std::time;

alias Limits = FixedList {Limit, 3};

struct Limit
{
	Vec3 axes;
	float min;
	float max;
	float stiffness;
}

struct GenericJoint3D
{
	Limits angular_limits;
	Limits linear_limits;

	bool collides;

	Vec3 anchor_a;
	Vec3 anchor_b;
	QuaternionValue relative_orientation;

	sz body_a;
	sz body_b;

	float stiffness;
}

<*
 World-space rotation vector (axis * angle, shortest arc) by which `qb` has drifted
 from the rest orientation `rest` relative to `qa`. Shared by joint creation and
 the angular solve.
*>
fn Vec3 joint_angular_error(QuaternionValue qa, QuaternionValue qb, QuaternionValue rest)
{
	QuaternionValue error = qa.conjugate() * qb * rest.conjugate();
	float direction = error.l >= 0 ? 1.0f : -1.0f;
	Vec3 error_local = error.v.xyz * 2 * direction;
	return qa * error_local;
}

// A linear joint constraint along `normal`; `target` is the position the error is
// measured against and `side` the one-sided direction (see XpbdConstraint.limit_side).
// Anchors stay in local space: the solver rotates them to world each iteration.
fn void push_linear_constraint(
	PhysicsWorld* world, Rigidbody* a, Rigidbody* b, GenericJoint3D* joint, Vec3 normal, float target, int side,
	float compliance
) @local
{
	world.constraints.push(
		{
			.body_a = a,
			.body_b = b,
			.normal = normal,
			.r_a = joint.anchor_a,
			.r_b = joint.anchor_b,
			.c_value = target,
			.compliance = compliance,
			.limit_side = side,
			.type = JOINT_POSITION,
		}
	);
}

// An angular joint constraint about `world_axis` (`local_axis` in body A's frame,
// recomputed by the solver); `target` is the rotation the error is measured against.
fn void push_angular_constraint(
	PhysicsWorld* world, Rigidbody* a, Rigidbody* b, GenericJoint3D* joint, Vec3 world_axis, Vec3 local_axis,
	float target, int side, float compliance
) @local
{
	world.constraints.push(
		{
			.body_a = a,
			.body_b = b,
			.normal = world_axis,
			.r_a = local_axis,
			.c_value = target,
			.compliance = compliance,
			.limit_side = side,
			.type = JOINT_ANGULAR,
			.joint_orientation = joint.relative_orientation,
		}
	);
}

fn void GenericJoint3D.create_xpbd_constraint(&joint, PhysicsWorld* world, float dt)
{
	Rigidbody* a = world.find_body(joint.body_a)!!;
	Rigidbody* b = world.find_body(joint.body_b)!!;

	Vec3 world_a = a.collider.translation + (a.collider.rotation * joint.anchor_a);
	Vec3 world_b = b.collider.translation + (b.collider.rotation * joint.anchor_b);

	float base_compliance = stiffness_to_compliance(joint.stiffness, dt, 0);

	foreach (limit : joint.linear_limits)
	{
		Vec3 normal = limit.axes;
		float position_error = (world_b - world_a).dot(normal);
		float compliance = stiffness_to_compliance(limit.stiffness, dt, base_compliance);

		// Free axis: hold the anchors together
		if (limit.min == 0 && limit.max == 0)
		{
			push_linear_constraint(world, a, b, joint, normal, 0, 0, compliance);
			continue;
		}

		// Limits are one-sided: they only act while violated, and drive the error
		// to the limit rather than to zero.
		if (limit.min != 0 && position_error < limit.min)
		{
			push_linear_constraint(world, a, b, joint, normal, limit.min, 1, compliance);
		}
		if (limit.max != 0 && position_error > limit.max)
		{
			push_linear_constraint(world, a, b, joint, normal, limit.max, -1, compliance);
		}
	}

	Vec3 world_error = joint_angular_error(a.collider.rotation, b.collider.rotation, joint.relative_orientation);
	int impulse_dir = math::min(a.impulse_dir, b.impulse_dir);

	foreach (limit : joint.angular_limits)
	{
		Vec3 normal = joint.relative_orientation * limit.axes * impulse_dir;
		Vec3 world_axis = a.collider.rotation * normal;

		float rotation_error = world_error.dot(world_axis);
		float compliance = stiffness_to_compliance(limit.stiffness, dt, base_compliance);

		// Locked axis: bilateral, target zero rotation error
		if (limit.min == 0 && limit.max == 0)
		{
			push_angular_constraint(world, a, b, joint, world_axis, normal, 0, 0, compliance);
			continue;
		}

		if (rotation_error < limit.min)
		{
			push_angular_constraint(world, a, b, joint, world_axis, normal, limit.min, 1, compliance);
		}
		if (rotation_error > limit.max)
		{
			push_angular_constraint(world, a, b, joint, world_axis, normal, limit.max, -1, compliance);
		}
	}
}

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

	// The caller's name for the joint, reported back in a `JointBreak`. The world
	// never reads it.
	sz id;

	// The load the joint breaks at: newtons through its linear limits and
	// newton-metres through its angular ones. 0 never breaks. A joint that
	// carries more in any substep is removed at the end of that substep's solve
	// and reported in `PhysicsWorld.joint_breaks`.
	float break_force;
	float break_torque;

	// What the joint carried in the last substep it was solved, in the same
	// units; kept while both its bodies sleep. Written by the world.
	float force;
	float torque;

	// Where this substep's constraints for the joint start in
	// `PhysicsWorld.constraints`, and how many there are. Written by
	// `create_xpbd_constraint`; meaningless outside a substep.
	sz first_constraint;
	sz constraint_count;
}

<*
 A joint that carried more than it was rated for, and was removed.
*>
struct JointBreak
{
	sz joint_id; // `GenericJoint3D.id`
	sz body_a;
	sz body_b;
	float force; // Newtons, in the substep it broke
	float torque; // Newton-metres, in the same substep
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

	joint.first_constraint = world.constraints.len();
	defer joint.constraint_count = world.constraints.len() - joint.first_constraint;

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

<*
 Whether the joint carried more than it is rated for in the last substep.
*>
fn bool GenericJoint3D.overloaded(&self)
{
	return (self.break_force > 0 && self.force > self.break_force)
		|| (self.break_torque > 0 && self.torque > self.break_torque);
}

<*
 Reads what each joint carried off the multipliers this substep's solve
 accumulated, then removes and reports the joints that carried more than they
 are rated for. A multiplier is a position-level impulse, so the force it stands
 for is λ / h² (Müller et al. 2020, "Detailed Rigid Body Simulation with
 Extended Position Based Dynamics").

 Runs between the solve and the derive, while `constraints` still holds the
 multipliers. A joint broken here held for this substep and is gone from the
 next. Its bodies collide again unless another joint between them still keeps
 them apart.

 @param dt : `The substep's length`
*>
fn void PhysicsWorld.measure_joints(&self, float dt)
{
	if (self.joints.len() == 0) return;
	float inv_h2 = 1.0f / (dt * dt);
	XpbdConstraint[] cs = self.constraints.array_view();

	bool broke = false;
	foreach (&joint : self.joints)
	{
		XpbdConstraint[] own = cs[joint.first_constraint:joint.constraint_count];
		if (own.len > 0 && !own[0].body_a.is_active() && !own[0].body_b.is_active()) continue;

		Vec3 force;
		Vec3 torque;
		foreach (&c : own)
		{
			if (c.type == JOINT_POSITION)
			{
				force += c.normal * c.lambda;
			}
			else
			{
				torque += c.normal * c.lambda;
			}
		}
		joint.force = force.length() * inv_h2;
		joint.torque = torque.length() * inv_h2;
		if (joint.overloaded()) broke = true;
	}
	if (!broke) return;

	// Reported in list order, then compacted, so the survivors keep theirs.
	GenericJoint3D[] all = self.joints.array_view();
	foreach (&joint : all)
	{
		if (!joint.overloaded()) continue;
		self.joint_breaks.push(
			{
				.joint_id = joint.id,
				.body_a = joint.body_a,
				.body_b = joint.body_b,
				.force = joint.force,
				.torque = joint.torque,
			}
		);
		if (!joint.collides && !self.holds_apart(joint.body_a, joint.body_b))
		{
			self.filters.remove({ joint.body_a, joint.body_b });
		}
	}
	sz kept = 0;
	foreach (joint : all)
	{
		if (!joint.overloaded()) all[kept++] = joint;
	}
	while (self.joints.len() > kept) self.joints.remove_last();
	self.islands_dirty = true;
}

// Whether a joint that is not breaking this substep keeps `a` and `b` from colliding.
fn bool PhysicsWorld.holds_apart(&self, sz a, sz b)
{
	foreach (&joint : self.joints)
	{
		if (joint.collides || joint.overloaded()) continue;
		if ((joint.body_a == a && joint.body_b == b) || (joint.body_a == b && joint.body_b == a)) return true;
	}
	return false;
}

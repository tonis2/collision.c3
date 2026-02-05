module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;

enum Axis : const int {
   X,
   Y,
   Z,
   EMPTY = -1,
}

alias Limits = ElasticArray{Limit, 3};

struct Limit {
    Vec3 axes;
    float min;
    float max;
    float stiffness;
    float damping;
}

struct GenericJoint3D {
   Limits angular_limits;
   Limits linear_limits;

   bool collides;

   Vec3 anchor_a;
   Vec3 anchor_b;
   QuaternionValue relative_orientation;

   usz body_a;
   usz body_b;

   float dampening;
   float stifness;
}

fn void GenericJoint3D.create_xpbd_constraint(&joint, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(joint.body_a)!!;
    Rigidbody* b = world.find_body(joint.body_b)!!;

    Vec3 world_a = a.collider.translation + (a.collider.rotation * joint.anchor_a);
    Vec3 world_b = b.collider.translation + (b.collider.rotation * joint.anchor_b);

    // Store local-space anchors — solver will rotate to world space each iteration
    Vec3 rA = joint.anchor_a;
    Vec3 rB = joint.anchor_b;

    // Map stiffness to XPBD compliance: compliance = 1/stiffness / dt^2
    float base_compliance = joint.stifness > 0 ? (1.0f / joint.stifness) / (dt * dt) : 0;

    foreach (limit: joint.linear_limits) {
        Vec3 normal = limit.axes;
        float position_error = (world_b - world_a).dot(normal);

        float compliance = limit.stiffness > 0 ? (1.0f / limit.stiffness) / (dt * dt) : base_compliance;

        // Free joint (no limits) — just constrain to anchor
        if (limit.min == 0 && limit.max == 0) {
            world.constraints.push({
                .body_a = a,
                .body_b = b,
                .normal = normal,
                .r_a = rA,
                .r_b = rB,
                .c_value = position_error,
                .compliance = compliance,
                .lambda = 0,
                .type = JOINT_POSITION,
            });
            continue;
        }

        // Check if outside limits
        if (limit.min != 0 && position_error < limit.min) {
            world.constraints.push({
                .body_a = a,
                .body_b = b,
                .normal = normal,
                .r_a = rA,
                .r_b = rB,
                .c_value = position_error - limit.min,
                .compliance = compliance,
                .lambda = 0,
                .type = JOINT_POSITION,
            });
        }

        if (limit.max != 0 && position_error > limit.max) {
            world.constraints.push({
                .body_a = a,
                .body_b = b,
                .normal = normal,
                .r_a = rA,
                .r_b = rB,
                .c_value = position_error - limit.max,
                .compliance = compliance,
                .lambda = 0,
                .type = JOINT_POSITION,
            });
        }
    }

    // Angular constraints
    QuaternionValue error_orientation = a.collider.rotation.conjugate() * b.collider.rotation * joint.relative_orientation.conjugate();
    float error_direction = error_orientation.l >= 0 ? 1 : -1;
    Vec3 error_local = error_orientation.v.xyz * 2 * error_direction;
    Vec3 world_error = a.collider.rotation * error_local;

    int impulse_dir = math::min(a.impulse_dir, b.impulse_dir);

    foreach (limit: joint.angular_limits) {
        Vec3 normal = joint.relative_orientation * limit.axes * impulse_dir;
        Vec3 world_axis = a.collider.rotation * normal;

        float rotation_error = world_error.dot(world_axis);
        float compliance = limit.stiffness > 0 ? (1.0f / limit.stiffness) / (dt * dt) : base_compliance;

        float min_limit = limit.min - rotation_error;
        float max_limit = rotation_error - limit.max;

        if (min_limit > 0) {
            world.constraints.push({
                .body_a = a,
                .body_b = b,
                .normal = world_axis,
                .c_value = min_limit * rotation_error,
                .compliance = compliance,
                .lambda = 0,
                .type = JOINT_ANGULAR,
            });
        }

        if (max_limit > 0) {
            world.constraints.push({
                .body_a = a,
                .body_b = b,
                .normal = world_axis,
                .c_value = -max_limit * rotation_error,
                .compliance = compliance,
                .lambda = 0,
                .type = JOINT_ANGULAR,
            });
        }
    }
}
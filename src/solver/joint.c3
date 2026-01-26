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

fn void GenericJoint3D.create_constraint(joint, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(joint.body_a)!!;
    Rigidbody* b = world.find_body(joint.body_b)!!;

    // if (a.sleeping || b.sleeping) return;

    world.contact_cache.set({joint.body_a, joint.body_b}, {});

    Vec3* impulse_cache = world.contact_cache.get_ref({joint.body_a, joint.body_b})!!;

    float inverse_mass_sum = a.inverse_mass + b.inverse_mass;

    // Need to check if impulse direction is - or +
    int impulse_dir = math::min(a.impulse_dir, b.impulse_dir);

    Vec3 world_a = a.collider.translation + (a.collider.rotation * joint.anchor_a);
    Vec3 world_b = b.collider.translation + (b.collider.rotation * joint.anchor_b);

    Vec3 rA = world_a - a.collider.translation;
    Vec3 rB = world_b - b.collider.translation;

    foreach (limit: joint.linear_limits) {
        Vec3 normal = limit.axes;

        usz offset = 0;

        if (limit.axes.y != 0) offset = 1;
        if (limit.axes.z != 0) offset = 2;

        Vec3 linear_a = -normal;
        Vec3 angular_a = rA.cross(-normal);

        Vec3 linear_b = normal;
        Vec3 angular_b = rB.cross(normal);

        float normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia(rA, -normal);
        if (!b.is_static) normal_inertia += b.get_intertia(rB, normal);

        float rigid_denom = inverse_mass_sum + normal_inertia;
        // float softness_denom = (limit.damping / dt) + (limit.stiffness * dt);
        // float total_denom = rigid_denom + softness_denom;

        float position_error = (world_b - world_a).dot(normal);

        // float bounce = (limit.stiffness * dt) / total_denom * position_error;
        // float bias = (limit.stiffness * dt) / rigid_denom * position_error;

        float denom = (0.2 / dt);

        Constraint axis_constraint = {
            .body_a = a,
            .body_b = b,
            .linear_a = linear_a,
            .angular_a = angular_a,
            .linear_b = linear_b,
            .angular_b = angular_b,
            .effective_mass = 1 / rigid_denom,
            .min = -float.max,
            .max = float.max,
            .impulse_cache = impulse_cache,
            .bias = denom * position_error,
            .bounce = 0,
            .impulse_offset = offset,
        };

        if (limit.min != 0) {
            float correction = (limit.min - position_error) * impulse_dir;
            if (correction > 0) {
                axis_constraint.bias = denom * correction;
                axis_constraint.min = -float.max;
                axis_constraint.max = 0;
            }
        }

        if (limit.max != 0) {
            float correction = (position_error - limit.max) * impulse_dir;
            if (correction > 0) {
                axis_constraint.bias = denom * correction;
                axis_constraint.min = 0;
                axis_constraint.max = float.max;
            }
        }

        world.constraints.push(axis_constraint);
    }
    

    // Relative orientation is neccesary for rotation error check, if joint original position is rotated
    QuaternionValue error_orientation = a.collider.rotation.conjugate() * b.collider.rotation * joint.relative_orientation.conjugate();

    float error_direction = error_orientation.l >= 0 ? 1 : -1;
    Vec3 error_local = error_orientation.v.xyz * 2 * error_direction;
    Vec3 world_error = a.collider.rotation * error_local;

    foreach (limit: joint.angular_limits) {
        Vec3 normal = joint.relative_orientation * limit.axes * impulse_dir;
        Vec3 world_axis = a.collider.rotation * normal ;

        usz offset = 0;

        if (limit.axes.y != 0) offset = 1;
        if (limit.axes.z != 0) offset = 2;

        float normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia(rA, -world_axis);
        if (!b.is_static) normal_inertia += b.get_intertia(rB, world_axis);

        float denom = (0.2 / dt);
        float rotation_error = world_error.dot(world_axis);
        
        float rigid_denom = inverse_mass_sum + normal_inertia;

        // float softness_denom = (limit.damping / dt) + (limit.stiffness * dt);
        // float total_denom = rigid_denom + softness_denom;

        // float bounce = (limit.stiffness * dt) / total_denom * rotation_error;
        float bias = (0.2 / dt);

        Constraint axis_constraint = {
            .body_a = a,
            .body_b = b,
            .linear_a = {0,0,0},
            .linear_b = {0,0,0},
            .angular_a = -world_axis,
            .angular_b = world_axis,
            .effective_mass = 1 / rigid_denom,
            .min = -float.max,
            .max = float.max,
            .bias = bias * rotation_error,
            .impulse_cache = impulse_cache,
            .bounce = 0,
            .impulse_offset = offset,
        };

        float min_limit = limit.min - rotation_error;
        float max_limit = rotation_error - limit.max;

        if (min_limit > 0) {
            axis_constraint.bias = bias * min_limit * rotation_error;
            axis_constraint.min = 0;
            axis_constraint.max = float.max;
            world.constraints.push(axis_constraint);
        }

        if (max_limit > 0) {
            axis_constraint.bias = bias * max_limit * rotation_error;
            axis_constraint.min = -float.max;
            axis_constraint.max = 0;
            world.constraints.push(axis_constraint);
        }
    }
}
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
    Number min;
    Number max;
    Number stiffness;
    Number damping;
}

struct GenericJoint3D {
   Limits angular_limits;
   Limits linear_limits;

   bool collides;

   Vec3 anchor_a;
   Vec3 anchor_b;
   QuaternionValue relative_orientation;

   uint body_a;
   uint body_b;

   Vec3 accumulated_linear_impulse;
   Number accumulated_angular_impulse;

   Number dampening;
   Number stifness;
}

fn void GenericJoint3D.create_constraint(joint, PhysicsWorld* world, Number dt) @local {
    Rigidbody* a = world.find_body(joint.body_a)!!;
    Rigidbody* b = world.find_body(joint.body_b)!!;

    // if (a.sleeping || b.sleeping) return;

    ContactKey key = {joint.body_a, joint.body_b};
    world.contact_cache.set(key, {});

    Vec3* impulse_cache = world.contact_cache.get_ref({joint.body_a, joint.body_b})!!;

    Number inverse_mass_sum = a.inverse_mass + b.inverse_mass;

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

        Number normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia2(rA, -normal);
        if (!b.is_static) normal_inertia += b.get_intertia2(rB, normal);

        // 
        Number rigid_denom = inverse_mass_sum + normal_inertia;
        // Number softness_denom = (limit.damping / dt) + (limit.stiffness * dt);
        // Number total_denom = rigid_denom + softness_denom;

        Number position_error = (world_b - world_a).dot(normal);

        // Number bounce = (limit.stiffness * dt) / total_denom * position_error;
        // Number bias = (limit.stiffness * dt) / rigid_denom * position_error;

        Number bias = 0.2 / rigid_denom * position_error;

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
            .bias = bias,
            .bounce = 0,
            .impulse_offset = offset,
        };

        Number denom = 0.2 / rigid_denom;

        if (limit.min != 0) {
            Number correction = (limit.min - position_error) * impulse_dir;
            if (correction > 0) {
                axis_constraint.bias = bias * correction;
                axis_constraint.min = -float.max;
                axis_constraint.max = 0;
            }
        }

        if (limit.max != 0) {
            Number correction = (position_error - limit.max) * impulse_dir;
            if (correction > 0) {
                axis_constraint.bias = bias * correction;
                axis_constraint.min = 0;
                axis_constraint.max = float.max;
            }
        }

        world.constraints.push(axis_constraint);
    }
    
    QuaternionValue error_orientation = a.collider.rotation.conjugate() * b.collider.rotation * joint.relative_orientation.conjugate();

    Number error_direction = error_orientation.l >= 0 ? 1 : -1;
    Vec3 error_local = error_orientation.v.xyz * 2 * error_direction;
    Vec3 world_error = a.collider.rotation * error_local;

    foreach (limit: joint.angular_limits) {
        Vec3 normal = joint.relative_orientation * limit.axes;
        Vec3 world_axis = a.collider.rotation * normal;
    
        float normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia2(rA, -world_axis);
        if (!b.is_static) normal_inertia += b.get_intertia2(rB, world_axis);

        Number denom = (0.2 / dt);
        Number rotation_error = world_error.dot(world_axis);
        
        Number rigid_denom = inverse_mass_sum + normal_inertia;
        // Number softness_denom = (limit.damping / dt) + (limit.stiffness * dt);
        // Number total_denom = rigid_denom + softness_denom;

        // Number bounce = (limit.stiffness * dt) / total_denom * rotation_error;
        // Number bias = (limit.stiffness * dt) / rigid_denom * rotation_error;

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
            .bias = denom * rotation_error,
            .impulse_cache = impulse_cache,
            .bounce = 0,
            .impulse_offset = 0,
        };

        if (limit.min != 0) {
            Number limit_error = (limit.min - (-rotation_error));
            if (limit_error > 0) {
                axis_constraint.bias = denom * limit_error;
                axis_constraint.min = -float.max;
                axis_constraint.max = 0;
                world.constraints.push(axis_constraint);
                continue;
            }
        }

        if (limit.max != 0) {
            Number limit_error = (-rotation_error - limit.max);
            if (limit_error > 0) {        
                axis_constraint.bias = denom * -limit_error;
                axis_constraint.min = 0;
                axis_constraint.max = float.max;
                world.constraints.push(axis_constraint);
                continue;
            }
        }

        if (limit.max == 0 && limit.min == 0) {
            world.constraints.push(axis_constraint);
        }
    }
}
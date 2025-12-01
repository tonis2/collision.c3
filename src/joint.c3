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

    if (a.sleeping || b.sleeping) return;

    ContactKey key = {joint.body_a, joint.body_b};
    world.contact_cache.set(key, {});

    Vec3* impulse_cache = world.contact_cache.get_ref({joint.body_a, joint.body_b})!!;

    Number inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;

    Vec3 world_a = a.collider.translation + (a.collider.rotation * joint.anchor_a);
    Vec3 world_b = b.collider.translation + (b.collider.rotation * joint.anchor_b);

    Vec3 rA = world_a - a.collider.translation;
    Vec3 rB = world_b - b.collider.translation;

    foreach (limit: joint.linear_limits) {
        Vec3 normal = limit.axes;

        Vec3 linear_a = -normal;
        Vec3 angular_a = rA.cross(-normal);

        Vec3 linear_b = normal;
        Vec3 angular_b = rB.cross(normal);

        float normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia2(rA, -normal);
        if (!b.is_static) normal_inertia += b.get_intertia2(rB, normal);

        Number position_error = (world_a - world_b).dot(normal);

        Number beta = 0.2f;
        Number bias = -(beta / dt) * position_error;

        Constraint axis_constraint = {
            .body_a = a,
            .body_b = b,
            .linear_a = linear_a,
            .angular_a = angular_a,
            .linear_b = linear_b,
            .angular_b = angular_b,
            .effective_mass = 1 / (inverse_mass_sum + normal_inertia),
            .min = limit.min,
            .max = limit.max,
            .bias = bias,
            .impulse_cache = impulse_cache,
            .impulse_offset = 0,
        };

        world.constraints.push(axis_constraint);
    }
    
    QuaternionValue relative_orientation = a.collider.rotation.conjugate() * b.collider.rotation;
    QuaternionValue error_orientation = relative_orientation * joint.relative_orientation.conjugate();

    Vec3 error_local = error_orientation.v.xyz * (error_orientation.l >= 0 ? 1 : -1);
    Vec3 world_error = a.collider.rotation * error_local;

    foreach (limit: joint.angular_limits) {
        Vec3 world_axis = a.collider.rotation * joint.relative_orientation * limit.axes;

        float normal_inertia;
        if (!a.is_static) normal_inertia += a.get_intertia2(rA, -world_axis);
        if (!b.is_static) normal_inertia += b.get_intertia2(rB, world_axis);

        Number beta = 0.2f;
        Number bias = (beta / dt) * world_error.dot(world_axis);

        Constraint axis_constraint = {
            .body_a = a,
            .body_b = b,
            .linear_a = {0,0,0},
            .linear_b = {0,0,0},
            .angular_a = -world_axis,
            .angular_b = world_axis,
            .effective_mass = 1 / (inverse_mass_sum + normal_inertia),
            .min = limit.min,
            .max = limit.max,
            .bias = bias,
            .impulse_cache = impulse_cache,
            .impulse_offset = 0,
        };

        world.constraints.push(axis_constraint);
    }
}
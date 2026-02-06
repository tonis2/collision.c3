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

   // AVBD warm-start caches (linear[0..2], angular[3..5])
   float[6] impulse;        // Lambda cache
   float[6] penalty_cache;  // Penalty cache
}

fn void GenericJoint3D.create_force(&joint, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(joint.body_a)!!;
    Rigidbody* b = world.find_body(joint.body_b)!!;

    Vec3 world_a = a.collider.translation + (a.collider.rotation * joint.anchor_a);
    Vec3 world_b = b.collider.translation + (b.collider.rotation * joint.anchor_b);

    // === Linear limits → one AvbdForce (up to 3 rows) ===
    if (joint.linear_limits.len() > 0) {
        AvbdForce force = {
            .body_a = a, .body_b = b,
            .anchor_a = joint.anchor_a,
            .anchor_b = joint.anchor_b,
            .is_contact = false,
            .is_angular = false,
        };

        usz num_rows = 0;
        Vec3 diff = world_a - world_b;

        foreach (limit: joint.linear_limits) {
            if (num_rows >= MAX_FORCE_ROWS) break;

            Vec3 normal = limit.axes;
            usz row = num_rows;

            // Position error: (A-B).dot(axis) to match AVBD Jacobian convention
            float position_error = diff.dot(normal);

            force.basis[row] = normal;
            force.c_value[row] = position_error;
            force.fmin[row] = -float.max;
            force.fmax[row] = float.max;
            force.stiffness[row] = float.max;

            // Warm-start: decay lambda and penalty from previous frame
            usz cache_idx = row;
            force.lambda[row] = joint.impulse[cache_idx] * AVBD_ALPHA * AVBD_GAMMA;
            force.penalty[row] = math::clamp(
                (joint.penalty_cache[cache_idx] > 0 ? joint.penalty_cache[cache_idx] : PENALTY_MIN) * AVBD_GAMMA,
                PENALTY_MIN, PENALTY_MAX);

            // Handle limits
            if (limit.min != 0 || limit.max != 0) {
                if (limit.min != 0 && position_error < limit.min) {
                    force.c_value[row] = position_error - limit.min;
                    force.fmin[row] = 0;
                    force.fmax[row] = float.max;
                } else if (limit.max != 0 && position_error > limit.max) {
                    force.c_value[row] = position_error - limit.max;
                    force.fmin[row] = -float.max;
                    force.fmax[row] = 0;
                }
            }

            num_rows++;
        }

        force.num_rows = num_rows;
        if (num_rows > 0) {
            world.forces.push(force);
        }
    }

    // === Angular limits → one AvbdForce (up to 3 rows) ===
    if (joint.angular_limits.len() > 0) {
        AvbdForce force = {
            .body_a = a, .body_b = b,
            .is_contact = false,
            .is_angular = true,
        };

        // Compute angular error
        QuaternionValue error_orientation = a.collider.rotation.conjugate() * b.collider.rotation * joint.relative_orientation.conjugate();
        float error_direction = error_orientation.l >= 0 ? 1.0f : -1.0f;
        Vec3 error_local = error_orientation.v.xyz * 2.0f * error_direction;
        Vec3 world_error = a.collider.rotation * error_local;

        usz num_rows = 0;

        foreach (limit: joint.angular_limits) {
            if (num_rows >= MAX_FORCE_ROWS) break;

            Vec3 normal = limit.axes;
            Vec3 world_axis = a.collider.rotation * (joint.relative_orientation * normal);
            usz row = num_rows;

            float rotation_error = world_error.dot(world_axis);

            force.basis[row] = world_axis;
            force.c_value[row] = rotation_error;
            force.fmin[row] = -float.max;
            force.fmax[row] = float.max;
            force.stiffness[row] = float.max;

            // Warm-start
            usz cache_idx = 3 + row;
            force.lambda[row] = joint.impulse[cache_idx] * AVBD_ALPHA * AVBD_GAMMA;
            force.penalty[row] = math::clamp(
                (joint.penalty_cache[cache_idx] > 0 ? joint.penalty_cache[cache_idx] : PENALTY_MIN) * AVBD_GAMMA,
                PENALTY_MIN, PENALTY_MAX);

            // Handle angular limits
            if (limit.min != 0 && rotation_error < limit.min) {
                force.c_value[row] = rotation_error - limit.min;
                force.fmin[row] = 0;
                force.fmax[row] = float.max;
            } else if (limit.max != 0 && rotation_error > limit.max) {
                force.c_value[row] = rotation_error - limit.max;
                force.fmin[row] = -float.max;
                force.fmax[row] = 0;
            }

            num_rows++;
        }

        force.num_rows = num_rows;
        if (num_rows > 0) {
            world.forces.push(force);
        }
    }

    // Store current lambda/penalty back for next frame warm-start
    // (will be overwritten by solver, but we seed from joint cache)
}

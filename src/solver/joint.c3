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

   // AVBD warm-start caches
   Vec3 lambda_lin;
   Vec3 lambda_ang;
   Vec3 penalty_lin;
   Vec3 penalty_ang;
}

// Create JointForce(s) from GenericJoint3D (matching reference Joint::initialize + constructor)
fn void GenericJoint3D.create_force(&joint, PhysicsWorld* world, float dt) {
    Rigidbody* a = world.find_body(joint.body_a)!!;
    Rigidbody* b = world.find_body(joint.body_b)!!;

    // Compute torque arm (reference: lengthSq(sizeA + sizeB))
    Vec3 size_a = a.transformed_aabb.max - a.transformed_aabb.min;
    Vec3 size_b = b.transformed_aabb.max - b.transformed_aabb.min;
    float torque_arm = (size_a + size_b).sq_magnitude();
    if (torque_arm < 1.0f) torque_arm = 1.0f;

    float alpha = world.avbd_alpha;
    float gamma = world.avbd_gamma;

    // Determine stiffness for linear and angular parts
    float stiffness_lin = 0.0f;
    float stiffness_ang = 0.0f;

    // If we have linear limits, this is a ball-socket (infinite stiffness for lock axes)
    if (joint.linear_limits.len() > 0) {
        stiffness_lin = float.max;  // Hard constraint by default
    }

    if (joint.angular_limits.len() > 0) {
        stiffness_ang = float.max;  // Hard constraint by default
    }

    JointForce jf = {
        .body_a = a,
        .body_b = b,
        .rA = joint.anchor_a,
        .rB = joint.anchor_b,
        .stiffness_lin = stiffness_lin,
        .stiffness_ang = stiffness_ang,
        .fracture = float.max,  // No fracture by default
        .torque_arm = torque_arm,
        .broken = false,
    };

    // Compute C0 at beginning of timestep (reference joint.cpp:37-38)
    jf.c0_lin = quat_transform(a.collider.translation, a.collider.rotation, joint.anchor_a)
              - quat_transform(b.collider.translation, b.collider.rotation, joint.anchor_b);
    jf.c0_ang = quat_sub(a.collider.rotation, b.collider.rotation) * torque_arm;

    // Warmstart from cached values (reference joint.cpp:42-49)
    jf.lambda_lin = joint.lambda_lin * (alpha * gamma);
    jf.lambda_ang = joint.lambda_ang * (alpha * gamma);
    jf.penalty_lin = vec3_clamp(joint.penalty_lin * gamma, PENALTY_MIN, PENALTY_MAX);
    jf.penalty_ang = vec3_clamp(joint.penalty_ang * gamma, PENALTY_MIN, PENALTY_MAX);

    // Clamp penalty to material stiffness
    jf.penalty_lin = vec3_min(jf.penalty_lin, stiffness_lin);
    jf.penalty_ang = vec3_min(jf.penalty_ang, stiffness_ang);

    // If no linear limits, zero out linear penalty
    if (joint.linear_limits.len() == 0) {
        jf.penalty_lin = { 0, 0, 0 };
        jf.lambda_lin = { 0, 0, 0 };
    }

    // If no angular limits, zero out angular penalty
    if (joint.angular_limits.len() == 0) {
        jf.penalty_ang = { 0, 0, 0 };
        jf.lambda_ang = { 0, 0, 0 };
    }

    world.joint_forces.push(jf);

    // Store back reference index for warm-start save-back
    // (the actual save-back happens after the solve in run_step)
}

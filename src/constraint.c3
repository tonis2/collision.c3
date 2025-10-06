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

enum LimitType : uint  {
    LINEAR,
    ANGULAR
}

alias Limits = ElasticArray{Limit, 3};

alias Jacobian = Vec3f[6];

struct Limit {
    Vec3f axes;
    float min;
    float max;
    float stiffness;
    float damping;
}

struct GenericJoint3D {
   Limits angular_limits;
   Limits linear_limits;

   Vec3f anchor_a;
   Vec3f anchor_b;

   uint body_a;
   uint body_b;

   Vec3f accumulated_linear_impulse;
   Vec3f accumulated_angular_impulse;
}

fn void GenericJoint3D.limit_linear(&self, Vec3f axes, float min = 0, float max = 0) {
    foreach (index, axis: axes) {
        if (axis == 0) continue;
        Vec3f normal = {0, 0, 0};
        normal[index] = axis;
        self.linear_limits.push({
            .min = min,
            .max = max,
            .axes = normal,
        });
    }
}

fn void GenericJoint3D.limit_angular(&self, Vec3f axes, float min = 0, float max = 0) {
    foreach (index, axis: axes) {
        if (axis == 0) continue;
        Vec3f normal = {0, 0, 0};
        normal[index] = axis;
        self.angular_limits.push({
            .min = min,
            .max = max,
            .axes = normal,
        });
    }
}
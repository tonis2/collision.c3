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

alias Limits = ElasticArray{Limit, 6};

alias Jacobian = Vec3f[6];

struct Limit {
    LimitType type;
    Vec3f axes;
    float min;
    float max;
    float stiffness;
    float damping;
}

struct GenericJoint3D {
   Limits limits;

   Vec3f anchor_a;
   Vec3f anchor_b;

   uint body_a;
   uint body_b;
}
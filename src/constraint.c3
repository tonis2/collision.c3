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

enum LimitType : const uint  {
    LINEAR,
    ANGULAR
}

alias Limits = ElasticArray{Limit, 6};

struct Limit {
    LimitType type;
    Axis[3] axes;
    float min;
    float max;
    float stiffness;
    float damping;
}

struct GenericJoint3D {
   Limits limits;

   Vec3f anchor_a;
   Vec3f anchor_b;

   Quaternionf target_orientation;
   Vec3f linear_impulse;
   Vec3f angular_impulse;
}
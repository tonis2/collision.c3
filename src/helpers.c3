module collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec2 @private = float[<2>];
alias Vec4 @private = float[<4>];
alias Vec3 @private = float[<3>];
alias ContactPoints @private = Vec3[2];
alias QuaternionValue = QuaternionNumber{float};
alias Vertices = List{Vec3};

fn usz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
fn usz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;

struct CollisionInfo {
    Simplex simplex;
    Vec3 normal;
    ContactPoints contact_points;
    float depth;
    bool collided;
}

struct HitInfo {
    bool collided;
    float depth;
    Vec3 normal;
    TriangleVerts triangle;
}


struct Face @private {
    usz a,b,c;
    Vec3 normal;
    float distance;
	bool active;
	bool visible;
}

struct Pair (Printable) {
    usz a;
    usz b;
}

struct MinMax {
    float min;
    float max;
}

fn MinMax min_max_from_axis(Vec3 axis, Vec3[] points) {
    MinMax corners = {
        .min = axis.dot(points[0]),
        .max = axis.dot(points[0])
    };

    foreach (point: points) corners.expand(axis.dot(point));

    return corners;
}

fn void MinMax.expand(&self, float value) {
    self.min = math::min(self.min, value);
    self.max = math::max(self.max, value);
}

fn void MinMax.expand_by_array(&self, float[] values) {
    foreach (value: values) self.expand(value);
}

fn MinMax min_max_from_box(Vec3 center, Vec3 extent, Vec3 axis) {
    float c = center.dot(axis);
    float r = math::abs(axis.x) * extent.x + math::abs(axis.y) * extent.y + math::abs(axis.z) * extent.z;

    return {
        .min = c - r,
        .max = c + r
    };
}

fn float MinMax.overlap(&a, MinMax b) {
    if (a.max < b.min || b.max < a.min) {
        // Separated 
        return -1;
    }
	
    return math::min(a.max, b.max) - math::max(a.min, b.min);
}

fn usz? MinMax.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("min %s max %s",  self.min, self.max)!;

fn Matrix3f QuaternionValue.as_mat3(self) {
    Matrix4f matrix = self.to_matrixf();
    return {
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };
}

fn Matrix3f mat3_outer(Vec3 a, Vec3 b) {
    return {
        a.x*b.x, a.x*b.y, a.x*b.z,
        a.y*b.x, a.y*b.y, a.y*b.z,
        a.z*b.x, a.z*b.y, a.z*b.z,
    };
}

fn Matrix3f mat3_diagonal(Vec3 d) {
    return {
        d.x, 0, 0,
        0, d.y, 0,
        0, 0, d.z,
    };
}

fn Vec3 mat3_solve(Matrix3f a, Vec3 b) {
    return a.inverse()!! * b;
}

fn Vec3 Vec3.abs(self) {
    return { math::abs(self.x), math::abs(self.y), math::abs(self.z) };
}

fn void Vec3.set(&self, Vec3 new) @private => mem::copy(self, &new, Vec3.sizeof, 0);

fn usz Vec3.hash(&self) @private {
    usz hash = 17;
    hash = (usz)(hash * 23f + self.x);
    hash = (usz)(hash * 23f + self.y);
    hash = (usz)(hash * 23f + self.z);
    return hash;
}

fn float Vec3.distance_sq(self, Vec3 v2) {
    Vec3 dist = self - v2;
    return dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;
}

fn usz? ContactPoints.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s", (*self)[0], (*self)[1])!;
fn usz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.a, self.b)!;
macro bool Pair.equals(a, Pair b) @operator(==) => (a.a == b.a && a.b == b.b) || (a.a == b.b && a.b == b.a);

macro uint Pair.hash(&self) {
    if (self.a > self.b) @swap(self.a, self.b);
    return builtin::@generic_hash(self.a) + builtin::@generic_hash(self.b);
}

macro bool QuaternionValue.equals(self, QuaternionValue b) @operator(==) => self.v.xyzw == b.v.xyzw;

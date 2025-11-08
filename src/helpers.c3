module collision;

import std::math;
import std::io;

alias Number = float;

alias Vec2 @private = Number[<2>];
alias Vec3 @private = Number[<3>];
alias Vec4 @private = Number[<4>];

// fn usz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
// fn usz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s,%s", self.x, self.y, self.z, self.w)!;

struct HitInfo {
    bool hits;
    Number depth;
    Vec3 normal;
}

struct Face @private {
    usz a,b,c;
    Vec3 normal;
    Number distance;
}

struct Pair (Printable) {
    usz first;
    usz second;
}

fn Matrix3f Quaternionf.as_mat3(self) {
    Matrix4f matrix = self.to_matrixf();
    return {
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };
}

fn void Vec3.set(&self, Vec3 new) @private => mem::copy(self, &new, Vec3.sizeof, 0);

fn usz Vec3.hash(&self) @private {
    usz hash = 17;
    hash = (usz)(hash * 23f + self.x);
    hash = (usz)(hash * 23f + self.y);
    hash = (usz)(hash * 23f + self.z);
    return hash;
}

fn Number Vec3.distance_sq(self, Vec3 v2) {
    Vec3 dist = self - v2;
    return dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;
}

fn usz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.first, self.second)!;

macro bool Pair.equals(self, Pair b) @operator(==) => self.first == b.first && self.second == b.second;
macro uint Pair.hash(Pair self) => builtin::@generic_hash(self.first) + builtin::@generic_hash(self.second);
macro bool Vec3.equals(self, Vec3 b) => self == b;
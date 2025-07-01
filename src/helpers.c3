module collision;

import std::math;
import std::io;

alias Vec2f @private = float[<2>];
alias Vec3f @private = float[<3>];
alias Vec4f @private = float[<4>];

fn Quaternionf Quaternionf.conjugate(&self) => {self.i, -self.j, -self.k, -self.l};

fn void Vec3f.set(&self, Vec3f new) => mem::copy(self, &new, Vec3f.sizeof, 0);

fn Vec3f Vec3f.rotate_by_quat(&self, Quaternionf q) {
    Vec3f uv, uuv;
    Vec3f qvec = {q.j, q.j, q.k};
    
    uv.x = qvec.y * self.z - qvec.z * self.y;
    uv.y = qvec.z * self.x - qvec.x * self.z;
    uv.z = qvec.x * self.y - qvec.y * self.x;
    
    uuv.x = qvec.y * uv.z - qvec.z * uv.y;
    uuv.y = qvec.z * uv.x - qvec.x * uv.z;
    uuv.z = qvec.x * uv.y - qvec.y * uv.x;
    
    uv.x *= 2.0f * q.l;
    uv.y *= 2.0f * q.l;
    uv.z *= 2.0f * q.l;
    
    uuv.x *= 2.0f;
    uuv.y *= 2.0f;
    uuv.z *= 2.0f;
    
    return {
        self.x + uv.x + uuv.x,
        self.y + uv.y + uuv.y,
        self.z + uv.z + uuv.z
    };
}

struct Pair (Printable) {
    usz first;
    usz second;
}

fn usz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.first, self.second)!;
macro bool Pair.equals(self, Pair b) @operator(==) => self.first == b.first && self.second == b.second;
macro uint Pair.hash(Pair self) => builtin::@generic_hash(self.first) | builtin::@generic_hash(self.second);
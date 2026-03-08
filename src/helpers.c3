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

fn void Vec3.set(&self, Vec3 new) => mem::copy(self, &new, Vec3.sizeof, 0);

fn usz Vec3.hash(&self) {
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

// === AVBD math helpers (ports from reference maths.h) ===

// Quaternion difference as angular vector: (a * inverse(b)).vec * 2
fn Vec3 quat_sub(QuaternionValue a, QuaternionValue b) {
    QuaternionValue diff = a * b.conjugate();
    return diff.v.xyz * 2.0f;
}

// Apply angular delta vector to quaternion: normalize(q + quat(v,0) * q * 0.5)
fn QuaternionValue quat_add_vec3(QuaternionValue q, Vec3 v) {
    QuaternionValue omega = {v.x * 0.5f, v.y * 0.5f, v.z * 0.5f, 0.0f};
    return (q + omega * q).normalize();
}

// Build orthonormal basis: row0=normal, row1,2=tangents (from reference maths.h)
fn Matrix3f orthonormal_basis(Vec3 normal) {
    Vec3 t1;
    if (math::abs(normal.x) > math::abs(normal.z)) {
        t1 = (Vec3){-normal.y, normal.x, 0};
    } else {
        t1 = (Vec3){0, -normal.z, normal.y};
    }
    t1 = t1.normalize();
    Vec3 t2 = normal.cross(t1);
    return {
        normal.x, normal.y, normal.z,
        t1.x, t1.y, t1.z,
        t2.x, t2.y, t2.z,
    };
}

// Cross-product (skew-symmetric) matrix
fn Matrix3f mat3_skew(Vec3 r) {
    return {
        0, -r.z, r.y,
        r.z, 0, -r.x,
        -r.y, r.x, 0,
    };
}

// Diagonal of column norms (from reference maths.h)
fn Matrix3f mat3_diagonalize(Matrix3f m) {
    float c0 = math::sqrt(m.m00*m.m00 + m.m10*m.m10 + m.m20*m.m20);
    float c1 = math::sqrt(m.m01*m.m01 + m.m11*m.m11 + m.m21*m.m21);
    float c2 = math::sqrt(m.m02*m.m02 + m.m12*m.m12 + m.m22*m.m22);
    return { c0, 0, 0, 0, c1, 0, 0, 0, c2 };
}

// Geometric stiffness for ball-socket joint (from reference joint.cpp:14-23)
fn Matrix3f geometric_stiffness_ball_socket(int k, Vec3 v) {
    float vk;
    switch (k) {
        case 0: vk = v.x;
        case 1: vk = v.y;
        case 2: vk = v.z;
        default: vk = 0;
    }
    Matrix3f m = { -vk, 0, 0, 0, -vk, 0, 0, 0, -vk };
    switch (k) {
        case 0: m.m00 += v.x; m.m10 += v.y; m.m20 += v.z;
        case 1: m.m01 += v.x; m.m11 += v.y; m.m21 += v.z;
        case 2: m.m02 += v.x; m.m12 += v.y; m.m22 += v.z;
    }
    return m;
}

// Rotate vector by quaternion (Rodrigues form, from reference maths.h)
fn Vec3 quat_rotate(QuaternionValue q, Vec3 v) {
    Vec3 u = q.v.xyz;
    Vec3 t = u.cross(v) * 2.0f;
    return v + t * q.l + u.cross(t);
}

// Transform: rotate then translate (from reference maths.h)
fn Vec3 quat_transform(Vec3 pos, QuaternionValue rot, Vec3 v) {
    return quat_rotate(rot, v) + pos;
}

// Clamp each component of a Vec3
fn Vec3 vec3_clamp(Vec3 v, float lo, float hi) {
    return {
        math::clamp(v.x, lo, hi),
        math::clamp(v.y, lo, hi),
        math::clamp(v.z, lo, hi),
    };
}

// Component-wise min(Vec3, float)
fn Vec3 vec3_min(Vec3 a, float b) {
    return { math::min(a.x, b), math::min(a.y, b), math::min(a.z, b) };
}

// 6x6 LDL^T solver (direct port from reference maths.h:481-547)
// Solves symmetric PD system [aLin, aCross^T; aCross, aAng] * [xLin; xAng] = [bLin; bAng]
fn void solve_ldlt(Matrix3f a_lin, Matrix3f a_ang, Matrix3f a_cross,
                   Vec3 b_lin, Vec3 b_ang, Vec3* x_lin, Vec3* x_ang) {
    // Extract elements from lower triangle storage
    float a11 = a_lin.m00;
    float a21 = a_lin.m10; float a22 = a_lin.m11;
    float a31 = a_lin.m20; float a32 = a_lin.m21; float a33 = a_lin.m22;
    float a41 = a_cross.m00; float a42 = a_cross.m01; float a43 = a_cross.m02; float a44 = a_ang.m00;
    float a51 = a_cross.m10; float a52 = a_cross.m11; float a53 = a_cross.m12; float a54 = a_ang.m10; float a55 = a_ang.m11;
    float a61 = a_cross.m20; float a62 = a_cross.m21; float a63 = a_cross.m22; float a64 = a_ang.m20; float a65 = a_ang.m21; float a66 = a_ang.m22;

    // Step 1: LDL^T decomposition
    float l21 = a21 / a11;
    float l31 = a31 / a11;
    float l41 = a41 / a11;
    float l51 = a51 / a11;
    float l61 = a61 / a11;

    float d1 = a11;

    float d2 = a22 - l21 * l21 * d1;

    float l32 = (a32 - l21 * l31 * d1) / d2;
    float l42 = (a42 - l21 * l41 * d1) / d2;
    float l52 = (a52 - l21 * l51 * d1) / d2;
    float l62 = (a62 - l21 * l61 * d1) / d2;

    float d3 = a33 - (l31 * l31 * d1 + l32 * l32 * d2);

    float l43 = (a43 - l31 * l41 * d1 - l32 * l42 * d2) / d3;
    float l53 = (a53 - l31 * l51 * d1 - l32 * l52 * d2) / d3;
    float l63 = (a63 - l31 * l61 * d1 - l32 * l62 * d2) / d3;

    float d4 = a44 - (l41 * l41 * d1 + l42 * l42 * d2 + l43 * l43 * d3);

    float l54 = (a54 - l41 * l51 * d1 - l42 * l52 * d2 - l43 * l53 * d3) / d4;
    float l64 = (a64 - l41 * l61 * d1 - l42 * l62 * d2 - l43 * l63 * d3) / d4;

    float d5 = a55 - (l51 * l51 * d1 + l52 * l52 * d2 + l53 * l53 * d3 + l54 * l54 * d4);

    float l65 = (a65 - l51 * l61 * d1 - l52 * l62 * d2 - l53 * l63 * d3 - l54 * l64 * d4) / d5;

    float d6 = a66 - (l61 * l61 * d1 + l62 * l62 * d2 + l63 * l63 * d3 + l64 * l64 * d4 + l65 * l65 * d5);

    // Step 2: Forward substitution: Solve Ly = b
    float y1 = b_lin.x;
    float y2 = b_lin.y - l21 * y1;
    float y3 = b_lin.z - l31 * y1 - l32 * y2;
    float y4 = b_ang.x - l41 * y1 - l42 * y2 - l43 * y3;
    float y5 = b_ang.y - l51 * y1 - l52 * y2 - l53 * y3 - l54 * y4;
    float y6 = b_ang.z - l61 * y1 - l62 * y2 - l63 * y3 - l64 * y4 - l65 * y5;

    // Step 3: Diagonal solve: Solve Dz = y
    float z1 = y1 / d1;
    float z2 = y2 / d2;
    float z3 = y3 / d3;
    float z4 = y4 / d4;
    float z5 = y5 / d5;
    float z6 = y6 / d6;

    // Step 4: Backward substitution: Solve L^T x = z
    (*x_ang).z = z6;
    (*x_ang).y = z5 - l65 * (*x_ang).z;
    (*x_ang).x = z4 - l54 * (*x_ang).y - l64 * (*x_ang).z;
    (*x_lin).z = z3 - l43 * (*x_ang).x - l53 * (*x_ang).y - l63 * (*x_ang).z;
    (*x_lin).y = z2 - l32 * (*x_lin).z - l42 * (*x_ang).x - l52 * (*x_ang).y - l62 * (*x_ang).z;
    (*x_lin).x = z1 - l21 * (*x_lin).y - l31 * (*x_lin).z - l41 * (*x_ang).x - l51 * (*x_ang).y - l61 * (*x_ang).z;
}

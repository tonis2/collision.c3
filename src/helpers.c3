module collision;

import std::math;
import std::io;

alias Number = float;

alias Vec2 @private = Number[<2>];
alias Vec3 @private = Number[<3>];
alias Vec4 @private = Number[<4>];

// fn usz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
// fn usz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s,%s", self.x, self.y, self.z, self.w)!;

fn usz Vec3.hash(&self) {
    usz hash = 17;
    hash = (usz)(hash * 23f + self.x);
    hash = (usz)(hash * 23f + self.y);
    hash = (usz)(hash * 23f + self.z);
    return hash;
}

macro bool Vec3.equals(self, Vec3 b) => self == b;


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

fn Matrix3f Quaternionf.as_mat3(self) {
    Matrix4f matrix = self.to_matrixf();
    return {
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };
}

fn void Vec3.set(&self, Vec3 new) => mem::copy(self, &new, Vec3.sizeof, 0);

fn bool Triangle.intersect_ray(&self, Ray ray, HitInfo* info) {
    const Number EPSILON = 1e-6f;
    
    Vec3 edge1 = self.v1 - self.v0;
    Vec3 edge2 = self.v2 - self.v0;
    Vec3 h = ray.direction.cross(edge2);
    Number a = edge1.dot(h);

    if (math::abs(a) < EPSILON) return false; // Ray is parallel to triangle

    Number f = 1.0f / a;
    Vec3 s = ray.origin - self.v0;
    Number u = f * s.dot(h);


    if (u < 0.0f || u > 1.0f) return false;

    Vec3 q = s.cross(edge1);
    Number v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 2) return false;

    // Compute t to find the intersection point on the ray
    Number t = f * edge2.dot(q);

    if (t > EPSILON) {
        info.depth = t;
        info.normal = edge1.cross(edge2).normalize();
        return true;
    } // ray intersection

    return false;
}

fn Aabb3 Triangle.get_aabb(&self) {
    Aabb3 box = {
        .min = {math::min(self.v0.x, self.v1.x, self.v2.x), math::min(self.v0.y, self.v1.y, self.v2.y), math::min(self.v0.z, self.v1.z, self.v2.z)},
        .max = {math::max(self.v0.x, self.v1.x, self.v2.x), math::max(self.v0.y, self.v1.y, self.v2.y), math::max(self.v0.z, self.v1.z, self.v2.z)}
    };  
    return box;
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("v1 %s, v2 %s v3 %s", self.v0, self.v1, self.v2)!;

fn bool Triangle.collides_aabb(&self, Aabb3 box) {
    Vec3 box_center = (box.min + box.max) * 0.5;
    Vec3 box_extents = box.max - box_center;
    Vec3[3] tri_verts = {
        self.v0 - box_center,
        self.v1 - box_center,
        self.v2 - box_center
    };

    for (int i = 0; i < 3; ++i) {
        if (math::max(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) < -box_extents[i] || math::min(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) > box_extents[i]) {
            return false; // Found a separating axis
        }
    }

    Vec3[3] tri_edges = {
        tri_verts[1] - tri_verts[0],
        tri_verts[2] - tri_verts[1],
        tri_verts[0] - tri_verts[2]
    };

    Vec3 tri_normal = tri_edges[0].cross(tri_edges[1]);
    Number tri_radius = math::abs(tri_verts[0].dot(tri_normal));
    Number box_radius = math::abs(box_extents.x * tri_normal.x) +
                       math::abs(box_extents.y * tri_normal.y) +
                       math::abs(box_extents.z * tri_normal.z);

    if (tri_radius > box_radius) {
        return false;
    }

    Vec3[3] axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    for (uint i = 0; i < 3; ++i) {
        for (uint j = 0; j < 3; ++j) {
            Vec3 axis = tri_edges[i].cross(axes[j]);
            Vec2 tri_interval = tri_verts[0..].get_interval(3, axis);

            Number box_radius_axis = box_extents.x * math::abs(axes[0].dot(axis)) +
                                    box_extents.y * math::abs(axes[1].dot(axis)) +
                                    box_extents.z * math::abs(axes[2].dot(axis));

            if (tri_interval.x > box_radius_axis || tri_interval.y < -box_radius_axis) {
                return false;
            }                         
        }
    } 

    return true;
}

struct Pair (Printable) {
    usz first;
    usz second;
}

fn usz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.first, self.second)!;
macro bool Pair.equals(self, Pair b) @operator(==) => self.first == b.first && self.second == b.second;
macro uint Pair.hash(Pair self) => builtin::@generic_hash(self.first) + builtin::@generic_hash(self.second);

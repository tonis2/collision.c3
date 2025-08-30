module collision;

import std::math;
import std::io;

alias Vec2f @private = float[<2>];
alias Vec3f @private = float[<3>];
alias Vec4f @private = float[<4>];


fn usz? Vec3f.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;

struct HitInfo {
    bool hits;
    float depth;
    Vec3f normal;
}

fn float radians(float value) => value * (float)math::PI / 180;

fn Quaternionf Quaternionf.mul2(q1, Quaternionf q2) {
    float w = (q1.v.w * q2.v.w) - (q1.v.x * q2.v.x) - (q1.v.y * q2.v.y) - (q1.v.z * q2.v.z);
    float x = (q1.v.x * q2.v.w) + (q1.v.w * q2.v.x) + (q1.v.y * q2.v.z) - (q1.v.z * q2.v.y);
    float y = (q1.v.y * q2.v.w) + (q1.v.w * q2.v.y) + (q1.v.z * q2.v.x) - (q1.v.x * q2.v.z);
    float z = (q1.v.z * q2.v.w) + (q1.v.w * q2.v.z) + (q1.v.x * q2.v.y) - (q1.v.y * q2.v.x);

    return {
        x, y, z, w
    };
}

fn Quaternionf Quaternionf.conjugate(&self) => {-self.v.x, -self.v.y, -self.v.z, self.v.w};

fn Quaternionf Quaternionf.rotate_by_axis_angle(self, Vec3f axis, float angle) {
    Vec3f normal_axis = axis.normalize();
    float half_angle = angle * 0.5;
    float sin_half = math::sin(half_angle);
   
    return {
        normal_axis.x * sin_half,
        normal_axis.y * sin_half,
        normal_axis.z * sin_half,
        math::cos(half_angle)
    };
}

fn void Vec3f.set(&self, Vec3f new) => mem::copy(self, &new, Vec3f.sizeof, 0);

fn Vec3f Vec3f.rotate_by_quat(&self, Quaternionf q) {
    Quaternionf p = {self.x, self.y, self.z, 0};
    Quaternionf result = q.mul2(p).mul2(q.conjugate()); 
    return {result.v.x, result.v.y, result.v.z};
}

fn Vec3f Vec3f.barycentric(self, Vec3f a, Vec3f b, Vec3f c) {
    Vec3f v0 = b - a;
    Vec3f v1 = c - a;
    Vec3f v2 = self - a;

    // Solve for coefficients a,b in v2 = a*v0 + b*v1 (in triangle plane)
    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);

    float denom = d00 * d11 - d01 * d01;

    float inv_den = 1.0f / denom;
    float u = (d11 * d20 - d01 * d21) * inv_den;
    float w = (d00 * d21 - d01 * d20) * inv_den;

    // barycentric weights relative to A,B,C:
    return {
        1.0f - u - w,
        u,
        w,
    };
}

fn bool Triangle.intersect_ray(&self, Ray ray, HitInfo* info) {
    const float EPSILON = 1e-6f;
    
    Vec3f edge1 = self.v1 - self.v0;
    Vec3f edge2 = self.v2 - self.v0;
    Vec3f h = ray.direction.cross(edge2);
    float a = edge1.dot(h);

    if (math::abs(a) < EPSILON) return false; // Ray is parallel to triangle

    float f = 1.0f / a;
    Vec3f s = ray.origin - self.v0;
    float u = f * s.dot(h);


    if (u < 0.0f || u > 1.0f) return false;

    Vec3f q = s.cross(edge1);
    float v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 2) return false;

    // Compute t to find the intersection point on the ray
    float t = f * edge2.dot(q);

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
    Vec3f box_center = (box.min + box.max) * 0.5;
    Vec3f box_extents = box.max - box_center;
    Vec3f[3] tri_verts = {
        self.v0 - box_center,
        self.v1 - box_center,
        self.v2 - box_center
    };

    for (int i = 0; i < 3; ++i) {
        if (math::max(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) < -box_extents[i] || math::min(tri_verts[0][i], tri_verts[1][i], tri_verts[2][i]) > box_extents[i]) {
            return false; // Found a separating axis
        }
    }

    Vec3f[3] tri_edges = {
        tri_verts[1] - tri_verts[0],
        tri_verts[2] - tri_verts[1],
        tri_verts[0] - tri_verts[2]
    };

    Vec3f tri_normal = tri_edges[0].cross(tri_edges[1]);
    float tri_radius = math::abs(tri_verts[0].dot(tri_normal));
    float box_radius = math::abs(box_extents.x * tri_normal.x) +
                       math::abs(box_extents.y * tri_normal.y) +
                       math::abs(box_extents.z * tri_normal.z);

    if (tri_radius > box_radius) {
        return false;
    }

    Vec3f[3] axes = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    for (uint i = 0; i < 3; ++i) {
        for (uint j = 0; j < 3; ++j) {
            Vec3f axis = tri_edges[i].cross(axes[j]);
            Vec2f tri_interval = tri_verts[0..].get_interval(3, axis);

            float box_radius_axis = box_extents.x * math::abs(axes[0].dot(axis)) +
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
macro uint Pair.hash(Pair self) => builtin::@generic_hash(self.first) | builtin::@generic_hash(self.second);

fn Matrix4f to_matrix(Vec3f translation, Vec3f scale, Quaternionf rotation) {
    Matrix4f translation_matrix = {
        1, 0, 0, translation.x,
        0, 1, 0, translation.y,
        0, 0, 1, translation.z,
        0.0, 0.0, 0.0, 1.0,
    };

    Matrix4f scale_matrix = {
        scale.x, 0, 0, 0,
        0, scale.y, 0, 0,
        0, 0, scale.z, 0,
        0.0, 0.0, 0.0, 1.0,
    };
    
   return translation_matrix * rotation.to_matrixf() * scale_matrix;
}

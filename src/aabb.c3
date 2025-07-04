module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

fn float radians(float value) => value * (float)math::PI / 180;

struct Aabb3 (Printable, ConvexShape) {
    Vec3f min;
    Vec3f max;
}

struct Obb3 {
    Vec3f min;
    Vec3f max;
}

fn usz? Aabb3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("min %s max %s", self.min, self.max)!;

fn bool Aabb3.is_empty(self) {
    return self.max.xyz == (Vec3f){0,0,0};
}

fn bool Aabb3.intersectsX(self, Aabb3 target) {
    return self.min.x < target.min.x && self.max.x > target.max.x;
}

fn bool Aabb3.intersectsY(self, Aabb3 target) {
    return self.min.y < target.min.y && self.max.y > target.max.y;
}

fn bool Aabb3.intersectsZ(self, Aabb3 target) {
    return self.min.z < target.min.z && self.max.z > target.max.z;
}

fn bool Aabb3.collides(self, Aabb3 target) {
    return self.min.x <= target.max.x &&
           self.max.x >= target.min.x &&
           self.min.y <= target.max.y &&
           self.max.y >= target.min.y &&
           self.min.z <= target.max.z &&
           self.max.z >= target.min.z;
}

fn bool Aabb3.contains_point(self, Vec3f point) {
    return (point.x >= self.min.x && point.x <= self.max.x) &&
           (point.y >= self.min.y && point.y <= self.max.y) &&
           (point.z >= self.min.z && point.z <= self.max.z);
}

fn void Aabb3.expand_min(&self, Vec3f point) {
    self.min.x = math::min(self.min.x, point.x);
    self.min.y = math::min(self.min.y, point.y);
    self.min.z = math::min(self.min.z, point.z);
}

fn void Aabb3.expand_max(&self, Vec3f point) {
    self.max.x = math::max(self.max.x, point.x);
    self.max.y = math::max(self.max.y, point.y);
    self.max.z = math::max(self.max.z, point.z);
}

fn void Aabb3.expand_by_point(&self, Vec3f point) {
    self.expand_max(point);
    self.expand_min(point);
}

fn Aabb3 Aabb3.scale(self, Vec3f scale) {
    Vec3f center = (self.min + self.max) * 0.5f;
    Vec3f halfExtents = (self.max - self.min) * 0.5f;
    Vec3f newHalfExtents = halfExtents * scale;

    return {
        center - newHalfExtents,
        center + newHalfExtents
    };
}

fn bool Aabb3.intersect(&self, Ray ray) @dynamic {
    Vec3f inv_dir = {1.0 / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z};
    
    float t1 = (self.min.x - ray.origin.x) * inv_dir.x;
    float t2 = (self.max.x - ray.origin.x) * inv_dir.x;
    float t3 = (self.min.y - ray.origin.y) * inv_dir.y;
    float t4 = (self.max.y - ray.origin.y) * inv_dir.y;
    float t5 = (self.min.z - ray.origin.z) * inv_dir.z;
    float t6 = (self.max.z - ray.origin.z) * inv_dir.z;

    float tmin = math::max(math::max(math::min(t1, t2), math::min(t3, t4)), math::min(t5, t6));
    float tmax = math::min(math::min(math::max(t1, t2), math::max(t3, t4)), math::max(t5, t6));

    // if tmax < 0, ray is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0) {
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax) {
        return false;
    }

    return true;
}

fn Aabb3 Aabb3.transform(self, Matrix4f matrix)
{
    Vec3f transform = (Vec3f){matrix.m03, matrix.m13, matrix.m23};

    Vec3f b_min = transform;
    Vec3f b_max = transform;

    Matrix3f new_matrix = {
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };
 
    foreach (uint i, value: new_matrix.m) {
        uint index = i % 3;

        float a = value * self.min[index];
        float b = value * self.max[index];

        b_min[index] += math::min(a, b);
        b_max[index] += math::max(a, b);
    }
    
    return {
        b_min,
        b_max
    };
}

fn Vec3f Aabb3.len(self) => self.max - self.min;
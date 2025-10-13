module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

struct Aabb3 (Printable, ConvexShape) {
    Vec3f min;
    Vec3f max;
}


fn Aabb3 aabb_from_half(Vec3f size) => {-size * 0.5, size * 0.5};

fn Vec3f Aabb3.size(self) => self.max - self.min;
fn Vec3f Aabb3.extent(self) => (self.max - self.min) * 0.5f;
fn Vec3f Aabb3.center(self) => (self.min + self.max) * 0.5f;

fn usz? Aabb3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s", self.min, self.max)!;

fn bool Aabb3.equals(self, Aabb3 b) @operator(==) => self.min == b.min && self.max == b.max;

fn Aabb3 Aabb3.add(self, Vec3f value) @operator(+) {
    return {
        .min = self.min + value,
        .max = self.max + value
    };
}

fn bool Aabb3.is_empty(self) => self.size().sq_magnitude() == 0;

fn bool Aabb3.intersectsX(self, Aabb3 target) {
    return self.min.x < target.min.x && self.max.x > target.max.x;
}

fn void Aabb3.fix(&self) {
    self.min = {math::min(self.min.x, self.max.x), math::min(self.min.y, self.max.y), math::min(self.min.z, self.max.z)};
    self.max = {math::max(self.min.x, self.max.x), math::max(self.min.y, self.max.y), math::max(self.min.z, self.max.z)};
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

fn Aabb3 Aabb3.scale(self, Vec3f scale) @operator(*) {
    Vec3f center = self.center();
    Vec3f half_size = self.extent();

    return {
        center - half_size,
        center + half_size
    };
}

fn Aabb3 Aabb3.transform(self, Vec3f translation = {0, 0, 0}, Vec3f scale = {1, 1, 1}, Quaternionf rotation = {0, 0, 0, 1}, bool skip_rotation = false)
{
    Aabb3 aabb = self + translation;
    Vec3f center = aabb.center();

    Vec3f extent;
    if (!skip_rotation) {
        extent = math::abs(rotation * (aabb.extent() * scale));
    } else {
        extent = aabb.extent() * scale;
    }

    return {
        .min = center - extent,
        .max = center + extent,
    };
}

fn Aabb3 Aabb3.child_bounds(self, uint index) {
    Vec3f c = self.center();
    Vec3f newMin = self.min;
    Vec3f newMax = self.max;

    if (index & 1) {newMin.x = c.x;} else {newMax.x = c.x;}
    if (index & 2) {newMin.y = c.y;} else {newMax.y = c.y;}
    if (index & 4) {newMin.z = c.z;} else {newMax.z = c.z;}

    return { newMin, newMax };
}

fn bool Aabb3.intersect_ray(&self, Ray ray, HitInfo* info = null) {
    float tmin = 0.0;
    float tmax = float.max;

    for (uint i = 0; i < 3; ++i) {
        if (math::abs(ray.direction[i]) < 1e-6) {
            if (ray.origin[i] < self.min[i] || ray.origin[i] > self.max[i]) {
                return false; // Parallel and outside the box
            }
        } else {
            float ood = 1.0 / ray.direction[i];
            float t1 = (self.min[i] - ray.origin[i]) * ood;
            float t2 = (self.max[i] - ray.origin[i]) * ood;
            if (t1 > t2) @swap(t1, t2);
            tmin = math::max(tmin, t1);
            tmax = math::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }
    if (info != null) info.depth = tmin;
    return true;
}

fn Vec3f Aabb3.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f point = {
        (direction.x > 0) ? self.max.x : self.min.x,
        (direction.y > 0) ? self.max.y : self.min.y,
        (direction.z > 0) ? self.max.z : self.min.z
    };

    return point;
}

fn Aabb3 Aabb3.get_aabb(&self) @dynamic => *self;

fn ShapeType Aabb3.get_type(&self) @dynamic => ShapeType.BOX;

fn Matrix3f Aabb3.inertia_tensor(&self, float mass) @dynamic {
    Vec3f size = self.size() * 2;

    float ix = (1 / 12.0f) * mass * (size.y + size.z);
    float iy = (1 / 12.0f) * mass * (size.x + size.z);
    float iz = (1 / 12.0f) * mass * (size.x + size.y);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}
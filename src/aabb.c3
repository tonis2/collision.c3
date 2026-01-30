module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

struct Aabb3 (Printable, CollisionShape) {
    Vec3 min;
    Vec3 max;
}

const NEW_AABB3 = (Aabb3) {.min = {float.max, float.max, float.max}, .max = {-float.max, -float.max, -float.max} };
const EMPTY_AABB3 = (Aabb3) {.min = {0,0,0}, .max = {0,0,0}};

fn Aabb3 aabb_from_half(Vec3 size) => {-size * 0.5, size * 0.5};

fn Vec3 Aabb3.size(self) => self.max - self.min;
fn Vec3 Aabb3.extent(self) => (self.max - self.min) * 0.5f;
fn Vec3 Aabb3.center(self) => (self.min + self.max) * 0.5f;

fn usz? Aabb3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s", self.min, self.max)!;
fn bool Aabb3.equals(self, Aabb3 b) @operator(==) => self.min == b.min && self.max == b.max;

fn Aabb3 Aabb3.add(self, Vec3 value) @operator(+) {
    return {
        .min = self.min + value,
        .max = self.max + value
    };
}

fn bool Aabb3.is_empty(self) => self.size().sq_magnitude() == 0;

fn void Aabb3.fix(&self) {
    self.min = {math::min(self.min.x, self.max.x), math::min(self.min.y, self.max.y), math::min(self.min.z, self.max.z)};
    self.max = {math::max(self.min.x, self.max.x), math::max(self.min.y, self.max.y), math::max(self.min.z, self.max.z)};
}

fn bool Aabb3.collides(self, Aabb3 target) {
    bool overlapX = (self.max.x >= target.min.x) && (self.min.x <= target.max.x);
    bool overlapY = (self.max.y >= target.min.y) && (self.min.y <= target.max.y);
    bool overlapZ = (self.max.z >= target.min.z) && (self.min.z <= target.max.z);
    return overlapX && overlapY && overlapZ;
}

fn bool Aabb3.contains_point(self, Vec3 point) {
    return (point.x >= self.min.x && point.x <= self.max.x) &&
           (point.y >= self.min.y && point.y <= self.max.y) &&
           (point.z >= self.min.z && point.z <= self.max.z);
}

fn void Aabb3.expand_min(&self, Vec3 point) {
    self.min.x = math::min(self.min.x, point.x);
    self.min.y = math::min(self.min.y, point.y);
    self.min.z = math::min(self.min.z, point.z);
}

fn void Aabb3.expand_max(&self, Vec3 point) {
    self.max.x = math::max(self.max.x, point.x);
    self.max.y = math::max(self.max.y, point.y);
    self.max.z = math::max(self.max.z, point.z);
}

fn void Aabb3.expand_by_point(&self, Vec3 point) {
    self.expand_max(point);
    self.expand_min(point);
}

fn void Aabb3.expand_by_aabb(&self, Aabb3 box) {
    self.expand_max(box.max);
    self.expand_min(box.min);
}

fn Aabb3 Aabb3.scale(self, Vec3 scale) @operator(*) {
    Vec3 center = self.center();
    Vec3 half_size = self.extent() * scale;

    return {
        center - half_size,
        center + half_size
    };
}

// Transform AABB from local space to world space
// world = rotation * (local * scale) + translation
fn Aabb3 Aabb3.transform(&self, Vec3 translation = {0, 0, 0}, Vec3 scale = {1, 1, 1}, Quaternionf rotation = {0, 0, 0, 1}, bool skip_rotation = false)
{
    Vec3[8] corners = {
        self.min,
        {self.max.x, self.min.y, self.min.z},
        {self.min.x, self.max.y, self.min.z},
        {self.min.x, self.min.y, self.max.z},
        {self.max.x, self.max.y, self.min.z},
        {self.max.x, self.min.y, self.max.z},
        {self.min.x, self.max.y, self.max.z},
        self.max
    };

    Vec3 new_min = rotation * (corners[0] * scale) + translation;
    Vec3 new_max = rotation * (corners[7] * scale) + translation;

    foreach (corner: corners) {
        Vec3 point = rotation * (corner * scale) + translation;

        new_min.x = math::min(point.x, new_min.x);
        new_min.y = math::min(point.y, new_min.y);
        new_min.z = math::min(point.z, new_min.z);

        new_max.x = math::max(point.x, new_max.x);
        new_max.y = math::max(point.y, new_max.y);
        new_max.z = math::max(point.z, new_max.z);
    }

    return {new_min, new_max};
}

// Transform AABB from world space to local space (inverse of transform)
// local = (inv_rot * (world - translation)) * inv_scale
fn Aabb3 Aabb3.inverse_transform(&self, Vec3 translation = {0, 0, 0}, Vec3 scale = {1, 1, 1}, Quaternionf rotation = {0, 0, 0, 1})
{
    Quaternionf inv_rot = rotation.conjugate();
    Vec3 inv_scale = { 1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z };

    Vec3[8] corners = {
        self.min,
        {self.max.x, self.min.y, self.min.z},
        {self.min.x, self.max.y, self.min.z},
        {self.min.x, self.min.y, self.max.z},
        {self.max.x, self.max.y, self.min.z},
        {self.max.x, self.min.y, self.max.z},
        {self.min.x, self.max.y, self.max.z},
        self.max
    };

    Aabb3 local_aabb = NEW_AABB3;

    foreach (corner : corners) {
        Vec3 local_pt = (inv_rot * (corner - translation)) * inv_scale;
        local_aabb.expand_by_point(local_pt);
    }

    return local_aabb;
}

fn Aabb3 Aabb3.child_bounds(self, uint index) {
    Vec3 c = self.center();
    Vec3 newMin = self.min;
    Vec3 newMax = self.max;

    if (index & 1) {newMin.x = c.x;} else {newMax.x = c.x;}
    if (index & 2) {newMin.y = c.y;} else {newMax.y = c.y;}
    if (index & 4) {newMin.z = c.z;} else {newMax.z = c.z;}

    return { newMin, newMax };
}

fn usz Aabb3.longest_axis(self) {
    Vec3 size = self.size();
    if (size.x > size.y && size.x > size.z) return 0;
    if (size.y > size.z) return 1;
    return 2;
}

fn HitInfo Aabb3.intersect_ray(self, Ray ray) {
    HitInfo response = {.collided = false};

    float tmin = 0.0;
    float tmax = float.max;

    for (uint i = 0; i < 3; ++i) {
        if (math::abs(ray.direction[i]) < 1e-6) {
            if (ray.origin[i] < self.min[i] || ray.origin[i] > self.max[i]) {
                return response; // Parallel and outside the box
            }
        } else {
            float ood = 1.0 / ray.direction[i];
            float t1 = (self.min[i] - ray.origin[i]) * ood;
            float t2 = (self.max[i] - ray.origin[i]) * ood;
            if (t1 > t2) @swap(t1, t2);
            tmin = math::max(tmin, t1);
            tmax = math::min(tmax, t2);
            if (tmin > tmax) return response;
        }
    }

    return {
        .collided = true,
        .depth = tmin
    };
}

fn Vec3 Aabb3.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 point = {
        (direction.x > 0) ? self.max.x : self.min.x,
        (direction.y > 0) ? self.max.y : self.min.y,
        (direction.z > 0) ? self.max.z : self.min.z
    };

    return point;
}

fn Aabb3 Aabb3.get_aabb(&self) @dynamic => *self;

fn ShapeType Aabb3.get_type(&self) @dynamic => ShapeType.BOX;

fn Matrix3f Aabb3.inertia_tensor(&self, float mass) @dynamic {
    Vec3 size = self.size() * self.size();

    float ix = (1 / 12.0f) * mass * (size.y + size.z);
    float iy = (1 / 12.0f) * mass * (size.x + size.z);
    float iz = (1 / 12.0f) * mass * (size.x + size.y);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}
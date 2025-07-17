module collision;

import std::math;
import std::io;

interface ConvexShape
{
    fn Vec3f furthest_point(Vec3f direction);
}

interface RayCast
{
    fn Vec3f intersect(Vec3f direction);
}

struct Ray {
    Vec3f origin;
    Vec3f direction; // Should be a unit vector.
}

struct Triangle {
    Vec3f v0, v1, v2;
}

struct Sphere (ConvexShape) {
    Vec3f center;
    float radius;
}

struct Cylinder (ConvexShape) {
    float height;
    float radius_top;
    float radius_bottom;
}

struct Capsule (ConvexShape) {
    float height;
    float radius_top;
    float radius_bottom;
}

// Shapes
struct TransformedConvex (ConvexShape) {
    Vec3f position;
    Vec3f scale;
    Quaternionf rotation;
    ConvexShape shape;
}

struct ConvexPolyhedron (ConvexShape) {
    Vec3f[] vertices;
}

fn Vec3f TransformedConvex.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f local_direction = direction.rotate_by_quat(self.rotation.conjugate());
    Vec3f point = self.shape.furthest_point(local_direction);
    return point.rotate_by_quat(self.rotation) * self.scale + self.position;
}

// struct TransformedConvex (ConvexShape) {
//     Vec3f position;
//     Vec3f scale;
//     ConvexShape shape;
// }

// fn Vec3f TransformedConvex.furthest_point(&self, Vec3f direction) @dynamic {
//     return self.shape.furthest_point(direction) * self.scale + self.position;
// }

fn Vec3f ConvexPolyhedron.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f furthest_point;
    float max_distance = -float.max;

    foreach (pos : self.vertices) {
        float distance = pos.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = pos;
        }
    }

    return furthest_point;
}

fn Vec3f Ray.intersect(&self, Vec3f direction) {
    // Assuming direction is a unit vector
    Vec3f intersection = self.origin + self.direction * direction;
    return intersection;
}

fn Vec2f Vec3f[].get_interval(self, uint count, Vec3f axis) {
    float min = self[0].dot(axis);
    float max = min;

    for (uint i = 1; i < count; ++i) {
        float value = self[i].dot(axis);
        min = math::min(value, min);
        max = math::max(value, max);
    }

    return {
        min,
        max,
    };
}

fn float Sphere.intersect(&self, Ray ray) @dynamic {
    Vec3f oc = ray.origin - self.center;
    float a = ray.direction.dot(ray.direction);
    float b = 2.0 * oc.dot(ray.direction);
    float c = oc.dot(oc) - self.radius * self.radius;
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return -1.0f; // No intersection
    } else {
        // Return the nearest positive intersection distance
        float t1 = (-b - math::sqrt(discriminant)) / (float)(2.0 * a);
        if (t1 > 0) return t1;
        float t2 = (-b + math::sqrt(discriminant)) / (float)(2.0 * a);
        if (t2 > 0) return t2;
        return -1.0;
    }
}

fn Vec3f Sphere.furthest_point(&self, Vec3f direction) @dynamic {
    return direction.normalize() * self.radius + self.center;
}

fn Vec3f Cylinder.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f dir_xz = {direction.x, 0, direction.z};
    Vec3f result = dir_xz.normalize() * self.height;
    result.y = (direction.y  > 0) ? self.radius_top : self.radius_bottom;
    return result;
}

fn Vec3f Capsule.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f result = direction.normalize() * self.height;
    result.y += (direction.y > 0) ? self.radius_top : self.radius_bottom;
    return result;
}

fn Vec3f Aabb3.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f point = {
        (direction.x > 0) ? self.max.x : self.min.x,
        (direction.y > 0) ? self.max.y : self.min.y,
        (direction.z > 0) ? self.max.z : self.min.z
    };
    return point;
}
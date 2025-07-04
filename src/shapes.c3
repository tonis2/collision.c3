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

struct Sphere {
    Vec3f center;
    float radius;
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

fn Vec3f Aabb3.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f point = {
        (direction.x > 0) ? self.max.x : self.min.x,
        (direction.y > 0) ? self.max.y : self.min.y,
        (direction.z > 0) ? self.max.z : self.min.z
    };
    return point;
}

fn Vec3f TransformedConvex.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f local_direction = direction.rotate_by_quat(self.rotation.conjugate());
    Vec3f point = self.shape.furthest_point(local_direction);
    return point.rotate_by_quat(self.rotation) * self.scale + self.position;
}

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

fn bool Triangle.intersect_ray(&self, Ray ray) @dynamic {
    const float EPSILON = 0.0000001f;

    Vec3f edge1 = self.v1 - self.v0;
    Vec3f edge2 = self.v2 - self.v0;
    Vec3f h = ray.direction.cross(edge2);
    float a = edge1.dot(h);

    if (a > -EPSILON && a < EPSILON) return false; // Ray is parallel to the triangle.

    float f = 1.0f / a;
    Vec3f s = ray.origin - self.v0;
    float u = f * s.dot(h);

    if (u < 0.0f || u > 1.0f) return false;

    Vec3f q = s.cross(edge1);
    float v = f * ray.direction.dot(q);

    if (v < 0.0f || u + v > 1.0f) return false;

    float t = f * edge2.dot(q);
    return t > EPSILON;
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("v1 %s, v2 %s v3 %s", self.v0, self.v1, self.v2)!;

fn Aabb3 Triangle.get_aabb(&self) {
    Aabb3 box = {
        .min = {math::min(self.v0.x, self.v1.x, self.v2.x), math::min(self.v0.y, self.v1.y, self.v2.y), math::min(self.v0.z, self.v1.z, self.v2.z)},
        .max = {math::max(self.v0.x, self.v1.x, self.v2.x), math::max(self.v0.y, self.v1.y, self.v2.y), math::max(self.v0.z, self.v1.z, self.v2.z)}
    };  
    return box;
}

<*
 @require self.len != 0 : `Triangle list is empty`
*>
fn Aabb3 Triangle[].calculate_aabb(self) {   
    Aabb3 box = {
        self[0].v0,
        self[0].v0
    };

    foreach (triangle: self) {
        box.expand_by_point(triangle.v0);
        box.expand_by_point(triangle.v1);
        box.expand_by_point(triangle.v2);
    }

    return box;
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
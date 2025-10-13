module collision;

import std::math;
import std::io;

enum ShapeType {
    SPHERE,
    CAPSULE,
    CYLINDER,
    BOX,
    CONVEX,
}

interface ConvexShape
{
    fn Vec3f furthest_point(Vec3f direction);
    fn Matrix3f inertia_tensor(float mass);
    fn Aabb3 get_aabb();
    fn ShapeType get_type();
}

interface RayCast
{
    fn Vec3f intersect(Vec3f direction);
}

struct Ray (Printable) {
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
    Vec3f translation;
    Vec3f scale;
    Quaternionf rotation;
    Matrix3f inertia;
    ConvexShape shape;
}

struct ConvexPolyhedron (ConvexShape) {
    Vec3f[] vertices;
}

fn TransformedConvex transformed_shape_from(ConvexShape shape = null, Vec3f translation = {0, 0, 0}, Vec3f scale = {1, 1, 1}, Quaternionf rotation = {0, 0, 0, 1}) {
    return {
        .shape = shape,
        .translation = translation,
        .scale = scale,
        .rotation = rotation
    };
}

fn usz? Ray.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("origin %s, direction %s", self.origin, self.direction)!;


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

fn Vec3f TransformedConvex.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f local_direction = self.rotation.conjugate() * direction;
    Vec3f point = self.shape.furthest_point(local_direction);
    return self.rotation * (point * self.scale) + self.translation;
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

fn Vec3f Sphere.furthest_point(&self, Vec3f direction) @dynamic => direction.normalize() * self.radius + self.center;

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

fn Matrix3f TransformedConvex.inertia_tensor(&self, float mass) @dynamic {
    Matrix3f local_inertia = self.shape.inertia_tensor(mass);

    Matrix4f matrix = self.rotation.normalize().to_matrixf();
    
    Matrix3f r = (Matrix3f){
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };

    return r * local_inertia * r.transpose();
}

fn Matrix3f TransformedConvex.world_inertia_from_local(&self, Matrix3f local_inertia) @dynamic {
    Matrix3f world_orient = self.rotation.as_mat3();
    return world_orient * local_inertia * world_orient.transpose();
}

fn Matrix3f ConvexPolyhedron.inertia_tensor(&self, float mass) @dynamic => MATRIX3F_IDENTITY;

fn Matrix3f Sphere.inertia_tensor(&self, float mass) @dynamic {
    float r2 = self.radius * self.radius;
    float i = mass * r2 * 2.0f / 5.0f;
	return {
        i, 0, 0,
        0, i, 0,
        0, 0, i
    };
}

fn Matrix3f Cylinder.inertia_tensor(&self, float mass) @dynamic {
    float r2 = self.radius_bottom * self.radius_top;
    float h2 = self.height * self.height;

    float ix = (mass / 12.0f) * (3 * r2 + h2);
    float iy = (mass / 2.0f) * r2;
    float iz = (mass / 12.0f) * (3 * r2 + h2);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}

fn Matrix3f Capsule.inertia_tensor(&self, float mass) @dynamic {
    float r2 = self.radius_bottom * self.radius_top;
    float h2 = self.height * self.height;

    float ix = (mass / 12.0f) * (3 * r2 + h2);
    float iy = (mass / 2.0f) * r2;
    float iz = (mass / 12.0f) * (3 * r2 + h2);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}

fn ShapeType ConvexPolyhedron.get_type(&self) @dynamic => ShapeType.CONVEX;
fn ShapeType TransformedConvex.get_type(&self) @dynamic => self.shape.get_type();
fn ShapeType Sphere.get_type(&self) @dynamic => ShapeType.SPHERE;
fn ShapeType Capsule.get_type(&self) @dynamic => ShapeType.CAPSULE;
fn ShapeType Cylinder.get_type(&self) @dynamic => ShapeType.CYLINDER;

fn Aabb3 TransformedConvex.get_transformed_aabb(&self) @dynamic => self.shape.get_aabb().transform(self.translation, self.scale, self.rotation);
fn Aabb3 TransformedConvex.get_aabb(&self) @dynamic => self.shape.get_aabb();

fn Aabb3 Sphere.get_aabb(&self) @dynamic => {.min = self.center - self.radius, .max = self.center + self.radius};

fn Aabb3 Capsule.get_aabb(&self) @dynamic {
    float radius_max = math::max(self.radius_top, self.radius_bottom);

    return {
        .min = {-radius_max, 0, -radius_max},
        .max = {radius_max, self.height, radius_max}
    };
}

fn Aabb3 Cylinder.get_aabb(&self) @dynamic {
    float radius_max = math::max(self.radius_top, self.radius_bottom);

    return {
        .min = {-radius_max, 0, -radius_max},
        .max = {radius_max, self.height, radius_max}
    };
}


fn Aabb3 ConvexPolyhedron.get_aabb(&self) @dynamic => {};

fn Vec3f TransformedConvex.get_world_center_of_mass(&self, Vec3f center_of_mass) => self.rotation * center_of_mass + self.translation;
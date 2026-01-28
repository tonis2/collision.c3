module collision;

import std::math;
import std::io;

const float EPSILON @local = 0.0001;

enum ShapeType {
    SPHERE,
    CAPSULE,
    CYLINDER,
    BOX,
    CONVEX,
    TRIANGLE
}

interface CollisionShape
{
    fn Vec3 furthest_point(Vec3 direction);
    fn Aabb3 get_aabb();
    fn ShapeType get_type();
	fn Matrix3f inertia_tensor(float mass) @optional;
    fn void free() @optional;
}

interface RayCast
{
    fn Vec3 intersect(Vec3 direction);
}

struct Ray (Printable) {
    Vec3 origin;
    Vec3 direction; // Should be a unit vector.
}

struct Sphere (CollisionShape) {
    Vec3 center;
    float radius;
}

struct Cylinder (CollisionShape) {
    float height;
    float radius_top;
    float radius_bottom;
}

struct Capsule (CollisionShape) {
    float height;
    float radius_top;
    float radius_bottom;
}

// Shapes
struct TransformedShape (CollisionShape) {
    Vec3 translation;
    Vec3 scale;
    Quaternionf rotation;
    CollisionShape shape;
}

fn usz? Ray.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("origin %s, direction %s", self.origin, self.direction)!;

fn Vec3 Ray.intersect(&self, Vec3 direction) {
    // Assuming direction is a unit vector
    Vec3 intersection = self.origin + self.direction * direction;
    return intersection;
}


fn float Sphere.intersect(&self, Ray ray) @dynamic {
    Vec3 oc = ray.origin - self.center;
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

fn Vec3 TransformedShape.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 local_direction = self.rotation.conjugate() * direction;
    Vec3 point = self.shape.furthest_point(local_direction);
    return self.rotation * (point * self.scale) + self.translation;
}

fn Vec3 Sphere.furthest_point(&self, Vec3 direction) @dynamic => direction.normalize() * self.radius + self.center;

fn Vec3 Cylinder.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 dir_xz = {direction.x, 0, direction.z};
    Vec3 result = dir_xz.normalize() * self.height;
    result.y = (direction.y  > 0) ? self.radius_top : self.radius_bottom;
    return result;
}

fn Vec3 Capsule.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 result = direction.normalize() * self.height;
    result.y += (direction.y > 0) ? self.radius_top : self.radius_bottom;
    return result;
}

fn Matrix3f TransformedShape.inertia_tensor(&self, float mass) @dynamic {
    Matrix3f local_inertia = self.shape.inertia_tensor(mass);

    Matrix4f matrix = self.rotation.normalize().to_matrixf();
    
    Matrix3f r = (Matrix3f){
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };

    return r * local_inertia * r.transpose();
}

fn Matrix3f TransformedShape.world_inertia_from_local(&self, Matrix3f local_inertia) {
    Matrix3f world_orient = self.rotation.as_mat3();
    return world_orient * local_inertia * world_orient.transpose();
}

fn Matrix4f TransformedShape.matrix(self) {
    Matrix4f translation_matrix = {
        1, 0, 0, self.translation.x,
        0, 1, 0, self.translation.y,
        0, 0, 1, self.translation.z,
        0.0, 0.0, 0.0, 1.0,
    };

    Matrix4f scale_matrix = {
        self.scale.x, 0, 0, 0,
        0, self.scale.y, 0, 0,
        0, 0, self.scale.z, 0,
        0.0, 0.0, 0.0, 1.0,
    };
    
    return translation_matrix * self.rotation.to_matrixf() * scale_matrix;
}

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

fn ShapeType TransformedShape.get_type(&self) @dynamic => self.shape.get_type();
fn ShapeType Sphere.get_type(&self) @dynamic => ShapeType.SPHERE;
fn ShapeType Capsule.get_type(&self) @dynamic => ShapeType.CAPSULE;
fn ShapeType Cylinder.get_type(&self) @dynamic => ShapeType.CYLINDER;

fn Aabb3 TransformedShape.get_transformed_aabb(&self) @dynamic => self.shape.get_aabb().transform(self.translation, self.scale, self.rotation);
fn Aabb3 TransformedShape.get_aabb(&self) @dynamic => self.shape.get_aabb();

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



fn Vec3 TransformedShape.get_world_center_of_mass(&self, Vec3 center_of_mass) => self.rotation * center_of_mass + self.translation;

fn float scalar_triple_product(Vec3 a, Vec3 b, Vec3 c) @local {
    // (a x b) . c
    float cross_x = a.y * b.z - a.z * b.y;
    float cross_y = a.z * b.x - a.x * b.z;
    float cross_z = a.x * b.y - a.y * b.x;
    return cross_x * c.x + cross_y * c.y + cross_z * c.z;
}

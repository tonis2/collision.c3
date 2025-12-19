module collision;

import std::math;
import std::io;

const Number EPSILON @local = 0.0001;

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
    fn Matrix3f inertia_tensor(Number mass);
    fn Aabb3 get_aabb();
    fn ShapeType get_type();
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
    Number radius;
}

struct Cylinder (CollisionShape) {
    Number height;
    Number radius_top;
    Number radius_bottom;
}

struct Capsule (CollisionShape) {
    Number height;
    Number radius_top;
    Number radius_bottom;
}

// Shapes
struct TransformedShape (CollisionShape) {
    Vec3 translation;
    Vec3 scale;
    Quaternionf rotation;
    CollisionShape shape;
}

struct ConvexPolyhedron (CollisionShape) {
    Vec3* vertices;
    usz count;
    // usz[] indices;
}

fn usz? Ray.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("origin %s, direction %s", self.origin, self.direction)!;

fn Vec3 Ray.intersect(&self, Vec3 direction) {
    // Assuming direction is a unit vector
    Vec3 intersection = self.origin + self.direction * direction;
    return intersection;
}


fn Number Sphere.intersect(&self, Ray ray) @dynamic {
    Vec3 oc = ray.origin - self.center;
    Number a = ray.direction.dot(ray.direction);
    Number b = 2.0 * oc.dot(ray.direction);
    Number c = oc.dot(oc) - self.radius * self.radius;
    Number discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return -1.0f; // No intersection
    } else {
        // Return the nearest positive intersection distance
        Number t1 = (-b - math::sqrt(discriminant)) / (Number)(2.0 * a);
        if (t1 > 0) return t1;
        Number t2 = (-b + math::sqrt(discriminant)) / (Number)(2.0 * a);
        if (t2 > 0) return t2;
        return -1.0;
    }
}

fn Vec3 TransformedShape.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 local_direction = self.rotation.conjugate() * direction;
    Vec3 point = self.shape.furthest_point(local_direction);
    return self.rotation * (point * self.scale) + self.translation;
}

fn Vec3 ConvexPolyhedron.furthest_point(&self, Vec3 direction) @dynamic {
    Vec3 furthest_point;
    Number max_distance = -Number.max;

    for (usz i = 0; i<self.count; i++) {
        Vec3 point = self.vertices[i];

        Number distance = point.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = point;
        }
        
    }

    // foreach (point: self.vertices) {}

    return furthest_point;
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

fn Matrix3f TransformedShape.inertia_tensor(&self, Number mass) @dynamic {
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

fn Matrix3f ConvexPolyhedron.inertia_tensor(&self, Number mass) @dynamic {
    Matrix3f inertia = MATRIX3F_IDENTITY;

    Number density = 1.0;

    Vec3 c = self.center_of_mass();

    // for (usz i; i < self.indices.len; i += 3) {
    //     Vec3 v1 = self.vertices[self.indices[i]].xyz;
    //     Vec3 v2 = self.vertices[self.indices[i + 1]].xyz;
    //     Vec3 v3 = self.vertices[self.indices[i + 2]].xyz;

    //     Number tetra_mass;
    //     Matrix3f tetra_I_origin = MATRIX3F_IDENTITY;

    //     inertia += compute_origin(v1, v2, v3, density);
    // }

    inertia.m00 -= mass * (c.y * c.y + c.z * c.z);
    inertia.m11 -= mass * (c.x * c.x + c.z * c.z);
    inertia.m22 -= mass * (c.x * c.x + c.y * c.y);

    inertia.m01 -= mass * (-c.x * c.y);
    inertia.m02 -= mass * (-c.x * c.z);
    inertia.m12 -= mass * (-c.y * c.z);
    
    inertia.m10 -= mass * (-c.y * c.x);
    inertia.m20 -= mass * (-c.z * c.x);
    inertia.m21 -= mass * (-c.z * c.y);

    return inertia;
}

fn Matrix3f Sphere.inertia_tensor(&self, Number mass) @dynamic {
    Number r2 = self.radius * self.radius;
    Number i = mass * r2 * 2.0f / 5.0f;
	return {
        i, 0, 0,
        0, i, 0,
        0, 0, i
    };
}

fn Matrix3f Cylinder.inertia_tensor(&self, Number mass) @dynamic {
    Number r2 = self.radius_bottom * self.radius_top;
    Number h2 = self.height * self.height;

    Number ix = (mass / 12.0f) * (3 * r2 + h2);
    Number iy = (mass / 2.0f) * r2;
    Number iz = (mass / 12.0f) * (3 * r2 + h2);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}

fn Matrix3f Capsule.inertia_tensor(&self, Number mass) @dynamic {
    Number r2 = self.radius_bottom * self.radius_top;
    Number h2 = self.height * self.height;

    Number ix = (mass / 12.0f) * (3 * r2 + h2);
    Number iy = (mass / 2.0f) * r2;
    Number iz = (mass / 12.0f) * (3 * r2 + h2);

    return {
        ix, 0, 0,
        0,  iy, 0,
        0, 0,  iz
    };
}

fn ShapeType TransformedShape.get_type(&self) @dynamic => self.shape.get_type();
fn ShapeType ConvexPolyhedron.get_type(&self) @dynamic => ShapeType.CONVEX;
fn ShapeType Sphere.get_type(&self) @dynamic => ShapeType.SPHERE;
fn ShapeType Capsule.get_type(&self) @dynamic => ShapeType.CAPSULE;
fn ShapeType Cylinder.get_type(&self) @dynamic => ShapeType.CYLINDER;

fn Aabb3 TransformedShape.get_transformed_aabb(&self) @dynamic => self.shape.get_aabb().transform(self.translation, self.scale, self.rotation);
fn Aabb3 TransformedShape.get_aabb(&self) @dynamic => self.shape.get_aabb();

fn Aabb3 Sphere.get_aabb(&self) @dynamic => {.min = self.center - self.radius, .max = self.center + self.radius};

fn Aabb3 Capsule.get_aabb(&self) @dynamic {
    Number radius_max = math::max(self.radius_top, self.radius_bottom);

    return {
        .min = {-radius_max, 0, -radius_max},
        .max = {radius_max, self.height, radius_max}
    };
}

fn Aabb3 Cylinder.get_aabb(&self) @dynamic {
    Number radius_max = math::max(self.radius_top, self.radius_bottom);

    return {
        .min = {-radius_max, 0, -radius_max},
        .max = {radius_max, self.height, radius_max}
    };
}


fn Aabb3 ConvexPolyhedron.get_aabb(&self) @dynamic {
    Vec3 min = {float.max, float.max, float.max};
    Vec3 max = {-float.max, -float.max, -float.max};


    for (usz i = 0; i<self.count; i++) {
        Vec3 point = self.vertices[i];

        min.x = math::min(point.x, min.x);
        min.y = math::min(point.y, min.y);
        min.z = math::min(point.z, min.z);

        max.x = math::max(point.x, max.x);
        max.y = math::max(point.y, max.y);
        max.z = math::max(point.z, max.z);
    }

    return {
        min,
        max
    };
}

fn Vec3 TransformedShape.get_world_center_of_mass(&self, Vec3 center_of_mass) => self.rotation * center_of_mass + self.translation;

fn Number scalar_triple_product(Vec3 a, Vec3 b, Vec3 c) @local {
    // (a x b) . c
    Number cross_x = a.y * b.z - a.z * b.y;
    Number cross_y = a.z * b.x - a.x * b.z;
    Number cross_z = a.x * b.y - a.y * b.x;
    return cross_x * c.x + cross_y * c.y + cross_z * c.z;
}

fn Vec3 ConvexPolyhedron.center_of_mass(self) {
    Number total_detJ = 0;
    Vec3 weighted_com = {0, 0, 0};
    Vec3 final_com = {0, 0, 0};

    // for (usz i; i < self.indices.len; i += 3) {
    //     Vec3 v1 = self.vertices[self.indices[i]].xyz;
    //     Vec3 v2 = self.vertices[self.indices[i + 1]].xyz;
    //     Vec3 v3 = self.vertices[self.indices[i + 2]].xyz;

    //     total_detJ += scalar_triple_product(v1, v2, v3);

    //     weighted_com.x += total_detJ * (v1.x + v2.x + v3.x);
    //     weighted_com.y += total_detJ * (v1.y + v2.y + v3.y);
    //     weighted_com.z += total_detJ * (v1.z + v2.z + v3.z);
    // }
    
    if (math::abs(total_detJ) > EPSILON) final_com = 0.25 * weighted_com / total_detJ;

    return final_com;
}

fn Matrix3f compute_origin(Vec3 a, Vec3 b, Vec3 c, Number density) @local {
    Matrix3f inertia_tensor;

    Number detJ = scalar_triple_product(a, b, c);

    Number v_mult_diag = (density * detJ) / (Number)60.0;
    Number v_mult_off_diag = (density * detJ) / (Number)120.0;

    // Diagonal elements (Moments of Inertia)
    inertia_tensor.m00 = v_mult_diag * (
        (a.y*a.y + a.y*b.y + b.y*b.y + a.y*c.y + b.y*c.y + c.y*c.y) +
        (a.z*a.z + a.z*b.z + b.z*b.z + a.z*c.z + b.z*c.z + c.z*c.z)
    );

    inertia_tensor.m11 = v_mult_diag * (
        (a.x*a.x + a.x*b.x + b.x*b.x + a.x*c.x + b.x*c.x + c.x*c.x) +
        (a.z*a.z + a.z*b.z + b.z*b.z + a.z*c.z + b.z*c.z + c.z*c.z)
    );

    inertia_tensor.m22 = v_mult_diag * (
        (a.x*a.x + a.x*b.x + b.x*b.x + a.x*c.x + b.x*c.x + c.x*c.x) +
        (a.y*a.y + a.y*b.y + b.y*b.y + a.y*c.y + b.y*c.y + c.y*c.y)
    );

    inertia_tensor.m01 = -v_mult_off_diag * (
        (2*a.x*a.y + b.x*a.y + c.x*a.y) + (a.x*b.y + 2*b.x*b.y + c.x*b.y) + (a.x*c.y + b.x*c.y + 2*c.x*c.y)
    );

    inertia_tensor.m02 = -v_mult_off_diag * (
        (2*a.x*a.z + b.x*a.z + c.x*a.z) + (a.x*b.z + 2*b.x*b.z + c.x*b.z) + (a.x*c.z + b.x*c.z + 2*c.x*c.z)
    );

    inertia_tensor.m12 = -v_mult_off_diag * (
        (2*a.y*a.z + b.y*a.z + c.y*a.z) + (a.y*b.z + 2*b.y*b.z + c.y*b.z) + (a.y*c.z + b.y*c.z + 2*c.y*c.z)
    );

    inertia_tensor.m10 = inertia_tensor.m01;
    inertia_tensor.m20 = inertia_tensor.m02;
    inertia_tensor.m21 = inertia_tensor.m12;

    return inertia_tensor;
}


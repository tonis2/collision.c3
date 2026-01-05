module collision;

import std::math;
import std::io;
import std::collections::list;

alias Number = float;

alias Vec2 @private = Number[<2>];
alias Vec4 @private = Number[<4>];
alias Vec3 @private = Number[<3>];
alias ContactPoints @private = Vec3[2];
alias QuaternionValue = QuaternionNumber{Number};

fn usz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
fn usz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;

alias Triangle = Vec3[3];

struct CollisionInfo {
    Simplex simplex;
    Vec3 normal;
    ContactPoints contact_points;
    Number depth;
    bool collided;
}

struct HitInfo {
    bool collided;
    Number depth;
    Vec3 normal;
    Triangle triangle;
}

struct Plane {
    Vec3 point;
    Vec3 normal;
	Number d;
}

struct Face @private {
    usz a,b,c;
    Vec3 normal;
    Number distance;
}

struct Pair (Printable) {
    usz first;
    usz second;
}

fn Matrix3f QuaternionValue.as_mat3(self) {
    Matrix4f matrix = self.to_matrixf();
    return {
        matrix.m00, matrix.m01, matrix.m02,
        matrix.m10, matrix.m11, matrix.m12,
        matrix.m20, matrix.m21, matrix.m22,
    };
}

fn void Vec3.set(&self, Vec3 new) @private => mem::copy(self, &new, Vec3.sizeof, 0);

fn usz Vec3.hash(&self) @private {
    usz hash = 17;
    hash = (usz)(hash * 23f + self.x);
    hash = (usz)(hash * 23f + self.y);
    hash = (usz)(hash * 23f + self.z);
    return hash;
}

fn Number Vec3.distance_sq(self, Vec3 v2) {
    Vec3 dist = self - v2;
    return dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;
}

fn usz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.first, self.second)!;
fn usz? ContactPoints.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s", (*self)[0], (*self)[1])!;

macro bool Pair.equals(self, Pair b) @operator(==) => self.first == b.first && self.second == b.second;
macro uint Pair.hash(&self) {
    if (self.first > self.second) @swap(self.first, self.second);
    return builtin::@generic_hash(self.first) + builtin::@generic_hash(self.second);
}

macro bool Vec3.equals(self, Vec3 b) => self == b;
macro bool QuaternionValue.equals(self, QuaternionValue b) @operator(==) => self.v.xyzw == b.v.xyzw;

fn Plane Triangle.get_plane(&self) {
	Vec3 p0 = (*self)[0];
	Vec3 p1 = (*self)[1];
	Vec3 p2 = (*self)[2];

	Vec3 edge1 = {p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
	Vec3 edge2 = {p2.x - p0.x, p2.y - p0.y, p2.z - p0.z};
	Vec3 normal = {
		edge1.y * edge2.z - edge1.z * edge2.y,
		edge1.z * edge2.x - edge1.x * edge2.z,
		edge1.x * edge2.y - edge1.y * edge2.x
	};

	return {
		.normal = normal.normalize(),
		.point = p0,
	};
}

fn Aabb3 Triangle.get_aabb(&self) @dynamic {
    Vec3[3] triangle = *self;
    return {
        .min = {math::min(triangle[0].x, triangle[1].x, triangle[2].x), math::min(triangle[0].y, triangle[1].y, triangle[2].y), math::min(triangle[0].z, triangle[1].z, triangle[2].z)},
        .max = {math::max(triangle[0].x, triangle[1].x, triangle[2].x), math::max(triangle[0].y, triangle[1].y, triangle[2].y), math::max(triangle[0].z, triangle[1].z, triangle[2].z)}
    };
}

fn usz? Triangle.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s\n%s", (*self)[0], (*self)[1], (*self)[2])!;


fn Number Plane.get_distance_to_point(plane, Vec3 point) {
    Vec3 v = {point.x - plane.point.x, point.y - plane.point.y, point.z - plane.point.z};
    return (v.x * plane.normal.x + v.y * plane.normal.y + v.z * plane.normal.z);	
}

fn bool Plane.contains_point(&plane, Vec3 point) {
    return (point.dot(plane.normal) + plane.d) >= 0;
}

fn Vec3 Plane.intersects(plane, Vec3 a, Vec3 b) {
	Number da = a.dot(plane.normal) + plane.d;
	Number db = b.dot(plane.normal) + plane.d;
	Number t = da / (da - db);
	return {
		a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z)
	};
}

<*
 @require in_points.len() > 0 : `in points should not be empty`
*>
fn void Plane.clip_againt(&plane, List{Vec3}* in_points, List{Vec3}* out_points) {
   Vec3 s = in_points.get(in_points.len() - 1);

   for (usz i = 0; i < in_points.len(); i++) {
		Vec3 e = in_points.get(i);

		if (plane.contains_point(e)) {
			if (!plane.contains_point(s)) {
				out_points.push(plane.intersects(s, e));
			}
			out_points.push(e);
		}
		else if (plane.contains_point(s)) {
			out_points.push(plane.intersects(s, e));
		}

		s = e;
   }
}

module collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec2 @private = float[<2>];
alias Vec4 @private = float[<4>];
alias Vec3 @private = float[<3>];
alias ContactPoints @private = Vec3[2];
alias QuaternionValue = QuaternionNumber {float};
alias Vertices = List {Vec3};

fn sz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
fn sz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;

struct CollisionInfo
{
	Simplex simplex;
	Vec3 normal;
	ContactPoints contact_points;
	float depth;
	bool collided;
}

struct HitInfo
{
	bool collided;
	float depth;
	Vec3 normal;
	TriangleVerts triangle;
}

struct Face @private
{
	sz a, b, c;
	Vec3 normal;
	float distance;
	bool active;
	bool visible;
}

struct Pair (Printable)
{
	sz a;
	sz b;
}

struct MinMax
{
	float min;
	float max;
}

fn MinMax min_max_from_axis(Vec3 axis, Vec3[] points)
{
	MinMax corners = { .min = axis.dot(points[0]), .max = axis.dot(points[0]) };

	foreach (point : points) corners.expand(axis.dot(point));

	return corners;
}

fn void MinMax.expand(&self, float value)
{
	self.min = math::min(self.min, value);
	self.max = math::max(self.max, value);
}

fn void MinMax.expand_by_array(&self, float[] values)
{
	foreach (value : values) self.expand(value);
}

fn MinMax min_max_from_box(Vec3 center, Vec3 extent, Vec3 axis)
{
	float c = center.dot(axis);
	float r = math::abs(axis.x) * extent.x + math::abs(axis.y) * extent.y + math::abs(axis.z) * extent.z;

	return { .min = c - r, .max = c + r };
}

fn float MinMax.overlap(&a, MinMax b)
{
	if (a.max < b.min || b.max < a.min)
	{
		// Separated
		return -1;
	}

	return math::min(a.max, b.max) - math::max(a.min, b.min);
}

fn sz? MinMax.to_format(
	&self,
	Formatter* formatter
) @dynamic => formatter.printf("min %s max %s", self.min, self.max)!;

fn Matrix3f QuaternionValue.as_mat3(self)
{
	Matrix4f matrix = self.to_matrix();
	return {
		matrix.m00, matrix.m10, matrix.m20,  // column 0
		matrix.m01, matrix.m11, matrix.m21,  // column 1
		matrix.m02, matrix.m12, matrix.m22,  // column 2
	};
}

fn void Vec3.set(&self, Vec3 new) => mem::copy(self, &new, Vec3::size, 0);

fn float Vec3.distance_sq(self, Vec3 v2) => (self - v2).length_sq();

fn sz? ContactPoints.to_format(
	&self,
	Formatter* formatter
) @dynamic => formatter.printf("%s\n%s", (*self)[0], (*self)[1])!;
fn sz? Pair.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s, %s", self.a, self.b)!;
macro bool Pair.equals(a, Pair b) @operator(==) => (a.a == b.a && a.b == b.b) || (a.a == b.b && a.b == b.a);

macro uint Pair.hash(self)
{
	// Order-normalized (Pair.equals is unordered) without mutating self.
	// Distribution only: exactness comes from Pair.equals inside the map.
	ulong lo = (ulong)math::min(self.a, self.b);
	ulong hi = (ulong)math::max(self.a, self.b);
	ulong h = (lo * 0x9E3779B97F4A7C15) ^ (hi * 0xC2B2AE3D27D4EB4F);
	return (uint)(h ^ (h >> 32));
}

// Removed: QuaternionValue.equals is now provided by std::math::quaternion

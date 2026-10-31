module collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec4 @private = float[<4>];
alias Vec3 @private = float[<3>];
alias ContactPoints @private = Vec3[2];
alias QuaternionValue = QuaternionNumber {float};

fn sz? Vec3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s", self.x, self.y, self.z)!;
fn sz? Vec4.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s,%s,%s,%s", self.x, self.y, self.z, self.w)!;

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

<*
 The 3x3 rotation matrix of a quaternion (normalised first), top-left block of
 `to_matrix()`. Kept on `to_matrix` rather than `rows()`: the two differ in the
 last float bits and the bench scenes are sensitive to that.
*>
fn Matrix3f QuaternionValue.as_mat3(self)
{
	Matrix4f m = self.to_matrix();
	return { m.m00, m.m10, m.m20, m.m01, m.m11, m.m21, m.m02, m.m12, m.m22 };
}

<*
 `R * inertia * Rᵀ` for the rotation matrix of `rotation`: a body-space inertia
 tensor expressed in world axes. The one place a local tensor is rotated.
*>
fn Matrix3f rotate_inertia(Matrix3f local_inertia, Quaternionf rotation)
{
	Matrix3f r = rotation.as_mat3();
	return r * local_inertia * r.transpose();
}

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


<*
 Outcome of a point-vs-surface query: whether it hit, where, which way out, and
 how deep. `hit == false` leaves the rest zeroed.
*>
struct ContactHit
{
	bool hit;
	Vec3 point;
	Vec3 normal;
	float depth;
}

<*
 Keep whichever of the two is deeper; a miss never replaces anything.
*>
fn void ContactHit.keep_deeper(&self, ContactHit other)
{
	if (other.hit && (!self.hit || other.depth > self.depth)) *self = other;
}

<*
 True when all three components are finite (no NaN or infinity).
*>
fn bool is_finite3(Vec3 v) => math::is_finite(v.x) && math::is_finite(v.y) && math::is_finite(v.z);

<*
 True when `self` and `o` are within `eps` of each other (a real distance, not squared).
*>
fn bool Vec3.near(self, Vec3 o, float eps) => self.distance_sq(o) <= eps * eps;

<*
 Some unit vector perpendicular to the unit vector `unit`. Crosses with the
 world axis `unit` leans on least, which keeps the result well away from zero length.
*>
fn Vec3 any_perpendicular(Vec3 unit)
{
	Vec3 aside = math::abs(unit.x) < math::abs(unit.z) ? (Vec3){ 1, 0, 0 } : (Vec3){ 0, 0, 1 };
	return unit.cross(aside).normalize();
}

<*
 The angle in [0, pi] from unit vector `a` to unit vector `b`, and the unit axis
 that turns `a` onto `b` through it (written to `axis`). Uses dmath::atan2 of
 the cross length and dot, so it is accurate over the whole range and the same on
 every platform. For parallel or antiparallel inputs the axis is any perpendicular of `a`.

 @param [&out] axis
*>
fn float angle_axis_between(Vec3 a, Vec3 b, Vec3* axis)
{
	Vec3 cross = a.cross(b);
	float across = cross.length();
	float angle = dmath::atan2(across, math::clamp(a.dot(b), -1.0f, 1.0f));
	*axis = across > 1e-7f ? cross / across : any_perpendicular(a);
	return angle;
}

<*
 The point of `pts` furthest along `dir` (first one wins ties). Returns the zero
 vector for an empty slice.
*>
fn Vec3 support_of(Vec3[] pts, Vec3 dir)
{
	if (pts.len == 0) return { 0, 0, 0 };

	Vec3 best = pts[0];
	float best_dot = best.dot(dir);
	foreach (p : pts[1..])
	{
		float d = p.dot(dir);
		if (d > best_dot)
		{
			best_dot = d;
			best = p;
		}
	}
	return best;
}

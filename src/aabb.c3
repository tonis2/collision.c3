module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::fixedlist;
import std::collections::map;

struct Aabb3 (Printable, CollisionShape)
{
	Vec3 min;
	Vec3 max;
}

const NEW_AABB3   = (Aabb3) {
	.min = { float::max, float::max, float::max },
	.max = { -float::max, -float::max, -float::max }
};
const EMPTY_AABB3 = (Aabb3) { .min = { 0, 0, 0 }, .max = { 0, 0, 0 } };

fn Aabb3 aabb_from_half(Vec3 size) => { -size * 0.5, size * 0.5 };

fn Vec3 Aabb3.size(self) => self.max - self.min;
fn Vec3 Aabb3.extent(self) => (self.max - self.min) * 0.5f;
fn Vec3 Aabb3.center(self) => (self.min + self.max) * 0.5f;

fn sz? Aabb3.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s\n%s", self.min, self.max)!;
fn bool Aabb3.equals(self, Aabb3 b) @operator(==) => self.min == b.min && self.max == b.max;

fn Aabb3 Aabb3.add(self, Vec3 value) @operator(+)
{
	return { .min = self.min + value, .max = self.max + value };
}

fn bool Aabb3.is_empty(self) => self.size().length_sq() == 0;

fn void Aabb3.fix(&self)
{
	Vec3 lo = math::min(self.min, self.max);
	self.max = math::max(self.min, self.max);
	self.min = lo;
}

fn bool Aabb3.collides(self, Aabb3 target) => self.max.comp_ge(target.min).and() && self.min.comp_le(target.max).and();

fn bool Aabb3.contains_point(self, Vec3 point) => point.comp_ge(self.min).and() && point.comp_le(self.max).and();

fn void Aabb3.expand_min(&self, Vec3 point) => self.min = math::min(self.min, point);

fn void Aabb3.expand_max(&self, Vec3 point) => self.max = math::max(self.max, point);

fn void Aabb3.expand_by_point(&self, Vec3 point)
{
	self.expand_max(point);
	self.expand_min(point);
}

fn void Aabb3.expand_by_aabb(&self, Aabb3 box)
{
	self.expand_max(box.max);
	self.expand_min(box.min);
}

// The box pushed out by `by` on every side, in the box's own units.
fn Aabb3 Aabb3.grow(self, float by)
{
	return { .min = self.min - { by, by, by }, .max = self.max + { by, by, by } };
}

fn Aabb3 Aabb3.scale(self, Vec3 scale) @operator(*)
{
	Vec3 center    = self.center();
	Vec3 half_size = self.extent() * scale;

	return { center - half_size, center + half_size };
}

<*
 The three rows of the rotation matrix a quaternion stands for.

 Deliberately *not* normalised (unlike `Quaternionf.to_matrix`): `q * v * q.conjugate()`
 scales its result by `|q|²`, and so does this, so the two agree for any
 quaternion. Nothing promises colliders carry normalised rotations.
*>
fn Vec3[3] Quaternionf.rows(self)
{
	float x = self.v.x;
	float y = self.v.y;
	float z = self.v.z;
	float w = self.v.w;

	float xx = x * x;
	float yy = y * y;
	float zz = z * z;
	float ww = w * w;

	float xy = x * y;
	float xz = x * z;
	float yz = y * z;

	float wx = w * x;
	float wy = w * y;
	float wz = w * z;

	return {
		{ ww + xx - yy - zz, 2 * (xy - wz),     2 * (xz + wy)     },
		{ 2 * (xy + wz),     ww - xx + yy - zz, 2 * (yz - wx)     },
		{ 2 * (xz - wy),     2 * (yz + wx),     ww - xx - yy + zz },
	};
}

<*
 The bound of a box under a linear map, given that map's rows.

 Shared by the three entry points below. The centre goes through the map; the
 extent goes through its *absolute value*, which keeps the result correct under
 a reflection as well as a rotation. `extent` must be non-negative (callers
 take `abs`, since an inverted box can reach this library).
*>
fn Aabb3 bound_from_rows(Vec3[3] rows, Vec3 centre, Vec3 extent, Vec3 translation) @local
{
	Vec3 c = (Vec3){ rows[0].dot(centre), rows[1].dot(centre), rows[2].dot(centre) } + translation;
	Vec3 e = {
		math::abs(rows[0]).dot(extent),
		math::abs(rows[1]).dot(extent),
		math::abs(rows[2]).dot(extent),
	};

	return { .min = c - e, .max = c + e };
}

<*
 A box in world space, given a local box and the transform that places it.

 	world = rotation * (local * scale) + translation

 ## Centre and extent, not eight corners

 The bound of an affinely transformed box is exact in closed form: the new
 centre is the old centre through the transform, and the new extent along each
 world axis is the *absolute* row of the linear part dotted with the old extent.
 That is one matrix build and six dot products, about 13x faster than rotating
 eight corners (77 ns vs 5.9 ns at -O3), and it runs for every body every step.

 The scale is folded into the centre and extent rather than the rows, which is
 exact for negative or non-uniform scale and for an inverted input box.

 `skip_rotation` skips the rotation and applies only scale and translation.
*>
fn Aabb3 Aabb3.transform(
	&self,
	Vec3 translation = { 0, 0, 0 },
	Vec3 scale = { 1, 1, 1 },
	Quaternionf rotation = { 0, 0, 0, 1 },
	bool skip_rotation = false
)
{
	Vec3 centre = self.center() * scale;
	Vec3 extent = math::abs(self.extent() * scale);

	if (skip_rotation) return { .min = centre + translation - extent, .max = centre + translation + extent };

	return bound_from_rows(rotation.rows(), centre, extent, translation);
}

<*
 A box in local space, given a world box and the transform that placed it.

 	local = (inv_rot * (world - translation)) * inv_scale

 The same closed form as `transform`, through the transpose: the matrix of a
 quaternion's conjugate is the transpose of the matrix of the quaternion, so the
 columns read here are the rows built there and no second build is needed.

 This is the tightest local-space box containing the *world box*, not the box
 you started from before `transform`: rotating a bound and back grows it. Callers
 such as the mesh broadphase rely on it being conservative rather than exact.
*>
fn Aabb3 Aabb3.inverse_transform(
	&self,
	Vec3 translation = { 0, 0, 0 },
	Vec3 scale = { 1, 1, 1 },
	Quaternionf rotation = { 0, 0, 0, 1 }
)
{
	Vec3 inv_scale = (Vec3){ 1, 1, 1 } / scale;
	Vec3[3] r = rotation.rows();

	// The rows of `inv_scale * transpose(r)` — the transpose because a
	// quaternion's conjugate has the transpose of its matrix, and the scale
	// folded into the rows because here it lands after the rotation rather than
	// before it.
	Vec3[3] rows = {
		(Vec3){ r[0][0], r[1][0], r[2][0] } * inv_scale.x,
		(Vec3){ r[0][1], r[1][1], r[2][1] } * inv_scale.y,
		(Vec3){ r[0][2], r[1][2], r[2][2] } * inv_scale.z,
	};

	return bound_from_rows(rows, self.center() - translation, math::abs(self.extent()), (Vec3){ 0, 0, 0 });
}

<*
 A box in world space, given a local box and the 4×4 that places it.

 The same closed form as `transform`, reading the linear part's rows straight
 off a composed world matrix. Column-major, translation at `m[12..14]`.

 Exact for any affine matrix (rotation, non-uniform scale, reflection, shear).
 Not correct for a projective matrix: the perspective row is ignored.
*>
fn Aabb3 Aabb3.transform_matrix(&self, Matrix4f m)
{
	float[<16>] v = m.m;
	Vec3[3] rows = {
		{ v[0], v[4], v[8]  },
		{ v[1], v[5], v[9]  },
		{ v[2], v[6], v[10] },
	};

	return bound_from_rows(rows, self.center(), math::abs(self.extent()), (Vec3){ v[12], v[13], v[14] });
}

<*
 Octant `index` of the box: bit 0 selects the upper half in x, bit 1 in y, bit 2 in z.
*>
fn Aabb3 Aabb3.child_bounds(self, uint index)
{
	Vec3 c = self.center();
	bool[<3>] upper = { (index & 1) != 0, (index & 2) != 0, (index & 4) != 0 };
	return { .min = math::select(upper, c, self.min), .max = math::select(upper, self.max, c) };
}

<*
 Total area of the six faces.
*>
fn float Aabb3.surface_area(self)
{
	Vec3 s = self.size();
	return 2.0f * (s.x * s.y + s.y * s.z + s.z * s.x);
}

<*
 The eight corners, in the order: min, +x, +y, +z, +xy, +xz, +yz, max
 (each "+a" is min with the max value on axis a).
*>
fn Vec3[8] Aabb3.corners(self)
{
	return {
		self.min,
		{ self.max.x, self.min.y, self.min.z },
		{ self.min.x, self.max.y, self.min.z },
		{ self.min.x, self.min.y, self.max.z },
		{ self.max.x, self.max.y, self.min.z },
		{ self.max.x, self.min.y, self.max.z },
		{ self.min.x, self.max.y, self.max.z },
		self.max,
	};
}

<*
 The tightest box around `pts`. An empty slice gives `NEW_AABB3` (inverted, so
 any later `expand_by_point` fixes it).
*>
fn Aabb3 aabb_of_points(Vec3[] pts)
{
	Aabb3 box = NEW_AABB3;
	foreach (p : pts) box.expand_by_point(p);
	return box;
}

fn sz Aabb3.longest_axis(self)
{
	Vec3 size = self.size();
	if (size.x > size.y && size.x > size.z) return 0;
	if (size.y > size.z) return 1;
	return 2;
}

fn HitInfo Aabb3.intersect_ray(self, Ray ray)
{
	HitInfo response = { .collided = false };

	float tmin = 0.0;
	float tmax = float::max;

	for (uint i = 0; i < 3; ++i)
	{
		if (math::abs(ray.direction[i]) < 1e-6)
		{
			if (ray.origin[i] < self.min[i] || ray.origin[i] > self.max[i])
			{
				return response; // Parallel and outside the box
			}
		}
		else
		{
			float ood = 1.0 / ray.direction[i];
			float t1  = (self.min[i] - ray.origin[i]) * ood;
			float t2  = (self.max[i] - ray.origin[i]) * ood;
			if (t1 > t2) @swap(t1, t2);
			tmin = math::max(tmin, t1);
			tmax = math::min(tmax, t2);
			if (tmin > tmax) return response;
		}
	}

	return { .collided = true, .depth = tmin };
}

fn Vec3 Aabb3.furthest_point(&self, Vec3 direction) @dynamic
{
	Vec3 point = {
		(direction.x > 0) ? self.max.x : self.min.x,
		(direction.y > 0) ? self.max.y : self.min.y,
		(direction.z > 0) ? self.max.z : self.min.z
	};

	return point;
}

fn Aabb3 Aabb3.get_aabb(&self) @dynamic => *self;

fn ShapeType Aabb3.get_type(&self) @dynamic => ShapeType.BOX;

fn Matrix3f Aabb3.inertia_tensor(&self, float mass) @dynamic
{
	Vec3 size = self.size() * self.size();

	float ix = (1 / 12.0f) * mass * (size.y + size.z);
	float iy = (1 / 12.0f) * mass * (size.x + size.z);
	float iz = (1 / 12.0f) * mass * (size.x + size.y);

	return { ix, 0, 0, 0, iy, 0, 0, 0, iz };
}

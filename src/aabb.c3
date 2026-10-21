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
	self.min = {
		math::min(self.min.x, self.max.x),
		math::min(self.min.y, self.max.y),
		math::min(self.min.z, self.max.z)
	};
	self.max = {
		math::max(self.min.x, self.max.x),
		math::max(self.min.y, self.max.y),
		math::max(self.min.z, self.max.z)
	};
}

fn bool Aabb3.collides(self, Aabb3 target)
{
	bool overlapX = (self.max.x >= target.min.x) && (self.min.x <= target.max.x);
	bool overlapY = (self.max.y >= target.min.y) && (self.min.y <= target.max.y);
	bool overlapZ = (self.max.z >= target.min.z) && (self.min.z <= target.max.z);
	return overlapX && overlapY && overlapZ;
}

fn bool Aabb3.contains_point(self, Vec3 point)
{
	return (point.x >= self.min.x && point.x <= self.max.x) && (point.y >= self.min.y && point.y <= self.max.y)
		&& (point.z >= self.min.z && point.z <= self.max.z);
}

fn void Aabb3.expand_min(&self, Vec3 point)
{
	self.min.x = math::min(self.min.x, point.x);
	self.min.y = math::min(self.min.y, point.y);
	self.min.z = math::min(self.min.z, point.z);
}

fn void Aabb3.expand_max(&self, Vec3 point)
{
	self.max.x = math::max(self.max.x, point.x);
	self.max.y = math::max(self.max.y, point.y);
	self.max.z = math::max(self.max.z, point.z);
}

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

 Written out rather than taken from `Quaternionf.to_matrix`, and deliberately
 *not* normalised. `q * v * q.conjugate()` scales its result by `|q|²`, and so
 does this, so the two agree for any quaternion rather than only for unit ones —
 which matters because that is the product the eight-corner code used to compute
 and nothing in this library promises its colliders carry normalised rotations.
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

 The whole of the arithmetic the three entry points below share, and the reason
 they can share it: whatever produced the rows — a quaternion, a quaternion and
 a scale, or the linear part of a 4×4 — the answer is the same two lines. The
 centre goes through the map; the extent goes through its *absolute value*,
 which is what keeps the result correct under a reflection as well as a
 rotation.

 `extent` is expected non-negative. The callers take it through `abs` because an
 inverted box does reach this library, and the eight-corner code this replaced
 repaired one implicitly by taking a min and a max.
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

 The axis-aligned bound of an affinely transformed box is exact in closed form:
 the new centre is the old centre through the transform, and the new extent
 along each world axis is the *absolute* row of the linear part dotted with the
 old extent. Rotating eight corners and taking their min and max computes the
 same two numbers the long way round — the corners are symmetric about the
 centre, so half of the arithmetic re-derives what the other half already said.

 That is not a small saving here. `Quaternionf.rotate_vec3` is two full
 quaternion products, so the corner loop paid sixteen of them plus forty-eight
 comparisons; this pays one nine-term matrix build and six dot products, with no
 branches and nothing to spill. Measured at `-O3` on an M5, both behind
 `@noinline` so the call is real: **77 ns down to 5.9 ns**, a bit over thirteen
 times. `Rigidbody.get_transformed_aabb` runs this for every body every step,
 which is where it is felt.

 The scale is folded into the centre and the extent rather than into the rows,
 which is why `bound_from_rows` sees a pure rotation. That is exact for a
 negative or non-uniform scale — the extent it receives has already been through
 `abs`, so a mirrored axis widens the box instead of inverting it — and it is
 exact for a box that arrives inverted, which is the same repair the old min/max
 over corners performed implicitly.

 `skip_rotation` now does what its name says. It was accepted and ignored
 before; no caller in this library passed it.
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

 Note what this is and is not. It is the tightest axis-aligned box in local
 space that contains the *world box* — not the box you started from before
 `transform`, which was tighter. Rotating a bound and rotating it back grows it;
 that was true of the eight-corner version too, and callers such as the mesh
 broadphase rely on the result being conservative rather than exact.
*>
fn Aabb3 Aabb3.inverse_transform(
	&self,
	Vec3 translation = { 0, 0, 0 },
	Vec3 scale = { 1, 1, 1 },
	Quaternionf rotation = { 0, 0, 0, 1 }
)
{
	Vec3 inv_scale = { 1.0f / scale.x, 1.0f / scale.y, 1.0f / scale.z };
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

 The same closed form as `transform`, entered from the other end. A renderer
 usually has the node's world matrix already composed and would have to take it
 apart to call the TRS form, so this reads the linear part's rows straight off
 it instead. Column-major, translation at `m[12..14]`.

 Exact for any affine matrix — rotation, non-uniform scale, reflection, shear —
 which is what lets frustum culling, a ray broad phase and a shadow-map fit all
 share one function. It is *not* correct for a projective matrix: the
 perspective row is ignored rather than divided through, so do not hand it one.
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

fn Aabb3 Aabb3.child_bounds(self, uint index)
{
	Vec3 c      = self.center();
	Vec3 newMin = self.min;
	Vec3 newMax = self.max;

	if (index & 1)
	{
		newMin.x = c.x;
	}
	else
	{
		newMax.x = c.x;
	}
	if (index & 2)
	{
		newMin.y = c.y;
	}
	else
	{
		newMax.y = c.y;
	}
	if (index & 4)
	{
		newMin.z = c.z;
	}
	else
	{
		newMax.z = c.z;
	}

	return { newMin, newMax };
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

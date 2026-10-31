module collision;

import std::math;

<*
 Three vertices in the space a test runs in. The one triangle vocabulary:
 `TriangleShape`, BVH/heightfield queries and the index-based `Triangle` all
 reduce to this.
*>
alias TriangleVerts = Vec3[3];

<*
 Unit normal from the winding `v0 -> v1 -> v2` (counter-clockwise = facing the viewer).
 NaN for a zero-area triangle.
*>
fn Vec3 TriangleVerts.normal(&self) => ((*self)[1] - (*self)[0]).cross((*self)[2] - (*self)[0]).normalize();

fn Aabb3 TriangleVerts.aabb(&self)
{
	return {
		.min = math::min((*self)[0], math::min((*self)[1], (*self)[2])),
		.max = math::max((*self)[0], math::max((*self)[1], (*self)[2])),
	};
}

fn Vec3 TriangleVerts.centroid(&self) => ((*self)[0] + (*self)[1] + (*self)[2]) / 3.0f;

fn float TriangleVerts.area(&self) => ((*self)[1] - (*self)[0]).cross((*self)[2] - (*self)[0]).length() * 0.5f;

<*
 Closest point on the triangle to `p` (Ericson, Real-Time Collision Detection).
*>
fn Vec3 TriangleVerts.closest_point(&self, Vec3 p)
{
	Vec3 a = (*self)[0];
	Vec3 b = (*self)[1];
	Vec3 c = (*self)[2];
	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 ap = p - a;

	float d1 = ab.dot(ap);
	float d2 = ac.dot(ap);
	if (d1 <= 0 && d2 <= 0) return a;

	Vec3 bp = p - b;
	float d3 = ab.dot(bp);
	float d4 = ac.dot(bp);
	if (d3 >= 0 && d4 <= d3) return b;

	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0)
	{
		float v = d1 / (d1 - d3);
		return a + ab * v;
	}

	Vec3 cp = p - c;
	float d5 = ab.dot(cp);
	float d6 = ac.dot(cp);
	if (d6 >= 0 && d5 <= d6) return c;

	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0)
	{
		float w = d2 / (d2 - d6);
		return a + ac * w;
	}

	float va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
	{
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}

	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	return a + ab * v + ac * w;
}

<*
 The vertices of index triangle `self` in `v`.

 @require self.in_range(v.len) : `Triangle index out of range`
*>
fn TriangleVerts Triangle.verts(self, Vec3[] v) => { v[self.x], v[self.y], v[self.z] };

<*
 True when all three indices address a vertex array of length `n`.
*>
fn bool Triangle.in_range(self, sz n) => self.max() < (uint)n;

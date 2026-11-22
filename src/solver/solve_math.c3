module collision;
import std::math;

// The solver's innermost arithmetic, written out as macros. The std methods for
// these are weak symbols, which LLVM never inlines, so each was a call through a
// stub, several per constraint per substep.

macro Vec3 cross3(Vec3 a, Vec3 b) => a.yzx * b.zxy - a.zxy * b.yzx;

// `m * v` for a column-major 3x3.
macro Vec3 apply3(Matrix3f m, Vec3 v)
{
	return {
		m.m[0] * v.x + m.m[3] * v.y + m.m[6] * v.z,
		m.m[1] * v.x + m.m[4] * v.y + m.m[7] * v.z,
		m.m[2] * v.x + m.m[5] * v.y + m.m[8] * v.z,
	};
}

// `q * v` for a unit quaternion.
macro Vec3 rotate3(QuaternionValue q, Vec3 v)
{
	Vec3 u = q.v.xyz;
	Vec3 t = cross3(u, v) * 2.0f;
	return v + t * q.v.w + cross3(u, t);
}

macro QuaternionValue quat_mul(QuaternionValue a, QuaternionValue b)
{
	Vec3 av = a.v.xyz;
	Vec3 bv = b.v.xyz;
	Vec3 axis = bv * a.v.w + av * b.v.w + cross3(av, bv);
	return { .v = { axis.x, axis.y, axis.z, a.v.w * b.v.w - av.dot(bv) } };
}

macro QuaternionValue quat_conj(QuaternionValue q) => { .v = { -q.v.x, -q.v.y, -q.v.z, q.v.w } };

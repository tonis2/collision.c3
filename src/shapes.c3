module collision;

import std::math;
import std::io;
import std::collections::list;

enum ShapeType
{
	SPHERE,
	CAPSULE,
	CYLINDER,
	BOX,
	CONVEX,
	TRIANGLE,
	<*
	 A `Heightmap`. **Not a GJK shape, and this is what says so.**

	 Its `furthest_point` answers with one of the four corners of the whole map,
	 which under GJK is a body resting on the plane through them rather than on
	 the ground. The narrow phase reads this type and routes the pair to
	 `PhysicsWorld.test_heightfield_collision`, which queries the triangles under
	 the other body and collides those — the same per-contact construction the
	 mesh BVH uses, over a grid instead of a tree.
	*>
	HEIGHTFIELD,
	COMPOUND
}

alias CollisionShapes = List {CollisionShape};

interface CollisionShape
{
	fn Vec3 furthest_point(Vec3 direction);


	fn Aabb3 get_aabb();


	fn ShapeType get_type();


	fn Matrix3f inertia_tensor(float mass) @optional;


	fn sz vertex_count() @optional;


	fn void free() @optional;

}

struct Ray (Printable)
{
	Vec3 origin;
	Vec3 direction; // Should be a unit vector.
}

struct Sphere (CollisionShape)
{
	Vec3 center;
	float radius;
}

// Cylinder centered at the origin, axis +y: flat caps at y = ±height/2 with
// the given radii (a truncated cone when they differ).
struct Cylinder (CollisionShape)
{
	float height;
	float radius_top;
	float radius_bottom;
}

// Capsule centered at the origin, axis +y: sphere caps centered at y = ±height/2
// (height covers the cylindrical section only, matching KHR_implicit_shapes).
struct Capsule (CollisionShape)
{
	float height;
	float radius_top;
	float radius_bottom;
}

// A single piece in a compound shape
struct CompoundPiece
{
	CollisionShape shape;
	Vec3 offset; // Local offset from compound center
	Aabb3 aabb; // The shape's own box (without `offset`), taken when the piece was added
}

// Compound shape: multiple convex pieces from CoACD decomposition
struct CompoundShape (CollisionShape)
{
	List {CompoundPiece} shapes;
	Aabb3 aabb; // Cached combined AABB
}

fn void CompoundShape.add_piece(&self, CollisionShape shape, Vec3 offset = { 0, 0, 0 })
{
	Aabb3 local_aabb = shape.get_aabb();
	self.shapes.push({ .shape = shape, .offset = offset, .aabb = local_aabb });
	Aabb3 piece_aabb = local_aabb + offset;
	if (self.shapes.len() == 1)
	{
		self.aabb = piece_aabb;
	}
	else
	{
		self.aabb.expand_by_aabb(piece_aabb);
	}
}

fn Vec3 CompoundShape.furthest_point(&self, Vec3 direction) @dynamic
{
	Vec3 furthest = { 0, 0, 0 };
	float max_dot = -float::max;
	foreach (&piece : self.shapes)
	{
		Vec3 point = piece.shape.furthest_point(direction) + piece.offset;
		float d = point.dot(direction);
		if (d > max_dot)
		{
			max_dot = d;
			furthest = point;
		}
	}
	return furthest;
}

fn Aabb3 CompoundShape.get_aabb(&self) @dynamic => self.aabb;
fn ShapeType CompoundShape.get_type(&self) @dynamic => ShapeType.COMPOUND;

fn void CompoundShape.free(&self) @dynamic
{
	foreach (&piece : self.shapes)
	{
		piece.shape.free();
	}
	self.shapes.free();
}

fn void CollisionShapes.release(&self)
{
	foreach (shape : self)
	{
		if (&shape.free) shape.free();
		mem::free(shape);
	}
	self.free();
}

fn Matrix3f CompoundShape.inertia_tensor(&self, float mass) @dynamic
{
	// Approximate: use AABB-based inertia
	return self.aabb.inertia_tensor(mass);
}

// A single triangle used as a temporary narrow-phase shape (BVH mesh triangles,
// soft body surface triangles). Lives on the stack - no allocation per test.
// Vertices are expected in the space the collision test runs in (usually world).
struct TriangleShape (CollisionShape)
{
	TriangleVerts v;
}

fn Vec3 TriangleShape.furthest_point(&self, Vec3 direction) @dynamic => support_of(self.v[:3], direction);

fn Aabb3 TriangleShape.get_aabb(&self) @dynamic => self.v.aabb();

fn ShapeType TriangleShape.get_type(&self) @dynamic => ShapeType.TRIANGLE;

// Shapes
struct TransformedShape (CollisionShape)
{
	Vec3 translation;
	Vec3 scale;
	Quaternionf rotation;
	CollisionShape shape;
}

fn sz? Ray.to_format(
	&self,
	Formatter* formatter
) @dynamic => formatter.printf("origin %s, direction %s", self.origin, self.direction)!;

fn float Sphere.intersect(&self, Ray ray)
{
	Vec3 oc = ray.origin - self.center;
	float a = ray.direction.dot(ray.direction);
	float b = 2.0 * oc.dot(ray.direction);
	float c = oc.dot(oc) - self.radius * self.radius;
	float discriminant = b * b - 4 * a * c;

	if (discriminant < 0)
	{
		return -1.0f; // No intersection
	}
	else
	{
		// Return the nearest positive intersection distance
		float t1 = (-b - math::sqrt(discriminant)) / (float)(2.0 * a);
		if (t1 > 0) return t1;
		float t2 = (-b + math::sqrt(discriminant)) / (float)(2.0 * a);
		if (t2 > 0) return t2;
		return -1.0;
	}
}

<*
 Local point to world: `rotation * (local * scale) + translation`.
*>
fn Vec3 TransformedShape.to_world(&self, Vec3 local) => self.rotation * (local * self.scale) + self.translation;

<*
 World point to local: the inverse of `to_world`.
*>
fn Vec3 TransformedShape.to_local(&self, Vec3 world)
{
	Vec3 inv_scale = (Vec3){ 1, 1, 1 } / self.scale;
	return (self.rotation.conjugate() * (world - self.translation)) * inv_scale;
}

<*
 World direction to local: rotation only, no scale or translation.
*>
fn Vec3 TransformedShape.dir_to_local(&self, Vec3 world_dir) => self.rotation.conjugate() * world_dir;

<*
 Local direction to world: rotation only, no scale or translation.
*>
fn Vec3 TransformedShape.dir_to_world(&self, Vec3 local_dir) => self.rotation * local_dir;

fn Vec3 TransformedShape.furthest_point(&self, Vec3 direction) @dynamic
{
	return self.to_world(self.shape.furthest_point(self.dir_to_local(direction)));
}

<*
 A `TransformedShape` with its rotation matrix cached, for the narrow phase
 where `furthest_point` runs on every GJK/EPA iteration. The two quaternion
 products of the original become two sets of three dot products.

 The rows come from `Quaternionf.rows()`, which is deliberately not normalised,
 so for any quaternion the results scale by `|q|^2` exactly as
 `TransformedShape.furthest_point` does, and for unit quaternions the two agree
 to float rounding. Only valid while `shape` is alive and unchanged.
*>
struct PreparedShape (CollisionShape)
{
	TransformedShape* shape;
	Vec3[3] rows; // rotation matrix, row i
	Vec3[3] cols; // its transpose (the conjugate's rows): row i is column i of `rows`
}

fn PreparedShape TransformedShape.prepare(&self)
{
	Vec3[3] r = self.rotation.rows();
	return {
		.shape = self,
		.rows = r,
		.cols = {
			{ r[0][0], r[1][0], r[2][0] },
			{ r[0][1], r[1][1], r[2][1] },
			{ r[0][2], r[1][2], r[2][2] },
		},
	};
}

fn Vec3 PreparedShape.furthest_point(&self, Vec3 direction) @dynamic
{
	Vec3 local_dir = { self.cols[0].dot(direction), self.cols[1].dot(direction), self.cols[2].dot(direction) };
	Vec3 p = self.shape.shape.furthest_point(local_dir) * self.shape.scale;
	return (Vec3){ self.rows[0].dot(p), self.rows[1].dot(p), self.rows[2].dot(p) } + self.shape.translation;
}

fn Aabb3 PreparedShape.get_aabb(&self) @dynamic => self.shape.get_aabb();
fn ShapeType PreparedShape.get_type(&self) @dynamic => self.shape.get_type();

fn Vec3 Sphere.furthest_point(&self, Vec3 direction) @dynamic => direction.normalize() * self.radius + self.center;

fn Vec3 Cylinder.furthest_point(&self, Vec3 direction) @dynamic
{
	float half = self.height * 0.5f;
	Vec3 dir_xz = { direction.x, 0, direction.z };
	if (dir_xz.length() > 1e-12f) dir_xz = dir_xz.normalize();

	// Candidate points on the top and bottom cap rims; pick the one with the
	// larger projection (handles truncated cones where the radii differ).
	Vec3 top = dir_xz * self.radius_top + (Vec3) { 0, half, 0 };
	Vec3 bottom = dir_xz * self.radius_bottom + (Vec3) { 0, -half, 0 };
	return direction.dot(top) > direction.dot(bottom) ? top : bottom;
}

fn Vec3 Capsule.furthest_point(&self, Vec3 direction) @dynamic
{
	float half = self.height * 0.5f;
	Vec3 n = direction.normalize();

	// Support of the convex hull of the two cap spheres.
	Vec3 top = n * self.radius_top + (Vec3) { 0, half, 0 };
	Vec3 bottom = n * self.radius_bottom + (Vec3) { 0, -half, 0 };
	return direction.dot(top) > direction.dot(bottom) ? top : bottom;
}

<*
 World-space inertia tensor of the wrapped shape: the local tensor with the
 (approximate) scale applied, then rotated with `rotate_inertia` (R I Rᵀ).
*>
fn Matrix3f TransformedShape.inertia_tensor(&self, float mass) @dynamic
{
	Matrix3f local_inertia = self.shape.inertia_tensor(mass);

	// Diagonal I_xx involves y² and z²; products of inertia scale as sx * sy.
	Vec3 s2 = self.scale * self.scale;

	local_inertia.m00 *= (s2.y + s2.z) * 0.5f;
	local_inertia.m11 *= (s2.x + s2.z) * 0.5f;
	local_inertia.m22 *= (s2.x + s2.y) * 0.5f;

	local_inertia.m01 *= self.scale.x * self.scale.y;
	local_inertia.m02 *= self.scale.x * self.scale.z;
	local_inertia.m12 *= self.scale.y * self.scale.z;
	local_inertia.m10 = local_inertia.m01;
	local_inertia.m20 = local_inertia.m02;
	local_inertia.m21 = local_inertia.m12;

	return rotate_inertia(local_inertia, self.rotation);
}

fn Matrix3f TransformedShape.world_inertia_from_local(&self, Matrix3f local_inertia) => rotate_inertia(local_inertia, self.rotation);

<*
 Translation * Rotation * Scale as a column-major 4x4 (translation in m[12..14]).
*>
fn Matrix4f TransformedShape.matrix(self)
{
	Matrix4f m = self.rotation.to_matrix();
	m = m.scale(self.scale);
	m.m[12] = self.translation.x;
	m.m[13] = self.translation.y;
	m.m[14] = self.translation.z;
	return m;
}

fn Matrix3f Sphere.inertia_tensor(&self, float mass) @dynamic
{
	float i = mass * self.radius * self.radius * 2.0f / 5.0f;
	return diag_inertia({ i, i, i });
}

<*
 Inertia of a cylinder along y with squared radius `r2` and height `h`.
*>
fn Vec3 cylinder_inertia(float mass, float r2, float h) @local
{
	float transverse = (mass / 12.0f) * (3 * r2 + h * h);
	return { transverse, 0.5f * mass * r2, transverse };
}

fn Matrix3f diag_inertia(Vec3 d) @local => { d.x, 0, 0, 0, d.y, 0, 0, 0, d.z };

<*
 A truncated cone is approximated by the equal-volume cylinder, whose squared
 radius is the mean of r(y)² over the height: (rt² + rt·rb + rb²) / 3. Exact
 when the radii match.
*>
fn Matrix3f Cylinder.inertia_tensor(&self, float mass) @dynamic
{
	float rt = self.radius_top;
	float rb = self.radius_bottom;
	return diag_inertia(cylinder_inertia(mass, (rt * rt + rt * rb + rb * rb) / 3.0f, self.height));
}

<*
 Cylinder plus two hemispheres (a sphere's worth), mass split by volume. With
 differing end radii the mean radius is used. About the centre, the hemispheres
 contribute m_h (2/5 r² + h²/4 + 3hr/8) transversely and 2/5 m_h r² axially.
*>
fn Matrix3f Capsule.inertia_tensor(&self, float mass) @dynamic
{
	float r = (self.radius_top + self.radius_bottom) * 0.5f;
	float h = self.height;
	float v_cyl = math::PI * r * r * h;
	float v_caps = 4.0f / 3.0f * math::PI * r * r * r;
	float total = v_cyl + v_caps;
	if (total <= 0) return diag_inertia({ 0, 0, 0 });

	float m_cyl = mass * v_cyl / total;
	float m_caps = mass - m_cyl;

	Vec3 i = cylinder_inertia(m_cyl, r * r, h);
	float transverse_caps = m_caps * (0.4f * r * r + 0.25f * h * h + 0.375f * h * r);
	i.x += transverse_caps;
	i.z += transverse_caps;
	i.y += 0.4f * m_caps * r * r;
	return diag_inertia(i);
}

fn ShapeType TransformedShape.get_type(&self) @dynamic => self.shape.get_type();
fn ShapeType Sphere.get_type(&self) @dynamic => ShapeType.SPHERE;
fn ShapeType Capsule.get_type(&self) @dynamic => ShapeType.CAPSULE;
fn ShapeType Cylinder.get_type(&self) @dynamic => ShapeType.CYLINDER;

fn Aabb3 TransformedShape.get_transformed_aabb(
	&self
) @dynamic => self.shape.get_aabb().transform(self.translation, self.scale, self.rotation);
fn Aabb3 TransformedShape.get_aabb(&self) @dynamic => self.shape.get_aabb();

fn Aabb3 Sphere.get_aabb(&self) @dynamic => { .min = self.center - self.radius, .max = self.center + self.radius };

fn Aabb3 Capsule.get_aabb(&self) @dynamic
{
	float radius_max = math::max(self.radius_top, self.radius_bottom);
	float half = self.height * 0.5f;

	return {
		.min = { -radius_max, -half - self.radius_bottom, -radius_max },
		.max = { radius_max, half + self.radius_top, radius_max },
	};
}

fn Aabb3 Cylinder.get_aabb(&self) @dynamic
{
	float radius_max = math::max(self.radius_top, self.radius_bottom);
	float half = self.height * 0.5f;

	return { .min = { -radius_max, -half, -radius_max }, .max = { radius_max, half, radius_max } };
}

module collision;
import std::math, std::collections::list, std::io;

// Below this a volume/length is treated as zero.
const double VEC_EPSILON = Vec3::size == 12 ? 1e-5 : 1e-9;

// Triangle structure
alias Triangle = uint[<3>];
alias MeshList = List {Mesh};

fn void MeshList.release(&self)
{
	foreach (&piece : self) piece.free();
	self.free();
}

<*
 @require self.in_range(vertices.len) : `Triangle index out of range`
*>
fn Vec3 Triangle.get_normal(self, Vec3[] vertices) => self.verts(vertices).normal();

<*
 @require self.in_range(vertices.len) : `Triangle index out of range`
*>
fn Vec3 Triangle.get_center(self, Vec3[] vertices) => self.verts(vertices).centroid();

<*
 @require self.in_range(vertices.len) : `Triangle index out of range`
*>
fn double Triangle.get_area(self, Vec3[] vertices) => self.verts(vertices).area();

// Mesh structure
struct Mesh (CollisionShape)
{
	List {Vec3} vertices;
	List {Triangle} triangles;
	TriBVH* bvh;
	Aabb3 aabb;
}

fn sz? Mesh.to_format(&self, Formatter* formatter) @dynamic
{
	sz lenght = 0;
	foreach (vert : self.vertices)
	{
		lenght += formatter.printf("%s,%s,%s\n", vert.x, vert.y, vert.z)!;
	}

	return lenght;
}

<*
 Put the mesh's two lists on a named allocator, once, before anything is added.

 A zero-valued list binds to the temp allocator on its first push and dangles when
 the temp scope ends. The BVH also points into the lists' storage, so the lists must
 not move or lose their pool.

 @param allocator : `Owns both lists; Mesh.free returns them to it.`
*>
fn void Mesh.init(&self, Allocator allocator, sz vertices = 16, sz triangles = 16)
{
	self.vertices.init(allocator, vertices > 0 ? vertices : 1);
	self.triangles.init(allocator, triangles > 0 ? triangles : 1);
}

<*
 @param allocator : `Owns both lists; Mesh.free returns them to it.`
*>
fn Mesh create_mesh(Vec3[] vertices, uint[] indices, Allocator allocator = mem)
{
	Mesh mesh;
	mesh.init(allocator, vertices.len, indices.len / 3);
	mesh.vertices.push_all(vertices);
	mesh.set_indices(indices);
	mesh.aabb = mesh.get_aabb();
	return mesh;
}

// Build BVH tree for accelerated collision detection
// Uses indexed mode - references mesh vertices directly without copying
fn void Mesh.build_bvh(&self)
{
	if (self.triangles.len() == 0) return;
	if (self.bvh != null) return; // Already built

	// Indexed BVH: no vertex duplication
	self.bvh = create_tri_bvh(self.vertices.entries, self.triangles.entries, self.triangles.len());
}

fn bool Mesh.has_bvh(&self) => self.bvh != null;

<*
 Build the BVH if the mesh does not have one yet.
*>
fn void Mesh.ensure_bvh(&self)
{
	if (!self.has_bvh()) self.build_bvh();
}

<*
 Visit every triangle with in-range indices as its three vertices. Invalid
 triangles are skipped.
*>
macro void Mesh.@foreach_tri(&self; @body(Vec3 a, Vec3 b, Vec3 c))
{
	sz vert_count = self.vertices.len();
	foreach (tri : self.triangles)
	{
		if (!tri.in_range(vert_count)) continue;
		@body(self.vertices[tri.x], self.vertices[tri.y], self.vertices[tri.z]);
	}
}

fn void Mesh.free(&self) @dynamic
{
	self.vertices.free();
	self.triangles.free();
	if (self.bvh != null) self.bvh.free();
	self.bvh = null;
}

fn void Mesh.add_triangle(&self, Vec3 v0, Vec3 v1, Vec3 v2)
{
	uint idx0 = (uint)self.vertices.len();
	self.vertices.push(v0);
	self.vertices.push(v1);
	self.vertices.push(v2);
	self.triangles.push({ idx0, idx0 + 1, idx0 + 2 });
}

<*
 Add the triangle fan (0, i, i + 1) of a convex polygon; fewer than three points add nothing.
*>
fn void Mesh.add_fan(&self, Vec3[] poly)
{
	for (sz i = 1; i + 1 < poly.len; i++) self.add_triangle(poly[0], poly[i], poly[i + 1]);
}

// Load from flat index array (GLTF-style)
fn void Mesh.set_indices(&self, uint[] indices)
{
	self.triangles.clear();
	for (sz i = 0; i + 2 < indices.len; i += 3)
	{
		self.triangles.push({ indices[i], indices[i + 1], indices[i + 2] });
	}
}

// Validate mesh triangles, removing any with out-of-bounds indices
fn void Mesh.validate(&self)
{
	sz vert_count = self.vertices.len();
	if (vert_count == 0) return;

	// Filter out invalid triangles. The kept list must live on the mesh's own
	// allocator: it replaces `self.triangles`, which Mesh.free returns to it.
	List {Triangle} valid_tris;
	valid_tris.init(self.triangles.allocator.ptr != null ? self.triangles.allocator : mem, self.triangles.len() + 1);
	foreach (tri : self.triangles)
	{
		if (tri.in_range(vert_count)) valid_tris.push(tri);
	}

	if (valid_tris.len() != self.triangles.len())
	{
		self.triangles.free();
		self.triangles = valid_tris;
	}
	else
	{
		valid_tris.free();
	}
}

// Weld duplicate vertices and remove degenerate triangles
fn void Mesh.remesh(&self)
{
	if (self.vertices.is_empty() || self.triangles.is_empty()) return;
	weld_remesh(&self.vertices, &self.triangles);
}

// Decompose concave mesh into convex pieces using CoACD algorithm
fn List {Mesh} Mesh.decompose(&self, CoacdParameters params = COACD_MEDIUM)
{
	return coacd_decompose(self, params);
}

fn double Mesh.get_volume(&self)
{
	double volume = 0.0;
	self.@foreach_tri(; Vec3 a, Vec3 b, Vec3 c)
	{
		volume += a.dot(b.cross(c)) / 6.0;
	};
	return math::abs(volume);
}

fn bool Mesh.is_convex(&self)
{
	if (self.triangles.len() < 4) return true;

	// Tight epsilon so hollow shapes still decompose
	const double EPSILON = 0.01;
	Vec3[] verts = self.vertices.array_view();

	foreach (tri : self.triangles)
	{
		// An invalid mesh counts as non-convex, which triggers decomposition
		if (!tri.in_range(verts.len)) return false;

		Vec3 normal = tri.get_normal(verts);
		double d = normal.dot(verts[tri.x]);

		foreach (v : verts)
		{
			if (normal.dot(v) - d > EPSILON) return false;
		}
	}

	return true;
}

<*
 The box of the current vertices. Always recomputed: a cached box goes stale when
 triangles are added or the mesh is welded or split. The `aabb` field is
 refreshed as a convenience for readers that want the last value.

 @require !self.vertices.is_empty() : `Vertices should not be empty`
*>
fn Aabb3 Mesh.get_aabb(&self) @dynamic
{
	self.aabb = aabb_of_points(self.vertices.array_view());
	return self.aabb;
}

fn Vec3 Mesh.furthest_point(&self, Vec3 direction) @dynamic => support_of(self.vertices.array_view(), direction);

<*
 Centroid of the (closed) mesh volume; zero for a degenerate one.
*>
fn Vec3 Mesh.center_of_mass(&self)
{
	float total_volume = 0;
	Vec3 weighted_com = { 0, 0, 0 };

	self.@foreach_tri(; Vec3 v1, Vec3 v2, Vec3 v3)
	{
		// Signed tetrahedron volume to the origin (times 6); its centroid is (v1 + v2 + v3) / 4.
		float detJ = v1.dot(v2.cross(v3));
		total_volume += detJ;
		weighted_com += detJ * (v1 + v2 + v3);
	};

	if (math::abs(total_volume) > VEC_EPSILON) return weighted_com / (4.0f * total_volume);
	return { 0, 0, 0 };
}

<*
 Inertia tensor about the centre of mass for uniform density and total `mass`.
 One pass accumulates the volume and the inertia about the origin (density 1);
 the result is then scaled to `mass / volume` and shifted by the parallel-axis theorem.
*>
fn Matrix3f Mesh.inertia_tensor(&self, float mass) @dynamic
{
	Matrix3f inertia = { };
	float signed_volume = 0;
	Vec3 weighted_com = { 0, 0, 0 };

	self.@foreach_tri(; Vec3 v1, Vec3 v2, Vec3 v3)
	{
		float detJ = v1.dot(v2.cross(v3));
		signed_volume += detJ;
		weighted_com += detJ * (v1 + v2 + v3);
		inertia += compute_inertia_contribution(v1, v2, v3, 1.0f);
	};

	// Dividing by the *signed* volume makes the result independent of winding.
	float volume = signed_volume / 6.0f;
	if (math::abs(volume) > VEC_EPSILON) inertia *= mass / volume;

	Vec3 c = math::abs(signed_volume) > VEC_EPSILON ? weighted_com / (4.0f * signed_volume) : (Vec3){ 0, 0, 0 };

	// Parallel axis: I_cm = I_origin - m (|c|² δ - c cᵀ)
	Vec3 c_sq = c * c;
	inertia.m00 -= mass * (c_sq.y + c_sq.z);
	inertia.m11 -= mass * (c_sq.x + c_sq.z);
	inertia.m22 -= mass * (c_sq.x + c_sq.y);

	inertia.m01 -= mass * c.x * c.y;
	inertia.m02 -= mass * c.x * c.z;
	inertia.m12 -= mass * c.y * c.z;

	inertia.m10 = inertia.m01;
	inertia.m20 = inertia.m02;
	inertia.m21 = inertia.m12;

	return inertia;
}

fn ShapeType Mesh.get_type(&self) @dynamic => ShapeType.CONVEX;
fn sz Mesh.vertex_count(&self) @dynamic => self.vertices.len();


// Plane structure for clipping
struct Plane
{
	Vec3 normal;
	double d;
}

fn Plane create_plane(Vec3 n = { 0, 0, 0 }, double dist = 0.0)
{
	return { n.normalize(), dist };
}

fn double Plane.distance(&self, Vec3 p)
{
	return self.normal.dot(p) - self.d;
}

// Compute intersection point between two vertices across a plane
fn Vec3 intersect_edge(Vec3 v0, Vec3 v1, double d0, double d1)
{
	double t = d0 / (d0 - d1);
	return v0 + (v1 - v0) * (float)t;
}


// ----------------------------------------------------------------------------
// Inertia Tensor Helper (used by Mesh.inertia_tensor)
// ----------------------------------------------------------------------------

fn Matrix3f compute_inertia_contribution(Vec3 a, Vec3 b, Vec3 c, float density) @local
{
	Matrix3f inertia_tensor;

	float detJ = a.dot(b.cross(c));

	float v_mult_diag = (density * detJ) / 60.0f;
	float v_mult_off_diag = (density * detJ) / 120.0f;

	// Diagonal elements (Moments of Inertia)
	inertia_tensor.m00 = v_mult_diag
		* ((a.y * a.y + a.y * b.y + b.y * b.y + a.y * c.y + b.y * c.y + c.y * c.y)
			+ (a.z * a.z + a.z * b.z + b.z * b.z + a.z * c.z + b.z * c.z + c.z * c.z));

	inertia_tensor.m11 = v_mult_diag
		* ((a.x * a.x + a.x * b.x + b.x * b.x + a.x * c.x + b.x * c.x + c.x * c.x)
			+ (a.z * a.z + a.z * b.z + b.z * b.z + a.z * c.z + b.z * c.z + c.z * c.z));

	inertia_tensor.m22 = v_mult_diag
		* ((a.x * a.x + a.x * b.x + b.x * b.x + a.x * c.x + b.x * c.x + c.x * c.x)
			+ (a.y * a.y + a.y * b.y + b.y * b.y + a.y * c.y + b.y * c.y + c.y * c.y));

	inertia_tensor.m01 = -v_mult_off_diag
		* ((2 * a.x * a.y + b.x * a.y + c.x * a.y) + (a.x * b.y + 2 * b.x * b.y + c.x * b.y)
			+ (a.x * c.y + b.x * c.y + 2 * c.x * c.y));

	inertia_tensor.m02 = -v_mult_off_diag
		* ((2 * a.x * a.z + b.x * a.z + c.x * a.z) + (a.x * b.z + 2 * b.x * b.z + c.x * b.z)
			+ (a.x * c.z + b.x * c.z + 2 * c.x * c.z));

	inertia_tensor.m12 = -v_mult_off_diag
		* ((2 * a.y * a.z + b.y * a.z + c.y * a.z) + (a.y * b.z + 2 * b.y * b.z + c.y * b.z)
			+ (a.y * c.z + b.y * c.z + 2 * c.y * c.z));

	inertia_tensor.m10 = inertia_tensor.m01;
	inertia_tensor.m20 = inertia_tensor.m02;
	inertia_tensor.m21 = inertia_tensor.m12;

	return inertia_tensor;
}

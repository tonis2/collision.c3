module collision;

import std::collections::fixedlist;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;
import std::thread;
import std::sort;
import collision::dmath;

// Manifold expansion only pays off on shapes with few vertices.
const MANIFOLD_VERTEX_LIMIT @private = 24;
// Two contacts closer than DEDUP_DIST_SQ with normals this aligned are one contact.
const float NORMAL_DEDUP_COS @private = 0.99f;

fn bool Rigidbody.has_mesh_bvh(&self)
{
	if (self.collider.get_type() == ShapeType.CONVEX)
	{
		Mesh* mesh = (Mesh*)self.collider.shape;
		return mesh.has_bvh();
	}
	return false;
}

fn bool Rigidbody.has_compound(&self)
{
	return self.collider.get_type() == ShapeType.COMPOUND;
}

<*
 Whether this body's collider is terrain.

 Checked before `has_mesh_bvh` when sorting pairs: a heightfield is not a `Mesh`
 and has no BVH, so the two never both hold, but a pair of two heightfields
 would otherwise fall through to GJK, which cannot handle them.
*>
fn bool Rigidbody.has_heightfield(&self)
{
	return self.collider.get_type() == ShapeType.HEIGHTFIELD;
}

// ============================================================================
// Contact construction
// ============================================================================

fn Collision collision_from_info(Pair pair, CollisionInfo info) @private @inline
{
	return {
		.a = pair.a,
		.b = pair.b,
		.normal = info.normal,
		.contact = info.contact_points,
		.depth = info.depth,
	};
}

<*
 Build a contact between a surface and another body, in pair order.

 @param normal : `Unit direction from the surface towards the other body.`
 @param surface_is_a : `Whether the surface is pair.a. If not, the normal and the two points are flipped so the contact still reads a -> b.`
*>
fn Collision oriented_collision(
	Pair pair, bool surface_is_a, Vec3 normal, Vec3 surface_point, Vec3 other_point, float depth
) @private
{
	Collision c = { .a = pair.a, .b = pair.b, .depth = depth };
	if (surface_is_a)
	{
		c.normal = normal;
		c.contact = { surface_point, other_point };
	}
	else
	{
		c.normal = -normal;
		c.contact = { other_point, surface_point };
	}
	return c;
}

// ============================================================================
// Parallel pair testing
// ============================================================================

// One group of pairs sharing a test function.
struct PairGroup
{
	Pair[] pairs;
	PairTestFn test;
}

// The one collision worker: runs the task's test function over its slice of pairs.
fn int collision_test_worker(void* arg) @private
{
	CollisionTask* task = (CollisionTask*)arg;
	PhysicsWorld* world = task.world;

	for (sz i = 0; i < task.count; i++)
	{
		Pair pair = task.pairs[i];
		Rigidbody* body_a = world.find_body(pair.a)!!;
		Rigidbody* body_b = world.find_body(pair.b)!!;

		Collision[] slots = task.results[i * MAX_CONTACTS_PER_PAIR : MAX_CONTACTS_PER_PAIR];
		task.counts[i] = task.test(world, body_a, body_b, pair, slots);
	}
	return 0;
}

// Plain convex-vs-convex pair: one GJK/EPA contact, or none.
fn sz test_convex_collision(
	PhysicsWorld* world, Rigidbody* body_a, Rigidbody* body_b, Pair pair, Collision[] out_contacts
) @private
{
	PreparedShape prep_a = body_a.collider.prepare();
	PreparedShape prep_b = body_b.collider.prepare();
	CollisionInfo info = collision::check_convex_collision(&prep_a, &prep_b, epa: true);
	if (!info.collided) return 0;
	out_contacts[0] = collision_from_info(pair, info);
	return 1;
}

<*
 Test every group's pairs in one parallel pass (one barrier) and append the
 contacts to `self.collisions`.

 Results go into per-pair slots and are read back in group order, then pair
 order, so the output does not depend on which thread ran what.
*>
fn void PhysicsWorld.dispatch_collision_tasks(&self, PairGroup[] groups)
{
	sz total = 0;
	foreach (group : groups) total += group.pairs.len;
	if (total == 0) return;

	self.collision_results.clear();
	self.collision_counts.clear();
	self.collision_results.reserve(total * MAX_CONTACTS_PER_PAIR);
	self.collision_counts.reserve(total);
	for (sz i = 0; i < total * MAX_CONTACTS_PER_PAIR; i++) self.collision_results.push({ });
	for (sz i = 0; i < total; i++) self.collision_counts.push(0);

	Collision* results = self.collision_results.array_view().ptr;
	sz* counts = self.collision_counts.array_view().ptr;

	// Batch each group into at most one task per worker
	self.collision_tasks.clear();
	sz workers = self.thread_pool.workers();
	sz offset = 0;
	foreach (group : groups)
	{
		sz count = group.pairs.len;
		if (count == 0) continue;
		sz batch_size = (count + workers - 1) / workers;

		for (sz start = 0; start < count; start += batch_size)
		{
			self.collision_tasks.push(
				{
					.world = self,
					.test = group.test,
					.pairs = group.pairs.ptr + start,
					.results = results + (offset + start) * MAX_CONTACTS_PER_PAIR,
					.counts = counts + offset + start,
					.count = math::min(batch_size, count - start),
				}
			);
		}
		offset += count;
	}

	if (self.collision_tasks.len() == 1)
	{
		// Single batch — run inline, skip thread pool overhead
		collision_test_worker(&self.collision_tasks[0]);
	}
	else
	{
		for (sz i = 0; i < self.collision_tasks.len(); i++)
		{
			self.thread_pool.push(&collision_test_worker, &self.collision_tasks[i]);
		}
		self.thread_pool.join();
	}

	for (sz i = 0; i < total; i++)
	{
		for (sz j = 0; j < self.collision_counts[i]; j++)
		{
			self.collisions.push(self.collision_results[i * MAX_CONTACTS_PER_PAIR + j]);
		}
	}
}

// ============================================================================
// Manifold expansion
// ============================================================================

fn bool is_manifold_shape(TransformedShape* collider) @private @inline
{
	ShapeType t = collider.get_type();
	if (t == ShapeType.BOX) return true;
	if (t == ShapeType.CONVEX && &collider.shape.vertex_count) return collider.shape.vertex_count() < MANIFOLD_VERTEX_LIMIT;
	return false;
}

// Rotate `v` by the matrix whose rows are `rows` (see `Quaternionf.rows`).
fn Vec3 rotate_by_rows(Vec3[3] rows, Vec3 v) @private @inline
{
	return { rows[0].dot(v), rows[1].dot(v), rows[2].dot(v) };
}

<*
 Expand a single EPA contact into a multi-contact manifold.

 For flat-on-flat contacts (box on floor), projects the vertices of the incident
 body onto the reference body's contact plane and creates additional contacts.
*>
fn void PhysicsWorld.generate_manifold(&self, Collision* collision)
{
	// Secondary contacts of a mesh/compound manifold are already spread out
	if (collision.skip_expand)
	{
		self.collisions.push(*collision);
		return;
	}

	Rigidbody* a = self.find_body(collision.a)!!;
	Rigidbody* b = self.find_body(collision.b)!!;

	bool a_manifold = is_manifold_shape(&a.collider);
	bool b_manifold = is_manifold_shape(&b.collider);

	// Incident body: must be a dynamic manifold shape (never expand static ground)
	bool a_candidate = a_manifold && !a.is_static;
	bool b_candidate = b_manifold && !b.is_static;

	if (!a_candidate && !b_candidate)
	{
		self.collisions.push(*collision);
		return;
	}

	// Of two candidates, the smaller one is the incident body
	bool incident_is_a = a_candidate;
	if (a_candidate && b_candidate)
	{
		incident_is_a = a.transformed_aabb.surface_area() <= b.transformed_aabb.surface_area();
	}

	Rigidbody* incident = incident_is_a ? a : b;

	Vec3[8] box_corners;
	Vec3[] local_verts;

	if (incident.collider.get_type() == ShapeType.BOX)
	{
		box_corners = ((Aabb3*)incident.collider.shape).corners();
		local_verts = &box_corners;
	}
	else
	{
		Mesh* mesh = (Mesh*)incident.collider.shape;
		local_verts = mesh.vertices.entries[:mesh.vertices.size];
	}

	Vec3 plane_point = incident_is_a ? collision.contact[1] : collision.contact[0];
	Vec3 plane_normal = incident_is_a ? -collision.normal : collision.normal;
	Vec3 orig_contact = incident_is_a ? collision.contact[0] : collision.contact[1];

	float depth_tol = collision.depth + MANIFOLD_DEPTH_TOL;
	float max_spread_sq = incident.transformed_aabb.size().length_sq();

	Vec3[3] rows = incident.collider.rotation.rows();
	Vec3 scale = incident.collider.scale;
	Vec3 translation = incident.collider.translation;

	// Always include original EPA contact
	self.collisions.push(*collision);

	foreach (local_v : local_verts)
	{
		Vec3 v = rotate_by_rows(rows, local_v * scale) + translation;

		float dist_to_orig = v.distance_sq(orig_contact);
		if (dist_to_orig < DEDUP_DIST_SQ || dist_to_orig > max_spread_sq) continue;

		float signed_dist = (v - plane_point).dot(plane_normal);
		if (signed_dist >= depth_tol) continue;

		float depth = math::max(0.0f, -signed_dist);
		Vec3 projected = v - plane_normal * signed_dist;

		Collision new_c = { .a = collision.a, .b = collision.b, .normal = collision.normal, .depth = depth, };
		if (incident_is_a)
		{
			new_c.contact = { v, projected };
		}
		else
		{
			new_c.contact = { projected, v };
		}
		self.collisions.push(new_c);
	}
}

<*
 Insert a contact into a sorted cache (deepest first, capacity
 MAX_CONTACTS_PER_PAIR). Near-duplicates — same contact point and normal — keep
 only the deeper entry, so manifolds spread across distinct points.
 Thread-safe: operates on a caller-owned cache. Returns the new count.
*>
fn sz insert_contact_sorted(Collision[] cache, sz count, Collision contact) @private
{
	// Dedup against existing contacts
	for (sz i = 0; i < count; i++)
	{
		if (cache[i].contact[0].distance_sq(contact.contact[0]) < DEDUP_DIST_SQ
			&& cache[i].normal.dot(contact.normal) > NORMAL_DEDUP_COS)
		{
			if (contact.depth <= cache[i].depth) return count; // Existing one is deeper
			// New one is deeper: remove the duplicate, then insert sorted below
			for (sz j = i; j + 1 < count; j++)
			{
				cache[j] = cache[j + 1];
			}
			count--;
			break;
		}
	}

	sz insert_pos = count;
	for (sz i = 0; i < count; i++)
	{
		if (contact.depth > cache[i].depth)
		{
			insert_pos = i;
			break;
		}
	}

	if (count < MAX_CONTACTS_PER_PAIR)
	{
		for (sz i = count; i > insert_pos; i--)
		{
			cache[i] = cache[i - 1];
		}
		cache[insert_pos] = contact;
		return count + 1;
	}

	if (insert_pos < MAX_CONTACTS_PER_PAIR)
	{
		for (sz i = MAX_CONTACTS_PER_PAIR - 1; i > insert_pos; i--)
		{
			cache[i] = cache[i - 1];
		}
		cache[insert_pos] = contact;
	}
	return count;
}

<*
 Check collisions using CoACD decomposed convex pieces: each piece is a
 convex-convex test against the other body. At least one body must be compound
 (pairs are pre-categorized).
*>
fn void PhysicsWorld.check_coacd_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair)
{
	bool compound_is_a = body_a.has_compound();
	Rigidbody* compound_body = compound_is_a ? body_a : body_b;
	Rigidbody* other_body = compound_is_a ? body_b : body_a;

	CompoundShape* compound = (CompoundShape*)compound_body.collider.shape;
	PreparedShape other_prep = other_body.collider.prepare();

	Collision[MAX_CONTACTS_PER_PAIR] contact_cache;
	sz contact_count = 0;

	foreach (&piece : compound.shapes)
	{
		// Piece transform is relative to the compound body
		TransformedShape piece_shape = {
			.translation = compound_body.collider.translation + compound_body.collider.rotation * piece.offset,
			.scale = compound_body.collider.scale,
			.rotation = compound_body.collider.rotation,
			.shape = piece.shape,
		};

		// Early AABB rejection (world space)
		// (The piece's local box is cached; `shape.get_aabb()` can cost O(vertices).)
		if (!piece.aabb.transform(piece_shape.translation, piece_shape.scale, piece_shape.rotation).collides(other_body.transformed_aabb)) continue;

		PreparedShape piece_prep = piece_shape.prepare();
		CollisionInfo info = compound_is_a
			? collision::check_convex_collision(&piece_prep, &other_prep, epa: true)
			: collision::check_convex_collision(&other_prep, &piece_prep, epa: true);

		if (info.collided && info.depth > 0)
		{
			contact_count = insert_contact_sorted(&contact_cache, contact_count, collision_from_info(pair, info));
		}
	}

	// Only the deepest contact is worth vertex-expanding; the rest already
	// form a manifold.
	for (sz i = 0; i < contact_count; i++)
	{
		contact_cache[i].skip_expand = i > 0;
		self.collisions.push(contact_cache[i]);
	}
}

// ============================================================================
// One-sided triangle surfaces (mesh BVH and heightfield)
// ============================================================================

// A contact between a triangle and the other body, from the surface's point of view.
struct SurfaceContact
{
	bool hit;
	Vec3 normal; // Unit, surface -> other body
	Vec3 on_surface;
	Vec3 on_other;
	float depth;
}

<*
 The contact for a sphere or capsule axis point against a one-sided triangle,
 given the closest pair of points between the core (point or axis) and the
 triangle.

 A core that has crossed to the back of the surface is pushed back out along the
 face normal instead of being ejected through the triangle.
*>
fn SurfaceContact one_sided_contact(Vec3 on_axis, Vec3 on_tri, float radius, Vec3 face_normal) @private
{
	Vec3 diff = on_axis - on_tri;
	float dist_sq = diff.length_sq();
	if (dist_sq >= radius * radius) return { };

	Vec3 n = face_normal;
	float depth = radius;
	if (dist_sq > DEGENERATE_DIST_SQ)
	{
		float dist = math::sqrt(dist_sq);
		if (diff.dot(face_normal) >= 0)
		{
			n = diff / dist;
			depth = radius - dist;
		}
		else
		{
			depth = radius + dist;
		}
	}
	return { .hit = true, .normal = n, .on_surface = on_tri, .on_other = on_axis - n * radius, .depth = depth };
}

// Running minimum of the distance between two point sets, for the capsule test.
struct ClosestPair
{
	float dist_sq;
	Vec3 on_axis;
	Vec3 on_tri;
}

fn void ClosestPair.offer(&self, Vec3 axis_pt, Vec3 tri_pt) @inline
{
	float d = (axis_pt - tri_pt).length_sq();
	if (d < self.dist_sq)
	{
		self.dist_sq = d;
		self.on_axis = axis_pt;
		self.on_tri = tri_pt;
	}
}

<*
 What a triangle is being tested against, worked out once for a whole surface.

 The specialisation is per body and the triangles are per contact, so the sphere
 and capsule world positions and the test function are chosen here instead of
 once per triangle. A struct rather than loose arguments because the mesh BVH and
 the heightfield build the same thing.
*>
struct TriangleProbe
{
	Rigidbody* other; // The body being tested against the surface
	PreparedShape other_prep; // Its collider with the rotation cached, for GJK/EPA
	Pair pair;
	// Whether the surface is `pair.a`; decides how contacts are oriented.
	bool surface_is_a;
	TriangleTestFn test; // Sphere, capsule or GJK/EPA
	Vec3 sphere_center;
	Vec3 capsule_p0;
	Vec3 capsule_p1;
	float world_radius;
}

alias TriangleTestFn = fn SurfaceContact(TriangleProbe*, TriangleVerts, Vec3);

fn SurfaceContact sphere_vs_triangle(TriangleProbe* probe, TriangleVerts tri, Vec3 face_normal) @private
{
	return one_sided_contact(probe.sphere_center, tri.closest_point(probe.sphere_center), probe.world_radius, face_normal);
}

fn SurfaceContact capsule_vs_triangle(TriangleProbe* probe, TriangleVerts tri, Vec3 face_normal) @private
{
	Vec3 p0 = probe.capsule_p0;
	Vec3 p1 = probe.capsule_p1;

	ClosestPair best = { .dist_sq = float::max };
	best.offer(p0, tri.closest_point(p0));
	best.offer(p1, tri.closest_point(p1));
	foreach (v : tri) best.offer(closest_point_on_segment(v, p0, p1), v);

	Vec3 c1, c2;
	for (usz i = 0; i < 3; i++)
	{
		closest_points_segments(p0, p1, tri[i], tri[(i + 1) % 3], &c1, &c2);
		best.offer(c1, c2);
	}
	return one_sided_contact(best.on_axis, best.on_tri, probe.world_radius, face_normal);
}

<*
 Any other shape, through GJK/EPA. A contact that EPA resolved through the back
 side means the body crossed the surface this step; it is rebuilt against the
 triangle's plane rather than ejecting the body through it.
*>
fn SurfaceContact convex_vs_triangle(TriangleProbe* probe, TriangleVerts tri, Vec3 face_normal) @private
{
	TriangleShape tri_shape = { .v = tri };
	bool surface_is_a = probe.surface_is_a;

	CollisionInfo info = surface_is_a
		? collision::check_convex_collision(&tri_shape, &probe.other_prep, epa: true)
		: collision::check_convex_collision(&probe.other_prep, &tri_shape, epa: true);
	if (!info.collided || info.depth <= 0) return { };

	Vec3 push_dir = surface_is_a ? info.normal : -info.normal;
	if (push_dir.dot(face_normal) < 0)
	{
		Vec3 deep = probe.other_prep.furthest_point(-face_normal);
		float plane_depth = (tri[0] - deep).dot(face_normal);
		if (plane_depth <= 0) return { };
		return {
			.hit = true,
			.normal = face_normal,
			.on_surface = deep + face_normal * plane_depth,
			.on_other = deep,
			.depth = plane_depth,
		};
	}

	sz s = surface_is_a ? 0 : 1;
	return {
		.hit = true,
		.normal = push_dir,
		.on_surface = info.contact_points[s],
		.on_other = info.contact_points[1 - s],
		.depth = info.depth,
	};
}

<*
 Work out how one body should be tested against triangles.

 The sphere and capsule paths need a uniform scale: their world radius is one
 number. Anything else takes the GJK/EPA path, which reads the collider through
 its support function and does not care.

 @param surface_is_a : `Whether the surface is pair.a.`
*>
fn TriangleProbe triangle_probe(Rigidbody* other_body, Pair pair, bool surface_is_a)
{
	TriangleProbe probe = {
		.other = other_body,
		.other_prep = other_body.collider.prepare(),
		.pair = pair,
		.surface_is_a = surface_is_a,
		.test = &convex_vs_triangle,
	};

	ShapeType other_type = other_body.collider.get_type();
	Vec3 other_scale = other_body.collider.scale;
	bool uniform_scale = math::abs(other_scale.x - other_scale.y) < UNIFORM_SCALE_TOL
		&& math::abs(other_scale.y - other_scale.z) < UNIFORM_SCALE_TOL;

	if (uniform_scale && other_type == SPHERE)
	{
		Sphere* sphere = (Sphere*)other_body.collider.shape;
		probe.test = &sphere_vs_triangle;
		probe.sphere_center = other_body.collider.to_world(sphere.center);
		probe.world_radius = sphere.radius * other_scale.x;
	}
	else if (uniform_scale && other_type == CAPSULE)
	{
		Capsule* capsule = (Capsule*)other_body.collider.shape;
		// Cap-sphere centers sit at ±height/2 on the local Y axis
		float half = capsule.height * 0.5f;
		probe.test = &capsule_vs_triangle;
		probe.capsule_p0 = other_body.collider.to_world({ 0, -half, 0 });
		probe.capsule_p1 = other_body.collider.to_world({ 0, half, 0 });
		probe.world_radius = math::max(capsule.radius_top, capsule.radius_bottom) * other_scale.x;
	}
	return probe;
}

<*
 One world-space triangle against one body, folded into the manifold.

 Everything a one-sided surface means is behind `probe.test`; a mesh and a
 heightfield are the same surface as far as a contact is concerned, so both go
 through here.

 @param count : `Contacts already in out_contacts.`
 @return `The new count.`
*>
fn sz triangle_contacts(TriangleProbe* probe, TriangleVerts tri, Collision[] out_contacts, sz count)
{
	if (!tri.aabb().collides(probe.other.transformed_aabb)) return count;

	// Face normal of the (one-sided) surface; requires consistent winding
	SurfaceContact hit = probe.test(probe, tri, tri.normal());
	if (!hit.hit) return count;

	return insert_contact_sorted(
		out_contacts,
		count,
		oriented_collision(probe.pair, probe.surface_is_a, hit.normal, hit.on_surface, hit.on_other, hit.depth)
	);
}

<*
 Clamp a triangle manifold's depths and mark all but the deepest.

 Extreme correction forces come out of a body that has sunk into a surface. The
 deepest contact is vertex-expanded later; the rest are already spread across
 triangles, so only the first needs it.
*>
fn void settle_triangle_manifold(Collision[] out_contacts, sz count)
{
	for (sz i = 0; i < count; i++)
	{
		out_contacts[i].depth = math::min(out_contacts[i].depth, CLAMPED_DEPTH_MAX);
		out_contacts[i].skip_expand = i > 0;
	}
}

<*
 A triangle surface (mesh BVH or heightfield) being tested against one body.

 Feed it the surface's local-space triangles that overlap `local_aabb`; it
 transforms them to world space (rotation rows built once, not per vertex) and
 accumulates the manifold. Allocation-free, so it is safe on worker threads.
*>
struct SurfaceQuery
{
	TransformedShape* surface;
	Vec3[3] rows; // The surface's rotation, row-wise
	TriangleProbe probe;
	Aabb3 local_aabb; // The other body's AABB in the surface's local space
	sz count;
}

fn SurfaceQuery begin_surface_query(Rigidbody* surface_body, Rigidbody* other_body, Pair pair, bool surface_is_a)
{
	TransformedShape* surface = &surface_body.collider;
	return {
		.surface = surface,
		.rows = surface.rotation.rows(),
		.probe = triangle_probe(other_body, pair, surface_is_a),
		.local_aabb = other_body.transformed_aabb.inverse_transform(surface.translation, surface.scale, surface.rotation),
	};
}

fn void SurfaceQuery.add_local_triangle(&self, TriangleVerts local, Collision[] out_contacts)
{
	TriangleVerts world;
	foreach (i, v : local)
	{
		world[i] = rotate_by_rows(self.rows, v * self.surface.scale) + self.surface.translation;
	}
	self.count = triangle_contacts(&self.probe, world, out_contacts, self.count);
}

fn sz SurfaceQuery.finish(&self, Collision[] out_contacts)
{
	settle_triangle_manifold(out_contacts, self.count);
	return self.count;
}

<*
 Test a mesh-BVH pair and collect a contact manifold: up to MAX_CONTACTS_PER_PAIR
 deduplicated contacts across the overlapped triangles, deepest first.
 Thread-safe — writes only into the caller-provided out_contacts.
 Returns the number of contacts written.
 @require out_contacts.len >= MAX_CONTACTS_PER_PAIR : `Output buffer too small`
*>
fn sz PhysicsWorld.test_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair, Collision[] out_contacts)
{
	bool mesh_is_a = body_a.has_mesh_bvh();
	Rigidbody* mesh_body = mesh_is_a ? body_a : body_b;
	Rigidbody* other_body = mesh_is_a ? body_b : body_a;

	Mesh* mesh = (Mesh*)mesh_body.collider.shape;
	SurfaceQuery query = begin_surface_query(mesh_body, other_body, pair, mesh_is_a);

	mesh.bvh.@foreach_triangle(query.local_aabb; TriangleVerts tri)
	{
		query.add_local_triangle(tri, out_contacts);
	};
	return query.finish(out_contacts);
}

<*
 Test a heightfield pair the same way, over the triangles under the other body.

 A `Heightmap` cannot be a GJK shape (its support point is one of the map's four
 corners, so a body would rest on the plane through them), so terrain is
 handled per contact: the triangles under the other body go through the same
 one-sided triangle path as a mesh.

 The query box is the other body's AABB in the field's own space, so a rotated or
 scaled heightfield works; contacts are built in world space.

 Thread-safe and allocation-free — writes only into out_contacts.

 @require out_contacts.len >= MAX_CONTACTS_PER_PAIR : `Output buffer too small`
*>
fn sz PhysicsWorld.test_heightfield_collision(
	&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair, Collision[] out_contacts
)
{
	bool field_is_a = body_a.has_heightfield();
	Rigidbody* field_body = field_is_a ? body_a : body_b;
	Rigidbody* other_body = field_is_a ? body_b : body_a;

	Heightmap* field = (Heightmap*)field_body.collider.shape;
	SurfaceQuery query = begin_surface_query(field_body, other_body, pair, field_is_a);

	field.@foreach_triangle(query.local_aabb; TriangleVerts tri)
	{
		query.add_local_triangle(tri, out_contacts);
	};
	return query.finish(out_contacts);
}

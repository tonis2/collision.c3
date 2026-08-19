<*
 EPA — the penetration depth and normal for two shapes GJK has already found to
 overlap, by expanding GJK's terminating simplex until its closest face stops
 moving.

 **The polytope has to stay closed for any of it to mean anything.** EPA's whole
 claim is "the closest face of the polytope to the origin approaches the closest
 point on the Minkowski boundary", and a polytope with a hole in it has faces
 that are closest only because the ones that would have been closer are missing.
 `reconstruct` used to drop a degenerate face and carry on, which opened exactly
 such a hole; the walk then converged on a face on the far side and reported it.
 Measured against a 1-unit box resting on a 100-unit ground slab: a normal of
 (0.70, 0.72, 0) — horizontal, for a body sitting on a floor — at a depth of
 71.2 world units, with contact points that were NaN because the face it came
 from had no area. The solver took that at face value and the body left the
 world. So a step that cannot keep the polytope closed now ends the expansion
 and says the answer is not to be trusted, rather than returning it.

 **`FixedList.push` does not bounds check** — it is `entries[size++] = element`
 — so both arrays here are guarded at their capacity rather than trusted to
 stop. Overrunning them was a write past the end of a stack array.

 **An unreliable answer is reported as no collision at all.** For a resting
 contact that costs one substep of contact force, which the next substep puts
 back; there is no visible sink. Reporting a made-up normal instead costs the
 body its position, its velocity and, once a NaN is in the transform, every
 subsequent frame it appears in.
*>
module collision;

import std::math;
import std::collections::list;
import std::collections::fixedlist;
import std::collections::map;
import std::io;
import std::os;

const sz EPA_MAX_VERTICES = 64;
const sz EPA_MAX_FACES = 64;

alias EdgeArray @private = FixedList {Pair, EPA_MAX_FACES};
alias FaceArray @private = FixedList {Face, EPA_MAX_FACES};
alias SupportArray @private = FixedList {SupportPoint, EPA_MAX_VERTICES};

const uint EPA_MAX_ITER = 32;
const float EPA_TOLERANCE = 0.0001f;
// Below this a triangle has no usable normal: the cross product of its edges is
// shorter than the float noise in the vertices it was built from.
const float EPA_MIN_FACE_AREA_SQ = 1e-12f;
// Two support points closer than this are the same vertex of the Minkowski
// difference. Both shapes are polyhedra, so the support function runs out of
// new vertices to return — that is convergence, not a reason to keep going.
const float EPA_VERTEX_MERGE_SQ = 1e-10f;

fn void EdgeArray.add_if_new(&self, Pair pair)
{
	// Reverse search
	for (int i = (int)self.len() - 1; i >= 0; i--)
	{
		Pair edge = self.get(i);
		if (edge == (Pair) { pair.b, pair.a })
		{
			self.remove_at(i);
			return;
		}
	}

	if (self.len() >= EPA_MAX_FACES) return;
	self.push(pair);
}

<*
 The face on three polytope vertices, oriented away from the origin.

 `active` is false for a triangle with no area. Such a face has no normal to
 speak of — `normalize()` on a zero-length cross product is a NaN in all three
 components, and a NaN normal survives every `<` test written against it — so it
 is flagged here and dropped by the callers rather than left to poison the walk.
*>
fn Face SupportArray.create_face(&self, sz a_index, sz b_index, sz c_index)
{
	Vec3 a = self.get(a_index).point;
	Vec3 b = self.get(b_index).point;
	Vec3 c = self.get(c_index).point;

	Vec3 cross = (b - a).cross(c - a);
	float area_sq = cross.length_sq();

	// Written as a positive test so a NaN area fails it rather than passing.
	if (!(area_sq > EPA_MIN_FACE_AREA_SQ))
	{
		return { .a = a_index, .b = b_index, .c = c_index, .distance = float::max, .normal = { 0, 0, 0 }, .active = false };
	}

	Vec3 normal = cross * (1.0f / math::sqrt(area_sq));
	float distance = a.dot(normal);

	// Point it away from the origin, which is what makes `distance` the
	// origin-to-plane distance and lets `get_closest` compare faces at all.
	if (distance < 0)
	{
		normal = -normal;
		distance = -distance;
	}

	return { .a = a_index, .b = b_index, .c = c_index, .distance = distance, .normal = normal, .active = true };
}

<*
 Replace every face the new point can see with faces joining it to the horizon.

 @return `false if the polytope could not be closed — a degenerate face or a full array. The caller must stop.`
*>
fn bool SupportArray.reconstruct(&self, FaceArray* faces, Vec3 new_point)
{
	EdgeArray edges;

	// Reverse loop
	for (int i = (int)faces.len() - 1; i >= 0; i--)
	{
		Face face = faces.get(i);

		if (face.normal.dot(new_point - self.get(face.a).point) > 0)
		{
			edges.add_if_new({ face.a, face.b });
			edges.add_if_new({ face.b, face.c });
			edges.add_if_new({ face.c, face.a });
			faces.remove_at(i);
		}
	}

	foreach (edge : edges)
	{
		if (faces.len() >= EPA_MAX_FACES) return false;

		Face new_face = self.create_face(edge.a, edge.b, self.len() - 1);
		// A face that cannot be built is a hole, and a hole invalidates every
		// distance comparison from here on. Say so instead of skipping it.
		if (!new_face.active) return false;

		faces.push(new_face);
	}

	return true;
}

<*
 The index of the face nearest the origin, or `self.len` when there is none.

 Degenerate faces are skipped rather than compared: their distance is
 `float::max` and their normal is zero, and either would win a comparison
 written the other way round.
*>
fn uint Face[].get_closest(self, float min_distance = float::max)
{
	uint closest = (uint)self.len;

	foreach (uint i, face : self)
	{
		if (!face.active) continue;

		if (face.distance < min_distance)
		{
			min_distance = face.distance;
			closest = i;
		}
	}

	return closest;
}

fn bool is_finite3(Vec3 v) @local => math::is_finite(v.x) && math::is_finite(v.y) && math::is_finite(v.z);

<*
 @require info.simplex.len() > 3 : `Simplex data must have enough points`
*>
fn void epa(CollisionInfo* info, CollisionShape shape_1, CollisionShape shape_2)
{
	SupportArray vertices;

	foreach (item : info.simplex)
	{
		vertices.push(item);
	}

	FaceArray faces;
	Face[4] seed = {
		vertices.create_face(0, 1, 2),
		vertices.create_face(0, 3, 1),
		vertices.create_face(0, 2, 3),
		vertices.create_face(1, 3, 2),
	};

	// All four or none: a seed tetrahedron missing a face is already the open
	// polytope this whole function exists to avoid.
	foreach (face : seed)
	{
		if (!face.active)
		{
			info.collided = false;
			return;
		}
	}
	faces.push_all(&seed);

	uint closest_face_index = faces.array_view().get_closest();
	if (closest_face_index >= faces.len())
	{
		info.collided = false;
		return;
	}

	Face closest_face = faces[closest_face_index];
	bool converged = false;

	for (uint i = 0; i < EPA_MAX_ITER; ++i)
	{
		closest_face = faces[closest_face_index];

		Vec3 face_normal = closest_face.normal;
		Vec3 a_support = shape_1.furthest_point(face_normal);
		Vec3 b_support = shape_2.furthest_point(-face_normal);

		SupportPoint support_point = { .a = a_support, .b = b_support, .point = a_support - b_support };

		float support_distance = face_normal.dot(support_point.point);

		if (support_distance - closest_face.distance < EPA_TOLERANCE)
		{
			converged = true;
			break;
		}

		// The support function of a polyhedron has finitely many answers. Once
		// it repeats one the polytope cannot grow, and pushing the duplicate
		// builds a zero-area face out of two copies of the same vertex.
		bool duplicate = false;
		foreach (existing : vertices)
		{
			if (existing.point.distance_sq(support_point.point) < EPA_VERTEX_MERGE_SQ)
			{
				duplicate = true;
				break;
			}
		}
		if (duplicate)
		{
			converged = true;
			break;
		}

		if (vertices.len() >= EPA_MAX_VERTICES) break;
		vertices.push(support_point);

		if (!vertices.reconstruct(&faces, support_point.point)) break;

		closest_face_index = faces.array_view().get_closest();
		if (closest_face_index >= faces.len()) break;
	}

	// Ran out of iterations, ran out of room, or opened a hole. Whatever the
	// current closest face says, it is not an answer about these two shapes.
	if (!converged)
	{
		info.collided = false;
		return;
	}

	if (!is_finite3(closest_face.normal) || !math::is_finite(closest_face.distance) || closest_face.distance < 0)
	{
		info.collided = false;
		return;
	}

	info.normal = closest_face.normal;
	info.depth = closest_face.distance;

	// The closest point on the face to the origin, which is along the normal
	// *towards* the face. It read `-info.depth` before, mirroring the point
	// through the origin and taking the barycentric of somewhere the triangle
	// is not.
	Vec3 p = info.normal * info.depth;

	SupportPoint a = vertices[closest_face.a];
	SupportPoint b = vertices[closest_face.b];
	SupportPoint c = vertices[closest_face.c];

	Vec3 bary = p.barycentric(a.point, b.point, c.point);

	if (!is_finite3(bary))
	{
		// Degenerate triangle after all: put the contact at the face's own
		// centroid rather than dividing by its area.
		bary = { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };
	}

	Vec3[2] support_a = { a.a, a.b };
	Vec3[2] support_b = { b.a, b.b };
	Vec3[2] support_c = { c.a, c.b };

	info.contact_points = {
		support_a[0] * bary.x + support_b[0] * bary.y + support_c[0] * bary.z,
		support_a[1] * bary.x + support_b[1] * bary.y + support_c[1] * bary.z,
	};

	if (!is_finite3(info.contact_points[0]) || !is_finite3(info.contact_points[1]))
	{
		info.collided = false;
	}
}

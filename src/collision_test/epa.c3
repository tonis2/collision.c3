<*
 EPA: the penetration depth and normal for two shapes GJK has found to overlap,
 by expanding GJK's terminating simplex until its closest face stops moving.

 `FixedList.push` does not bounds check (`entries[size++] = element`), so both
 arrays here are guarded at their capacity rather than trusted to stop.

 An unreliable answer is reported as no collision at all. For a resting contact
 that costs one substep of contact force, which the next substep restores;
 reporting a made-up normal would instead risk NaNs in the body's transform.
*>
module collision;

import std::math;
import std::collections::list;
import std::collections::fixedlist;
import std::collections::map;

const sz EPA_MAX_VERTICES = 64;
const sz EPA_MAX_FACES = 64;

alias EdgeArray @private = FixedList {Pair, EPA_MAX_FACES};
alias FaceArray @private = FixedList {Face, EPA_MAX_FACES};
alias SupportArray @private = FixedList {SupportPoint, EPA_MAX_VERTICES};

const uint EPA_MAX_ITER = 32;
const float EPA_TOLERANCE = 0.0001f;
// Below this a triangle has no usable normal (its edge cross product is float noise).
const float EPA_MIN_FACE_AREA_SQ = 1e-12f;
// Two support points closer than this are the same vertex of the Minkowski
// difference: polyhedra run out of new support vertices, which is convergence.
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

 `active` is false for a triangle with no area: its normal would be NaN, which
 survives every `<` comparison, so callers drop such faces instead.
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

	// Point away from the origin so `distance` is the origin-to-plane distance.
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
		// A face that cannot be built is a hole, which invalidates every later comparison.
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
fn uint Face[].get_closest(self)
{
	float min_distance = float::max;
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

<*
 Fills `info.normal`, `depth` and `contact_points` from the simplex GJK ended on.

 @return `false when no reliable answer was found (iteration/size limit, open polytope, non-finite result).`
 @require info.simplex.len() > 3 : `Simplex data must have enough points`
*>
fn bool epa_core(CollisionInfo* info, CollisionShape shape_1, CollisionShape shape_2) @local
{
	SupportArray vertices;

	foreach (item : info.simplex) vertices.push(item);

	FaceArray faces;
	Face[4] seed = {
		vertices.create_face(0, 1, 2),
		vertices.create_face(0, 3, 1),
		vertices.create_face(0, 2, 3),
		vertices.create_face(1, 3, 2),
	};

	// All four or none: a seed tetrahedron missing a face is already an open polytope.
	foreach (face : seed)
	{
		if (!face.active) return false;
	}
	faces.push_all(&seed);

	uint closest_face_index = faces.array_view().get_closest();
	if (closest_face_index >= faces.len()) return false;

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

		// A polyhedron's support function has finitely many answers; once it
		// repeats one the polytope cannot grow, and pushing the duplicate would
		// build a zero-area face from two copies of the same vertex.
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

	// Out of iterations, out of room, or an open polytope: not an answer about these shapes.
	if (!converged) return false;

	if (!is_finite3(closest_face.normal) || !math::is_finite(closest_face.distance) || closest_face.distance < 0) return false;

	info.normal = closest_face.normal;
	info.depth = closest_face.distance;

	// The closest point on the face to the origin lies along the normal, towards the face.
	Vec3 p = info.normal * info.depth;

	SupportPoint a = vertices[closest_face.a];
	SupportPoint b = vertices[closest_face.b];
	SupportPoint c = vertices[closest_face.c];

	Vec3 bary = p.barycentric(a.point, b.point, c.point);

	// Degenerate triangle after all: use the face centroid rather than dividing by its area.
	if (!is_finite3(bary)) bary = { 1.0f / 3.0f, 1.0f / 3.0f, 1.0f / 3.0f };

	info.contact_points = {
		a.a * bary.x + b.a * bary.y + c.a * bary.z,
		a.b * bary.x + b.b * bary.y + c.b * bary.z,
	};

	return is_finite3(info.contact_points[0]) && is_finite3(info.contact_points[1]);
}

<*
 Runs EPA on a colliding `info`; clears `info.collided` when no reliable answer exists.
*>
fn void epa(CollisionInfo* info, CollisionShape shape_1, CollisionShape shape_2)
{
	if (!epa_core(info, shape_1, shape_2)) info.collided = false;
}

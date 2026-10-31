module collision;
import std::math, std::collections::list;

// Vertices closer to a cutting plane than this are on it, not on either side.
const double CLIP_EPSILON @local = 1e-6;

// A cut must leave at least this many vertices (a tetrahedron) on each side, and
// vertices within this distance of the plane count as on neither.
const int MIN_VERTICES_PER_SIDE @local = 4;
const double SIDE_EPSILON @local = 0.001;

// Triangles whose normal is shorter than this have no usable cut direction.
const float MIN_NORMAL_LENGTH @local = 0.001f;

// At most this many triangles are sampled for candidate cut planes per part.
const sz CUT_SAMPLE_TRIANGLES @local = 50;

// Candidate cuts examined per part = plane_candidates * this.
const int CANDIDATES_PER_PLANE_SETTING @local = 3;

// Parts below this fraction of the mesh diagonal are dropped as slivers; larger
// ones still keep thin surfaces such as floors.
const double MIN_HULL_DIAGONAL_FRACTION @local = 0.005;

// Page size of the arena that holds the parts being split.
const sz ARENA_PAGE_SIZE @local = 64 * 1024;

// Hollow shapes need many splits to separate their walls.
const int MAX_SPLIT_DEPTH @local = 8;

// -1, 0 or +1: which side of a plane a signed distance falls on.
fn int side_of(double d, double eps) @local => d > eps ? 1 : (d < -eps ? -1 : 0);

// Split one triangle by a plane, sending its pieces to the two meshes.
// Vertices on the plane go to both sides.
fn void clip_triangle(Mesh* mesh, Triangle tri, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh) @local
{
	TriangleVerts verts = tri.verts(mesh.vertices.array_view());

	double[3] dists;
	int[3] sides;
	int pos_count = 0;
	int neg_count = 0;
	for (int i = 0; i < 3; i++)
	{
		dists[i] = plane.distance(verts[i]);
		sides[i] = side_of(dists[i], CLIP_EPSILON);
		if (sides[i] > 0) pos_count++;
		if (sides[i] < 0) neg_count++;
	}

	if (neg_count == 0)
	{
		pos_mesh.add_triangle(verts[0], verts[1], verts[2]);
		return;
	}
	if (pos_count == 0)
	{
		neg_mesh.add_triangle(verts[0], verts[1], verts[2]);
		return;
	}

	// The triangle crosses the plane: walk its edges, keeping each vertex on its
	// side(s) and adding the intersection point to both where an edge crosses.
	Vec3[4] pos_poly;
	Vec3[4] neg_poly;
	int pos_n = 0;
	int neg_n = 0;

	for (int i = 0; i < 3; i++)
	{
		int j = (i + 1) % 3;
		if (sides[i] >= 0) pos_poly[pos_n++] = verts[i];
		if (sides[i] <= 0) neg_poly[neg_n++] = verts[i];

		if (sides[i] * sides[j] < 0)
		{
			Vec3 crossing = intersect_edge(verts[i], verts[j], dists[i], dists[j]);
			pos_poly[pos_n++] = crossing;
			neg_poly[neg_n++] = crossing;
		}
	}

	pos_mesh.add_fan(pos_poly[:pos_n]);
	neg_mesh.add_fan(neg_poly[:neg_n]);
}

// No cap surfaces: collision only needs the vertices, not watertight pieces.
fn void clip_mesh(Mesh* mesh, Plane plane, Mesh* pos_mesh, Mesh* neg_mesh) @local
{
	foreach (tri : mesh.triangles) clip_triangle(mesh, tri, plane, pos_mesh, neg_mesh);
}

// ============================================================================
// CoACD - Collision-Aware Convex Decomposition
// Based on: "CoACD: Collision-Aware Convex Decomposition" (Wei et al.)
// Reference: https://github.com/SarahWeiii/CoACD
// ============================================================================

struct CoacdParameters
{
	float threshold; // Concavity threshold (0.01-0.1, lower = more precise)
	int max_hulls; // Maximum output hull count
	int plane_candidates; // Candidate planes per axis for cut search
	int max_iterations; // Maximum decomposition iterations
	float k; // Volume metric weight (default 0.3)
}

// Preset configurations
const CoacdParameters COACD_VERY_FAST = {
	.threshold = 0.15,
	.max_hulls = 8,
	.plane_candidates = 3,
	.max_iterations = 50,
	.k = 0.3
};

const CoacdParameters COACD_FAST = {
	.threshold = 0.04,
	.max_hulls = 32,
	.plane_candidates = 5,
	.max_iterations = 100,
	.k = 0.3
};

const CoacdParameters COACD_MEDIUM = {
	.threshold = 0.05,
	.max_hulls = 32,
	.plane_candidates = 8,
	.max_iterations = 150,
	.k = 0.3
};

const CoacdParameters COACD_SLOW = {
	.threshold = 0.03,
	.max_hulls = 64,
	.plane_candidates = 12,
	.max_iterations = 300,
	.k = 0.3
};

// ----------------------------------------------------------------------------
// Cut search
// ----------------------------------------------------------------------------

// Balance of a cut: larger-side / smaller-side vertex count (lower is better),
// or float::max when either side has too few vertices to form a hull.
fn float score_cut(Mesh* mesh, Plane cut) @local
{
	int pos_count = 0;
	int neg_count = 0;

	foreach (v : mesh.vertices)
	{
		int side = side_of(cut.distance(v), SIDE_EPSILON);
		if (side > 0) pos_count++;
		if (side < 0) neg_count++;
	}

	if (pos_count < MIN_VERTICES_PER_SIDE || neg_count < MIN_VERTICES_PER_SIDE) return float::max;
	return (float)math::max(pos_count, neg_count) / (float)math::min(pos_count, neg_count);
}

// Find the best cutting plane among the planes of large triangles. For hollow
// shapes the inner faces point into the cavity, so cutting along them separates
// the walls. Falls back to the middle of the longest bounding-box axis.
fn Plane find_best_cut(Mesh* mesh, Aabb3 bounds, CoacdParameters* params) @local
{
	Vec3[] verts = mesh.vertices.array_view();
	Plane best_cut;
	float best_score = float::max;

	if (!mesh.triangles.is_empty())
	{
		double total_area = 0;
		foreach (tri : mesh.triangles) total_area += tri.get_area(verts);
		double min_area = total_area / (double)mesh.triangles.len() * 0.5; // At least half the average

		sz stride = math::max((sz)1, mesh.triangles.len() / CUT_SAMPLE_TRIANGLES);
		int candidates_checked = 0;
		int max_candidates = params.plane_candidates * CANDIDATES_PER_PLANE_SETTING;

		for (sz i = 0; i < mesh.triangles.len() && candidates_checked < max_candidates; i += stride)
		{
			Triangle tri = mesh.triangles[i];
			if (tri.get_area(verts) < min_area) continue;

			Vec3 normal = tri.get_normal(verts);
			if (normal.length() < MIN_NORMAL_LENGTH) continue;

			normal = normal.normalize();
			double d = normal.dot(tri.get_center(verts));
			candidates_checked++;

			// Both orientations of the plane: the sides swap, the balance may not.
			Plane[2] orientations = { { normal, d }, { -normal, -d } };
			foreach (cut : orientations)
			{
				float score = score_cut(mesh, cut);
				if (score < best_score)
				{
					best_score = score;
					best_cut = cut;
				}
			}
		}
	}

	if (best_score >= float::max)
	{
		sz axis = bounds.longest_axis();
		Vec3 normal = { 0, 0, 0 };
		normal[axis] = 1;
		best_cut = { normal, bounds.center()[axis] };
	}

	return best_cut;
}

// ----------------------------------------------------------------------------
// Decomposition: keep splitting until the pieces are convex (or deep enough).
// ----------------------------------------------------------------------------

// Part in the decomposition queue with depth tracking. Its mesh is scratch on the
// arena of `decompose_parts`.
struct DecompPart @local
{
	Mesh mesh;
	int depth;
}

// Take ownership of `hull` into `result` if it is a real solid at least
// `min_diagonal` across; otherwise free it.
fn void push_hull(List {Mesh}* result, Mesh hull, double min_diagonal) @local
{
	if (hull.vertices.len() >= 4 && hull.get_aabb().size().length() >= min_diagonal)
	{
		result.push(hull);
	}
	else
	{
		hull.free();
	}
}

<*
 Decompose `mesh` into convex hulls. The input is not modified (the work runs on
 a validated, welded copy), so its vertices, triangles and BVH stay valid.

 The returned list and hulls are on the heap; free them with `MeshList.release`.
 All intermediate parts live on an arena that is released on return.

 @param [&in] mesh
*>
fn List {Mesh} coacd_decompose(Mesh* mesh, CoacdParameters params = COACD_MEDIUM)
{
	List {Mesh} result;
	result.init(mem);
	if (mesh.vertices.len() < 4 || mesh.triangles.len() < 2) return result;

	decompose_parts(mesh, &params, &result);
	return result;
}

<*
 Splitting runs on its own arena rather than the temp allocator: the parts pile up
 over the run, and once the temp page is nearly full every nested `@pool` (in
 `remesh`, `quickhull`) would allocate and fill a fresh page of its own.
*>
fn void decompose_parts(Mesh* mesh, CoacdParameters* params, List {Mesh}* result) @local
{
	DynamicArenaAllocator arena;
	arena.init(mem, ARENA_PAGE_SIZE);
	defer arena.free();

	Mesh work;
	work.init(&arena, mesh.vertices.len(), mesh.triangles.len());
	work.vertices.push_all(mesh.vertices.array_view());
	work.triangles.push_all(mesh.triangles.array_view());
	work.validate();
	work.remesh();

	if (work.is_convex())
	{
		push_hull(result, quickhull(work.vertices.array_view()), 0);
		return;
	}

	Aabb3 bounds = work.get_aabb();
	double min_hull_diagonal = bounds.size().length() * MIN_HULL_DIAGONAL_FRACTION;

	// FIFO queue walked by `head`; consumed slots are never revisited.
	List {DecompPart} queue;
	queue.init(&arena);
	queue.push({ .mesh = work, .depth = 0 });

	sz head = 0;
	int iteration = 0;

	while (head < queue.len() && result.len() < params.max_hulls && iteration < params.max_iterations)
	{
		iteration++;

		DecompPart part = queue[head++];

		if (part.depth >= MAX_SPLIT_DEPTH || part.mesh.is_convex())
		{
			push_hull(result, quickhull(part.mesh.vertices.array_view()), min_hull_diagonal);
			continue;
		}

		Plane cut = find_best_cut(&part.mesh, part.mesh.get_aabb(), params);

		Mesh pos_mesh;
		Mesh neg_mesh;
		pos_mesh.init(&arena);
		neg_mesh.init(&arena);
		clip_mesh(&part.mesh, cut, &pos_mesh, &neg_mesh);
		pos_mesh.remesh();
		neg_mesh.remesh();

		bool pos_valid = pos_mesh.vertices.len() >= 4 && pos_mesh.triangles.len() >= 2;
		bool neg_valid = neg_mesh.vertices.len() >= 4 && neg_mesh.triangles.len() >= 2;

		if (!pos_valid && !neg_valid)
		{
			// Can't split further: keep the part as one hull.
			push_hull(result, quickhull(part.mesh.vertices.array_view()), min_hull_diagonal);
			continue;
		}

		int new_depth = part.depth + 1;
		if (pos_valid) queue.push({ .mesh = pos_mesh, .depth = new_depth });
		if (neg_valid) queue.push({ .mesh = neg_mesh, .depth = new_depth });
	}

	// Whatever is still queued becomes a hull each, up to the limit.
	for (sz i = head; i < queue.len(); i++)
	{
		if (result.len() >= params.max_hulls) break;
		push_hull(result, quickhull(queue[i].mesh.vertices.array_view()), min_hull_diagonal);
	}
}

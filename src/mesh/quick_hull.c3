module collision;

import std::math;
import std::collections::list;


// QuickHull 3D Convex Hull Algorithm
// Used for computing collision meshes from complex GLTF geometry
//
// The face set is kept edge-adjacent: every face knows the face across each of
// its three edges, and faces are only ever removed by a flood fill that starts
// at the face the inserted point was assigned to and crosses into neighbours.
// That connectivity is the load-bearing part. Deciding visibility per face,
// with no regard for whether the visible faces touch, calls two faces that do
// not share an edge "visible" as soon as a point sits within epsilon of both -
// which the near-coplanar panels of a real glTF mesh do constantly - and coning
// the new vertex to two disjoint horizon loops leaves a surface whose edges are
// shared by more than two faces. Each later pass then finds one horizon edge
// per face along such an edge, so the horizon, and with it the face count,
// multiplies: on truck.glb's third primitive it went 25 -> 109 -> 2867 ->
// 1677983 and the run never came back. See test_hull_truck_panel.

// Use type-appropriate epsilon - float has ~7 digits, double has ~15
// Vec3::size is 12 for float[<3>] (3*4) or 24 for double[<3>] (3*8)
const double HULL_EPSILON = Vec3::size == 12 ? 1e-5 : 1e-10;

// Neighbour slot of a face whose adjacency has not been filled in yet
const sz NO_FACE @local = -1;

// Face of the convex hull during construction
struct HullFace @local
{
	sz[3] vertices; // Indices into point array
	sz[3] neighbours; // Face across edge vertices[i] -> vertices[(i + 1) % 3]
	double[<3>] normal;
	double distance; // Distance from origin along normal
	bool alive; // False if face has been removed
	uint stamp; // Flood fill generation, see find_horizon
	List {int} outside; // Points outside this face
}

// One edge of the horizon: edge `edge` of face `face`, i.e. the directed edge
// face.vertices[edge] -> face.vertices[(edge + 1) % 3]
struct HorizonEdge @local
{
	sz face;
	int edge;
}

// One frame of the horizon walk - the face being visited and which of its
// edges the walk has already looked at, counted from `start`
struct HorizonStep @local
{
	sz face;
	int start;
	int step;
}

fn void HullFace.free(&self)
{
	self.outside.free();
}

// The hull's planes are held in double even though the points are float.
// A thin triangle loses most of the significant bits of its cross product to
// cancellation, and in float there are few enough left that the normal points
// somewhere invented - which is how a hull stops being convex without any
// topological fault, its own vertices reading as far outside one of its faces.
fn double[<3>] to_double(Vec3 p) @local
{
	return { (double)p.x, (double)p.y, (double)p.z };
}

// Compute face normal and distance from 3 points.
// Returns false when the three points are collinear: the cross product is zero
// there and normalizing it stores a NaN plane, which answers false to every
// comparison and so strands the face in the hull with no way to remove it.
fn bool HullFace.compute_plane(&self, Vec3[] points)
{
	double[<3>] a = to_double(points[self.vertices[0]]);
	double[<3>] b = to_double(points[self.vertices[1]]);
	double[<3>] c = to_double(points[self.vertices[2]]);

	double[<3>] cross = (b - a).cross(c - a);
	double len = cross.length();
	// Written this way on purpose: !(len > 0) also rejects a NaN length
	if (!(len > 0)) return false;

	self.normal = cross * (1.0 / len);
	self.distance = self.normal.dot(a);
	return true;
}

// Signed distance from point to face plane
fn double HullFace.point_distance(&self, Vec3 p)
{
	return self.normal.dot(to_double(p)) - self.distance;
}

// Tolerance for "is this point outside that plane", scaled to the input.
// An absolute epsilon is meaningless: on a metre-scale mesh it sits below the
// float noise of the coordinates themselves, and on a millimetre-scale one it
// swallows the model. The magnitude sum is the usual QuickHull choice.
fn double hull_tolerance(Vec3[] points) @local
{
	Vec3 extent = { 0, 0, 0 };
	foreach (p : points)
	{
		extent.x = math::max(extent.x, math::abs(p.x));
		extent.y = math::max(extent.y, math::abs(p.y));
		extent.z = math::max(extent.z, math::abs(p.z));
	}
	return HULL_EPSILON * (double)(extent.x + extent.y + extent.z);
}

// Find the farthest point from origin along a direction
fn int find_farthest_point(Vec3[] points, Vec3 direction) @local
{
	int best = 0;
	double best_dist = direction.dot(points[0]);

	for (sz i = 1; i < points.len; i++)
	{
		double dist = direction.dot(points[i]);
		if (dist > best_dist)
		{
			best_dist = dist;
			best = (int)i;
		}
	}
	return best;
}

// Find extreme points along each axis to build initial simplex
fn int[6] find_extreme_points_hull(Vec3[] points) @local
{
	int[6] extreme = { 0, 0, 0, 0, 0, 0 };

	for (sz i = 1; i < points.len; i++)
	{
		if (points[i].x < points[extreme[0]].x) extreme[0] = (int)i;
		if (points[i].x > points[extreme[1]].x) extreme[1] = (int)i;
		if (points[i].y < points[extreme[2]].y) extreme[2] = (int)i;
		if (points[i].y > points[extreme[3]].y) extreme[3] = (int)i;
		if (points[i].z < points[extreme[4]].z) extreme[4] = (int)i;
		if (points[i].z > points[extreme[5]].z) extreme[5] = (int)i;
	}
	return extreme;
}

// Find two points that are farthest apart
fn int[2] find_most_separated(Vec3[] points, int[6] extreme) @local
{
	double max_dist = 0;
	int[2] result = { 0, 1 };

	for (int i = 0; i < 6; i++)
	{
		for (int j = i + 1; j < 6; j++)
		{
			Vec3 diff = points[extreme[i]] - points[extreme[j]];
			double dist = diff.dot(diff);
			if (dist > max_dist)
			{
				max_dist = dist;
				result[0] = extreme[i];
				result[1] = extreme[j];
			}
		}
	}
	return result;
}

// Find point farthest from line defined by two points.
// Returns -1 when every point is within tolerance of the line, i.e. the input
// is collinear and has no triangle to start from.
fn int find_farthest_from_line(Vec3[] points, int p0, int p1, double tolerance) @local
{
	Vec3 a = points[p0];
	Vec3 b = points[p1];
	Vec3 ab = b - a;
	double ab_len_sq = ab.dot(ab);
	if (ab_len_sq <= 0) return -1;

	// Compared against squared distances below
	double max_dist = tolerance * tolerance;
	int result = -1;

	for (sz i = 0; i < points.len; i++)
	{
		if ((int)i == p0 || (int)i == p1) continue;

		Vec3 ap = points[i] - a;
		double t = ap.dot(ab) / ab_len_sq;
		Vec3 closest = a + ab * (Vec3) { (float)t, (float)t, (float)t };
		Vec3 diff = points[i] - closest;
		double dist = diff.dot(diff);

		if (dist > max_dist)
		{
			max_dist = dist;
			result = (int)i;
		}
	}
	return result;
}

// Find point farthest from plane defined by three points.
// Returns -1 when every point is within tolerance of the plane, i.e. the input
// is coplanar and has no volume to enclose.
fn int find_farthest_from_plane(Vec3[] points, int p0, int p1, int p2, double tolerance) @local
{
	double[<3>] a = to_double(points[p0]);
	double[<3>] b = to_double(points[p1]);
	double[<3>] c = to_double(points[p2]);

	double[<3>] cross = (b - a).cross(c - a);
	double len = cross.length();
	if (!(len > 0)) return -1;

	double[<3>] normal = cross * (1.0 / len);
	double d = normal.dot(a);

	double max_dist = tolerance;
	int result = -1;

	for (sz i = 0; i < points.len; i++)
	{
		if ((int)i == p0 || (int)i == p1 || (int)i == p2) continue;

		double dist = math::abs(normal.dot(to_double(points[i])) - d);
		if (dist > max_dist)
		{
			max_dist = dist;
			result = (int)i;
		}
	}
	return result;
}

// Record that edge `ea` of face `fa` and edge `eb` of face `fb` are the same edge
fn void link_edge(List {HullFace}* faces, sz fa, int ea, sz fb, int eb) @local
{
	faces.get_ref(fa).neighbours[ea] = fb;
	faces.get_ref(fb).neighbours[eb] = fa;
}

// Index of the edge of `face` that is shared with face `other`.
// Adjacency is only ever written in pairs, so the search always finds it; two
// faces can never share two edges because a face's three indices are distinct.
fn int shared_edge(HullFace* face, sz other) @local
{
	for (int e = 0; e < 3; e++)
	{
		if (face.neighbours[e] == other) return e;
	}
	return -1;
}

// Build initial tetrahedron from 4 non-coplanar points, into an empty face list.
// Writes the four chosen point indices to `tetra` and returns false when the
// input has no volume to enclose - fewer than four points, or all of them
// equal, collinear or coplanar within tolerance.
fn bool build_initial_tetrahedron(Vec3[] points, List {HullFace}* faces, double tolerance, sz[4]* tetra) @local
{
	if (points.len < 4) return false;

	// Find extreme points
	int[6] extreme = find_extreme_points_hull(points);

	// Find two most separated points
	int[2] base = find_most_separated(points, extreme);
	int p0 = base[0];
	int p1 = base[1];

	if (p0 == p1) return false;

	// Duplicated points give two distinct indices for one location, so the
	// indices differing is not enough - the span has to be real
	if ((points[p1] - points[p0]).length() <= tolerance) return false;

	// Find third point farthest from line p0-p1
	int p2 = find_farthest_from_line(points, p0, p1, tolerance);
	if (p2 < 0) return false;

	// Find fourth point farthest from plane p0-p1-p2
	int p3 = find_farthest_from_plane(points, p0, p1, p2, tolerance);
	if (p3 < 0) return false;

	// Check if p3 is above or below the plane to ensure consistent winding
	Vec3 normal = (points[p1] - points[p0]).cross(points[p2] - points[p0]);
	double dist = normal.dot(points[p3] - points[p0]);

	// Swap p1 and p2 if p3 is above the plane (ensures outward normals)
	if (dist > 0)
	{
		int temp = p1;
		p1 = p2;
		p2 = temp;
	}

	// Create 4 faces of tetrahedron with outward-facing normals. The neighbours
	// start at NO_FACE rather than at the zero a plain struct would give them,
	// which would read as "face 0" and be wrong quietly.
	HullFace f0 = { .vertices = { p0, p1, p2 }, .neighbours = { NO_FACE, NO_FACE, NO_FACE }, .alive = true };
	HullFace f1 = { .vertices = { p0, p2, p3 }, .neighbours = { NO_FACE, NO_FACE, NO_FACE }, .alive = true };
	HullFace f2 = { .vertices = { p0, p3, p1 }, .neighbours = { NO_FACE, NO_FACE, NO_FACE }, .alive = true };
	HullFace f3 = { .vertices = { p1, p3, p2 }, .neighbours = { NO_FACE, NO_FACE, NO_FACE }, .alive = true };

	if (!f0.compute_plane(points) || !f1.compute_plane(points)
		|| !f2.compute_plane(points) || !f3.compute_plane(points))
	{
		return false;
	}

	faces.push(f0);
	faces.push(f1);
	faces.push(f2);
	faces.push(f3);

	// Twin edges, read off the winding above: face 0 walks p0->p1->p2->p0,
	// face 1 p0->p2->p3->p0, face 2 p0->p3->p1->p0, face 3 p1->p3->p2->p1
	link_edge(faces, 0, 0, 2, 2); // p0-p1
	link_edge(faces, 0, 1, 3, 2); // p1-p2
	link_edge(faces, 0, 2, 1, 0); // p2-p0
	link_edge(faces, 1, 1, 3, 1); // p2-p3
	link_edge(faces, 1, 2, 2, 0); // p3-p0
	link_edge(faces, 2, 1, 3, 0); // p3-p1

	*tetra = { p0, p1, p2, p3 };
	return true;
}

// Assign each point to the face it's farthest outside of
fn void assign_points_to_faces(Vec3[] points, List {HullFace}* faces, bool[] hull_vertices, double tolerance) @local
{
	for (sz i = 0; i < points.len; i++)
	{
		// Skip the points the initial tetrahedron was built from
		if (hull_vertices[i]) continue;

		// Find face this point is farthest outside of
		double max_dist = tolerance;
		int best_face = -1;

		for (sz f = 0; f < faces.len(); f++)
		{
			if (!faces.get(f).alive) continue;
			double dist = faces.get_ref(f).point_distance(points[i]);
			if (dist > max_dist)
			{
				max_dist = dist;
				best_face = (int)f;
			}
		}

		if (best_face >= 0)
		{
			faces.get_ref(best_face).outside.push((int)i);
		}
	}
}

// Mark every face visible from the eye point that is reachable across edges
// from `seed`, and collect the horizon - the edges where the visible region
// meets a face that is not visible - in loop order.
//
// Visible faces are stamped with `stamp`; nothing is cleared between calls,
// which is why the stamp is a generation counter and not a flag.
//
// A face counts as visible if the eye is on its outer side at all, tolerance or
// no tolerance. The tolerance decides which points are worth inserting; it has
// no business deciding which faces an accepted point can see. Stopping the walk
// at a face the point is only just outside of leaves that face in the hull with
// the new vertex outside it, and every later insertion builds on the dent: on a
// 2000 point sphere that left points eighty times the tolerance outside their
// own hull, and on truck.glb's second primitive twelve times.
fn void find_horizon(
	List {HullFace}* faces,
	Vec3 eye_point,
	sz seed,
	uint stamp,
	List {HorizonEdge}* horizon,
	List {sz}* visible
) @local
{
	List {HorizonStep} walk;
	defer walk.free();

	faces.get_ref(seed).stamp = stamp;
	visible.push(seed);
	walk.push({ .face = seed, .start = 0, .step = 0 });

	// Depth-first across edges. Entering a face at the edge just crossed and
	// walking its edges from there is what puts the horizon in loop order.
	while (walk.len() > 0)
	{
		HorizonStep* top = walk.get_ref(walk.len() - 1);
		if (top.step == 3)
		{
			walk.remove_at(walk.len() - 1);
			continue;
		}

		sz face = top.face;
		int edge = (top.start + top.step) % 3;
		top.step++;

		sz next = faces.get_ref(face).neighbours[edge];
		if (next == NO_FACE) continue;
		if (faces.get(next).stamp == stamp) continue; // already visible

		if (faces.get_ref(next).point_distance(eye_point) > 0)
		{
			faces.get_ref(next).stamp = stamp;
			visible.push(next);
			int back = shared_edge(faces.get_ref(next), face);
			if (back < 0) continue; // adjacency is written in pairs, so unreachable
			walk.push({ .face = next, .start = (back + 1) % 3, .step = 0 });
		}
		else
		{
			horizon.push({ .face = face, .edge = edge });
		}
	}
}

// A horizon is usable only if its edges form one simple closed loop: edge i has
// to end where edge i + 1 begins, and no vertex may be visited twice. Both are
// checked rather than assumed. A horizon that pinches at a vertex still chains
// end to start, but coning it puts three faces on the two edges that meet the
// new vertex there, and from then on the hull is not a surface any more.
// Skipping such a point costs a little accuracy; the alternative costs the hull.
fn bool horizon_is_simple(
	List {HullFace}* faces,
	List {HorizonEdge}* horizon,
	uint[] vertex_stamp,
	uint stamp
) @local
{
	sz count = horizon.len();
	if (count < 3) return false;

	for (sz i = 0; i < count; i++)
	{
		HorizonEdge e = horizon.get(i);
		HorizonEdge n = horizon.get((i + 1) % count);
		sz begin = faces.get(e.face).vertices[e.edge];
		sz end = faces.get(e.face).vertices[(e.edge + 1) % 3];
		if (end != faces.get(n.face).vertices[n.edge]) return false;
		if (vertex_stamp[begin] == stamp) return false;
		vertex_stamp[begin] = stamp;
	}
	return true;
}

// Cone the new vertex to the horizon loop.
// Every new face is linked to the face across its horizon edge and to its two
// neighbours in the cone, so the face set stays closed and edge-manifold.
fn void build_cone(List {HullFace}* faces, Vec3[] points, int apex, List {HorizonEdge}* horizon) @local
{
	sz first = faces.len();
	sz count = horizon.len();

	for (sz i = 0; i < count; i++)
	{
		HorizonEdge edge = horizon.get(i);
		HullFace* retired = faces.get_ref(edge.face);
		sz a = retired.vertices[edge.edge];
		sz b = retired.vertices[(edge.edge + 1) % 3];
		sz outer = retired.neighbours[edge.edge];

		// Edge 0 is the horizon edge, edge 1 (b -> apex) is the next cone
		// face's edge 2, edge 2 (apex -> a) is the previous one's edge 1
		HullFace face = {
			.vertices = { a, b, apex },
			.neighbours = { outer, first + (i + 1) % count, first + (i + count - 1) % count },
			.alive = true,
		};

		if (!face.compute_plane(points))
		{
			// Sliver: the new vertex is collinear with the horizon edge. Keep
			// the plane of the face across that edge, so the triangle still
			// closes the hull but never claims a point of its own.
			face.normal = faces.get(outer).normal;
			face.distance = faces.get(outer).distance;
		}

		faces.push(face);

		// And point the face on the far side of the horizon edge back at it
		HullFace* far = faces.get_ref(outer);
		int back = shared_edge(far, edge.face);
		if (back >= 0) far.neighbours[back] = first + i;
	}
}

// Redistribute points from the faces just removed to the faces just created
fn void redistribute_points(
	List {HullFace}* faces,
	Vec3[] points,
	sz first_new,
	List {sz}* retired,
	bool[] hull_vertices,
	double tolerance
) @local
{
	foreach (ri : *retired)
	{
		HullFace* dead = faces.get_ref(ri);
		foreach (pi : dead.outside)
		{
			// Skip points that are now hull vertices
			if (hull_vertices[pi]) continue;

			double max_dist = tolerance;
			int best_face = -1;

			for (sz f = first_new; f < faces.len(); f++)
			{
				double dist = faces.get_ref(f).point_distance(points[pi]);
				if (dist > max_dist)
				{
					max_dist = dist;
					best_face = (int)f;
				}
			}

			if (best_face >= 0)
			{
				faces.get_ref(best_face).outside.push(pi);
			}
		}
		dead.outside.clear();
	}
}

// Compute convex hull of a point cloud using QuickHull algorithm.
// Returns an empty mesh for input that encloses no volume: fewer than four
// points, or points that are all equal, collinear or coplanar.
//
// What comes back is a closed, outward-facing, convex surface with every input
// point inside it or on it, to within HULL_EPSILON of how far the input reaches
// from the origin. It can hold a zero-area triangle, where an input point ends
// up exactly on an edge of the finished hull - the seam it covers has no area,
// and dropping it would leave a hole in a surface that is otherwise closed.
fn Mesh quickhull(Vec3[] points)
{
	Mesh result;

	if (points.len < 4)
	{
		// Degenerate case - return empty or simple mesh
		return result;
	}

	double tolerance = hull_tolerance(points);

	List {HullFace} faces;
	defer
	{
		foreach (&f : faces)
		{
			f.free();
		}
		faces.free();
	}

	// Build initial tetrahedron
	sz[4] tetra;
	if (!build_initial_tetrahedron(points, &faces, tolerance, &tetra))
	{
		return result;
	}

	// Track which points are already hull vertices (prevents infinite loops with float precision)
	bool[] hull_vertices = mem::new_array(bool, points.len);
	defer mem::free(hull_vertices);
	// Generation stamps for the horizon check below, same counter as the faces
	uint[] vertex_stamp = mem::new_array(uint, points.len);
	defer mem::free(vertex_stamp);
	hull_vertices[tetra[0]] = true;
	hull_vertices[tetra[1]] = true;
	hull_vertices[tetra[2]] = true;
	hull_vertices[tetra[3]] = true;

	// Assign all other points to faces
	assign_points_to_faces(points, &faces, hull_vertices, tolerance);

	// Faces that still hold outside points, walked front to back and only ever
	// appended to. A face is normally queued once: it can gain points only in
	// the pass that creates it, and the pass that takes its farthest point
	// removes the face. The one re-queue is the skipped-point path below.
	List {sz} pending;
	defer pending.free();
	for (sz i = 0; i < faces.len(); i++)
	{
		if (faces[i].outside.len() > 0) pending.push(i);
	}

	List {sz} visible;
	List {HorizonEdge} horizon;
	defer
	{
		visible.free();
		horizon.free();
	}

	// No iteration cap, and none needed: a pass either walks past a queue entry
	// or marks a point as a hull vertex, after which nothing can put that point
	// back into an outside list. Points and queue entries are both finite, so
	// the loop ends whether or not the geometry behaves.
	uint stamp = 0;

	for (sz q = 0; q < pending.len(); q++)
	{
		sz seed = pending[q];
		if (!faces[seed].alive) continue;

		// Find farthest outside point and its index in the list
		double max_dist = tolerance;
		int farthest = -1;
		sz farthest_idx = 0;

		for (sz j = 0; j < faces[seed].outside.len(); j++)
		{
			int pi = faces[seed].outside[j];
			// Skip points already in hull (shouldn't happen, but safety check)
			if (hull_vertices[pi]) continue;

			double dist = faces[seed].point_distance(points[pi]);
			if (dist > max_dist)
			{
				max_dist = dist;
				farthest = pi;
				farthest_idx = j;
			}
		}

		if (farthest < 0) continue;

		// Offered to the hull exactly once, whether it is taken below or not:
		// a point that can come back can restart the whole loop
		hull_vertices[farthest] = true;
		faces[seed].outside.remove_at(farthest_idx);

		// Everything the new vertex can see, found by crossing edges from the
		// face that held it, plus the horizon around it
		stamp++;
		visible.clear();
		horizon.clear();
		find_horizon(&faces, points[farthest], seed, stamp, &horizon, &visible);

		if (!horizon_is_simple(&faces, &horizon, vertex_stamp, stamp))
		{
			// Leave the hull alone and come back for this face's other points
			pending.push(seed);
			continue;
		}

		// Remember where new faces start
		sz first_new = faces.len();

		// Expand hull to include this point
		build_cone(&faces, points, farthest, &horizon);

		// Retire the faces the new vertex saw and hand their outside points on
		foreach (vi : visible)
		{
			faces[vi].alive = false;
		}
		redistribute_points(&faces, points, first_new, &visible, hull_vertices, tolerance);

		for (sz f = first_new; f < faces.len(); f++)
		{
			if (faces[f].outside.len() > 0) pending.push(f);
		}

	}

	// Build result mesh from alive faces
	// First, collect unique vertices
	List {int} vertex_map; // original index -> new index
	defer vertex_map.free();

	// Initialize with -1
	for (sz i = 0; i < points.len; i++)
	{
		vertex_map.push(-1);
	}

	// Mark used vertices
	foreach (&f : faces)
	{
		if (!f.alive) continue;
		for (int j = 0; j < 3; j++)
		{
			sz vi = f.vertices[j];
			if (vertex_map[vi] < 0)
			{
				vertex_map[vi] = (int)result.vertices.len();
				result.vertices.push(points[vi]);
			}
		}
	}

	// Add triangles with remapped indices
	foreach (&f : faces)
	{
		if (!f.alive) continue;
		Triangle tri = { (uint)vertex_map[f.vertices[0]], (uint)vertex_map[f.vertices[1]], (uint)vertex_map[f.vertices[2]] };
		result.triangles.push(tri);
	}

	return result;
}

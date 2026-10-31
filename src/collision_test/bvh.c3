module collision;

import std::math;
import std::io;
import std::collections::list;

// ============================================================================
// Triangle BVH for concave mesh collision. One owner holds the flat node array
// and the triangle order; it points at the caller's vertex / triangle arrays.
// ============================================================================

const int MAX_TRIS_PER_LEAF = 8;
const float BVH_EPSILON = 0.00001;

// Depth of the explicit traversal stack. A median-split tree over 2^32
// triangles is ~29 levels deep and each level leaves at most one sibling behind.
const int BVH_STACK_DEPTH = 64;

// Ray directions below this are treated as parallel to a slab.
const float RAY_PARALLEL_EPS @local = 1e-6;

<*
 One node of a `TriBVH`, stored depth first: the left child of an inner node is the
 next node, the right child is at `first`.
 A leaf (`count > 0`) holds the triangles `order[first : count]`.
*>
struct BvhNode
{
	Aabb3 aabb;
	uint first;
	uint count; // 0 for an inner node
}

fn bool BvhNode.is_leaf(&self) => self.count > 0;

<*
 Bounding volume hierarchy over a triangle soup, owned as one unit: `free` releases
 the nodes, the triangle order and the tree itself. The vertex and triangle arrays
 belong to the caller (usually a `Mesh`) and must outlive and not move under the tree.
*>
struct TriBVH
{
	List {BvhNode} nodes; // nodes[0] is the root
	uint* order; // Triangle numbers, grouped by leaf
	Vec3* vertices;
	Triangle* triangles;
	sz tri_count;
	Allocator allocator;
}

fn void TriBVH.free(&self)
{
	Allocator allocator = self.allocator;
	self.nodes.free();
	alloc::free(allocator, self.order);
	alloc::free(allocator, self);
}

<*
 The `i`-th triangle of a leaf, as vertices.
*>
fn TriangleVerts TriBVH.leaf_tri(&self, BvhNode* leaf, uint i) @inline
{
	Triangle tri = self.triangles[self.order[leaf.first + i]];
	return { self.vertices[tri.x], self.vertices[tri.y], self.vertices[tri.z] };
}

<*
 Walk every leaf whose box overlaps `aabb`, left subtree before right.
 `@visit` sets `*stop` to end the walk early. One iterative walker shared by
 `box_intersects` and `@foreach_triangle`.
*>
macro void TriBVH.@walk_leaves(&self, Aabb3 aabb, bool* stop; @visit(BvhNode* leaf))
{
	BvhNode* nodes = self.nodes.entries;
	uint[BVH_STACK_DEPTH] node_stack;
	sz stack_size = 1;
	node_stack[0] = 0;

	while (stack_size > 0)
	{
		BvhNode* node = &nodes[node_stack[--stack_size]];
		if (!node.aabb.collides(aabb)) continue;

		if (node.is_leaf())
		{
			@visit(node);
			if (*stop) break;
			continue;
		}

		assert(stack_size + 2 <= node_stack.len);
		node_stack[stack_size++] = node.first;
		node_stack[stack_size++] = (uint)(node - nodes) + 1;
	}
}

<*
 First triangle overlapping `aabb`, with its penetration depth.
*>
fn HitInfo TriBVH.box_intersects(&self, Aabb3 aabb)
{
	HitInfo hit;
	bool stop = false;
	self.@walk_leaves(aabb, &stop; BvhNode* leaf)
	{
		for (uint i = 0; i < leaf.count; i++)
		{
			TriangleVerts triangle = self.leaf_tri(leaf, i);
			float overlap = triangle.collides_aabb(aabb);
			if (overlap > 0)
			{
				hit = { .triangle = triangle, .depth = overlap, .collided = true };
				stop = true;
				break;
			}
		}
	};
	return hit;
}

<*
 Visit every triangle overlapping the query AABB without allocating.
 Only yes/no overlap is needed here, so it uses the early-out SAT test.
*>
macro void TriBVH.@foreach_triangle(&self, Aabb3 aabb; @body(tri))
{
	bool never = false;
	self.@walk_leaves(aabb, &never; BvhNode* leaf)
	{
		for (uint i = 0; i < leaf.count; i++)
		{
			TriangleVerts triangle = self.leaf_tri(leaf, i);
			if (triangle.overlaps_aabb(aabb)) @body(triangle);
		}
	};
}

// ============================================================================
// Build
// ============================================================================

<*
 Per-triangle data the build needs, indexed by triangle number.
*>
struct BuildData @local
{
	Vec3* centroids;
	Aabb3* boxes;
	uint* order; // Permutation of triangle numbers; becomes `TriBVH.order`
	List {BvhNode}* nodes;
}

<*
 Put the element a full sort would leave at `nth` there, with everything before
 it no greater on `axis` and everything after it no less. Reorders
 `order[start : count]` in place and touches nothing outside that range.

 A node needs a median to split at, not an ordered range: each half is
 re-partitioned on its own axis by the recursion, so any ordering beyond the
 median is thrown away. (A full sort per node made building quadratic.)

 Three-way partitioning: equal centroids are common in tiled or mirrored meshes,
 and a two-way split of a range that is all one value makes no progress.

 @require count > 0 : `An empty range has no nth element.`
 @require nth >= start && nth < start + count : `The rank has to lie inside the range.`
*>
fn void select_nth(Vec3* centroids, uint* order, sz start, sz count, sz nth, sz axis) @local
{
	sz low = start;
	sz high = start + count - 1;

	while (low < high)
	{
		// Median of the three ends keeps already-sorted (scan-order) input off the quadratic path.
		float a = centroids[order[low]][axis];
		float b = centroids[order[low + (high - low) / 2]][axis];
		float c = centroids[order[high]][axis];
		float pivot = math::max(math::min(a, b), math::min(math::max(a, b), c));

		// Dutch-flag partition: below pivot, equal to it, above it.
		sz less = low;
		sz i = low;
		sz greater = high;
		while (i <= greater)
		{
			float value = centroids[order[i]][axis];
			if (value < pivot)
			{
				@swap(order[less], order[i]);
				less++;
				i++;
			}
			else if (value > pivot)
			{
				@swap(order[greater], order[i]);
				greater--;
			}
			else
			{
				i++;
			}
		}

		// The equal block is already in its final place, so a rank inside it is answered.
		if (nth < less)
		{
			high = less - 1;
		}
		else if (nth > greater)
		{
			low = greater + 1;
		}
		else
		{
			return;
		}
	}
}

<*
 Build a BVH over `triangles`. Does not copy vertex or triangle data; the tree keeps
 pointers into them. Free it with `TriBVH.free`.

 @require vertices != null : `Vertices must have values`
 @require triangles != null : `Triangles must have values`
 @require tri_count > 0 && tri_count <= uint::max : `Triangles must have values`
*>
fn TriBVH* create_tri_bvh(Vec3* vertices, Triangle* triangles, sz tri_count, Allocator allocator = mem)
{
	TriBVH* tree = alloc::new(allocator, TriBVH);
	tree.vertices = vertices;
	tree.triangles = triangles;
	tree.tri_count = tri_count;
	tree.allocator = allocator;
	tree.order = alloc::alloc_array(allocator, uint, tri_count);
	// Inner nodes split at least 9 triangles in two, so leaves hold 4 or more.
	tree.nodes.init(allocator, tri_count / 2 + 1);

	@pool()
	{
		Vec3[] centroids = mem::talloc_array(Vec3, tri_count);
		Aabb3[] boxes = mem::talloc_array(Aabb3, tri_count);
		for (sz i = 0; i < tri_count; i++)
		{
			Triangle idx = triangles[i];
			TriangleVerts tri = { vertices[idx.x], vertices[idx.y], vertices[idx.z] };
			centroids[i] = tri.centroid();
			boxes[i] = tri.aabb();
			tree.order[i] = (uint)i;
		}

		BuildData data = { centroids.ptr, boxes.ptr, tree.order, &tree.nodes };
		build_bvh_node(&data, 0, tri_count);
	};
	return tree;
}

<*
 Append the node for `order[start : count]` and its subtree; returns its index.
*>
fn uint build_bvh_node(BuildData* data, sz start, sz count) @local
{
	uint index = (uint)data.nodes.len();
	data.nodes.push({ .aabb = NEW_AABB3, .first = (uint)start, .count = (uint)count });

	// One pass for both the node box and the box of centroids (the split axis).
	Aabb3 box = NEW_AABB3;
	Aabb3 centroid_box = NEW_AABB3;
	for (sz i = start; i < start + count; i++)
	{
		uint tri = data.order[i];
		box.expand_by_aabb(data.boxes[tri]);
		centroid_box.expand_by_point(data.centroids[tri]);
	}
	(*data.nodes)[index].aabb = box;

	if (count <= MAX_TRIS_PER_LEAF) return index;

	sz mid = start + count / 2;
	select_nth(data.centroids, data.order, start, count, mid, centroid_box.longest_axis());

	build_bvh_node(data, start, mid - start);
	uint right = build_bvh_node(data, mid, start + count - mid);
	(*data.nodes)[index].first = right;
	(*data.nodes)[index].count = 0;
	return index;
}

// ============================================================================
// Ray query
// ============================================================================

<*
 Entry distance of the ray into `box`, or `float::max` when it misses or only
 enters beyond `max_t`. `inv_dir` is `1 / direction`, valid on non-parallel axes.
*>
fn float ray_box_entry(Aabb3 box, Vec3 origin, Vec3 direction, Vec3 inv_dir, float max_t) @inline @local
{
	float tmin = 0;
	float tmax = max_t;
	for (uint i = 0; i < 3; ++i)
	{
		if (math::abs(direction[i]) < RAY_PARALLEL_EPS)
		{
			if (origin[i] < box.min[i] || origin[i] > box.max[i]) return float::max;
			continue;
		}
		float t1 = (box.min[i] - origin[i]) * inv_dir[i];
		float t2 = (box.max[i] - origin[i]) * inv_dir[i];
		if (t1 > t2) @swap(t1, t2);
		tmin = math::max(tmin, t1);
		tmax = math::min(tmax, t2);
		if (tmin > tmax) return float::max;
	}
	return tmin;
}

<*
 Closest triangle hit along the ray: `.depth` is the ray parameter `t`.
 Iterative, nearer child first, and subtrees that cannot beat the best hit so
 far are skipped.
*>
fn HitInfo TriBVH.ray_intersects(&self, Ray ray)
{
	Vec3 inv_dir = 1.0f / ray.direction;
	float best_t = float::max;
	TriangleVerts best_tri;
	bool found = false;

	BvhNode* nodes = self.nodes.entries;
	uint[BVH_STACK_DEPTH] node_stack;
	float[BVH_STACK_DEPTH] entry_stack;

	float root_t = ray_box_entry(nodes[0].aabb, ray.origin, ray.direction, inv_dir, best_t);
	if (root_t == float::max) return { .collided = false };
	node_stack[0] = 0;
	entry_stack[0] = root_t;
	sz stack_size = 1;

	while (stack_size > 0)
	{
		--stack_size;
		BvhNode* node = &nodes[node_stack[stack_size]];
		if (entry_stack[stack_size] >= best_t) continue;

		if (node.is_leaf())
		{
			for (uint i = 0; i < node.count; i++)
			{
				TriangleVerts triangle = self.leaf_tri(node, i);
				float t;
				if (triangle.ray_t(ray, &t) && t < best_t)
				{
					best_t = t;
					best_tri = triangle;
					found = true;
				}
			}
			continue;
		}

		uint left = (uint)(node - nodes) + 1;
		uint right = node.first;
		float tl = ray_box_entry(nodes[left].aabb, ray.origin, ray.direction, inv_dir, best_t);
		float tr = ray_box_entry(nodes[right].aabb, ray.origin, ray.direction, inv_dir, best_t);

		assert(stack_size + 2 <= node_stack.len);
		// Push the farther child first so the nearer one pops first.
		if (tl <= tr)
		{
			if (tr != float::max) { node_stack[stack_size] = right; entry_stack[stack_size++] = tr; }
			if (tl != float::max) { node_stack[stack_size] = left; entry_stack[stack_size++] = tl; }
		}
		else
		{
			if (tl != float::max) { node_stack[stack_size] = left; entry_stack[stack_size++] = tl; }
			if (tr != float::max) { node_stack[stack_size] = right; entry_stack[stack_size++] = tr; }
		}
	}

	if (!found) return { .collided = false };
	return { .collided = true, .depth = best_t, .normal = best_tri.normal(), .triangle = best_tri };
}

// ============================================================================
// Triangle collision utilities
// ============================================================================

<*
 Moller-Trumbore. On a hit writes the ray parameter to `t`.
 @param [&out] t
*>
fn bool TriangleVerts.ray_t(&self, Ray ray, float* t) @inline
{
	Vec3 edge1 = (*self)[1] - (*self)[0];
	Vec3 edge2 = (*self)[2] - (*self)[0];

	Vec3 h = ray.direction.cross(edge2);
	float a = edge1.dot(h);
	if (a > -BVH_EPSILON && a < BVH_EPSILON) return false;

	float f = 1.0f / a;
	Vec3 s = ray.origin - (*self)[0];
	float u = f * s.dot(h);
	if (u < 0.0f || u > 1.0f) return false;

	Vec3 q = s.cross(edge1);
	float v = f * ray.direction.dot(q);
	if (v < 0.0f || u + v > 1.0f) return false;

	float hit_t = f * edge2.dot(q);
	if (hit_t < BVH_EPSILON) return false;

	*t = hit_t;
	return true;
}

fn HitInfo TriangleVerts.intersect_ray(&self, Ray ray)
{
	float t;
	if (!self.ray_t(ray, &t)) return { .collided = false };
	return { .collided = true, .depth = t, .normal = self.normal() };
}

<*
 Test one separating axis for triangle `t` (box-centred) against a box of half
 size `extent`. Evaluates to true when the axis separates them; otherwise folds
 the overlap into `*depth`.
*>
macro bool @sat_axis(Vec3[3] t, Vec3 extent, Vec3 axis, float* depth, bool need_depth)
{
	float d0 = axis.dot(t[0]);
	float d1 = axis.dot(t[1]);
	float d2 = axis.dot(t[2]);
	MinMax triangle_mm = { math::min(d0, math::min(d1, d2)), math::max(d0, math::max(d1, d2)) };
	MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, axis);
	float overlap = triangle_mm.overlap(box_mm);
	if (overlap == -1) return true;
	if (need_depth && overlap != 0) *depth = math::min(overlap, *depth);
	return false;
}

<*
 SAT triangle-vs-box. Returns -1 when separated, otherwise the smallest overlap
 over the axes tested (`float::max` if every overlap was exactly zero).
 `need_depth = false` skips the depth bookkeeping for yes/no callers.
*>
fn float TriangleVerts.sat_aabb(&self, Aabb3 box, bool need_depth)
{
	Vec3 extent = box.extent();
	Vec3 center = box.center();

	Vec3[3] t = { (*self)[0] - center, (*self)[1] - center, (*self)[2] - center };
	Vec3[3] edges = { t[1] - t[0], t[2] - t[1], t[0] - t[2] };
	Vec3[3] axes = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

	float depth = float::max;

	foreach (axis : axes)
	{
		if (@sat_axis(t, extent, axis, &depth, need_depth)) return -1;
	}

	if (@sat_axis(t, extent, edges[0].cross(edges[1]), &depth, need_depth)) return -1;

	foreach (axis : axes)
	{
		foreach (edge : edges)
		{
			Vec3 cross_axis = axis.cross(edge);
			if (cross_axis == (Vec3){ 0, 0, 0 }) continue;
			if (@sat_axis(t, extent, cross_axis, &depth, need_depth)) return -1;
		}
	}

	return depth;
}

<*
 Overlap depth of the triangle with a box, or -1 when they are separated.
*>
fn float TriangleVerts.collides_aabb(&self, Aabb3 box) => self.sat_aabb(box, true);

<*
 Whether the triangle overlaps the box at all (no depth computed).
*>
fn bool TriangleVerts.overlaps_aabb(&self, Aabb3 box) => self.sat_aabb(box, false) != -1;

// ============================================================================
// Geometric proximity utilities for specialized collision tests
// ============================================================================

// Closest point on line segment to a given point
fn Vec3 closest_point_on_segment(Vec3 p, Vec3 a, Vec3 b)
{
	Vec3 ab = b - a;
	float ab_sq = ab.dot(ab);
	if (ab_sq < BVH_EPSILON) return a;
	float t = (p - a).dot(ab) / ab_sq;
	t = math::clamp(t, 0.0f, 1.0f);
	return a + ab * t;
}

// What capsule_push_point found. `hit` false leaves the rest zeroed.
struct CapsuleHit
{
	bool hit;
	Vec3 normal; // Unit direction out of the capsule, from the axis toward p
	float depth; // How far inside the surface p is, along that normal
	Vec3 point; // Where on the (expanded) surface p belongs
}

<*
 Push a point out of a capsule given in world space: the segment a→b swept by
 `radius`, grown by `p_radius` so a point that is itself a sphere is separated by
 its own radius too.

 The world-space counterpart to the mesh tests above — a triangle-less body (a
 particle chain, a point cloud) has nothing for a convex test to work with, but
 every particle can still be asked this. No capsule *shape* is involved and no
 transform is applied: the two endpoints are already wherever the caller posed
 them, which is what lets a host pose bones however it likes and still ask.

 `point` is the position that resolves the overlap, so a caller wanting a hard
 separation assigns it and a caller wanting a soft one moves toward it.

 A point exactly on the axis has no direction to be pushed in — every one is
 equally correct — so rather than normalizing a zero vector into NaNs, one
 perpendicular to the axis is chosen, picked against whichever axis the segment
 leans on least so the cross product cannot come out near zero. A degenerate
 segment (a sphere) falls back to +y.

 @param p_radius : `Radius of the point itself; 0 treats it as a point.`
*>
fn CapsuleHit capsule_push_point(Vec3 p, Vec3 a, Vec3 b, float radius, float p_radius = 0)
{
	CapsuleHit hit;

	float total = radius + p_radius;
	if (total <= 0) return hit;

	Vec3 c = closest_point_on_segment(p, a, b);
	Vec3 diff = p - c;
	float dist_sq = diff.dot(diff);
	if (dist_sq >= total * total) return hit;

	float dist = math::sqrt(dist_sq);
	Vec3 normal;

	if (dist < BVH_EPSILON)
	{
		Vec3 axis = b - a;
		float axis_sq = axis.dot(axis);
		if (axis_sq < BVH_EPSILON)
		{
			normal = { 0, 1, 0 };
		}
		else
		{
			Vec3 dir = axis / math::sqrt(axis_sq);
			Vec3 aside = math::abs(dir.x) < math::abs(dir.z) ? (Vec3){ 1, 0, 0 } : (Vec3){ 0, 0, 1 };
			normal = dir.cross(aside).normalize();
		}
		dist = 0;
	}
	else
	{
		normal = diff / dist;
	}

	hit.hit = true;
	hit.normal = normal;
	hit.depth = total - dist;
	hit.point = c + normal * total;
	return hit;
}

// Closest points between two line segments (Ericson's method)
fn void closest_points_segments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2, Vec3* c1, Vec3* c2)
{
	Vec3 d1 = q1 - p1;
	Vec3 d2 = q2 - p2;
	Vec3 r = p1 - p2;

	float a = d1.dot(d1);
	float e = d2.dot(d2);
	float f = d2.dot(r);

	float s, t;

	if (a <= BVH_EPSILON && e <= BVH_EPSILON)
	{
		*c1 = p1;
		*c2 = p2;
		return;
	}

	if (a <= BVH_EPSILON)
	{
		s = 0.0f;
		t = math::clamp(f / e, 0.0f, 1.0f);
	}
	else
	{
		float c = d1.dot(r);
		if (e <= BVH_EPSILON)
		{
			t = 0.0f;
			s = math::clamp(-c / a, 0.0f, 1.0f);
		}
		else
		{
			float b = d1.dot(d2);
			float denom = a * e - b * b;

			if (denom > BVH_EPSILON)
			{
				s = math::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
			}
			else
			{
				s = 0.0f;
			}

			t = (b * s + f) / e;

			if (t < 0.0f)
			{
				t = 0.0f;
				s = math::clamp(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f)
			{
				t = 1.0f;
				s = math::clamp((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	*c1 = p1 + d1 * s;
	*c2 = p2 + d2 * t;
}

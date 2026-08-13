module collision;

import std::math;
import std::io;
import std::collections::list;

const int MAX_PARTS_PER_LEAF = 4;


// ============================================================================
// Triangle BVH for concave mesh collision (original implementation, restored)
// ============================================================================

const int MAX_TRIS_PER_LEAF = 8;
const float BVH_EPSILON @local = 0.00001;

struct TriangleInfo @local
{
	sz v0, v1, v2;
	Vec3 centroid;
	Aabb3 aabb;
}

// BVH Node for triangle mesh collision
struct TriBVHNode
{
	Aabb3 aabb;
	TriBVHNode* left;
	TriBVHNode* right;

	// For indexed mode (memory efficient)
	Vec3* vertices;
	Triangle* triangles;
	sz* sorted_indices; // Sorted triangle indices for this subtree

	sz start; // Start index in sorted_indices
	sz count; // Number of triangles (leaf only)
	bool indexed; // True if using indexed mode
}

fn bool TriBVHNode.is_leaf(&self) => self.left == null && self.right == null;

fn void TriBVHNode.free(&self)
{
	if (self.left != null)
	{
		self.left.free();
		mem::free(self.left);
	}

	if (self.right != null)
	{
		self.right.free();
		mem::free(self.right);
	}
}

// Free indexed BVH including the sorted_indices array (called on root only)
fn void TriBVHNode.free_indexed(&self)
{
	if (self.indexed && self.sorted_indices != null)
	{
		mem::free(self.sorted_indices);
	}
	self.free();
}

<*
 One triangle's centroid on a split axis. Axes past 2 read as Z, which is what
 `longest_axis` can never return but a caller passing a raw number might.
*>
fn float centroid_on(TriangleInfo info, sz axis) @local
{
	switch (axis)
	{
		case 0: return info.centroid.x;
		case 1: return info.centroid.y;
		default: return info.centroid.z;
	}
}

<*
 Put the element a full sort would leave at `nth` there, with everything before
 it no greater on `axis` and everything after it no less. Reorders
 `order[start : count]` in place and touches nothing outside that range.

 A node needs a median to split at, not an ordered range: each half is
 re-partitioned on its own axis by the recursion below, so any ordering imposed
 here beyond the median is work the next level immediately throws away. Sorting
 instead is what made building quadratic — every node sorted its whole range, so
 the root alone cost O(n^2) and a 48k-triangle mesh spent 1.5 s in here.

 Three-way partitioning, not two. Equal centroids are ordinary in real meshes —
 anything mirrored, tiled or with duplicated geometry has them in quantity — and
 a two-way split of a range that is all one value makes no progress at all.

 @require count > 0 : `An empty range has no nth element.`
 @require nth >= start && nth < start + count : `The rank has to lie inside the range.`
*>
fn void select_nth(TriangleInfo* info, sz* order, sz start, sz count, sz nth, sz axis) @local
{
	sz low = start;
	sz high = start + count - 1;

	while (low < high)
	{
		// Median of the three ends, which keeps input that is already sorted or
		// reversed off the quadratic path. A mesh emitted in scan order is both,
		// depending on the axis.
		float a = centroid_on(info[order[low]], axis);
		float b = centroid_on(info[order[low + (high - low) / 2]], axis);
		float c = centroid_on(info[order[high]], axis);
		float pivot = math::max(math::min(a, b), math::min(math::max(a, b), c));

		// Dutch-flag partition: below pivot, equal to it, above it.
		sz less = low;
		sz i = low;
		sz greater = high;
		while (i <= greater)
		{
			float value = centroid_on(info[order[i]], axis);
			if (value < pivot)
			{
				sz swap = order[less]; order[less] = order[i]; order[i] = swap;
				less++;
				i++;
			}
			else if (value > pivot)
			{
				sz swap = order[greater]; order[greater] = order[i]; order[i] = swap;
				greater--;
			}
			else
			{
				i++;
			}
		}

		// The equal block is already in its final place, so a rank landing inside
		// it is answered — that is the case a two-way partition cannot close.
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

fn int compare_triangles(TriangleInfo a, TriangleInfo b, sz* split_axis) @local
{
	float val_a, val_b;

	if (*split_axis == 0)
	{
		val_a = a.centroid.x;
		val_b = b.centroid.x;
	}
	else if (*split_axis == 1)
	{
		val_a = a.centroid.y;
		val_b = b.centroid.y;
	}
	else
	{
		val_a = a.centroid.z;
		val_b = b.centroid.z;
	}

	if (val_a < val_b) return -1;
	if (val_a > val_b) return 1;
	return 0;
}


// Query triangle BVH with AABB, returns hit info with first colliding triangle
fn HitInfo TriBVHNode.box_intersects(&self, Aabb3 aabb)
{
	HitInfo hit;

	if (!self.aabb.collides(aabb))
	{
		return hit;
	}

	if (self.is_leaf())
	{
		for (sz i = 0; i < self.count; i++)
		{
			Vec3 v0, v1, v2;

			if (self.indexed)
			{
				// Indexed mode: look up triangle indices, then vertex positions
				sz tri_idx = self.sorted_indices[self.start + i];
				Triangle tri = self.triangles[tri_idx];
				v0 = self.vertices[tri.x];
				v1 = self.vertices[tri.y];
				v2 = self.vertices[tri.z];
			}

			TriangleVerts triangle = { v0, v1, v2 };

			float overlap = triangle.collides_aabb(aabb);

			if (overlap > 0)
			{
				return { .triangle = triangle, .depth = overlap, .collided = true, };
			}
		}
		return hit;
	}

	if (self.left != null)
	{
		HitInfo left_hit = self.left.box_intersects(aabb);
		if (left_hit.collided) return left_hit;
	}

	if (self.right != null)
	{
		HitInfo right_hit = self.right.box_intersects(aabb);
		if (right_hit.collided) return right_hit;
	}

	return hit;
}

// Visit every triangle overlapping the query AABB without allocating.
// Iterative traversal with an explicit stack; @body receives each TriangleVerts.
macro void TriBVHNode.@foreach_triangle(&self, Aabb3 aabb; @body(tri))
{
	TriBVHNode*[64] node_stack;
	sz stack_size = 1;
	node_stack[0] = self;

	while (stack_size > 0)
	{
		TriBVHNode* node = node_stack[--stack_size];
		if (!node.aabb.collides(aabb)) continue;

		if (node.is_leaf())
		{
			for (sz i = 0; i < node.count; i++)
			{
				sz tri_idx = node.sorted_indices[node.start + i];
				Triangle tri = node.triangles[tri_idx];
				TriangleVerts triangle = { node.vertices[tri.x], node.vertices[tri.y], node.vertices[tri.z] };

				if (triangle.collides_aabb(aabb) > 0)
				{
					@body(triangle);
				}
			}
			continue;
		}

		assert (stack_size + 2 <= node_stack.len);
		if (node.left != null) node_stack[stack_size++] = node.left;
		if (node.right != null) node_stack[stack_size++] = node.right;
	}
}

// Query all overlapping triangles (for multiple contact points)
fn void TriBVHNode.query_all_triangles(&self, Aabb3 aabb, List {TriangleVerts}* results)
{
	self.@foreach_triangle(aabb; TriangleVerts tri)
	{
		results.push(tri);
	};
}

// ============================================================================
// Indexed Triangle BVH (memory efficient - uses mesh vertices directly)
// ============================================================================

// Create indexed BVH tree from mesh vertices and triangles
// Does NOT copy vertex data - uses pointers to existing mesh data

<*
 @require vertices != null : `Vertices must have values`
 @require triangles != null : `Triangles must have values`
 @require tri_count > 0 : `Triangles must have values`
*>
fn TriBVHNode* create_indexed_tri_bvh(Vec3* vertices, Triangle* triangles, sz tri_count)
{
	// Build triangle info for sorting
	TriangleInfo* tri_info = mem::new_array(TriangleInfo, tri_count);
	sz* sorted_indices = mem::new_array(sz, tri_count);

	for (sz i = 0; i < tri_count; i++)
	{
		Triangle tri = triangles[i];
		Vec3 v0 = vertices[tri.x];
		Vec3 v1 = vertices[tri.y];
		Vec3 v2 = vertices[tri.z];

		tri_info[i] = {
			.v0 = i, // Store triangle index, not vertex index
			.v1 = 0,
			.v2 = 0,
			.centroid = (v0 + v1 + v2) / 3.0f,
			.aabb = NEW_AABB3
		};

		tri_info[i].aabb.expand_by_point(v0);
		tri_info[i].aabb.expand_by_point(v1);
		tri_info[i].aabb.expand_by_point(v2);

		sorted_indices[i] = i;
	}

	TriBVHNode* tree = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, 0, tri_count);

	mem::free(tri_info);

	// Propagate shared data to all nodes
	set_indexed_bvh_data(tree, vertices, triangles, sorted_indices);

	return tree;
}

fn void set_indexed_bvh_data(TriBVHNode* node, Vec3* vertices, Triangle* triangles, sz* sorted_indices) @local
{
	node.vertices = vertices;
	node.triangles = triangles;
	node.sorted_indices = sorted_indices;
	node.indexed = true;
	if (node.left != null) set_indexed_bvh_data(node.left, vertices, triangles, sorted_indices);
	if (node.right != null) set_indexed_bvh_data(node.right, vertices, triangles, sorted_indices);
}

fn TriBVHNode* build_indexed_bvh_recursive(
	TriangleInfo* tri_info,
	sz* sorted_indices,
	Vec3* vertices,
	Triangle* triangles,
	sz start,
	sz count
) @local
{
	TriBVHNode* node = mem::new(TriBVHNode, { .start = start, .count = count, .aabb = NEW_AABB3, .indexed = true });

	for (sz i = start; i < start + count; i++)
	{
		node.aabb.expand_by_aabb(tri_info[sorted_indices[i]].aabb);
	}

	if (count <= MAX_TRIS_PER_LEAF)
	{
		return node;
	}

	Aabb3 centroid_aabb = NEW_AABB3;
	for (sz i = start; i < start + count; i++)
	{
		centroid_aabb.expand_by_point(tri_info[sorted_indices[i]].centroid);
	}

	sz split_axis = centroid_aabb.longest_axis();

	// Partition about the median centroid on that axis. Only the median matters:
	// the two halves below are each split on their own axis, so ordering them
	// here would be discarded immediately — see `select_nth`.
	sz mid = start + count / 2;
	select_nth(tri_info, sorted_indices, start, count, mid, split_axis);

	node.left = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, start, mid - start);
	node.right = build_indexed_bvh_recursive(tri_info, sorted_indices, vertices, triangles, mid, start + count - mid);
	node.count = 0;

	return node;
}

// Ray intersection with triangle BVH
fn HitInfo TriBVHNode.ray_intersects(&self, Ray ray)
{
	HitInfo hit_info = { .collided = false };

	if (self.aabb.intersect_ray(ray).collided == false)
	{
		return hit_info;
	}

	if (self.is_leaf())
	{
		float closest_t = float::max;

		for (sz i = 0; i < self.count; i++)
		{
			Vec3 v0, v1, v2;

			if (self.indexed)
			{
				// Indexed mode: look up triangle indices, then vertex positions
				sz tri_idx = self.sorted_indices[self.start + i];
				Triangle tri = self.triangles[tri_idx];
				v0 = self.vertices[tri.x];
				v1 = self.vertices[tri.y];
				v2 = self.vertices[tri.z];
			}

			TriangleVerts triangle = { v0, v1, v2 };
			HitInfo hit = triangle.intersect_ray(ray);

			if (hit.collided && hit.depth < closest_t)
			{
				closest_t = hit.depth;
				hit_info = hit;
				hit_info.triangle = triangle;
			}
		}

		return hit_info;
	}

	HitInfo left_hit = { .collided = false };
	HitInfo right_hit = { .collided = false };

	if (self.left != null)
	{
		left_hit = self.left.ray_intersects(ray);
	}
	if (self.right != null)
	{
		right_hit = self.right.ray_intersects(ray);
	}

	if (left_hit.collided && right_hit.collided)
	{
		return left_hit.depth < right_hit.depth ? left_hit : right_hit;
	}
	if (left_hit.collided) return left_hit;
	if (right_hit.collided) return right_hit;

	return hit_info;
}

// ============================================================================
// Triangle collision utilities
// ============================================================================

alias TriangleVerts = Vec3[3];

fn HitInfo TriangleVerts.intersect_ray(&self, Ray ray)
{
	HitInfo hit_info = { .collided = false };

	Vec3 edge1 = (*self)[1] - (*self)[0];
	Vec3 edge2 = (*self)[2] - (*self)[0];

	Vec3 h = ray.direction.cross(edge2);
	float a = edge1.dot(h);

	if (a > -BVH_EPSILON && a < BVH_EPSILON) return hit_info;

	float f = 1.0f / a;
	Vec3 s = ray.origin - (*self)[0];
	float u = f * s.dot(h);

	if (u < 0.0f || u > 1.0f) return hit_info;

	Vec3 q = s.cross(edge1);
	float v = f * ray.direction.dot(q);

	if (v < 0.0f || u + v > 1.0f) return hit_info;

	float t = f * edge2.dot(q);

	if (t < BVH_EPSILON) return hit_info;

	return { .collided = true, .depth = t, .normal = edge1.cross(edge2).normalize() };
}

fn float TriangleVerts.collides_aabb(&self, Aabb3 box)
{
	Vec3 center = box.center();
	Vec3 extent = box.extent();

	Vec3 v0 = (*self)[0] - center;
	Vec3 v1 = (*self)[1] - center;
	Vec3 v2 = (*self)[2] - center;

	Vec3 f0 = v1 - v0;
	Vec3 f1 = v2 - v1;
	Vec3 f2 = v0 - v2;

	Vec3[] t = { v0, v1, v2 };

	Vec3[] axes = { { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };

	float result_depth = float::max;

	// Test box axes
	foreach (axis : axes)
	{
		MinMax triangle_mm = min_max_from_axis(axis, t);
		MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, axis);
		float depth = triangle_mm.overlap(box_mm);
		if (depth == -1) return -1;
		if (depth != 0) result_depth = math::min(depth, result_depth);
	}

	// Test triangle normal
	Vec3 normal = f0.cross(f1);
	{
		MinMax triangle_mm = min_max_from_axis(normal, t);
		MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, normal);
		float depth = triangle_mm.overlap(box_mm);
		if (depth == -1) return -1;
		if (depth != 0) result_depth = math::min(depth, result_depth);
	}

	// Test cross products of edges
	Vec3[] edges = { f0, f1, f2 };

	foreach (axis : axes)
	{
		foreach (edge : edges)
		{
			Vec3 new_axis = axis.cross(edge);

			if (new_axis == (Vec3) { 0, 0, 0 }) continue;

			MinMax triangle_mm = min_max_from_axis(new_axis, t);
			MinMax box_mm = min_max_from_box({ 0, 0, 0 }, extent, new_axis);
			float depth = triangle_mm.overlap(box_mm);
			if (depth == -1) return -1;
			if (depth != 0) result_depth = math::min(depth, result_depth);
		}
	}

	return result_depth;
}

fn Vec3 TriangleVerts.get_normal(&self)
{
	Vec3 edge1 = (*self)[1] - (*self)[0];
	Vec3 edge2 = (*self)[2] - (*self)[0];
	return edge1.cross(edge2).normalize();
}

// ============================================================================
// Geometric proximity utilities for specialized collision tests
// ============================================================================

// Closest point on triangle to a given point (Ericson's Real-Time Collision Detection)
fn Vec3 closest_point_on_triangle(Vec3 p, Vec3 a, Vec3 b, Vec3 c)
{
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

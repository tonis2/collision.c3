module collision;
import std::math;

<*
 A convex collider as the distance query sees it: a core with a radius around
 it. A sphere's core is its centre and a capsule's is its segment, so GJK runs
 on a point or a line, which it answers in a step or two, and the radius comes
 off the distance afterwards: an overlap shallower than the radius needs no
 EPA. Anything else is its own core with no radius, and so is a sphere or a
 capsule scaled unevenly, which is no longer round.
*>
struct CoredShape
{
	PreparedShape* prepared;
	Vec3[2] core; // World ends of the segment, the same point twice for a sphere
	float radius;
	bool cored;
	// Added to every support point: the shape moved without being re-prepared.
	Vec3 shift;
}

fn CoredShape PreparedShape.cored(&self)
{
	CoredShape out = { .prepared = self };
	Vec3 scale = math::abs(self.shape.scale);
	if (scale.x != scale.y || scale.y != scale.z) return out;
	switch (self.kind)
	{
		case SPHERE:
			Sphere* sphere = (Sphere*)self.shape.shape;
			Vec3 centre = self.to_world(sphere.center);
			out.core = { centre, centre };
			out.radius = sphere.radius * scale.x;
			out.cored = true;
		case CAPSULE:
			Capsule* capsule = (Capsule*)self.shape.shape;
			if (capsule.radius_top != capsule.radius_bottom) return out;
			float half = capsule.height * 0.5f;
			out.core = { self.to_world({ 0, half, 0 }), self.to_world({ 0, -half, 0 }) };
			out.radius = capsule.radius_top * scale.x;
			out.cored = true;
		case BOX:
		case GENERIC:
	}
	return out;
}

fn Vec3 CoredShape.support(&self, Vec3 direction) @inline
{
	if (!self.cored) return self.prepared.support(direction) + self.shift;
	return (self.core[0].dot(direction) >= self.core[1].dot(direction) ? self.core[0] : self.core[1]) + self.shift;
}

<*
 How far apart two cores are, and the closest point on each.
*>
struct GjkDistance
{
	// The cores overlap, or touch closer than GJK can tell apart: how deep is
	// for EPA to say.
	bool overlap;
	// Between the cores. Past the `beyond` the query was given, only a lower
	// bound: it stopped as soon as it knew.
	float distance;
	Vec3 on_a;
	Vec3 on_b;
}

// Up to four points of the Minkowski difference, and the weights that make the
// point of their hull nearest the origin.
struct GjkSimplex @local
{
	SupportPoint[4] at;
	float[4] weight;
	int count;
}

fn void GjkSimplex.keep1(&self, int i)
{
	self.at[0] = self.at[i];
	self.weight[0] = 1;
	self.count = 1;
}

fn void GjkSimplex.keep2(&self, int i, int j, float wi, float wj)
{
	SupportPoint a = self.at[i];
	SupportPoint b = self.at[j];
	self.at[0] = a;
	self.at[1] = b;
	self.weight[0] = wi;
	self.weight[1] = wj;
	self.count = 2;
}

fn void GjkSimplex.keep3(&self, int i, int j, int k, float wi, float wj, float wk)
{
	SupportPoint a = self.at[i];
	SupportPoint b = self.at[j];
	SupportPoint c = self.at[k];
	self.at[0] = a;
	self.at[1] = b;
	self.at[2] = c;
	self.weight[0] = wi;
	self.weight[1] = wj;
	self.weight[2] = wk;
	self.count = 3;
}

// The point of segment i-j nearest the origin; the simplex keeps what holds it.
fn Vec3 GjkSimplex.segment(&self, int i, int j)
{
	Vec3 a = self.at[i].point;
	Vec3 b = self.at[j].point;
	Vec3 ab = b - a;
	float length_sq = ab.dot(ab);
	float t = length_sq > 0 ? -a.dot(ab) / length_sq : 0;
	if (t <= 0)
	{
		self.keep1(i);
		return a;
	}
	if (t >= 1)
	{
		self.keep1(j);
		return b;
	}
	self.keep2(i, j, 1 - t, t);
	return a + ab * t;
}

// The point of triangle i-j-k nearest the origin, by its Voronoi regions
// (Ericson, Real-Time Collision Detection, 5.1.5).
fn Vec3 GjkSimplex.triangle(&self, int i, int j, int k)
{
	Vec3 a = self.at[i].point;
	Vec3 b = self.at[j].point;
	Vec3 c = self.at[k].point;
	Vec3 ab = b - a;
	Vec3 ac = c - a;

	float d1 = -ab.dot(a);
	float d2 = -ac.dot(a);
	if (d1 <= 0 && d2 <= 0)
	{
		self.keep1(i);
		return a;
	}
	float d3 = -ab.dot(b);
	float d4 = -ac.dot(b);
	if (d3 >= 0 && d4 <= d3)
	{
		self.keep1(j);
		return b;
	}
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0)
	{
		float t = d1 / (d1 - d3);
		self.keep2(i, j, 1 - t, t);
		return a + ab * t;
	}
	float d5 = -ab.dot(c);
	float d6 = -ac.dot(c);
	if (d6 >= 0 && d5 <= d6)
	{
		self.keep1(k);
		return c;
	}
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0)
	{
		float t = d2 / (d2 - d6);
		self.keep2(i, k, 1 - t, t);
		return a + ac * t;
	}
	float va = d3 * d6 - d5 * d4;
	if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0)
	{
		float t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		self.keep2(j, k, 1 - t, t);
		return b + (c - b) * t;
	}
	float sum = va + vb + vc;
	// A triangle with no area: the regions above have all failed by rounding.
	if (!(sum > 0)) return self.segment(i, j);
	float v = vb / sum;
	float w = vc / sum;
	self.keep3(i, j, k, 1 - v - w, v, w);
	return a + ab * v + ac * w;
}

<*
 The point of the tetrahedron nearest the origin: the nearest of its faces the
 origin is outside of. Inside all four, `inside` is set and the simplex kept.
*>
fn Vec3 GjkSimplex.tetrahedron(&self, bool* inside)
{
	int[3][4] faces = { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 } };
	int[4] opposite = { 3, 1, 2, 0 };

	float best = float::max;
	Vec3 best_point;
	GjkSimplex kept = *self;
	bool outside_any = false;
	for (int f = 0; f < 4; f++)
	{
		Vec3 x0 = self.at[faces[f][0]].point;
		Vec3 n = cross3(self.at[faces[f][1]].point - x0, self.at[faces[f][2]].point - x0);
		Vec3 to_opposite = self.at[opposite[f]].point - x0;
		float origin_side = -x0.dot(n);
		float opposite_side = to_opposite.dot(n);
		// A flat tetrahedron has no inside, so each of its faces is tried.
		bool flat = opposite_side * opposite_side <= TETRAHEDRON_FLAT * n.dot(n) * to_opposite.dot(to_opposite);
		if (!flat && origin_side * opposite_side >= 0) continue;

		outside_any = true;
		GjkSimplex trial = *self;
		Vec3 point = trial.triangle(faces[f][0], faces[f][1], faces[f][2]);
		float distance_sq = point.dot(point);
		if (distance_sq < best)
		{
			best = distance_sq;
			best_point = point;
			kept = trial;
		}
	}
	*inside = !outside_any;
	*self = kept;
	return best_point;
}

fn SupportPoint support_pair(CoredShape* a, CoredShape* b, Vec3 direction) @local @inline
{
	Vec3 on_a = a.support(direction);
	Vec3 on_b = b.support(-direction);
	return { .point = on_a - on_b, .a = on_a, .b = on_b };
}

<*
 GJK as a distance query: the closest points of two convex cores, or that they
 overlap. Exact for polyhedra and points, and converging for round shapes.

 @param [&in] a : "The first core"
 @param [&in] b : "The second core"
 @param start : "A first search direction, from a towards b"
 @param beyond : "A distance past which the caller does not care: the search stops once it is known"
*>
fn GjkDistance gjk_distance(CoredShape* a, CoredShape* b, Vec3 start, float beyond)
{
	GjkSimplex simplex;
	Vec3 direction = start.length_sq() > 1e-20f ? start : (Vec3){ 1, 0, 0 };
	simplex.at[0] = support_pair(a, b, direction);
	simplex.weight[0] = 1;
	simplex.count = 1;
	Vec3 v = simplex.at[0].point;
	float scale_sq = v.dot(v);

	for (int iteration = 0; iteration < GJK_DISTANCE_ITERATIONS; iteration++)
	{
		float vv = v.dot(v);
		if (vv <= GJK_OVERLAP * scale_sq) return { .overlap = true };

		SupportPoint w = support_pair(a, b, -v);
		float vw = v.dot(w.point);
		scale_sq = math::max(scale_sq, w.point.dot(w.point));
		// v·w / |v| is a lower bound on the distance: past `beyond`, done.
		if (vw > 0 && vw * vw > beyond * beyond * vv) return { .distance = vw / math::sqrt(vv) };
		if (vv - vw <= GJK_DISTANCE_TOLERANCE * vv) break;

		bool repeated = false;
		for (int k = 0; k < simplex.count; k++)
		{
			if (simplex.at[k].point.distance_sq(w.point) <= GJK_OVERLAP * scale_sq) repeated = true;
		}
		if (repeated) break;

		simplex.at[simplex.count++] = w;
		switch (simplex.count)
		{
			case 2:
				v = simplex.segment(0, 1);
			case 3:
				v = simplex.triangle(0, 1, 2);
			default:
				bool inside;
				v = simplex.tetrahedron(&inside);
				if (inside) return { .overlap = true };
		}
	}

	if (v.dot(v) <= GJK_OVERLAP * scale_sq) return { .overlap = true };
	GjkDistance out = { .distance = v.length() };
	for (int k = 0; k < simplex.count; k++)
	{
		out.on_a += simplex.at[k].a * simplex.weight[k];
		out.on_b += simplex.at[k].b * simplex.weight[k];
	}
	return out;
}

<*
 The contact between two convex colliders up to `margin` apart, in EPA's form:
 the normal from `a` towards `b`, a point on each, and
 `contact[0] - contact[1] = normal * depth`, the depth negative by a gap.

 A pair that is apart is GJK's closest points, never EPA on shapes grown by the
 margin: growing rounds every edge, and EPA on a curved surface runs to its
 iteration limit. A sphere or a capsule is asked for its distance first, so an
 overlap shallower than its radius is that too. Only overlaps go on to EPA,
 after the plain GJK, which finds them faster than the distance query does.

 @param [&in] a : "A convex collider"
 @param [&in] b : "A convex collider"
 @param margin : "How far apart the two may be and still touch"
*>
fn CollisionInfo convex_contact(TransformedShape* a, TransformedShape* b, float margin)
{
	PreparedShape prep_a = a.prepare();
	PreparedShape prep_b = b.prepare();
	CoredShape core_a = prep_a.cored();
	CoredShape core_b = prep_b.cored();
	float radii = core_a.radius + core_b.radius;

	bool round = core_a.cored || core_b.cored;
	if (!round)
	{
		CollisionInfo info = check_convex_collision(&prep_a, &prep_b, epa: true);
		if (info.collided || margin <= 0) return info;
	}

	GjkDistance gap = gjk_distance(&core_a, &core_b, b.translation - a.translation, radii + margin);
	if (!gap.overlap)
	{
		if (gap.distance > radii + margin) return { };
		Vec3 normal = (gap.on_b - gap.on_a) / gap.distance;
		CollisionInfo info = { .collided = true, .normal = normal, .depth = radii - gap.distance };
		info.contact_points = { gap.on_a + normal * core_a.radius, gap.on_b - normal * core_b.radius };
		return info;
	}

	if (round)
	{
		CollisionInfo info = check_convex_collision(&prep_a, &prep_b, epa: true);
		if (info.collided || margin <= 0) return info;
	}

	// Touching, to within what either test can tell, so neither knows which
	// way is out. Moved a hair apart along the line between them, they are
	// apart, and their closest points say.
	Vec3 between = b.translation - a.translation;
	if (between.length_sq() > 0)
	{
		core_b.shift = between.normalize() * TOUCH_SHIFT;
		GjkDistance moved = gjk_distance(&core_a, &core_b, between, radii + margin + TOUCH_SHIFT);
		if (!moved.overlap && moved.distance > 0)
		{
			Vec3 normal = (moved.on_b - moved.on_a) / moved.distance;
			Vec3 on_b = moved.on_b - core_b.shift;
			CollisionInfo info = { .collided = true, .normal = normal, .depth = radii - (on_b - moved.on_a).dot(normal) };
			info.contact_points = { moved.on_a + normal * core_a.radius, on_b - normal * core_b.radius };
			return info;
		}
	}

	// Not even that: grown by the margin they overlap, and EPA decides.
	GrownShape grown = { .inner = &prep_a, .by = margin };
	CollisionInfo info = check_convex_collision(&grown, &prep_b, epa: true);
	if (!info.collided) return { };
	info.depth -= margin;
	info.contact_points[0] -= info.normal * margin;
	return info;
}

const int GJK_DISTANCE_ITERATIONS @local = 32;
// How far a touching pair is moved apart to tell which way is out, in world units.
const float TOUCH_SHIFT @local = 1e-4f;
// Converged once the next support point brings the distance no closer than this, relatively.
const float GJK_DISTANCE_TOLERANCE @local = 1e-5f;
// A squared distance this small against the shapes' own size is touching: overlap.
const float GJK_OVERLAP @local = 1e-10f;
// How close to flat, as a squared sine, a tetrahedron may be and still have an inside.
const float TETRAHEDRON_FLAT @local = 1e-10f;

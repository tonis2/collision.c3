module collision;
import std::math;

<*
 A box as the separating-axis test sees it: its centre, its three unit axes in
 the world, and its half size along each.
*>
struct OrientedBox @local
{
	Vec3 centre;
	Vec3[3] axes;
	Vec3 half;
}

fn OrientedBox oriented_box(TransformedShape* collider) @local
{
	Aabb3* box = (Aabb3*)collider.shape;
	Vec3[3] rows = collider.rotation.rows();
	Vec3 middle = (box.min + box.max) * 0.5f * collider.scale;
	return {
		.centre = (Vec3){ rows[0].dot(middle), rows[1].dot(middle), rows[2].dot(middle) } + collider.translation,
		.axes = {
			{ rows[0][0], rows[1][0], rows[2][0] },
			{ rows[0][1], rows[1][1], rows[2][1] },
			{ rows[0][2], rows[1][2], rows[2][2] },
		},
		.half = math::abs((box.max - box.min) * 0.5f * collider.scale),
	};
}

// How far the box reaches along `axis` from its centre.
fn float OrientedBox.reach(&self, Vec3 axis)
{
	return self.half.x * math::abs(self.axes[0].dot(axis))
		+ self.half.y * math::abs(self.axes[1].dot(axis))
		+ self.half.z * math::abs(self.axes[2].dot(axis));
}

// The corner furthest along `direction`.
fn Vec3 OrientedBox.corner_along(&self, Vec3 direction)
{
	Vec3 at = self.centre;
	for (int i = 0; i < 3; i++)
	{
		at += self.axes[i] * (self.axes[i].dot(direction) >= 0 ? self.half[i] : -self.half[i]);
	}
	return at;
}

// Closest points of two segments, each a centre, a unit direction and a half length.
fn Vec3[2] closest_on_edges(Vec3 pa, Vec3 da, float ha, Vec3 pb, Vec3 db, float hb) @local
{
	Vec3 r = pa - pb;
	float b = da.dot(db);
	float c = da.dot(r);
	float f = db.dot(r);
	float denom = 1.0f - b * b;
	float s = denom > 1e-6f ? math::clamp((b * f - c) / denom, -ha, ha) : 0.0f;
	float t = math::clamp(b * s + f, -hb, hb);
	s = math::clamp(b * t - c, -ha, ha);
	return { pa + da * s, pb + db * t };
}

<*
 Two boxes by the separating axis theorem: their three face axes each and the
 nine crosses of their edges. The contact is along the axis they overlap least
 on, in the form EPA answers with (the normal from `a` towards `b`, a point on
 each, `contact[0] - contact[1] = normal * depth`), so the manifold built from
 it does not know which test made it.

 Exact where GJK and EPA iterate, and it measures a gap as readily as an
 overlap: a pair up to `margin` apart gets a contact with the depth negative by
 the gap, which with GJK took a second test on a grown shape and EPA's slowest
 case, the rounded edges.

 @param [&in] a : "A box collider"
 @param [&in] b : "A box collider"
 @param margin : "How far apart the boxes may be and still touch"
*>
fn CollisionInfo box_box_collision(TransformedShape* a, TransformedShape* b, float margin)
{
	OrientedBox ba = oriented_box(a);
	OrientedBox bb = oriented_box(b);
	Vec3 between = bb.centre - ba.centre;

	// Separation along the best axis so far: negative is an overlap.
	float best = -float::max;
	Vec3 normal;
	// 0-2 a face of `a`, 3-5 a face of `b`, 6-14 an edge of each.
	int kind = -1;

	for (int i = 0; i < 6; i++)
	{
		Vec3 axis = i < 3 ? ba.axes[i] : bb.axes[i - 3];
		float along = between.dot(axis);
		float gap = math::abs(along) - ba.reach(axis) - bb.reach(axis);
		if (gap > margin) return { };
		if (gap > best)
		{
			best = gap;
			normal = along < 0 ? -axis : axis;
			kind = i;
		}
	}

	// An edge axis wins only by a clear margin: two faces resting on each other
	// share parallel edges, and their crosses would otherwise tie with the face.
	float face_best = best;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Vec3 axis = ba.axes[i].cross(bb.axes[j]);
			float length = axis.length();
			if (length < EDGE_PARALLEL) continue;
			axis /= length;
			float along = between.dot(axis);
			float gap = math::abs(along) - ba.reach(axis) - bb.reach(axis);
			if (gap > margin) return { };
			if (gap > best && gap > face_best + EDGE_PREFERENCE)
			{
				best = gap;
				normal = along < 0 ? -axis : axis;
				kind = 6 + i * 3 + j;
			}
		}
	}

	CollisionInfo info = { .collided = true, .normal = normal, .depth = -best };
	switch
	{
		case kind < 3:
			// A face of `a`: the corner of `b` furthest into it.
			Vec3 on_b = bb.corner_along(-normal);
			info.contact_points = { on_b + normal * info.depth, on_b };
		case kind < 6:
			Vec3 on_a = ba.corner_along(normal);
			info.contact_points = { on_a, on_a - normal * info.depth };
		default:
			// An edge of each: the two edges that reach furthest towards each
			// other, and the closest points between them.
			int i = (kind - 6) / 3;
			int j = (kind - 6) % 3;
			Vec3 mid_a = ba.corner_along(normal) - ba.axes[i] * (ba.axes[i].dot(normal) >= 0 ? ba.half[i] : -ba.half[i]);
			Vec3 mid_b = bb.corner_along(-normal) - bb.axes[j] * (bb.axes[j].dot(-normal) >= 0 ? bb.half[j] : -bb.half[j]);
			info.contact_points = closest_on_edges(mid_a, ba.axes[i], ba.half[i], mid_b, bb.axes[j], bb.half[j]);
	}
	return info;
}

// A cross of two edges this short is two parallel edges, which have no axis of their own.
const float EDGE_PARALLEL @local = 1e-4f;
// How much less an edge axis must overlap than the best face axis to be chosen, in world units.
const float EDGE_PREFERENCE @local = 1e-4f;

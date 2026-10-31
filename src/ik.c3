<*
 Inverse kinematics over a chain of points.

 Forward kinematics asks where a chain ends up given its angles. This asks the
 other question: given where the end should be, what angles put it there. It is
 the arithmetic behind dragging a character's foot and having the knee and hip
 bend to follow, and behind aiming any jointed thing — a robot arm, a camera
 boom, a tentacle — at a point.

 **The chain is a polyline, not a skeleton.** Rigs store rotations in conventions a
 library cannot guess, so the seam is world space: the caller runs its own forward
 kinematics, hands over world positions, and converts the returned world rotations
 back into whatever it stores.

 Joint limits are deliberately absent; `solver/joint.c3` has the vocabulary (`Limit`).
*>
module collision::ik;

import std::math;
import std::math::quaternion;
import collision::dmath;

// Its own rather than borrowed from `module collision`, whose Vec3 is @private.
// Structurally the same three floats, so values pass between the two — and
// between this and a caller's own vector type — with no conversion at all.
alias Vec3 @private = float[<3>];

<*
 Raised when the two arrays cannot describe a chain: fewer than two points, or
 a `turns` that is not exactly one shorter than `points`.

 A fault rather than a silent clamp, because both are the caller mis-sizing a
 buffer, and a solve that quietly ran over a chain the caller did not mean
 returns numbers that look like an answer.
*>
faultdef IK_CHAIN_SHAPE;

<*
 How many sweeps a solve makes before giving up.

 Enough to converge on a two- or three-link limb within one mouse move, cheap
 enough to run on every one: a sweep is a handful of cross products per pivot,
 and nothing is allocated.
*>
const uint IK_PASSES = 12;

<*
 The largest turn one pivot may take in one sweep, in radians — about 29°.

 What keeps a pointer that jumped across the screen, or a target that teleported
 between frames, from folding a knee through the leg in a single step. A solve
 started from the previous answer walks there over a few frames instead, which
 is both stabler and what a person watching expects to see.
*>
const float IK_MAX_STEP = 0.5;

<*
 Bend a chain of points so its last point reaches toward `target`.

 	`float gap = ik::solve_chain(points[..], turns[..], target)!;`

 Cyclic coordinate descent. `points[0]` is the anchor and never moves; the last
 point is the effector, the one being aimed. Every point between them is a pivot
 carried by the pivots above it, and one sweep turns each pivot in turn by the
 shortest rotation carrying the effector toward the target.

 CCD is the method a point chain can actually support: no Jacobian, no matrix
 inverse, no per-joint frame, and no allocation. It converges quickly on the
 short chains a limb is made of, which is what it is for.

 **Sweeping from the effector upward, not down from the anchor.** The pivot
 nearest the effector has the least leverage, so it moves first and the ones
 above it correct whatever is left. Sweeping the other way has the anchor swing
 the whole chain first, and every pivot below spends the rest of the sweep
 undoing it — it still converges, just slower.

 **A point chain has no twist.** Rotating a segment about its own length moves
 no point, so what comes back is a composition of shortest-arc turns and nothing
 else: the minimum-twist answer. For a limb that is the right answer — pulling
 an ankle should not roll the thigh — and a caller wanting twist should apply it
 itself, on top.

 Segment lengths are preserved exactly, because every step is a rotation about a
 point already on the chain. A solve that stretched them would reach any target
 at all, which is the failure that looks most like success.

 **What the reported turns mean.** Entry i is the *accumulated* world rotation
 the chain below pivot i has undergone — its ancestors' turns folded in — taken
 about that pivot's own position. Accumulated rather than each pivot's own share,
 because a caller converting into a parent-relative frame needs its ancestors'
 total anyway, and having it here saves that caller a second walk.

 @param points : `In and out. World positions, anchor first, effector last; rewritten to where the solve left them. At least two.`
 @param turns : `Out, overwritten rather than composed onto. One rotation per pivot, in the same order as the points and one shorter. See the note above for what each entry means.`
 @param target : `Where the effector should end up. Out of reach is not an error: the chain straightens toward it, which is what a person dragging past the limb's length expects to see.`
 @param passes : `How many sweeps to make. Also the termination condition when the target cannot be reached, since the residual then stops falling.`
 @param tolerance : `Stop early once the effector is this close. In the caller's own units, so a library cannot pick it; zero, the default, never stops early.`
 @param max_step : `The per-pivot per-sweep clamp, in radians. Zero or less removes it.`
 @return `How far the effector still is from the target when the solve stopped. Out of reach is read off this rather than off a flag, because the number is also what a caller would want to show.`
 @return? IK_CHAIN_SHAPE : `The arrays cannot describe a chain.`
*>
fn float? solve_chain(
	Vec3[] points,
	Quaternionf[] turns,
	Vec3 target,
	uint passes = IK_PASSES,
	float tolerance = 0.0,
	float max_step = IK_MAX_STEP)
{
	usz count = points.len;
	if (count < 2 || turns.len != count - 1) return IK_CHAIN_SHAPE~;

	foreach (&turn : turns) *turn = quaternion::IDENTITY;

	usz tip = count - 1;

	for (uint pass = 0; pass < passes; pass++)
	{
		float gap = (target - points[tip]).length();
		if (tolerance > 0.0f && gap <= tolerance) return gap;

		// From the pivot nearest the effector up to the anchor. `k` counts down
		// through the points so that `pivot` never has to go below zero, which
		// an unsigned loop variable cannot do.
		for (usz k = tip; k > 0; k--)
		{
			usz pivot = k - 1;
			Quaternionf step = shortest_arc(
				points[tip] - points[pivot], target - points[pivot], max_step);

			// Everything below the pivot rides with it. The pivot itself is the
			// centre of the turn and so stays exactly where it was, which is what
			// keeps the segments above this one untouched.
			for (usz i = pivot + 1; i < count; i++)
			{
				points[i] = points[pivot] + turned(step, points[i] - points[pivot]);
			}

			// Every pivot at or below this one has just been carried by `step`,
			// and that is what makes the reported turns accumulated.
			//
			// Renormalised as it goes — though the drift it guards against is
			// small, and no check in the suite fails without it. `turns` is reset
			// at the top of every call, so what can accumulate is one call's worth
			// of products rather than a session's. It stays because it costs a
			// square root on a handful of quaternions, and because the guarantee a
			// caller wants from this is "unit", not "unit to within however many
			// passes you happened to ask for".
			for (usz i = pivot; i < turns.len; i++)
			{
				turns[i] = step.mul(turns[i]).normalize();
			}
		}
	}

	return (target - points[tip]).length();
}

<*
 `v` rotated by `q`, for a unit `q`. The expanded form (two cross products) is
 cheaper than std's `q * v` (`rotate_vec3`), and this runs once per point per
 pivot per sweep.
*>
macro Vec3 turned(Quaternionf q, Vec3 v)
{
	Vec3 axis = q.v.xyz;
	Vec3 half = axis.cross(v) + q.v.w * v;
	return v + axis.cross(half) * 2.0f;
}

<*
 The shortest turn carrying `from` onto `to`, clamped to `limit` radians.

 `atan2` of the cross and dot rather than `acos` of the dot: a converging solve
 is made mostly of small corrections, and near zero `acos` has almost no
 precision left — its derivative is unbounded exactly where the answers matter.
 `atan2` is well conditioned across the whole range.

 A zero-length input has no direction to turn and gives back the identity.
 Antiparallel inputs have no *shortest* turn, so `any_perpendicular` picks one.
 The quaternion is built with dmath, not std's `from_axis_angle` (libm sin/cos
 differ between platforms and would break lockstep determinism).

 @param limit : `Zero or less removes the clamp.`
*>
fn Quaternionf shortest_arc(Vec3 from, Vec3 to, float limit = 0.0)
{
	float from_len = from.length();
	float to_len = to.length();
	if (from_len <= 0.0f || to_len <= 0.0f) return quaternion::IDENTITY;

	Vec3 axis;
	float angle = collision::angle_axis_between(from / from_len, to / to_len, &axis);
	if (angle <= 0.0f) return quaternion::IDENTITY;

	if (limit > 0.0f && angle > limit) angle = limit;
	return dmath::quat_from_axis_angle(axis, angle);
}

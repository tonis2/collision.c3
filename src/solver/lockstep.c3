module collision;

import std::collections::list;
import std::sort;
import std::math;

// Lockstep netcode support: bit-wise state hashing for desync detection and
// world snapshot/restore for rollback. See determinism.md for the full
// determinism contract.

faultdef SNAPSHOT_MISMATCH;

// FNV-1a over 64-bit words. Floats are hashed by bit pattern: the lockstep
// bar is bit-equality, so +0.0 and -0.0 intentionally hash differently.
const ulong FNV_OFFSET_BASIS @local = 0xcbf2_9ce4_8422_2325;

fn ulong hash_u64(ulong h, ulong v) @local => (h ^ v) * 0x100_0000_01b3;
fn ulong hash_f32(ulong h, float f) @local => hash_u64(h, (ulong)bitcast(f, uint));

fn ulong hash_vec3(ulong h, Vec3 v) @local
{
	h = hash_f32(h, v.x);
	h = hash_f32(h, v.y);
	return hash_f32(h, v.z);
}

fn ulong hash_quat(ulong h, QuaternionValue q) @local
{
	h = hash_f32(h, q.v[0]);
	h = hash_f32(h, q.v[1]);
	h = hash_f32(h, q.v[2]);
	return hash_f32(h, q.v[3]);
}

<*
 Bit-wise hash of all dynamic simulation state, iterated in body insertion
 order. Peers in lockstep exchange this per frame (or every N frames): equal
 inputs must produce equal hashes, and the first mismatching frame pinpoints
 a desync. O(bodies + soft-body particles).
*>
fn ulong PhysicsWorld.state_hash(&self)
{
	ulong h = FNV_OFFSET_BASIS;

	h = hash_u64(h, (ulong)self.bodies.len());
	self.@each_body(; Rigidbody* body)
	{
		h = hash_u64(h, (ulong)body.id);
		h = hash_vec3(h, body.collider.translation);
		h = hash_quat(h, body.collider.rotation);
		h = hash_vec3(h, body.linear_velocity);
		h = hash_vec3(h, body.angular_velocity);
		h = hash_f32(h, body.sleep_timer);
		h = hash_u64(h, body.sleeping ? 1 : 0);
	};

	h = hash_u64(h, (ulong)self.soft_bodies.len());
	foreach (&sb : self.soft_bodies)
	{
		h = hash_u64(h, (ulong)sb.id);
		foreach (p : sb.positions) h = hash_vec3(h, p);
		foreach (v : sb.velocities) h = hash_vec3(h, v);
	}

	return h;
}

struct RigidbodySnapshot
{
	sz id;
	Vec3 translation;
	QuaternionValue rotation;
	Vec3 linear_velocity;
	Vec3 angular_velocity;
	float sleep_timer;
	bool sleeping;
}

struct SoftBodySnapshot
{
	sz id;
	List {Vec3} positions;
	List {Vec3} velocities;
}

struct PairAnchor
{
	Pair pair;
	FrictionAnchor anchor;
}

fn int pair_anchor_cmp(PairAnchor x, PairAnchor y) @local => pair_id_cmp(x.pair, y.pair);

struct WorldSnapshot
{
	List {RigidbodySnapshot} bodies;
	List {SoftBodySnapshot} soft_bodies;
	// Overlap sets are part of simulation state: they decide whether the
	// next step emits ENTER/START or nothing, so rollback must restore them
	// for event replay to match.
	List {Pair} active_triggers;
	List {Pair} active_contacts;
	// Where static friction holds each touching pair: the next step solves
	// friction against these, so a rollback without them diverges.
	List {PairAnchor} friction_anchors;
}

fn void WorldSnapshot.free(&self)
{
	foreach (&sb : self.soft_bodies)
	{
		sb.positions.free();
		sb.velocities.free();
	}
	self.bodies.free();
	self.soft_bodies.free();
	self.active_triggers.free();
	self.active_contacts.free();
	self.friction_anchors.free();
}

<*
 Capture all dynamic simulation state. Static topology (bodies, joints,
 filters, colliders) is NOT captured: restore() requires the same body and
 soft-body set to still exist. Free the snapshot with WorldSnapshot.free().

 Every list is seated on `allocator` before anything is pushed: a snapshot is
 taken inside a frame scope and used outside it, and a zero-valued list would
 bind to the temp allocator and be gone by the frame it exists to rewind to.

 @param allocator : `Owns every list in the snapshot; WorldSnapshot.free returns them to it.`
*>
fn WorldSnapshot PhysicsWorld.snapshot(&self, Allocator allocator = mem)
{
	WorldSnapshot snap;
	snap.bodies.init(allocator, math::max(self.bodies.len(), (sz)1));
	snap.soft_bodies.init(allocator, math::max(self.soft_bodies.len(), (sz)1));
	snap.active_triggers.init(allocator, 1);
	snap.active_contacts.init(allocator, 1);
	snap.friction_anchors.init(allocator, 1);

	self.@each_body(; Rigidbody* body)
	{
		snap.bodies.push(
			{
				.id = body.id,
				.translation = body.collider.translation,
				.rotation = body.collider.rotation,
				.linear_velocity = body.linear_velocity,
				.angular_velocity = body.angular_velocity,
				.sleep_timer = body.sleep_timer,
				.sleeping = body.sleeping,
			}
		);
	};

	foreach (&sb : self.soft_bodies)
	{
		SoftBodySnapshot s = { .id = sb.id };
		// The nested pair: seated too, since `free` must walk them explicitly.
		s.positions.init(allocator, math::max(sb.positions.len(), (sz)1));
		s.velocities.init(allocator, math::max(sb.velocities.len(), (sz)1));
		s.positions.add_all(&sb.positions);
		s.velocities.add_all(&sb.velocities);
		snap.soft_bodies.push(s);
	}

	self.active_triggers.@each(; Pair pair, bool active)
	{
		snap.active_triggers.push(pair);
	};
	self.active_contacts.@each(; Pair pair, bool active)
	{
		snap.active_contacts.push(pair);
	};
	self.friction_anchors.@each(; Pair pair, FrictionAnchor anchor)
	{
		snap.friction_anchors.push({ pair, anchor });
	};
	// Canonical order, so snapshots themselves are comparable across peers
	if (snap.active_triggers.len() > 1) sort::quicksort(snap.active_triggers.array_view(), &pair_id_cmp);
	if (snap.active_contacts.len() > 1) sort::quicksort(snap.active_contacts.array_view(), &pair_id_cmp);
	if (snap.friction_anchors.len() > 1) sort::quicksort(snap.friction_anchors.array_view(), &pair_anchor_cmp);

	return snap;
}

<*
 Re-apply a snapshot taken from this world (rollback netcode, save states).
 The world must contain exactly the bodies and soft bodies the snapshot
 captured; re-simulating from the restored state reproduces the original
 steps bit-identically given the same inputs.

 Returns SNAPSHOT_MISMATCH if the body or soft-body sets differ.
*>
fn void? PhysicsWorld.restore(&self, WorldSnapshot* snap)
{
	if (self.bodies.len() != snap.bodies.len()) return SNAPSHOT_MISMATCH~;
	if (self.soft_bodies.len() != snap.soft_bodies.len()) return SNAPSHOT_MISMATCH~;

	foreach (&s : snap.bodies)
	{
		Rigidbody*? body = self.find_body(s.id);
		if (catch body) return SNAPSHOT_MISMATCH~;
		body.collider.translation = s.translation;
		body.collider.rotation = s.rotation;
		body.linear_velocity = s.linear_velocity;
		body.angular_velocity = s.angular_velocity;
		body.sleep_timer = s.sleep_timer;
		body.sleeping = s.sleeping;

		// The inverse-inertia cache is keyed by rotation, so the restored rotation
		// invalidates it by itself; only the broadphase needs syncing.
		self.refresh_broadphase(body);
	}

	foreach (i, &s : snap.soft_bodies)
	{
		SoftBody* sb = &self.soft_bodies[i];
		if (sb.id != s.id || sb.positions.len() != s.positions.len()) return SNAPSHOT_MISMATCH~;
		for (sz j = 0; j < s.positions.len(); j++)
		{
			sb.positions[j] = s.positions[j];
			sb.velocities[j] = s.velocities[j];
		}
	}

	// The overlap sets outlive this call, so they must not bind to the temp pool.
	self.seat_storage();

	self.active_triggers.clear();
	foreach (pair : snap.active_triggers) self.active_triggers.set(pair, true);
	self.active_contacts.clear();
	foreach (pair : snap.active_contacts) self.active_contacts.set(pair, true);
	self.friction_anchors.clear();
	foreach (entry : snap.friction_anchors) self.friction_anchors.set(entry.pair, entry.anchor);

	// Frame-transient outputs from the pre-restore timeline are stale
	self.trigger_events.clear();
	self.contact_events.clear();
	self.collisions.clear();
}

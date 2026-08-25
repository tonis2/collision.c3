module collision;

import std::math;
import std::collections::list;
import std::collections::map;

alias CellList = List {sz};
alias SpatialMap = HashMap {sz, CellList};
alias PairSet = HashMap {Pair, bool};
alias LargeList = List {LargeBody};
alias BoxMap = HashMap {sz, Aabb3};

<*
 A body whose AABB covers more cells than the grid is worth spending on, kept
 whole beside the grid instead of being smeared across it.

 The box travels with the id because it is the only thing the large phase has to
 cull with: an entry here is tested against every other body's box once per
 `@get_pairs`, and without the box that test would be "pair it with everything".
*>
struct LargeBody
{
	sz id;
	Aabb3 box;
}

<*
 How many cells a box may cover and still go into the grid. Past this it goes
 into the large bucket instead — see `SpatialHash3D.is_large`.

 64 is an 8x8x8-cell region, which at the default `cell_size` of 2.0 is a
 16-unit cube. Below that the grid is doing what a grid is for; above it the
 insert is copying one id into hundreds of lists so that the pair loop can find
 it hundreds of times and discard all but one.

 A guess, and named so it can be measured rather than argued about. It is per
 hash, so a world with a different `cell_size` can move it without touching
 this file.
*>
const uint LARGE_CELLS = 64;

struct SpatialHash3D
{
	float cell_size;
	SpatialMap table;
	PairSet unique_map;
	<*
	 Bodies too big for the grid. Never in `table` — that is the whole of the
	 mechanism, and why `insert` returns early.

	 A level's static geometry is exactly this list: a floor, a wall and a
	 platform are all large and none of them ever moves, so the cost they used
	 to pay per frame in the grid is paid here as one AABB test per body per
	 step.
	*>
	LargeList large;
	<*
	 Every id currently in this hash, with the box it was last inserted or
	 updated with. Both a registry and the large phase's cull: `@get_pairs`
	 walks it once per large body, and an entry that does not overlap is not a
	 pair.

	 It is the reason a large body costs O(bodies) per step rather than
	 O(cells it covers), and the reason that cost is bounded by something a
	 caller can see.
	*>
	BoxMap boxes;
	<*
	 The cell count past which a box is large. Zero means `LARGE_CELLS`, so a
	 zero-valued hash behaves like a configured one — the same rule
	 `cell_size` follows through `DEFAULT_PHYSICS_WORLD`.
	*>
	uint large_cells;
	// Where the per-cell lists are allocated. See `SpatialHash3D.cell` for why
	// leaving this to chance is not survivable.
	Allocator allocator;
}

<*
 Put the map's storage on a named allocator. Optional — `cell` falls back to the
 heap — but the explicit call is what a long-lived owner should make.
*>
fn void SpatialHash3D.init(&self, Allocator allocator)
{
	self.allocator = allocator;
	self.table.init(allocator);
	self.unique_map.init(allocator);
	self.large.init(allocator);
	self.boxes.init(allocator);
}

<*
 Seat the containers that are not the cell table, for a hash nobody called
 `init` on.

 The same defence `cell` makes and for the same reason: a zero-valued C3 list or
 map binds itself to the *temp* allocator on its first push, which is a pool
 that is gone by the next frame — and the failure surfaces as a broadphase
 yielding ids nobody inserted, nowhere near this line.
*>
fn void SpatialHash3D.bind(&self) @inline
{
	if (self.allocator.ptr == null) self.allocator = mem;
	if (self.large.allocator.ptr == null) self.large.init(self.allocator);
	if (self.boxes.allocator.ptr == null) self.boxes.init(self.allocator);
}

<*
 The list of ids in a cell, on an allocator that will outlive the frame.

 **`get_or_create_ref` hands back a zero-valued `CellList` for a cell that did
 not exist, and a zero-valued C3 list binds itself to the *temp* allocator on
 its first push.** That is invisible for the length of a test, where the temp
 pool outlives every assertion, and fatal in anything that keeps a hash across
 frames: the pool is released, the cells go on pointing into it, and the
 broadphase starts yielding body ids nobody ever inserted. It surfaces as an
 unwrapped NOT_FOUND deep inside the solver, nowhere near this line.

 The fallback is the heap rather than temp, so a caller that never calls `init`
 gets the behaviour they almost certainly meant.
*>
fn CellList* SpatialHash3D.cell(&self, sz key)
{
	if (self.allocator.ptr == null) self.allocator = mem;
	CellList* cells = self.table.get_or_create_ref(key);
	if (cells.allocator.ptr == null) cells.init(self.allocator);
	return cells;
}

fn void SpatialHash3D.free(&self)
{
	self.table.@each(; sz id, CellList cells)
	{
		cells.free();
	};
	self.table.free();
	self.unique_map.free();
	self.large.free();
	self.boxes.free();
}

fn int[<3>] SpatialHash3D.world_to_cell(&self, Vec3 pos)
{
	return {
		(int)math::floor(pos.x / self.cell_size),
		(int)math::floor(pos.y / self.cell_size),
		(int)math::floor(pos.z / self.cell_size),
	};
}

<*
 Exact (injective) table key for an integer grid cell: each coordinate maps to
 its own 21-bit field, so distinct cells within ±2^20 cells per axis can never
 share a table entry. The key IS the cell identity — distribution is the
 HashMap's job, so no information may be lost here.
*>
fn sz SpatialHash3D.cell_key(&self, int x, int y, int z) @inline
{
	const ulong MASK = 0x1f_ffff; // 21 bits per axis
	return (sz)((((ulong)x & MASK) << 42) | (((ulong)y & MASK) << 21) | ((ulong)z & MASK));
}

<*
 Whether this box goes into the large bucket instead of into the grid.

 **The arithmetic is in float on purpose.** The question is asked of boxes that
 have just come out of a transform, and the two answers that matter most are the
 ones an integer cell count cannot give: a box the size of a level overflows the
 product, and a box with a NaN corner makes every comparison false. Both have to
 come back "large", because the large bucket is the only path that never divides
 the box by a cell size — so both are handled by leaving the comparison
 un-negated and letting NaN fall through it.

 @param box : `The world box, as it will be stored.`
 @return `true when the box covers more cells than the threshold, is unbounded, or is not a number.`
*>
fn bool SpatialHash3D.is_large(&self, Aabb3 box)
{
	float size = self.cell_size > 0 ? self.cell_size : 1.0f;
	Vec3 span = box.size() * (1.0f / size);
	// One per axis for the cell the minimum corner itself sits in, which is what
	// the `<=` in each of `insert`'s loops adds.
	float cells = (span.x + 1.0f) * (span.y + 1.0f) * (span.z + 1.0f);
	uint limit = self.large_cells > 0 ? self.large_cells : LARGE_CELLS;
	return !(cells <= (float)limit);
}

<*
 Push an id into every cell its box covers. `insert`'s loop, named so that
 `update` can reach it when a body stops being large.
*>
fn void SpatialHash3D.fill_cells(&self, Aabb3 box, sz id)
{
	int[<3>] min_cell = self.world_to_cell(box.min);
	int[<3>] max_cell = self.world_to_cell(box.max);

	for (int x = min_cell.x; x <= max_cell.x; ++x)
	{
		for (int y = min_cell.y; y <= max_cell.y; ++y)
		{
			for (int z = min_cell.z; z <= max_cell.z; ++z)
			{
				self.cell(self.cell_key(x, y, z)).push(id);
			}
		}
	}
}

<*
 Drop an id from every cell its box covers, freeing the cells it emptied.
 `remove`'s loop, named for the same reason `fill_cells` is.
*>
fn void SpatialHash3D.erase_cells(&self, Aabb3 box, sz id)
{
	int[<3>] min_cell = self.world_to_cell(box.min);
	int[<3>] max_cell = self.world_to_cell(box.max);

	for (int x = min_cell.x; x <= max_cell.x; ++x)
	{
		for (int y = min_cell.y; y <= max_cell.y; ++y)
		{
			for (int z = min_cell.z; z <= max_cell.z; ++z)
			{
				sz cell_id = self.cell_key(x, y, z);
				CellList*? cells = self.table.get_ref(cell_id);
				if (catch cells) continue;
				cells.remove_using_test(fn (id, ctx) => *id == *(sz*)ctx, &&id);
				if (cells.is_empty())
				{
					cells.free();
					self.table.remove(cell_id);
				}
			}
		}
	}
}

<*
 Take an id out of the large bucket.

 A linear scan, and it stays one: the bucket holds the bodies that are too big
 for a grid, which in every scene this has been measured on is the level's static
 geometry — tens of entries, none of which move.

 @return `true when the id was in the bucket, which is also "it was not in the grid".`
*>
fn bool SpatialHash3D.drop_large(&self, sz id)
{
	foreach (i, entry : self.large)
	{
		if (entry.id != id) continue;
		self.large.remove_at(i);
		return true;
	}
	return false;
}

<*
 Adds a 3D AABB box into SpatialHash.

 **A box bigger than `large_cells` cells does not go into the grid at all.** It
 goes into `large`, and `@get_pairs` tests it against every other body's box
 once per step instead of finding it once per shared cell. The contract this
 used to carry was the other answer to the same question — a level's ground
 plane is 49,152 cells at the default cell size, and the assert stopped it at
 the door rather than letting the broadphase walk it sixty times a second. The
 bucket is what makes the box merely large instead of refused.

 What is left of the contract is the half that was never about size: a box with
 a NaN corner is memory corruption or a NaN transform upstream, and every
 comparison it reaches is false, so it is caught here where the box still has a
 caller attached to it. **The old lower bound is gone** — it rejected any
 collider under 0.183 units a side, which is a real shape (crate chips, gravel,
 debris) and costs exactly one cell.

 @param box : `3D aabb box, that's inserted into the map`
 @param id : `ID thats attached to the box, could be mesh ID`
 @require box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
*>
fn void SpatialHash3D.insert(&self, Aabb3 box, sz id)
{
	self.bind();
	self.boxes.set(id, box);

	if (self.is_large(box))
	{
		self.large.push({ id, box });
		return;
	}

	self.fill_cells(box, id);
}

<*
 Removes a box from the map.

 The half `update` has had inline since it was written — it drops an id from the
 cells a box left. `remove_body` is what needed it spelled out: a dead id still
 sitting in its cells goes on producing broadphase pairs forever.

 @param box : `The box the id was last inserted or updated with`
 @param id : `The id to drop`
*>
fn void SpatialHash3D.remove(&self, Aabb3 box, sz id)
{
	self.bind();
	(void)self.boxes.remove(id);
	// The bucket is checked by id and not by size: a body that was inserted
	// large and is being removed with a box that has since been recomputed
	// smaller would otherwise be dropped from neither place.
	if (self.drop_large(id)) return;

	self.erase_cells(box, id);
}

<*
 Updates box location in spatial map
 @param old_box : `Previous box location`
 @param new_box : `New box location`
 @param id : `Box id`
 @require old_box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
 @require new_box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
*>
fn void SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, sz id)
{
	self.bind();
	self.boxes.set(id, new_box);

	// **The four transitions, and only two of them are the diff below.** A body
	// can cross the large threshold in either direction — a shape rescaled, a
	// mesh swapped, a rotation that widens a box — and the two crossings are the
	// ones that would otherwise leave an id in both places or in neither.
	bool was_large = self.drop_large(id);
	if (self.is_large(new_box))
	{
		self.large.push({ id, new_box });
		if (!was_large) self.erase_cells(old_box, id);
		return;
	}
	if (was_large)
	{
		self.fill_cells(new_box, id);
		return;
	}

	int[<3>] old_min = self.world_to_cell(old_box.min);
	int[<3>] old_max = self.world_to_cell(old_box.max);

	int[<3>] new_min = self.world_to_cell(new_box.min);
	int[<3>] new_max = self.world_to_cell(new_box.max);

	// Early exit if grid cells haven't changed
	if (old_min == new_min && old_max == new_max) return;

	// Remove from old cells that are NOT in new range
	for (int x = old_min.x; x <= old_max.x; ++x)
	{
		for (int y = old_min.y; y <= old_max.y; ++y)
		{
			for (int z = old_min.z; z <= old_max.z; ++z)
			{
				bool in_new = x >= new_min.x && x <= new_max.x && y >= new_min.y && y <= new_max.y && z >= new_min.z
					&& z <= new_max.z;
				if (!in_new)
				{
					sz cell_id = self.cell_key(x, y, z);
					CellList*? cells = self.table.get_ref(cell_id);
					if (try cells)
					{
						cells.remove_using_test(fn (id, ctx) => *id == *(sz*)ctx, &&id);
						if (cells.is_empty())
						{
							cells.free();
							self.table.remove(cell_id);
						}
					}
				}
			}
		}
	}

	// Add to new cells that were NOT in old range
	for (int x = new_min.x; x <= new_max.x; ++x)
	{
		for (int y = new_min.y; y <= new_max.y; ++y)
		{
			for (int z = new_min.z; z <= new_max.z; ++z)
			{
				bool in_old = x >= old_min.x && x <= old_max.x && y >= old_min.y && y <= old_max.y && z >= old_min.z
					&& z <= old_max.z;
				if (!in_old)
				{
					self.cell(self.cell_key(x, y, z)).push(id);
				}
			}
		}
	}
}

<*
 Every pair of ids that share a cell, plus every pair a large body overlaps.

 **Two phases, because there are two storages.** The grid answers small against
 small, which is what a grid is for. The bucket answers large against everything
 — one AABB test per large body per body, exact rather than conservative, and
 the reason a level's floor is one entry rather than 864 cells.

 The large phase costs `large.len() * boxes.len()` box tests a step. That is a
 number a caller can see and reason about, which the alternative — a large body
 smeared over the grid and rediscovered once per shared cell — is not.
*>
macro void SpatialHash3D.@get_pairs(&self; @callback(Pair pair))
{
	self.table.@each(; sz cell_id, CellList cells)
	{
		for (sz i = 0; i < cells.len(); ++i)
		{
			for (sz j = i + 1; j < cells.len(); ++j)
			{
				sz a = cells[i];
				sz b = cells[j];

				if (a == b) continue;

				Pair pair = { math::min(a, b), math::max(a, b) };

				if (!self.unique_map.has_key(pair))
				{
					@callback(pair);
					self.unique_map.set(pair, true);
				}
			}
		}
	};

	foreach (entry : self.large)
	{
		// **Nested `if` and not a guard `continue`.** The body of `@each` is a
		// macro body block, not a loop body: a `continue` in it is the enclosing
		// loop's, which here is the `foreach` over `large`, so the first
		// self-pair would abandon the whole scan and the level's floor would
		// pair with nothing. It compiles, and the symptom is a body falling
		// through the ground.
		self.boxes.@each(; sz other, Aabb3 box)
		{
			// The cull, and the reason the box is stored beside the id. Without
			// it a floor slab would pair with every body in the level, static
			// geometry included, and the saving would go straight back out
			// through the narrow phase.
			if (other != entry.id && entry.box.collides(box))
			{
				Pair pair = { math::min(entry.id, other), math::max(entry.id, other) };

				if (!self.unique_map.has_key(pair))
				{
					@callback(pair);
					self.unique_map.set(pair, true);
				}
			}
		};
	}

	self.unique_map.clear();
}

<*
 Iterates ids in the cell containing `pos` and its 26 neighboring cells, and
 every large body whose box reaches that neighbourhood.

 The large half is not an extra: a body in the bucket is in no cell at all, so
 without it the one query that answers "what is near this point" would be blind
 to exactly the floor the point is standing on.
*>
macro void SpatialHash3D.@get_nearby_objects(&self, Vec3 pos; @callback(sz id))
{
	int[<3>] cell = self.world_to_cell(pos);

	if (!self.large.is_empty())
	{
		Vec3 reach = { self.cell_size, self.cell_size, self.cell_size };
		Aabb3 near = { pos - reach, pos + reach };
		foreach (entry : self.large)
		{
			if (entry.box.collides(near)) @callback(entry.id);
		}
	}

	for (int dx = -1; dx <= 1; ++dx)
	{
		for (int dy = -1; dy <= 1; ++dy)
		{
			for (int dz = -1; dz <= 1; ++dz)
			{
				if (try cells = self.table.get(self.cell_key(cell.x + dx, cell.y + dy, cell.z + dz)))
				{
					foreach (item : cells) @callback(item);
				}
			}
		}
	}
}

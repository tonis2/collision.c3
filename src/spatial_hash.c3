module collision;

import std::math;
import std::collections::list;
import std::collections::map;

alias CellList = List {sz};
alias SpatialMap = HashMap {sz, CellList};
alias LargeList = List {LargeBody};
alias BoxMap = HashMap {sz, BoxEntry};
alias GridScratch = List {GridEntry};

<*
 A body whose AABB covers more cells than the grid is worth spending on, kept
 whole beside the grid instead of being smeared across it. The box travels with
 the id because it is what the large phase culls with.
*>
struct LargeBody
{
	sz id;
	Aabb3 box;
	bool is_static;
}

<*
 What the hash remembers about every id: the box it was last inserted or updated
 with, the grid cell of that box's minimum corner (which decides which cell
 reports its pairs), and its flags.
*>
struct BoxEntry
{
	Aabb3 box;
	int[<3>] min_cell;
	bool is_static;
	bool large;
}

// One id of the cell being scanned in `@get_pairs`, with what the pair test needs.
struct GridEntry @local
{
	sz id;
	int[<3>] min_cell;
	bool is_static;
}

<*
 How many cells a box may cover and still go into the grid. Past this it goes
 into the large bucket — see `SpatialHash3D.is_large`.

 64 is an 8x8x8-cell region (a 16-unit cube at the default `cell_size` of 2.0).
 Above that an insert copies one id into hundreds of lists only for the pair
 loop to find it hundreds of times. A guess; it is per hash (`large_cells`).
*>
const uint LARGE_CELLS = 64;

// Cell size used when `cell_size` is zero or negative.
const float FALLBACK_CELL_SIZE @local = 1.0f;

struct SpatialHash3D
{
	float cell_size;
	SpatialMap table;
	<*
	 Bodies too big for the grid. Never in `table`. A level's static geometry
	 (floor, walls) lives here: one AABB test per body per step instead of
	 hundreds of cells.
	*>
	LargeList large;
	<*
	 Every id currently in this hash with its `BoxEntry`. The registry the large
	 phase scans and the source of each id's minimum cell for pair ownership.
	*>
	BoxMap boxes;
	// Reused by `@get_pairs` to hold the ids of one cell with their lookups done.
	GridScratch scratch;
	<*
	 The cell count past which a box is large. Zero means `LARGE_CELLS`, so a
	 zero-valued hash behaves like a configured one.
	*>
	uint large_cells;
	// Where cell lists and containers are allocated; see `bind`.
	Allocator allocator;
}

<*
 Put the hash's storage on a named allocator. Optional — `bind` falls back to the
 heap — but a long-lived owner should call it.
*>
fn void SpatialHash3D.init(&self, Allocator allocator)
{
	self.allocator = allocator;
	self.table.init(allocator);
	self.large.init(allocator);
	self.boxes.init(allocator);
	self.scratch.init(allocator);
}

<*
 Seat every container on a real allocator, for a hash nobody called `init` on.

 A zero-valued C3 list or map binds itself to the *temp* allocator on its first
 push; that pool is gone by the next frame and the failure surfaces as a
 broadphase yielding ids nobody inserted, far from this line. The fallback is the
 heap.
*>
fn void SpatialHash3D.bind(&self) @inline
{
	if (self.allocator.ptr == null) self.allocator = mem;
	if (!self.table.is_initialized()) self.table.init(self.allocator);
	if (self.large.allocator.ptr == null) self.large.init(self.allocator);
	if (!self.boxes.is_initialized()) self.boxes.init(self.allocator);
	if (self.scratch.allocator.ptr == null) self.scratch.init(self.allocator);
}

<*
 The list of ids in a cell, created on the hash's allocator (a fresh `CellList`
 would otherwise bind to the temp allocator — see `bind`).
*>
fn CellList* SpatialHash3D.cell(&self, sz key)
{
	self.bind();
	CellList* cells = self.table.get_or_create_ref(key);
	if (cells.allocator.ptr == null) cells.init(self.allocator);
	return cells;
}

fn void SpatialHash3D.free(&self)
{
	self.table.@each_entry(; entry)
	{
		entry.value.free();
	};
	self.table.free();
	self.large.free();
	self.boxes.free();
	self.scratch.free();
}

fn int[<3>] SpatialHash3D.world_to_cell(&self, Vec3 pos)
{
	float inv = 1.0f / (self.cell_size > 0 ? self.cell_size : FALLBACK_CELL_SIZE);
	Vec3 scaled = pos * inv;
	return { (int)math::floor(scaled.x), (int)math::floor(scaled.y), (int)math::floor(scaled.z) };
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

 The arithmetic is in float on purpose: a box the size of a level overflows an
 integer cell product, and a box with a NaN corner makes every comparison false.
 Both must come back "large" (the only path that never divides by a cell size),
 so the comparison is left un-negated and NaN falls through it.

 @param box : `The world box, as it will be stored.`
 @return `true when the box covers more cells than the threshold, is unbounded, or is not a number.`
*>
fn bool SpatialHash3D.is_large(&self, Aabb3 box)
{
	float size = self.cell_size > 0 ? self.cell_size : FALLBACK_CELL_SIZE;
	Vec3 span = box.size() * (1.0f / size);
	// One extra per axis for the cell the minimum corner itself sits in.
	float cells = (span.x + 1.0f) * (span.y + 1.0f) * (span.z + 1.0f);
	uint limit = self.large_cells > 0 ? self.large_cells : LARGE_CELLS;
	return !(cells <= (float)limit);
}

<*
 Visit every integer cell in the inclusive range `lo..hi`.
*>
macro void @foreach_cell(int[<3>] lo, int[<3>] hi; @body(int x, int y, int z))
{
	for (int x = lo.x; x <= hi.x; ++x)
	{
		for (int y = lo.y; y <= hi.y; ++y)
		{
			for (int z = lo.z; z <= hi.z; ++z)
			{
				@body(x, y, z);
			}
		}
	}
}

fn bool cell_in_range(int x, int y, int z, int[<3>] lo, int[<3>] hi) @local @inline
{
	return x >= lo.x && x <= hi.x && y >= lo.y && y <= hi.y && z >= lo.z && z <= hi.z;
}

<*
 Drop `id` from one cell, freeing the cell when it empties.
*>
fn void SpatialHash3D.erase_from_cell(&self, sz cell_id, sz id)
{
	CellList*? cells = self.table.get_ref(cell_id);
	if (catch cells) return;
	cells.remove_using_test(fn (item, ctx) => *item == *(sz*)ctx, &&id);
	if (cells.is_empty())
	{
		cells.free();
		(void)self.table.remove(cell_id);
	}
}

<*
 Push an id into every cell its box covers.
*>
fn void SpatialHash3D.fill_cells(&self, Aabb3 box, sz id)
{
	@foreach_cell(self.world_to_cell(box.min), self.world_to_cell(box.max); int x, int y, int z)
	{
		self.cell(self.cell_key(x, y, z)).push(id);
	};
}

<*
 Drop an id from every cell its box covers.
*>
fn void SpatialHash3D.erase_cells(&self, Aabb3 box, sz id)
{
	@foreach_cell(self.world_to_cell(box.min), self.world_to_cell(box.max); int x, int y, int z)
	{
		self.erase_from_cell(self.cell_key(x, y, z), id);
	};
}

<*
 Take an id out of the large bucket. A linear scan: the bucket holds a level's
 static geometry, tens of entries that never move.

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

fn bool SpatialHash3D.large_contains(&self, sz id)
{
	foreach (entry : self.large) if (entry.id == id) return true;
	return false;
}

<*
 Record `box` and flags for `id` in the registry.
*>
fn void SpatialHash3D.record(&self, sz id, Aabb3 box, bool is_static, bool large)
{
	BoxEntry* entry = self.boxes.get_or_create_ref(id);
	*entry = { .box = box, .min_cell = self.world_to_cell(box.min), .is_static = is_static, .large = large };
}

<*
 Adds a 3D AABB box into the hash.

 A box bigger than `large_cells` cells goes into `large` instead of the grid and
 is tested against every other box once per `@get_pairs`. A NaN corner is caught
 here, where the box still has a caller attached.

 @param box : `3D aabb box, that's inserted into the map`
 @param id : `ID thats attached to the box, could be mesh ID`
 @param is_static : `Two static ids are never reported as a pair. Kept by later updates.`
 @require box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
*>
fn void SpatialHash3D.insert(&self, Aabb3 box, sz id, bool is_static = false)
{
	self.bind();
	bool large = self.is_large(box);
	self.record(id, box, is_static, large);

	if (large)
	{
		self.large.push({ id, box, is_static });
		return;
	}

	self.fill_cells(box, id);
}

<*
 Removes a box from the map. A dead id left in its cells would produce
 broadphase pairs forever.

 @param box : `The box the id was last inserted or updated with`
 @param id : `The id to drop`
*>
fn void SpatialHash3D.remove(&self, Aabb3 box, sz id)
{
	self.bind();
	(void)self.boxes.remove(id);
	// Checked by id, not by size: a body inserted large and removed with a box
	// since recomputed smaller would otherwise be dropped from neither place.
	if (self.drop_large(id)) return;

	self.erase_cells(box, id);
}

<*
 Updates box location in spatial map. The id keeps its static flag.
 @param old_box : `Previous box location`
 @param new_box : `New box location`
 @param id : `Box id`
 @require old_box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
 @require new_box.size().length_sq() >= 0 : `Aabb box is not a number — a NaN corner, from a NaN transform or memory corruption`
*>
fn void SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, sz id)
{
	self.bind();
	int[<3>] new_min = self.world_to_cell(new_box.min);
	int[<3>] new_max = self.world_to_cell(new_box.max);
	bool now_large = self.is_large(new_box);

	// One registry lookup; the entry is rewritten in place.
	bool is_static = false;
	bool was_large = false;
	BoxEntry*? known = self.boxes.get_ref(id);
	if (try known)
	{
		is_static = known.is_static;
		was_large = known.large;
		*known = { .box = new_box, .min_cell = new_min, .is_static = is_static, .large = now_large };
	}
	else
	{
		self.boxes.set(id, { .box = new_box, .min_cell = new_min, .is_static = false, .large = now_large });
		was_large = self.large_contains(id);
	}

	if (!was_large && !now_large)
	{
		int[<3>] old_min = self.world_to_cell(old_box.min);
		int[<3>] old_max = self.world_to_cell(old_box.max);
		if (old_min == new_min && old_max == new_max) return;

		@foreach_cell(old_min, old_max; int x, int y, int z)
		{
			if (!cell_in_range(x, y, z, new_min, new_max)) self.erase_from_cell(self.cell_key(x, y, z), id);
		};
		@foreach_cell(new_min, new_max; int x, int y, int z)
		{
			if (!cell_in_range(x, y, z, old_min, old_max)) self.cell(self.cell_key(x, y, z)).push(id);
		};
		return;
	}

	// A body can cross the large threshold in either direction (rescale, mesh
	// swap, widening rotation); the crossings are what would otherwise leave an
	// id in both places or in neither.
	if (was_large) (void)self.drop_large(id);
	if (now_large)
	{
		self.large.push({ id, new_box, is_static });
		if (!was_large) self.erase_cells(old_box, id);
	}
	else
	{
		self.fill_cells(new_box, id);
	}
}

<*
 Every pair of ids that share a cell, plus every pair a large body overlaps.
 Each unique pair is reported exactly once, normalized `a < b`, and pairs of two
 static ids are never reported.

 **Grid phase, no dedup set.** A pair sharing several cells is reported only
 from the cell holding the componentwise maximum of the two boxes' minimum-corner
 cells. That cell lies in both boxes' ranges whenever they share any cell, and
 each cell is visited once, so the pair comes out once with no allocation. Order
 follows the table's iteration order and the order ids sit in each cell, both of
 which are fixed by insertion order, so two identical runs report identically.

 **Large phase.** One box test per large body per registered id. A large-large
 pair is reported from the lower id's scan only.
*>
macro void SpatialHash3D.@get_pairs(&self; @callback(Pair pair))
{
	self.bind();
	self.table.@each_entry(; entry)
	{
		sz cell_id = entry.key;
		CellList* cells = &entry.value;
		sz n = cells.len();
		if (n > 1)
		{
			self.scratch.clear();
			for (sz i = 0; i < n; ++i)
			{
				sz id = cells.entries[i];
				BoxEntry*? info = self.boxes.get_ref(id);
				if (try info) self.scratch.push({ id, info.min_cell, info.is_static });
			}

			sz m = self.scratch.len();
			for (sz i = 0; i < m; ++i)
			{
				GridEntry first = self.scratch.entries[i];
				for (sz j = i + 1; j < m; ++j)
				{
					GridEntry second = self.scratch.entries[j];
					if (first.id == second.id || (first.is_static && second.is_static)) continue;

					int[<3>] owner = math::max(first.min_cell, second.min_cell);
					if (self.cell_key(owner.x, owner.y, owner.z) != cell_id) continue;

					Pair pair = { math::min(first.id, second.id), math::max(first.id, second.id) };
					@callback(pair);
				}
			}
		}
	};

	foreach (large : self.large)
	{
		// Nested `if`, not a guard `continue`: the `@each` body is not a loop
		// body, so a `continue` there is the enclosing `foreach`'s and the first
		// self-pair would abandon the scan (the floor would pair with nothing).
		self.boxes.@each(; sz other, BoxEntry info)
		{
			bool skip = other == large.id
				|| (large.is_static && info.is_static)
				|| (info.large && other < large.id);
			if (!skip && large.box.collides(info.box))
			{
				Pair pair = { math::min(large.id, other), math::max(large.id, other) };
				@callback(pair);
			}
		};
	}
}

<*
 Iterates ids in the cell containing `pos` and its 26 neighboring cells, and
 every large body whose box reaches that neighbourhood (a large body is in no
 cell, so without it a query would be blind to the floor the point stands on).
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

	@foreach_cell(cell - 1, cell + 1; int x, int y, int z)
	{
		if (try cells = self.table.get(self.cell_key(x, y, z)))
		{
			foreach (item : cells) @callback(item);
		}
	};
}

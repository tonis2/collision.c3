module collision;

import std::math;
import std::collections::list;
import std::collections::map;

alias CellList = List {sz};
alias SpatialMap = HashMap {sz, CellList};
alias PairSet = HashMap {Pair, bool};

struct SpatialHash3D
{
	float cell_size;
	SpatialMap table;
	PairSet unique_map;
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
fn CellList* SpatialHash3D.cell(&self, sz key) @local
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
 Adds a 3D AABB box into SpatialHash
 @param box : `3D aabb box, that's inserted into the map`
 @param id : `ID thats attached to the box, could be mesh ID`
 @require box.size().length_sq() < 100000 : `Aabb box is too big, could be because memory errors`
 @require box.size().length_sq() > 0.1 : `Aabb box is too small, could be because memory errors`
*>
fn void SpatialHash3D.insert(&self, Aabb3 box, sz id)
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
 Removes a box from the map.

 The half `update` has had inline since it was written — it drops an id from the
 cells a box left. `remove_body` is what needed it spelled out: a dead id still
 sitting in its cells goes on producing broadphase pairs forever.

 @param box : `The box the id was last inserted or updated with`
 @param id : `The id to drop`
*>
fn void SpatialHash3D.remove(&self, Aabb3 box, sz id)
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
 Updates box location in spatial map
 @param old_box : `Previous box location`
 @param new_box : `New box location`
 @param id : `Box id`
 @require old_box.size().length_sq() < 100000 && old_box.size().length_sq() > 0.1 : `Aabb box size invalid, could be because memory errors`
 @require new_box.size().length_sq() < 100000 && new_box.size().length_sq() > 0.1 : `Aabb box size invalid, could be because memory errors`
*>
fn void SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, sz id)
{
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

	self.unique_map.clear();
}

<*
 Iterates ids in the cell containing `pos` and its 26 neighboring cells.
*>
macro void SpatialHash3D.@get_nearby_objects(&self, Vec3 pos; @callback(sz id))
{
	int[<3>] cell = self.world_to_cell(pos);

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

module collision;

import std::math;
import std::collections::list;
import std::collections::map;

// Vertices closer than this are welded into one.
const float WELD_RADIUS @local = 0.0707107f;

// Below this many vertices a linear scan beats building the grid.
const sz WELD_GRID_MIN_VERTICES @local = 48;

// Sentinel for "no vertex" in the per-cell chains.
const uint NO_VERTEX @local = uint::max;

// Grid cell of a point, packed one 21-bit field per axis. A collision between
// far-apart cells only adds candidates; every candidate is distance-checked.
fn ulong weld_cell_key(int[<3>] cell) @local @inline
{
	const ulong MASK = 0x1f_ffff;
	return (((ulong)cell.x & MASK) << 42) | (((ulong)cell.y & MASK) << 21) | ((ulong)cell.z & MASK);
}

fn int[<3>] weld_cell_of(Vec3 p) @local @inline
{
	Vec3 scaled = p * (1.0f / WELD_RADIUS);
	return { (int)math::floor(scaled.x), (int)math::floor(scaled.y), (int)math::floor(scaled.z) };
}

<*
 Welds duplicate vertices and removes degenerate triangles.

 Each vertex is merged into the earliest kept vertex within `WELD_RADIUS`; a hash
 grid with cells one radius wide means only the 27 surrounding cells are searched.

 @require !vertices.is_empty() && !triangles.is_empty() : `Vertices and Triangles must not be empty`
*>
fn void weld_remesh(List {Vec3}* vertices, List {Triangle}* triangles)
{
	// Scratch only: released with the pool; the welded result is written back into the caller's lists.
	@pool()
	{
		sz count = vertices.len();

		List {Vec3} kept;
		kept.init(tmem, count);
		// Per kept vertex: the next kept vertex in the same grid cell
		List {uint} next_in_cell;
		next_in_cell.init(tmem, count);
		// Grid cell -> most recently added kept vertex in it
		HashMap {ulong, uint} cell_head;
		cell_head.init(tmem);
		// Old vertex index -> kept vertex index
		uint[] remap = mem::talloc_array(uint, count);
		List {Triangle} kept_tris;
		kept_tris.init(tmem, triangles.len());

		bool use_grid = count > WELD_GRID_MIN_VERTICES;

		foreach (i, point : vertices)
		{
			int[<3>] cell = weld_cell_of(point);
			uint match_index = NO_VERTEX;

			if (use_grid)
			{
				for (int dx = -1; dx <= 1; dx++)
				{
					for (int dy = -1; dy <= 1; dy++)
					{
						for (int dz = -1; dz <= 1; dz++)
						{
							uint*? head = cell_head.get_ref(weld_cell_key(cell + (int[<3>]){ dx, dy, dz }));
							if (catch head) continue;
							for (uint v = *head; v != NO_VERTEX; v = next_in_cell[v])
							{
								// Earliest kept vertex wins, as a linear scan in insertion order would give.
								if (v < match_index && kept[v].near(point, WELD_RADIUS)) match_index = v;
							}
						}
					}
				}
			}
			else
			{
				foreach (v, kept_point : kept)
				{
					if (kept_point.near(point, WELD_RADIUS))
					{
						match_index = (uint)v;
						break;
					}
				}
			}

			if (match_index == NO_VERTEX)
			{
				match_index = (uint)kept.len();
				kept.push(point);
				if (use_grid)
				{
					ulong key = weld_cell_key(cell);
					uint*? existing = cell_head.get_ref(key);
					if (try existing)
					{
						next_in_cell.push(*existing);
						*existing = match_index;
					}
					else
					{
						next_in_cell.push(NO_VERTEX);
						cell_head.set(key, match_index);
					}
				}
			}
			remap[i] = match_index;
		}

		// Remap triangles and drop degenerate ones (two or more vertices merged)
		foreach (tri : triangles)
		{
			uint i0 = remap[tri[0]];
			uint i1 = remap[tri[1]];
			uint i2 = remap[tri[2]];
			if (i0 != i1 && i0 != i2 && i1 != i2) kept_tris.push({ i0, i1, i2 });
		}

		vertices.clear();
		triangles.clear();
		vertices.push_all(kept.array_view());
		triangles.push_all(kept_tris.array_view());
	};
}

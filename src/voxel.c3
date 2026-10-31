module collision;

import std::math;


// ============================================================================
// Solid voxelization and distance fields through the inside of a mesh
//
// Turns a closed triangle mesh into the cells inside it, then measures distance
// through those cells rather than across open space, so limbs stay separate and
// influence cannot leak through a gap.
//
//     VoxelGrid grid = create_voxel_grid(vertices, triangles);
//     defer grid.free();
//
//     float[] field = mem::new_array(float, grid.cells.len);
//     grid.distance_from_segment(start, end, field);
//     float d = grid.sample(field, point);
//
// Resolution is set on the model's longest side, not its triangle count. Features
// thinner than a cell may not fill, and a passage that does not fill is one no
// path travels.
// ============================================================================

// Cells along the model's longest side.
const uint DEFAULT_VOXEL_RESOLUTION = 48;

// Empty cells kept around the model on every side. Two guarantees every interior
// cell has all 26 neighbours in range, so the sweeps need no bounds test.
const int VOXEL_PAD = 2;

// How far a marching ray steps past a surface before looking for the next, as a
// fraction of a cell. Surfaces closer than this merge and invert the parity.
const float VOXEL_CROSSING_STEP = 0.02;

// Crossings counted along one line before it is abandoned. A closed mesh gives a handful.
const usz MAX_VOXEL_CROSSINGS = 64;

// Distance reported for a cell no path reached. Compare before adding to a field value.
const float VOXEL_UNREACHED = float::max;

// Fractions of a cell the grid origin is shifted by, per axis — different on
// each so no axis lines up with round-number geometry.
const Vec3 VOXEL_ORIGIN_JITTER = { 0.0137f, 0.0219f, 0.0311f };

// Of the three axis directions, how many must find a voxel inside for it to count.
const char VOXEL_VOTES_NEEDED = 2;

// Cap on chamfer sweeps; a path that doubles back needs one forward/backward pair per
// reversal. The loop stops early once a pass changes nothing.
const usz MAX_VOXEL_SWEEPS = 12;

// Which cells of a box around the mesh are inside it. `index` and `cells` are the two
// directions of one mapping; fields are sized by `cells`, so solving costs interior
// volume rather than bounding-box volume.
struct VoxelGrid
{
	Vec3 origin;   // the centre of voxel (0, 0, 0)
	float cell;
	int nx, ny, nz;

	int[] index;   // voxel index -> entry in `cells`, or -1 outside the mesh
	int[] cells;   // entry -> voxel index, ascending
	ChamferStencil stencil; // neighbour offsets and step lengths, fixed by the grid's shape
}

// The 3x3x3 neighbourhood in scan order. Slots before the centre (13) are the neighbours
// a forward sweep has already visited; slots after it, a backward sweep.
const usz STENCIL_SIZE = 27;
const usz STENCIL_CENTRE = 13;

struct ChamferStencil
{
	int[STENCIL_SIZE] offset; // voxel-index step to each neighbour
	float[STENCIL_SIZE] cost; // true distance to it
}

fn ChamferStencil make_chamfer_stencil(int nx, int ny, float cell) @local
{
	ChamferStencil stencil;
	usz slot = 0;
	for (int dz = -1; dz <= 1; dz++)
	{
		for (int dy = -1; dy <= 1; dy++)
		{
			for (int dx = -1; dx <= 1; dx++)
			{
				stencil.offset[slot] = dz * nx * ny + dy * nx + dx;
				stencil.cost[slot] = cell * math::sqrt((float)(dx * dx + dy * dy + dz * dz));
				slot++;
			}
		}
	}
	return stencil;
}

// False for a mesh that is open everywhere, flat, or smaller than one cell.
// Callers should treat that as "no grid" rather than as an error.
fn bool VoxelGrid.usable(&self) => self.cells.len > 0;

fn void VoxelGrid.free(&self)
{
	if (self.index.len > 0) mem::free(self.index);
	if (self.cells.len > 0) mem::free(self.cells);
	*self = {};
}

fn int VoxelGrid.at(&self, int x, int y, int z) @inline => (z * self.ny + y) * self.nx + x;

fn Vec3 VoxelGrid.center(&self, int voxel)
{
	int x = voxel % self.nx;
	int y = (voxel / self.nx) % self.ny;
	int z = voxel / (self.nx * self.ny);
	return {
		self.origin.x + (float)x * self.cell,
		self.origin.y + (float)y * self.cell,
		self.origin.z + (float)z * self.cell,
	};
}

// Where an entry sits in space, for callers holding entries rather than voxels.
fn Vec3 VoxelGrid.entry_center(&self, int entry) => self.center(self.cells[entry]);

// `point` in grid units: voxel centres sit at whole numbers.
fn Vec3 VoxelGrid.to_grid(&self, Vec3 point) @inline => (point - self.origin) / self.cell;

// The voxel whose cell contains `point`, not clamped to the grid.
fn int[<3>] VoxelGrid.voxel_of(&self, Vec3 point) @inline
{
	Vec3 g = self.to_grid(point);
	return { (int)math::floor(g.x + 0.5f), (int)math::floor(g.y + 0.5f), (int)math::floor(g.z + 0.5f) };
}

// Lower voxel of the 2x2x2 block around `point`, and the position inside it
// (0..1 per axis), both clamped to the grid.
fn int[<3>] VoxelGrid.block_of(&self, Vec3 point, Vec3* frac) @inline
{
	Vec3 g = self.to_grid(point);
	int[<3>] base = {
		math::clamp((int)math::floor(g.x), 0, self.nx - 2),
		math::clamp((int)math::floor(g.y), 0, self.ny - 2),
		math::clamp((int)math::floor(g.z), 0, self.nz - 2),
	};
	*frac = {
		math::clamp(g.x - (float)base[0], 0.0f, 1.0f),
		math::clamp(g.y - (float)base[1], 0.0f, 1.0f),
		math::clamp(g.z - (float)base[2], 0.0f, 1.0f),
	};
	return base;
}

// The entries at the eight corners around `point` that lie inside the mesh,
// with their trilinear weights; returns how many. Corners with no weight are
// skipped, so a point exactly on a voxel centre reads one entry.
fn usz VoxelGrid.corners(&self, Vec3 point, int[8]* entry, float[8]* weight)
{
	Vec3 f;
	int[<3>] base = self.block_of(point, &f);

	usz count = 0;
	for (int cz = 0; cz < 2; cz++)
	{
		float wz = cz == 1 ? f.z : 1.0f - f.z;
		for (int cy = 0; cy < 2; cy++)
		{
			float wy = cy == 1 ? f.y : 1.0f - f.y;
			for (int cx = 0; cx < 2; cx++)
			{
				float w = wz * wy * (cx == 1 ? f.x : 1.0f - f.x);
				if (w <= 0.0f) continue;

				int e = self.index[self.at(base[0] + cx, base[1] + cy, base[2] + cz)];
				if (e < 0) continue;

				(*entry)[count] = e;
				(*weight)[count] = w;
				count++;
			}
		}
	}
	return count;
}

<*
 Shell search shared by the `nearest_*` queries: the cell `point` falls in, then
 rings of cells out to `reach`; returns the entry nearest `point` in the first ring
 holding an acceptable one, or -1.

 `@accept(entry, ok)` sets `*ok`. It is an out-parameter because `return` in a macro
 body would leave the caller and `continue` would bind to the caller's loop.
*>
macro int VoxelGrid.@nearest_where(&self, Vec3 point, int reach; @accept(entry, ok))
{
	int[<3>] c = self.voxel_of(point);
	int x = math::clamp(c[0], 0, self.nx - 1);
	int y = math::clamp(c[1], 0, self.ny - 1);
	int z = math::clamp(c[2], 0, self.nz - 1);

	bool accepted = false;
	int direct = self.index[self.at(x, y, z)];
	if (direct >= 0)
	{
		@accept(direct, &accepted);
		if (accepted) return direct;
	}

	int best = -1;
	float nearest = float::max;

	for (int radius = 1; radius <= reach; radius++)
	{
		for (int cz = math::max(z - radius, 0); cz <= math::min(z + radius, self.nz - 1); cz++)
		{
			for (int cy = math::max(y - radius, 0); cy <= math::min(y + radius, self.ny - 1); cy++)
			{
				for (int cx = math::max(x - radius, 0); cx <= math::min(x + radius, self.nx - 1); cx++)
				{
					// Only the new shell: the inside was already searched and found empty. Keeps the
					// cost a cube of the reach rather than a fourth power.
					if (math::abs(cx - x) < radius &&
						math::abs(cy - y) < radius &&
						math::abs(cz - z) < radius) continue;

					int voxel = self.at(cx, cy, cz);
					int entry = self.index[voxel];
					if (entry < 0) continue;
					@accept(entry, &accepted);
					if (!accepted) continue;

					float distance = (self.center(voxel) - point).length();
					if (distance < nearest)
					{
						nearest = distance;
						best = entry;
					}
				}
			}
		}
		if (best >= 0) return best;
	}

	return -1;
}

// Whether `point` is in solid material: one lookup, no casts. Accurate to half a cell;
// nearer the surface than that is a coin flip. A point outside the grid is outside
// the mesh, so this is safe on a grid that was never built.
fn bool VoxelGrid.inside(&self, Vec3 point)
{
	if (self.cells.len == 0) return false;

	int[<3>] v = self.voxel_of(point);

	if (v[0] < 0 || v[1] < 0 || v[2] < 0) return false;
	if (v[0] >= self.nx || v[1] >= self.ny || v[2] >= self.nz) return false;

	return self.index[self.at(v[0], v[1], v[2])] >= 0;
}

// The entry nearest `point`, or -1 when the mesh is not close.
//
// The point's own cell is tried first, then the shells around it: a point on the
// surface often lands in a cell the parity test called empty. A wider `reach` pulls
// mid-air points onto the body but raises the odds of hitting some other part.
//
// @param reach : `Shells to search outward, in cells. Cost grows with its cube.`
fn int VoxelGrid.nearest_cell(&self, Vec3 point, int reach = VOXEL_PAD)
{
	return self.@nearest_where(point, reach; int entry, bool* ok)
	{
		*ok = true;
	};
}

// The entry nearest `point` that `field` has actually been solved for, or -1 when no
// shell out to `reach` holds one. Differs from `nearest_cell` where the interior comes
// apart: a spine thinner than a cell voxelizes as islands that a flood seeded on the
// body never enters. Same shell search, same warning about wide reach.
//
// @param field : `Solved by distance_from_segment. Entries at VOXEL_UNREACHED are treated as absent.`
// @param reach : `Shells to search outward, in cells. Cost grows with its cube.`
fn int VoxelGrid.nearest_solved(&self, float[] field, Vec3 point, int reach = VOXEL_PAD)
{
	if (field.len != self.cells.len) return -1;

	return self.@nearest_where(point, reach; int entry, bool* ok)
	{
		*ok = field[entry] < VOXEL_UNREACHED;
	};
}

// One entry per point, for callers that query the same set repeatedly (the shell
// search is too slow to repeat per frame). The caller owns the returned array.
fn int[] VoxelGrid.map_points(&self, Vec3[] points)
{
	int[] entries = mem::new_array(int, (sz)points.len);
	foreach (usz i, point : points) entries[i] = self.nearest_cell(point);
	return entries;
}

// Voxelize the inside of a closed triangle mesh.
//
// By parity: a line through a closed surface crosses it an even number of times and
// the spans between crossings 1-2, 3-4, ... are inside. Crossings come from marching
// the mesh's BVH. Three axis directions vote and two of three wins, which tolerates
// open, self-intersecting or flipped-face meshes where one direction is wrong.
//
// The grid retains nothing, so `shared_bvh` only has to outlive this call. The caller
// owns the result.
fn VoxelGrid create_voxel_grid(
	Vec3[] vertices,
	Triangle[] triangles,
	uint resolution = DEFAULT_VOXEL_RESOLUTION,
	TriBVH* shared_bvh = null)
{
	if (vertices.len == 0 || triangles.len == 0 || resolution < 4) return {};

	VoxelGrid frame = voxel_frame(vertices, triangles, resolution);
	if (frame.cell <= 0.0f) return {};

	usz total = (usz)frame.nx * (usz)frame.ny * (usz)frame.nz;
	char[] votes = mem::new_array(char, (sz)total);
	defer mem::free(votes);

	vote_interior(&frame, vertices, triangles, shared_bvh, votes);
	return compact_interior(frame, votes);
}

// Origin, cell size and dimensions for a mesh at `resolution`; `cell` is 0 when
// the mesh has no extent. No cell data yet.
fn VoxelGrid voxel_frame(Vec3[] vertices, Triangle[] triangles, uint resolution) @local
{
	// Measured over the triangles rather than the vertex array: an unreferenced
	// vertex parked far away would otherwise stretch the grid over empty space.
	Aabb3 bounds = NEW_AABB3;
	foreach (triangle : triangles)
	{
		bounds.expand_by_point(vertices[triangle.x]);
		bounds.expand_by_point(vertices[triangle.y]);
		bounds.expand_by_point(vertices[triangle.z]);
	}

	Vec3 extent = bounds.size();
	float longest = extent.max();
	if (!(longest > 0.0f)) return {};

	float cell = longest / (float)resolution;

	// Sample lines run through cell centres, and round-number models put edges exactly
	// there; a line through a shared edge counts once and inverts the parity behind it.
	// The origin is nudged off those coincidences.
	float margin = cell * (float)VOXEL_PAD;
	Vec3 origin = bounds.min - margin + VOXEL_ORIGIN_JITTER * cell;

	return {
		.origin = origin,
		.cell = cell,
		.nx = (int)math::ceil(extent.x / cell) + 2 * VOXEL_PAD + 1,
		.ny = (int)math::ceil(extent.y / cell) + 2 * VOXEL_PAD + 1,
		.nz = (int)math::ceil(extent.z / cell) + 2 * VOXEL_PAD + 1,
	};
}

// Count, per voxel, how many of the three axis directions found it inside.
fn void vote_interior(
	VoxelGrid* frame,
	Vec3[] vertices,
	Triangle[] triangles,
	TriBVH* shared_bvh,
	char[] votes) @local
{
	// Reuse a BVH the caller already built over exactly these arrays; building another is
	// this function's most expensive step. The pointers are checked, not trusted: a BVH
	// over a different mesh would silently voxelize the wrong body, so a mismatch falls
	// back to building one.
	bool own_bvh = shared_bvh == null
		|| shared_bvh.vertices != vertices.ptr
		|| shared_bvh.triangles != triangles.ptr;

	TriBVH* bvh = own_bvh
		? create_tri_bvh(vertices.ptr, triangles.ptr, (sz)triangles.len)
		: shared_bvh;
	defer if (own_bvh) bvh.free();

	march_votes(frame, bvh, votes);
}

// Cast a line along each axis through every cell column and vote on the spans
// between paired surface crossings.
fn void march_votes(VoxelGrid* frame, TriBVH* bvh, char[] votes) @local
{
	int[3] dims = { frame.nx, frame.ny, frame.nz };
	float cell = frame.cell;
	Vec3 origin = frame.origin;

	float step = cell * VOXEL_CROSSING_STEP;
	float[MAX_VOXEL_CROSSINGS] crossings;

	for (int axis = 0; axis < 3; axis++)
	{
		int u = (axis + 1) % 3;
		int v = (axis + 2) % 3;

		Vec3 direction = { 0, 0, 0 };
		direction[axis] = 1.0f;

		for (int iu = 0; iu < dims[u]; iu++)
		{
			for (int iv = 0; iv < dims[v]; iv++)
			{
				Vec3 start = origin;
				start[u] = origin[u] + (float)iu * cell;
				start[v] = origin[v] + (float)iv * cell;
				// Half a cell before the first centre, so the distance from
				// here to cell k's centre is exactly (k + 0.5) cells.
				start[axis] = origin[axis] - cell * 0.5f;

				usz found = 0;
				float travelled = 0.0f;
				while (found < MAX_VOXEL_CROSSINGS)
				{
					Ray ray = { .origin = start + direction * travelled, .direction = direction };
					HitInfo hit = bvh.ray_intersects(ray);
					if (!hit.collided) break;

					travelled += hit.depth;
					crossings[found++] = travelled;
					travelled += step;
				}

				// An odd count means the surface did not close along this line. The unpaired last
				// crossing is dropped, not the whole line; the vote covers the rest.
				usz pairs = found / 2;
				for (usz p = 0; p < pairs; p++)
				{
					int lo = (int)math::ceil(crossings[p * 2] / cell - 0.5f);
					int hi = (int)math::floor(crossings[p * 2 + 1] / cell - 0.5f);

					// Clamped inside the padding, so every interior cell keeps all 26 neighbours in range.
					lo = math::max(lo, VOXEL_PAD - 1);
					hi = math::min(hi, dims[axis] - VOXEL_PAD);

					for (int k = lo; k <= hi; k++)
					{
						int[3] ci;
						ci[axis] = k;
						ci[u] = iu;
						ci[v] = iv;
						votes[(usz)frame.at(ci[0], ci[1], ci[2])]++;
					}
				}
			}
		}
	}
}

// Keep the voxels two of three directions voted inside, as the compact
// index/cells pair. Returns an unusable grid when nothing is inside.
fn VoxelGrid compact_interior(VoxelGrid frame, char[] votes) @local
{
	usz interior = 0;
	foreach (vote : votes) if (vote >= VOXEL_VOTES_NEEDED) interior++;
	if (interior == 0) return {};

	VoxelGrid grid = frame;
	grid.index = mem::new_array(int, (sz)votes.len);
	grid.cells = mem::new_array(int, (sz)interior);
	grid.stencil = make_chamfer_stencil(grid.nx, grid.ny, grid.cell);

	usz next = 0;
	for (usz i = 0; i < votes.len; i++)
	{
		if (votes[i] >= VOXEL_VOTES_NEEDED)
		{
			grid.index[i] = (int)next;
			grid.cells[next++] = (int)i;
			continue;
		}
		grid.index[i] = -1;
	}

	return grid;
}

// ============================================================================
// Distance fields
//
// A field is one float per interior cell, so it is always `grid.cells.len`
// long. Seed it, solve it, then read it back at whatever points you like.
// Seeding more than once before solving gives a multi-source field — the
// distance to the *nearest* seed — which is what a flow field wants.
// ============================================================================

fn void VoxelGrid.clear_field(&self, float[] field)
{
	foreach (&distance : field) *distance = VOXEL_UNREACHED;
}

// Put a zero on every interior cell the segment passes through and return how many;
// zero means the segment touches no inside.
//
// It seeds only the cells the segment passes through while there are any: a zero is
// not an approximation, and reaching sideways would declare a neighbouring finger to
// be at no distance. Reach (second and third pass) applies only when the segment lies
// outside the mesh altogether.
//
// Pass the same point twice to seed a single position.
fn usz VoxelGrid.seed_segment(&self, float[] field, Vec3 head, Vec3 tail)
{
	if (!self.usable() || field.len != self.cells.len) return 0;

	Vec3 along = tail - head;
	// Half a cell between samples, so a segment cannot step over a cell it
	// passes through.
	usz steps = (usz)math::max(2.0f, math::ceil(along.length() / (self.cell * 0.5f)) + 1.0f);

	for (int reach = 0; reach <= VOXEL_PAD; reach++)
	{
		usz seeded = 0;
		for (usz s = 0; s < steps; s++)
		{
			int entry = self.nearest_cell(head + along * ((float)s / (float)(steps - 1)), reach);
			if (entry < 0 || field[entry] == 0.0f) continue;
			field[entry] = 0.0f;
			seeded++;
		}
		if (seeded > 0) return seeded;
	}
	return 0;
}

// Propagate the seeds through the interior with a chamfer transform: each pass walks
// the cells in scan order relaxing against the 13 neighbours already visited, then
// back against the other 13. Stops when a pass changes nothing. Diagonal steps use true
// lengths, so straight runs are overestimated by a few percent, uniformly.
fn void VoxelGrid.solve_field(&self, float[] field)
{
	if (!self.usable() || field.len != self.cells.len) return;

	usz count = self.cells.len;
	bool changed = true;
	for (usz sweep = 0; sweep < MAX_VOXEL_SWEEPS && changed; sweep++)
	{
		changed = false;

		for (usz i = 0; i < count; i++)
		{
			if (relax_voxel(self, field, i, 0, STENCIL_CENTRE)) changed = true;
		}
		for (usz i = count; i > 0; i--)
		{
			if (relax_voxel(self, field, i - 1, STENCIL_CENTRE + 1, STENCIL_SIZE)) changed = true;
		}
	}
}

// Clear, seed and solve in one call — the common case.
// False means the segment touched no inside, which leaves the field unusable.
fn bool VoxelGrid.distance_from_segment(&self, Vec3 head, Vec3 tail, float[] field)
{
	if (!self.usable() || field.len != self.cells.len) return false;

	self.clear_field(field);
	if (self.seed_segment(field, head, tail) == 0) return false;
	self.solve_field(field);
	return true;
}

// One cell against half of its neighbourhood (shared by both sweep directions). No
// bounds test: the padding keeps all 26 steps inside the grid.
fn bool relax_voxel(VoxelGrid* grid, float[] field, usz entry, usz first, usz last) @local
{
	int voxel = grid.cells[entry];
	float best = field[entry];

	for (usz k = first; k < last; k++)
	{
		int neighbour = grid.index[voxel + grid.stencil.offset[k]];
		if (neighbour < 0) continue;

		float distance = field[neighbour];
		if (distance >= VOXEL_UNREACHED) continue;

		distance += grid.stencil.cost[k];
		if (distance < best) best = distance;
	}

	if (best >= field[entry]) return false;
	field[entry] = best;
	return true;
}

// Read a field at a point by interpolating the eight cells around it, skipping corners
// outside the mesh or never reached, so the result is not banded at cell resolution.
// `fallback` is an entry to use when no corner is usable (e.g. from `nearest_cell`),
// or -1 for none.
fn float VoxelGrid.sample(&self, float[] field, Vec3 point, int fallback = -1)
{
	int[8] entry;
	float[8] weight;
	usz count = self.corners(point, &entry, &weight);

	float total = 0.0f;
	float sum = 0.0f;
	for (usz c = 0; c < count; c++)
	{
		float distance = field[entry[c]];
		if (distance >= VOXEL_UNREACHED) continue;

		sum += distance * weight[c];
		total += weight[c];
	}

	if (total > 0.0f) return sum / total;
	if (fallback < 0) return VOXEL_UNREACHED;
	return field[fallback];
}

// ============================================================================
// Many sources at once
//
// Solves every source in one pass instead of one sweep per source: each cell keeps
// only its nearest few sources. The trade is that a source outside a cell's nearest
// few is unknown there, which suits callers that keep a handful of influences anyway.
//
//     VoxelSource[] field = mem::new_array(VoxelSource, grid.cells.len * slots);
//     grid.clear_sources(field);
//     foreach (usz b, bone : bones) grid.seed_source(field, slots, (int)b, bone.head, bone.tail);
//     grid.solve_sources(field, slots);
// ============================================================================

// An empty slot, and the id no source may use.
const int NO_VOXEL_SOURCE = -1;

// The most slots a cell may keep. A guard against passing the source count instead of
// the slot count, which would size the field in gigabytes.
const usz MAX_VOXEL_SOURCES = 16;

// One source reaching one cell, and how far through the body it came. A cell's slots
// run ascending by distance with empty ones last, so slot zero is the nearest.
struct VoxelSource
{
	int source;
	float distance;
}

fn void VoxelGrid.clear_sources(&self, VoxelSource[] field)
{
	foreach (&slot : field) *slot = { .source = NO_VOXEL_SOURCE, .distance = VOXEL_UNREACHED };
}

// Put `source` at zero on every interior cell the segment passes through, and
// report how many it reached. The reach rules are `seed_segment`'s, and for the
// same reason.
fn usz VoxelGrid.seed_source(&self, VoxelSource[] field, usz slots, int source, Vec3 head, Vec3 tail)
{
	if (!self.usable() || slots == 0 || field.len != self.cells.len * slots) return 0;
	if (source < 0) return 0;

	Vec3 along = tail - head;
	usz steps = (usz)math::max(2.0f, math::ceil(along.length() / (self.cell * 0.5f)) + 1.0f);

	for (int reach = 0; reach <= VOXEL_PAD; reach++)
	{
		usz seeded = 0;
		for (usz s = 0; s < steps; s++)
		{
			int entry = self.nearest_cell(head + along * ((float)s / (float)(steps - 1)), reach);
			if (entry < 0) continue;

			VoxelSource[] cell = field[(usz)entry * slots : slots];
			offer_source(cell, source, 0.0f);

			// Counted by asking the cell, not by whether the offer changed it: a bone landing on a
			// cell another bone already zeroed sees no change but still touched the inside.
			if (cell[0].source != NO_VOXEL_SOURCE && cell[0].distance <= 0.0f)
			{
				for (usz k = 0; k < slots; k++)
				{
					if (cell[k].distance > 0.0f || cell[k].source == NO_VOXEL_SOURCE) break;
					if (cell[k].source == source) { seeded++; break; }
				}
			}
		}
		if (seeded > 0) return seeded;
	}
	return 0;
}

// Propagate every source through the interior at once: `solve_field`'s chamfer sweep
// with a list per cell. A source evicted from a cell's list stops spreading there,
// which keeps this to a few sweeps and matches the answer the caller keeps.
fn void VoxelGrid.solve_sources(&self, VoxelSource[] field, usz slots)
{
	if (!self.usable() || slots == 0 || slots > MAX_VOXEL_SOURCES) return;
	if (field.len != self.cells.len * slots) return;

	usz count = self.cells.len;
	bool changed = true;
	for (usz sweep = 0; sweep < MAX_VOXEL_SWEEPS && changed; sweep++)
	{
		changed = false;

		for (usz i = 0; i < count; i++)
		{
			if (relax_sources(self, field, slots, i, 0, STENCIL_CENTRE)) changed = true;
		}
		for (usz i = count; i > 0; i--)
		{
			if (relax_sources(self, field, slots, i - 1, STENCIL_CENTRE + 1, STENCIL_SIZE)) changed = true;
		}
	}
}

// One cell's list against half of its neighbourhood (shared by both sweep directions).
fn bool relax_sources(
	VoxelGrid* grid,
	VoxelSource[] field,
	usz slots,
	usz entry,
	usz first,
	usz last) @local
{
	int voxel = grid.cells[entry];
	VoxelSource[] here = field[entry * slots : slots];
	bool moved = false;

	// Once a list is full nothing further than its worst entry can enter it, so one compare
	// against a neighbour's nearest source can reject all of its slots. Deep inside a body
	// this keeps the cost near the single-source sweep.
	bool full = here[slots - 1].source != NO_VOXEL_SOURCE;
	float worst = here[slots - 1].distance;

	for (usz k = first; k < last; k++)
	{
		int neighbour = grid.index[voxel + grid.stencil.offset[k]];
		if (neighbour < 0) continue;

		VoxelSource[] there = field[(usz)neighbour * slots : slots];
		if (there[0].source == NO_VOXEL_SOURCE) continue;
		if (full && there[0].distance + grid.stencil.cost[k] >= worst) continue;

		for (usz s = 0; s < slots; s++)
		{
			// Ascending with the empties last, so the first empty ends the list.
			if (there[s].source == NO_VOXEL_SOURCE) break;
			if (!offer_source(here, there[s].source, there[s].distance + grid.stencil.cost[k])) continue;

			moved = true;
			full = here[slots - 1].source != NO_VOXEL_SOURCE;
			worst = here[slots - 1].distance;
		}
	}
	return moved;
}

// Offer one source at one distance to a cell's list, keeping it ascending and
// one entry per source. False when the list already knew as much or better.
fn bool offer_source(VoxelSource[] cell, int source, float distance) @local
{
	usz slots = cell.len;

	// Nothing to do if the list is full of nearer sources (the common case).
	if (cell[slots - 1].source != NO_VOXEL_SOURCE && distance >= cell[slots - 1].distance) return false;

	// Where it goes, and whether it is already in here somewhere.
	usz at = slots;
	usz was = slots;
	for (usz s = 0; s < slots; s++)
	{
		if (cell[s].source == source)
		{
			if (cell[s].distance <= distance) return false;
			was = s;
			break;
		}
	}
	for (usz s = 0; s < slots; s++)
	{
		if (cell[s].source == NO_VOXEL_SOURCE || distance < cell[s].distance) { at = s; break; }
	}
	if (at >= slots) return false;

	// Shift down to `at`, stopping at the old copy of this source if present (which removes
	// it and leaves room to write).
	usz from = was < slots ? was : slots - 1;
	for (usz s = from; s > at; s--) cell[s] = cell[s - 1];
	cell[at] = { .source = source, .distance = distance };
	return true;
}

// What one cell knows about one source, or VOXEL_UNREACHED when that source is
// not among its nearest.
fn float VoxelGrid.source_at(&self, VoxelSource[] field, usz slots, int entry, int source)
{
	if (entry < 0) return VOXEL_UNREACHED;

	VoxelSource[] cell = field[(usz)entry * slots : slots];
	for (usz s = 0; s < slots; s++)
	{
		if (cell[s].source == NO_VOXEL_SOURCE) break;
		if (cell[s].source == source) return cell[s].distance;
	}
	return VOXEL_UNREACHED;
}

// The nearest cell to `point` that any source reached, or -1 within `reach`.
// `nearest_solved` for a many-source field.
fn int VoxelGrid.nearest_sourced(&self, VoxelSource[] field, usz slots, Vec3 point, int reach = VOXEL_PAD)
{
	if (slots == 0 || field.len != self.cells.len * slots) return -1;

	return self.@nearest_where(point, reach; int entry, bool* ok)
	{
		*ok = field[(usz)entry * slots].source != NO_VOXEL_SOURCE;
	};
}

// Read a many-source field at a point into `into`: `sample`'s interpolation for every
// source at once, skipping corners outside the mesh or reached by no source.
//
// `home` decides which sources are asked about and the corners only how far away they
// are, so a source ruled out for the point's own cell is never admitted via a corner.
//
// @param home : `The cell the point is in, from nearest_cell. Negative asks about nothing.`
// @param into : `Filled with what each of home's sources reads here, ascending by distance and terminated by an empty slot when there is room. Sized by the caller; slots beyond its length are dropped.`
// @return `How many entries were written.`
fn usz VoxelGrid.sample_sources(
	&self,
	VoxelSource[] field,
	usz slots,
	Vec3 point,
	int home,
	VoxelSource[] into)
{
	if (into.len == 0) return 0;
	if (home < 0 || slots == 0 || slots > MAX_VOXEL_SOURCES) return 0;
	if (field.len != self.cells.len * slots) return 0;

	// The corners once, so the per-source loop below is eight array reads rather
	// than eight index computations.
	int[8] corner;
	float[8] share;
	usz corners = self.corners(point, &corner, &share);

	VoxelSource[] here = field[(usz)home * slots : slots];
	usz written = 0;

	for (usz s = 0; s < slots && written < into.len; s++)
	{
		int source = here[s].source;
		if (source == NO_VOXEL_SOURCE) break;

		float sum = 0.0f;
		float total = 0.0f;
		for (usz c = 0; c < corners; c++)
		{
			float distance = self.source_at(field, slots, corner[c], source);
			if (distance >= VOXEL_UNREACHED) continue;
			sum += distance * share[c];
			total += share[c];
		}

		// The home cell always has an answer for its own sources, so the average
		// falls back on it rather than on nothing.
		into[written++] = {
			.source = source,
			.distance = total > 0.0f ? sum / total : here[s].distance,
		};
	}

	if (written < into.len) into[written] = { .source = NO_VOXEL_SOURCE, .distance = VOXEL_UNREACHED };
	return written;
}

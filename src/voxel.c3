module collision;

import std::math;
import std::core::mem;

// ============================================================================
// Solid voxelization and distance fields through the inside of a mesh
//
// Turns a closed triangle mesh into the set of cells that are *inside* it, then
// measures distance through that set rather than across open space. Two points
// on either side of a gap are far apart here even when a straight line between
// them is short, which is the property that separates limbs, keeps a flow field
// from routing through a wall, and stops influence leaking between rooms.
//
//     VoxelGrid grid = create_voxel_grid(vertices, triangles);
//     defer grid.free();
//
//     float[] field = mem::new_array(float, grid.cells.len);
//     grid.distance_from_segment(start, end, field);
//     float d = grid.sample(field, point);
//
// Resolution is set on the model's longest side, not on its triangle count, so
// a 200k-triangle scan and a 1k-triangle proxy of the same shape produce the
// same grid and cost the same to solve. What resolution buys is the thinnest
// feature that survives: anything narrower than a cell may not fill, and a
// passage that does not fill is one no path travels down.
// ============================================================================

// Cells along the model's longest side.
const uint DEFAULT_VOXEL_RESOLUTION = 48;

// Empty cells kept around the model on every side. Two is what makes the sweeps
// safe: no interior cell can sit on the grid boundary, so stepping to any of a
// cell's 26 neighbours stays in range and inside the same row, and the inner
// loop needs no bounds test at all.
const int VOXEL_PAD = 2;

// How far a marching ray steps past a surface before looking for the next one,
// as a fraction of a cell. Two surfaces closer together than this merge into one
// crossing and the parity inverts for the rest of that line.
const float VOXEL_CROSSING_STEP = 0.02;

// Crossings counted along one line before it is abandoned. A closed mesh gives a
// handful; finding out costs a ray cast each.
const usz MAX_VOXEL_CROSSINGS = 64;

// The distance reported for a cell no path reached. Compare against it before
// adding anything to a value read out of a field.
const float VOXEL_UNREACHED = float::max;

// Sweeps run before a field is accepted as it stands. Each pass propagates
// distance along the scan order and back, so a path that doubles back needs one
// pair per reversal. The loop stops as soon as a pass changes nothing.
const usz MAX_VOXEL_SWEEPS = 12;

// Which cells of a box around the mesh are inside it, and how they connect.
//
// `index` and `cells` are two directions of one mapping: `index` answers "is
// this voxel inside, and where is it in the compact list", `cells` answers
// "which voxel is this entry". Fields are sized by `cells`, so solving costs
// interior volume rather than bounding-box volume — on a limbed shape that is a
// fifth of the box.
struct VoxelGrid
{
	Vec3 origin;   // the centre of voxel (0, 0, 0)
	float cell;
	int nx, ny, nz;

	int[] index;   // voxel index -> entry in `cells`, or -1 outside the mesh
	int[] cells;   // entry -> voxel index, ascending
}

// False for a mesh that is open everywhere, flat, or smaller than one cell.
// Callers should treat that as "no grid" rather than as an error.
fn bool VoxelGrid.usable(self) => self.cells.len > 0;

fn void VoxelGrid.free(&self)
{
	if (self.index.len > 0) mem::free(self.index);
	if (self.cells.len > 0) mem::free(self.cells);
	*self = {};
}

fn int VoxelGrid.at(self, int x, int y, int z) @inline => (z * self.ny + y) * self.nx + x;

fn Vec3 VoxelGrid.center(self, int voxel)
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

// The entry nearest `point`, or -1 when the mesh is not close.
//
// The cell the point falls in first, then the shells around it: a point on the
// surface sits exactly where the inside stops, so half of them land in a cell
// the parity test called empty and have to reach in one step. The search stops
// at VOXEL_PAD, past which the nearest inside is some other part of the model.
fn int VoxelGrid.nearest_cell(&self, Vec3 point)
{
	int x = (int)math::floor((point.x - self.origin.x) / self.cell + 0.5f);
	int y = (int)math::floor((point.y - self.origin.y) / self.cell + 0.5f);
	int z = (int)math::floor((point.z - self.origin.z) / self.cell + 0.5f);

	x = math::clamp(x, 0, self.nx - 1);
	y = math::clamp(y, 0, self.ny - 1);
	z = math::clamp(z, 0, self.nz - 1);

	int direct = self.index[self.at(x, y, z)];
	if (direct >= 0) return direct;

	int best = -1;
	float nearest = float::max;

	for (int radius = 1; radius <= VOXEL_PAD; radius++)
	{
		for (int cz = math::max(z - radius, 0); cz <= math::min(z + radius, self.nz - 1); cz++)
		{
			for (int cy = math::max(y - radius, 0); cy <= math::min(y + radius, self.ny - 1); cy++)
			{
				for (int cx = math::max(x - radius, 0); cx <= math::min(x + radius, self.nx - 1); cx++)
				{
					int voxel = self.at(cx, cy, cz);
					int entry = self.index[voxel];
					if (entry < 0) continue;

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

// One entry per point, for callers that query the same set repeatedly — the
// shell search above is not cheap enough to repeat per frame. The caller owns
// the returned array.
fn int[] VoxelGrid.map_points(&self, Vec3[] points)
{
	int[] entries = mem::new_array(int, (sz)points.len);
	foreach (usz i, point : points) entries[i] = self.nearest_cell(point);
	return entries;
}

// Voxelize the inside of a closed triangle mesh.
//
// Solid voxelization by parity: a line through a closed surface crosses it an
// even number of times, and the spans between the first and second crossing,
// the third and fourth, and so on, are inside. Crossings come from marching the
// mesh's own BVH — a cast, a step past what it hit, another cast — so the test
// is the same intersection routine everything else here uses.
//
// Three directions vote and two out of three wins. On a watertight mesh all
// three agree and the vote costs only time; what it buys is the meshes that
// turn up in practice — open, self-intersecting, or with a stray flipped face —
// where one direction can be wrong for a whole line and the other two are not.
//
// The grid borrows nothing: the BVH is built and freed here. The caller owns
// the result.
fn VoxelGrid create_voxel_grid(Vec3[] vertices, Triangle[] triangles, uint resolution = DEFAULT_VOXEL_RESOLUTION)
{
	if (vertices.len == 0 || triangles.len == 0 || resolution < 4) return {};

	// Measured over the triangles rather than the vertex array: an unreferenced
	// vertex parked far away would otherwise stretch the grid over empty space
	// and cost resolution everywhere that matters.
	Aabb3 bounds = NEW_AABB3;
	foreach (triangle : triangles)
	{
		bounds.expand_by_point(vertices[triangle.x]);
		bounds.expand_by_point(vertices[triangle.y]);
		bounds.expand_by_point(vertices[triangle.z]);
	}

	Vec3 extent = bounds.size();
	float longest = math::max(extent.x, math::max(extent.y, extent.z));
	if (!(longest > 0.0f)) return {};

	float cell = longest / (float)resolution;

	// Sample lines run through cell centres, and a model built on round numbers
	// puts its edges exactly there. A line through a shared edge is counted once
	// instead of twice and inverts the parity for everything behind it — a
	// fraction of a cell moves the lines off those coincidences.
	float margin = cell * (float)VOXEL_PAD;
	Vec3 origin = {
		bounds.min.x - margin + cell * 0.0137f,
		bounds.min.y - margin + cell * 0.0219f,
		bounds.min.z - margin + cell * 0.0311f,
	};

	int[3] dims = {
		(int)math::ceil(extent.x / cell) + 2 * VOXEL_PAD + 1,
		(int)math::ceil(extent.y / cell) + 2 * VOXEL_PAD + 1,
		(int)math::ceil(extent.z / cell) + 2 * VOXEL_PAD + 1,
	};

	usz total = (usz)dims[0] * (usz)dims[1] * (usz)dims[2];
	char[] votes = mem::new_array(char, (sz)total);
	defer mem::free(votes);

	TriBVHNode* bvh = create_indexed_tri_bvh(vertices.ptr, triangles.ptr, (sz)triangles.len);
	defer
	{
		bvh.free_indexed();
		mem::free(bvh);
	}

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

				// An odd count means the surface did not close along this line —
				// a hole, or two faces the step above merged. The unpaired last
				// crossing is dropped rather than the whole line: the spans
				// before it are still good, and the vote covers the rest.
				usz pairs = found / 2;
				for (usz p = 0; p < pairs; p++)
				{
					int lo = (int)math::ceil(crossings[p * 2] / cell - 0.5f);
					int hi = (int)math::floor(crossings[p * 2 + 1] / cell - 0.5f);

					// Clamped inside the padding rather than inside the grid,
					// which is what guarantees every interior cell has all 26
					// neighbours in range.
					lo = math::max(lo, VOXEL_PAD - 1);
					hi = math::min(hi, dims[axis] - VOXEL_PAD);

					for (int k = lo; k <= hi; k++)
					{
						int[3] cell_index;
						cell_index[axis] = k;
						cell_index[u] = iu;
						cell_index[v] = iv;
						votes[(usz)((cell_index[2] * dims[1] + cell_index[1]) * dims[0] + cell_index[0])]++;
					}
				}
			}
		}
	}

	usz interior = 0;
	foreach (vote : votes) if (vote >= 2) interior++;
	if (interior == 0) return {};

	VoxelGrid grid = {
		.origin = origin,
		.cell = cell,
		.nx = dims[0],
		.ny = dims[1],
		.nz = dims[2],
		.index = mem::new_array(int, (sz)total),
		.cells = mem::new_array(int, (sz)interior),
	};

	usz next = 0;
	for (usz i = 0; i < total; i++)
	{
		if (votes[i] >= 2)
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

// Put a zero on every interior cell the segment passes through, and report how
// many it found. Zero means the segment touches no inside at all, which the
// caller usually wants to distinguish from "far away".
//
// Pass the same point twice to seed a single position.
fn usz VoxelGrid.seed_segment(&self, float[] field, Vec3 head, Vec3 tail)
{
	if (!self.usable() || field.len != self.cells.len) return 0;

	Vec3 along = tail - head;
	// Half a cell between samples, so a segment cannot step over a cell it
	// passes through.
	usz steps = (usz)math::max(2.0f, math::ceil(along.length() / (self.cell * 0.5f)) + 1.0f);

	usz seeded = 0;
	for (usz s = 0; s < steps; s++)
	{
		int entry = self.nearest_cell(head + along * ((float)s / (float)(steps - 1)));
		if (entry < 0 || field[entry] == 0.0f) continue;
		field[entry] = 0.0f;
		seeded++;
	}
	return seeded;
}

// Propagate the seeds through the interior.
//
// A chamfer transform rather than a queue: each pass walks the cells in scan
// order relaxing against the 13 neighbours already visited, then back the other
// way against the other 13. Two passes solve any path that only goes one way; a
// path that doubles back needs one pair per reversal, and the loop stops when a
// pass changes nothing.
//
// The 26-neighbour step lengths are the true diagonals, so a straight run is
// overestimated by a few percent — the same few percent everywhere, which is
// what matters when distances are compared to each other.
fn void VoxelGrid.solve_field(&self, float[] field)
{
	if (!self.usable() || field.len != self.cells.len) return;

	// The 3x3x3 neighbourhood in scan order, centre included at 13 so the halves
	// fall out of the index: everything before it is a neighbour a forward pass
	// has already visited, everything after it one a backward pass has.
	int plane = self.nx * self.ny;
	int[27] offset;
	float[27] cost;
	usz slot = 0;
	for (int dz = -1; dz <= 1; dz++)
	{
		for (int dy = -1; dy <= 1; dy++)
		{
			for (int dx = -1; dx <= 1; dx++)
			{
				offset[slot] = dz * plane + dy * self.nx + dx;
				cost[slot] = self.cell * math::sqrt((float)(dx * dx + dy * dy + dz * dz));
				slot++;
			}
		}
	}

	usz count = self.cells.len;
	bool changed = true;
	for (usz sweep = 0; sweep < MAX_VOXEL_SWEEPS && changed; sweep++)
	{
		changed = false;

		for (usz i = 0; i < count; i++)
		{
			if (relax_voxel(self, field, i, &offset, &cost, 0, 13)) changed = true;
		}
		for (usz i = count; i > 0; i--)
		{
			if (relax_voxel(self, field, i - 1, &offset, &cost, 14, 27)) changed = true;
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

// One cell against half of its neighbourhood. Split out of the sweep only so the
// two directions cannot drift apart.
//
// No bounds test on the stepped index: the padding puts every interior cell at
// least two cells from the edge of the grid, so all 26 steps land inside it.
fn bool relax_voxel(VoxelGrid* grid, float[] field, usz entry, int[27]* offset, float[27]* cost, usz first, usz last) @local
{
	int voxel = grid.cells[entry];
	float best = field[entry];

	for (usz k = first; k < last; k++)
	{
		int neighbour = grid.index[voxel + (*offset)[k]];
		if (neighbour < 0) continue;

		float distance = field[neighbour];
		if (distance >= VOXEL_UNREACHED) continue;

		distance += (*cost)[k];
		if (distance < best) best = distance;
	}

	if (best >= field[entry]) return false;
	field[entry] = best;
	return true;
}

// Read a field at a point rather than at a cell.
//
// A straight lookup bands the result at the grid's own resolution — every point
// in a cell gets the identical value. Interpolating over the eight cells around
// the point smooths that out for eight lookups, skipping any corner that is
// outside the mesh or that no path reached, so the average never leans on a cell
// with no answer.
//
// `fallback` is an entry to fall back on when no corner is usable — pass the one
// `nearest_cell` gave for this point, or -1 for none.
fn float VoxelGrid.sample(&self, float[] field, Vec3 point, int fallback = -1)
{
	float gx = (point.x - self.origin.x) / self.cell;
	float gy = (point.y - self.origin.y) / self.cell;
	float gz = (point.z - self.origin.z) / self.cell;

	int bx = math::clamp((int)math::floor(gx), 0, self.nx - 2);
	int by = math::clamp((int)math::floor(gy), 0, self.ny - 2);
	int bz = math::clamp((int)math::floor(gz), 0, self.nz - 2);

	float fx = math::clamp(gx - (float)bx, 0.0f, 1.0f);
	float fy = math::clamp(gy - (float)by, 0.0f, 1.0f);
	float fz = math::clamp(gz - (float)bz, 0.0f, 1.0f);

	float total = 0.0f;
	float sum = 0.0f;

	for (int cz = 0; cz < 2; cz++)
	{
		float wz = cz == 1 ? fz : 1.0f - fz;
		for (int cy = 0; cy < 2; cy++)
		{
			float wy = cy == 1 ? fy : 1.0f - fy;
			for (int cx = 0; cx < 2; cx++)
			{
				float weight = wz * wy * (cx == 1 ? fx : 1.0f - fx);
				if (weight <= 0.0f) continue;

				int entry = self.index[self.at(bx + cx, by + cy, bz + cz)];
				if (entry < 0) continue;

				float distance = field[entry];
				if (distance >= VOXEL_UNREACHED) continue;

				sum += distance * weight;
				total += weight;
			}
		}
	}

	if (total > 0.0f) return sum / total;
	if (fallback < 0) return VOXEL_UNREACHED;
	return field[fallback];
}

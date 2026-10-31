module collision;

import std::math;
import std::collections::list;
import std::collections::map;

// ============================================================================
// Heightmap Terrain - collision against a grid of heights (y up, grid in x/z)
// ============================================================================

alias Heights = List {float};

const float HEIGHTMAP_RAY_STEP_CELLS = 0.5f; // March step along a ray, in cells
const float HEIGHTMAP_RAY_ADAPTIVE_FRACTION = 0.5f; // Fraction of the height above terrain skipped per step
const int HEIGHTMAP_RAY_REFINE_ITERATIONS = 8; // Bisection steps once the ray is under the surface
const float HEIGHTMAP_DEFAULT_MAX_RAY_DISTANCE = 1000.0f;
const float HEIGHTMAP_DEFAULT_RAY_START_HEIGHT = 1000.0f; // Height mesh sampling rays start from
const float CAPSULE_DEGENERATE_LENGTH = 0.001f; // Shorter than this, a capsule is a sphere
const float CAPSULE_SAMPLE_SPACING = 0.5f; // Sphere probes along a capsule, as a fraction of its radius
const float TILE_DEFAULT_LOAD_RADIUS = 200.0f;
const float TILE_DEFAULT_UNLOAD_RADIUS = 250.0f; // Larger than the load radius so tiles do not flicker

<*
 Height and surface normal at one world position. `valid` is false where there
 is no terrain data (a tile that is not loaded); a single `Heightmap` clamps to
 its edge and is always valid.
*>
struct HeightSample
{
	bool valid;
	float height;
	Vec3 normal;
}

<*
 Anything that can be sampled for ground height. `Heightmap` and
 `TiledHeightmap` implement it; collision queries below are written once over it.
*>
interface Terrain
{
	fn float get_height(float wx, float wz);
	fn Vec3 get_normal(float wx, float wz);
	<* Height and normal in one lookup. *>
	fn HeightSample sample(float wx, float wz);
}

struct Heightmap (CollisionShape, Terrain)
{
	Heights heights; // 2D grid of heights (row-major: heights[z * width + x])
	uint width; // Number of samples in X direction
	uint depth; // Number of samples in Z direction
	float cell_size; // World distance between samples
	Vec3 origin; // World position of corner (min X, min Z)
	float min_height; // Cached min height for AABB
	float max_height; // Cached max height for AABB
}

// ============================================================================
// Creation and Destruction
// ============================================================================

<*
 Takes ownership of `heights`, which must be on a real allocator (not temp) so
 `free` can release it.
*>
fn Heightmap create_heightmap_from_list(
	Heights heights,
	uint width,
	uint depth,
	float cell_size,
	Vec3 origin = { 0, 0, 0 }
)
{
	Heightmap hm = { .heights = heights, .width = width, .depth = depth, .cell_size = cell_size, .origin = origin };
	hm.update_height_bounds();
	return hm;
}

fn Heightmap create_heightmap(uint width, uint depth, float cell_size, Vec3 origin = { 0, 0, 0 })
{
	sz size = (sz)width * (sz)depth;

	// Explicit allocator: a zero-valued List would bind to the temp allocator.
	Heights heights;
	heights.init(mem, size);
	for (sz i = 0; i < size; i++) heights.push(0);

	return create_heightmap_from_list(heights, width, depth, cell_size, origin);
}

<*
 @require heights.len >= (sz)width * (sz)depth : `Height data too small for dimensions`
*>
fn Heightmap create_heightmap_from_data(
	float[] heights,
	uint width,
	uint depth,
	float cell_size,
	Vec3 origin = { 0, 0, 0 }
)
{
	sz size = (sz)width * (sz)depth;

	Heights height_list;
	height_list.init(mem, size);
	height_list.push_all(heights[:size]);

	return create_heightmap_from_list(height_list, width, depth, cell_size, origin);
}

fn void Heightmap.free(&self) @dynamic
{
	self.heights.free();
}

// ============================================================================
// Height Access
// ============================================================================

// Get raw height at grid coordinates (no interpolation)
fn float Heightmap.get_height_at_grid(&self, uint gx, uint gz)
{
	if (gx >= self.width || gz >= self.depth) return 0;
	return self.heights[(sz)(gz * self.width + gx)];
}

<*
 Set height at grid coordinates. The cached min/max (used by `get_aabb`) only
 ever widen here, so lowering the old extreme leaves the AABB conservative;
 call `update_height_bounds` to tighten it.
*>
fn void Heightmap.set_height_at_grid(&self, uint gx, uint gz, float height)
{
	if (gx >= self.width || gz >= self.depth) return;

	self.heights[(sz)(gz * self.width + gx)] = height;

	if (height < self.min_height) self.min_height = height;
	if (height > self.max_height) self.max_height = height;
}

// Convert world position to grid coordinates (x, z)
fn float[<2>] Heightmap.world_to_grid(&self, float wx, float wz)
{
	return { (wx - self.origin.x) / self.cell_size, (wz - self.origin.z) / self.cell_size };
}

// Convert grid coordinates to world position (x, z)
fn float[<2>] Heightmap.grid_to_world(&self, float gx, float gz)
{
	return { self.origin.x + gx * self.cell_size, self.origin.z + gz * self.cell_size };
}

// (`math::lerp` is private to the stdlib.)
fn float lerp(float a, float b, float t) @inline @local => a + (b - a) * t;

// The four samples around a world position and the position inside that cell.
struct HeightCell @private
{
	float h00, h10, h01, h11;
	float fx, fz;
}

fn HeightCell Heightmap.cell_at(&self, float wx, float wz)
{
	float[<2>] g = self.world_to_grid(wx, wz);
	float gx = math::clamp(g[0], 0.0f, (float)(self.width - 1));
	float gz = math::clamp(g[1], 0.0f, (float)(self.depth - 1));

	uint x0 = (uint)gx;
	uint z0 = (uint)gz;
	uint x1 = math::min(x0 + 1, self.width - 1);
	uint z1 = math::min(z0 + 1, self.depth - 1);

	return {
		.h00 = self.get_height_at_grid(x0, z0),
		.h10 = self.get_height_at_grid(x1, z0),
		.h01 = self.get_height_at_grid(x0, z1),
		.h11 = self.get_height_at_grid(x1, z1),
		.fx = gx - (float)x0,
		.fz = gz - (float)z0,
	};
}

fn float HeightCell.height(&self)
{
	return lerp(lerp(self.h00, self.h10, self.fx), lerp(self.h01, self.h11, self.fx), self.fz);
}

// Normal of the bilinear surface: its analytic gradient at the position.
fn Vec3 HeightCell.normal(&self, float cell_size)
{
	float dx = lerp(self.h10 - self.h00, self.h11 - self.h01, self.fz) / cell_size;
	float dz = lerp(self.h01 - self.h00, self.h11 - self.h10, self.fx) / cell_size;
	Vec3 n = { -dx, 1.0f, -dz };
	return n.normalize();
}

// Interpolated height at a world position (bilinear; clamped to the map edge)
fn float Heightmap.get_height(&self, float wx, float wz) @dynamic
{
	HeightCell cell = self.cell_at(wx, wz);
	return cell.height();
}

// Surface normal at a world position, from the height gradient
fn Vec3 Heightmap.get_normal(&self, float wx, float wz) @dynamic
{
	HeightCell cell = self.cell_at(wx, wz);
	return cell.normal(self.cell_size);
}

fn HeightSample Heightmap.sample(&self, float wx, float wz) @dynamic
{
	HeightCell cell = self.cell_at(wx, wz);
	return { .valid = true, .height = cell.height(), .normal = cell.normal(self.cell_size) };
}

// Recompute cached height bounds (call after bulk-modifying heights)
fn void Heightmap.update_height_bounds(&self)
{
	if (self.heights.len() == 0) return;

	self.min_height = float::max;
	self.max_height = -float::max;

	foreach (h : self.heights)
	{
		if (h < self.min_height) self.min_height = h;
		if (h > self.max_height) self.max_height = h;
	}
}

// ============================================================================
// CollisionShape Interface
// ============================================================================

// Approximation for GJK: the best of the four map corners. The narrow phase
// does not use it for heightfields (see `get_type`).
fn Vec3 Heightmap.furthest_point(&self, Vec3 direction) @dynamic
{
	float far_x = self.origin.x + (self.width - 1f) * self.cell_size;
	float far_z = self.origin.z + (self.depth - 1f) * self.cell_size;
	Vec3[4] corners = {
		{ self.origin.x, 0, self.origin.z },
		{ far_x, 0, self.origin.z },
		{ self.origin.x, 0, far_z },
		{ far_x, 0, far_z },
	};

	Vec3 best = corners[0];
	float best_dot = -float::max;
	foreach (&corner : corners)
	{
		corner.y = self.get_height(corner.x, corner.z);
		float d = corner.dot(direction);
		if (d > best_dot)
		{
			best_dot = d;
			best = *corner;
		}
	}
	return best;
}

fn Aabb3 Heightmap.get_aabb(&self) @dynamic
{
	return {
		.min = { self.origin.x, self.min_height, self.origin.z },
		.max = {
			self.origin.x + (self.width - 1f) * self.cell_size,
			self.max_height,
			self.origin.z + (self.depth - 1f) * self.cell_size
		}
	};
}

<*
 Reports `HEIGHTFIELD`, not `CONVEX`: `furthest_point` only knows the map's four
 corners, so GJK would settle bodies onto the plane through them. This type
 routes the pair to `PhysicsWorld.test_heightfield_collision`, which collides
 the triangles under the other body instead.
*>
fn ShapeType Heightmap.get_type(&self) @dynamic => ShapeType.HEIGHTFIELD;

// ============================================================================
// Ray Intersection
// ============================================================================

struct HeightmapHit
{
	bool hit;
	float distance;
	Vec3 point;
	Vec3 normal;
}

// Ray intersection by marching across the heightmap, then bisecting the crossing.
fn HeightmapHit Heightmap.ray_intersect(&self, Ray ray, float max_distance = HEIGHTMAP_DEFAULT_MAX_RAY_DISTANCE)
{
	HeightmapHit result = { .hit = false };

	Aabb3 bounds = self.get_aabb();
	HitInfo aabb_hit = bounds.intersect_ray(ray);
	if (!aabb_hit.collided) return result;

	// Start at the ray origin, or at the AABB entry point if that is ahead of it
	float t = aabb_hit.depth > 0 ? aabb_hit.depth : 0;
	float step_size = self.cell_size * HEIGHTMAP_RAY_STEP_CELLS;
	Vec3 pos = ray.origin + ray.direction * t;

	while (t < max_distance)
	{
		if (pos.x < bounds.min.x || pos.x > bounds.max.x || pos.z < bounds.min.z || pos.z > bounds.max.z) break;

		float terrain_height = self.get_height(pos.x, pos.z);

		if (pos.y <= terrain_height)
		{
			float t_low = math::max(0.0f, t - step_size);
			float t_high = t;

			for (int i = 0; i < HEIGHTMAP_RAY_REFINE_ITERATIONS; i++)
			{
				float t_mid = (t_low + t_high) * 0.5f;
				Vec3 mid_pos = ray.origin + ray.direction * t_mid;
				if (mid_pos.y <= self.get_height(mid_pos.x, mid_pos.z))
				{
					t_high = t_mid;
				}
				else
				{
					t_low = t_mid;
				}
			}

			result.hit = true;
			result.distance = t_high;
			result.point = ray.origin + ray.direction * t_high;
			result.normal = self.get_normal(result.point.x, result.point.z);
			return result;
		}

		// Far above the terrain, skip further
		float height_above = pos.y - terrain_height;
		t += math::max(step_size, height_above * HEIGHTMAP_RAY_ADAPTIVE_FRACTION);
		pos = ray.origin + ray.direction * t;
	}

	return result;
}

// ============================================================================
// Collision Detection (written once over `Terrain`)
// ============================================================================

// Sphere against the surface below its centre: depth along the surface normal.
fn ContactHit sphere_vs_sample(Vec3 center, float radius, HeightSample s) @local
{
	if (!s.valid) return {};

	Vec3 terrain_point = { center.x, s.height, center.z };
	float dist = (center - terrain_point).dot(s.normal);
	if (dist >= radius) return {};

	return { .hit = true, .point = center - s.normal * dist, .normal = s.normal, .depth = radius - dist };
}

fn ContactHit terrain_collide_sphere(Terrain terrain, Vec3 center, float radius)
{
	return sphere_vs_sample(center, radius, terrain.sample(center.x, center.z));
}

// Deepest of a row of sphere probes along the capsule axis.
fn ContactHit terrain_collide_capsule(Terrain terrain, Vec3 base, Vec3 tip, float radius)
{
	Vec3 axis = tip - base;
	float length = axis.length();
	if (length < CAPSULE_DEGENERATE_LENGTH) return terrain_collide_sphere(terrain, base, radius);

	int samples = math::max(2, (int)(length / (radius * CAPSULE_SAMPLE_SPACING)));

	ContactHit best;
	for (int i = 0; i <= samples; i++)
	{
		Vec3 point = base + axis * ((float)i / (float)samples);
		best.keep_deeper(terrain_collide_sphere(terrain, point, radius));
	}
	return best;
}

// Result of a terrain query for an entity.
struct TerrainContact
{
	ContactHit contact; // `hit` is false when not touching; `normal` is then the ground normal
	float ground_height; // Terrain height below the entity (0 where there is no terrain)
}

<*
 Sphere entity against terrain. Samples the ground once and reuses it for both
 the contact and the reported ground height/normal.
*>
fn TerrainContact terrain_test_sphere(Terrain terrain, Vec3 position, float radius)
{
	HeightSample s = terrain.sample(position.x, position.z);
	ContactHit hit = sphere_vs_sample(position, radius, s);
	if (!hit.hit) hit.normal = s.normal;
	return { .contact = hit, .ground_height = s.height };
}

// Capsule entity (standing character) against terrain; ground height is read at the base.
fn TerrainContact terrain_test_capsule(Terrain terrain, Vec3 base, Vec3 tip, float radius)
{
	HeightSample s = terrain.sample(base.x, base.z);
	ContactHit hit = terrain_collide_capsule(terrain, base, tip, radius);
	if (!hit.hit) hit.normal = s.normal;
	return { .contact = hit, .ground_height = s.height };
}

fn ContactHit Heightmap.collide_sphere(&self, Vec3 center, float radius) => terrain_collide_sphere(self, center, radius);
fn ContactHit Heightmap.collide_capsule(&self, Vec3 base, Vec3 tip, float radius) => terrain_collide_capsule(self, base, tip, radius);
fn TerrainContact Heightmap.test_collision(&self, Vec3 position, float radius) => terrain_test_sphere(self, position, radius);
fn TerrainContact Heightmap.test_capsule(&self, Vec3 base, Vec3 tip, float radius) => terrain_test_capsule(self, base, tip, radius);

<*
 Visit the two triangles of every grid cell under `query_box` (in the map's own
 space) without allocating. Cell range is clamped to the grid. Each triangle is
 wound so its face normal points up (+y), matching `get_normal`; a one-sided
 contact test depends on that.
*>
macro void Heightmap.@foreach_triangle(&self, Aabb3 query_box; @body(TriangleVerts tri))
{
	float[<2>] g_min = self.world_to_grid(query_box.min.x, query_box.min.z);
	float[<2>] g_max = self.world_to_grid(query_box.max.x, query_box.max.z);

	int x0 = math::max(0, (int)math::floor(g_min[0]));
	int z0 = math::max(0, (int)math::floor(g_min[1]));
	int x1 = math::min((int)self.width - 2, (int)math::ceil(g_max[0]));
	int z1 = math::min((int)self.depth - 2, (int)math::ceil(g_max[1]));

	for (int gz = z0; gz <= z1; gz++)
	{
		for (int gx = x0; gx <= x1; gx++)
		{
			float h00 = self.get_height_at_grid((uint)gx, (uint)gz);
			float h10 = self.get_height_at_grid((uint)(gx + 1), (uint)gz);
			float h01 = self.get_height_at_grid((uint)gx, (uint)(gz + 1));
			float h11 = self.get_height_at_grid((uint)(gx + 1), (uint)(gz + 1));

			float[<2>] w0 = self.grid_to_world((float)gx, (float)gz);
			float[<2>] w1 = self.grid_to_world((float)(gx + 1), (float)(gz + 1));

			Vec3 v00 = { w0[0], h00, w0[1] };
			Vec3 v10 = { w1[0], h10, w0[1] };
			Vec3 v01 = { w0[0], h01, w1[1] };
			Vec3 v11 = { w1[0], h11, w1[1] };

			@body((TriangleVerts){ v00, v01, v10 });
			@body((TriangleVerts){ v10, v01, v11 });
		}
	}
}

// All triangles under `query_box`, collected into a list (see `@foreach_triangle`).
fn void Heightmap.query_triangles(&self, Aabb3 query_box, List {TriangleVerts}* triangles)
{
	self.@foreach_triangle(query_box; TriangleVerts tri)
	{
		triangles.push(tri);
	};
}

// ============================================================================
// Tiled Heightmap System (for streaming large terrains)
// ============================================================================

struct HeightmapTile
{
	Heightmap heightmap;
	int tile_x;
	int tile_z;
}

alias HeightmapLoadCallback = fn void(int tile_x, int tile_z, float[] heights, void* user_data);
alias TileMap = HashMap {long, HeightmapTile};

struct TiledHeightmap (Terrain)
{
	TileMap tiles; // Keyed by `tile_key(tile_x, tile_z)`; entries have stable addresses
	uint tile_width; // Grid samples per tile in X
	uint tile_depth; // Grid samples per tile in Z
	float cell_size; // World distance between samples
	float[<2>] tile_world_size; // World size of each tile (x, z); tiles share their edge samples
	Vec3 origin; // World origin

	// Streaming settings
	float load_radius; // Load tiles whose centre is within this distance
	float unload_radius; // Unload tiles whose centre is beyond this distance

	// Fills a new tile's heights; without one tiles are flat zero
	HeightmapLoadCallback load_callback;
	void* user_data;
}

fn TiledHeightmap create_tiled_heightmap(uint tile_width, uint tile_depth, float cell_size, Vec3 origin = { 0, 0, 0 })
{
	TiledHeightmap terrain = {
		.tile_width = tile_width,
		.tile_depth = tile_depth,
		.cell_size = cell_size,
		.tile_world_size = { (tile_width - 1f) * cell_size, (tile_depth - 1f) * cell_size },
		.origin = origin,
		.load_radius = TILE_DEFAULT_LOAD_RADIUS,
		.unload_radius = TILE_DEFAULT_UNLOAD_RADIUS,
	};
	terrain.tiles.init(mem);
	return terrain;
}

fn void TiledHeightmap.free(&self)
{
	foreach (tile : self.tiles.value_iter())
	{
		tile.heightmap.free();
	}
	self.tiles.free();
}

fn long tile_key(int tx, int tz) @inline @local => ((long)tx << 32) | (long)(uint)tz;

// Tile indices (x, z) for a world position
fn int[<2>] TiledHeightmap.world_to_tile(&self, float wx, float wz)
{
	return {
		(int)math::floor((wx - self.origin.x) / self.tile_world_size[0]),
		(int)math::floor((wz - self.origin.z) / self.tile_world_size[1]),
	};
}

// Get world origin of a tile
fn Vec3 TiledHeightmap.tile_origin(&self, int tx, int tz)
{
	return { self.origin.x + tx * self.tile_world_size[0], 0, self.origin.z + tz * self.tile_world_size[1] };
}

// Squared XZ distance from a tile's centre to a position
fn float TiledHeightmap.tile_dist_sq(&self, int tx, int tz, Vec3 pos)
{
	float dx = self.origin.x + (tx + 0.5f) * self.tile_world_size[0] - pos.x;
	float dz = self.origin.z + (tz + 0.5f) * self.tile_world_size[1] - pos.z;
	return dx * dx + dz * dz;
}

// Find a loaded tile; faults with NOT_FOUND when it is not loaded
fn HeightmapTile*? TiledHeightmap.find_tile(&self, int tx, int tz)
{
	return self.tiles.get_ref(tile_key(tx, tz));
}

// Load a tile (calls load_callback to get height data); no-op if already loaded
fn void TiledHeightmap.load_tile(&self, int tx, int tz)
{
	if (try self.find_tile(tx, tz)) return;

	// Zero-valued terrain: bind the map to the heap, not the temp allocator
	if (!self.tiles.is_initialized()) self.tiles.init(mem);

	Heightmap hm = create_heightmap(self.tile_width, self.tile_depth, self.cell_size, self.tile_origin(tx, tz));
	if (self.load_callback != null)
	{
		self.load_callback(tx, tz, hm.heights.array_view(), self.user_data);
		hm.update_height_bounds();
	}

	self.tiles.set(tile_key(tx, tz), { .heightmap = hm, .tile_x = tx, .tile_z = tz });
}

// Unload a tile
fn void TiledHeightmap.unload_tile(&self, int tx, int tz)
{
	if (try tile = self.find_tile(tx, tz))
	{
		tile.heightmap.free();
		self.tiles.remove(tile_key(tx, tz))!!;
	}
}

// Unload tiles farther than `unload_radius` from every position
fn void TiledHeightmap.unload_far_tiles(&self, Vec3[] positions)
{
	float limit_sq = self.unload_radius * self.unload_radius;
	@pool()
	{
		foreach (key : self.tiles.tkeys())
		{
			HeightmapTile* tile = self.tiles.get_ref(key)!!;
			bool near = false;
			foreach (pos : positions)
			{
				if (self.tile_dist_sq(tile.tile_x, tile.tile_z, pos) <= limit_sq)
				{
					near = true;
					break;
				}
			}
			if (!near)
			{
				tile.heightmap.free();
				self.tiles.remove(key)!!;
			}
		}
	};
}

// Update loaded tiles based on viewer position
fn void TiledHeightmap.update(&self, Vec3 viewer_pos)
{
	Vec3[1] positions = { viewer_pos };
	self.update_multi(&positions);
}

// Update tiles based on multiple positions (player + active NPCs)
fn void TiledHeightmap.update_multi(&self, Vec3[] positions)
{
	if (positions.len == 0) return;

	self.unload_far_tiles(positions);

	// +1: the radius test is on tile centres, so cover the tile the circle edge falls in
	int radius_x = (int)math::ceil(self.load_radius / self.tile_world_size[0]) + 1;
	int radius_z = (int)math::ceil(self.load_radius / self.tile_world_size[1]) + 1;
	float load_sq = self.load_radius * self.load_radius;

	foreach (pos : positions)
	{
		int[<2>] center = self.world_to_tile(pos.x, pos.z);
		for (int tz = center[1] - radius_z; tz <= center[1] + radius_z; tz++)
		{
			for (int tx = center[0] - radius_x; tx <= center[0] + radius_x; tx++)
			{
				if (self.tile_dist_sq(tx, tz, pos) <= load_sq) self.load_tile(tx, tz);
			}
		}
	}
}

// Ensure tile is loaded for a specific position (for NPCs outside normal load radius)
fn void TiledHeightmap.ensure_loaded(&self, Vec3 position)
{
	int[<2>] t = self.world_to_tile(position.x, position.z);
	self.load_tile(t[0], t[1]);
}

// Height/normal from the tile under the position; invalid where it is not loaded.
fn HeightSample TiledHeightmap.sample(&self, float wx, float wz) @dynamic
{
	int[<2>] t = self.world_to_tile(wx, wz);
	if (try tile = self.find_tile(t[0], t[1])) return tile.heightmap.sample(wx, wz);
	return { .valid = false, .height = 0, .normal = { 0, 1, 0 } };
}

// Height at world position (0 where the tile is not loaded)
fn float TiledHeightmap.get_height(&self, float wx, float wz) @dynamic
{
	int[<2>] t = self.world_to_tile(wx, wz);
	if (try tile = self.find_tile(t[0], t[1])) return tile.heightmap.get_height(wx, wz);
	return 0;
}

// Normal at world position (up where the tile is not loaded)
fn Vec3 TiledHeightmap.get_normal(&self, float wx, float wz) @dynamic
{
	int[<2>] t = self.world_to_tile(wx, wz);
	if (try tile = self.find_tile(t[0], t[1])) return tile.heightmap.get_normal(wx, wz);
	return { 0, 1, 0 };
}

// Ray intersection across all loaded tiles
fn HeightmapHit TiledHeightmap.ray_intersect(&self, Ray ray, float max_distance = HEIGHTMAP_DEFAULT_MAX_RAY_DISTANCE)
{
	HeightmapHit best = { .hit = false, .distance = float::max };

	foreach (tile : self.tiles.value_iter())
	{
		HeightmapHit hit = tile.heightmap.ray_intersect(ray, max_distance);
		if (hit.hit && hit.distance < best.distance) best = hit;
	}

	return best;
}

<*
 Collides against the tile under the sphere centre. The contact is measured
 straight below the centre, so neighbouring tiles cannot contribute.
*>
fn ContactHit TiledHeightmap.collide_sphere(&self, Vec3 center, float radius) => terrain_collide_sphere(self, center, radius);
fn ContactHit TiledHeightmap.collide_capsule(&self, Vec3 base, Vec3 tip, float radius) => terrain_collide_capsule(self, base, tip, radius);
fn TerrainContact TiledHeightmap.test_collision(&self, Vec3 position, float radius) => terrain_test_sphere(self, position, radius);
fn TerrainContact TiledHeightmap.test_capsule(&self, Vec3 base, Vec3 tip, float radius) => terrain_test_capsule(self, base, tip, radius);

// ============================================================================
// Mesh to Heightmap Conversion
// ============================================================================

// Converts a mesh to heights by casting rays straight down at each grid point.
struct MeshHeightSampler
{
	Mesh* mesh; // Source mesh (needs a BVH; `create_*` below build it)
	TransformedShape* shape; // Optional: world transform of the mesh
	Vec3 ray_dir; // Downward direction in mesh space; fixed at creation (shape must not rotate afterwards)
	float ray_start_height; // World Y to start rays from (above terrain)
	float default_height; // Height to use when ray misses
}

<*
 Sampler for a mesh, optionally under a world transform (e.g. a glTF node).
 Builds the mesh BVH if needed.
*>
fn MeshHeightSampler create_mesh_sampler(
	Mesh* mesh,
	TransformedShape* shape = null,
	float ray_start_height = HEIGHTMAP_DEFAULT_RAY_START_HEIGHT,
	float default_height = 0.0f
)
{
	mesh.ensure_bvh();
	Vec3 down = { 0, -1, 0 };
	return {
		.mesh = mesh,
		.shape = shape,
		.ray_dir = shape ? shape.dir_to_local(down).normalize() : down,
		.ray_start_height = ray_start_height,
		.default_height = default_height,
	};
}

// Sample a single world-space height from the mesh at (wx, wz)
fn float MeshHeightSampler.sample_height(&self, float wx, float wz)
{
	if (self.mesh == null || !self.mesh.has_bvh()) return self.default_height;

	Vec3 origin = { wx, self.ray_start_height, wz };
	if (self.shape != null) origin = self.shape.to_local(origin);
	Ray ray = { .origin = origin, .direction = self.ray_dir };

	HitInfo hit = self.mesh.bvh.ray_intersects(ray);
	if (!hit.collided) return self.default_height;

	Vec3 local_hit = ray.origin + ray.direction * hit.depth;
	return self.shape != null ? self.shape.to_world(local_hit).y : local_hit.y;
}

// Fill a height array by sampling mesh (for use with TiledHeightmap callback)
fn void MeshHeightSampler.sample_region(&self, float[] heights, uint width, uint depth, float cell_size, Vec3 origin)
{
	for (uint gz = 0; gz < depth; gz++)
	{
		for (uint gx = 0; gx < width; gx++)
		{
			float wx = origin.x + gx * cell_size;
			float wz = origin.z + gz * cell_size;

			heights[gz * width + gx] = self.sample_height(wx, wz);
		}
	}
}

// Heightmap sampled from a mesh (builds its BVH if needed); pass `shape` for a mesh with a world transform.
fn Heightmap create_heightmap_from_mesh(
	Mesh* mesh,
	uint width,
	uint depth,
	float cell_size,
	Vec3 origin = { 0, 0, 0 },
	float ray_start_height = HEIGHTMAP_DEFAULT_RAY_START_HEIGHT,
	TransformedShape* shape = null
)
{
	Heightmap hm = create_heightmap(width, depth, cell_size, origin);

	MeshHeightSampler sampler = create_mesh_sampler(mesh, shape, ray_start_height);
	sampler.sample_region(hm.heights.array_view(), width, depth, cell_size, origin);

	hm.update_height_bounds();
	return hm;
}

// Tile loader that knows the tiled heightmap's configuration
struct MeshTileLoader
{
	MeshHeightSampler sampler;
	TiledHeightmap* terrain; // Pointer to terrain for tile size info
}

fn MeshTileLoader create_mesh_tile_loader(
	Mesh* mesh,
	TiledHeightmap* terrain,
	float ray_start_height = HEIGHTMAP_DEFAULT_RAY_START_HEIGHT,
	TransformedShape* shape = null
)
{
	return { .sampler = create_mesh_sampler(mesh, shape, ray_start_height), .terrain = terrain };
}

// Tile load callback that uses MeshTileLoader as user_data
fn void mesh_tile_loader_callback(int tile_x, int tile_z, float[] heights, void* user_data)
{
	MeshTileLoader* loader = (MeshTileLoader*)user_data;

	loader.sampler.sample_region(
		heights,
		loader.terrain.tile_width,
		loader.terrain.tile_depth,
		loader.terrain.cell_size,
		loader.terrain.tile_origin(tile_x, tile_z)
	);
}

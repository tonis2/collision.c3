module collision;

import std::math;
import std::io;
import std::collections::list;

// ============================================================================
// Heightmap Terrain - Efficient collision for large terrain surfaces
// ============================================================================

alias Heights = List{float};


struct Heightmap (CollisionShape) {
    Heights heights;    // 2D grid of heights (row-major: heights[z * width + x])
    uint width;             // Number of samples in X direction
    uint depth;             // Number of samples in Z direction
    float cell_size;        // World distance between samples
    Vec3 origin;            // World position of corner (min X, min Z)
    float min_height;       // Cached min height for AABB
    float max_height;       // Cached max height for AABB
}

// ============================================================================
// Creation and Destruction
// ============================================================================

fn Heightmap create_heightmap(uint width, uint depth, float cell_size, Vec3 origin = {0, 0, 0}) {
    usz size = (usz)width * (usz)depth;

    Heights heights;
    heights.reserve(size);

    // Initialize to zero
    for (usz i = 0; i < size; i++) {
        heights.push(0);
    }

    return {
        .heights = heights,
        .width = width,
        .depth = depth,
        .cell_size = cell_size,
        .origin = origin,
        .min_height = 0,
        .max_height = 0
    };
}


<*
 @require heights.len >= (usz)width * (usz)depth : `Height data too small for dimensions`
*>
fn Heightmap create_heightmap_from_data(float[] heights, uint width, uint depth, float cell_size, Vec3 origin = {0, 0, 0}) {
    usz size = (usz)width * (usz)depth;

    Heights height_list;
    height_list.push_all(heights[:size]);

    Heightmap hm = {
        .heights = height_list,
        .width = width,
        .depth = depth,
        .cell_size = cell_size,
        .origin = origin
    };

    hm.update_height_bounds();
    return hm;
}

// Create heightmap from existing List (takes ownership)
fn Heightmap create_heightmap_from_list(Heights heights, uint width, uint depth, float cell_size, Vec3 origin = {0, 0, 0}) {
    Heightmap hm = {
        .heights = heights,
        .width = width,
        .depth = depth,
        .cell_size = cell_size,
        .origin = origin
    };

    hm.update_height_bounds();
    return hm;
}

fn void Heightmap.free(&self) @dynamic {
    self.heights.free();
}

// ============================================================================
// Height Access
// ============================================================================

// Get raw height at grid coordinates (no interpolation)
fn float Heightmap.get_height_at_grid(&self, uint gx, uint gz) {
    if (gx >= self.width || gz >= self.depth) {
        return 0;
    }
    return self.heights[(usz)(gz * self.width + gx)];
}

// Set height at grid coordinates
fn void Heightmap.set_height_at_grid(&self, uint gx, uint gz, float height) {
    if (gx >= self.width || gz >= self.depth) return;

    self.heights[(usz)(gz * self.width + gx)] = height;

    // Update bounds
    if (height < self.min_height) self.min_height = height;
    if (height > self.max_height) self.max_height = height;
}

// Convert world position to grid coordinates
fn void Heightmap.world_to_grid(&self, float wx, float wz, float* gx, float* gz) {
    *gx = (wx - self.origin.x) / self.cell_size;
    *gz = (wz - self.origin.z) / self.cell_size;
}

// Convert grid coordinates to world position
fn void Heightmap.grid_to_world(&self, float gx, float gz, float* wx, float* wz) {
    *wx = self.origin.x + gx * self.cell_size;
    *wz = self.origin.z + gz * self.cell_size;
}

// Get interpolated height at world position (bilinear interpolation)
fn float Heightmap.get_height(&self, float wx, float wz) {
    float gx, gz;
    self.world_to_grid(wx, wz, &gx, &gz);

    // Clamp to valid range
    gx = math::clamp(gx, 0.0f, (float)(self.width - 1));
    gz = math::clamp(gz, 0.0f, (float)(self.depth - 1));

    // Get integer grid coordinates
    uint x0 = (uint)gx;
    uint z0 = (uint)gz;
    uint x1 = math::min(x0 + 1, self.width - 1);
    uint z1 = math::min(z0 + 1, self.depth - 1);

    // Fractional part for interpolation
    float fx = gx - (float)x0;
    float fz = gz - (float)z0;

    // Sample four corners
    float h00 = self.get_height_at_grid(x0, z0);
    float h10 = self.get_height_at_grid(x1, z0);
    float h01 = self.get_height_at_grid(x0, z1);
    float h11 = self.get_height_at_grid(x1, z1);

    // Bilinear interpolation
    float h0 = h00 * (1 - fx) + h10 * fx;
    float h1 = h01 * (1 - fx) + h11 * fx;

    return h0 * (1 - fz) + h1 * fz;
}

// Get height and normal at world position
fn float Heightmap.get_height_and_normal(&self, float wx, float wz, Vec3* normal) {
    float height = self.get_height(wx, wz);
    *normal = self.get_normal(wx, wz);
    return height;
}

// Get surface normal at world position (from height gradient)
fn Vec3 Heightmap.get_normal(&self, float wx, float wz) {
    // Sample heights at small offsets
    float delta = self.cell_size * 0.5f;

    float hL = self.get_height(wx - delta, wz);
    float hR = self.get_height(wx + delta, wz);
    float hD = self.get_height(wx, wz - delta);
    float hU = self.get_height(wx, wz + delta);

    // Compute gradient
    float dx = (hR - hL) / (2 * delta);
    float dz = (hU - hD) / (2 * delta);

    // Normal from gradient (assuming Y is up)
    Vec3 normal = { -dx, 1.0f, -dz };
    return normal.normalize();
}

// Update cached height bounds (call after modifying heights)
fn void Heightmap.update_height_bounds(&self) {
    if (self.heights.len() == 0) return;

    self.min_height = float.max;
    self.max_height = -float.max;

    foreach (h : self.heights) {
        if (h < self.min_height) self.min_height = h;
        if (h > self.max_height) self.max_height = h;
    }
}

// ============================================================================
// CollisionShape Interface
// ============================================================================

fn Vec3 Heightmap.furthest_point(&self, Vec3 direction) @dynamic {
    // For heightmaps, we approximate by checking corner points
    // This is used by GJK - for better results, sample more points
    Vec3 best = self.origin;
    best.y = self.get_height(best.x, best.z);
    float best_dot = best.dot(direction);

    // Check corners
    Vec3[4] corners = {
        { self.origin.x, 0, self.origin.z },
        { self.origin.x + (self.width - 1f) * self.cell_size, 0, self.origin.z },
        { self.origin.x, 0, self.origin.z + (self.depth - 1f) * self.cell_size },
        { self.origin.x + (self.width - 1f) * self.cell_size, 0, self.origin.z + (self.depth - 1f) * self.cell_size }
    };

    foreach (&corner : corners) {
        corner.y = self.get_height(corner.x, corner.z);
        float d = corner.dot(direction);
        if (d > best_dot) {
            best_dot = d;
            best = *corner;
        }
    }

    return best;
}

fn Aabb3 Heightmap.get_aabb(&self) @dynamic {
    return {
        .min = { self.origin.x, self.min_height, self.origin.z },
        .max = {
            self.origin.x + (self.width - 1f) * self.cell_size,
            self.max_height,
            self.origin.z + (self.depth - 1f) * self.cell_size
        }
    };
}

fn ShapeType Heightmap.get_type(&self) @dynamic => ShapeType.CONVEX; // Treat as convex for type checking

// ============================================================================
// Ray Intersection
// ============================================================================

struct HeightmapHit {
    bool hit;
    float distance;
    Vec3 point;
    Vec3 normal;
}

// Ray intersection with heightmap using grid traversal
fn HeightmapHit Heightmap.ray_intersect(&self, Ray ray, float max_distance = 1000.0f) {
    HeightmapHit result = { .hit = false };

    // Check if ray intersects heightmap AABB first
    Aabb3 bounds = self.get_aabb();
    HitInfo aabb_hit = bounds.intersect_ray(ray);
    if (!aabb_hit.collided) {
        return result;
    }

    // Start position (either ray origin or AABB entry point)
    float t_start = 0;
    if (aabb_hit.depth > 0) {
        t_start = aabb_hit.depth;
    }

    Vec3 pos = ray.origin + ray.direction * t_start;

    // Step through the heightmap using DDA-like algorithm
    float step_size = self.cell_size * 0.5f;
    float t = t_start;

    while (t < max_distance) {
        // Check if we're still within bounds
        if (pos.x < bounds.min.x || pos.x > bounds.max.x ||
            pos.z < bounds.min.z || pos.z > bounds.max.z) {
            break;
        }

        // Get terrain height at current XZ position
        float terrain_height = self.get_height(pos.x, pos.z);

        // Check if ray is below terrain
        if (pos.y <= terrain_height) {
            // Binary search for exact intersection point
            float t_low = math::max(0.0f, t - step_size);
            float t_high = t;

            for (int i = 0; i < 8; i++) {
                float t_mid = (t_low + t_high) * 0.5f;
                Vec3 mid_pos = ray.origin + ray.direction * t_mid;
                float mid_height = self.get_height(mid_pos.x, mid_pos.z);

                if (mid_pos.y <= mid_height) {
                    t_high = t_mid;
                } else {
                    t_low = t_mid;
                }
            }

            result.hit = true;
            result.distance = t_high;
            result.point = ray.origin + ray.direction * t_high;
            result.normal = self.get_normal(result.point.x, result.point.z);
            return result;
        }

        // Adaptive step size based on distance above terrain
        float height_above = pos.y - terrain_height;
        float adaptive_step = math::max(step_size, height_above * 0.5f);

        t += adaptive_step;
        pos = ray.origin + ray.direction * t;
    }

    return result;
}

// ============================================================================
// Collision Detection
// ============================================================================

// Check collision between a sphere and the heightmap
fn bool Heightmap.collide_sphere(&self, Vec3 sphere_center, float sphere_radius, Vec3* contact_point, Vec3* contact_normal, float* penetration) {
    // Get terrain height at sphere's XZ position
    float terrain_height = self.get_height(sphere_center.x, sphere_center.z);
    Vec3 terrain_normal = self.get_normal(sphere_center.x, sphere_center.z);

    // Point on terrain directly below sphere
    Vec3 terrain_point = { sphere_center.x, terrain_height, sphere_center.z };

    // Distance from sphere center to terrain surface (along normal)
    float dist = (sphere_center - terrain_point).dot(terrain_normal);

    if (dist < sphere_radius) {
        *penetration = sphere_radius - dist;
        *contact_normal = terrain_normal;
        *contact_point = sphere_center - terrain_normal * dist;
        return true;
    }

    return false;
}

// Check collision between a capsule and the heightmap
fn bool Heightmap.collide_capsule(&self, Vec3 capsule_base, Vec3 capsule_tip, float radius, Vec3* contact_point, Vec3* contact_normal, float* penetration) {
    // Check multiple points along capsule axis
    Vec3 axis = capsule_tip - capsule_base;
    float length = axis.length();

    if (length < 0.001f) {
        // Degenerate capsule - treat as sphere
        return self.collide_sphere(capsule_base, radius, contact_point, contact_normal, penetration);
    }

    Vec3 axis_norm = axis / length;

    // Sample points along the capsule
    int samples = math::max(2, (int)(length / (radius * 0.5f)));

    bool any_collision = false;
    float max_penetration = 0;
    Vec3 best_contact;
    Vec3 best_normal;

    for (int i = 0; i <= samples; i++) {
        float t = (float)i / (float)samples;
        Vec3 point = capsule_base + axis * t;

        Vec3 cp, cn;
        float pen;
        if (self.collide_sphere(point, radius, &cp, &cn, &pen)) {
            any_collision = true;
            if (pen > max_penetration) {
                max_penetration = pen;
                best_contact = cp;
                best_normal = cn;
            }
        }
    }

    if (any_collision) {
        *contact_point = best_contact;
        *contact_normal = best_normal;
        *penetration = max_penetration;
    }

    return any_collision;
}

// Check collision with AABB and return all potentially colliding triangles
fn void Heightmap.query_triangles(&self, Aabb3 query_box, List{TriangleVerts}* triangles) {
    // Convert AABB to grid coordinates
    float gx_min, gz_min, gx_max, gz_max;
    self.world_to_grid(query_box.min.x, query_box.min.z, &gx_min, &gz_min);
    self.world_to_grid(query_box.max.x, query_box.max.z, &gx_max, &gz_max);

    // Clamp to valid range
    int x0 = math::max(0, (int)math::floor(gx_min));
    int z0 = math::max(0, (int)math::floor(gz_min));
    int x1 = math::min((int)self.width - 2, (int)math::ceil(gx_max));
    int z1 = math::min((int)self.depth - 2, (int)math::ceil(gz_max));

    // Generate triangles for each cell in range
    for (int gz = z0; gz <= z1; gz++) {
        for (int gx = x0; gx <= x1; gx++) {
            // Get heights at cell corners
            float h00 = self.get_height_at_grid((uint)gx, (uint)gz);
            float h10 = self.get_height_at_grid((uint)(gx + 1), (uint)gz);
            float h01 = self.get_height_at_grid((uint)gx, (uint)(gz + 1));
            float h11 = self.get_height_at_grid((uint)(gx + 1), (uint)(gz + 1));

            // World positions
            float wx0, wz0, wx1, wz1;
            self.grid_to_world((float)gx, (float)gz, &wx0, &wz0);
            self.grid_to_world((float)(gx + 1), (float)(gz + 1), &wx1, &wz1);

            Vec3 v00 = { wx0, h00, wz0 };
            Vec3 v10 = { wx1, h10, wz0 };
            Vec3 v01 = { wx0, h01, wz1 };
            Vec3 v11 = { wx1, h11, wz1 };

            // Two triangles per cell
            triangles.push({ v00, v10, v01 });
            triangles.push({ v10, v11, v01 });
        }
    }
}

// ============================================================================
// Tiled Heightmap System (for streaming large terrains)
// ============================================================================

struct HeightmapTile {
    Heightmap heightmap;
    int tile_x;
    int tile_z;
    bool loaded;
}

struct TiledHeightmap {
    List{HeightmapTile} tiles;
    uint tile_width;        // Grid samples per tile
    uint tile_depth;        // Grid samples per tile
    float cell_size;        // World distance between samples
    float tile_world_size;  // World size of each tile
    Vec3 origin;            // World origin

    // Streaming settings
    float load_radius;      // Load tiles within this radius
    float unload_radius;    // Unload tiles outside this radius

    // Callback for loading tile data
    HeightmapLoadCallback load_callback;
    void* user_data;
}

alias HeightmapLoadCallback = fn void(int tile_x, int tile_z, float[] heights, void* user_data);

fn TiledHeightmap create_tiled_heightmap(uint tile_width, uint tile_depth, float cell_size, Vec3 origin = {0, 0, 0}) {
    return {
        .tile_width = tile_width,
        .tile_depth = tile_depth,
        .cell_size = cell_size,
        .tile_world_size = (tile_width - 1f) * cell_size,
        .origin = origin,
        .load_radius = 200.0f,
        .unload_radius = 250.0f
    };
}

fn void TiledHeightmap.free(&self) {
    foreach (&tile : self.tiles) {
        if (tile.loaded) {
            tile.heightmap.free();
        }
    }
    self.tiles.free();
}

// Get tile indices for a world position
fn void TiledHeightmap.world_to_tile(&self, float wx, float wz, int* tx, int* tz) {
    *tx = (int)math::floor((wx - self.origin.x) / self.tile_world_size);
    *tz = (int)math::floor((wz - self.origin.z) / self.tile_world_size);
}

// Get world origin of a tile
fn Vec3 TiledHeightmap.tile_origin(&self, int tx, int tz) {
    return {
        self.origin.x + tx * self.tile_world_size,
        0,
        self.origin.z + tz * self.tile_world_size
    };
}

// Find tile by coordinates
fn HeightmapTile*? TiledHeightmap.find_tile(&self, int tx, int tz) {
    foreach (&tile : self.tiles) {
        if (tile.tile_x == tx && tile.tile_z == tz) {
            return tile;
        }
    }
    return null;
}

// Load a tile (call load_callback to get height data)
fn void TiledHeightmap.load_tile(&self, int tx, int tz) {
    // Check if already loaded
    if (try self.find_tile(tx, tz)) {
        return;
    }

    Vec3 tile_origin = self.tile_origin(tx, tz);

    // Create heightmap for this tile
    Heightmap hm = create_heightmap(self.tile_width, self.tile_depth, self.cell_size, tile_origin);

    // Call user callback to fill in height data
    if (self.load_callback != null) {
        self.load_callback(tx, tz, hm.heights.array_view(), self.user_data);
        hm.update_height_bounds();
    }

    self.tiles.push({
        .heightmap = hm,
        .tile_x = tx,
        .tile_z = tz,
        .loaded = true
    });
}

// Unload a tile
fn void TiledHeightmap.unload_tile(&self, int tx, int tz) {
    for (usz i = 0; i < self.tiles.len(); i++) {
        if (self.tiles[i].tile_x == tx && self.tiles[i].tile_z == tz) {
            self.tiles[i].heightmap.free();
            self.tiles.remove_at(i);
            return;
        }
    }
}

// Update loaded tiles based on viewer position
fn void TiledHeightmap.update(&self, Vec3 viewer_pos) {
    int viewer_tx, viewer_tz;
    self.world_to_tile(viewer_pos.x, viewer_pos.z, &viewer_tx, &viewer_tz);

    // Calculate tile range to keep loaded
    int tile_radius = (int)math::ceil(self.load_radius / self.tile_world_size) + 1;

    // Unload distant tiles
    for (isz i = (isz)self.tiles.len() - 1; i >= 0; i--) {
        HeightmapTile* tile = &self.tiles[(usz)i];

        float tile_center_x = self.origin.x + (tile.tile_x + 0.5f) * self.tile_world_size;
        float tile_center_z = self.origin.z + (tile.tile_z + 0.5f) * self.tile_world_size;

        float dx = tile_center_x - viewer_pos.x;
        float dz = tile_center_z - viewer_pos.z;
        float dist = math::sqrt(dx * dx + dz * dz);

        if (dist > self.unload_radius) {
            tile.heightmap.free();
            self.tiles.remove_at((usz)i);
        }
    }

    // Load nearby tiles
    for (int tz = viewer_tz - tile_radius; tz <= viewer_tz + tile_radius; tz++) {
        for (int tx = viewer_tx - tile_radius; tx <= viewer_tx + tile_radius; tx++) {
            float tile_center_x = self.origin.x + (tx + 0.5f) * self.tile_world_size;
            float tile_center_z = self.origin.z + (tz + 0.5f) * self.tile_world_size;

            float dx = tile_center_x - viewer_pos.x;
            float dz = tile_center_z - viewer_pos.z;
            float dist = math::sqrt(dx * dx + dz * dz);

            if (dist <= self.load_radius) {
                self.load_tile(tx, tz);
            }
        }
    }
}

// Get height at world position (queries appropriate tile)
fn float TiledHeightmap.get_height(&self, float wx, float wz) {
    int tx, tz;
    self.world_to_tile(wx, wz, &tx, &tz);

    if (try tile = self.find_tile(tx, tz)) {
        return tile.heightmap.get_height(wx, wz);
    }

    return 0; // Tile not loaded
}

// Get normal at world position
fn Vec3 TiledHeightmap.get_normal(&self, float wx, float wz) {
    int tx, tz;
    self.world_to_tile(wx, wz, &tx, &tz);

    if (try tile = self.find_tile(tx, tz)) {
        return tile.heightmap.get_normal(wx, wz);
    }

    return { 0, 1, 0 }; // Default up normal
}

// Ray intersection across all loaded tiles
fn HeightmapHit TiledHeightmap.ray_intersect(&self, Ray ray, float max_distance = 1000.0f) {
    HeightmapHit best = { .hit = false, .distance = float.max };

    foreach (&tile : self.tiles) {
        HeightmapHit hit = tile.heightmap.ray_intersect(ray, max_distance);
        if (hit.hit && hit.distance < best.distance) {
            best = hit;
        }
    }

    return best;
}

// Collide sphere against all loaded tiles
fn bool TiledHeightmap.collide_sphere(&self, Vec3 center, float radius, Vec3* contact_point, Vec3* contact_normal, float* penetration) {
    // Find which tile the sphere center is in
    int tx, tz;
    self.world_to_tile(center.x, center.z, &tx, &tz);

    bool any_collision = false;
    float max_penetration = 0;
    Vec3 best_contact;
    Vec3 best_normal;

    // Check center tile and neighbors (sphere might overlap tile boundaries)
    for (int dz = -1; dz <= 1; dz++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (try tile = self.find_tile(tx + dx, tz + dz)) {
                Vec3 cp, cn;
                float pen;
                if (tile.heightmap.collide_sphere(center, radius, &cp, &cn, &pen)) {
                    any_collision = true;
                    if (pen > max_penetration) {
                        max_penetration = pen;
                        best_contact = cp;
                        best_normal = cn;
                    }
                }
            }
        }
    }

    if (any_collision) {
        *contact_point = best_contact;
        *contact_normal = best_normal;
        *penetration = max_penetration;
    }

    return any_collision;
}

// Collide capsule against all loaded tiles
fn bool TiledHeightmap.collide_capsule(&self, Vec3 capsule_base, Vec3 capsule_tip, float radius, Vec3* contact_point, Vec3* contact_normal, float* penetration) {
    // Find tiles that the capsule might touch
    float min_x = math::min(capsule_base.x, capsule_tip.x) - radius;
    float max_x = math::max(capsule_base.x, capsule_tip.x) + radius;
    float min_z = math::min(capsule_base.z, capsule_tip.z) - radius;
    float max_z = math::max(capsule_base.z, capsule_tip.z) + radius;

    int tx0, tz0, tx1, tz1;
    self.world_to_tile(min_x, min_z, &tx0, &tz0);
    self.world_to_tile(max_x, max_z, &tx1, &tz1);

    bool any_collision = false;
    float max_penetration = 0;
    Vec3 best_contact;
    Vec3 best_normal;

    for (int tz = tz0; tz <= tz1; tz++) {
        for (int tx = tx0; tx <= tx1; tx++) {
            if (try tile = self.find_tile(tx, tz)) {
                Vec3 cp, cn;
                float pen;
                if (tile.heightmap.collide_capsule(capsule_base, capsule_tip, radius, &cp, &cn, &pen)) {
                    any_collision = true;
                    if (pen > max_penetration) {
                        max_penetration = pen;
                        best_contact = cp;
                        best_normal = cn;
                    }
                }
            }
        }
    }

    if (any_collision) {
        *contact_point = best_contact;
        *contact_normal = best_normal;
        *penetration = max_penetration;
    }

    return any_collision;
}

// ============================================================================
// Multi-Entity Support
// ============================================================================

// Result of terrain collision test
struct TerrainContact {
    bool collided;
    Vec3 point;         // Contact point on terrain
    Vec3 normal;        // Surface normal at contact
    float penetration;  // How deep into terrain
    float ground_height;// Terrain height at entity position
}

// Test any entity against terrain - returns contact info
fn TerrainContact TiledHeightmap.test_collision(&self, Vec3 position, float radius) {
    TerrainContact result;

    result.ground_height = self.get_height(position.x, position.z);
    result.normal = self.get_normal(position.x, position.z);

    result.collided = self.collide_sphere(position, radius, &result.point, &result.normal, &result.penetration);

    return result;
}

// Test capsule entity (standing character) against terrain
fn TerrainContact TiledHeightmap.test_capsule(&self, Vec3 base, Vec3 tip, float radius) {
    TerrainContact result;

    // Use base position for ground height
    result.ground_height = self.get_height(base.x, base.z);
    result.normal = self.get_normal(base.x, base.z);

    result.collided = self.collide_capsule(base, tip, radius, &result.point, &result.normal, &result.penetration);

    return result;
}

// Same for single Heightmap
fn TerrainContact Heightmap.test_collision(&self, Vec3 position, float radius) {
    TerrainContact result;

    result.ground_height = self.get_height(position.x, position.z);
    result.normal = self.get_normal(position.x, position.z);

    result.collided = self.collide_sphere(position, radius, &result.point, &result.normal, &result.penetration);

    return result;
}

fn TerrainContact Heightmap.test_capsule(&self, Vec3 base, Vec3 tip, float radius) {
    TerrainContact result;

    result.ground_height = self.get_height(base.x, base.z);
    result.normal = self.get_normal(base.x, base.z);

    result.collided = self.collide_capsule(base, tip, radius, &result.point, &result.normal, &result.penetration);

    return result;
}

// Update tiles based on multiple positions (player + active NPCs)
fn void TiledHeightmap.update_multi(&self, Vec3[] positions) {
    if (positions.len == 0) return;

    // Unload tiles that are far from ALL positions
    for (isz i = (isz)self.tiles.len() - 1; i >= 0; i--) {
        HeightmapTile* tile = &self.tiles[(usz)i];

        float tile_center_x = self.origin.x + (tile.tile_x + 0.5f) * self.tile_world_size;
        float tile_center_z = self.origin.z + (tile.tile_z + 0.5f) * self.tile_world_size;

        // Check if tile is within range of ANY position
        bool keep_loaded = false;
        foreach (pos : positions) {
            float dx = tile_center_x - pos.x;
            float dz = tile_center_z - pos.z;
            float dist = math::sqrt(dx * dx + dz * dz);

            if (dist <= self.unload_radius) {
                keep_loaded = true;
                break;
            }
        }

        if (!keep_loaded) {
            tile.heightmap.free();
            self.tiles.remove_at((usz)i);
        }
    }

    // Load tiles near any position
    foreach (pos : positions) {
        int pos_tx, pos_tz;
        self.world_to_tile(pos.x, pos.z, &pos_tx, &pos_tz);

        int tile_radius = (int)math::ceil(self.load_radius / self.tile_world_size) + 1;

        for (int tz = pos_tz - tile_radius; tz <= pos_tz + tile_radius; tz++) {
            for (int tx = pos_tx - tile_radius; tx <= pos_tx + tile_radius; tx++) {
                float tile_center_x = self.origin.x + (tx + 0.5f) * self.tile_world_size;
                float tile_center_z = self.origin.z + (tz + 0.5f) * self.tile_world_size;

                float dx = tile_center_x - pos.x;
                float dz = tile_center_z - pos.z;
                float dist = math::sqrt(dx * dx + dz * dz);

                if (dist <= self.load_radius) {
                    self.load_tile(tx, tz);
                }
            }
        }
    }
}

// Ensure tile is loaded for a specific position (for NPCs outside normal load radius)
fn void TiledHeightmap.ensure_loaded(&self, Vec3 position) {
    int tx, tz;
    self.world_to_tile(position.x, position.z, &tx, &tz);
    self.load_tile(tx, tz);
}

// ============================================================================
// Mesh to Heightmap Conversion
// ============================================================================

// Context for mesh-based height sampling
struct MeshHeightSampler {
    Mesh* mesh;                 // Source mesh (must have BVH built)
    TransformedShape* shape;    // Optional: transformed shape for world-space mesh
    float ray_start_height;     // Y position to start rays from (above terrain)
    float default_height;       // Height to use when ray misses
}

// Create a sampler for converting mesh to heightmap
fn MeshHeightSampler create_mesh_sampler(Mesh* mesh, float ray_start_height = 1000.0f, float default_height = 0.0f) {
    return {
        .mesh = mesh,
        .shape = null,
        .ray_start_height = ray_start_height,
        .default_height = default_height
    };
}

// Create a sampler with transform (for GLTF meshes that have world transforms)
fn MeshHeightSampler create_mesh_sampler_transformed(Mesh* mesh, TransformedShape* shape, float ray_start_height = 1000.0f, float default_height = 0.0f) {
    return {
        .mesh = mesh,
        .shape = shape,
        .ray_start_height = ray_start_height,
        .default_height = default_height
    };
}

// Sample a single height from mesh at world position (wx, wz)
fn float MeshHeightSampler.sample_height(&self, float wx, float wz) {
    if (self.mesh == null || !self.mesh.has_bvh()) {
        return self.default_height;
    }

    // Create downward ray
    Ray ray = {
        .origin = { wx, self.ray_start_height, wz },
        .direction = { 0, -1, 0 }
    };

    // If we have a transform, convert ray to local space
    if (self.shape != null) {
        // Transform ray origin to local space
        Vec3 local_origin = self.shape.rotation.conjugate() * (ray.origin - self.shape.translation);
        local_origin = local_origin / self.shape.scale;

        // Transform ray direction to local space (direction only, no translation)
        Vec3 local_dir = self.shape.rotation.conjugate() * ray.direction;

        ray.origin = local_origin;
        ray.direction = local_dir.normalize();
    }

    // Query BVH
    HitInfo hit = self.mesh.bvh.ray_intersects(ray);

    if (hit.collided) {
        // Calculate world-space hit point
        Vec3 local_hit = ray.origin + ray.direction * hit.depth;

        if (self.shape != null) {
            // Transform back to world space
            Vec3 world_hit = self.shape.rotation * (local_hit * self.shape.scale) + self.shape.translation;
            return world_hit.y;
        } else {
            return local_hit.y;
        }
    }

    return self.default_height;
}

// Fill a height array by sampling mesh (for use with TiledHeightmap callback)
fn void MeshHeightSampler.sample_region(&self, float[] heights, uint width, uint depth, float cell_size, Vec3 origin) {
    for (uint gz = 0; gz < depth; gz++) {
        for (uint gx = 0; gx < width; gx++) {
            float wx = origin.x + gx * cell_size;
            float wz = origin.z + gz * cell_size;

            heights[gz * width + gx] = self.sample_height(wx, wz);
        }
    }
}

// Create a complete heightmap from mesh by sampling
fn Heightmap create_heightmap_from_mesh(Mesh* mesh, uint width, uint depth, float cell_size, Vec3 origin = {0, 0, 0}, float ray_start_height = 1000.0f) {
    // Build BVH if not already built
    if (!mesh.has_bvh()) {
        mesh.build_bvh();
    }

    Heightmap hm = create_heightmap(width, depth, cell_size, origin);

    MeshHeightSampler sampler = create_mesh_sampler(mesh, ray_start_height);
    sampler.sample_region(hm.heights.array_view(), width, depth, cell_size, origin);

    hm.update_height_bounds();
    return hm;
}

// Create heightmap from transformed mesh (GLTF with world transform)
fn Heightmap create_heightmap_from_transformed_mesh(Mesh* mesh, TransformedShape* shape, uint width, uint depth, float cell_size, Vec3 origin = {0, 0, 0}, float ray_start_height = 1000.0f) {
    // Build BVH if not already built
    if (!mesh.has_bvh()) {
        mesh.build_bvh();
    }

    Heightmap hm = create_heightmap(width, depth, cell_size, origin);

    MeshHeightSampler sampler = create_mesh_sampler_transformed(mesh, shape, ray_start_height);
    sampler.sample_region(hm.heights.array_view(), width, depth, cell_size, origin);

    hm.update_height_bounds();
    return hm;
}

// Callback helper for TiledHeightmap - use MeshHeightSampler as user_data
fn void mesh_tile_load_callback(int tile_x, int tile_z, float[] heights, void* user_data) {
    MeshHeightSampler* sampler = (MeshHeightSampler*)user_data;

    // This requires knowing the tile parameters - caller must set these up
    // For a complete solution, use MeshTileLoader below
}

// Complete tile loader that knows about the tiled heightmap configuration
struct MeshTileLoader {
    MeshHeightSampler sampler;
    TiledHeightmap* terrain;    // Pointer to terrain for tile size info
}

fn MeshTileLoader create_mesh_tile_loader(Mesh* mesh, TiledHeightmap* terrain, float ray_start_height = 1000.0f) {
    // Build BVH if needed
    if (!mesh.has_bvh()) {
        mesh.build_bvh();
    }

    return {
        .sampler = create_mesh_sampler(mesh, ray_start_height),
        .terrain = terrain
    };
}

fn MeshTileLoader create_mesh_tile_loader_transformed(Mesh* mesh, TransformedShape* shape, TiledHeightmap* terrain, float ray_start_height = 1000.0f) {
    // Build BVH if needed
    if (!mesh.has_bvh()) {
        mesh.build_bvh();
    }

    return {
        .sampler = create_mesh_sampler_transformed(mesh, shape, ray_start_height),
        .terrain = terrain
    };
}

// Tile load callback that uses MeshTileLoader
fn void mesh_tile_loader_callback(int tile_x, int tile_z, float[] heights, void* user_data) {
    MeshTileLoader* loader = (MeshTileLoader*)user_data;

    Vec3 tile_origin = loader.terrain.tile_origin(tile_x, tile_z);

    loader.sampler.sample_region(
        heights,
        loader.terrain.tile_width,
        loader.terrain.tile_depth,
        loader.terrain.cell_size,
        tile_origin
    );
}

// ============================================================================
// Usage Examples (commented)
// ============================================================================

// Single heightmap (small terrain)                                                                                                                  
//   Heightmap terrain = create_heightmap(256, 256, 1.0f);  // 256x256 grid, 1m spacing                                                                   
//   terrain.set_height_at_grid(100, 100, 5.0f);  // Set a point                                                                                          
//   float h = terrain.get_height(50.5f, 75.2f);   // Get interpolated height                                                                             
                                                                                                                                                       
//   // Character ground check                                                                                                                            
//   Vec3 contact, normal;                                                                                                                                
//   float penetration;                                                                                                                                   
//   if (terrain.collide_capsule(char_base, char_tip, 0.5f, &contact, &normal, &penetration)) {                                                           
//       // Push character out of ground                                                                                                                  
//       character.translation += normal * penetration;                                                                                                   
//   }                                                                                                                                                    
                                                                                                                                                       
//   // Tiled heightmap (large streaming terrain)                                                                                                         
//   TiledHeightmap world = create_tiled_heightmap(65, 65, 2.0f);  // 65x65 tiles, 2m cells                                                               
//   world.load_callback = &my_tile_loader;                                                                                                               
//   world.load_radius = 300.0f;                                                                                                                          
                                                                                                                                                       
//   // Each frame                                                                                                                                        
//   world.update(player_position);  // Loads/unloads tiles automatically                                                                                 
//   float ground_height = world.get_height(player.x, player.z);                                                                                          
                                                                                                                                                                                  
                                                                                                                                                       
//   // Your GLTF ground mesh                                                                                                                             
//   Mesh* ground = &gltf_scene.meshes[ground_index];                                                                                                     
                                                                                                                                                       
//   // Create heightmap by sampling the mesh (64x64 grid, 2m spacing)                                                                                    
//   Heightmap terrain = create_heightmap_from_mesh(                                                                                                      
//       ground,                                                                                                                                          
//       width: 64,                                                                                                                                       
//       depth: 64,                                                                                                                                       
//       cell_size: 2.0f,                                                                                                                                 
//       origin: {-64, 0, -64}  // Center the terrain                                                                                                     
//   );                                                                                                                                                   
                                                                                                                                                       
//   With GLTF Transform                                                                                                                                  
                                                                                                                                                       
//   // If mesh has a world transform from GLTF                                                                                                           
//   TransformedShape shape = {                                                                                                                           
//       .translation = gltf_node.translation,                                                                                                            
//       .rotation = gltf_node.rotation,                                                                                                                  
//       .scale = gltf_node.scale,                                                                                                                        
//       .shape = ground                                                                                                                                  
//   };                                                                                                                                                   
                                                                                                                                                       
//   Heightmap terrain = create_heightmap_from_transformed_mesh(                                                                                          
//       ground,                                                                                                                                          
//       &shape,                                                                                                                                          
//       width: 128,                                                                                                                                      
//       depth: 128,                                                                                                                                      
//       cell_size: 1.0f                                                                                                                                  
//   );                                                                                                                                                   
                                                                                                                                                       
//   Tiled/Streaming Terrain                                                                                                                              
                                                                                                                                                       
//   // Create tiled terrain                                                                                                                              
//   TiledHeightmap terrain = create_tiled_heightmap(                                                                                                     
//       tile_width: 65,                                                                                                                                  
//       tile_depth: 65,                                                                                                                                  
//       cell_size: 2.0f                                                                                                                                  
//   );                                                                                                                                                   
//   terrain.load_radius = 200.0f;                                                                                                                        
//   terrain.unload_radius = 300.0f;                                                                                                                      
                                                                                                                                                       
//   // Create the loader (holds mesh + terrain reference)                                                                                                
//   MeshTileLoader loader = create_mesh_tile_loader(ground_mesh, &terrain);                                                                              
                                                                                                                                                       
//   // Or with transform:                                                                                                                                
//   // MeshTileLoader loader = create_mesh_tile_loader_transformed(ground_mesh, &shape, &terrain);                                                       
                                                                                                                                                       
//   // Set up callback                                                                                                                                   
//   terrain.load_callback = &mesh_tile_loader_callback;                                                                                                  
//   terrain.user_data = &loader;                                                                                                                         
                                                                                                                                                       
//   // In game loop - tiles load/unload automatically                                                                                                    
//   terrain.update(player.position);                                                                                                                     
//   float ground_height = terrain.get_height(player.x, player.z);                                                                                        
                                                                                                                                                       
//   What Happens                                                                                                                                         
                                                                                                                                                       
//   1. When player enters a new tile's load_radius:                                                                                                      
//     - mesh_tile_loader_callback is called                                                                                                              
//     - It raycasts downward at each grid point                                                                                                          
//     - Heights are filled into the tile's heightmap                                                                                                     
//   2. When player leaves a tile's unload_radius:                                                                                                        
//     - Tile heightmap is freed automatically                                                                                                            
                                                                                                                                                       
//   The mesh BVH is built once and shared - only the height samples are stored per tile.   
<*
 `TiledHeightmap` streaming and the `Terrain` queries shared with `Heightmap`.

	`c3c test --test-filter collision_tests::terrain`

 **Tiles must actually load.** `find_tile` used to end in `return null;`, a
 successful null, so `load_tile` believed every tile was already there and no
 tile ever loaded. These tests drive loading through a `load_callback`.
*>
module collision_tests::terrain;
import collision;
import std::core::test;
import std::math;

const uint TILE_SAMPLES = 5; // 4 cells per tile
const float CELL = 1.0f;
const float TILE_SIZE = 4.0f;

int g_loads;

// Height 0.5 * x (world), so the surface is continuous across tile edges.
fn void ramp_loader(int tile_x, int tile_z, float[] heights, void* user_data)
{
	g_loads++;
	for (uint gz = 0; gz < TILE_SAMPLES; gz++)
	{
		for (uint gx = 0; gx < TILE_SAMPLES; gx++)
		{
			heights[gz * TILE_SAMPLES + gx] = 0.5f * (tile_x * TILE_SIZE + gx * CELL);
		}
	}
}

fn TiledHeightmap make_terrain() @local
{
	TiledHeightmap terrain = collision::create_tiled_heightmap(TILE_SAMPLES, TILE_SAMPLES, CELL);
	terrain.load_callback = &ramp_loader;
	g_loads = 0;
	return terrain;
}

fn void test_load_tile_calls_callback_once() @test
{
	TiledHeightmap terrain = make_terrain();
	defer terrain.free();

	test::@check(terrain.tiles.len() == 0);
	terrain.load_tile(0, 0);
	test::@check(g_loads == 1);
	test::@check(terrain.tiles.len() == 1);
	test::@check(@ok(terrain.find_tile(0, 0)));
	test::@check(!@ok(terrain.find_tile(1, 0)));

	// Already loaded: the callback does not run again
	terrain.load_tile(0, 0);
	test::@check(g_loads == 1);
	test::@check(terrain.tiles.len() == 1);
}

fn void test_height_across_tile_boundary() @test
{
	TiledHeightmap terrain = make_terrain();
	defer terrain.free();
	terrain.load_tile(0, 0);
	terrain.load_tile(1, 0);
	terrain.load_tile(-1, 0);

	float[8] xs = { -2.0f, -0.25f, 0.0f, 1.5f, 3.99f, 4.0f, 4.25f, 7.0f };
	foreach (x : xs)
	{
		test::@check(math::abs(terrain.get_height(x, 1.0f) - 0.5f * x) < 1e-4f);
	}

	// Slope is the same on both sides of the seam
	Vec3 a = terrain.get_normal(3.9f, 1);
	Vec3 b = terrain.get_normal(4.1f, 1);
	test::@check(math::abs(a.x - b.x) < 1e-4f);
	test::@check(a.x < 0);

	// Unloaded tile: no data, and no collision
	test::@check(terrain.get_height(100, 100) == 0);
	TerrainContact c = terrain.test_collision({ 100, 0.1f, 100 }, 1.0f);
	test::@check(!c.contact.hit);
}

fn void test_collide_sphere_in_loaded_tile() @test
{
	TiledHeightmap terrain = make_terrain();
	defer terrain.free();
	terrain.load_tile(0, 0);

	// Surface at x = 2 is y = 1; a unit sphere centred 0.5 above its normal depth penetrates
	TerrainContact c = terrain.test_collision({ 2, 1.5f, 1 }, 1.0f);
	test::@check(c.contact.hit);
	test::@check(math::abs(c.ground_height - 1.0f) < 1e-4f);
	test::@check(c.contact.depth > 0 && c.contact.depth < 1.0f);
	test::@check(c.contact.normal.y > 0.8f);

	// Well above: no hit, but the ground height is still reported
	TerrainContact above = terrain.test_collision({ 2, 10, 1 }, 1.0f);
	test::@check(!above.contact.hit);
	test::@check(math::abs(above.ground_height - 1.0f) < 1e-4f);

	// Capsule: the deepest probe wins
	ContactHit cap = terrain.collide_capsule({ 2, 5, 1 }, { 2, 1.5f, 1 }, 1.0f);
	test::@check(cap.hit);
}

fn void test_update_loads_and_unloads_by_distance() @test
{
	TiledHeightmap terrain = make_terrain();
	defer terrain.free();
	terrain.load_radius = 6.0f;
	terrain.unload_radius = 9.0f;

	terrain.update({ 2, 0, 2 });
	test::@check(terrain.tiles.len() > 0);
	test::@check(@ok(terrain.find_tile(0, 0)));
	test::@check(!@ok(terrain.find_tile(10, 0)));

	// Walk far away: the first tile goes, the new neighbourhood arrives
	terrain.update({ 102, 0, 2 });
	test::@check(!@ok(terrain.find_tile(0, 0)));
	test::@check(@ok(terrain.find_tile(25, 0)));
	test::@check(math::abs(terrain.get_height(102, 2) - 51.0f) < 1e-3f);

	// Several viewers keep both neighbourhoods alive
	terrain.update_multi({ { 2, 0, 2 }, { 102, 0, 2 } });
	test::@check(@ok(terrain.find_tile(0, 0)));
	test::@check(@ok(terrain.find_tile(25, 0)));

	terrain.unload_tile(0, 0);
	test::@check(!@ok(terrain.find_tile(0, 0)));
}

fn void test_non_square_tiles() @test
{
	// 3 x 5 samples at 2m: tiles are 4m wide in x and 8m deep in z
	TiledHeightmap terrain = collision::create_tiled_heightmap(3, 5, 2.0f);
	defer terrain.free();
	test::@check(terrain.tile_world_size[0] == 4.0f);
	test::@check(terrain.tile_world_size[1] == 8.0f);

	int[<2>] t = terrain.world_to_tile(5.0f, 9.0f);
	test::@check(t[0] == 1 && t[1] == 1);
	Vec3 o = terrain.tile_origin(1, 1);
	test::@check(o.x == 4.0f && o.z == 8.0f);

	terrain.load_tile(1, 1);
	test::@check(@ok(terrain.find_tile(1, 1)));
	test::@check(terrain.get_height(5.0f, 9.0f) == 0);
}

fn void test_heightmap_matches_terrain_interface() @test
{
	float[9] data = { 0, 1, 2, 0, 1, 2, 0, 1, 2 }; // ramp in x, 1 per cell
	Heightmap hm = collision::create_heightmap_from_data(&data, 3, 3, 1.0f);
	defer hm.free();

	test::@check(math::abs(hm.get_height(0.5f, 0.5f) - 0.5f) < 1e-5f);

	HeightSample s = hm.sample(1.25f, 0.75f);
	test::@check(s.valid);
	test::@check(math::abs(s.height - 1.25f) < 1e-5f);
	Vec3 n = hm.get_normal(1.25f, 0.75f);
	test::@check(math::abs(n.x - s.normal.x) < 1e-6f && math::abs(n.y - s.normal.y) < 1e-6f);
	test::@check(math::abs(n.x + math::sqrt(0.5f)) < 1e-5f);

	TerrainContact c = hm.test_collision({ 1.25f, 1.5f, 0.75f }, 1.0f);
	test::@check(c.contact.hit);
	test::@check(math::abs(c.ground_height - 1.25f) < 1e-5f);
}

# Collision library

A physics and collision detection library for the [C3 language](https://c3-lang.org/).

## Features

**Collision Detection**
- GJK (Gilbert-Johnson-Keerthi) narrow-phase collision
- EPA (Expanding Polytope Algorithm) for contact points and penetration depth
- Spatial hashing for broad-phase optimization
- `TriBVH` (Bounding Volume Hierarchy) for triangle mesh collision and ray casts

**Physics Simulation**
- Rigid body dynamics with mass and shape-specific inertia (sphere, capsule, cylinder, mesh)
- XPBD position-based solver with friction and restitution
- Multi-contact manifolds for stable stacking and mesh contacts
- Swept CCD — fast bodies don't tunnel through static geometry
- Gravity, linear/angular damping, and sleep states
- Generic 6-DOF joint constraints with one-sided `min`/`max` limits per axis
- XPBD soft bodies with distance, volume and bending constraints

**Collision Shapes**
- `Sphere` - Sphere with radius
- `Capsule` - Capsule with height and radius
- `Cylinder` - Cylinder with height and top/bottom radius
- `Aabb3` - Axis-aligned bounding box
- `Mesh` - Triangle mesh with BVH acceleration
- `CompoundShape` - Multiple convex pieces combined
- `Heightmap` - Terrain collision with ray casting

**Advanced**
- CoACD convex decomposition for concave meshes
- Parallel constraint solving with thread pool
- Collision filtering between body pairs
- Deterministic lockstep support: bit-identical simulation across machines,
  state hashing, and snapshot/restore for rollback netcode.

## Quick Start

```c3
import collision;

fn void main() {
    // Create physics world with default settings
    PhysicsWorld world = collision::create_world();
    defer world.free();

    // Create a static ground plane
    world.add_body(body: {
        .id = 0,
        .is_static = true,
        .material = { .dynamic_friction = 0.5, .restitution = 0.3 },
        .collider = {
            .shape = &&collision::aabb_from_half({10, 10, 0.5}),
            .translation = {0, 0, -0.5},
        }
    })!!;

    // Create a dynamic sphere
    world.add_body(body: {
        .id = 1,
        .mass = 1.0,
        .material = { .dynamic_friction = 0.5, .restitution = 0.8 },
        .collider = {
            .shape = &&(Sphere){ .radius = 0.5 },
            .translation = {0, 0, 5},
        }
    })!!;

    // Game loop
    while (running) {
        world.run_step(1.0 / 60.0);  // Fixed 60Hz timestep

        // Get body position after simulation
        if (try Rigidbody* ball = world.find_body(1)) {
            Vec3f pos = ball.collider.translation;
        }
    }
}
```

## Applying Forces

```c3
Rigidbody* body = world.find_body(1);

// Apply impulse at center of mass
body.apply_linear_impulse({0, 0, 10});

// Apply angular impulse (torque)
body.apply_angular_impulse({0, 1, 0});

// Apply impulse at specific point (creates both linear and angular motion)
body.apply_impulse({5, 0, 0}, point: {0, 0.5, 0});
```

## Collision Shapes

```c3
// Sphere
Sphere sphere = { .radius = 1.0 };

// Capsule (pill shape), centered at origin, axis +y. height is the
// cylindrical section only (KHR_implicit_shapes convention), so this one
// spans y in [-1.5, 1.5].
Capsule capsule = { .height = 2.0, .radius_top = 0.5, .radius_bottom = 0.5 };

// Box from its full size, centered at origin
Aabb3 box = collision::aabb_from_half({2, 2, 2});  // 2x2x2 box

// Cylinder, centered at origin, axis +y (caps at y = ±height/2)
Cylinder cylinder = { .height = 2.0, .radius_top = 0.5, .radius_bottom = 0.5 };

// Triangle mesh; build_bvh creates the TriBVH used for collision and ray casts
// (mesh.free releases both)
Mesh mesh = collision::create_mesh(vertices, indices);
mesh.build_bvh();
defer mesh.free();
```

## Heightmaps

Heightmaps are a grid in x/z with heights in y. A `Heightmap` is a collider
(`ShapeType.HEIGHTFIELD`); `Heightmap` and `TiledHeightmap` both implement the
`Terrain` interface (`get_height`, `get_normal`, `sample`).

```c3
// Single heightmap: 256x256 samples, 1m apart
Heightmap terrain = create_heightmap(256, 256, 1.0f);
defer terrain.free();
terrain.set_height_at_grid(100, 100, 5.0f);
float h = terrain.get_height(50.5f, 75.2f);        // bilinear

// Character ground check
ContactHit hit = terrain.collide_capsule(char_base, char_tip, 0.5f);
if (hit.hit) character.translation += hit.normal * hit.depth;

// Same query plus the ground height below the entity
TerrainContact tc = terrain.test_capsule(char_base, char_tip, 0.5f);
float ground = tc.ground_height;                   // tc.contact is a ContactHit

// From a mesh: rays are cast down at each grid point (pass `shape: &transformed`
// for a mesh with a world transform; the default null is untransformed)
Heightmap from_mesh = create_heightmap_from_mesh(&ground, 64, 64, 2.0f, origin: { -64, 0, -64 });

// Tiled, streaming terrain filled from a mesh
TiledHeightmap world = create_tiled_heightmap(65, 65, 2.0f);
defer world.free();
world.load_radius = 200.0f;
world.unload_radius = 300.0f;
MeshTileLoader loader = create_mesh_tile_loader(&ground, &world);
world.load_callback = &mesh_tile_loader_callback;
world.user_data = &loader;

world.update(player_position);                     // loads/unloads tiles
float ground_height = world.get_height(player.x, player.z);
```

`world_to_grid` and `world_to_tile` return vectors (`float[<2>]` / `int[<2>]`).
Tiles share their edge samples, so a custom `load_callback(tile_x, tile_z,
heights, user_data)` must produce continuous heights across tile edges. Where
no tile is loaded, `get_height` is 0 and collision queries report no hit.

## Spatial Hash (Broad Phase)

```c3
SpatialHash3D spatial_map = { .cell_size = 2.0 };
defer spatial_map.free();

// Insert AABBs (`is_static = true` marks bodies that never move)
spatial_map.insert(aabb1, id: 0);
spatial_map.insert(aabb2, id: 1, is_static: true);

// Get potentially colliding pairs
spatial_map.@get_pairs(; Pair pair) {
    // pair.a and pair.b are IDs of nearby objects
    // Perform narrow-phase collision here
};
```

## Direct Collision Testing

```c3
// Check collision between two shapes
TransformedShape shape_a = { .shape = &sphere, .translation = pos_a };
TransformedShape shape_b = { .shape = &box, .translation = pos_b };

CollisionInfo info = collision::check_convex_collision(&shape_a, &shape_b, epa: true);
if (info.collided) {
    Vec3f normal = info.normal;
    float depth = info.depth;
}
```

## Convex Decomposition

For concave meshes, decompose into convex pieces:

```c3
Mesh concave_mesh = collision::create_mesh(vertices, indices);
MeshList pieces = concave_mesh.decompose(collision::COACD_FAST);
defer pieces.release();

// Create compound shape from pieces
CompoundShape compound;
foreach (piece : pieces) {
    compound.add_piece(&piece);
}
```

## Joints

`GenericJoint3D` holds up to three angular and three linear `Limit`s, one per axis.
A limit with `min == max == 0` locks its axis; otherwise `min`/`max` are one-sided
bounds that are enforced only while violated. `stiffness` (on the joint and on each
`Limit`) sets how hard the constraint is.

```c3
// Ball joint: anchors held together on all three axes
GenericJoint3D joint = {
    .body_a = 1, .body_b = 2,
    .anchor_a = {0.5, 0, 0}, .anchor_b = {-0.5, 0, 0},
    .relative_orientation = {0, 0, 0, 1},
    .collides = false,
};
joint.linear_limits.push({ .axes = {1, 0, 0}, .stiffness = 100 });
joint.linear_limits.push({ .axes = {0, 1, 0}, .stiffness = 100 });
joint.linear_limits.push({ .axes = {0, 0, 1}, .stiffness = 100 });

// Hinge-style angular range about x
joint.angular_limits.push({ .axes = {1, 0, 0}, .min = -0.5, .max = 1.0, .stiffness = 100 });
world.add_constraint(joint)!!;
```

## Configuration

`create_world` takes the settings and seats the world's storage in one call.
Start from `DEFAULT_PHYSICS_WORLD` and change what you need, so the fields you
do not name keep their defaults:

```c3
PhysicsWorld config = collision::DEFAULT_PHYSICS_WORLD;
config.gravity = {0, 0, -9.8};       // Gravity vector (library convention is z-up)
config.spatial_map.cell_size = 2.0;  // Broad-phase cell size
config.sleep_timer = 5.0;            // Seconds before bodies sleep
config.linear_dampening = 0.9;       // Velocity damping per step
config.angular_dampening = 0.9;
config.sleep_delta = 0.1;            // Velocity threshold for sleep
config.ccd_enabled = true;           // Swept CCD for fast-moving bodies
config.thread_count = 0;             // Solver worker threads; 0 = auto (CPU cores - 1)

PhysicsWorld world = collision::create_world(config);
defer world.free();
```

It takes an allocator too — `create_world(config, my_allocator)` — which owns
every container in the world until `free`. **Assigning `DEFAULT_PHYSICS_WORLD`
on its own gives you the settings and not the storage**, and a world whose
containers were never seated binds each of them to the temp allocator on first
use: invisible until a pool is released a frame later. `PhysicsWorld.init` is
the same thing for a world you already have.

Triangle meshes are treated as one-sided: contacts always push bodies out along
the triangle face normal, so meshes need consistent (outward) winding.

## Deterministic lockstep

The simulation is bit-deterministic: two machines that create the same bodies
in the same order and step with the same inputs produce bit-identical worlds,
so multiplayer games can send only player inputs over the wire. The full
contract.

- Build with `-O3` or below (default `--fp-math=strict`); `-O4`/`-O5` enable
  fast math and break bit-equality
- Create bodies in the same order on every peer, step with a fixed,
  bit-identical `dt` (accumulator pattern), and apply inputs at agreed
  simulation ticks only

```c3
const float TICK_DT = 1.0f / 60.0f;
float accumulator;

// Game loop: fixed simulation ticks, never the variable frame time
accumulator += frame_time;
while (accumulator >= TICK_DT) {
    apply_player_inputs(&world, current_tick); // same inputs on every peer
    world.run_step(TICK_DT, step_count: 4);
    accumulator -= TICK_DT;
    current_tick++;
}

// Desync detection: exchange hashes with peers (cheap, every frame if you like)
ulong hash = world.state_hash();

// Rollback netcode: snapshot, simulate ahead, restore and replay on a
// mispredicted input — the replay is bit-exact
WorldSnapshot snap = world.snapshot();
defer snap.free();
// ... simulate ahead, receive authoritative inputs ...
world.restore(&snap)!!;
```

## Tests

```sh
c3c test                                       # every test, leak-tracked
c3c test --test-noleak                         # faster, and blind to ownership bugs
```

Tests live in `test/`, one module per area (`collision_tests::world`,
`::softbody`, `::voxel`, `::hull`, …) and are listed under `test-sources` in
`project.json`, so `c3c test` is the only thing that builds them.



## Links

- [Example with Vulkan and GLTF](https://github.com/tonis2/game.c3/blob/main/main.c3)
- [C3 Language](https://c3-lang.org/)

---


https://github.com/user-attachments/assets/d93ff7c8-d79b-4c2a-a527-667454feb942



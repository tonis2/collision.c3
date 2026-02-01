# Collision library

A physics and collision detection library for the [C3 language](https://c3-lang.org/).

## Features

**Collision Detection**
- GJK (Gilbert-Johnson-Keerthi) narrow-phase collision
- EPA (Expanding Polytope Algorithm) for contact points and penetration depth
- Spatial hashing for broad-phase optimization
- BVH (Bounding Volume Hierarchy) for triangle mesh collision

**Physics Simulation**
- Rigid body dynamics with mass and inertia
- Impulse-based collision resolution with friction and restitution
- Gravity, linear/angular damping, and sleep states
- Generic 6-DOF joint constraints with limits

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

## Quick Start

```c3
import collision;

fn void main() {
    // Create physics world with default settings
    PhysicsWorld world = collision::DEFAULT_PHYSICS_WORLD;
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

// Capsule (pill shape)
Capsule capsule = { .height = 2.0, .radius_top = 0.5, .radius_bottom = 0.5 };

// Box from half-extents
Aabb3 box = collision::aabb_from_half({1, 1, 1});  // 2x2x2 box

// Cylinder
Cylinder cylinder = { .height = 2.0, .radius_top = 0.5, .radius_bottom = 0.5 };

// Triangle mesh with BVH
Mesh mesh = collision::create_mesh(vertices, indices);
mesh.build_bvh();
defer mesh.free();
```

## Spatial Hash (Broad Phase)

```c3
SpatialHash3D spatial_map = { .cell_size = 2.0 };
defer spatial_map.free();

// Insert AABBs
spatial_map.insert(aabb1, id: 0)!!;
spatial_map.insert(aabb2, id: 1)!!;

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

CollisionInfo info = collision::check_collision(&shape_a, &shape_b);
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

## Configuration

```c3
PhysicsWorld world = {
    .gravity = {0, 0, -9.8},       // Gravity vector
    .spatial_map.cell_size = 2.0,  // Broad-phase cell size
    .sleep_timer = 5.0,            // Seconds before bodies sleep
    .linear_dampening = 0.9,       // Velocity damping per step
    .angular_dampening = 0.9,
    .sleep_delta = 0.1,            // Velocity threshold for sleep
};
```

## Links

- [Example with Vulkan and GLTF](https://github.com/tonis2/game.c3/blob/main/main.c3)
- [C3 Language](https://c3-lang.org/)

---


https://github.com/user-attachments/assets/d93ff7c8-d79b-4c2a-a527-667454feb942



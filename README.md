
# Collision detection library for [C3 language](https://c3-lang.org/)


#### Contains algorithms needed for detecting collision on 3D meshes

Currently contains

* SpatialHash Grid (Broad collision detection)
* GJK (Narrow collision detection)
* EPA (Contact resolving)
* AABB
* BVH for triangles (terrain collision)
* Physics world resolver



-------

https://github.com/tonis2/collision.c3/raw/refs/heads/main/example/collider.mp4




#### Code examples

Import the library

```
import collision;
```

```c
    
    Vec3f[] cube = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
    };

    ConvexPolyhedron shape_1 = (ConvexPolyhedron){cube};

    // Add transforms
    TransformedConvex transformed_shape_1 = {
        .shape = &shape_1, 
        .position = {-2.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation {0.5, 0, 0, 1}
    };
    
    TransformedConvex transformed_shape_2 = {
        .shape = &shape_1, 
        .position = {2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };

    // Check collision
    assert(!collision::check_collision(&transformed_shape_1, &transformed_shape_2).collided);
```


Use spatial hashmap, to optimize which meshes to detect collision on.

```c
    
    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb3[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    // Add AABB boxes to the map
    foreach (usz i, item: boxes) spatial_map.insert({.min = item.min, .max = item.max, .id = i })!!;

    spatial_map.@get_pairs(;Pair pair) { 
        // Detect collsion for pairs that are close to eachother    
    };
```


Basic AABB collision detecting
```c

    Aabb3 box = {
        .min = {1,1,1}, 
        .max = {2.5,2.5,2.5},
    };

    Aabb3 box2 = {
        .min = {2,2,2}, 
        .max = {3,3,3},
    };

    // Transform a box based on Matrix4
    box.transform(MATRIX4F_IDENTITY);

    // Detect if collides
    if (first.aabb.collides(second.aabb)) {}

```


Physics world resolver basic example is [here](https://github.com/tonis2/collision.c3/blob/main/example/main.c3) 

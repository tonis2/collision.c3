
# Collision detection library for [C3 language](https://c3-lang.org/)


#### Contains algorithms needed for detecting collision on 3D meshes

Currently contains

* SpatialHash Grid
* GJK
* EPA
* AABB
* BVH for triangles (terrain collision)


#### Examples

Import the library 
```
import collision;
```

Detect collision for vertices array with [GJK algorithm](https://en.wikipedia.org/wiki/Gilbert%E2%80%93Johnson%E2%80%93Keerthi_distance_algorithm)

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

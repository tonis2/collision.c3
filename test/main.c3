module collision_tests @test;

import collision;

import std::math;
import std::io;

alias Vec2f @private = float[<2>];
alias Vec3f @private = float[<3>];
alias Vec4f @private = float[<4>];

fn void test_physics() {
    Aabb3 aabb = {
        {-1.0, -1.0, -1.0},
        {1.0, 1.0, 1.0}
    };

    Matrix4f matrix = MATRIX4F_IDENTITY.translate({1, 5, 1}).scale({3.0, 1.0, 1.0});

    aabb = aabb.transform(matrix);
    assert(aabb.min == {-2, 4, 0});
    assert(aabb.max == {4, 6, 2});

    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb3[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    foreach (usz i, item: boxes) spatial_map.insert({.min = item.min, .max = item.max, .id = i })!!;
    spatial_map.@get_pairs(;Pair pair) { assert(pair.first == 0 && pair.second == 1); };

    ///
    Vec3f[] cube = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
    };

    ConvexPolyhedron shape_1 = (ConvexPolyhedron){cube};

    TransformedConvex transformed_shape_1 = {
        .shape = &shape_1, 
        .position = {-2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedConvex transformed_shape_2 = {
        .shape = &shape_1, 
        .position = {2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };

    TransformedConvex transformed_shape_3 = {
        .shape = &shape_1, 
        .position = {0.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedConvex transformed_shape_4 = {
        .shape = &shape_1, 
        .position = {0, 0, 0}, 
        .scale = {3, 3, 3},
    };

    TransformedConvex transformed_shape_5 = {
        .shape = &shape_1, 
        .position = {-1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0.5, 0, 1 }
    };
    
    TransformedConvex transformed_shape_6 = {
        .shape = &shape_1, 
        .position = {1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, -0.5, 0, 1 }
    };

    assert(!collision::check_collision(&transformed_shape_1, &transformed_shape_2).collided);
    assert(collision::check_collision(&transformed_shape_3, &transformed_shape_4).collided);
    assert(!collision::check_collision(&transformed_shape_5, &transformed_shape_6).collided);
    
    // CollisionInfo epa_result = epa(collision_result.simplex, &shape_1, &shape_2);

    // {
    //     Aabb3 aabb = {
    //         {3.486510, 1693.368896, 2.556290},
    //         {3.537758, 1698.706421, 2.556290}
    //     };
    // }
}

fn void test_ray() {
     
     Ray testRay = {{0.0, 0.0, 0.0}, (Vec3f){0.0, 0.0, 1}.normalize()};

     Sphere testSphere = {{0, 0, 5}, 1.0};
     assert(testSphere.intersect(testRay) > 0);

    //  Aabb3 testAABB = {{-1, -1, 4}, {1, 1, 6}};

    //  assert(testAABB.intersect(testRay));
}


fn void test_flow() {

    // Create grid map
    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb3[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    Vec3f[] cube = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
    };

    // Store AABB boxes in spatial map, so you don't need to loop over all the boxes, then detecting collision
    foreach (usz i, item: boxes) spatial_map.insert({.min = item.min, .max = item.max, .id = i })!!;


    // Loop over the AABB pairs that are in same grid are
    spatial_map.@get_pairs(;Pair pair) {
        // Test the collisions
        ConvexPolyhedron shape_1 = (ConvexPolyhedron){cube};

        TransformedConvex transformed_shape_1 = {
            .shape = &shape_1, 
            .position = {-2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };
        
        TransformedConvex transformed_shape_2 = {
            .shape = &shape_1, 
            .position = {2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };

         assert(!collision::check_collision(&transformed_shape_1, &transformed_shape_2).collided);
     };
}


fn void test_octree() {
    Aabb3 scene_boundary = {{-100, -100, -100}, {100, 100, 100}};

    Octree octree = {.boundary = scene_boundary, .capacity = 4};
    defer octree.free();



}
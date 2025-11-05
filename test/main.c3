module collision_tests @test;

import collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec3 = double[<3>];
alias Aabb = Aabb3 {double};
alias SpatialHash = SpatialHash3D {double};
alias ConvexPolyhedron = ConvexPolyhedron3D {double};
alias TransformedShape = TransformedConvex {double};

fn void test_physics() {
    Aabb aabb = {
        {-1.0, -1.0, -1.0},
        {1.0, 1.0, 1.0}
    };

    aabb = aabb.transform(translation: {1, 5, 1}, scale: {3.0, 1.0, 1.0}, rotation: {0,0,0,1});
    
    assert(aabb.min == {-2, 4, 0});
    assert(aabb.max == {4, 6, 2});

    SpatialHash spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    foreach (usz i, item: boxes) spatial_map.insert(item, i)!!;

    spatial_map.@get_pairs(;Pair{double} pair) { 
        assert(pair.first == 0 && pair.second == 1);
     };

    Vec3[] cube = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
    };

    ConvexPolyhedron shape_1 = (ConvexPolyhedron){cube};

    TransformedShape transformed_shape_1 = {
        .shape = &shape_1, 
        .translation = {-2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedShape transformed_shape_2 = {
        .shape = &shape_1, 
        .translation = {2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };

    TransformedShape transformed_shape_3 = {
        .shape = &shape_1, 
        .translation = {0.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedShape transformed_shape_4 = {
        .shape = &shape_1, 
        .translation = {0, 0, 0}, 
        .scale = {3, 3, 3},
    };

    TransformedShape transformed_shape_5 = {
        .shape = &shape_1, 
        .translation = {-1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0.5, 0, 1 }
    };
    
    TransformedShape transformed_shape_6 = {
        .shape = &shape_1, 
        .translation = {1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, -0.5, 0, 1 }
    };

    assert(!collision::check_collision{double}(&transformed_shape_1, &transformed_shape_2).collided);
    assert(!collision::check_collision{double}(&transformed_shape_5, &transformed_shape_6).collided);
    // assert(collision::check_collision(&transformed_shape_3, &transformed_shape_4).collided);

    
    //CollisionInfo epa_result = epa(collision_result.simplex, &shape_1, &shape_2);

    // {
    //     Aabb3 aabb = {
    //         {3.486510, 1693.368896, 2.556290},
    //         {3.537758, 1698.706421, 2.556290}
    //     };
    // }
}

fn void test_ray() {
     
     Ray{double} testRay = {{0.0, 0.0, 0.0}, (Vec3){0.0, 0.0, 1}.normalize()};

     Sphere{double} testSphere = {{0, 0, 5}, 1.0};
     assert(testSphere.intersect(testRay) > 0);
}


fn void test_aabb() {
    Aabb box = collision::aabb_from_half({1,1,1});     
    collision::Matrix3{double} intertia = box.inertia_tensor(3.0);

    collision::Matrix3{double} result = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    assert(intertia == result);
}


fn void test_flow() {

    // Create grid map
    SpatialHash spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    Vec3[] cube = {
        {-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5}, {-0.5, 0.5, -0.5},
        {-0.5, -0.5, 0.5}, {0.5, -0.5, 0.5}, {0.5, 0.5, 0.5}, {-0.5, 0.5, 0.5}
    };

    // Store AABB boxes in spatial map, so you don't need to loop over all the boxes, then detecting collision
    foreach (usz i, item: boxes) spatial_map.insert(item, i)!!;


    // Loop over the AABB pairs that are in same grid are
    spatial_map.@get_pairs(;Pair{double} pair) {
        // Test the collisions
        ConvexPolyhedron shape_1 = (ConvexPolyhedron){cube};

        TransformedShape transformed_shape_1 = {
            .shape = &shape_1, 
            .translation = {-2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };
        
        TransformedShape transformed_shape_2 = {
            .shape = &shape_1, 
            .translation = {2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };

         assert(!collision::check_collision{double}(&transformed_shape_1, &transformed_shape_2).collided);
     };
}


fn void test_bvh() {
    Triangle{double}[] terrain = {
        {{-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}},
        {{1, 0, -1}, {1, 0, 1}, {-1, 0, 1}},
        {{1, 0, -1}, {3, 1, -1}, {1, 0, 1}},
        {{3, 1, -1}, {3, 1, 1}, {1, 0, 1}}
    };

    BVHNode{double} bvh;

    bvh.build_recursive(terrain);

    Ray{double} test_ray = {
        .origin = {0, 5, 0},
        .direction = {0, -1, 0}
    };

    Aabb box = {
        .min = {3, 1, 1},
        .max = {4, 3, 3}
    };

    HitInfo{double} hit;
    bool result = bvh.ray_intersects(test_ray, &hit);
    bool result_2 = bvh.box_intersects(box, &hit);

    assert(result == true);
    assert(result_2 == true);
}
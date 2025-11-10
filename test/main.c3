module collision_tests @test;

import collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec2 = float[<2>];
alias Vec3 = float[<3>];
alias Vec4 = float[<4>];

alias Vertices = List{Vec4};
alias Indices = List{usz};

fn void test_physics() {
    Aabb3 aabb = {
        {-1.0, -1.0, -1.0},
        {1.0, 1.0, 1.0}
    };

    aabb = aabb.transform(translation: {1, 5, 1}, scale: {3.0, 1.0, 1.0}, rotation: {0,0,0,1});

    assert(aabb.min == {-2, 4, 0});
    assert(aabb.max == {4, 6, 2});

    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb3[] boxes = {
        {{0,0,0}, {1,1,1}},
        {{1,1,1}, {2.5,2.5,2.5}},
        {{5,5,5}, {6,6,6}},
    };

    foreach (usz i, item: boxes) spatial_map.insert(item, i)!!;

    spatial_map.@get_pairs(;Pair pair) { 
        assert(pair.first == 0 && pair.second == 1);
     };

    Vec4[] cube = {
        {-0.5, -0.5, -0.5, 0}, {0.5, -0.5, -0.5, 0}, {0.5, 0.5, -0.5, 0}, {-0.5, 0.5, -0.5, 0},
        {-0.5, -0.5, 0.5, 0}, {0.5, -0.5, 0.5, 0}, {0.5, 0.5, 0.5, 0}, {-0.5, 0.5, 0.5, 0}
    };

    ConvexPolyhedron shape_1 = (ConvexPolyhedron){.vertices = cube, .indices = {}};

    TransformedConvex transformed_shape_1 = {
        .shape = &shape_1, 
        .translation = {-2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedConvex transformed_shape_2 = {
        .shape = &shape_1, 
        .translation = {2.0, 0, 0}, 
        .scale = {1, 1, 1},
    };

    TransformedConvex transformed_shape_3 = {
        .shape = &shape_1, 
        .translation = {0.0, 0, 0}, 
        .scale = {1, 1, 1},
    };
    
    TransformedConvex transformed_shape_4 = {
        .shape = &shape_1, 
        .translation = {0, 0, 0}, 
        .scale = {3, 3, 3},
    };

    TransformedConvex transformed_shape_5 = {
        .shape = &shape_1, 
        .translation = {-1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0.5, 0, 1 }
    };
    
    TransformedConvex transformed_shape_6 = {
        .shape = &shape_1, 
        .translation = {1.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, -0.5, 0, 1 }
    };

    assert(!collision::check_collision(&transformed_shape_1, &transformed_shape_2).collided);
    assert(!collision::check_collision(&transformed_shape_5, &transformed_shape_6).collided);
    //assert(collision::check_collision(&transformed_shape_3, &transformed_shape_4).collided);

    
    //CollisionInfo epa_result = epa(collision_result.simplex, &shape_1, &shape_2);

    // {
    //     Aabb3 aabb = {
    //         {3.486510, 1693.368896, 2.556290},
    //         {3.537758, 1698.706421, 2.556290}
    //     };
    // }
}

fn void test_ray() {
     
     Ray testRay = {{0.0, 0.0, 0.0}, (Vec3){0.0, 0.0, 1}.normalize()};

     Sphere testSphere = {{0, 0, 5}, 1.0};
     assert(testSphere.intersect(testRay) > 0);
}


fn void test_aabb() {
    Aabb3 box = collision::aabb_from_half({1,1,1});     
    Matrix3f intertia = box.inertia_tensor(3.0);

    Matrix3f result = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    assert(intertia == result);
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

    Vec4[] cube = {
        {-0.5, -0.5, -0.5, 0}, {0.5, -0.5, -0.5, 0}, {0.5, 0.5, -0.5, 0}, {-0.5, 0.5, -0.5, 0},
        {-0.5, -0.5, 0.5, 0}, {0.5, -0.5, 0.5, 0}, {0.5, 0.5, 0.5, 0}, {-0.5, 0.5, 0.5, 0}
    };

    // Store AABB boxes in spatial map, so you don't need to loop over all the boxes, then detecting collision
    foreach (usz i, item: boxes) spatial_map.insert(item, i)!!;


    // Loop over the AABB pairs that are in same grid are
    spatial_map.@get_pairs(;Pair pair) {
        // Test the collisions
        ConvexPolyhedron shape_1 = (ConvexPolyhedron){.vertices = cube, .indices = {}};

        TransformedConvex transformed_shape_1 = {
            .shape = &shape_1, 
            .translation = {-2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };
        
        TransformedConvex transformed_shape_2 = {
            .shape = &shape_1, 
            .translation = {2.0, 0, 0}, 
            .scale = {1, 1, 1},
        };

         assert(!collision::check_collision(&transformed_shape_1, &transformed_shape_2).collided);
     };
}


fn void test_bvh() {
    Triangle[] terrain = {
        {{-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}},
        {{1, 0, -1}, {1, 0, 1}, {-1, 0, 1}},
        {{1, 0, -1}, {3, 1, -1}, {1, 0, 1}},
        {{3, 1, -1}, {3, 1, 1}, {1, 0, 1}}
    };

    BVHNode bvh;

    bvh.build_recursive(terrain);

    Ray test_ray = {
        .origin = {0, 5, 0},
        .direction = {0, -1, 0}
    };

    Aabb3 box = {
        .min = {3, 1, 1},
        .max = {4, 3, 3}
    };

    HitInfo hit;
    bool result = bvh.ray_intersects(test_ray, &hit);
    bool result_2 = bvh.box_intersects(box, &hit);

    assert(result == true);
    // assert(result_2 == true);
}


fn void test_remesh() {
    Vertices vertices;
    Indices indices;

    vertices.push_all({
        {0.0f, 0.0f, 0.0f, 0},
        {1.0f, 0.0f, 0.0f, 0},
        {0.0f, 1.0f, 0.0f, 0},
        {1.0f, 0.0f, 0.0f, 0},
        {1.0f, 1.0f, 0.0f, 0},
        {0.0f, 1.0f, 0.0f, 0}
    });

    indices.push_all({0, 1, 2, 3, 4, 5});

    collision::weld_remesh(vertices: &vertices, indices: &indices);

    assert(vertices.len() == 4);
    assert(indices.len() == 6);
}

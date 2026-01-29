module collision_tests @test;

import collision;

import std::math;
import std::io;
import std::collections::list;

alias Vec2 = float[<2>];
alias Vec3 = float[<3>];
alias Vec4 = float[<4>];

alias Vertices = List{Vec3};
alias Indices = List{ushort};

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

    Vec3[] cube = {
        {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1},
        {1, 0, -1}, {1, 0, 1}, {-1, 0, 1},
        {-1, -1, 0}, {1, -1, 0}, {0, 1, 0},
        {3, 1, -1}, {3, 1, 1}, {1, 0, 1}
    };

    ConvexPolyhedron shape_1 = {.vertices = cube.ptr, .count = cube.len};

    TransformedShape transformed_shape_1 = {
        .shape = &shape_1, 
        .translation = {-2.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0, 0, 1 }
    };
    
    TransformedShape transformed_shape_2 = {
        .shape = &shape_1, 
        .translation = {2.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0, 0, 1 }
    };

    TransformedShape transformed_shape_3 = {
        .shape = &shape_1, 
        .translation = {0.0, 0, 0}, 
        .scale = {1, 1, 1},
        .rotation = {0, 0, 0, 1 }
    };
    
    TransformedShape transformed_shape_4 = {
        .shape = &shape_1, 
        .translation = {0, 0, 0}, 
        .scale = {3, 3, 3},
        .rotation = {0, 0, 0, 1 }
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

    assert(!collision::check_convex_collision(&transformed_shape_1, &transformed_shape_2).collided);
    assert(collision::check_convex_collision(&transformed_shape_5, &transformed_shape_6).collided);
    assert(collision::check_convex_collision(&transformed_shape_3, &transformed_shape_4).collided);

    
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

fn void test_bvh() {
    Vec3[] terrain = {
        {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1},
        {1, 0, -1}, {1, 0, 1}, {-1, 0, 1},
        {-1, -1, 0}, {1, -1, 0}, {0, 1, 0},
        {3, 1, -1}, {3, 1, 1}, {1, 0, 1}
    };

    Vec3[] terrain2 = {
        {0,0,0}, {1,0,0}, {0,1,0},
        {10,10,10}, {11,10,10}, {10,11,10},
        {-10,-10,-10}, {-9,-10,-10}, {-10,-9,-10},
    };

    BVHNode* bvh = collision::create_bvh_tree(terrain.ptr, terrain.len);
    BVHNode* bvh2 = collision::create_bvh_tree(terrain2.ptr, terrain2.len);

    defer {
        bvh.free();
        mem::free(bvh);

        bvh2.free();
        mem::free(bvh2);
    };

    Triangle triangle = {
        {-1.0f, -1.0f, 0.0f}, 
        {1.0f, -1.0f, 0.0f}, 
        {0.0f,  1.0f, 0.0f}
    };

    Ray test_ray = {
        .origin = {0.0f, 0.0f, 5.0f},
        .direction = {0, 0, -1}
    };

    Aabb3 box = {
        .min = {-1.0f, -1.0f, -1.0f},
        .max = {1.0f,  1.0f,  1.0f}
    };

    Aabb3 box2 = {
        .min = {-0.1f, -0.1f, -0.1f},
        .max = {0.1f, 0.1f, 0.1f}
    };

    Aabb3 box3 = {
        .min = {4.0f, 4.0f, 4.0f},
        .max = {6.0f, 6.0f, 6.0f}
    };

    assert(bvh2.box_intersects(box2).collided);
    assert(bvh2.box_intersects(box3).collided == false);

    assert(triangle.intersect_ray(test_ray).collided);
    // assert(bvh.ray_intersects(test_ray).collided);

    HitInfo result_2 = bvh.box_intersects(box);
    assert(result_2.collided == true);

    Triangle tri_1 = {
        {0.0f, 0.0f, 0.0f},
        {0.5f, 0.0f, 0.0f},
        {0.0f, 0.5f, 0.0f}
    };

    Triangle tri_2 = {
        {1.1f, -2.0f, 0.0f},
        {1.1f,  2.0f, 0.0f},
        {1.1f, 0.0f, 2.0f}
    };

    Triangle tri_3 = {
        {15.5f, 15.5f, 15.5f},
        {-12.0f, 10.0f, 10.0f},
        {12.0f, 10.0f, 10.0f}
    };

    assert(collision::check_convex_collision(&box, &tri_1).collided);
    assert(!collision::check_convex_collision(&box, &tri_2).collided);
    assert(!collision::check_convex_collision(&box, &tri_3).collided);
    // assert(collision::check_convex_collision(&box, &&result_2.triangle.get_aabb()).collided);
}

fn void test_remesh() {
    Vertices vertices;
    Indices indices;

    vertices.push_all({
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {1.0f, 0.0f, 0.0},
        {1.0f, 1.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    });

    indices.push_all({0, 1, 2, 3, 4, 5});

    collision::weld_remesh(vertices: &vertices, indices: &indices);

    assert(vertices.len() == 4);
    assert(indices.len() == 3);
}

fn void test_spatial_hash_update() {
    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    // Insert a box at position (0,0,0) to (1,1,1)
    Aabb3 box1_old = {{0, 0, 0}, {0.9, 0.9, 0.9}};
    spatial_map.insert(box1_old, 0)!!;

    // Insert another box that overlaps
    Aabb3 box2 = {{0.5, 0.5, 0.5}, {1.5, 1.5, 1.5}};
    spatial_map.insert(box2, 1)!!;

    // Verify they form a pair (both in cell 0,0,0)
    usz pair_count = 0;
    spatial_map.@get_pairs(; Pair pair) {
        pair_count++;
        assert(pair.first == 0 && pair.second == 1);
    };
    assert(pair_count == 1);

    // Move box1 to a non-overlapping position
    Aabb3 box1_new = {{5, 5, 5}, {5.9, 5.9, 5.9}};
    spatial_map.update(box1_old, box1_new, 0)!!;

    // Verify no pairs after move
    pair_count = 0;
    spatial_map.@get_pairs(; Pair pair) {
        pair_count++;
    };
    assert(pair_count == 0);
}

fn void test_spatial_hash_update_same_cell() {
    SpatialHash3D spatial_map = { .cell_size = 2.0 };
    defer spatial_map.free();

    // Insert a box
    Aabb3 box_old = {{0.1, 0.1, 0.1}, {0.5, 0.5, 0.5}};
    spatial_map.insert(box_old, 0)!!;

    // Move within the same cell (both positions map to cell 0,0,0)
    Aabb3 box_new = {{0.2, 0.2, 0.2}, {0.6, 0.6, 0.6}};
    spatial_map.update(box_old, box_new, 0)!!;

    // Should still be in the map
    usz count = 0;
    spatial_map.@get_nearby_objects({0.3, 0.3, 0.3}; usz id) {
        if (id == 0) count++;
    };
    assert(count >= 1);
}

fn void test_spatial_hash_update_cross_cells() {
    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    // Insert a box spanning cells (0,0,0) and (1,0,0)
    Aabb3 box_old = {{0.5, 0.5, 0.5}, {1.5, 0.9, 0.9}};
    spatial_map.insert(box_old, 0)!!;

    // Insert a stationary box in cell (2,0,0)
    Aabb3 box_stationary = {{2.1, 0.5, 0.5}, {2.9, 0.9, 0.9}};
    spatial_map.insert(box_stationary, 1)!!;

    // Initially no collision (boxes don't share cells)
    usz pair_count = 0;
    spatial_map.@get_pairs(; Pair pair) {
        pair_count++;
    };
    assert(pair_count == 0);

    // Move box to span cells (1,0,0) and (2,0,0) - now overlaps with stationary box's cell
    Aabb3 box_new = {{1.5, 0.5, 0.5}, {2.5, 0.9, 0.9}};
    spatial_map.update(box_old, box_new, 0)!!;

    // Should now have a pair
    pair_count = 0;
    spatial_map.@get_pairs(; Pair pair) {
        pair_count++;
    };
    assert(pair_count == 1);
}

fn void test_spatial_hash_multiple_updates() {
    SpatialHash3D spatial_map = { .cell_size = 1.0 };
    defer spatial_map.free();

    Aabb3 box = {{0, 0, 0}, {0.9, 0.9, 0.9}};
    spatial_map.insert(box, 0)!!;

    // Perform multiple sequential updates
    for (int i = 1; i <= 5; i++) {
        Aabb3 new_box = {{(float)i, 0, 0}, {(float)i + 0.9, 0.9, 0.9}};
        spatial_map.update(box, new_box, 0)!!;
        box = new_box;
    }

    // Verify the box is now at position (5,0,0)
    usz count = 0;
    spatial_map.@get_nearby_objects({5.5, 0.5, 0.5}; usz id) {
        if (id == 0) count++;
    };
    assert(count >= 1);

    // Verify it's NOT at the old position
    count = 0;
    spatial_map.@get_nearby_objects({0.5, 0.5, 0.5}; usz id) {
        if (id == 0) count++;
    };
    assert(count == 0);
}

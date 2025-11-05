import collision;

fn main() {
    PhysicsWorld physics_world = collision::create_world({ .cell_size = 2.0 });
    defer physics_world.free();

    Aabb3 ground_aabb = collider::aabb_from_half({3, 3, 3});

    Rigidbody ground = {
        .id = 0,
        .aabb = ground_aabb,
        .is_static = true,
        .material = {
            .dynamic_friction = 0.5,
            .static_friction = 0.5,
            .restitution = 0.5,
        },
        .collider = collision::transformed_shape_from(
            translation: {0, 0, 0},
            rotation: {0, 0, 0, 1},
            shape: &ground_aabb
        )
    };

    Rigidbody mesh = {
        .id = 1,
        .aabb = collider::aabb_from_half({1, 1, 1}),
        .is_static = false,
        .material = {
            .dynamic_friction = 0.5,
            .static_friction = 0.5,
            .restitution = 0.5,
        },
        .collider = collision::transformed_shape_from(
            translation: {0, 0, 3},
            rotation: {0, 0, 0, 1},
            shape: &&(Sphere){.radius = 5}
        ),
        .motion = {
            .mass = 1.0,
            .linear_velocity = {0, 0, 0},
            .angular_velocity = {0, 0, 0},
            .linear_dampening = 0.9,
            .angular_dampening = 0.9
        }
    };


    physics_world.add_body(body: ground)!!;
    physics_world.add_body(body: mesh)!!;

    Clock time = clock::now();
    
    bool game_running = true;

     while (game_running) {

        // Move physics world forward one time-step
        physics_world.run_step((Number)time.mark());

        // Get or update data from rigidbody collider
        Rigidbody* rigid_body = physics_world.find_body(1);
        io::printfn("%s", rigid_body.collider.translation);

        // Push the body upward
        rigid_body.apply_linear_impulse({0, 0, 5});


        game_running = false;
     }
}
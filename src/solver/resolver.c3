module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;

const EPSILON @local = 0.001f;

alias CollisionFilter = HashMap{Pair, bool};
alias RigidBodyMap = LinkedHashMap{usz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator{usz, Rigidbody};

struct Constraint {
    Rigidbody* body_a;
    Rigidbody* body_b;

    Vec3* impulse_cache;

    float min;
    float max;
    float bias;
    float effective_mass;
    float friction;
    float bounce;

    usz impulse_offset;

    Vec3 linear_a;
    Vec3 angular_a;

    Vec3 linear_b;
    Vec3 angular_b;
}

struct Collision {
    usz a;
    usz b;
    Vec3 normal;
    Vec3[2] contact;
    float depth;
    Vec3 impulse;
}

enum PhysicsCombine {
    AVERAGE,
    MINIMUM,
    MAXIMUM,
    MULTIPLY
}

struct Material {
    float static_friction;
    float dynamic_friction;
    float restitution;
    PhysicsCombine friction_combine;
    PhysicsCombine restitution_combine;
}

struct Rigidbody {
    inline TransformedShape collider;
    usz id;

    float sleep_timer;

    bool sleeping;
    bool can_sleep;
    bool is_static;
    int impulse_dir;

    Aabb3 aabb;
    Aabb3 transformed_aabb;
    Material material;

    // Motion
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    float mass;
    float inverse_mass;
    float gravity_factor;
    float linear_dampening;
    float angular_dampening;

    Vec3 mass_center;
    //Vec3 inertia_diagonal;
    //QuaternionValue inertia_orientation;
    Matrix3f inertia;
}

struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{Constraint} constraints;
    List{GenericJoint3D} joints;
    SpatialHash3D spatial_map;
    CollisionFilter filters;
    Vec3 gravity;
    float sleep_timer;
}

fn void PhysicsWorld.free(&self) {
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.constraints.free();
    self.filters.free();
    self.joints.free();
}

fn PhysicsWorld create_world(float cell_size = 2.0, Vec3 gravity = {0.0f, 0.0f, -9.8f}, float sleep_timer = 5.0) {
    PhysicsWorld world = {
        .gravity = gravity,
        .spatial_map.cell_size = cell_size,
        .sleep_timer = sleep_timer
    };

    return world;
}

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody*? PhysicsWorld.find_body(self, usz id) => self.bodies.get_ref(id);

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint) {

    // Add collision filter for joints, so they won't collide with eachother
    if (joint.collides == false) {
        self.filters.set({joint.body_a, joint.body_b}, true);
    }

    self.joints.push(joint);
}

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
 @require body.collider.rotation != (QuaternionValue){0,0,0,0} : `Body rotation must not be 0`
 @require body.collider.scale != {0,0,0} : `Body scale must not be 0`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    body.is_static = body.mass == 0;
    body.inverse_mass = !body.is_static ? 1.0f / body.mass : 0;

    body.impulse_dir = (self.gravity * body.gravity_factor).z > 0 ? 1 : -1;

    // Compute AABB from collider
    body.aabb = body.collider.get_aabb();

    body.transformed_aabb = body.get_transformed_aabb();
    if (!body.is_static) body.inertia = body.aabb.scale(body.collider.scale).inertia_tensor(body.mass).inverse()!!;

    if (body.linear_dampening == 0) body.linear_dampening = 1;
    if (body.angular_dampening == 0) body.angular_dampening = 1;

    // Don't let joint bodies sleep for now
    foreach (joint: self.joints) {
        if (joint.body_a == body.id || joint.body_b == body.id) {
            body.can_sleep = false;
        }
    }

    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self) {
    return self.aabb.transform(translation: self.collider.translation, scale: self.collider.scale, rotation: self.collider.rotation);
}

fn Matrix3f Rigidbody.get_inverse_world_inertia(self) {
    if (self.is_static) return MATRIX3F_IDENTITY;
    return self.collider.world_inertia_from_local(self.inertia);
}

fn void Rigidbody.apply_linear_impulse(&self, Vec3 impulse) => self.linear_velocity += impulse * self.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3 impulse) {
    Matrix3f inertia = self.get_inverse_world_inertia();
    self.angular_velocity += inertia * impulse;
}

fn void Rigidbody.apply_impulse(&self, Vec3 impulse, Vec3 point) {
    self.linear_velocity += impulse * self.inverse_mass;
    self.apply_angular_impulse(point.cross(impulse));
}

// fn Vec3 Rigidbody.get_intertia(self, Vec3 point, Vec3 normal) {
//     return (self.get_inverse_world_inertia() * point.cross(normal)).cross(point);
// }

fn float Rigidbody.get_intertia(self, Vec3 point, Vec3 normal) {
    Vec3 ang = point.cross(normal);
    Vec3 accel = self.get_inverse_world_inertia() * ang;
    return accel.dot(ang);
}

fn Vec3 Rigidbody.relative_velocity(self, Vec3 point) => self.linear_velocity + self.angular_velocity.cross(point);

fn bool Rigidbody.is_rotating(self) => self.angular_velocity.sq_magnitude() > 0.1f;
fn bool Rigidbody.is_moving(self) => self.linear_velocity.sq_magnitude() > 0.1f;
fn bool Rigidbody.is_sleeping(self) => self.sleeping;
fn void Rigidbody.stop_sleeping(&self) {
    self.sleeping = false;
    self.sleep_timer = 0;
}
fn bool Rigidbody.is_active(self) => self.sleeping == false && self.is_static == false;


fn bool Rigidbody.has_mesh_bvh(&self) {
    if (self.collider.get_type() == ShapeType.CONVEX) {
        Mesh* mesh = (Mesh*)self.collider.shape;
        return mesh.has_bvh();
    }
    return false;
}

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 20) {

    // Create collisions
    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.find_body(pair.a)!!;
        Rigidbody* body_b = self.find_body(pair.b)!!;
        
        // Check if collision filters have this combo of rigid bodies, if true skip the collision check
        bool has_filter = self.filters.has_key({body_a.id, body_b.id}) == true;

        // Check if aabb boxes collide first
        if (!has_filter && body_a.transformed_aabb.collides(body_b.transformed_aabb)) {

            // If contact happens and one body is asleep, but other is awake, wake up sleeping body
            if (body_a.sleeping && body_b.is_active()) body_a.stop_sleeping();
            if (body_b.sleeping && body_a.is_active()) body_b.stop_sleeping();

            if (body_a.is_active() || body_b.is_active()) {
                // Check for BVH mesh collisions
                bool a_has_bvh = body_a.has_mesh_bvh();
                bool b_has_bvh = body_b.has_mesh_bvh();

                if (a_has_bvh || b_has_bvh) {
                    // BVH-accelerated mesh collision
                    self.check_bvh_collision(body_a, body_b, pair);
                } else {
                    // Check collision with GJK algorithm (simple convex vs convex)
                    CollisionInfo collision_info = collision::check_convex_collision(&body_a.collider, &body_b.collider, epa:true);

                    if (collision_info.collided) {
                        // Create collision, so distance constraint can be created for it later
                        Collision collision = {
                            .a = pair.a,
                            .b = pair.b,
                            .normal = collision_info.normal,
                            .contact = collision_info.contact_points,
                            .depth = collision_info.depth,
                        };

                        self.collisions.push(collision);
                    }
                }
            }
        }
    };

 
    foreach (&collision: self.collisions) {
        // Create normal / distance contraint for collision
        collision.create_constraint(self, time);
    }

    // Create joint constraints
    foreach (&joint: self.joints) {
        joint.create_constraint(self, time);
    }

    // Solve contraints for step counts
    for (usz i; i < step_count; i++) {
        foreach (&contraint: self.constraints) {
            self.solve_constraint(contraint);
        }
    }

    self.collisions.clear();
    self.constraints.clear();

    // Apply the calculated impulses as rotation / position changes, plus do other maintenance
    RigidBodyIterator rigid_bodies = self.bodies.value_iter();
    while (rigid_bodies.next()) {
        Rigidbody* body = rigid_bodies.get()!!;

        // If body is sleeping skip it
        if (!body.is_active()) continue;

        // Apply linear velocity and translation
        body.collider.translation += body.linear_velocity * time;

        // Apply angular velocity as rotation
        Vec3 axis = body.angular_velocity * (time * 0.5);
        if (axis != {0,0,0}) {
            body.collider.rotation = (body.collider.rotation + (QuaternionValue){...axis, 0} * body.collider.rotation).normalize();
        }

        // Apply gravity
        body.apply_linear_impulse(self.gravity * body.gravity_factor * body.mass * time);

        // Apply dampening friction (Wind friction ..etc)
        body.angular_velocity *= math::pow(body.angular_dampening, time);
        body.linear_velocity *= math::pow(body.linear_dampening, time);

        bool rotating = body.is_rotating();
        bool moving = body.is_moving();

        // Update the spatial map AABB boxes,for searching the rigid bodies that are close together 
        if (rotating || moving) {
            Aabb3 new_aabb = body.get_transformed_aabb();
            self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
            body.transformed_aabb = new_aabb;
        }

        // Sleeping is broken cause of joints currently
        if (body.can_sleep == true && moving == false && rotating == false) {
            body.sleep_timer += time;
            if (body.sleep_timer > self.sleep_timer) {
                // body.sleeping = true;
                // body.sleep_timer = 0;
            };
        };
    }
}

// Check collisions using triangle BVH from Mesh
// Queries BVH to find potentially colliding triangles, then runs GJK/EPA
fn void PhysicsWorld.check_bvh_collision(&self, Rigidbody* body_a, Rigidbody* body_b, Pair pair) @local {
    bool a_has_bvh = body_a.has_mesh_bvh();
    bool b_has_bvh = body_b.has_mesh_bvh();

    // Determine which body has the mesh BVH to query
    // Prefer body_a's BVH if both have one
    Rigidbody* mesh_body;
    Rigidbody* other_body;
    bool mesh_is_a;

    if (a_has_bvh) {
        mesh_body = body_a;
        other_body = body_b;
        mesh_is_a = true;
    } else {
        mesh_body = body_b;
        other_body = body_a;
        mesh_is_a = false;
    }

    Mesh* mesh = (Mesh*)mesh_body.collider.shape;

    // Transform other body's AABB to mesh body's local space for BVH query
    Aabb3 local_aabb = other_body.transformed_aabb.inverse_transform(
        mesh_body.collider.translation,
        mesh_body.collider.scale,
        mesh_body.collider.rotation
    );

    // Query BVH for overlapping triangles
    List{TriangleVerts} triangles;
    defer triangles.free();
    mesh.bvh.query_all_triangles(local_aabb, &triangles);

    // Find deepest contact
    Vec3 best_normal;
    Vec3[2] best_contact;
    float max_depth = 0;

    foreach (tri : triangles) {
        // Create mesh from triangle vertices for GJK
        Mesh tri_mesh;
		tri_mesh.vertices.push_all({tri[0], tri[1], tri[2]});
        tri_mesh.triangles.push({0, 1, 2});
        defer tri_mesh.free();

        TransformedShape tri_shape = {
            .translation = mesh_body.collider.translation,
            .scale = mesh_body.collider.scale,
            .rotation = mesh_body.collider.rotation,
            .shape = &tri_mesh,
        };

        // Check collision with correct argument order
        CollisionInfo info;
        if (mesh_is_a) {
            info = collision::check_convex_collision(&tri_shape, &other_body.collider, epa:true);
        } else {
            info = collision::check_convex_collision(&other_body.collider, &tri_shape, epa:true);
        }

        if (info.collided && info.depth > max_depth) {
            max_depth = info.depth;
            best_normal = info.normal;
            best_contact = info.contact_points;
        }
    }

    // Create collision with actual contact points from GJK/EPA
    if (max_depth > 0) {
        self.collisions.push({
            .a = pair.a,
            .b = pair.b,
            .normal = best_normal,
            .contact = best_contact,
            .depth = max_depth,
        });
    }
}


<*
 @require constraint.body_a != constraint.body_b : `Collision bodies should not be same index`
*>
fn void PhysicsWorld.solve_constraint(&self, Constraint* constraint) @local {
    Rigidbody* a = constraint.body_a;
    Rigidbody* b = constraint.body_b;

    Vec3* cache = constraint.impulse_cache;

    // Total velocity diffence (angular + linear)
    float relative_velocity = constraint.relative_velocity();

    // assert(relative_velocity < 100000, "Velocity to high, error with bodies %s %s", a.id, b.id);

    // 2. Calculate Lambda (The corrective impulse magnitude)
    float lambda = -(relative_velocity + constraint.bias + constraint.bounce) * constraint.effective_mass;

    if (constraint.friction != 0) {
        float max_friction = constraint.friction * cache.x;
        constraint.min = -max_friction;
        constraint.max = max_friction;
    }

    // Todo: fix hot starting
    float old_impulse = cache[0][constraint.impulse_offset];

    // Apply min / max impulse constraints to the new impulse
    float new_impulse = math::max(constraint.min, math::min(lambda, constraint.max));

    // Store impulse to it's cache
    cache[0][constraint.impulse_offset] = new_impulse;

    // Apply impulses
    if (!a.is_static) {
        a.apply_linear_impulse(constraint.linear_a * new_impulse);
        a.apply_angular_impulse(constraint.angular_a * new_impulse);
    }

    if (!b.is_static) {
        b.apply_linear_impulse(constraint.linear_b * new_impulse);
        b.apply_angular_impulse(constraint.angular_b * new_impulse);
    }       
}

fn float Constraint.relative_velocity(self) {
    Rigidbody* a = self.body_a;
    Rigidbody* b = self.body_b;
    return self.linear_a.dot(a.linear_velocity) 
            + self.angular_a.dot(a.angular_velocity)
            + self.linear_b.dot(b.linear_velocity)
            + self.angular_b.dot(b.angular_velocity);
}

fn void Collision.create_constraint(&collision, PhysicsWorld* world, float dt) @local {
    Rigidbody* a = world.find_body(collision.a)!!;
    Rigidbody* b = world.find_body(collision.b)!!;
    Vec3 normal = collision.normal.normalize();

    Vec3* impulse_cache = &collision.impulse;

    float inverse_mass_sum = a.inverse_mass + b.inverse_mass;
    float restitution = math::max(a.material.restitution, b.material.restitution);      
    float static_friction = a.material.static_friction * b.material.static_friction;

    Vec3 rA = collision.contact[0] - a.collider.translation;
    Vec3 rB = collision.contact[1] - b.collider.translation;
    
    float beta = 0.2f;
    float slop = 0.01f;
    int impulse_dir = math::min(a.impulse_dir, b.impulse_dir);

    float bias = ((beta / dt) * math::max(collision.depth - slop, 0.0f)) * impulse_dir;

    float normal_inertia;
    if (!a.is_static) normal_inertia += a.get_intertia(rA, -normal);
    if (!b.is_static) normal_inertia += b.get_intertia(rB, normal);

    Vec3 linear_a = -normal;
    Vec3 angular_a = rA.cross(-normal);

    Vec3 linear_b = normal;
    Vec3 angular_b = rB.cross(normal);

    Constraint normal_constraint = {
        .body_a = a,
        .body_b = b,
        .linear_a = linear_a,
        .angular_a = angular_a,
        .linear_b = linear_b,
        .angular_b = angular_b,
        .effective_mass = 1 / (inverse_mass_sum + normal_inertia),
        .min = 0,
        .max = float.max,
        .bias = bias,
        .impulse_cache = impulse_cache,
        .impulse_offset = 0,
    };

    float relative_velocity = normal_constraint.relative_velocity();

    normal_constraint.bounce = (relative_velocity * restitution);

    Vec3 helper = (math::abs(normal.y) > 0.99)
        ? {1.0f, 0.0f, 0.0f}
        : {0.0f, 1.0f, 0.0f};
  
    Vec3 tangent_1 = normal.cross(helper).normalize();
    Vec3 tangent_2 = normal.cross(tangent_1);

    float tangent_inertia;
    if (!a.is_static) tangent_inertia += a.get_intertia(rA, -tangent_1);
    if (!b.is_static) tangent_inertia += b.get_intertia(rB, tangent_1);

    float tangent_inertia_2;
    if (!a.is_static) tangent_inertia_2 += a.get_intertia(rA, -tangent_2);
    if (!b.is_static) tangent_inertia_2 += b.get_intertia(rB, tangent_2);

    // FRICTION
    Constraint tangent_constraint = {
        ...normal_constraint,
        .friction = static_friction,
        .linear_a = -tangent_1,
        .angular_a = rA.cross(-tangent_1),
        .linear_b = tangent_1,
        .angular_b = rB.cross(tangent_1),
        .bias = 0,
        .effective_mass = 1 / (inverse_mass_sum + tangent_inertia),
        .impulse_offset = 1,  
    };
    
    Constraint tangent_constraint_2 = {
        ...normal_constraint,
        .friction = static_friction,
        .linear_a = -tangent_2,
        .angular_a = rA.cross(-tangent_2),
        .linear_b = tangent_2,
        .angular_b = rB.cross(tangent_2),
        .bias = 0,
        .effective_mass = 1 / (inverse_mass_sum + tangent_inertia_2),
        .impulse_offset = 2,
    };

    world.constraints.push_all({normal_constraint, tangent_constraint, tangent_constraint_2});
}
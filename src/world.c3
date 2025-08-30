module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;


struct PhysicsWorld {
    List{Rigidbody} bodies;
    List{Collision} collisions;

    SpatialHash3D spatial_map;
    List{BVHNode} bvh_nodes;
}

struct Collision {
    usz a;
    usz b;
    Vec3f normal;
    Vec3f[2] contact;
    float depth;
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

struct Motion {
    bool is_kinematic;
    float mass;
    float inverse_mass;
    float gravity_factor;

    Vec3f mass_center;
    Vec3f inertia_diagonal;
    Quaternionf inertia_orientation;
    Matrix3f inertia_inverse;

    Vec3f linear_velocity;
    Vec3f angular_velocity;

    Vec3f torque;
    Vec3f force;
}

struct Rigidbody {
    int bvh_node;
    uint id;
    uint spatial_index;

    Vec3f translation;
    Vec3f scale;
    Quaternionf rotation;
    Aabb3 aabb;
    Aabb3 old_aabb;
    
    ConvexShape collider;
    Material material;

    Motion motion;
    bool dirty;
}

fn void PhysicsWorld.free(&self) {
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.bvh_nodes.free();
}

struct PhysicsWorldConfig {
    float cell_size;
}

fn PhysicsWorld create_world(PhysicsWorldConfig config) {
    PhysicsWorld world;
    world.spatial_map.cell_size = config.cell_size;
    return world;
}

<*
 @require self.bodies.len() > 0 : `Must have rigid bodies`
*>
fn Rigidbody* PhysicsWorld.find_body(self, uint id) {
   foreach (&body: self.bodies) {
     if (body.id == id) return body;
   }

   return null;
}

<*
 @require body.collider != null : `Must have collider attached`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    uint spatial_index = (uint)self.bodies.len();
    body.spatial_index = spatial_index;
    body.motion.inverse_mass = body.motion.mass > 0 ? 1.0f / body.motion.mass : 0;
    body.old_aabb = body.get_transformed_aabb();

    if (body.motion.inverse_mass > 0) {
        body.motion.inertia_inverse = body.old_aabb.get_world_invertia(body.motion.mass, body.rotation).inverse()!!;
    }

    self.bodies.push(body);
    self.spatial_map.insert(body.old_aabb, spatial_index)!;
}

fn void? PhysicsWorld.add_bvh_node(&self, Rigidbody body, Aabb3 aabb) {
    self.bodies.push(body);
    self.spatial_map.insert(aabb, body.id)!;
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self) => self.aabb.transform(to_matrix(self.translation, self.scale, self.rotation));

fn void Rigidbody.apply_force(&self, Vec3f force) => self.motion.force += force;
fn void Rigidbody.apply_force_at_point(&self, Vec3f force, Vec3f point) {
    self.motion.force += force;
    Vec3f r = point - self.translation;
    self.motion.torque += r.cross(force);
}


// fn void Rigidbody.apply_inertia(&self, Vec3f force) {}
// fn void Rigidbody.rotate(&self, Quaternionf rotation) {}

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, Vec3f gravity = {0.0f, 0.0f, -9.8f}) {

    // Apply mass
    foreach (&body: self.bodies) {
        body.apply_force(gravity * body.motion.mass);
    }

    foreach (&body: self.bodies) {
        if (body.motion.is_kinematic == false) continue;

        Vec3f linear_acceleration = body.motion.force * body.motion.inverse_mass;
        body.motion.linear_velocity += linear_acceleration * time;
        body.translation += body.motion.linear_velocity * time;

        body.motion.angular_velocity += body.motion.inertia_inverse * body.motion.torque * time;

        body.motion.force = {0, 0, 0};
        body.motion.torque = {0, 0, 0};
    }
    

    // for (uint i; i < self.bodies.len(); i++) {
    //     for (uint j = i+1; j < self.bodies.len(); j++) {
    //             Rigidbody* body_a = self.bodies.get_ref(i);
    //             Rigidbody* body_b = self.bodies.get_ref(j);
        
    //             TransformedConvex transformed_shape_1 = collision::transformed_shape_from(
    //                 shape: body_a.collider, 
    //                 translation: body_a.translation,
    //                 scale: body_a.scale,
    //                 rotation: body_a.rotation
    //             );

    //             TransformedConvex transformed_shape_2 = collision::transformed_shape_from(
    //                 shape: body_b.collider, 
    //                 translation: body_b.translation,
    //                 scale: body_b.scale,
    //                 rotation: body_b.rotation
    //             );

    //             CollisionInfo collision_info = collision::check_collision(&transformed_shape_1, &transformed_shape_2);

    //             if (collision_info.collided) {
    //                 collision::epa(&collision_info, &transformed_shape_1, &transformed_shape_2);

    //                 self.collisions.push({
    //                     body_a,
    //                     body_b,
    //                     collision_info.normal,
    //                     collision_info.contact_points,
    //                     collision_info.depth,
    //                 });

    //                 body_a.dirty = true;
    //                 body_b.dirty = true;
    //             }   
    //     }
    // }

    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.bodies.get_ref(pair.first);
        Rigidbody* body_b = self.bodies.get_ref(pair.second);
  
        TransformedConvex transformed_shape_1 = collision::transformed_shape_from(
            shape: body_a.collider, 
            translation: body_a.translation,
            scale: body_a.scale,
            rotation: body_a.rotation
        );

        TransformedConvex transformed_shape_2 = collision::transformed_shape_from(
            shape: body_b.collider, 
            translation: body_b.translation,
            scale: body_b.scale,
            rotation: body_b.rotation
        );
  
        CollisionInfo collision_info = collision::check_collision(&transformed_shape_1, &transformed_shape_2);
      
        if (collision_info.collided) {
            collision::epa(&collision_info, &transformed_shape_1, &transformed_shape_2);
        
            self.collisions.push({
                pair.first,
                pair.second,
                collision_info.normal,
                collision_info.contact_points,
                collision_info.depth,
            });
        }
    };

    foreach (collision : self.collisions) {
        self.resolve_collision(collision);
    }

    // Update spatial map AABB boxes 
    foreach (&body: self.bodies) {
        if (body.motion.is_kinematic == false) {
            continue;
        };

        // float angle = body.motion.angular_velocity.sq_magnitude();
      
        // if (angle > 0.0001f) {
        //     Vec3f axis = body.motion.angular_velocity.normalize();
        //     body.rotation = body.rotation.rotate_by_axis_angle(axis, angle);
        // }

      
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.old_aabb, new_aabb, body.spatial_index)!!;
        body.old_aabb = new_aabb;
        body.dirty = false;
    }
            
    self.collisions.clear();
}

const float EPSILON = 1e-6f;

<*
 @require collision.a != collision.b : `Collision bodies must not reference same body`
*>
fn void PhysicsWorld.resolve_collision(&self, Collision collision) {
    Rigidbody* a = self.bodies.get_ref(collision.a);
    Rigidbody* b = self.bodies.get_ref(collision.b);

    float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;

    //Correction
    // float slop = 0.01f;
    // float percent = 0.6f;

    Vec3f correction = collision.normal * collision.depth;
    //Vec3f correction = collision.normal * math::max(collision.depth + slop, 0.0f) * percent;

    Vec3f relative_a = collision.contact[0];
    Vec3f relative_b = collision.contact[1];

    a.translation -= correction * a.motion.inverse_mass;
    b.translation += correction * b.motion.inverse_mass;

	// Vec3f angular_velocity_a = a.motion.angular_velocity.cross(relative_a);
    // Vec3f angular_velocity_b = b.motion.angular_velocity.cross(relative_b);

    Vec3f full_velocity_a = a.motion.linear_velocity;
    Vec3f full_velocity_b = b.motion.linear_velocity;

    Vec3f contact_velocity = full_velocity_b - full_velocity_a;
    float impulse_force = contact_velocity.dot(collision.normal);
    if (impulse_force > 0) return;

    float restitution = a.material.restitution + b.material.restitution;

    float j = -(1 + restitution) * impulse_force;
    j /= inverse_mass_sum;

    // Vec3f inertia_a = a.motion.inertia_inverse * relative_a.cross(collision.normal).cross(relative_a);
	// Vec3f inertia_b = b.motion.inertia_inverse * relative_b.cross(collision.normal).cross(relative_b);
	// float angular_effect = (inertia_a + inertia_b).dot(collision.normal);

    Vec3f impulse = collision.normal * j;
    
    if (a.motion.inverse_mass > 0) {
        a.motion.linear_velocity -= impulse * a.motion.inverse_mass; 
        //a.motion.angular_velocity += a.motion.inertia_inverse * ra.cross(impulse);
    }

    if (b.motion.inverse_mass > 0) {
        b.motion.linear_velocity += impulse * b.motion.inverse_mass; 

        // b.motion.angular_velocity += b.motion.inertia_inverse * rb.cross(impulse);
    }
}   
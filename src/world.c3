module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;


Vec3f gravity = {0.0f, -9.81f, 0.0f};

struct PhysicsWorld {
    List{Rigidbody} bodies;
    List{Collision} collisions;

    SpatialHash3D spatial_map;
    List{BVHNode} bvh_nodes;
}

struct Collision {
    Rigidbody* body_a;
    Rigidbody* body_b;
    Vec3f normal;
    Vec3f contact;
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
    Vec4f inertia_orientation;
    Vec3f linear_velocity;
    Vec3f angular_velocity;

    Vec3f torque;
    Vec3f force;

    Matrix3f inverse_invertia;
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
    body.motion.inverse_invertia = body.aabb.get_world_invertia(body.motion.mass, body.rotation);
    body.motion.inverse_mass = body.motion.mass > 0 ? 1.0f / body.motion.mass : 0;

    uint spatial_index = (uint)self.bodies.len();
    body.spatial_index = spatial_index;

    self.bodies.push(body);
    self.spatial_map.insert(body.get_transformed_aabb(), spatial_index)!;
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

fn void Rigidbody.rotate(&self, Quaternionf rotation) {

}

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time) {

    // Apply mass
    foreach (&body: self.bodies) {
        body.apply_force((Vec3f){0, gravity.y + body.motion.gravity_factor, 0} * body.motion.mass);
    }

    foreach (&body: self.bodies) {
        if (body.motion.is_kinematic == false) continue;

        Vec3f linear_acceleration = body.motion.force * body.motion.inverse_mass;
        body.motion.linear_velocity += linear_acceleration * time;
        body.translation += body.motion.linear_velocity * time;
        body.motion.force = {0, 0, 0};
        body.motion.torque = {0, 0, 0};
    }

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
                body_a,
                body_b,
                collision_info.normal,
                collision_info.contact_points[0],
                collision_info.depth,
            });

            body_a.dirty = true;
            body_b.dirty = true;
        }   
    };

    foreach (collision : self.collisions) {
        resolve_collision(collision);
    }

    // Update spatial map AABB boxes 
    foreach (&body: self.bodies) {
        if (body.motion.is_kinematic == false || body.dirty == false) {
            continue;
        };
    
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.old_aabb, new_aabb, body.spatial_index)!!;
        body.old_aabb = new_aabb;
        body.dirty = false;
    }
            
    self.collisions.clear();
}

<*
 @require collision.body_a != null && collision.body_b != null : `Must have both collision bodies attached`
 @require collision.body_a != collision.body_b : `Collision bodies must not reference same body`
*>
fn void resolve_collision(Collision collision) {
    Rigidbody* a = collision.body_a;
    Rigidbody* b = collision.body_b;

    float slop = 0.02f;
    float percent = 0.5f;

    Vec3f correction = collision.normal * math::max(collision.depth + slop, 0.0f) * percent * -1;
    
    if (a.motion.inverse_mass != 0) a.translation += correction * a.motion.inverse_mass;
    if (b.motion.inverse_mass != 0) b.translation -= correction * b.motion.inverse_mass;

    Vec3f rv = b.motion.linear_velocity - a.motion.linear_velocity;
    float velocity_along_normal = rv.dot(collision.normal);

    // Do not resolve if velocities are separating
    if (velocity_along_normal >= 0) return;

    float e = a.material.restitution + b.material.restitution; // Restitution (bounciness)
    float j = -(1 + e) * velocity_along_normal;
    j /= a.motion.inverse_mass + b.motion.inverse_mass;

    Vec3f impulse = collision.normal * j;
    if (a.motion.inverse_mass != 0) a.motion.linear_velocity -= impulse * a.motion.inverse_mass;
    if (b.motion.inverse_mass != 0) b.motion.linear_velocity += impulse * b.motion.inverse_mass;
}
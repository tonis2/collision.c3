module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;


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

        // io::printfn("collided %s %s", collision_info.collided, std::time::now());
        if (collision_info.collided) {
            collision::epa(&collision_info, &transformed_shape_1, &transformed_shape_2);
            
            // io::printfn(" ");
            // io::printf("%s,%s,%s", collision_info.contact_points[0].x, collision_info.contact_points[0].y, collision_info.contact_points[0].z);
            // io::printf(" %s,%s,%s", collision_info.contact_points[1].x, collision_info.contact_points[1].y, collision_info.contact_points[1].z);
            // io::printfn(" ");
            self.collisions.push({
                body_a,
                body_b,
                collision_info.normal,
                collision_info.contact_points,
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
        if (body.motion.is_kinematic == false) {
            continue;
        };

        float angle = body.motion.angular_velocity.sq_magnitude();
      
        if (angle > 0.0001f) {
            Vec3f axis = body.motion.angular_velocity.normalize();
            body.rotation = body.rotation.rotate_by_axis_angle(axis, angle);
        }
    
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.old_aabb, new_aabb, body.spatial_index)!!;
        body.old_aabb = new_aabb;
        body.dirty = false;
    }
            
    self.collisions.clear();
}

const float EPSILON = 1e-6f;

<*
 @require collision.body_a != null && collision.body_b != null : `Must have both collision bodies attached`
 @require collision.body_a != collision.body_b : `Collision bodies must not reference same body`
*>
fn void resolve_collision(Collision collision) {
    Rigidbody* a = collision.body_a;
    Rigidbody* b = collision.body_b;

    float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;

    // Correction
    float slop = 0.01f;
    float percent = 0.8f;

    Vec3f correction = (collision.normal * -1) * math::max(collision.depth + slop, 0.0f) * percent / inverse_mass_sum;

    a.translation += correction * a.motion.inverse_mass;
    b.translation -= correction * b.motion.inverse_mass;

    // Collision resolve
    Vec3f ra = collision.contact[0] - a.translation;
    Vec3f rb = collision.contact[1] - b.translation;

    Vec3f velocity_a = a.motion.linear_velocity + a.motion.angular_velocity.cross(rb);
    Vec3f velocity_b = b.motion.linear_velocity + b.motion.angular_velocity.cross(rb);

    Vec3f relative_velocity = (velocity_b - velocity_a);

    float velocity_along_normal = relative_velocity.dot(collision.normal);

    // // Do not resolve if velocities are separating
    if (velocity_along_normal >= 0) return;

    float denominator = inverse_mass_sum +
                        collision.normal.dot(
                            (a.motion.inertia_inverse * ra.cross(collision.normal)).cross(ra) +
                            (b.motion.inertia_inverse * rb.cross(collision.normal)).cross(rb)
                        );
    
    float restitution = a.material.restitution + b.material.restitution; // Restitution (bounciness)
    float j = ( -(1.0f + restitution ) * velocity_along_normal ) /  denominator;

    // Friction
    Vec3f friction_impulse;
    float static_friction = math::sqrt(a.material.static_friction * b.material.static_friction);

    Vec3f tangent = relative_velocity - relative_velocity.dot(collision.normal) * collision.normal;
    if (tangent.sq_magnitude() > EPSILON) {tangent = tangent.normalize();}
    else {tangent = (Vec3f){0,0,0};};

    float jt = -relative_velocity.dot(tangent);
    jt /= inverse_mass_sum;

    if (math::abs(jt) < j * static_friction) {
        friction_impulse = jt * tangent;
    } else {
        float dynamic_friction = math::sqrt(a.material.dynamic_friction * b.material.dynamic_friction);
        friction_impulse = -j * dynamic_friction * tangent;
    }

    // Motion
    Vec3f impulse = collision.normal * j;

    if (a.motion.inverse_mass > 0) {
        a.motion.linear_velocity -= impulse * a.motion.inverse_mass;
        a.motion.angular_velocity += a.motion.inertia_inverse * ra.cross(impulse);
    }

    if (b.motion.inverse_mass > 0) {
        b.motion.linear_velocity += impulse * b.motion.inverse_mass;
        b.motion.angular_velocity += b.motion.inertia_inverse * rb.cross(impulse);
    }

}   
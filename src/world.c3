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
    float mass;
    float inverse_mass;
    float gravity_factor;

    Vec3f mass_center;
    Vec3f inertia_diagonal;
    Quaternionf inertia_orientation;
    Matrix3f inertia;

    Vec3f linear_velocity;
    Vec3f angular_velocity;

    Vec3f torque;
    Vec3f force;
}

struct Rigidbody {
    int bvh_node;
    uint id;
    uint spatial_index;

    Aabb3 aabb;
    Aabb3 old_aabb;
 

    TransformedConvex collider;
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


fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    uint spatial_index = (uint)self.bodies.len();
    body.spatial_index = spatial_index;
    body.motion.inverse_mass = body.motion.mass > 0 ? 1.0f / body.motion.mass : 0;
    body.old_aabb = body.get_transformed_aabb();

    if (body.motion.inverse_mass > 0) {
        body.motion.inertia = body.get_local_inertia();
    } else {
        body.motion.inertia = MATRIX3F_IDENTITY;
    }

    self.bodies.push(body);
    self.spatial_map.insert(body.old_aabb, spatial_index)!;
}

fn void? PhysicsWorld.add_bvh_node(&self, Rigidbody body, Aabb3 aabb) {
    self.bodies.push(body);
    self.spatial_map.insert(aabb, body.id)!;
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self) => self.aabb.transform(to_matrix(self.collider.translation, self.collider.scale, self.collider.rotation));

fn Matrix3f Rigidbody.get_local_inertia(self) => self.collider.inertia_tensor(self.motion.mass);


fn Matrix3f Rigidbody.get_inverse_world_inertia(self) {
    if (self.is_static()) return MATRIX3F_IDENTITY;
    return self.collider.world_inertia_from_local(self.motion.inertia.inverse()!!);
}

fn void Rigidbody.apply_force(&self, Vec3f force) => self.motion.force += force;
fn void Rigidbody.apply_force_at_point(&self, Vec3f force, Vec3f point) {
    self.motion.force += force;
    Vec3f r = point - self.collider.translation;
    self.motion.torque += r.cross(force);
}


fn void Rigidbody.apply_linear_impulse(&self, Vec3f impulse) => self.motion.linear_velocity += impulse * self.motion.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3f impulse) {
    Matrix3f inertia = self.get_inverse_world_inertia();
    self.motion.angular_velocity += inertia * impulse;
}

fn bool Rigidbody.is_static(self) => self.motion.mass == 0;

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, Vec3f gravity = {0.0f, 0.0f, -9.8f}) {

    // Apply mass
    foreach (&body: self.bodies) {
        if (body.is_static()) continue;

        body.collider.translation += body.motion.linear_velocity * time;

        body.apply_linear_impulse(gravity * body.motion.mass * time);

        float angle = body.motion.angular_velocity.sq_magnitude() * time;
   
        if (angle > 0.001f) {
            body.collider.rotation = QUATERNIONF_IDENTITY.rotate_by_axis_angle(body.motion.angular_velocity.normalize(), angle).normalize();   
        }

        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.old_aabb, new_aabb, body.spatial_index)!!;
        body.old_aabb = new_aabb;
    }

    // for (uint i; i < self.bodies.len(); i++) {
    //     for (uint j = i+1; j < self.bodies.len(); j++) {
    // }

    self.spatial_map.@get_pairs(;Pair pair) {
        Rigidbody* body_a = self.bodies.get_ref(pair.first);
        Rigidbody* body_b = self.bodies.get_ref(pair.second);
  
        CollisionInfo collision_info = collision::check_collision(&body_a.collider, &body_b.collider);
      
        if (collision_info.collided) {
            collision::epa(&collision_info, &body_a.collider, &body_b.collider);

            if (collision_info.collided) {
                self.collisions.push({
                    .a = pair.first,
                    .b = pair.second,
                    .normal = collision_info.normal,
                    .contact = collision_info.contact_points,
                    .depth = collision_info.depth,
                });
            }
        }
    };

    foreach (collision : self.collisions) {
        self.resolve_collision(collision);
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

    // Values
    float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;
    float restitution = a.material.restitution * b.material.restitution;

    if (inverse_mass_sum == 0) return;

    Vec3f ra = collision.contact[0] - a.collider.translation;
    Vec3f rb = collision.contact[1] - b.collider.translation;

    Vec3f relative_velocity = b.motion.linear_velocity + b.motion.angular_velocity.cross(collision.normal) - 
                              a.motion.linear_velocity + a.motion.angular_velocity.cross(collision.normal);

    float impulse_force = relative_velocity.dot(collision.normal);
    if (impulse_force > 0.0f) return;

    float impulse_denominator = inverse_mass_sum + (
        (a.get_inverse_world_inertia() * ra.cross(collision.normal)).cross(ra) +
        (b.get_inverse_world_inertia() * rb.cross(collision.normal)).cross(rb)
    )
    .dot(collision.normal);

    float jn = (-(1.0f + restitution) * impulse_force) / impulse_denominator;
    Vec3f impulse = jn * collision.normal;

    // // Apply normal direction impulse
    if (!a.is_static()) {
        a.apply_linear_impulse(-impulse);
        a.apply_angular_impulse(ra.cross(-impulse));
    };
    if (!b.is_static()) {
        b.apply_linear_impulse(impulse);
        b.apply_angular_impulse(rb.cross(impulse));
    };

    // Correction
    Vec3f correction = collision.normal * collision.depth;
    a.collider.translation -= correction * a.motion.inverse_mass / inverse_mass_sum;
    b.collider.translation += correction * b.motion.inverse_mass / inverse_mass_sum;

    // relative_velocity = b.motion.linear_velocity + b.motion.angular_velocity.cross(collision.normal) - 
    //                     a.motion.linear_velocity + a.motion.angular_velocity.cross(collision.normal);



    // Vec3f tangent_direction = (relative_velocity - (relative_velocity.dot(collision.normal) * collision.normal)).normalize();

    // float static_friction = a.material.static_friction * b.material.static_friction;

    // float jt = (-relative_velocity.dot(tangent_direction)) * inverse_mass_sum;
    // Vec3f friction_impulse = jt * tangent_direction;


    // // Apply friction
    // if (!a.is_static()) {
    //     a.apply_linear_impulse(-friction_impulse);
    //     a.apply_angular_impulse(ra.cross(-friction_impulse));
    // };
    // if (!b.is_static()) {
    //     b.apply_linear_impulse(-friction_impulse);
    //     b.apply_angular_impulse(rb.cross(-friction_impulse));
    // };
    
    //float dynamic_friction = math::sqrt(a.material.dynamic_friction * b.material.dynamic_friction);
    // if (tangent_velocity.sq_magnitude() > EPSILON) {
    //     Vec3f tangent_direction = tangent_velocity.normalize();
        
    //     // float tangent_impulse_denominator = inverse_mass_sum + (
    //     //     (a.get_inverse_world_inertia() * ra.cross(tangent_direction)).cross(ra) +
    //     //     (b.get_inverse_world_inertia() * rb.cross(tangent_direction)).cross(rb)
    //     // )
    //     // .dot(tangent_direction);

    //     
    //     

    //     Vec3f friction_impulse;

    //     friction_impulse = jt * tangent_direction;
    //     // if (math::abs(jt) < jn * static_friction) {
    //     //     friction_impulse = jt * tangent_direction;
    //     // } else {
    //     //     
    //     //     friction_impulse = -jn * dynamic_friction * tangent_direction;



    //     // }


    // }

}   
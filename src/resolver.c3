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
    float linear_dampening;
    float angular_dampening;

    Vec3f mass_center;
    Vec3f inertia_diagonal;
    Quaternionf inertia_orientation;
    Matrix3f inertia;

    Vec3f linear_velocity;
    Vec3f angular_velocity;
}

struct Rigidbody {
    int bvh_node;
    uint id;
    uint spatial_index;

    Aabb3 aabb;
    Aabb3 transformed_aabb;
 
    TransformedConvex collider;
    Material material;

    Motion motion;
    bool dirty;
    bool is_static;
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
    body.motion.inverse_mass = !body.is_static ? 1.0f / body.motion.mass : 0;
    body.transformed_aabb = body.get_transformed_aabb();

    if (!body.is_static) body.motion.inertia = body.collider.inertia_tensor(body.motion.mass).inverse()!!;

    if (body.motion.linear_dampening == 0) body.motion.linear_dampening = 1;
    if (body.motion.angular_dampening == 0) body.motion.angular_dampening = 1;
 
    self.bodies.push(body);
    self.spatial_map.insert(body.transformed_aabb, spatial_index)!;
}

fn void? PhysicsWorld.add_bvh_node(&self, Rigidbody body, Aabb3 aabb) {
    self.bodies.push(body);
    self.spatial_map.insert(aabb, body.id)!;
}

fn Aabb3 Rigidbody.get_transformed_aabb(&self) => self.aabb.transform(to_matrix(self.collider.translation, self.collider.scale, self.collider.rotation));

fn Matrix3f Rigidbody.get_inverse_world_inertia(self) {
    if (self.is_static) return MATRIX3F_IDENTITY;
    return self.collider.world_inertia_from_local(self.motion.inertia);
}

fn void Rigidbody.apply_linear_impulse(&self, Vec3f impulse) => self.motion.linear_velocity += impulse * self.motion.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3f impulse) {
    Matrix3f inertia = self.get_inverse_world_inertia();
    self.motion.angular_velocity += inertia * impulse;
}

fn void Rigidbody.apply_impulse(&self, Vec3f impulse, Vec3f point) {
    self.motion.linear_velocity += impulse * self.motion.inverse_mass;
    self.apply_angular_impulse(point.cross(impulse));
}

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, Vec3f gravity = {0.0f, 0.0f, -9.8f}) {

    uint step_count = 10;
    
    for (usz i; i < step_count; i++) {
        float step_time = time / step_count;

        foreach (&body: self.bodies) {
            if (body.is_static) continue;

            body.collider.translation += body.motion.linear_velocity * step_time;

            // Apply gravity
            body.apply_linear_impulse(gravity * body.motion.mass * step_time);

            Vec3f axis = body.motion.angular_velocity * (step_time * 0.5);

            body.collider.rotation += (Quaternionf){...axis, 0}.mul2(body.collider.rotation);
            body.collider.rotation = body.collider.rotation.normalize();

            body.motion.angular_velocity *= math::pow(body.motion.angular_dampening, step_time);
            body.motion.linear_velocity *= math::pow(body.motion.linear_dampening, step_time);
        }

        self.spatial_map.@get_pairs(;Pair pair) {
            Rigidbody* body_a = self.bodies.get_ref(pair.first);
            Rigidbody* body_b = self.bodies.get_ref(pair.second);

            CollisionInfo collision_info = collision::check_collision(&body_a.collider, &body_b.collider);
        
            if (collision_info.collided) {
                collision::epa(&collision_info, &body_a.collider, &body_b.collider);

                if (collision_info.collided) {
                    Collision collision = {
                        .a = pair.first,
                        .b = pair.second,
                        .normal = collision_info.normal,
                        .contact = collision_info.contact_points,
                        .depth = collision_info.depth,
                    };

                    // self.collisions.push(collision);

                    self.resolve_collision(collision);
                }
            }
        };


        // foreach (collision: self.collisions) self.resolve_collision(collision);
    }


    foreach (&body: self.bodies) {
        if (body.is_static) continue;
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.transformed_aabb, new_aabb, body.spatial_index)!!;
        body.transformed_aabb = new_aabb;
    }

    // for (uint i = 0; i < self.bodies.len(); i++) {
    //     for (uint j = i + 1; j < self.bodies.len(); j++) {
            
    //     }
    // }
               
    self.collisions.clear();
}

<*
 @require collision.a != collision.b : `Collision bodies must not reference same body`
*>
fn void PhysicsWorld.resolve_collision(&self, Collision collision) {
    Rigidbody* a = self.bodies.get_ref(collision.a);
    Rigidbody* b = self.bodies.get_ref(collision.b);

    // Values
    float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;
    float restitution = a.material.restitution + b.material.restitution;
    float static_friction = math::sqrt(a.material.static_friction * b.material.static_friction);

    if (inverse_mass_sum == 0) return;

    Vec3f ra = collision.contact[0] - a.collider.translation;
    Vec3f rb = collision.contact[1] - b.collider.translation;

    Vec3f relative_velocity;

    if (!b.is_static) relative_velocity += b.motion.linear_velocity + b.motion.angular_velocity.cross(rb);
    if (!a.is_static) relative_velocity -= a.motion.linear_velocity + a.motion.angular_velocity.cross(ra);

    float impulse_force = relative_velocity.dot(collision.normal);
    if (impulse_force > 0.0f) return;

    Vec3f inertia_a = (a.get_inverse_world_inertia() * ra.cross(collision.normal)).cross(ra);
    Vec3f inertia_b = (b.get_inverse_world_inertia() * rb.cross(collision.normal)).cross(rb);

    float impulse_denominator = inverse_mass_sum + (inertia_a + inertia_b).dot(collision.normal);

    float jn = -(1.0f + restitution) * impulse_force / (inverse_mass_sum + impulse_denominator);
    Vec3f normal_impulse = jn * collision.normal;

    Vec3f tangent_direction = relative_velocity - collision.normal * relative_velocity.dot(collision.normal);

    if (tangent_direction.sq_magnitude() > 0.001f) {
        tangent_direction = tangent_direction.normalize();
        
        float jt = -relative_velocity.dot(tangent_direction) / (inverse_mass_sum + (inertia_a + inertia_b).dot(tangent_direction));
     
        float max_friction = static_friction * jn;

        jt = math::max(-max_friction, math::min(jt, max_friction));

        Vec3f friction_impulse = jt * tangent_direction;

        if (!a.is_static) a.apply_impulse(-(friction_impulse + normal_impulse), ra);
        if (!b.is_static) b.apply_impulse(friction_impulse + normal_impulse, rb);
    } else {
        if (!a.is_static) a.apply_impulse(-normal_impulse, ra);
        if (!b.is_static) b.apply_impulse(normal_impulse, rb);
    };
    
    float percent = 0.6f;
    float slop = 0.02f;
    float correction_magnitude = math::max(collision.depth - slop, 0.0f) / inverse_mass_sum * percent;

    // Correction
    Vec3f correction = collision.normal * collision.depth;

    if (!a.is_static) a.collider.translation -= correction;
    if (!b.is_static) b.collider.translation += correction;
}   
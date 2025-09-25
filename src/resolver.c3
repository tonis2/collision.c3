module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;


struct PhysicsWorld {
    List{Rigidbody} bodies;
    List{Collision} collisions;
    List{GenericJoint3D} constraints;

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
    //Vec3f inertia_diagonal;
    //Quaternionf inertia_orientation;
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
    self.constraints.free();
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

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D body) => self.constraints.push(body);

fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    uint spatial_index = (uint)self.bodies.len();
    body.spatial_index = spatial_index;

    body.is_static = body.motion.mass == 0;

    body.motion.inverse_mass = !body.is_static ? 1.0f / body.motion.mass : 0;

    body.aabb = body.collider.get_aabb();
    
    body.transformed_aabb = body.get_transformed_aabb();

    // if (!body.is_static) body.motion.inertia = body.collider.inertia_tensor(body.motion.mass).inverse()!!;

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


fn Vec3f Rigidbody.local_to_world(self, Vec3f local) => self.collider.translation + local.rotate_by_quat(self.collider.rotation);

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, Vec3f gravity = {0.0f, 0.0f, -9.8f}, uint step_count = 10) {
    for (usz i; i < step_count; i++) {
        float step_time = time / step_count;

        foreach (&body: self.bodies) {
            if (body.is_static) continue;

            // Apply linear velocity and translation
            body.collider.translation += body.motion.linear_velocity * step_time;

            // Apply angular velocity as rotation
            Vec3f axis = body.motion.angular_velocity * (step_time * 0.5);
            body.collider.rotation += (Quaternionf){...axis, 0}.mul2(body.collider.rotation);
            body.collider.rotation = body.collider.rotation.normalize();

            // Apply gravity
            body.apply_linear_impulse(gravity * body.motion.gravity_factor * body.motion.mass * step_time);

            // Apply dampening (Wind friction ..etc)
            body.motion.angular_velocity *= math::pow(body.motion.angular_dampening, step_time);
            body.motion.linear_velocity *= math::pow(body.motion.linear_dampening, step_time);
        }

        foreach (&joint: self.constraints) {
            // Rigidbody* a = self.find_body(joint.body_a);
            // Rigidbody* b = self.find_body(joint.body_b);

            // Vec3f rA = a.local_to_world(joint.anchor_a);
            // Vec3f rB = b.local_to_world(joint.anchor_b);

            // Vec3f d = rA - rB;

            //io::printfn("%s", d);

            // io::printfn("%s %s", world_anchor_a, world_anchor_b);

            // Quaternionf world_frame_a = joint.frame_a * a.collider.rotation;
            // Quaternionf world_frame_b = joint.frame_b * b.collider.rotation;

            // Vec3f rA = world_anchor_a - a.collider.translation;
            // Vec3f rB = world_anchor_b - b.collider.translation;


            // Jacobian jacobian;

            // // for (usz i; i < joint.limints.len() * 2; i+= 2) {
            // //     Vec3f limit = limit.axes.rotate_by_quat(joint.frame_b);
            // //     if (limit.type == LINEAR) {
            // //         jacobian[i] = limit.transpose();
            // //         jacobian[i + 1] = ;
            // //     }
            // // }

            // foreach (limit: joint.limits) {
            //     b.collider.translation += rB * limit.axes.rotate_by_quat(joint.frame_b);
            //     a.collider.translation += rA * limit.axes.rotate_by_quat(joint.frame_a);
            // }
            // Vec3f rA = joint.anchor_a;
            // Vec3f rB = joint.anchor_b;

            // Vec3f world_a = rA;
            // Vec3f world_b = rB;

            // if (!a.is_static) world_a = a.collider.translation.rotate_by_quat(joint.frame_a) - rA;
            // if (!b.is_static) world_b = b.collider.translation.rotate_by_quat(joint.frame_b) - rB;


            // io::printfn("%s", rB.z);

        
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

                    self.collisions.push(collision);
                }
            }
        };


        foreach (collision: self.collisions) self.resolve_collision(collision);
        self.collisions.clear();
    }

    foreach (&body: self.bodies) {
        if (body.is_static) continue;
        Aabb3 new_aabb = body.get_transformed_aabb();
        self.spatial_map.update(body.transformed_aabb, new_aabb, body.spatial_index)!!;
        body.transformed_aabb = new_aabb;
    }
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

    if (!a.is_static) relative_velocity -= a.motion.linear_velocity + a.motion.angular_velocity.cross(ra);
    if (!b.is_static) relative_velocity += b.motion.linear_velocity + b.motion.angular_velocity.cross(rb);

    float impulse_force = relative_velocity.dot(collision.normal);
    if (impulse_force > 0.0001f) return;

    Vec3f total_inertia;

    if (!a.is_static) total_inertia += (a.get_inverse_world_inertia() * ra.cross(collision.normal)).cross(ra);
    if (!b.is_static) total_inertia += (b.get_inverse_world_inertia() * rb.cross(collision.normal)).cross(rb);

    float impulse_denominator = inverse_mass_sum + total_inertia.dot(collision.normal);

    float jn = -(1.0f + restitution) * impulse_force / impulse_denominator;
    Vec3f normal_impulse = jn * collision.normal;

    Vec3f tangent_direction = relative_velocity - collision.normal * relative_velocity.dot(collision.normal);

    if (tangent_direction.sq_magnitude() > 0.0001f) {
        tangent_direction = tangent_direction.normalize();
        
        float jt = -relative_velocity.dot(tangent_direction) / (inverse_mass_sum + total_inertia.dot(tangent_direction));
     
        float max_friction = static_friction * jn;

        jt = math::max(-max_friction, math::min(jt, max_friction));

        Vec3f friction_impulse = jt * tangent_direction;

        if (!a.is_static) a.apply_impulse(-(friction_impulse + normal_impulse), ra);
        if (!b.is_static) b.apply_impulse(friction_impulse + normal_impulse, rb);
    } else {
        if (!a.is_static) a.apply_impulse(-normal_impulse, ra);
        if (!b.is_static) b.apply_impulse(normal_impulse, rb);
    };
    
    // float percent = 0.6f;
    // float slop = 0.02f;
    // float correction_magnitude = math::max(collision.depth - slop, 0.0f) / inverse_mass_sum * percent;

    // Correction
    Vec3f correction = collision.normal * collision.depth;

    if (!a.is_static) a.collider.translation -= correction;
    if (!b.is_static) b.collider.translation += correction;
}



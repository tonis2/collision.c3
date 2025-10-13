module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;

alias CollisionFilter = HashMap{uint, bool};
alias RigidBodyMap = LinkedHashMap{usz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator{usz, Rigidbody};


struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{GenericJoint3D} constraints;

    SpatialHash3D spatial_map;
    List{BVHNode} bvh_nodes;

    CollisionFilter filters;
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

    Aabb3 aabb;
    Aabb3 transformed_aabb;
 
    TransformedConvex collider;
    Material material;

    Motion motion;
    bool sleeping;
    bool is_static;
}

fn void PhysicsWorld.free(&self) {
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.bvh_nodes.free();
    self.constraints.free();
    self.filters.free();
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
fn Rigidbody*? PhysicsWorld.find_body(self, usz id) => self.bodies.get_ref(id);

fn void? PhysicsWorld.add_constraint(&self, GenericJoint3D joint) {
    if (joint.collides == false) {
        self.filters.set(joint.body_a + joint.body_b, false);
    }

    self.constraints.push(joint);
}

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    body.is_static = body.motion.mass == 0;
    body.motion.inverse_mass = !body.is_static ? 1.0f / body.motion.mass : 0;
    body.aabb = body.collider.get_aabb();
    body.sleeping = false;

    body.transformed_aabb = body.aabb.transform(body.collider.translation, body.collider.scale, body.collider.rotation);
    if (!body.is_static) body.motion.inertia = body.aabb.inertia_tensor(body.motion.mass).inverse()!!;
    
    // if (!body.is_static) body.motion.inertia = body.collider.inertia_tensor(body.motion.mass).inverse()!!;

    if (body.motion.linear_dampening == 0) body.motion.linear_dampening = 1;
    if (body.motion.angular_dampening == 0) body.motion.angular_dampening = 1;

    if (!body.is_static) body.apply_linear_impulse(gravity * body.motion.gravity_factor * body.motion.mass * 0.02f);
    
    self.bodies.set(body.id, body);
    self.spatial_map.insert(body.transformed_aabb, body.id)!;
}

// fn void? PhysicsWorld.add_bvh_node(&self, Rigidbody body, Aabb3 aabb) {
//     self.bodies.push(body);
//     self.spatial_map.insert(aabb, body.id)!;
// }

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

fn Vec3f Rigidbody.get_intertia(self, Vec3f point, Vec3f normal) {
    return (self.get_inverse_world_inertia() * point.cross(normal)).cross(point);
}

fn Vec3f Rigidbody.relative_velocity(self, Vec3f point) {
    return self.motion.linear_velocity + self.motion.angular_velocity.cross(point);
}


fn bool Rigidbody.is_rotating(self) => self.motion.angular_velocity.sq_magnitude() > 0.001f;
fn bool Rigidbody.is_moving(self) => self.motion.linear_velocity.sq_magnitude() > 0.001f;
fn bool Rigidbody.is_sleeping(self) => self.sleeping;


Vec3f gravity @private = {0.0f, 0.0f, -9.8f};

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, float time, uint step_count = 15) {

    RigidBodyIterator rigid_bodies = self.bodies.value_iter();

    // Warm start joints
    foreach (&joint: self.constraints) {
        Rigidbody* a = self.find_body(joint.body_a)!!;
        Rigidbody* b = self.find_body(joint.body_b)!!;

        if (!a.is_static) a.apply_impulse(-joint.accumulated_linear_impulse, a.collider.rotation * joint.anchor_a);
        if (!b.is_static) b.apply_impulse(joint.accumulated_linear_impulse, b.collider.rotation * joint.anchor_b);

        // if (!a.is_static) a.apply_angular_impulse(-joint.accumulated_angular_impulse);
        // if (!b.is_static) b.apply_angular_impulse(joint.accumulated_angular_impulse);
    }

    for (usz i; i < step_count; i++) {
        float time_step = time / step_count;

        while (rigid_bodies.next()) {
            Rigidbody* body = rigid_bodies.get()!!;

            if (body.is_static) continue;
            // Apply linear velocity and translation
            body.collider.translation += body.motion.linear_velocity * time_step;

            // Apply angular velocity as rotation
            Vec3f axis = body.motion.angular_velocity * (time_step * 0.5);

            if (axis != {0,0,0}) {
                body.collider.rotation = (body.collider.rotation + (Quaternionf){...axis, 0}.mul2(body.collider.rotation)).normalize();
            }

            // Apply gravity
            body.apply_linear_impulse(gravity * body.motion.gravity_factor * body.motion.mass * time_step);
        

            // Apply dampening (Wind friction ..etc)
            body.motion.angular_velocity *= math::pow(body.motion.angular_dampening, time_step);
            body.motion.linear_velocity *= math::pow(body.motion.linear_dampening, time_step);
        }

        // Reset rigid body iterator !
        rigid_bodies.started = false;

        // Check spatial map, for broadphase collision and do narrow phase for detected pairs
        self.spatial_map.@get_pairs(;Pair pair) {
            Rigidbody* body_a = self.find_body(pair.first)!!;
            Rigidbody* body_b = self.find_body(pair.second)!!;

            if (true) {
                CollisionInfo collision_info = collision::check_collision(&body_a.collider, &body_b.collider);

                if (collision_info.collided && self.filters.has_key((uint)(pair.first + pair.second)) == false) {
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
            }
        };

        foreach (collision: self.collisions) self.resolve_collision(collision);
        self.collisions.clear();

        // Apply constraints
        foreach (&joint: self.constraints) {
            Rigidbody* a = self.find_body(joint.body_a)!!;
            Rigidbody* b = self.find_body(joint.body_b)!!;
        
            float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;

            Vec3f rA = joint.anchor_a;
            Vec3f rB = joint.anchor_b;

            if (!a.is_static) rA = a.collider.rotation * rA;
            if (!b.is_static) rB = b.collider.rotation * rB;

            Vec3f worldspace_joint;

            if (!a.is_static) worldspace_joint -= a.collider.translation + rA;
            if (!a.is_static) worldspace_joint += b.collider.translation + rB;

            Vec3f relative_velocity;

            if (!a.is_static) relative_velocity -= a.relative_velocity(rA);
            if (!b.is_static) relative_velocity += b.relative_velocity(rB);

            // Linear limits resolving
            if (!joint.linear_limits.is_empty()) {

                foreach (limit: joint.linear_limits) {
                    Vec3f normal = limit.axes;

                    Vec3f total_inertia;

                    if (!a.is_static) total_inertia += a.get_intertia(rA, normal);
                    if (!b.is_static) total_inertia += b.get_intertia(rB, normal);

                    float effective_mass = inverse_mass_sum + total_inertia.dot(normal);
                    if (effective_mass < 0.00001f) return;

                    float contact_velocity = relative_velocity.dot(normal);
                    float position_error = worldspace_joint.dot(normal);

                    float bias = time_step * position_error;

                    float j = -(bias + contact_velocity) / effective_mass;
            
                    Vec3f impulse = j * normal;

                    if (!a.is_static) a.apply_impulse(-impulse, rA);
                    if (!b.is_static) b.apply_impulse(impulse, rB);

                    joint.accumulated_linear_impulse += impulse;
                }
            }

            // Angular limits resolving
            if (!joint.angular_limits.is_empty()) {
                
                Quaternionf rotation_a = a.collider.rotation;
                Quaternionf rotation_b = b.collider.rotation;

                Quaternionf relative_orientation = (rotation_b * rotation_a);
                Vec3f total_axis = relative_orientation.v.xyz;
                float total_angle = relative_orientation.angle();

                foreach LIMITS: (limit: joint.angular_limits) {
                    float position_error = 0.0f;
                    Vec3f normal = limit.axes;

                    float current_angle = total_axis.dot(normal) * total_angle;

                    if (current_angle > limit.max) {
                        position_error = current_angle - limit.max;
                    } else if (current_angle < limit.min) {
                        position_error = current_angle - limit.min;
                    } else {
                        continue LIMITS;
                    }

                    float effective_mass = 0;

                    if (!a.is_static) effective_mass += normal.dot(a.get_inverse_world_inertia() * normal);
                    if (!b.is_static) effective_mass += normal.dot(b.get_inverse_world_inertia() * normal);
                    if (effective_mass < 0.00001f) return;

                    float angular_velocity = (b.motion.angular_velocity - a.motion.angular_velocity).dot(normal);

                    float bias = time_step * position_error * 0.02f;
                    float j = -(angular_velocity + bias) / effective_mass;

                    // float oldAccumulated = joint.accumulated_angular_impulse;
                    // joint.accumulated_angular_impulse = math::max(0.0f, oldAccumulated + j);
                    // float impulseToApply = joint.accumulated_angular_impulse - oldAccumulated;

                    Vec3f angular_impulse = j * normal;

                    if (!a.is_static) a.apply_angular_impulse(-angular_impulse);
                    if (!b.is_static) b.apply_angular_impulse(angular_impulse);
                }   
            }
        }
    }

    while (rigid_bodies.next()) {
        Rigidbody* body = rigid_bodies.get()!!;
        if (!body.is_static) {
            Aabb3 new_aabb;

            bool rotating = body.is_rotating();
            bool moving = body.is_moving();

            bool skip_rotation = body.collider.get_type() == ShapeType.SPHERE;

            // Skip updating AABB when bodies are not in motion
            if (moving || rotating) {
                new_aabb = body.aabb.transform(body.collider.translation, body.collider.scale, body.collider.rotation, skip_rotation: skip_rotation);
            }

            // if (moving == false && rotating == false) {
            //     body.sleeping = true;
            // };

            self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
            body.transformed_aabb = new_aabb;
        };
    };
}

fn void PhysicsWorld.resolve_collision(&self, Collision collision) {
    Rigidbody* a = self.find_body(collision.a)!!;
    Rigidbody* b = self.find_body(collision.b)!!;

    // Values
    float inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;
    float restitution = a.material.restitution + b.material.restitution;
    float static_friction = math::sqrt(a.material.static_friction * b.material.static_friction);

    if (inverse_mass_sum == 0) return;

    Vec3f rA = collision.contact[0] - a.collider.translation;
    Vec3f rB = collision.contact[1] - b.collider.translation;

    Vec3f relative_velocity;

    if (!a.is_static) relative_velocity -= a.relative_velocity(rA);
    if (!b.is_static) relative_velocity += b.relative_velocity(rB);

    float impulse_force = relative_velocity.dot(collision.normal);
    if (impulse_force > 0.0001f) return;

    Vec3f total_inertia;

    if (!a.is_static) total_inertia += a.get_intertia(rA, collision.normal);
    if (!b.is_static) total_inertia += b.get_intertia(rB, collision.normal);

    float impulse_denominator = inverse_mass_sum + total_inertia.dot(collision.normal);

    float jn = -(1.0f + restitution) * impulse_force / impulse_denominator;
    Vec3f normal_impulse = jn * collision.normal;

    Vec3f tangent_direction = relative_velocity - relative_velocity.dot(collision.normal) * collision.normal;
    
    if (tangent_direction.sq_magnitude() > 0.001f) {
        tangent_direction = tangent_direction.normalize();

        Vec3f friction_inertia = {0,0,0};
        if (!a.is_static) friction_inertia += a.get_intertia(rA, tangent_direction);
        if (!b.is_static) friction_inertia += b.get_intertia(rB, tangent_direction);
            
        float jt = -relative_velocity.dot(tangent_direction) / (inverse_mass_sum + friction_inertia.dot(tangent_direction));
     
        float max_friction = static_friction * jn;

        jt = math::clamp(jt, -max_friction, max_friction);

        Vec3f friction_impulse = jt * tangent_direction;

        if (!a.is_static) a.apply_impulse(-(friction_impulse + normal_impulse), rA);
        if (!b.is_static) b.apply_impulse(friction_impulse + normal_impulse, rB);
    } else {
        if (!a.is_static) a.apply_impulse(-normal_impulse, rA);
        if (!b.is_static) b.apply_impulse(normal_impulse, rB);
    };
    
    // float percent = 0.4f;
    // float slop = 0.08f;
    // float correction_magnitude = math::max(collision.depth - slop, 0.0f) / inverse_mass_sum * percent;

    // if (collision.depth > slop) {
    //     // Correction
     
    // }

    Vec3f correction = collision.normal * collision.depth;

    if (!a.is_static) a.collider.translation -= correction;
    if (!b.is_static) b.collider.translation += correction;
}



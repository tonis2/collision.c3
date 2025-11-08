module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::time;
import std::collections::map;

alias CollisionFilter = HashMap{uint, bool};
alias CollisionCache = HashMap{uint, Collision};
alias RigidBodyMap = LinkedHashMap{usz, Rigidbody};
alias RigidBodyIterator = LinkedHashMapValueIterator{usz, Rigidbody};


struct PhysicsWorld {
    RigidBodyMap bodies;
    List{Collision} collisions;
    List{Collision} collision_cache;
    List{GenericJoint3D} constraints;
    CollisionCache collision_cache;

    SpatialHash3D spatial_map;

    CollisionFilter filters;
}

struct Collision {
    usz a;
    usz b;
    Vec3 normal;
    Vec3[2] contact;
    Number depth;
}

enum PhysicsCombine {
    AVERAGE,
    MINIMUM,
    MAXIMUM,
    MULTIPLY
}

fn usz contact_id (usz a, usz b) => (usz)(a.hash() + b.hash()); 

struct Material {
    Number static_friction;
    Number dynamic_friction;
    Number restitution;
    PhysicsCombine friction_combine;
    PhysicsCombine restitution_combine;
}

struct Motion {
    Number mass;
    Number inverse_mass;
    Number gravity_factor;
    Number linear_dampening;
    Number angular_dampening;

    Vec3 mass_center;
    //Vec3 inertia_diagonal;
    //Quaternionf inertia_orientation;
    Matrix3f inertia;

    Vec3 linear_velocity;
    Vec3 angular_velocity;
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

struct PhysicsWorldConfig {
    Number cell_size;
}

fn void PhysicsWorld.free(&self) {
    self.bodies.free();
    self.collisions.free();
    self.spatial_map.free();
    self.constraints.free();
    self.filters.free();
    self.collision_cache.free();
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

fn Aabb3 Rigidbody.get_transformed_aabb(&self) {
    bool skip_rotation = self.collider.get_type() == ShapeType.SPHERE;
    return self.aabb.transform(translation: self.collider.translation, scale: self.collider.scale, rotation: self.collider.rotation, skip_rotation: skip_rotation);
} 

<*
 @require body.collider.shape != null : `Added rigid body must have collision shape`
*>
fn void? PhysicsWorld.add_body(&self, Rigidbody body) {
    body.is_static = body.motion.mass == 0;
    body.motion.inverse_mass = !body.is_static ? 1.0f / body.motion.mass : 0;
    body.aabb = body.collider.get_aabb();
    body.sleeping = false;

    body.transformed_aabb = body.get_transformed_aabb();
    if (!body.is_static) body.motion.inertia = body.aabb.inertia_tensor(body.motion.mass).inverse()!!;

    if (body.motion.linear_dampening == 0) body.motion.linear_dampening = 1;
    if (body.motion.angular_dampening == 0) body.motion.angular_dampening = 1;

    // if (!body.is_static) body.apply_linear_impulse(gravity * body.motion.gravity_factor * body.motion.mass * 0.02f);
    
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

fn void Rigidbody.apply_linear_impulse(&self, Vec3 impulse) => self.motion.linear_velocity += impulse * self.motion.inverse_mass;

fn void Rigidbody.apply_angular_impulse(&self, Vec3 impulse) {
    Matrix3f inertia = self.get_inverse_world_inertia();
    self.motion.angular_velocity += inertia * impulse;
}

fn void Rigidbody.apply_impulse(&self, Vec3 impulse, Vec3 point) {
    self.motion.linear_velocity += impulse * self.motion.inverse_mass;
    self.apply_angular_impulse(point.cross(impulse));
}

fn Vec3 Rigidbody.get_intertia(self, Vec3 point, Vec3 normal) {
    return (self.get_inverse_world_inertia() * point.cross(normal)).cross(point);
}

fn Vec3 Rigidbody.relative_velocity(self, Vec3 point) {
    return self.motion.linear_velocity + self.motion.angular_velocity.cross(point);
}


fn bool Rigidbody.is_rotating(self) => self.motion.angular_velocity.sq_magnitude() > 0.001f;
fn bool Rigidbody.is_moving(self) => self.motion.linear_velocity.sq_magnitude() > 0.001f;
fn bool Rigidbody.is_sleeping(self) => self.sleeping;


Vec3 gravity @private = {0.0f, 0.0f, -9.8f};

<*
 @require self.bodies.len() > 0 : `Must have some bodies to simulate`
*>
fn void PhysicsWorld.run_step(&self, Number time, uint step_count = 15) {

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
        Number time_step = time / step_count;

        while (rigid_bodies.next()) {
            Rigidbody* body = rigid_bodies.get()!!;

            if (body.is_static) continue;
            // Apply linear velocity and translation
            body.collider.translation += body.motion.linear_velocity * time_step;

            // Apply angular velocity as rotation
            Vec3 axis = body.motion.angular_velocity * (time_step * 0.5);

            if (axis != {0,0,0}) {
                body.collider.rotation = (body.collider.rotation + (Quaternionf){...axis, 0} * body.collider.rotation).normalize();
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

            // Check if aabb boxes collide first
            if (body_a.transformed_aabb.collides(body_b.transformed_aabb)) {
                // Check collision with GJK algorithm
                CollisionInfo collision_info = collision::check_collision(&body_a.collider, &body_b.collider);

                if (collision_info.collided && self.filters.has_key((uint)(pair.first + pair.second)) == false) {
                    // Get contact points with EPA
                    collision::epa(&collision_info, &body_a.collider, &body_b.collider);
                    
                    if (collision_info.collided) {
                        Collision collision = {
                            .a = pair.first,
                            .b = pair.second,
                            .normal = collision_info.normal,
                            .contact = collision_info.contact_points,
                            .depth = collision_info.depth,
                        };

                        self.collision_cache.set(pair.hash(), collision);
                        self.collisions.push(collision);
                    }
                }
            }
        };

        foreach (collision: self.collisions) self.resolve_collision(collision);
        self.collisions.clear();
        self.collision_cache.clear();

        // Apply constraints
        foreach (&joint: self.constraints) {
            Rigidbody* a = self.find_body(joint.body_a)!!;
            Rigidbody* b = self.find_body(joint.body_b)!!;
        
            Number inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;

            Vec3 rA = joint.anchor_a;
            Vec3 rB = joint.anchor_b;

            if (!a.is_static) rA = a.collider.rotation * rA;
            if (!b.is_static) rB = b.collider.rotation * rB;

            Vec3 worldspace_joint;

            if (!a.is_static) worldspace_joint -= a.collider.translation + rA;
            if (!a.is_static) worldspace_joint += b.collider.translation + rB;

            Vec3 relative_velocity;

            if (!a.is_static) relative_velocity -= a.relative_velocity(rA);
            if (!b.is_static) relative_velocity += b.relative_velocity(rB);

            // Linear limits resolving
            if (!joint.linear_limits.is_empty()) {

                foreach (limit: joint.linear_limits) {
                    Vec3 normal = limit.axes;

                    Vec3 total_inertia;

                    if (!a.is_static) total_inertia += a.get_intertia(rA, normal);
                    if (!b.is_static) total_inertia += b.get_intertia(rB, normal);

                    Number effective_mass = inverse_mass_sum + total_inertia.dot(normal);
                    if (effective_mass < 0.00001f) return;

                    Number contact_velocity = relative_velocity.dot(normal);
                    Number position_error = worldspace_joint.dot(normal);

                    Number bias = time_step * position_error;

                    Number j = -(bias + contact_velocity) / effective_mass;
            
                    Vec3 impulse = j * normal;

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
                Vec3 total_axis = relative_orientation.v.xyz;
                Number total_angle = relative_orientation.to_angle();

                foreach LIMITS: (limit: joint.angular_limits) {
                    Number position_error = 0.0f;
                    Vec3 normal = limit.axes;

                    Number current_angle = total_axis.dot(normal) * total_angle;

                    if (current_angle > limit.max) {
                        position_error = current_angle - limit.max;
                    } else if (current_angle < limit.min) {
                        position_error = current_angle - limit.min;
                    } else {
                        continue LIMITS;
                    }

                    Number effective_mass = 0;

                    if (!a.is_static) effective_mass += normal.dot(a.get_inverse_world_inertia() * normal);
                    if (!b.is_static) effective_mass += normal.dot(b.get_inverse_world_inertia() * normal);
                    if (effective_mass < 0.00001f) return;

                    Number angular_velocity = (b.motion.angular_velocity - a.motion.angular_velocity).dot(normal);

                    Number bias = time_step * position_error * 0.02f;
                    Number j = -(angular_velocity + bias) / effective_mass;

                    // Number oldAccumulated = joint.accumulated_angular_impulse;
                    // joint.accumulated_angular_impulse = math::max(0.0f, oldAccumulated + j);
                    // Number impulseToApply = joint.accumulated_angular_impulse - oldAccumulated;

                    Vec3 angular_impulse = j * normal;

                    if (!a.is_static) a.apply_angular_impulse(-angular_impulse);
                    if (!b.is_static) b.apply_angular_impulse(angular_impulse);
                }   
            }
        }
    }

    while (rigid_bodies.next()) {
        Rigidbody* body = rigid_bodies.get()!!;
        if (!body.is_static) {
            bool rotating = body.is_rotating();
            bool moving = body.is_moving();

            if (rotating || moving) {
                Aabb3 new_aabb;
                new_aabb = body.get_transformed_aabb();
                self.spatial_map.update(body.transformed_aabb, new_aabb, body.id)!!;
                body.transformed_aabb = new_aabb;
            }
            
            // if (moving == false && rotating == false) {
            //     body.sleeping = true;
            // };
        };
    };
}

<*
 @require collision.a != collision.b : `Collision bodies should not be same index`
*>
fn void PhysicsWorld.resolve_collision(&self, Collision collision) {
    Rigidbody* a = self.find_body(collision.a)!!;
    Rigidbody* b = self.find_body(collision.b)!!;

    // Values
    Number inverse_mass_sum = a.motion.inverse_mass + b.motion.inverse_mass;
    Number restitution = a.material.restitution + b.material.restitution;
    Number static_friction = math::sqrt(a.material.static_friction * b.material.static_friction);

    if (inverse_mass_sum == 0) return;

    Vec3 rA = collision.contact[0] - a.collider.translation;
    Vec3 rB = collision.contact[1] - b.collider.translation;

    Vec3 relative_velocity;

    if (!a.is_static) relative_velocity -= a.relative_velocity(rA);
    if (!b.is_static) relative_velocity += b.relative_velocity(rB);

    Number impulse_force = relative_velocity.dot(collision.normal);
    if (impulse_force > 0.0001f) return;

    Vec3 total_inertia;

    if (!a.is_static) total_inertia += a.get_intertia(rA, collision.normal);
    if (!b.is_static) total_inertia += b.get_intertia(rB, collision.normal);

    Number impulse_denominator = inverse_mass_sum + total_inertia.dot(collision.normal);

    Number jn = -(1.0f + restitution) * impulse_force / impulse_denominator;
    Vec3 normal_impulse = jn * collision.normal;

    Vec3 tangent_direction = relative_velocity - relative_velocity.dot(collision.normal) * collision.normal;
    
    if (tangent_direction.sq_magnitude() > 0.001f) {
        tangent_direction = tangent_direction.normalize();

        Vec3 friction_inertia = {0,0,0};
        if (!a.is_static) friction_inertia += a.get_intertia(rA, tangent_direction);
        if (!b.is_static) friction_inertia += b.get_intertia(rB, tangent_direction);
            
        Number jt = -relative_velocity.dot(tangent_direction) / (inverse_mass_sum + friction_inertia.dot(tangent_direction));
     
        Number max_friction = static_friction * jn;

        jt = math::clamp(jt, -max_friction, max_friction);

        Vec3 friction_impulse = jt * tangent_direction;

        if (!a.is_static) a.apply_impulse(-(friction_impulse + normal_impulse), rA);
        if (!b.is_static) b.apply_impulse(friction_impulse + normal_impulse, rB);
    } else {
        if (!a.is_static) a.apply_impulse(-normal_impulse, rA);
        if (!b.is_static) b.apply_impulse(normal_impulse, rB);
    };
    
    Number percent = 0.4f;
    Number slop = 0.02f;
    Number correction_magnitude = math::max(collision.depth - slop, 0.0f) / inverse_mass_sum * percent;

    Vec3 correction = collision.normal * correction_magnitude;

    if (!a.is_static) a.collider.translation -= correction;
    if (!b.is_static) b.collider.translation += correction;
}



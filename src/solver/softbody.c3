module collision;

import std::collections::list;
import std::collections::map;
import std::math;
import std::io;
import std::core::mem;

// XPBD constraint types for soft bodies
struct DistanceConstraint {
    usz p0;          // Particle index 0
    usz p1;          // Particle index 1
    float rest_length;
    float compliance; // Inverse stiffness (0 = infinitely stiff)
    float lambda;     // Lagrange multiplier (warm-starting)
}

struct VolumeConstraint {
    float rest_volume;
    float compliance;
    float lambda;
}

struct BendingConstraint {
    usz p0;           // Shared edge vertex 0
    usz p1;           // Shared edge vertex 1
    usz p2;           // Triangle 1 opposite vertex
    usz p3;           // Triangle 2 opposite vertex
    float rest_angle; // Rest dihedral angle
    float compliance;
    float lambda;
}

// Collision constraint generated per-frame during soft-rigid narrow phase
struct SoftCollisionConstraint {
    usz particle;        // Particle index
    Vec3 normal;         // Contact normal (pointing away from rigidbody)
    float depth;         // Penetration depth
    Vec3 contact_point;  // Contact point on rigidbody surface
    usz rigidbody_id;    // Rigidbody involved (0 = ground plane)
    float friction;      // Combined friction coefficient
}

// Particle pinning: fix a particle to a world position or rigidbody
struct ParticlePin {
    usz particle;        // Particle index
    Vec3 local_offset;   // Offset in rigidbody local space (or world pos if body_id == 0)
    usz body_id;         // 0 = world-space pin, otherwise rigidbody id
}

alias SoftBodyList = List{SoftBody};

struct SoftBody {
    usz id;

    // Per-particle data
    List{Vec3} positions;
    List{Vec3} prev_positions;    // For XPBD velocity derivation
    List{Vec3} velocities;
    List{float} inverse_masses;

    // Surface mesh connectivity
    List{Triangle} triangles;

    // Constraints
    List{DistanceConstraint} distance_constraints;
    List{VolumeConstraint} volume_constraints;
    List{BendingConstraint} bending_constraints;
    List{SoftCollisionConstraint} collision_constraints;
    List{ParticlePin} pins;

    // Per-body properties
    Material material;
    float gravity_factor;
    float damping;

    // XPBD solver settings
    uint substeps;
    uint constraint_iterations;

    // AABB for broad phase
    Aabb3 aabb;

    // Sleep state
    float sleep_timer;
    bool sleeping;
    bool can_sleep;
}

fn void SoftBody.free(&self) {
    self.positions.free();
    self.prev_positions.free();
    self.velocities.free();
    self.inverse_masses.free();
    self.triangles.free();
    self.distance_constraints.free();
    self.volume_constraints.free();
    self.bending_constraints.free();
    self.collision_constraints.free();
    self.pins.free();
}

// Create a soft body from a mesh with the given particle mass
fn SoftBody create_softbody_from_mesh(Mesh* mesh, float total_mass, usz id) {
    SoftBody sb;
    sb.id = id;
    sb.gravity_factor = 1.0;
    sb.damping = 0.99;
    sb.substeps = 4;
    sb.constraint_iterations = 8;
    sb.can_sleep = true;
    sb.material = {
        .static_friction = 0.5,
        .dynamic_friction = 0.3,
        .restitution = 0.2,
    };

    usz num_particles = mesh.vertices.len();
    float particle_mass = total_mass / (float)num_particles;
    float inv_mass = particle_mass > 0 ? 1.0 / particle_mass : 0;

    // Copy vertices as particles
    foreach (v : mesh.vertices) {
        sb.positions.push(v);
        sb.prev_positions.push(v);
        sb.velocities.push({0, 0, 0});
        sb.inverse_masses.push(inv_mass);
    }

    // Copy triangles
    foreach (tri : mesh.triangles) {
        sb.triangles.push(tri);
    }

    // Extract unique edges from triangles as distance constraints
    sb.extract_distance_constraints();

    // Compute initial AABB
    sb.aabb = sb.compute_aabb();

    return sb;
}

fn void SoftBody.extract_distance_constraints(&self) {
    HashMap{Pair, bool} seen_edges;
    defer seen_edges.free();

    foreach (tri : self.triangles) {
        // Three edges per triangle
        uint[<3>][3] edges = {
            {tri.x, tri.y, 0},
            {tri.y, tri.z, 0},
            {tri.z, tri.x, 0},
        };

        for (usz e = 0; e < 3; e++) {
            usz a = edges[e][0];
            usz b = edges[e][1];
            Pair key = { math::min(a, b), math::max(a, b) };

            if (!seen_edges.has_key(key)) {
                seen_edges.set(key, true);

                Vec3 pa = self.positions[a];
                Vec3 pb = self.positions[b];
                float rest_len = (pb - pa).length();

                self.distance_constraints.push({
                    .p0 = a,
                    .p1 = b,
                    .rest_length = rest_len,
                    .compliance = 0.0, // Default: rigid
                    .lambda = 0,
                });
            }
        }
    }
}

// Build adjacency and create bending constraints from adjacent triangles
fn void SoftBody.extract_bending_constraints(&self) {
    // Map each edge to the triangles that share it
    HashMap{Pair, usz[2]} edge_triangles;
    HashMap{Pair, usz} edge_tri_count;
    defer edge_triangles.free();
    defer edge_tri_count.free();

    for (usz ti = 0; ti < self.triangles.len(); ti++) {
        Triangle tri = self.triangles[ti];
        uint[2][3] edges = {
            {tri.x, tri.y},
            {tri.y, tri.z},
            {tri.z, tri.x},
        };

        for (usz e = 0; e < 3; e++) {
            usz a = edges[e][0];
            usz b = edges[e][1];
            Pair key = { math::min(a, b), math::max(a, b) };

            usz count = 0;
            if (try c = edge_tri_count.get(key)) {
                count = c;
            }

            if (count < 2) {
                usz[2] tris;
                if (try existing = edge_triangles.get(key)) {
                    tris = existing;
                }
                tris[count] = ti;
                edge_triangles.set(key, tris);
                edge_tri_count.set(key, count + 1);
            }
        }
    }

    // For each edge shared by exactly 2 triangles, create a bending constraint
    edge_tri_count.@each(; Pair key, usz count) {
        if (count != 2) return;

        usz[2] tris = edge_triangles.get(key)!!;
        Triangle t0 = self.triangles[tris[0]];
        Triangle t1 = self.triangles[tris[1]];

        // Find the opposite vertices (not on the shared edge)
        usz opposite0 = find_opposite_vertex(t0, key.a, key.b);
        usz opposite1 = find_opposite_vertex(t1, key.a, key.b);

        // Compute rest dihedral angle
        float rest_angle = self.compute_dihedral_angle(key.a, key.b, opposite0, opposite1);

        self.bending_constraints.push({
            .p0 = key.a,
            .p1 = key.b,
            .p2 = opposite0,
            .p3 = opposite1,
            .rest_angle = rest_angle,
            .compliance = 0.001, // Bending is usually softer
            .lambda = 0,
        });
    };
}

fn usz find_opposite_vertex(Triangle tri, usz edge_a, usz edge_b) @local {
    if (tri.x != edge_a && tri.x != edge_b) return tri.x;
    if (tri.y != edge_a && tri.y != edge_b) return tri.y;
    return tri.z;
}

fn float SoftBody.compute_dihedral_angle(&self, usz e0, usz e1, usz opp0, usz opp1) {
    Vec3 p0 = self.positions[e0];
    Vec3 p1 = self.positions[e1];
    Vec3 p2 = self.positions[opp0];
    Vec3 p3 = self.positions[opp1];

    Vec3 edge = (p1 - p0).normalize();
    Vec3 n0 = (p2 - p0).cross(p1 - p0).normalize();
    Vec3 n1 = (p1 - p0).cross(p3 - p0).normalize();

    float cos_angle = math::clamp(n0.dot(n1), -1.0f, 1.0f);
    float sin_angle = n0.cross(n1).dot(edge);

    return math::atan2(sin_angle, cos_angle);
}

// Compute AABB from current particle positions
fn Aabb3 SoftBody.compute_aabb(&self) {
    if (self.positions.len() == 0) return EMPTY_AABB3;

    Vec3 min_v = self.positions[0];
    Vec3 max_v = self.positions[0];

    for (usz i = 1; i < self.positions.len(); i++) {
        Vec3 p = self.positions[i];
        min_v.x = math::min(min_v.x, p.x);
        min_v.y = math::min(min_v.y, p.y);
        min_v.z = math::min(min_v.z, p.z);
        max_v.x = math::max(max_v.x, p.x);
        max_v.y = math::max(max_v.y, p.y);
        max_v.z = math::max(max_v.z, p.z);
    }

    return { min_v, max_v };
}

// Compute signed volume of the mesh (for volume preservation)
fn float SoftBody.compute_signed_volume(&self) {
    float volume = 0;
    foreach (tri : self.triangles) {
        Vec3 v0 = self.positions[tri.x];
        Vec3 v1 = self.positions[tri.y];
        Vec3 v2 = self.positions[tri.z];
        volume += v0.dot(v1.cross(v2)) / 6.0;
    }
    return volume;
}

// ============================================================
// XPBD Solver
// ============================================================

fn void SoftBody.xpbd_solve(&self, Vec3 gravity, float dt, PhysicsWorld* world = null) {
    if (self.sleeping) return;

    usz num_particles = self.positions.len();
    if (num_particles == 0) return;

    float sub_dt = dt / (float)self.substeps;

    for (uint sub = 0; sub < self.substeps; sub++) {
        // === Step 1: Predict positions ===
        for (usz i = 0; i < num_particles; i++) {
            if (self.inverse_masses[i] == 0) continue; // Fixed particle

            // Apply gravity to velocity
            self.velocities[i] += gravity * self.gravity_factor * sub_dt;

            // Save current position
            self.prev_positions[i] = self.positions[i];

            // Predict new position
            self.positions[i] += self.velocities[i] * sub_dt;
        }

        // === Step 2: Project constraints ===
        for (uint iter = 0; iter < self.constraint_iterations; iter++) {
            // Distance constraints
            foreach (&dc : self.distance_constraints) {
                self.solve_distance_constraint(dc, sub_dt);
            }

            // Volume constraints
            foreach (&vc : self.volume_constraints) {
                self.solve_volume_constraint(vc, sub_dt);
            }

            // Bending constraints
            foreach (&bc : self.bending_constraints) {
                self.solve_bending_constraint(bc, sub_dt);
            }

            // Collision constraints (generated during broad/narrow phase)
            foreach (&cc : self.collision_constraints) {
                self.solve_collision_constraint(cc, sub_dt, world);
            }
        }

        // === Step 3: Apply pin constraints (after all iterations) ===
        foreach (&pin : self.pins) {
            self.apply_pin(pin, world);
        }

        // === Step 4: Derive velocities from position changes ===
        for (usz i = 0; i < num_particles; i++) {
            if (self.inverse_masses[i] == 0) continue;

            self.velocities[i] = (self.positions[i] - self.prev_positions[i]) / sub_dt;

            // Apply damping
            self.velocities[i] *= self.damping;
        }
    }

    // Clear per-frame collision constraints
    self.collision_constraints.clear();

    // Reset Lagrange multipliers for next frame
    foreach (&dc : self.distance_constraints) dc.lambda = 0;
    foreach (&vc : self.volume_constraints) vc.lambda = 0;
    foreach (&bc : self.bending_constraints) bc.lambda = 0;

    // Update AABB
    self.aabb = self.compute_aabb();
}

// XPBD distance constraint solver
fn void SoftBody.solve_distance_constraint(&self, DistanceConstraint* c, float dt) @local {
    Vec3 p0 = self.positions[c.p0];
    Vec3 p1 = self.positions[c.p1];
    float w0 = self.inverse_masses[c.p0];
    float w1 = self.inverse_masses[c.p1];

    float w_sum = w0 + w1;
    if (w_sum == 0) return;

    Vec3 diff = p1 - p0;
    float dist = diff.length();
    if (dist < 1e-7) return;

    Vec3 grad = diff / dist;
    float constraint_value = dist - c.rest_length;

    // XPBD: compliance-scaled correction
    float alpha = c.compliance / (dt * dt);
    float delta_lambda = -(constraint_value + alpha * c.lambda) / (w_sum + alpha);
    c.lambda += delta_lambda;

    // Apply position corrections
    if (w0 > 0) self.positions[c.p0] -= grad * (w0 * delta_lambda);
    if (w1 > 0) self.positions[c.p1] += grad * (w1 * delta_lambda);
}

// XPBD volume constraint solver (pressure-based)
fn void SoftBody.solve_volume_constraint(&self, VolumeConstraint* c, float dt) @local {
    float current_volume = self.compute_signed_volume();
    float constraint_value = current_volume - c.rest_volume;

    if (math::abs(constraint_value) < 1e-7) return;

    // Compute gradients: dV/dp_i for each particle
    // For signed volume, gradient w.r.t vertex i involves adjacent triangles
    // Simplified approach: uniform pressure on all surface vertices
    usz n = self.positions.len();
    float total_grad_sq = 0;
    float total_w = 0;

    // For each triangle, compute area-weighted normal contribution
    List{Vec3} gradients;
    defer gradients.free();

    for (usz i = 0; i < n; i++) {
        gradients.push({0, 0, 0});
    }

    foreach (tri : self.triangles) {
        Vec3 v0 = self.positions[tri.x];
        Vec3 v1 = self.positions[tri.y];
        Vec3 v2 = self.positions[tri.z];

        // Gradient of signed volume w.r.t each vertex of this triangle
        Vec3 g0 = v1.cross(v2) / 6.0;
        Vec3 g1 = v2.cross(v0) / 6.0;
        Vec3 g2 = v0.cross(v1) / 6.0;

        gradients[tri.x] += g0;
        gradients[tri.y] += g1;
        gradients[tri.z] += g2;
    }

    for (usz i = 0; i < n; i++) {
        float w = self.inverse_masses[i];
        total_grad_sq += w * gradients[i].dot(gradients[i]);
    }

    if (total_grad_sq < 1e-10) return;

    float alpha = c.compliance / (dt * dt);
    float delta_lambda = -(constraint_value + alpha * c.lambda) / (total_grad_sq + alpha);
    c.lambda += delta_lambda;

    for (usz i = 0; i < n; i++) {
        float w = self.inverse_masses[i];
        if (w > 0) {
            self.positions[i] += gradients[i] * (w * delta_lambda);
        }
    }
}

// XPBD bending constraint solver (dihedral angle)
fn void SoftBody.solve_bending_constraint(&self, BendingConstraint* c, float dt) @local {
    Vec3 p0 = self.positions[c.p0];
    Vec3 p1 = self.positions[c.p1];
    Vec3 p2 = self.positions[c.p2];
    Vec3 p3 = self.positions[c.p3];

    float w0 = self.inverse_masses[c.p0];
    float w1 = self.inverse_masses[c.p1];
    float w2 = self.inverse_masses[c.p2];
    float w3 = self.inverse_masses[c.p3];

    // Compute current dihedral angle
    Vec3 edge = p1 - p0;
    float edge_len = edge.length();
    if (edge_len < 1e-7) return;

    Vec3 edge_dir = edge / edge_len;

    Vec3 n0 = (p2 - p0).cross(p1 - p0);
    Vec3 n1 = (p1 - p0).cross(p3 - p0);

    float n0_len = n0.length();
    float n1_len = n1.length();
    if (n0_len < 1e-7 || n1_len < 1e-7) return;

    n0 = n0 / n0_len;
    n1 = n1 / n1_len;

    float cos_angle = math::clamp(n0.dot(n1), -1.0f, 1.0f);
    float sin_angle = n0.cross(n1).dot(edge_dir);
    float current_angle = math::atan2(sin_angle, cos_angle);

    float constraint_value = current_angle - c.rest_angle;

    // Wrap angle difference to [-pi, pi]
    while (constraint_value > math::PI) constraint_value -= 2 * math::PI;
    while (constraint_value < -math::PI) constraint_value += 2 * math::PI;

    if (math::abs(constraint_value) < 1e-6) return;

    // Simplified gradient: move opposite vertices along face normals
    Vec3 g2 = n0 / n0_len;
    Vec3 g3 = n1 / n1_len;
    // Edge vertices get combined gradients
    Vec3 g0 = -(g2 + g3) * 0.5;
    Vec3 g1 = -(g2 + g3) * 0.5;

    float grad_sq = w0 * g0.dot(g0) + w1 * g1.dot(g1) + w2 * g2.dot(g2) + w3 * g3.dot(g3);
    if (grad_sq < 1e-10) return;

    float alpha = c.compliance / (dt * dt);
    float delta_lambda = -(constraint_value + alpha * c.lambda) / (grad_sq + alpha);
    c.lambda += delta_lambda;

    if (w0 > 0) self.positions[c.p0] += g0 * (w0 * delta_lambda);
    if (w1 > 0) self.positions[c.p1] += g1 * (w1 * delta_lambda);
    if (w2 > 0) self.positions[c.p2] += g2 * (w2 * delta_lambda);
    if (w3 > 0) self.positions[c.p3] += g3 * (w3 * delta_lambda);
}

// XPBD collision constraint solver — unified position-based coupling
fn void SoftBody.solve_collision_constraint(&self, SoftCollisionConstraint* c, float dt, PhysicsWorld* world = null) @local {
    float w_particle = self.inverse_masses[c.particle];
    if (w_particle == 0) return;

    Vec3 p = self.positions[c.particle];

    // Compute penetration along normal
    float d = (p - c.contact_point).dot(c.normal);
    if (d >= 0) return; // Not penetrating

    float correction = -d;

    // If coupled to a rigid body, share the correction
    if (c.rigidbody_id != 0 && world != null) {
        Rigidbody*? body_opt = world.find_body(c.rigidbody_id);
        if (try body = body_opt) {
            if (!body.is_static) {
                Vec3 r = c.contact_point - body.collider.translation;
                float w_body = collision::compute_generalized_inverse_mass(body, r, c.normal);
                float w_total = w_particle + w_body;

                if (w_total > 1e-10) {
                    float particle_share = w_particle / w_total;
                    float body_share = w_body / w_total;

                    // Move particle
                    self.positions[c.particle] += c.normal * (correction * particle_share);

                    // Move rigid body (opposite direction)
                    collision::apply_positional_correction(body, -c.normal, r, correction * body_share);
                }

                // Friction
                if (c.friction > 0) {
                    Vec3 vel = self.velocities[c.particle];
                    float vn = vel.dot(c.normal);
                    Vec3 vt = vel - c.normal * vn;
                    float vt_mag = vt.length();
                    if (vt_mag > 1e-6) {
                        float friction_impulse = math::min(c.friction * math::abs(correction) / dt, vt_mag);
                        self.velocities[c.particle] -= (vt / vt_mag) * friction_impulse;
                    }
                }
                return;
            }
        }
    }

    // No rigid body coupling — just correct particle
    self.positions[c.particle] += c.normal * correction;

    // Friction
    if (c.friction > 0) {
        Vec3 vel = self.velocities[c.particle];
        float vn = vel.dot(c.normal);
        Vec3 vt = vel - c.normal * vn;
        float vt_mag = vt.length();

        if (vt_mag > 1e-6) {
            float friction_impulse = math::min(c.friction * math::abs(correction) / dt, vt_mag);
            self.velocities[c.particle] -= (vt / vt_mag) * friction_impulse;
        }
    }
}

// Apply particle pin constraint
fn void SoftBody.apply_pin(&self, ParticlePin* pin, PhysicsWorld* world = null) @local {
    if (pin.body_id != 0 && world != null) {
        // Pin to rigidbody: compute world position from local offset
        if (try body = world.find_body(pin.body_id)) {
            Vec3 world_pos = body.collider.rotation * pin.local_offset + body.collider.translation;
            self.positions[pin.particle] = world_pos;
            self.velocities[pin.particle] = body.relative_velocity(
                world_pos - body.collider.translation);
            return;
        }
    }

    // World-space pin
    self.positions[pin.particle] = pin.local_offset;
    self.velocities[pin.particle] = {0, 0, 0};
}

// Pin a particle to a world position
fn void SoftBody.pin_particle(&self, usz particle, Vec3 world_pos) {
    self.pins.push({
        .particle = particle,
        .local_offset = world_pos,
        .body_id = 0,
    });
    self.inverse_masses[particle] = 0; // Make it immovable during solving
}

// Pin a particle to a rigidbody (follows body as it moves/rotates)
fn void SoftBody.pin_to_rigidbody(&self, usz particle, usz body_id, Vec3 local_offset) {
    self.pins.push({
        .particle = particle,
        .local_offset = local_offset,
        .body_id = body_id,
    });
    self.inverse_masses[particle] = 0;
}

// Unpin a particle and restore its mass
fn void SoftBody.unpin_particle(&self, usz particle, float mass) {
    // Remove pin
    self.pins.remove_using_test(fn (pin, ctx) => pin.particle == *(usz*)ctx, &particle);
    // Restore inverse mass
    if (mass > 0) {
        self.inverse_masses[particle] = 1.0 / mass;
    }
}

// Set distance constraint compliance (0 = rigid, higher = softer)
fn void SoftBody.set_distance_compliance(&self, float compliance) {
    foreach (&dc : self.distance_constraints) {
        dc.compliance = compliance;
    }
}

// Set volume constraint compliance
fn void SoftBody.set_volume_compliance(&self, float compliance) {
    foreach (&vc : self.volume_constraints) {
        vc.compliance = compliance;
    }
}

// Set bending constraint compliance
fn void SoftBody.set_bending_compliance(&self, float compliance) {
    foreach (&bc : self.bending_constraints) {
        bc.compliance = compliance;
    }
}

// Enable volume preservation
fn void SoftBody.enable_volume_preservation(&self, float compliance = 0.0) {
    float rest_vol = self.compute_signed_volume();
    self.volume_constraints.push({
        .rest_volume = rest_vol,
        .compliance = compliance,
        .lambda = 0,
    });
}

// Enable bending resistance
fn void SoftBody.enable_bending(&self, float compliance = 0.001) {
    self.extract_bending_constraints();
    self.set_bending_compliance(compliance);
}

// Wake the soft body from sleep
fn void SoftBody.wake(&self) {
    self.sleeping = false;
    self.sleep_timer = 0;
}

// Check if body has enough velocity to be considered moving
fn bool SoftBody.is_moving(&self) {
    const float SLEEP_THRESHOLD = 0.01;
    foreach (v : self.velocities) {
        if (v.sq_magnitude() > SLEEP_THRESHOLD) return true;
    }
    return false;
}

// Generate ground plane collision constraints (z=0 default ground)
fn void SoftBody.generate_ground_collisions(&self, float ground_z = 0, float friction = 0.5) {
    for (usz i = 0; i < self.positions.len(); i++) {
        if (self.inverse_masses[i] == 0) continue;

        Vec3 p = self.positions[i];
        if (p.z < ground_z) {
            self.collision_constraints.push({
                .particle = i,
                .normal = {0, 0, 1},
                .depth = ground_z - p.z,
                .contact_point = {p.x, p.y, ground_z},
                .rigidbody_id = 0,
                .friction = friction,
            });
        }
    }
}

// Detect and generate collision constraints against a rigidbody
fn void SoftBody.generate_rigidbody_collisions(&self, Rigidbody* body) {
    // Quick AABB check
    if (!self.aabb.collides(body.transformed_aabb)) return;

    // Combined friction
    float friction = (self.material.static_friction + body.material.static_friction) * 0.5f;

    // Create a temporary triangle mesh for per-triangle testing
    Mesh tri_mesh;
    tri_mesh.vertices.init(allocator: mem);
    tri_mesh.triangles.init(allocator: mem);
    tri_mesh.vertices.push({0, 0, 0});
    tri_mesh.vertices.push({0, 0, 0});
    tri_mesh.vertices.push({0, 0, 0});
    tri_mesh.triangles.push({0, 1, 2});
    defer tri_mesh.free();

    TransformedShape tri_shape = {
        .translation = {0, 0, 0},
        .scale = {1, 1, 1},
        .rotation = {0, 0, 0, 1},
        .shape = &tri_mesh,
    };

    // Test each surface triangle against the rigidbody
    foreach (tri : self.triangles) {
        Vec3 v0 = self.positions[tri.x];
        Vec3 v1 = self.positions[tri.y];
        Vec3 v2 = self.positions[tri.z];

        // Quick per-triangle AABB rejection
        Aabb3 tri_aabb = {
            .min = {
                math::min(v0.x, math::min(v1.x, v2.x)),
                math::min(v0.y, math::min(v1.y, v2.y)),
                math::min(v0.z, math::min(v1.z, v2.z))
            },
            .max = {
                math::max(v0.x, math::max(v1.x, v2.x)),
                math::max(v0.y, math::max(v1.y, v2.y)),
                math::max(v0.z, math::max(v1.z, v2.z))
            }
        };

        if (!tri_aabb.collides(body.transformed_aabb)) continue;

        // Set triangle vertices (world space, identity transform)
        tri_mesh.vertices[0] = v0;
        tri_mesh.vertices[1] = v1;
        tri_mesh.vertices[2] = v2;

        CollisionInfo info = collision::check_convex_collision(&tri_shape, &body.collider, epa: true);

        if (info.collided && info.depth > 0) {
            // Create collision constraints for each vertex of the penetrating triangle
            usz[3] indices = { tri.x, tri.y, tri.z };
            for (usz v = 0; v < 3; v++) {
                usz idx = indices[v];
                Vec3 p = self.positions[idx];

                // Check if this vertex is on the penetrating side
                float proj = (p - info.contact_points[1]).dot(info.normal);
                if (proj < 0) {
                    self.collision_constraints.push({
                        .particle = idx,
                        .normal = -info.normal,
                        .depth = info.depth,
                        .contact_point = p - info.normal * proj,
                        .rigidbody_id = body.id,
                        .friction = friction,
                    });
                }
            }
        }
    }
}


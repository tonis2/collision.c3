module collision;
import std::math;

struct Rigidbody {
    float mass;
    float inverse_mass;
    float inertia_tensor;
    float inverse_inertia_tensor;
    float radius;

    // State
    Vec3f position;
    Quaternionf orientation;
    Vec3f linear_velocity;
    Vec3f angular_velocity;

    // Forces
    Vec3f force;
    Vec3f torque;
}

fn void Rigidbody.apply_force(&self, Vec3f force) => self.force += force;
fn void Rigidbody.apply_force_at_point(&self, Vec3f force, Vec3f point) {
    self.force += force;
    Vec3f r = point - self.position;
    self.torque += r.cross(force);
}

fn void Rigidbody.run_step(&self, float time) {
    if (self.inverse_mass == 0) return;

    Vec3f linear_acceleration = self.force * self.inverse_mass;
    self.linear_velocity += linear_acceleration * time;
    self.position += self.linear_velocity * time;

    // Angular motion
    Vec3f angular_acceleration = self.torque * self.inverse_inertia_tensor;
    self.angular_velocity += angular_acceleration * time;
    
    Quaternionf quaternion = {0, self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z};
    self.orientation = (self.orientation + (quaternion * self.orientation) * 0.5 * time).normalize();

    // Clear forces
    self.force = {0, 0, 0};
    self.torque = {0, 0, 0};
}

fn void Rigidbody.resolve_collision(&self, Rigidbody* second, Vec3f normal, uint depth) {
    // --- Positional Correction (to prevent sinking) ---
    float slop = 0.01; // Penetration allowance
    float percent = 0.2; // Penetration correction percentage
    Vec3f correction = normal * math::max(depth - slop, (float)0.0) / (self.inverse_mass + second.inverse_mass) * percent;
    if (self.inverse_mass != 0) self.position += correction * self.inverse_mass;
    if (second.inverse_mass != 0) second.position += correction * second.inverse_mass;
}
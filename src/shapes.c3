module collision;

import std::math;

interface ConvexShape
{
    fn Vec3f furthest_point(Vec3f direction);
}

// Shapes
struct TransformedConvex (ConvexShape) {
    Vec3f position;
    Vec3f scale;
    Quaternionf rotation;
    ConvexShape shape;
}

struct ConvexPolyhedron (ConvexShape) {
    Vec3f[] vertices;
}

fn Vec3f Aabb3.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f point = {
        (direction.x > 0) ? self.max.x : self.min.x,
        (direction.y > 0) ? self.max.y : self.min.y,
        (direction.z > 0) ? self.max.z : self.min.z
    };
    return point;
}

fn Vec3f TransformedConvex.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f local_direction = direction.rotate_by_quat(self.rotation.conjugate());
    Vec3f point = self.shape.furthest_point(local_direction);
    return point.rotate_by_quat(self.rotation) * self.scale + self.position;
}

fn Vec3f ConvexPolyhedron.furthest_point(&self, Vec3f direction) @dynamic {
    Vec3f furthest_point;
    float max_distance = -float.max;

    foreach (pos : self.vertices) {
        float distance = pos.dot(direction);
        if (distance > max_distance) {
            max_distance = distance;
            furthest_point = pos;
        }
    }

    return furthest_point;
}
module collision;

import std::math;
import std::collections::elastic_array;

const usz GJK_MAX_ITERATION = 30;

alias Simplex = ElasticArray{SupportPoint, 4};

struct SupportPoint {
    Vec3f point;  // A - B
    Vec3f a;
    Vec3f b;
}

<*
 @require array.len > 0 : `Array should not be empty`
*>
fn void Simplex.replace_array(&self, SupportPoint[] array, usz start_index = 0)
{
    self.size = array.len;
	foreach (index, &value : array) {self.entries[start_index + index] = *value;}
}

fn void Simplex.push_start(&self, SupportPoint point)
{
    self.entries = {point, self.entries[0], self.entries[1], self.entries[2]};
    self.size = math::min((usz)(self.size + 1), (usz)4);
}

fn bool same_direction(Vec3f direction, Vec3f ao) => direction.dot(ao) > 0;

// Handles the line case (2 points in simplex)
fn bool line_case(Simplex* simplex, Vec3f* direction) @private {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);

    Vec3f a = a_support.point;
    Vec3f b = b_support.point;
    Vec3f ab = b - a;
    Vec3f ao = -a;

    if (same_direction(ab, ao)) {
        direction.set(ab.cross(ao).cross(ab));
    } else {
        simplex.replace_array({ a_support });
        direction.set(ao);
    }

    return false;
}


// Handles the triangle case (3 points in simplex)
fn bool triangle_case(Simplex* simplex, Vec3f* direction) @private {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);
    SupportPoint c_support = simplex.get(2);

    Vec3f a = a_support.point;
    Vec3f b = b_support.point;
    Vec3f c = c_support.point;

    Vec3f ab = b - a;
    Vec3f ac = c - a;
    Vec3f ao = -a;

    Vec3f abc = ab.cross(ac);

    if (same_direction(abc.cross(ac), ao)) {
        if (same_direction(ac, ao)) {
            simplex.replace_array({ a_support, c_support });
            direction.set(ac.cross(ao).cross(ac));
        } else {
            return line_case(simplex, direction);
        }
    } else {
        if (same_direction(ab.cross(abc), ao)) {
             return line_case(simplex, direction);
        } else {
            if (same_direction(abc, ao)) {
                direction.set(abc);
            } else {
                simplex.replace_array({ a_support, c_support, b_support });
                direction.set(-abc);
            }
        }
    }

    return false;
}

// Handles the tetrahedron case (4 points in simplex)
fn bool tetrahedron_case(Simplex* simplex, Vec3f* direction) {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);
    SupportPoint c_support = simplex.get(2);
    SupportPoint d_support = simplex.get(3);

    Vec3f a = a_support.point;
    Vec3f b = b_support.point;
    Vec3f c = c_support.point;
    Vec3f d = d_support.point;

    Vec3f ab = b - a;
    Vec3f ac = c - a;
    Vec3f ad = d - a;
    Vec3f ao = -a;

    Vec3f abc = ab.cross(ac);
    Vec3f acd = ac.cross(ad);
    Vec3f adb = ad.cross(ab);

    if (same_direction(abc, ao)) {
        simplex.replace_array({ a_support, b_support, c_support });
        return triangle_case(simplex, direction);
    }
    if (same_direction(acd, ao)) {
        simplex.replace_array({ a_support, c_support, d_support });
        return triangle_case(simplex, direction);
    }
    if (same_direction(adb, ao)) {
        simplex.replace_array({ a_support, d_support, b_support });
        return triangle_case(simplex, direction);
    }

    return true; // Origin is inside tetrahedron
}

fn bool next_simplex(Simplex* simplex, Vec3f* direction) {
    switch (simplex.len()) {
        case 2: return line_case(simplex, direction);
        case 3: return triangle_case(simplex, direction);
        case 4: return tetrahedron_case(simplex, direction);
    }

    return false; // Should not be reached
}

// GJK algorithm to find, if shapes collide
fn CollisionInfo check_collision(ConvexShape shape_1, ConvexShape shape_2) {
    // Initial direction (arbitrary)
    Vec3f direction = {1, 0, 0};
    
    Simplex simplex;
    {
        Vec3f a_support = shape_1.furthest_point(direction);
        Vec3f b_support = shape_2.furthest_point(-direction);
        Vec3f point = a_support - b_support;

        // Initial simplex point
        simplex.push_start({
            point,
            a_support,
            b_support
        });

        // New search direction is towards origin
        direction = -point;
    }

    // Main GJK loop
    for (usz i = 0; i < GJK_MAX_ITERATION; ++i) {
        // Get new support point
        Vec3f a_support = shape_1.furthest_point(direction);
        Vec3f b_support = shape_2.furthest_point(-direction);
        Vec3f mikowski_difference = a_support - b_support;
        
        // If we didn't reach the origin, no collision
        if (mikowski_difference.dot(direction) <= 0) {
            return { .collided = false };
        }

        // Add new point to simplex
        simplex.push_start({
            mikowski_difference,
            a_support,
            b_support
        });

        if (next_simplex(&simplex, &direction)) {
            return { .collided = true, .simplex = simplex };
        }
    }

    return { .collided = false };
}
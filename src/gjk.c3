module collision;
import std::io;
import std::math;
import std::collections::elastic_array;

const usz GJK_MAX_ITERATION = 30;
const Number GJK_TOLERANCE = 0.0001f;

alias Simplex = ElasticArray{SupportPoint, 4};

struct SupportPoint {
    Vec3 point;  // A - B
    Vec3 a;
    Vec3 b;
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

fn bool same_direction(Vec3 direction, Vec3 ao) => direction.dot(ao) > 0;

// Handles the line case (2 points in simplex)
fn bool line_case(Simplex* simplex, Vec3* direction) @private {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);

    Vec3 a = a_support.point;
    Vec3 b = b_support.point;
    Vec3 ab = b - a;
    Vec3 ao = -a;

    if (same_direction(ab, ao)) {
        direction.set(ab.cross(ao).cross(ab));
    } else {
        simplex.replace_array({ a_support });
        direction.set(ao);
    }

    return false;
}


// Handles the triangle case (3 points in simplex)
fn bool triangle_case(Simplex* simplex, Vec3* direction) @private {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);
    SupportPoint c_support = simplex.get(2);

    Vec3 a = a_support.point;
    Vec3 b = b_support.point;
    Vec3 c = c_support.point;

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);

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
fn bool tetrahedron_case(Simplex* simplex, Vec3* direction) @private {
    SupportPoint a_support = simplex.get(0);
    SupportPoint b_support = simplex.get(1);
    SupportPoint c_support = simplex.get(2);
    SupportPoint d_support = simplex.get(3);

    Vec3 a = a_support.point;
    Vec3 b = b_support.point;
    Vec3 c = c_support.point;
    Vec3 d = d_support.point;

    Vec3 ab = b - a;
    Vec3 ac = c - a;
    Vec3 ad = d - a;
    Vec3 ao = -a;

    Vec3 abc = ab.cross(ac);
    Vec3 acd = ac.cross(ad);
    Vec3 adb = ad.cross(ab);

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

fn bool next_simplex(Simplex* simplex, Vec3* direction) @private {
    switch (simplex.len()) {
        case 2: return line_case(simplex, direction);
        case 3: return triangle_case(simplex, direction);
        case 4: return tetrahedron_case(simplex, direction);
    }

    return false; // Should not be reached
}

// GJK algorithm to find, if shapes collide
fn CollisionInfo check_convex_collision(CollisionShape shape_1, CollisionShape shape_2, bool epa = false) {
    // Initial direction (arbitrary)
    Vec3 direction = {1, 0, 0};
    
    Simplex simplex;
    {
        Vec3 a_support = shape_1.furthest_point(direction);
        Vec3 b_support = shape_2.furthest_point(-direction);
        Vec3 point = a_support - b_support;

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
        Vec3 a_support = shape_1.furthest_point(direction);
        Vec3 b_support = shape_2.furthest_point(-direction);
        Vec3 mikowski_difference = a_support - b_support;

        // If we didn't reach the origin, no collision
        if (mikowski_difference.dot(direction) < GJK_TOLERANCE) {
            return { .collided = false };
        }

        // Add new point to simplex
        simplex.push_start({
            mikowski_difference,
            a_support,
            b_support
        });

        if (next_simplex(&simplex, &direction)) {
            CollisionInfo info = { .collided = true, .simplex = simplex };
            if (epa == true) collision::epa(&info, shape_1, shape_2);
            return info;
        }
    }

    return { .collided = false };
}
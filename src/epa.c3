module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::io;

struct Face {
    uint a,b,c;
    Vec3f normal;
    float distance;
}

struct CollisionInfo {
    Simplex simplex;
    Vec3f normal;
    Vec3f[2] contact_points;
    float depth;
    bool collided;
}


const uint EPA_MAX_ITER = 16;
const float EPA_TOLERANCE = 0.0001;

const uint MAX_FACES = 40;

alias EdgeArray @private = ElasticArray{Pair, MAX_FACES};
alias FaceArray @private = ElasticArray{Face, MAX_FACES};
alias SupportArray @private = ElasticArray{SupportPoint, MAX_FACES};
alias IndexArray @private = ElasticArray{uint, MAX_FACES};

<*
 @require array.len > 0 : `Array should not be empty`
*>
fn void FaceArray.replace_array(&self, Face[] array, usz start_index = 0)
{
    self.size = array.len;
	foreach (index, &value : array) {self.entries[start_index + index] = *value;}
}

fn void EdgeArray.add(&self, usz a, usz b)
{
    Pair pair = {b, a};

    for (uint i = 0; i < self.len(); i++) {
        Pair edge = self.get(i);
        if (edge == pair) {
            self.set(i, self.last()!!);
            self.remove_last();
            return;
        }
    }

    self.push({a, b});
}

fn void Face.calculate_normal(&self, Vec3f a, Vec3f b, Vec3f c)
{
    Vec3f ab = b - a;
    Vec3f ac = c - a;
    self.normal = (b - a).cross(c - a).normalize();
    self.distance = self.normal.dot(a);

    if (self.distance < 0) {
        self.normal   *= -1;
        self.distance *= -1;
    }
}


fn void SupportArray.expand(&self, FaceArray* faces, Vec3f support)
{
    EdgeArray edges;
    IndexArray remove_faces;

    for (uint i = 0; i < faces.len(); i++) {
        Face face = faces.get(i);

        if (face.normal.dot(support) > 0) {
            edges.add(face.a, face.b);
            edges.add(face.b, face.c);
            edges.add(face.c, face.a);
            
            faces.set(i, faces.last()!!);
            faces.remove_last();
            i--;
        }
    }

    foreach (edge : edges) {
        Face face = {.a = (uint)edge.first, .b = (uint)edge.second, .c = (uint)self.len() - 1};

        Vec3f a = self.get(face.a).point;
        Vec3f b = self.get(face.b).point;
        Vec3f c = self.get(face.c).point;

        face.calculate_normal(a, b, c);
        faces.push(face);
    };
}

fn uint Face[].get_closest(self, float min_distance = float.max) @private {
    uint closest = 0;

    foreach (uint i, face: self) {
        if (face.distance < min_distance) {
            min_distance = face.distance;
            closest = i;
        }
    }

    return closest;
}

<*
 @require info.simplex.len() > 3 : `Collision simplex should have 4 values`
*>
fn void epa(CollisionInfo* info, ConvexShape shape_1, ConvexShape shape_2) {
    SupportArray vertices;

    foreach (item: info.simplex) {
        vertices.push(item);
    };

    FaceArray faces;
    faces.add_array({
        (Face){.a = 0, .b = 1, .c = 2},
        (Face){.a = 0, .b = 3, .c = 1},
        (Face){.a = 0, .b = 2, .c = 3},
        (Face){.a = 1, .b = 3, .c = 2}
    });

    foreach (&face : faces) { 
        Vec3f a = vertices.get(face.a).point;
        Vec3f b = vertices.get(face.b).point;
        Vec3f c = vertices.get(face.c).point;
        face.calculate_normal(a, b, c);
    }    
    
    uint closest_face_index = faces.array_view().get_closest();

    Face closest_face;

    for (int i = 0; i < EPA_MAX_ITER; ++i) {
        closest_face = faces[closest_face_index];

        Vec3f face_normal = closest_face.normal;

        Vec3f a_suppport = shape_1.furthest_point(face_normal);
        Vec3f b_support = shape_2.furthest_point(-face_normal);

        SupportPoint support_point = {
            .a = a_suppport,
            .b = b_support,
            .point = a_suppport - b_support
        };

        float support_distance = face_normal.dot(support_point.point);
        if (support_distance - closest_face.distance < EPA_TOLERANCE) {
            // We have a hit
            info.normal = closest_face.normal;
            info.depth = closest_face.distance;
            assert(info.normal != (Vec3f){0, 0, 0});

            Vec3f p = info.normal * -info.depth;

            SupportPoint a = vertices[closest_face.a];
            SupportPoint b = vertices[closest_face.b];
            SupportPoint c = vertices[closest_face.c];

            Vec3f bary = p.barycentric(a.point, b.point, c.point);

            Vec3f[2] support_a = {a.a, a.b};
            Vec3f[2] support_b = {b.a, b.b};
            Vec3f[2] support_c = {c.a, c.b};

            info.contact_points = {
                support_a[0] * bary.x + support_b[0] * bary.y + support_c[0] * bary.z,
                support_a[1] * bary.x + support_b[1] * bary.y + support_c[1] * bary.z,
            };
            return;
        }

        usz offset = faces.len();

        vertices.push(support_point);
        vertices.expand(&faces, support_point.point);

        closest_face_index = faces.entries[offset..].get_closest();
    }

    info.normal = closest_face.normal;
    info.depth = closest_face.distance;
    // info.collided = false;
}
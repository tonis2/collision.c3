module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::io;
import std::os;

struct Face @private {
    usz a,b,c;
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

alias EdgeArray @private = ElasticArray{Pair, 20};
alias FaceArray @private = ElasticArray{Face, 40};
alias SupportArray @private = ElasticArray{SupportPoint, 40};

fn void EdgeArray.add_if_new(&self, Pair pair)
{   
    // Reverse search
    for (int i = (int)self.len() - 1; i >= 0; i--) {
        Pair edge = self.get(i);
        if (edge == (Pair){pair.second, pair.first}) {
            self.remove_at(i);
            return;
        }
    }

    self.push(pair);
}

fn Face SupportArray.create_face(self, usz a_index, usz b_index, usz c_index)
{
    Vec3f a = self.get(a_index).point;
    Vec3f b = self.get(b_index).point;
    Vec3f c = self.get(c_index).point;

    Vec3f normal = (b - a).cross(c - a).normalize();
    float distance = a.dot(normal);

    // assert(normal != (Vec3f){0,0,0});

    if ((-a).dot(normal) > 0) {
        normal   *= -1;
        distance *= -1;
    }

    return {    
        .a = a_index,
        .b = b_index,
        .c = c_index,
        .distance = distance,
        .normal = normal
    };
}

fn void SupportArray.reconstruct(self, FaceArray* faces, Vec3f new_point)
{
    EdgeArray edges;

    // Reverse loop
    for (int i = (int)faces.len() - 1; i >= 0; i--) {
        Face face = faces.get(i);

        if (face.normal.dot(new_point - self.get(face.a).point) > 0) {
            edges.add_if_new({face.a, face.b});
            edges.add_if_new({face.b, face.c});
            edges.add_if_new({face.c, face.a});

            faces.remove_at(i);
        }
    }

    foreach (edge: edges) {
        Face new_face = self.create_face(edge.first, edge.second, self.len() - 1);
        if (new_face.normal == (Vec3f){0,0,0}) continue;
        faces.push(new_face);
    }
}

fn uint Face[].get_closest(self, float min_distance = float.max) {
    uint closest = 0;

    foreach (uint i, face: self) {
        if (face.distance < min_distance) {
            min_distance = face.distance;
            closest = i;
        }
    }

    return closest;
}

const uint EPA_MAX_ITER = 16;
const float EPA_TOLERANCE = 0.0001f;

<*
 @require info.simplex.len() > 3 : `Collision simplex should have 4 values`
*>
fn void epa(CollisionInfo* info, ConvexShape shape_1, ConvexShape shape_2) {
    SupportArray vertices;

    foreach (item: info.simplex) {
        vertices.push(item);
    };

    FaceArray faces;
    faces.push_all({
        vertices.create_face(0, 1, 2),
        vertices.create_face(0, 3, 1),
        vertices.create_face(0, 2, 3),
        vertices.create_face(1, 3, 2),
    });
    
    uint closest_face_index = faces.array_view().get_closest();

    Face closest_face;

    for (int i = 0; i < EPA_MAX_ITER; ++i) {
        closest_face = faces[closest_face_index];

        Vec3f face_normal = closest_face.normal;
        Vec3f a_support = shape_1.furthest_point(face_normal);
        Vec3f b_support = shape_2.furthest_point(-face_normal);

        SupportPoint support_point = {
            .a = a_support,
            .b = b_support,
            .point = a_support - b_support
        };

        float support_distance = face_normal.dot(support_point.point);

        if (support_distance - closest_face.distance < EPA_TOLERANCE) {
          
            // We have a hit
            info.normal = closest_face.normal;
            info.depth = closest_face.distance;

            Vec3f p = info.normal * -info.depth;
 
            SupportPoint a = vertices[closest_face.a];
            SupportPoint b = vertices[closest_face.b];
            SupportPoint c = vertices[closest_face.c];

            Vec3f bary = p.barycenter(a.point, b.point, c.point);

            Vec3f[2] support_a = {a.a, a.b};
            Vec3f[2] support_b = {b.a, b.b};
            Vec3f[2] support_c = {c.a, c.b};

            info.contact_points = {
                support_a[0] * bary.x + support_b[0] * bary.y + support_c[0] * bary.z,
                support_a[1] * bary.x + support_b[1] * bary.y + support_c[1] * bary.z,
            };
            return;
        }

        vertices.push(support_point);
        vertices.reconstruct(&faces, support_point.point);

        closest_face_index = faces.array_view().get_closest();
    }
    
    info.normal = closest_face.normal;
    info.depth = closest_face.distance;
}
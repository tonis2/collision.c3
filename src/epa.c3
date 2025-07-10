module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::io;

struct Face {
    usz a,b,c;
    Vec3f normal;
    // usz[3] indices;
    float distance;
}

struct CollisionInfo {
    Simplex simplex;
    Vec3f normal;
    Vec3f[2] contact_points;
    float depth;
    bool collided;
}

const uint EPA_MAX_FACES = 24;
const uint EPA_MAX_LOOSE_EDGES = EPA_MAX_FACES / 2;
const uint EPA_MAX_ITER = 32;
const float EPA_TOLERANCE = 0.0001;

alias EdgeArray = ElasticArray{Pair, EPA_MAX_LOOSE_EDGES};
alias FaceArray = ElasticArray{Face, EPA_MAX_FACES};
alias Polytope = ElasticArray{SupportPoint, EPA_MAX_FACES};

<*
 @require array.len > 0 : `Array should not be empty`
*>
fn void FaceArray.replace_array(&self, Face[] array, usz start_index = 0)
{
    self.size = array.len;
	foreach (index, &value : array) {self.entries[start_index + index] = *value;}
}

fn void epa(CollisionInfo* info, ConvexShape shape_1, ConvexShape shape_2) {
    FaceArray faces;
    Polytope polytope;
    polytope.add_array(info.simplex.array_view());

    faces.push({.a = 0, .b = 1, .c = 2});
    faces.push({.a = 0, .b = 3, .c = 1});
    faces.push({.a = 0, .b = 2, .c = 3});
    faces.push({.a = 1, .b = 3, .c = 2});

    for (int i = 0; i < EPA_MAX_ITER; ++i) {
        float min_distance = float.max;
        uint closest_face_index = 0;

        foreach (uint index, face: faces) {
            Vec3f p0 = polytope[face.a].point;
            Vec3f p1 = polytope[face.b].point;
            Vec3f p2 = polytope[face.c].point;

            Vec3f normal = (Vec3f){p1 - p0}.cross((Vec3f){p2 - p0}).normalize();
            float distance = normal.dot(p0);

            if (distance < 0) {
                normal *= -1.0f;
                distance *= -1.0;
            }

            faces[index].normal = normal;
            faces[index].distance = distance;

            if (distance < min_distance) {
                min_distance = distance;
                closest_face_index = index;
            }
        }

        Face closest_face = faces[closest_face_index];

        Vec3f a = shape_1.furthest_point(closest_face.normal);
        Vec3f b = shape_2.furthest_point(-closest_face.normal);

        SupportPoint new_support = {
            a - b,
            a,
            b
        };

        float support_distance = closest_face.normal.dot(new_support.point);

        if (support_distance - closest_face.distance < 0.001f) {
            info.normal = closest_face.normal;
            info.depth = support_distance;
            info.collided = true;

            Vec3f p_on_minkowski = closest_face.normal * closest_face.distance;

           // Collision found, get contact points
           
            SupportPoint s0 = polytope[closest_face.a];
            SupportPoint s1 = polytope[closest_face.b];
            SupportPoint s2 = polytope[closest_face.c];
            Vec3f bary = p_on_minkowski.barycentric(s0.point, s1.point, s2.point);

            info.contact_points = {
                bary.x * s0.support_a + bary.y * s1.support_a + bary.z * s2.support_a,
                bary.x * s0.support_b + bary.y * s1.support_b + bary.z * s2.support_b,
            };

            return;
        }

        polytope.push(new_support);

        usz new_point_idx = polytope.len() - 1;

        FaceArray new_faces;
        EdgeArray unique_edges;

        foreach (uint index, face: faces) {
            if (face.normal.dot(new_support.point - polytope[face.a].point) > 0) {
                Pair[] edges = {{face.a, face.b}, {face.b, face.c}, {face.c, face.a}};

                foreach (edge: edges) {
                    usz removed = unique_edges.remove_using_test(fn (first_edge, ctx) => *first_edge == *(Pair*)ctx, &&edge);
                    if (removed == 0) unique_edges.push(edge);
                }
            } else {
                new_faces.push(face);
            }
        }

        foreach (edge: unique_edges) {
            new_faces.push({.a = edge.first, .b = edge.second, .c = new_point_idx});
        }

        //TODO: Calculate face normals here for new_points only

        faces.replace_array(new_faces.array_view());
    }

    info.normal = faces[0].normal;
    info.depth = faces[0].distance;
    info.collided = false;
}
module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::io;

struct Face {
    Vec3f a,b,c;
    Vec3f normal;
    // usz[3] indices;
    float distance;
}

struct CollisionInfo {
    Simplex simplex;
    Vec3f normal;
    // Vec3f[2] contact_points;
    float depth;
    bool collided;
}

struct Edge {
    Vec3f a, b;
    uint index_a, index_b;
}

const uint EPA_MAX_FACES = 24;
const uint EPA_MAX_LOOSE_EDGES = EPA_MAX_FACES / 2;
const uint EPA_MAX_ITER = 32;
const float EPA_TOLERANCE = 0.0001;

alias EdgeArray = ElasticArray{Edge, EPA_MAX_LOOSE_EDGES};
alias FaceArray = ElasticArray{Face, EPA_MAX_FACES};

macro bool Edge.equals(self, Edge b) @operator(==) => self.a == b.b;

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
    Simplex simplex = info.simplex;

    faces.push({.a = simplex[0].point, .b = simplex[1].point, .c = simplex[2].point});
    faces.push({.a = simplex[0].point, .b = simplex[3].point, .c = simplex[1].point});
    faces.push({.a = simplex[0].point, .b = simplex[2].point, .c = simplex[3].point});
    faces.push({.a = simplex[1].point, .b = simplex[3].point, .c = simplex[2].point});

    for (int i = 0; i < EPA_MAX_ITER; ++i) {
        float min_distance = float.max;
        uint closest_face_index = 0;
      
        foreach (uint index, face: faces) {
            Vec3f normal = (Vec3f){face.b - face.a}.cross((Vec3f){face.c - face.a});
            float distance = normal.dot(face.a);

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
        Vec3f support_point = shape_1.furthest_point(closest_face.normal) - shape_2.furthest_point(-closest_face.normal);
        float support_distance = closest_face.normal.dot(support_point);

        if (support_distance - closest_face.distance < 0.001f) {
           info.normal = closest_face.normal;
           info.depth = support_distance;
           info.collided = true;
           return;
        }

        FaceArray new_faces;
        EdgeArray unique_edges;

        foreach (uint index, &face: faces) {
            if (face.normal.dot(support_point - face.a) > 0) {
                Edge[] edges = {{.a = face.a, .b = face.b}, {.a = face.b, .b = face.c}, {.a = face.c, .b = face.a}};

                foreach (edge: edges) {
                    usz removed = unique_edges.remove_using_test(fn (first_edge, ctx) => *first_edge == *(Edge*)ctx, &&edge);
                    if (removed == 0) unique_edges.push(edge);
                }
            } else {
                new_faces.push(*face);
            }
        }

        foreach (edge: unique_edges) {
            new_faces.push({.a = edge.a, .b = edge.b, .normal = support_point, .distance = support_distance});
        }

        faces.replace_array(new_faces.array_view());
    }

    info.normal = faces[0].normal;
    info.depth = faces[0].distance;
    info.collided = false;
}
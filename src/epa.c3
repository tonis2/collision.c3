module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

struct Face {
    Vec3f a,b,c;
    Vec3f normal;
    usz[3] indices;
    float distance;
}

struct CollisionInfo {
    Simplex simplex;
    Vec3f normal;
    Vec3f[2] contact_points;
    float depth;
    bool collided;
}

struct Plane {
    Vec3f normal;
    float distance;
}

fn Plane plane_from_tri(Vec3f a, Vec3f b, Vec3f c) {
    Vec3f ab = b - c;
    Vec3f ac = c - a;
    Vec3f normal = ab.cross(ac).normalize();

    return {
        .distance = -ab.dot(normal),
        .normal = normal 
    };
}

fn Vec3f plane_project(Plane plane, Vec3f point) {
    Vec3f distance = point.dot(plane.normal) + plane.distance;
    return point - (plane.normal * distance);
}

const uint EPA_MAX_FACES = 65;
const uint EPA_MAX_LOOSE_EDGES = EPA_MAX_FACES / 2;
const uint EPA_MAX_ITER = 32;
const float EPA_TOLERANCE = 0.0001;

alias PolytopeArray = ElasticArray{Vec3f, EPA_MAX_LOOSE_EDGES};
alias FaceArray = ElasticArray{usz, EPA_MAX_FACES};
alias PairArray = ElasticArray{Pair, EPA_MAX_LOOSE_EDGES};

fn void FaceArray.replace_array(&self, usz[] array, usz start_index = 0)
{
    self.size = array.len + start_index;
	foreach (index, &value : array) {self.entries[start_index + index] = *value;}
}

fn void PairArray.add_edge_if_unique(&self, Pair pair) {
    if (!self.contains(pair)) {self.push(pair);}
    else { self.remove_item(pair); }
}

fn Face find_closest_face(PolytopeArray polytope, FaceArray faces) {
    Face closest_face = { .distance = float.max };

    for (usz i = 0; i < faces.len(); i += 3) {
        Vec3f a = polytope[faces[i]];
        Vec3f b = polytope[faces[i + 1]];
        Vec3f c = polytope[faces[i + 2]];

        Vec3f normal = (b - a).cross(c - a).normalize();
        float dist = normal.dot(a);

        // io::printfn("%s %s %s", a, b, c);

        if (dist < 0) {
            dist *= -1;
            normal = -normal;
        }

        if (dist < closest_face.distance) {
            closest_face.distance = dist;
            closest_face.normal = normal;
            closest_face.a = a;
            closest_face.b = b;
            closest_face.c = c;
            closest_face.indices[0] = faces[i];
            closest_face.indices[1] = faces[i+1];
            closest_face.indices[2] = faces[i+2];
        }
    }

    return closest_face;
}

fn CollisionInfo epa(Simplex simplex, ConvexShape shape_1, ConvexShape shape_2) {
    FaceArray faces;
    PolytopeArray polytope;

    foreach (item: simplex) polytope.push(item.point);
   
    faces.add_array({
        0, 1, 2,
        0, 3, 1,
        0, 2, 3,
        1, 3, 2
    });

    for (int i = 0; i < EPA_MAX_ITER; ++i) {
        Face closest_face = find_closest_face(polytope, faces);

        Vec3f new_point = shape_1.furthest_point(closest_face.normal) - shape_2.furthest_point(-closest_face.normal);
        float new_dist = closest_face.normal.dot(new_point);

        if (math::abs(new_dist - closest_face.distance) < EPA_TOLERANCE) {
            return {
                .normal = closest_face.normal,
                .depth = closest_face.distance
            };
        }

        polytope.push(new_point);

        PairArray edges;
        FaceArray new_faces;

        for (usz j = 0; j < faces.len(); j += 3) {
            Vec3f a = polytope[faces[j]];
            Vec3f b = polytope[faces[j+1]];
            Vec3f c = polytope[faces[j+2]];
            Vec3f face_normal = (b-a).cross(c-a);

            // Can see the new point
            if (face_normal.dot(new_point - a) > 0) {
                edges.add_edge_if_unique({faces[j], faces[j+1]});
                edges.add_edge_if_unique({faces[j+1], faces[j+2]});
                edges.add_edge_if_unique({faces[j+2], faces[j]});
            } else {
                new_faces.push(faces[j]);
                new_faces.push(faces[j+1]);
                new_faces.push(faces[j+2]);
            }
        }

        foreach (edge : edges) {
            new_faces.push(edge.first);
            new_faces.push(edge.second);
            new_faces.push(polytope.len() - 1);
        }

        faces.replace_array(new_faces.array_view());
    }

    Face final_face = find_closest_face(polytope, faces);

    return {
        .normal = final_face.normal,
        .depth = final_face.distance
    };
}
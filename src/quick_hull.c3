module collision;

import std::collections::elastic_array;
import std::io;
import std::math;
import std::collections::list;
import std::collections::map;
import std::sort;


struct QuickHull {
    List{Face} faces;
}

fn float distance_to_plane(Vec3f point, Vec3f plane_point, Vec3f plane_normal) @local => (point - plane_point).dot(plane_normal);


fn void QuickHull.free(&self) {
    self.faces.free();
}

fn bool are_collinear(Vec3f a, Vec3f b, Vec3f c) @local => (b-a).cross(c-a).sq_magnitude() > 1e-9;
fn bool are_coplanar(Vec3f a, Vec3f b, Vec3f c, Vec3f d) @local {
   Vec3f normal = (b-a).cross(c-a);
   return math::abs(normal.dot(d-a)) > 1e-9;
}

fn float signed_distance(Vec3f point, Vec3f a, Vec3f b, Vec3f c) @local => (b-a).cross(c-a).dot(point-a);

<*
 @require points.len > 4 : `Hull input size must be > 4`
*>
fn void QuickHull.compute_from_points(&self, Vec3f[] points) {

    usz[6] extremes;

    usz p1_idx = 0;
    usz p2_idx = 0;
    usz p3_idx = 0;
    usz p4_idx = 0;

    foreach (usz i, point: points) {
        if (point.x > points[extremes[0]].x) extremes[0] = i;
        if (point.x < points[extremes[1]].x) extremes[1] = i;
        if (point.y > points[extremes[2]].y) extremes[2] = i;
        if (point.y < points[extremes[3]].y) extremes[3] = i;
        if (point.z > points[extremes[4]].z) extremes[4] = i;
        if (point.z < points[extremes[5]].z) extremes[5] = i;
    }    

    float max_dist = 0;

    for (usz i = 0;i < extremes.len ;i++) {
        for (usz j = i + 1;i < extremes.len ;i++) {
            Vec3f min = extremes[i];
            Vec3f max = extremes[j];

            float dist = (points[extremes[i]] - points[extremes[j]]).sq_magnitude();

            if (dist > max_dist) {
                max_dist = dist;
                p1_idx = extremes[i];
                p2_idx = extremes[j];
            }
        }
    }

    Vec3f line_dir = (points[p1_idx] - points[p2_idx]).normalize();

    max_dist = 0;
    // Find point furthest from the line between p1 and p2
    foreach (int i, point: points) {
        if (i == p1_idx || i == p2_idx) continue;
        Vec3f to_point = points[i] - points[p1_idx];
        Vec3f projection = line_dir * to_point.dot(line_dir);

        float dist = (to_point - projection).sq_magnitude();

        if (dist > max_dist) {
            max_dist = dist;
            p3_idx = i;
        }
    }

    Vec3f plane_normal = (points[p2_idx] - points[p1_idx]).cross(points[p3_idx] - points[p1_idx]).normalize();

    foreach (int i, point: points) {
        if (i == p1_idx || i == p2_idx || i == p3_idx) continue;
        float dist = distance_to_plane(points[i], points[p1_idx], plane_normal);

        if (dist > max_dist) {
            max_dist = dist;
            p4_idx = i;
        }
    }

    self.faces.clear();
    self.faces.push_all({
        {
            .a = p1_idx,
            .b = p2_idx,
            .c = p3_idx,
        },
        {
            .a = p2_idx,
            .b = p3_idx,
            .c = p4_idx,
        },
        {
            .a = p3_idx,
            .b = p4_idx,
            .c = p1_idx,
        },
        {
            .a = p4_idx,
            .b = p1_idx,
            .c = p2_idx,
        }
    });

    foreach (&face : self.faces) {
        Vec3f a = points[face.a];
        Vec3f b = points[face.b];
        Vec3f c = points[face.c];
        Vec3f normal = (b - a).cross(c - a).normalize();

        if (normal.dot(points[p1_idx] - points[p4_idx])) {
            normal *= -1;
        }

        face.normal = normal;
    }

    io::printfn("%s", self.faces.len());

    // foreach (usz i, point: points) {
    //     if (i == p1_idx || i == p2_idx || i == p3_idx || i == p4_idx) {
    //         continue;
    //     };
   
    //     self.add_point_to_hull(i, points);
    // }
}

fn void List{Pair}.add_edge(&self, Pair pair) {
    // If not removed, add new edge
    if (self.remove_item({pair.second, pair.first}) == 0) {
        self.push(pair);
    }
}

fn void QuickHull.add_point_to_hull(&self, usz index, Vec3f[] points) {
    List{usz} visible_faces;
    defer visible_faces.free();

    List{Pair} horizon;
    defer horizon.free();

    // foreach (point: points) {
    //     foreach (usz i, face: self.faces) {
    //         float dist = distance_to_plane(point, points[face.a], face.normal);
    //         if (dist > 1e-6) {
    //             visible_faces.push(i);
    //         }
    //     }
    // }

    // foreach (index: visible_faces) {
    //     Face face = self.faces[index]
    //     io::printfn("%s\n%s", points[face.a], points[face.b]);
    // }
    
}
module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;
import std::time;
import std::sort;

alias VertexList @local = List{Vec3};
alias IndexList @local = List{ushort};

struct ConvexPiece {
    List{Vec3} vertices;
    List{ushort} indices;
    Aabb3 aabb;
}

fn void ConvexPiece.free(&self) {
    self.vertices.free();
    self.indices.free();
}

<*
 @require mesh.vertices.len() > 0 : `mesh vertices should not be empty`
 @require concave_mesh != null : `mesh output must be an active list`
*>
fn void acd_recursive(ConvexPiece mesh, List{ConvexPiece}* concave_mesh, Number tolerance = 0.005) {

    ConvexPiece convex_piece = {
        .aabb = NEW_AABB3
    };

    convex_piece.vertices.add_all(&mesh.vertices);

    collision::hull_from_points(&convex_piece.vertices, &convex_piece.indices);

    foreach (point: convex_piece.vertices) convex_piece.aabb.expand_by_point(point);

    // Calculate concavity
    Plane split_plane;
    Number concavity = 0;
    usz triangle_count = convex_piece.indices.len() / 3;

    // For each triangle in the hull, find the mesh vertex deepest behind it
    for (usz i = 0; i < triangle_count; i++) {
        usz index = i * 3;

        Vec3 p0 = convex_piece.vertices.get(convex_piece.indices.get(index));
        Vec3 p1 = convex_piece.vertices.get(convex_piece.indices.get(index + 1));
        Vec3 p2 = convex_piece.vertices.get(convex_piece.indices.get(index + 2));

		Triangle triangle = {p0, p1, p2};

        Plane face_plane = triangle.get_plane();

        foreach (vertex: mesh.vertices) {
            float dist = face_plane.get_distance_to_point(vertex);
            
            if (dist < concavity) {
                concavity = dist;
                split_plane.normal = face_plane.normal;
            }
        }
    }

    if (math::abs(concavity) <= tolerance || mesh.vertices.len() <= 4) {
        concave_mesh.push(convex_piece);
        // This piece is "convex enough", store the hull
        return;
    }

    foreach (vertex: mesh.vertices) split_plane.point += vertex;
    split_plane.point /= mesh.vertices.len();

    // Free hull cause it's not used it end mesh.
    convex_piece.free();

    ConvexPiece left = {
        .aabb = NEW_AABB3
    };

    ConvexPiece right = {
        .aabb = NEW_AABB3
    };

    triangle_count = mesh.vertices.len() / 3;
    // Split mesh
    for (usz i = 0; i < triangle_count; i++) {
        usz index = i * 3;

        Vec3 p0 = mesh.vertices.get(index);
        Vec3 p1 = mesh.vertices.get(index + 1);
        Vec3 p2 = mesh.vertices.get(index + 2);

        Triangle triangle = {p0, p1, p2};

        Vec3 centroid = (p0 + p1 + p2) / 3;
		
        float dist = split_plane.get_distance_to_point(centroid);

        if (dist > 0) {
            left.vertices.push_all({p0, p1, p2});
        } else {
            right.vertices.push_all({p0, p1, p2});
        }
    }

    if (!left.vertices.is_empty()) acd_recursive(left, concave_mesh);
    if (!right.vertices.is_empty()) acd_recursive(right, concave_mesh);
}

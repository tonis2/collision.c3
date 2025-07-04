module collision;

import std::math;
import std::io;
import std::collections::list;
import std::collections::elastic_array;


alias OctreeList = ElasticArray{Octree*, 8};

const uint MAX_DEPTH = 8;

struct Octree {
    Aabb3 boundary;
    List{Triangle} triangles;
    OctreeList children;
    bool subdivided;
    uint depth;
    uint capacity;
}

fn void Octree.free(&self) {
    foreach (child: self.children) child.free();
    self.triangles.free();
}    

fn void Octree.subdivide(&self) {
    Vec3f size = {
        (self.boundary.max.x - self.boundary.min.x) / (float)2.0,
        (self.boundary.max.y - self.boundary.min.y) / (float)2.0,
        (self.boundary.max.z - self.boundary.min.z) / (float)2.0
    };

    Vec3f center = {
        self.boundary.min.x + size.x,
        self.boundary.min.y + size.y,
        self.boundary.min.z + size.z
    };

    Aabb3[8] octants = {
        {self.boundary.min, center},
        {{center.x, self.boundary.min.y, self.boundary.min.z}, {self.boundary.max.x, center.y, center.z}},
        {{center.x, self.boundary.min.y, center.z}, {self.boundary.max.x, center.y, self.boundary.max.z}},
        {{self.boundary.min.x, self.boundary.min.y, center.z}, {center.x, center.y, self.boundary.max.z}},
        {{self.boundary.min.x, center.y, self.boundary.min.z}, {center.x, self.boundary.max.y, center.z}},
        {{center.x, center.y, self.boundary.min.z}, {self.boundary.max.x, self.boundary.max.y, center.z}},
        {{center.x, center.y, center.z}, self.boundary.max},
        {{self.boundary.min.x, center.y, center.z}, {center.x, self.boundary.max.y, self.boundary.max.z}}
    };

    foreach (uint i, tree: octants) {
        self.children[i] = &&(Octree) {.boundary = octants[i], .capacity = self.capacity, .depth = self.depth + 1};
    }

    self.subdivided = true;

    foreach (tri: self.triangles) {
        foreach (child: self.children) {
            child.insert(tri);
        }
    }

    self.triangles.clear();
}

fn bool Octree.insert(&self, Triangle triangle) {
    // If the triangle doesn't belong in this node, ignore it
    if (!self.boundary.collides(triangle.get_aabb())) {
        return false;
    }

    // If we have space and haven't subdivided, add it here
    if (self.triangles.len() < self.capacity && !self.subdivided) {
        self.triangles.push(triangle);
        return true;
    }

    // If we've reached capacity, subdivide
    if (!self.subdivided) {
        if (self.depth >= MAX_DEPTH) {
            // Still add triangle here if max depth is reached
            self.triangles.push(triangle);
            return true;
        }
        self.subdivide();
    }

    // Pass the triangle down to the children
    bool inserted = false;

    foreach (tri: self.triangles) {
        foreach (child: self.children) {
            child.insert(tri);
            inserted = true;
        }
    }

    return inserted;
}


fn void Octree.query(&self, Aabb3 range, List{Triangle}* found) {

    if (!self.boundary.collides(range)) {
        return;
    }

    // If subdivided, query the children
    if (self.subdivided) {
        foreach (child: self.children) child.query(range, found);
    } else {
        // Otherwise, check triangles in this leaf node
        foreach (tri: self.triangles) {
            if (range.collides(tri.get_aabb())) {
                found.push(tri);
            }
        }
    }
}
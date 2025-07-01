module collision;

import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

alias CellList = List{usz};
alias SpatialMap = HashMap{usz, CellList};

struct GridAabb3 {
    Vec3f min;
    Vec3f max;
    usz id;
}

struct GridCoord {
    int x, y, z;
}

struct SpatialHash3D {
    double cell_size;
    SpatialMap table;
    HashMap{usz, bool} unique_map;
}

fn bool GridCoord.equals(&self, GridCoord other) => self.x == other.x && self.y == other.y && self.z == other.z;

fn usz GridCoord.hash(&self) {
    usz h1 = self.x.hash();
    usz h2 = self.y.hash();
    usz h3 = self.z.hash();
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

fn void SpatialHash3D.free(&self) {
    self.table.@each(; usz id, CellList cells) {
        cells.free();
    };
    self.table.free();
    self.unique_map.free();
}

fn GridCoord SpatialHash3D.world_to_grid(&self, Vec3f pos) {
    return (GridCoord) {
        (int)(math::floor(pos.x / self.cell_size)),
        (int)(math::floor(pos.y / self.cell_size)),
        (int)(math::floor(pos.z / self.cell_size)),
    };
}

fn void? SpatialHash3D.insert(&self, GridAabb3 box) {
    GridCoord min_cell = self.world_to_grid(box.min);
    GridCoord max_cell = self.world_to_grid(box.max);

    for (int x = min_cell.x; x <= max_cell.x; ++x) {
        for (int y = min_cell.y; y <= max_cell.y; ++y) {

            for (int z = min_cell.z; z <= max_cell.z; ++z) {
                usz cell_id = (GridCoord){x, y, z}.hash();

                if (!self.table.has_key(cell_id)) {
                    self.table.set(cell_id, {});
                };

                self.table.get_ref(cell_id)!.push(box.id);
            }
        }
    }
}

macro void SpatialHash3D.@get_pairs(&self; @callback(Pair pair)) {
    self.table.@each(;usz id, CellList cells) {
        for (usz i = 0; i < cells.len(); ++i) {
            for (usz j = i + 1; j < cells.len(); ++j) {
                usz a = cells[i];
                usz b = cells[j];

                Pair pair = {math::min(a, b), math::max(a, b)};
                usz pair_hash = pair.hash();

                if (!self.unique_map.has_key(pair_hash)) {
                    @callback(pair);
                    self.unique_map.set(pair_hash, true);
                }
            }
        }
    };

    self.unique_map.clear();
}

macro void SpatialHash3D.@get_pairs_in_region(&self, Vec3f min, Vec3f max ; @callback(Pair pair)) {
    GridCoord min_cell = self.world_to_grid(box.min);
    GridCoord max_cell = self.world_to_grid(box.max);

    for (uint x = min_cell.x; x <= max_cell.x; ++x) {
    for (uint y = min_cell.y; y <= max_cell.y; ++y) {
            for (uint z = min_cell.z; z <= max_cell.z; ++z) {
                usz cell_id = (GridCoord){x, y, z}.hash();
                CellList cells = self.table.get(cell_id);
                
                for (usz i = 0; i < cells.len(); ++i) {
                    for (usz j = i + 1; j < cells.len(); ++j) {
                        usz a = cells[i];
                        usz b = cells[j];
                        Pair pair = {math::min(a, b), math::max(a, b)};
                        
                        usz pair_hash = pair.hash();
                        if (!self.unique_map.has_key(pair_hash)) {
                            @callback(pair);
                            self.unique_map.set(pair_hash, true);
                        }
                    }
                }
            }
        }
    }

    self.unique_map.clear();
}

macro void SpatialHash3D.@get_neighbor_cells(&self, GridCoord cell; @callback(Vec3f position)) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                @callback({cell.x + dx, cell.y + dy, cell.z + dz});
            }
        }
    }
}

macro void SpatialHash3D.@get_nearby_objects(&self, Vec3f pos; @callback(usz id)) {
    GridCoord cell = self.world_to_grid(pos);

    self.@get_neighbor_cells(cell; Vec3f position) {
        usz cell_id = (GridCoord){position.x, position.y, position.z}.hash();
        CellList cells = self.table.get(cell_id);
        foreach (cell: cells) @callback(cell);
    };
}
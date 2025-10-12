module collision;

import std::io;
import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

alias CellList = List{usz};
alias SpatialMap = HashMap{int, CellList};

struct GridCoord {
    int x, y, z;
}

struct SpatialHash3D {
    double cell_size;
    SpatialMap table;
    HashMap{usz, bool} unique_map;
}

fn usz? GridCoord.to_format(&self, Formatter* formatter) @dynamic => formatter.printf("%s %s %s", self.x, self.y, self.z)!;

fn bool GridCoord.equals(&self, GridCoord other) => self.x == other.x && self.y == other.y && self.z == other.z;

fn int GridCoord.hash(&self) {    
    int hash = 17;
    hash = hash * 23 + self.x;
    hash = hash * 23 + self.y;
    hash = hash * 23 + self.z;
    return hash;
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

fn void? SpatialHash3D.insert(&self, Aabb3 box, usz id) {
    GridCoord min_cell = self.world_to_grid(box.min);
    GridCoord max_cell = self.world_to_grid(box.max);

    for (int x = min_cell.x; x <= max_cell.x; ++x) {
        for (int y = min_cell.y; y <= max_cell.y; ++y) {
            for (int z = min_cell.z; z <= max_cell.z; ++z) {
                int cell_id = self.world_to_grid({x, y, z}).hash();
                if (!self.table.has_key(cell_id)) {
                    self.table.set(cell_id, {});
                };

                self.table.get_ref(cell_id)!.push(id);
            }
        }
    }
}

alias CellIDs = ElasticArray{int, 200};

fn void? SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, usz entity) {
    if (old_box == new_box) return;

    GridCoord new_min_cell = self.world_to_grid(new_box.min);
    GridCoord new_max_cell = self.world_to_grid(new_box.max);

    GridCoord old_min_cell = self.world_to_grid(old_box.min);
    GridCoord old_max_cell = self.world_to_grid(old_box.max);

    CellIDs old_ids, new_ids;
    
    for (int x = old_min_cell.x; x <= old_max_cell.x; ++x) {
        for (int y = old_min_cell.y; y <= old_max_cell.y; ++y) {
            for (int z = old_min_cell.z; z <= old_max_cell.z; ++z) {
                int cell_id = self.world_to_grid({x, y, z}).hash();
                old_ids.push(cell_id);
            }
        }
    }

    for (int x = new_min_cell.x; x <= new_max_cell.x; ++x) {
        for (int y = new_min_cell.y; y <= new_max_cell.y; ++y) {
            for (int z = new_min_cell.z; z <= new_max_cell.z; ++z) {
                int cell_id = self.world_to_grid({x, y, z}).hash();
                new_ids.push(cell_id);      
            }
        }
    }

    foreach (cell: old_ids) {
        CellList*? cells = self.table.get_ref(cell);
        if (try cells) {
            cells.remove_using_test(fn (id, ctx) => *id == *(usz*)ctx, &&entity);
            if (cells.is_empty()) self.table.remove(cell);
        };
    }

    foreach (cell: new_ids) {
        if (old_ids.contains(cell)) {
            if (!self.table.has_key(cell)) {
                self.table.set(cell, {});
            };

            self.table.get_ref(cell)!.push(entity);
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

    for (int x = min_cell.x; x <= max_cell.x; ++x) {
        for (int y = min_cell.y; y <= max_cell.y; ++y) {
                for (int z = min_cell.z; z <= max_cell.z; ++z) {
                    int cell_id = self.world_to_grid({x, y, z}).hash();
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
        int cell_id = self.world_to_grid({position.x, position.y, position.z}).hash();
        CellList cells = self.table.get(cell_id);
        foreach (cell: cells) @callback(cell);
    };
}
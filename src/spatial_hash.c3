module collision;

import std::io;
import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

alias CellList = List{usz};
alias SpatialMap = HashMap{usz, CellList};

struct SpatialHash3D {
    Number cell_size;
    SpatialMap table;
    HashMap{usz, bool} unique_map;
}

fn void SpatialHash3D.free(&self) {
    self.table.@each(; usz id, CellList cells) {
        cells.free();
    };
    self.table.free();
    self.unique_map.free();
}

fn Vec3 SpatialHash3D.world_to_grid(&self, Vec3 pos) {
    return (Vec3) {
        math::floor(pos.x / self.cell_size),
        math::floor(pos.y / self.cell_size),
        math::floor(pos.z / self.cell_size),
    };
}

<*
 Adds a 3D AABB box into SpatialHash
 @param box : `3D aabb box, that's inserted into the map`
 @param id : `ID thats attached to the box, could be mesh ID`
 @require box.size().sq_magnitude() < 10000 : `Aabb box is too big, could be because memory errors`
 @require box.size().sq_magnitude() > 0.1 : `Aabb box is too small, could be because memory errors`
*>    
fn void? SpatialHash3D.insert(&self, Aabb3 box, usz id) {
    Vec3 min_cell = self.world_to_grid(box.min);
    Vec3 max_cell = self.world_to_grid(box.max);

    for (Number x = min_cell.x; x <= max_cell.x; ++x) {
        for (Number y = min_cell.y; y <= max_cell.y; ++y) {
            for (Number z = min_cell.z; z <= max_cell.z; ++z) {
                usz cell_id = self.world_to_grid({x, y, z}).hash();
                if (!self.table.has_key(cell_id)) {
                    self.table.set(cell_id, {});
                };

                self.table.get_ref(cell_id)!.push(id);
            }
        }
    }
}

alias CellIDs = ElasticArray{usz, 60};


<*
 Updates box location in spatial map
 @param old_box : `Previous box location`
 @param new_box : `New box location`
 @param id : `Box id`
 @require old_box.size().sq_magnitude() < 10000 && old_box.size().sq_magnitude() > 0.1 : `Aabb box size invalid, could be because memory errors`
 @require new_box.size().sq_magnitude() < 10000 && new_box.size().sq_magnitude() > 0.1 : `Aabb box size invalid, could be because memory errors`
*>    
fn void? SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, usz id) {
    if (old_box == new_box) return;

    Vec3 new_min_cell = self.world_to_grid(new_box.min);
    Vec3 new_max_cell = self.world_to_grid(new_box.max);

    Vec3 old_min_cell = self.world_to_grid(old_box.min);
    Vec3 old_max_cell = self.world_to_grid(old_box.max);

    CellIDs old_ids, new_ids;
    
    for (Number x = old_min_cell.x; x <= old_max_cell.x; ++x) {
        for (Number y = old_min_cell.y; y <= old_max_cell.y; ++y) {
            for (Number z = old_min_cell.z; z <= old_max_cell.z; ++z) {
                usz cell_id = self.world_to_grid({x, y, z}).hash();
                old_ids.push(cell_id);
            }
        }
    }

    for (Number x = new_min_cell.x; x <= new_max_cell.x; ++x) {
        for (Number y = new_min_cell.y; y <= new_max_cell.y; ++y) {
            for (Number z = new_min_cell.z; z <= new_max_cell.z; ++z) {
                usz cell_id = self.world_to_grid({x, y, z}).hash();
                new_ids.push(cell_id);      
            }
        }
    }

    foreach (cell: old_ids) {
        CellList*? cells = self.table.get_ref(cell);
        if (try cells) {
            cells.remove_using_test(fn (id, ctx) => *id == *(usz*)ctx, &&id);
            if (cells.is_empty()) self.table.remove(cell);
        };
    }

    foreach (cell: new_ids) {
        if (old_ids.contains(cell)) {
            if (!self.table.has_key(cell)) {
                self.table.set(cell, {});
            };

            self.table.get_ref(cell)!.push(id);
        }
    }
}

macro void SpatialHash3D.@get_pairs(&self; @callback(Pair pair)) {
    self.table.@each(;usz id, CellList cells) {
        for (usz i = 0; i < cells.len(); ++i) {
            for (usz j = i + 1; j < cells.len(); ++j) {
                usz a = cells[i];
                usz b = cells[j];

                if (a == b) continue;
                
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

macro void SpatialHash3D.@get_pairs_in_region(&self, Vec3 min, Vec3 max ; @callback(Pair pair)) {
    Vec3 min_cell = self.world_to_grid(box.min);
    Vec3 max_cell = self.world_to_grid(box.max);

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

macro void SpatialHash3D.@get_neighbor_cells(&self, Vec3 cell; @callback(Vec3 position)) {
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                @callback({cell.x + dx, cell.y + dy, cell.z + dz});
            }
        }
    }
}

macro void SpatialHash3D.@get_nearby_objects(&self, Vec3 pos; @callback(usz id)) {
    Vec3 cell = self.world_to_grid(pos);

    self.@get_neighbor_cells(cell; Vec3 position) {
        int cell_id = self.world_to_grid({position.x, position.y, position.z}).hash();
        CellList cells = self.table.get(cell_id);
        foreach (cell: cells) @callback(cell);
    };
}
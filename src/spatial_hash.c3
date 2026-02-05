module collision;

import std::io;
import std::math;
import std::collections::list;
import std::collections::elastic_array;
import std::collections::map;

alias CellList = List{usz};
alias SpatialMap = HashMap{usz, CellList};

struct SpatialHash3D {
    float cell_size;
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
 @require box.size().sq_magnitude() < 100000 : `Aabb box is too big, could be because memory errors`
 @require box.size().sq_magnitude() > 0.1 : `Aabb box is too small, could be because memory errors`
*>    
fn void? SpatialHash3D.insert(&self, Aabb3 box, usz id) {
    Vec3 min_cell = self.world_to_grid(box.min);
    Vec3 max_cell = self.world_to_grid(box.max);

    for (int x = (int)min_cell.x; x <= (int)max_cell.x; ++x) {
        for (int y = (int)min_cell.y; y <= (int)max_cell.y; ++y) {
            for (int z = (int)min_cell.z; z <= (int)max_cell.z; ++z) {
                usz cell_id = (Vec3){(float)x, (float)y, (float)z}.hash();
                if (!self.table.has_key(cell_id)) {
                    self.table.set(cell_id, {});
                }
                self.table.get_ref(cell_id)!.push(id);
            }
        }
    }
}

<*
 Updates box location in spatial map
 @param old_box : `Previous box location`
 @param new_box : `New box location`
 @param id : `Box id`
 @require old_box.size().sq_magnitude() < 10000 && old_box.size().sq_magnitude() > 0.1 : `Aabb box size invalid, could be because memory errors`
 @require new_box.size().sq_magnitude() < 10000 && new_box.size().sq_magnitude() > 0.1 : `Aabb box size invalid, could be because memory errors`
*>
fn void? SpatialHash3D.update(&self, Aabb3 old_box, Aabb3 new_box, usz id) {
    Vec3 old_min_cell = self.world_to_grid(old_box.min);
    Vec3 old_max_cell = self.world_to_grid(old_box.max);

    Vec3 new_min_cell = self.world_to_grid(new_box.min);
    Vec3 new_max_cell = self.world_to_grid(new_box.max);

    // Early exit if grid cells haven't changed
    if (old_min_cell == new_min_cell && old_max_cell == new_max_cell) return;

    int old_min_x = (int)old_min_cell.x;
    int old_max_x = (int)old_max_cell.x;
    int old_min_y = (int)old_min_cell.y;
    int old_max_y = (int)old_max_cell.y;
    int old_min_z = (int)old_min_cell.z;
    int old_max_z = (int)old_max_cell.z;

    int new_min_x = (int)new_min_cell.x;
    int new_max_x = (int)new_max_cell.x;
    int new_min_y = (int)new_min_cell.y;
    int new_max_y = (int)new_max_cell.y;
    int new_min_z = (int)new_min_cell.z;
    int new_max_z = (int)new_max_cell.z;

    // Remove from old cells that are NOT in new range
    for (int x = old_min_x; x <= old_max_x; ++x) {
        for (int y = old_min_y; y <= old_max_y; ++y) {
            for (int z = old_min_z; z <= old_max_z; ++z) {
                bool in_new = x >= new_min_x && x <= new_max_x &&
                              y >= new_min_y && y <= new_max_y &&
                              z >= new_min_z && z <= new_max_z;
                if (!in_new) {
                    usz cell_id = (Vec3){(float)x, (float)y, (float)z}.hash();
                    CellList*? cells = self.table.get_ref(cell_id);
                    if (try cells) {
                        cells.remove_using_test(fn (id, ctx) => *id == *(usz*)ctx, &&id);
                        if (cells.is_empty()) self.table.remove(cell_id);
                    }
                }
            }
        }
    }

    // Add to new cells that were NOT in old range
    for (int x = new_min_x; x <= new_max_x; ++x) {
        for (int y = new_min_y; y <= new_max_y; ++y) {
            for (int z = new_min_z; z <= new_max_z; ++z) {
                bool in_old = x >= old_min_x && x <= old_max_x &&
                              y >= old_min_y && y <= old_max_y &&
                              z >= old_min_z && z <= old_max_z;
                if (!in_old) {
                    usz cell_id = (Vec3){(float)x, (float)y, (float)z}.hash();
                    if (!self.table.has_key(cell_id)) {
                        self.table.set(cell_id, {});
                    }
                    self.table.get_ref(cell_id)!.push(id);
                }
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
        usz cell_id = self.world_to_grid({position.x, position.y, position.z}).hash();
        if (try cells = self.table.get(cell_id)) {
            foreach (item: cells) @callback(item);
        }
    };
}
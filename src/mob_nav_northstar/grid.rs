use bevy::prelude::*;
use bevy_northstar::grid::GridSettings;
use bevy_northstar::prelude::{CardinalIsoGrid, GridSettingsBuilder, Nav, filter};
use bevy_voxel_world::prelude::VoxelWorld;

use crate::cave_world::CaveWorld;

use super::MobNavNorthstarConfig;

#[derive(Resource, Default)]
pub(crate) struct MobNavNorthstarRollingGrid {
    pub(crate) grid_entity: Option<Entity>,
    pub(crate) min_world: IVec3,
    pub(crate) max_world: IVec3,
    pub(crate) revision: u64,
    pub(crate) initialized: bool,
}

pub(crate) fn ensure_rolling_grid_for_player(
    commands: &mut Commands,
    rolling: &mut MobNavNorthstarRollingGrid,
    voxel_world: &VoxelWorld<CaveWorld>,
    config: &MobNavNorthstarConfig,
    player_world: IVec3,
) {
    let center_world = IVec3::new(
        player_world.x,
        player_world.y.clamp(config.min_world_y, config.max_world_y),
        player_world.z,
    );
    let dimensions = effective_dimensions(config);

    let needs_rebuild = !rolling.initialized
        || !is_inside_bounds_with_margin(
            player_world,
            rolling.min_world,
            rolling.max_world,
            config.recenter_margin_voxels,
        );
    if !needs_rebuild {
        return;
    }

    let (min_world, max_world) = rolling_bounds(center_world, dimensions, config);
    let grid = build_grid(voxel_world, config, dimensions, min_world);

    if let Some(grid_entity) = rolling.grid_entity {
        commands.entity(grid_entity).insert(grid);
    } else {
        let grid_entity = commands.spawn((Name::new("NorthstarGrid"), grid)).id();
        rolling.grid_entity = Some(grid_entity);
    }

    rolling.min_world = min_world;
    rolling.max_world = max_world;
    rolling.revision = rolling.revision.wrapping_add(1);
    rolling.initialized = true;
}

pub(crate) fn world_to_local(world: IVec3, min_world: IVec3, max_world: IVec3) -> Option<UVec3> {
    if world.cmplt(min_world).any() || world.cmpgt(max_world).any() {
        return None;
    }

    // CardinalIso uses Z as vertical; we map Y-up world into grid-local coordinates.
    Some(UVec3::new(
        (world.x - min_world.x) as u32,
        (world.z - min_world.z) as u32,
        (world.y - min_world.y) as u32,
    ))
}

pub(crate) fn local_to_world(local: UVec3, min_world: IVec3, center_offset: Vec3) -> Vec3 {
    let world = local_to_world_ivec(local, min_world);
    world.as_vec3() + center_offset
}

pub(crate) fn find_nearest_walkable_local(
    voxel_world: &VoxelWorld<CaveWorld>,
    seed_world: IVec3,
    min_world: IVec3,
    max_world: IVec3,
    max_radius: i32,
    agent_height_voxels: i32,
) -> Option<UVec3> {
    let seed_local = world_to_local(seed_world, min_world, max_world)?;
    if is_walkable_cell(voxel_world, seed_world, agent_height_voxels) {
        return Some(seed_local);
    }

    for radius in 1..=max_radius.max(0) {
        let mut best: Option<(i32, UVec3)> = None;

        for z in (seed_world.z - radius)..=(seed_world.z + radius) {
            for y in (seed_world.y - radius)..=(seed_world.y + radius) {
                for x in (seed_world.x - radius)..=(seed_world.x + radius) {
                    if x != seed_world.x - radius
                        && x != seed_world.x + radius
                        && y != seed_world.y - radius
                        && y != seed_world.y + radius
                        && z != seed_world.z - radius
                        && z != seed_world.z + radius
                    {
                        continue;
                    }

                    let world = IVec3::new(x, y, z);
                    let Some(local) = world_to_local(world, min_world, max_world) else {
                        continue;
                    };
                    if !is_walkable_cell(voxel_world, world, agent_height_voxels) {
                        continue;
                    }

                    let dist2 = (world - seed_world).length_squared();
                    match best {
                        Some((best_dist2, _)) if dist2 >= best_dist2 => {}
                        _ => best = Some((dist2, local)),
                    }
                }
            }
        }

        if let Some((_, nearest)) = best {
            return Some(nearest);
        }
    }

    None
}

fn local_to_world_ivec(local: UVec3, min_world: IVec3) -> IVec3 {
    IVec3::new(
        min_world.x + local.x as i32,
        min_world.y + local.z as i32,
        min_world.z + local.y as i32,
    )
}

fn effective_dimensions(config: &MobNavNorthstarConfig) -> IVec3 {
    let world_height_span = (config.max_world_y - config.min_world_y + 1).max(1);
    let width = (config.grid_width.max(1)) as i32;
    let height = ((config.grid_height.max(1)) as i32).min(world_height_span);
    let depth = (config.grid_depth.max(1)) as i32;
    IVec3::new(width, height, depth)
}

fn rolling_bounds(
    center_world: IVec3,
    dimensions: IVec3,
    config: &MobNavNorthstarConfig,
) -> (IVec3, IVec3) {
    let half = dimensions / 2;

    let mut min_world = IVec3::new(
        center_world.x - half.x,
        center_world.y - half.y,
        center_world.z - half.z,
    );
    let mut max_world = min_world + dimensions - IVec3::ONE;

    if min_world.y < config.min_world_y {
        let shift = config.min_world_y - min_world.y;
        min_world.y += shift;
        max_world.y += shift;
    }
    if max_world.y > config.max_world_y {
        let shift = max_world.y - config.max_world_y;
        min_world.y -= shift;
        max_world.y -= shift;
    }

    (min_world, max_world)
}

fn is_inside_bounds_with_margin(
    world: IVec3,
    min_world: IVec3,
    max_world: IVec3,
    margin: i32,
) -> bool {
    let margin = margin.max(0);
    let x_margin = margin.min((max_world.x - min_world.x).max(0) / 2);
    let y_margin = margin.min((max_world.y - min_world.y).max(0) / 2);
    let z_margin = margin.min((max_world.z - min_world.z).max(0) / 2);

    world.x >= min_world.x + x_margin
        && world.x <= max_world.x - x_margin
        && world.y >= min_world.y + y_margin
        && world.y <= max_world.y - y_margin
        && world.z >= min_world.z + z_margin
        && world.z <= max_world.z - z_margin
}

fn build_grid(
    voxel_world: &VoxelWorld<CaveWorld>,
    config: &MobNavNorthstarConfig,
    dimensions: IVec3,
    min_world: IVec3,
) -> CardinalIsoGrid {
    let settings = grid_settings_from_dimensions(dimensions, config);
    let mut grid = CardinalIsoGrid::new(&settings);
    let agent_height_voxels = config.agent_height_voxels.max(1);

    for grid_z in 0..dimensions.y {
        for grid_y in 0..dimensions.z {
            for grid_x in 0..dimensions.x {
                let local = UVec3::new(grid_x as u32, grid_y as u32, grid_z as u32);
                let world = local_to_world_ivec(local, min_world);
                if is_walkable_cell(voxel_world, world, agent_height_voxels) {
                    grid.set_nav(local, Nav::Passable(1));
                }
            }
        }
    }

    grid.build();
    grid
}

fn grid_settings_from_dimensions(
    dimensions: IVec3,
    config: &MobNavNorthstarConfig,
) -> GridSettings {
    GridSettingsBuilder::new_3d(
        dimensions.x as u32,
        dimensions.z as u32,
        dimensions.y as u32,
    )
    .chunk_size(config.chunk_size.max(1))
    .chunk_depth(config.chunk_depth.max(1))
    .default_impassable()
    .add_neighbor_filter(filter::NoCornerCuttingFlat)
    .build()
}

pub(crate) fn is_walkable_cell(
    voxel_world: &VoxelWorld<CaveWorld>,
    world: IVec3,
    agent_height_voxels: i32,
) -> bool {
    let floor_voxel = voxel_world.get_voxel(world - IVec3::Y);
    if !floor_voxel.is_solid() {
        return false;
    }

    for clearance in 0..agent_height_voxels {
        let current = voxel_world.get_voxel(world + IVec3::Y * clearance);
        if !current.is_air() {
            return false;
        }
    }

    true
}

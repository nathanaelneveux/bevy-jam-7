//! Enemy navigation grid lifecycle.
//!
//! This module owns the Northstar grid setup for enemy agents:
//! - config defaults
//! - grid build/spawn
//! - world/grid coordinate conversion
//! - dynamic recentering for infinite worlds

use bevy::prelude::*;
use bevy_northstar::prelude::*;
use serde::Deserialize;

use crate::{
    cave_noise::{CEILING_MAX_Y, CaveNoise, FLOOR_MIN_Y},
    player_controller::Player,
};

/// Offset from cell min corner to center in world space.
const ENEMY_CELL_CENTER_OFFSET: f32 = 0.5;
/// Fallback recenter margin in cells when not configured.
const ENEMY_NAV_RECENTER_MARGIN_CELLS: u32 = 16;

/// Settings used to generate and maintain a Northstar 3D grid.
#[derive(Deserialize, Clone, Debug)]
pub(crate) struct EnemyNavigationConfig {
    /// World-space grid minimum corner.
    #[serde(default = "default_world_min")]
    pub(crate) world_min: [i32; 3],
    /// Grid dimensions in cells.
    #[serde(default = "default_nav_dimensions")]
    pub(crate) dimensions: [u32; 3],
    /// Northstar chunk width/height.
    #[serde(default = "default_chunk_size")]
    pub(crate) chunk_size: u32,
    /// Northstar chunk depth.
    #[serde(default = "default_chunk_depth")]
    pub(crate) chunk_depth: u32,
    /// Distance from X/Z grid edge that triggers a recenter rebuild.
    #[serde(default = "default_recenter_margin")]
    pub(crate) recenter_margin: u32,
}

impl Default for EnemyNavigationConfig {
    fn default() -> Self {
        Self {
            world_min: default_world_min(),
            dimensions: default_nav_dimensions(),
            chunk_size: default_chunk_size(),
            chunk_depth: default_chunk_depth(),
            recenter_margin: default_recenter_margin(),
        }
    }
}

/// Active pathfinding grid resource.
#[derive(Resource, Clone, Copy)]
pub(crate) struct EnemyNavigationGrid {
    /// Entity that owns the Northstar grid component.
    pub(crate) entity: Entity,
    /// Inclusive world-space origin (minimum corner) for grid mapping.
    pub(crate) world_min: IVec3,
    /// Grid dimensions in cells.
    pub(crate) dimensions: UVec3,
    /// Northstar chunk width/height.
    pub(crate) chunk_size: u32,
    /// Northstar chunk depth.
    pub(crate) chunk_depth: u32,
    /// Rebuild trigger distance from edge in cells.
    pub(crate) recenter_margin: u32,
}

/// Marker for cleanup/rebuild of the spawned nav grid entity.
#[derive(Component)]
pub(crate) struct EnemyNavigationGridMarker;

/// Marker for entities managed by the enemy navigation lifecycle.
#[derive(Component)]
pub(crate) struct EnemyNavigationAgent;

/// Builds and spawns the active navigation grid entity from config.
pub(crate) fn create_navigation_grid(
    commands: &mut Commands,
    config: &EnemyNavigationConfig,
) -> EnemyNavigationGrid {
    let dimensions = UVec3::new(
        config.dimensions[0].max(3),
        config.dimensions[1].max(1),
        config.dimensions[2].max(3),
    );
    let world_min = IVec3::new(
        config.world_min[0],
        config.world_min[1],
        config.world_min[2],
    );
    let chunk_size = config.chunk_size.max(3);
    let chunk_depth = config.chunk_depth.max(1);
    let max_margin = (dimensions.x.min(dimensions.z) / 2).max(1);
    let recenter_margin = config.recenter_margin.min(max_margin).max(1);

    let grid = build_navigation_grid(world_min, dimensions, chunk_size, chunk_depth);
    let entity = commands.spawn((grid, EnemyNavigationGridMarker)).id();

    EnemyNavigationGrid {
        entity,
        world_min,
        dimensions,
        chunk_size,
        chunk_depth,
        recenter_margin,
    }
}

/// Rebuilds the navigation grid around the player when they approach X/Z bounds.
pub(crate) fn recenter_navigation_grid_around_player(
    mut commands: Commands,
    player: Query<&Transform, With<Player>>,
    nav_grid: Option<ResMut<EnemyNavigationGrid>>,
    mut agents: Query<(Entity, &Transform, &mut AgentPos), With<EnemyNavigationAgent>>,
) {
    let Some(mut nav_grid) = nav_grid else {
        return;
    };
    let Ok(player_transform) = player.single() else {
        return;
    };

    let player_voxel = IVec3::new(
        player_transform.translation.x.floor() as i32,
        player_transform.translation.y.floor() as i32,
        player_transform.translation.z.floor() as i32,
    );
    let local = player_voxel - nav_grid.world_min;
    let dims = nav_grid.dimensions.as_ivec3();
    let margin = nav_grid.recenter_margin as i32;
    let near_x_edge = local.x < margin || local.x >= dims.x - margin;
    let near_z_edge = local.z < margin || local.z >= dims.z - margin;

    if !near_x_edge && !near_z_edge {
        return;
    }

    let new_world_min = IVec3::new(
        player_voxel.x - dims.x / 2,
        nav_grid.world_min.y,
        player_voxel.z - dims.z / 2,
    );
    if new_world_min == nav_grid.world_min {
        return;
    }

    let new_grid = build_navigation_grid(
        new_world_min,
        nav_grid.dimensions,
        nav_grid.chunk_size,
        nav_grid.chunk_depth,
    );
    let old_grid_entity = nav_grid.entity;
    let new_grid_entity = commands.spawn((new_grid, EnemyNavigationGridMarker)).id();

    nav_grid.entity = new_grid_entity;
    nav_grid.world_min = new_world_min;

    commands.entity(old_grid_entity).despawn();

    for (entity, transform, mut agent_pos) in &mut agents {
        commands.entity(entity).insert(AgentOfGrid(new_grid_entity));
        if let Some(new_cell) =
            world_to_grid_cell(transform.translation, new_world_min, nav_grid.dimensions)
        {
            agent_pos.0 = new_cell;
        }

        commands
            .entity(entity)
            .remove::<(Pathfind, Path, NextPos, PathfindingFailed)>();
    }
}

/// Converts world position to local nav-grid cell coordinates.
pub(crate) fn world_to_grid_cell(
    position: Vec3,
    world_min: IVec3,
    dimensions: UVec3,
) -> Option<UVec3> {
    let world_voxel = IVec3::new(
        position.x.floor() as i32,
        position.y.floor() as i32,
        position.z.floor() as i32,
    );
    let local = world_voxel - world_min;

    if local.x < 0
        || local.y < 0
        || local.z < 0
        || local.x as u32 >= dimensions.x
        || local.y as u32 >= dimensions.y
        || local.z as u32 >= dimensions.z
    {
        return None;
    }

    Some(UVec3::new(local.x as u32, local.y as u32, local.z as u32))
}

/// Converts local nav-grid cell coordinates to world-space center position.
pub(crate) fn grid_cell_to_world_center(cell: UVec3, world_min: IVec3) -> Vec3 {
    Vec3::new(
        world_min.x as f32 + cell.x as f32 + ENEMY_CELL_CENTER_OFFSET,
        world_min.y as f32 + cell.y as f32 + ENEMY_CELL_CENTER_OFFSET,
        world_min.z as f32 + cell.z as f32 + ENEMY_CELL_CENTER_OFFSET,
    )
}

/// Default nav grid minimum corner.
fn default_world_min() -> [i32; 3] {
    [-48, FLOOR_MIN_Y + 1, -48]
}

/// Default nav grid dimensions, derived from cave floor/ceiling bounds.
fn default_nav_dimensions() -> [u32; 3] {
    let nav_height = (CEILING_MAX_Y - FLOOR_MIN_Y - 1).max(1) as u32;
    [96, nav_height, 96]
}

/// Default Northstar chunk width/height.
fn default_chunk_size() -> u32 {
    8
}

/// Default Northstar chunk depth.
fn default_chunk_depth() -> u32 {
    8
}

/// Default margin from the nav-grid edge that triggers a recenter rebuild.
fn default_recenter_margin() -> u32 {
    ENEMY_NAV_RECENTER_MARGIN_CELLS
}

/// Builds a passable 3D Northstar grid from cave noise.
fn build_navigation_grid(
    world_min: IVec3,
    dimensions: UVec3,
    chunk_size: u32,
    chunk_depth: u32,
) -> OrdinalGrid3d {
    let settings = GridSettingsBuilder::new_3d(dimensions.x, dimensions.y, dimensions.z)
        .chunk_size(chunk_size)
        .chunk_depth(chunk_depth)
        .enable_diagonal_connections()
        .default_impassable()
        .build();
    let mut grid = OrdinalGrid3d::new(&settings);
    let cave_noise = CaveNoise::new();

    for x in 0..dimensions.x {
        for z in 0..dimensions.z {
            let world_x = world_min.x + x as i32;
            let world_z = world_min.z + z as i32;
            let (floor_y, ceiling_y) = cave_noise.sample_column(world_x, world_z);

            let local_min = floor_y + 1 - world_min.y;
            let local_max = ceiling_y - 1 - world_min.y;
            let start_y = local_min.max(0);
            let end_y = local_max.min(dimensions.y as i32 - 1);

            if end_y < start_y {
                continue;
            }

            for y in start_y as u32..=end_y as u32 {
                grid.set_nav(UVec3::new(x, y, z), Nav::Passable(1));
            }
        }
    }

    grid.build();
    grid
}

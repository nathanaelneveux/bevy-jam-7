//! Mob navigation grid lifecycle.
//!
//! This module owns Northstar grid setup for all navigable agents:
//! - config defaults + config asset loading
//! - grid build/spawn + hot-reload
//! - world/grid coordinate conversion
//! - dynamic recentering for infinite worlds

use bevy::tasks::{AsyncComputeTaskPool, Task, futures_lite::future};
use bevy::{asset::AssetEvent, prelude::*};
use bevy_common_assets::ron::RonAssetPlugin;
use bevy_northstar::prelude::*;
use serde::Deserialize;

use crate::{
    cave_noise::{CEILING_MAX_Y, CaveNoise, FLOOR_MIN_Y},
    player_controller::Player,
};

/// Asset path for shared mob navigation settings.
const MOB_NAV_CONFIG_PATH: &str = "navigation/descent.nav.ron";
/// Offset from cell min corner to center in world space.
const MOB_CELL_CENTER_OFFSET: f32 = 0.5;
/// Fallback recenter margin in cells when not configured.
const MOB_NAV_RECENTER_MARGIN_CELLS: u32 = 16;

pub struct MobNavigationPlugin;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum MobNavigationUpdateSet {
    ConfigReload,
    RecenterRequest,
    RecenterApply,
}

impl Plugin for MobNavigationPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            RonAssetPlugin::<MobNavigationConfig>::new(&["nav.ron"]),
            NorthstarPlugin::<OrdinalNeighborhood3d>::default(),
        ))
        .configure_sets(
            Update,
            (
                MobNavigationUpdateSet::ConfigReload,
                MobNavigationUpdateSet::RecenterRequest.after(MobNavigationUpdateSet::ConfigReload),
                MobNavigationUpdateSet::RecenterApply
                    .after(MobNavigationUpdateSet::RecenterRequest),
            ),
        )
        .init_resource::<MobNavigationConfigState>()
        .init_resource::<MobNavigationRecenterState>()
        .add_systems(Startup, load_mob_navigation_config)
        .add_systems(
            Update,
            (
                initialize_or_reload_mob_navigation_from_config,
                ApplyDeferred,
            )
                .chain()
                .in_set(MobNavigationUpdateSet::ConfigReload),
        )
        .add_systems(
            Update,
            request_navigation_grid_recenter.in_set(MobNavigationUpdateSet::RecenterRequest),
        )
        .add_systems(
            Update,
            apply_ready_navigation_grid_recenter.in_set(MobNavigationUpdateSet::RecenterApply),
        );
    }
}

/// Shared nav settings used to generate and maintain a Northstar 3D grid.
#[derive(Asset, TypePath, Deserialize, Clone, Debug)]
pub(crate) struct MobNavigationConfig {
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

impl Default for MobNavigationConfig {
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

/// Tracks load/reload lifecycle for the external navigation config asset.
#[derive(Resource, Default)]
struct MobNavigationConfigState {
    handle: Handle<MobNavigationConfig>,
    initialized: bool,
    needs_reload: bool,
}

/// Active pathfinding grid resource.
#[derive(Resource, Clone, Copy)]
pub(crate) struct MobNavigationGrid {
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
pub(crate) struct MobNavigationGridMarker;

/// Marker for entities managed by shared navigation systems.
#[derive(Component)]
pub(crate) struct MobNavigationAgent;

/// Pending async navigation-grid rebuild state.
#[derive(Resource, Default)]
pub(crate) struct MobNavigationRecenterState {
    task: Option<Task<PendingNavigationGrid>>,
}

struct PendingNavigationGrid {
    world_min: IVec3,
    grid: OrdinalGrid3d,
}

/// Requests loading the shared navigation config asset.
fn load_mob_navigation_config(
    mut state: ResMut<MobNavigationConfigState>,
    asset_server: Res<AssetServer>,
) {
    state.handle = asset_server.load(MOB_NAV_CONFIG_PATH);
}

/// Handles initial nav config load and hot reloads.
#[allow(clippy::too_many_arguments)]
fn initialize_or_reload_mob_navigation_from_config(
    mut commands: Commands,
    mut state: ResMut<MobNavigationConfigState>,
    configs: Res<Assets<MobNavigationConfig>>,
    mut config_events: MessageReader<AssetEvent<MobNavigationConfig>>,
    existing_grids: Query<Entity, With<MobNavigationGridMarker>>,
    mut agents: Query<(Entity, &Transform, Option<&mut AgentPos>), With<MobNavigationAgent>>,
    mut recenter_state: ResMut<MobNavigationRecenterState>,
) {
    let watched_id = state.handle.id();
    if state.initialized {
        for event in config_events.read() {
            if event.is_modified(watched_id) {
                state.needs_reload = true;
            }
        }
    }

    let Some(config) = configs.get(&state.handle) else {
        return;
    };

    if state.initialized && !state.needs_reload {
        return;
    }

    clear_pending_mob_navigation_recenter(&mut recenter_state);

    for entity in &existing_grids {
        commands.entity(entity).try_despawn();
    }

    let nav_grid = create_navigation_grid(&mut commands, config);

    // Rebind any existing navigation agents to the current grid so all mob types
    // can share the same nav lifecycle.
    for (entity, transform, maybe_agent_pos) in &mut agents {
        let Some(new_cell) = world_to_grid_cell(
            transform.translation,
            nav_grid.world_min,
            nav_grid.dimensions,
        ) else {
            commands
                .entity(entity)
                .try_remove::<(AgentOfGrid, Pathfind, Path, NextPos, PathfindingFailed)>();
            continue;
        };

        if let Some(mut agent_pos) = maybe_agent_pos {
            agent_pos.0 = new_cell;
        } else {
            commands.entity(entity).try_insert(AgentPos(new_cell));
        }

        commands
            .entity(entity)
            .try_insert(AgentOfGrid(nav_grid.entity))
            .try_remove::<(Pathfind, Path, NextPos, PathfindingFailed)>();
    }

    commands.insert_resource(nav_grid);
    state.initialized = true;
    state.needs_reload = false;
    info!("Mob navigation initialized from {MOB_NAV_CONFIG_PATH}");
}

/// Cancels any in-flight async navigation rebuild.
fn clear_pending_mob_navigation_recenter(recenter_state: &mut MobNavigationRecenterState) {
    recenter_state.task = None;
}

/// Builds and spawns the active navigation grid entity from config.
fn create_navigation_grid(
    commands: &mut Commands,
    config: &MobNavigationConfig,
) -> MobNavigationGrid {
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
    let entity = commands.spawn((grid, MobNavigationGridMarker)).id();

    MobNavigationGrid {
        entity,
        world_min,
        dimensions,
        chunk_size,
        chunk_depth,
        recenter_margin,
    }
}

/// Queues an async navigation rebuild when the player nears the current X/Z bounds.
pub(crate) fn request_navigation_grid_recenter(
    player: Query<&Transform, With<Player>>,
    nav_grid: Option<Res<MobNavigationGrid>>,
    mut recenter_state: ResMut<MobNavigationRecenterState>,
) {
    let Some(nav_grid) = nav_grid else {
        return;
    };
    if recenter_state.task.is_some() {
        return;
    }
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

    let dimensions = nav_grid.dimensions;
    let chunk_size = nav_grid.chunk_size;
    let chunk_depth = nav_grid.chunk_depth;
    let task_pool = AsyncComputeTaskPool::get();
    recenter_state.task = Some(task_pool.spawn(async move {
        let grid = build_navigation_grid(new_world_min, dimensions, chunk_size, chunk_depth);
        PendingNavigationGrid {
            world_min: new_world_min,
            grid,
        }
    }));
}

/// Applies a completed async navigation rebuild to the active grid.
pub(crate) fn apply_ready_navigation_grid_recenter(
    mut nav_grid: Option<ResMut<MobNavigationGrid>>,
    mut nav_grids: Query<&mut OrdinalGrid3d, With<MobNavigationGridMarker>>,
    mut agents: Query<(Entity, &Transform, &mut AgentPos), With<MobNavigationAgent>>,
    mut recenter_state: ResMut<MobNavigationRecenterState>,
    mut commands: Commands,
) {
    let Some(nav_grid) = nav_grid.as_mut() else {
        recenter_state.task = None;
        return;
    };

    let Some(mut task) = recenter_state.task.take() else {
        return;
    };

    let Some(pending) = future::block_on(future::poll_once(&mut task)) else {
        recenter_state.task = Some(task);
        return;
    };

    let Ok(mut grid_component) = nav_grids.get_mut(nav_grid.entity) else {
        return;
    };
    *grid_component = pending.grid;
    nav_grid.world_min = pending.world_min;

    let world_min = nav_grid.world_min;
    let dimensions = nav_grid.dimensions;
    for (entity, transform, mut agent_pos) in &mut agents {
        if let Some(new_cell) = world_to_grid_cell(transform.translation, world_min, dimensions) {
            agent_pos.0 = new_cell;
        }

        commands
            .entity(entity)
            .try_remove::<(Pathfind, Path, NextPos, PathfindingFailed)>();
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
        world_min.x as f32 + cell.x as f32 + MOB_CELL_CENTER_OFFSET,
        world_min.y as f32 + cell.y as f32 + MOB_CELL_CENTER_OFFSET,
        world_min.z as f32 + cell.z as f32 + MOB_CELL_CENTER_OFFSET,
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
    MOB_NAV_RECENTER_MARGIN_CELLS
}

/// Builds a passable 3D Northstar grid from cave noise.
///
/// This is the single source of truth for shared navigation so runtime grid
/// state cannot drift from chunk event ordering.
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_to_grid_cell_maps_interior_position() {
        let world_min = IVec3::new(-8, -4, -8);
        let dimensions = UVec3::new(32, 16, 32);
        let world = Vec3::new(-3.2, 1.8, 7.9);

        let cell = world_to_grid_cell(world, world_min, dimensions);
        assert_eq!(cell, Some(UVec3::new(4, 5, 15)));
    }

    #[test]
    fn world_to_grid_cell_rejects_out_of_bounds_position() {
        let world_min = IVec3::ZERO;
        let dimensions = UVec3::new(4, 4, 4);

        let cell = world_to_grid_cell(Vec3::new(4.0, 1.0, 1.0), world_min, dimensions);
        assert_eq!(cell, None);
    }

    #[test]
    fn grid_cell_to_world_center_returns_centered_position() {
        let world_min = IVec3::new(10, -2, 5);
        let cell = UVec3::new(3, 4, 2);

        let center = grid_cell_to_world_center(cell, world_min);
        assert_eq!(center, Vec3::new(13.5, 2.5, 7.5));
    }
}

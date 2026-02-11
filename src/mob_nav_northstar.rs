use bevy::prelude::*;
use bevy_northstar::prelude::{CardinalIsoGrid, GridSettingsBuilder, Nav, PathfindArgs};
use bevy_voxel_world::prelude::VoxelWorld;

use crate::cave_noise::{CEILING_MAX_Y, FLOOR_MIN_Y};
use crate::cave_world::CaveWorld;
use crate::mob_nav::{
    MobNavMovementMode, MobNavPlanRequest, MobNavPlanResult, MobNavPlanResultKind, MobNavUpdateSet,
};

pub struct MobNavNorthstarPlugin;

impl Plugin for MobNavNorthstarPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MobNavNorthstarConfig::default())
            .init_resource::<MobNavNorthstarRollingGrid>()
            .add_systems(
                Update,
                plan_ground_paths_with_northstar.in_set(MobNavUpdateSet::PlanPaths),
            );
    }
}

#[derive(Resource, Debug, Clone)]
pub struct MobNavNorthstarConfig {
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_depth: u32,
    pub recenter_margin_voxels: i32,
    pub chunk_size: u32,
    pub chunk_depth: u32,
    pub agent_height_voxels: i32,
    pub snap_search_radius_voxels: i32,
    pub min_world_y: i32,
    pub max_world_y: i32,
    pub waypoint_center_offset: Vec3,
    pub mode: MobNavNorthstarPathMode,
}

impl Default for MobNavNorthstarConfig {
    fn default() -> Self {
        Self {
            grid_width: 128,
            grid_height: 48,
            grid_depth: 128,
            recenter_margin_voxels: 20,
            chunk_size: 8,
            chunk_depth: 8,
            agent_height_voxels: 2,
            snap_search_radius_voxels: 24,
            min_world_y: FLOOR_MIN_Y - 4,
            max_world_y: CEILING_MAX_Y + 8,
            waypoint_center_offset: Vec3::splat(0.5),
            mode: MobNavNorthstarPathMode::Refined,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub enum MobNavNorthstarPathMode {
    AStar,
    ThetaStar,
    Refined,
}

#[derive(Resource, Default)]
struct MobNavNorthstarRollingGrid {
    grid: Option<CardinalIsoGrid>,
    min_world: IVec3,
    max_world: IVec3,
    center_world: IVec3,
}

fn plan_ground_paths_with_northstar(
    mut requests: MessageReader<MobNavPlanRequest>,
    mut results: MessageWriter<MobNavPlanResult>,
    voxel_world: VoxelWorld<CaveWorld>,
    config: Res<MobNavNorthstarConfig>,
    mut rolling: ResMut<MobNavNorthstarRollingGrid>,
) {
    for request in requests.read() {
        if request.movement_mode != MobNavMovementMode::Ground {
            continue;
        }

        let start_world = request.from.floor().as_ivec3();
        let goal_world = request.to.floor().as_ivec3();

        ensure_rolling_grid_for_request(
            &mut rolling,
            &voxel_world,
            &config,
            start_world,
            goal_world,
        );

        let (min_world, max_world, Some(grid)) =
            (rolling.min_world, rolling.max_world, rolling.grid.as_ref())
        else {
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Blocked,
            });
            continue;
        };

        let Some(start_local) = find_nearest_passable_local(
            grid,
            start_world,
            min_world,
            max_world,
            config.snap_search_radius_voxels,
        ) else {
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Blocked,
            });
            continue;
        };

        let Some(goal_local) = find_nearest_passable_local(
            grid,
            goal_world,
            min_world,
            max_world,
            config.snap_search_radius_voxels,
        ) else {
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Blocked,
            });
            continue;
        };

        let mut args = match config.mode {
            MobNavNorthstarPathMode::AStar => PathfindArgs::new(start_local, goal_local).astar(),
            MobNavNorthstarPathMode::ThetaStar => {
                PathfindArgs::new(start_local, goal_local).thetastar()
            }
            MobNavNorthstarPathMode::Refined => {
                PathfindArgs::new(start_local, goal_local).refined()
            }
        };

        let result = if let Some(path) = grid.pathfind(&mut args) {
            let mut waypoints = path
                .into_iter()
                .map(|local| local_to_world(local, min_world, config.waypoint_center_offset))
                .collect::<Vec<_>>();
            if waypoints
                .first()
                .is_some_and(|first| first.distance(request.from) <= 0.75)
            {
                waypoints.remove(0);
            }
            MobNavPlanResultKind::Path(waypoints)
        } else {
            MobNavPlanResultKind::Blocked
        };

        results.write(MobNavPlanResult {
            request_id: request.request_id,
            entity: request.entity,
            result,
        });
    }
}

fn ensure_rolling_grid_for_request(
    rolling: &mut MobNavNorthstarRollingGrid,
    voxel_world: &VoxelWorld<CaveWorld>,
    config: &MobNavNorthstarConfig,
    start_world: IVec3,
    goal_world: IVec3,
) {
    let request_center = midpoint(start_world, goal_world);
    let request_center = IVec3::new(
        request_center.x,
        request_center
            .y
            .clamp(config.min_world_y, config.max_world_y),
        request_center.z,
    );

    let needs_rebuild = rolling.grid.is_none()
        || !is_inside_bounds_with_margin(
            start_world,
            rolling.min_world,
            rolling.max_world,
            config.recenter_margin_voxels,
        )
        || !is_inside_bounds_with_margin(
            goal_world,
            rolling.min_world,
            rolling.max_world,
            config.recenter_margin_voxels,
        );

    if !needs_rebuild {
        return;
    }

    let dimensions = effective_dimensions(config);
    let (min_world, max_world, center_world) = rolling_bounds(request_center, dimensions, config);

    let settings = GridSettingsBuilder::new_3d(
        dimensions.x as u32,
        dimensions.y as u32,
        dimensions.z as u32,
    )
    .chunk_size(config.chunk_size.max(1))
    .chunk_depth(config.chunk_depth.max(1))
    .default_impassable()
    .build();
    let mut grid = CardinalIsoGrid::new(&settings);

    let agent_height_voxels = config.agent_height_voxels.max(1);
    for z in 0..dimensions.z {
        for y in 0..dimensions.y {
            for x in 0..dimensions.x {
                let world = min_world + IVec3::new(x, y, z);
                if is_walkable_cell(voxel_world, world, agent_height_voxels) {
                    grid.set_nav(UVec3::new(x as u32, y as u32, z as u32), Nav::Passable(1));
                }
            }
        }
    }
    grid.build();

    rolling.grid = Some(grid);
    rolling.min_world = min_world;
    rolling.max_world = max_world;
    rolling.center_world = center_world;
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
) -> (IVec3, IVec3, IVec3) {
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

    let snapped_center = midpoint(min_world, max_world);
    (min_world, max_world, snapped_center)
}

fn midpoint(a: IVec3, b: IVec3) -> IVec3 {
    IVec3::new((a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2)
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

fn is_walkable_cell(
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

fn find_nearest_passable_local(
    grid: &CardinalIsoGrid,
    seed_world: IVec3,
    min_world: IVec3,
    max_world: IVec3,
    max_radius: i32,
) -> Option<UVec3> {
    let seed_local = world_to_local(seed_world, min_world, max_world)?;
    if grid.is_passable(seed_local) {
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
                    if !grid.is_passable(local) {
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

fn world_to_local(world: IVec3, min_world: IVec3, max_world: IVec3) -> Option<UVec3> {
    if world.cmplt(min_world).any() || world.cmpgt(max_world).any() {
        return None;
    }
    let local = world - min_world;
    Some(UVec3::new(local.x as u32, local.y as u32, local.z as u32))
}

fn local_to_world(local: UVec3, min_world: IVec3, center_offset: Vec3) -> Vec3 {
    let world = min_world + local.as_ivec3();
    world.as_vec3() + center_offset
}

use bevy::prelude::*;
use bevy_northstar::prelude::{
    AgentOfGrid, AgentPos, AvoidanceFailed, Blocking, NextPos, Path as NorthstarPath, Pathfind,
    PathfindingFailed, RerouteFailed,
};
use bevy_voxel_world::prelude::VoxelWorld;

use crate::cave_world::CaveWorld;
use crate::mob_nav::{
    MobNavMovementMode, MobNavPlanRequest, MobNavPlanResult, MobNavPlanResultKind,
};
use crate::player_controller::Player;

use super::MobNavNorthstarConfig;
use super::grid::{
    MobNavNorthstarRollingGrid, ensure_rolling_grid_for_player, find_nearest_walkable_local,
    local_to_world,
};

#[derive(Component, Debug, Clone, Copy)]
pub(crate) struct MobNavNorthstarPendingRequest {
    request_id: u64,
    min_world: IVec3,
    request_from: Vec3,
    revision: u64,
}

pub(crate) fn plan_ground_paths_with_northstar(
    mut commands: Commands,
    mut requests: MessageReader<MobNavPlanRequest>,
    mut results: MessageWriter<MobNavPlanResult>,
    voxel_world: VoxelWorld<CaveWorld>,
    config: Res<MobNavNorthstarConfig>,
    mut rolling: ResMut<MobNavNorthstarRollingGrid>,
    player: Single<&GlobalTransform, With<Player>>,
) {
    let ground_requests = requests
        .read()
        .filter(|request| request.movement_mode == MobNavMovementMode::Ground)
        .copied()
        .collect::<Vec<_>>();
    if ground_requests.is_empty() {
        return;
    }

    let player_world = player.translation().floor().as_ivec3();
    ensure_rolling_grid_for_player(
        &mut commands,
        &mut rolling,
        &voxel_world,
        &config,
        player_world,
    );

    let Some(grid_entity) = rolling.grid_entity else {
        for request in ground_requests {
            write_blocked_result(
                &mut commands,
                &mut results,
                request.entity,
                request.request_id,
            );
        }
        return;
    };

    for request in ground_requests {
        let start_world = request.from.floor().as_ivec3();
        let goal_world = request.to.floor().as_ivec3();

        let Some(start_local) = find_nearest_walkable_local(
            &voxel_world,
            start_world,
            rolling.min_world,
            rolling.max_world,
            config.snap_search_radius_voxels,
            config.agent_height_voxels.max(1),
        ) else {
            write_blocked_result(
                &mut commands,
                &mut results,
                request.entity,
                request.request_id,
            );
            continue;
        };

        let Some(goal_local) = find_nearest_walkable_local(
            &voxel_world,
            goal_world,
            rolling.min_world,
            rolling.max_world,
            config.snap_search_radius_voxels,
            config.agent_height_voxels.max(1),
        ) else {
            write_blocked_result(
                &mut commands,
                &mut results,
                request.entity,
                request.request_id,
            );
            continue;
        };

        if start_local == goal_local {
            clear_northstar_request_state(&mut commands, request.entity);
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Path(Vec::new()),
            });
            continue;
        }

        let mut pathfind = Pathfind::new(goal_local).mode(config.pathfind_mode);
        if config.allow_partial_paths && supports_partial_paths(config.pathfind_mode) {
            pathfind = pathfind.partial();
        }

        commands.entity(request.entity).insert((
            AgentOfGrid(grid_entity),
            AgentPos(start_local),
            Blocking,
            pathfind,
            MobNavNorthstarPendingRequest {
                request_id: request.request_id,
                min_world: rolling.min_world,
                request_from: request.from,
                revision: rolling.revision,
            },
        ));
        commands.entity(request.entity).remove::<(
            NorthstarPath,
            NextPos,
            PathfindingFailed,
            AvoidanceFailed,
            RerouteFailed,
        )>();
    }
}

pub(crate) fn collect_northstar_plan_results(
    mut commands: Commands,
    mut results: MessageWriter<MobNavPlanResult>,
    config: Res<MobNavNorthstarConfig>,
    rolling: Res<MobNavNorthstarRollingGrid>,
    query: Query<(
        Entity,
        &MobNavNorthstarPendingRequest,
        Option<&NorthstarPath>,
        Has<PathfindingFailed>,
        Has<RerouteFailed>,
    )>,
) {
    for (entity, pending, path, pathfinding_failed, reroute_failed) in &query {
        if pending.revision != rolling.revision {
            results.write(MobNavPlanResult {
                request_id: pending.request_id,
                entity,
                result: MobNavPlanResultKind::Blocked,
            });
            clear_northstar_request_state(&mut commands, entity);
            continue;
        }

        if let Some(path) = path {
            let mut waypoints = path
                .path()
                .iter()
                .map(|&local| {
                    local_to_world(local, pending.min_world, config.waypoint_center_offset)
                })
                .collect::<Vec<_>>();
            if waypoints
                .first()
                .is_some_and(|first| first.distance(pending.request_from) <= 0.75)
            {
                waypoints.remove(0);
            }

            results.write(MobNavPlanResult {
                request_id: pending.request_id,
                entity,
                result: MobNavPlanResultKind::Path(waypoints),
            });
            clear_northstar_request_state(&mut commands, entity);
            continue;
        }

        if pathfinding_failed || reroute_failed {
            results.write(MobNavPlanResult {
                request_id: pending.request_id,
                entity,
                result: MobNavPlanResultKind::Blocked,
            });
            clear_northstar_request_state(&mut commands, entity);
        }
    }
}

fn clear_northstar_request_state(commands: &mut Commands, entity: Entity) {
    commands.entity(entity).remove::<(
        MobNavNorthstarPendingRequest,
        Pathfind,
        NorthstarPath,
        NextPos,
        PathfindingFailed,
        AvoidanceFailed,
        RerouteFailed,
    )>();
}

fn write_blocked_result(
    commands: &mut Commands,
    results: &mut MessageWriter<MobNavPlanResult>,
    entity: Entity,
    request_id: u64,
) {
    clear_northstar_request_state(commands, entity);
    results.write(MobNavPlanResult {
        request_id,
        entity,
        result: MobNavPlanResultKind::Blocked,
    });
}

fn supports_partial_paths(mode: bevy_northstar::prelude::PathfindMode) -> bool {
    matches!(
        mode,
        bevy_northstar::prelude::PathfindMode::AStar
            | bevy_northstar::prelude::PathfindMode::ThetaStar
    )
}

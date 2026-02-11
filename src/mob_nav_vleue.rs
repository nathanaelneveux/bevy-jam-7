use avian3d::prelude::Collider;
use bevy::math::vec2;
use bevy::prelude::*;
use bevy_voxel_world::custom_meshing::CHUNK_SIZE_F;
use bevy_voxel_world::prelude::{Chunk, ChunkWillDespawn, ChunkWillSpawn, NeedsDespawn};
use vleue_navigator::prelude::{
    ManagedNavMesh, NavMesh, NavMeshSettings, NavMeshStatus, NavMeshUpdateMode,
    NavmeshUpdaterPlugin, Triangulation, VleueNavigatorPlugin,
};

use crate::cave_world::CaveWorld;
use crate::mob_nav::{
    MobNavAgent, MobNavGoal, MobNavMovementMode, MobNavPlanRequest, MobNavPlanResult,
    MobNavPlanResultKind, MobNavRepath, MobNavUpdateSet,
};

pub struct MobNavVleuePlugin;

impl Plugin for MobNavVleuePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(MobNavVleueConfig::default())
            .add_plugins((
                VleueNavigatorPlugin,
                NavmeshUpdaterPlugin::<Collider, MobNavObstacle>::default(),
            ))
            .add_systems(Startup, spawn_ground_navmesh)
            .add_systems(
                Update,
                (
                    tag_chunk_colliders_as_obstacles,
                    untag_chunk_entities_without_colliders,
                    sync_navmesh_bounds,
                    queue_repath_on_navmesh_ready,
                ),
            )
            .add_systems(
                Update,
                plan_ground_paths_with_vleue.in_set(MobNavUpdateSet::PlanPaths),
            );
    }
}

#[derive(Component, Debug, Clone, Copy, Default)]
pub struct MobNavObstacle;

#[derive(Resource, Debug, Clone)]
pub struct MobNavVleueConfig {
    pub fallback_half_extent: f32,
    pub bounds_padding_chunks: i32,
    pub navmesh_height: f32,
    pub navmesh_upward_shift: f32,
    pub navmesh_agent_radius: f32,
    pub rebuild_debounce_seconds: f32,
}

impl Default for MobNavVleueConfig {
    fn default() -> Self {
        Self {
            fallback_half_extent: 256.0,
            bounds_padding_chunks: 1,
            navmesh_height: 1.25,
            navmesh_upward_shift: 0.2,
            navmesh_agent_radius: 0.45,
            rebuild_debounce_seconds: 0.15,
        }
    }
}

#[derive(Component)]
struct MobNavGroundNavmesh;

fn spawn_ground_navmesh(mut commands: Commands, config: Res<MobNavVleueConfig>) {
    commands.spawn((
        MobNavGroundNavmesh,
        NavMeshSettings {
            fixed: square_triangulation(config.fallback_half_extent),
            upward_shift: config.navmesh_upward_shift,
            agent_radius: config.navmesh_agent_radius,
            ..default()
        },
        NavMeshUpdateMode::Debounced(config.rebuild_debounce_seconds),
        Transform::from_xyz(0.0, config.navmesh_height, 0.0)
            .with_rotation(Quat::from_rotation_x(core::f32::consts::FRAC_PI_2)),
    ));
}

fn tag_chunk_colliders_as_obstacles(
    mut commands: Commands,
    chunks: Query<
        (Entity, &Chunk<CaveWorld>),
        (
            With<Collider>,
            Without<MobNavObstacle>,
            Without<NeedsDespawn>,
        ),
    >,
) {
    for (entity, chunk) in &chunks {
        if chunk.lod_level == 0 {
            commands.entity(entity).insert(MobNavObstacle);
        }
    }
}

fn untag_chunk_entities_without_colliders(
    mut commands: Commands,
    missing_collider: Query<Entity, (With<MobNavObstacle>, Without<Collider>)>,
    despawning: Query<Entity, (With<MobNavObstacle>, With<NeedsDespawn>)>,
    invalid_lod: Query<(Entity, &Chunk<CaveWorld>), (With<MobNavObstacle>, With<Collider>)>,
) {
    for entity in &missing_collider {
        commands.entity(entity).remove::<MobNavObstacle>();
    }
    for entity in &despawning {
        commands.entity(entity).remove::<MobNavObstacle>();
    }
    for (entity, chunk) in &invalid_lod {
        if chunk.lod_level != 0 {
            commands.entity(entity).remove::<MobNavObstacle>();
        }
    }
}

fn sync_navmesh_bounds(
    mut chunk_spawned: MessageReader<ChunkWillSpawn<CaveWorld>>,
    mut chunk_despawned: MessageReader<ChunkWillDespawn<CaveWorld>>,
    chunks: Query<&Chunk<CaveWorld>, Without<NeedsDespawn>>,
    mut navmesh: Single<&mut NavMeshSettings, With<MobNavGroundNavmesh>>,
    config: Res<MobNavVleueConfig>,
) {
    if chunk_spawned.is_empty() && chunk_despawned.is_empty() {
        return;
    }

    let _ = chunk_spawned.read().count();
    let _ = chunk_despawned.read().count();

    navmesh.fixed = if let Some((min_x, max_x, min_z, max_z)) =
        compute_loaded_xz_bounds(&chunks, config.bounds_padding_chunks)
    {
        Triangulation::from_outer_edges(&[
            vec2(min_x, min_z),
            vec2(max_x, min_z),
            vec2(max_x, max_z),
            vec2(min_x, max_z),
        ])
    } else {
        square_triangulation(config.fallback_half_extent)
    };
    navmesh.upward_shift = config.navmesh_upward_shift;
    navmesh.agent_radius = config.navmesh_agent_radius;
}

fn compute_loaded_xz_bounds(
    chunks: &Query<&Chunk<CaveWorld>, Without<NeedsDespawn>>,
    padding_chunks: i32,
) -> Option<(f32, f32, f32, f32)> {
    let mut min_chunk_x = i32::MAX;
    let mut max_chunk_x = i32::MIN;
    let mut min_chunk_z = i32::MAX;
    let mut max_chunk_z = i32::MIN;
    let mut has_chunks = false;

    for chunk in chunks.iter() {
        if chunk.lod_level != 0 {
            continue;
        }
        has_chunks = true;
        min_chunk_x = min_chunk_x.min(chunk.position.x);
        max_chunk_x = max_chunk_x.max(chunk.position.x);
        min_chunk_z = min_chunk_z.min(chunk.position.z);
        max_chunk_z = max_chunk_z.max(chunk.position.z);
    }

    if !has_chunks {
        return None;
    }

    let min_x = (min_chunk_x - padding_chunks) as f32 * CHUNK_SIZE_F;
    let max_x = (max_chunk_x + padding_chunks + 1) as f32 * CHUNK_SIZE_F;
    let min_z = (min_chunk_z - padding_chunks) as f32 * CHUNK_SIZE_F;
    let max_z = (max_chunk_z + padding_chunks + 1) as f32 * CHUNK_SIZE_F;

    Some((min_x, max_x, min_z, max_z))
}

fn square_triangulation(half_extent: f32) -> Triangulation {
    Triangulation::from_outer_edges(&[
        vec2(-half_extent, -half_extent),
        vec2(half_extent, -half_extent),
        vec2(half_extent, half_extent),
        vec2(-half_extent, half_extent),
    ])
}

fn plan_ground_paths_with_vleue(
    mut requests: MessageReader<MobNavPlanRequest>,
    mut results: MessageWriter<MobNavPlanResult>,
    navmesh_assets: Res<Assets<NavMesh>>,
    navmesh_state: Single<&NavMeshStatus, With<MobNavGroundNavmesh>>,
) {
    if **navmesh_state != NavMeshStatus::Built {
        return;
    }

    let Some(navmesh) = navmesh_assets.get(&ManagedNavMesh::get_single()) else {
        return;
    };

    for request in requests.read() {
        if request.movement_mode != MobNavMovementMode::Ground {
            continue;
        }

        let result = if let Some(path) = navmesh.transformed_path(request.from, request.to) {
            let mut waypoints = path.path;
            if waypoints
                .first()
                .is_some_and(|first| first.distance(request.from) <= 0.25)
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

fn queue_repath_on_navmesh_ready(
    mut commands: Commands,
    navmesh_state: Single<Ref<NavMeshStatus>, With<MobNavGroundNavmesh>>,
    ground_agents: Query<(Entity, &MobNavAgent), With<MobNavGoal>>,
) {
    if !navmesh_state.is_changed() || **navmesh_state != NavMeshStatus::Built {
        return;
    }

    for (entity, agent) in &ground_agents {
        if agent.movement_mode != MobNavMovementMode::Ground {
            continue;
        }
        commands.entity(entity).insert(MobNavRepath);
    }
}

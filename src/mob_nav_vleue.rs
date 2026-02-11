use bevy::math::vec2;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy_voxel_world::custom_meshing::{CHUNK_SIZE_F, CHUNK_SIZE_I};
use bevy_voxel_world::prelude::{
    Chunk, ChunkWillChangeLod, ChunkWillDespawn, ChunkWillSpawn, NeedsDespawn, VoxelWorld,
    WorldVoxel,
};
use vleue_navigator::prelude::{
    ManagedNavMesh, NavMesh, NavMeshSettings, NavMeshStatus, NavMeshUpdateMode,
    NavmeshUpdaterPlugin, PrimitiveObstacle, Triangulation, VleueNavigatorPlugin,
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
            .init_resource::<ChunkNavObstacleMap>()
            .add_plugins((
                VleueNavigatorPlugin,
                NavmeshUpdaterPlugin::<PrimitiveObstacle, MobNavObstacle>::default(),
            ))
            .add_systems(Startup, spawn_ground_navmesh)
            .add_systems(
                Update,
                (
                    rebuild_chunk_obstacles,
                    clear_despawned_chunk_obstacles,
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

#[derive(Resource, Default)]
struct ChunkNavObstacleMap {
    by_chunk: HashMap<Entity, Vec<Entity>>,
}

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

fn rebuild_chunk_obstacles(
    mut commands: Commands,
    mut obstacle_map: ResMut<ChunkNavObstacleMap>,
    voxel_world: VoxelWorld<CaveWorld>,
    config: Res<MobNavVleueConfig>,
    chunks: Query<
        (Entity, &Chunk<CaveWorld>),
        (With<Mesh3d>, Changed<Mesh3d>, Without<NeedsDespawn>),
    >,
    mut lod_changes: MessageReader<ChunkWillChangeLod<CaveWorld>>,
) {
    let mut dirty_chunks = std::collections::HashSet::new();
    for (entity, _) in &chunks {
        dirty_chunks.insert(entity);
    }
    for event in lod_changes.read() {
        dirty_chunks.insert(event.entity);
    }

    for chunk_entity in dirty_chunks {
        if let Some(existing) = obstacle_map.by_chunk.remove(&chunk_entity) {
            despawn_entities(&mut commands, existing);
        }

        let Ok((_entity, chunk)) = chunks.get(chunk_entity) else {
            continue;
        };
        if chunk.lod_level != 0 {
            continue;
        }

        let Some(chunk_data) = voxel_world.get_chunk_data(chunk.position) else {
            continue;
        };
        if chunk_data.is_empty() {
            continue;
        }

        let y = config.navmesh_height.floor() as i32;
        let blocked = collect_blocked_cells_at_height(chunk.position, y, |world| {
            chunk_data.get_voxel_at_world_position(world)
        });
        let rectangles = merge_blocked_cells_to_rectangles(&blocked);

        if rectangles.is_empty() {
            continue;
        }

        let chunk_world_min_x = chunk.position.x * CHUNK_SIZE_I;
        let chunk_world_min_z = chunk.position.z * CHUNK_SIZE_I;

        let mut spawned = Vec::with_capacity(rectangles.len());
        for rect in rectangles {
            let width = (rect.x_end - rect.x_start + 1) as f32;
            let depth = (rect.z_end - rect.z_start + 1) as f32;
            let center_x = chunk_world_min_x as f32 + rect.x_start as f32 + width * 0.5;
            let center_z = chunk_world_min_z as f32 + rect.z_start as f32 + depth * 0.5;

            let entity = commands
                .spawn((
                    MobNavObstacle,
                    PrimitiveObstacle::Rectangle(Rectangle {
                        half_size: vec2(width * 0.5, depth * 0.5),
                    }),
                    Transform::from_xyz(center_x, config.navmesh_height, center_z),
                ))
                .id();
            spawned.push(entity);
        }

        obstacle_map.by_chunk.insert(chunk_entity, spawned);
    }
}

fn clear_despawned_chunk_obstacles(
    mut commands: Commands,
    mut obstacle_map: ResMut<ChunkNavObstacleMap>,
    mut chunk_despawned: MessageReader<ChunkWillDespawn<CaveWorld>>,
) {
    for event in chunk_despawned.read() {
        if let Some(existing) = obstacle_map.by_chunk.remove(&event.entity) {
            despawn_entities(&mut commands, existing);
        }
    }
}

fn despawn_entities(commands: &mut Commands, entities: Vec<Entity>) {
    for entity in entities {
        if let Ok(mut commands) = commands.get_entity(entity) {
            commands.despawn();
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

fn collect_blocked_cells_at_height<I: Copy + PartialEq>(
    chunk_pos: IVec3,
    y: i32,
    get_voxel: impl Fn(IVec3) -> Option<WorldVoxel<I>>,
) -> Vec<bool> {
    let mut blocked = vec![false; (CHUNK_SIZE_I * CHUNK_SIZE_I) as usize];
    let chunk_world_min = chunk_pos * CHUNK_SIZE_I;
    let chunk_world_max = chunk_world_min + IVec3::splat(CHUNK_SIZE_I - 1);

    if y < chunk_world_min.y || y > chunk_world_max.y {
        return blocked;
    }

    for z in 0..CHUNK_SIZE_I {
        for x in 0..CHUNK_SIZE_I {
            let world = IVec3::new(chunk_world_min.x + x, y, chunk_world_min.z + z);
            let is_solid =
                get_voxel(world).is_some_and(|voxel| matches!(voxel, WorldVoxel::Solid(_)));
            blocked[(z * CHUNK_SIZE_I + x) as usize] = is_solid;
        }
    }

    blocked
}

#[derive(Clone, Copy)]
struct GridRect {
    x_start: i32,
    x_end: i32,
    z_start: i32,
    z_end: i32,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
struct Span {
    x_start: i32,
    x_end: i32,
}

fn merge_blocked_cells_to_rectangles(blocked: &[bool]) -> Vec<GridRect> {
    #[derive(Clone, Copy)]
    struct ActiveRect {
        x_start: i32,
        x_end: i32,
        z_start: i32,
        z_end: i32,
    }

    let mut finished = Vec::new();
    let mut active: HashMap<Span, ActiveRect> = HashMap::default();

    for z in 0..CHUNK_SIZE_I {
        let spans = row_spans(blocked, z);
        let current_spans: std::collections::HashSet<_> = spans.iter().copied().collect();

        let mut next_active: HashMap<Span, ActiveRect> = HashMap::default();

        for span in spans {
            if let Some(mut rect) = active.remove(&span) {
                rect.z_end = z;
                next_active.insert(span, rect);
            } else {
                next_active.insert(
                    span,
                    ActiveRect {
                        x_start: span.x_start,
                        x_end: span.x_end,
                        z_start: z,
                        z_end: z,
                    },
                );
            }
        }

        for (span, rect) in active.drain() {
            if !current_spans.contains(&span) {
                finished.push(rect);
            }
        }

        active = next_active;
    }

    finished.extend(active.into_values());

    finished
        .into_iter()
        .map(|r| GridRect {
            x_start: r.x_start,
            x_end: r.x_end,
            z_start: r.z_start,
            z_end: r.z_end,
        })
        .collect()
}

fn row_spans(blocked: &[bool], z: i32) -> Vec<Span> {
    let mut spans = Vec::new();
    let mut x = 0;

    while x < CHUNK_SIZE_I {
        let idx = (z * CHUNK_SIZE_I + x) as usize;
        if !blocked[idx] {
            x += 1;
            continue;
        }

        let start = x;
        while x < CHUNK_SIZE_I && blocked[(z * CHUNK_SIZE_I + x) as usize] {
            x += 1;
        }
        spans.push(Span {
            x_start: start,
            x_end: x - 1,
        });
    }

    spans
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

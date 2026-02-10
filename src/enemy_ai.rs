use std::collections::HashMap;

use bevy::prelude::*;
use bevy_common_assets::ron::RonAssetPlugin;
use bevy_northstar::prelude::*;
use big_brain::prelude::*;
use serde::Deserialize;

use crate::{
    cave_noise::{CEILING_MAX_Y, CaveNoise, FLOOR_MIN_Y},
    player_controller::Player,
};

const ENEMY_CONFIG_PATH: &str = "enemies/descent.enemy.ron";
const ENEMY_RENDER_RADIUS: f32 = 0.35;
const ENEMY_CELL_CENTER_OFFSET: f32 = 0.5;
const ENEMY_POSITION_TOLERANCE: f32 = 0.05;

pub struct EnemyAiPlugin;

impl Plugin for EnemyAiPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((
            RonAssetPlugin::<EnemyAiConfig>::new(&["enemy.ron"]),
            BigBrainPlugin::new(PreUpdate),
            NorthstarPlugin::<OrdinalNeighborhood3d>::default(),
        ))
        .init_resource::<EnemyConfigState>()
        .init_resource::<EnemyArchetypeLibrary>()
        .add_systems(Startup, load_enemy_ai_config)
        .add_systems(
            PreUpdate,
            (
                target_in_range_scorer_system.in_set(BigBrainSet::Scorers),
                chase_target_action_system.in_set(BigBrainSet::Actions),
                hold_position_action_system.in_set(BigBrainSet::Actions),
            ),
        )
        .add_systems(
            Update,
            (
                initialize_enemy_ai_from_config,
                move_enemy_agents.after(PathingSet),
                clear_pathfinding_failures,
            ),
        );
    }
}

#[derive(Resource, Default)]
struct EnemyConfigState {
    handle: Handle<EnemyAiConfig>,
    initialized: bool,
}

#[derive(Resource, Default)]
struct EnemyArchetypeLibrary(HashMap<String, EnemyArchetypeConfig>);

#[derive(Resource, Clone, Copy)]
struct EnemyNavigationGrid {
    entity: Entity,
    world_min: IVec3,
    dimensions: UVec3,
}

#[derive(Asset, TypePath, Deserialize, Clone, Debug)]
struct EnemyAiConfig {
    #[serde(default)]
    nav: EnemyNavigationConfig,
    #[serde(default)]
    archetypes: Vec<EnemyArchetypeConfig>,
    #[serde(default)]
    spawns: Vec<EnemySpawnConfig>,
}

#[derive(Deserialize, Clone, Debug)]
struct EnemyNavigationConfig {
    #[serde(default = "default_world_min")]
    world_min: [i32; 3],
    #[serde(default = "default_nav_dimensions")]
    dimensions: [u32; 3],
    #[serde(default = "default_chunk_size")]
    chunk_size: u32,
    #[serde(default = "default_chunk_depth")]
    chunk_depth: u32,
}

impl Default for EnemyNavigationConfig {
    fn default() -> Self {
        Self {
            world_min: default_world_min(),
            dimensions: default_nav_dimensions(),
            chunk_size: default_chunk_size(),
            chunk_depth: default_chunk_depth(),
        }
    }
}

#[derive(Deserialize, Clone, Debug)]
struct EnemyArchetypeConfig {
    id: String,
    move_speed: f32,
    chase_range: f32,
    stopping_distance: f32,
    repath_interval: f32,
    #[serde(default)]
    path_mode: EnemyPathMode,
    #[serde(default = "default_enemy_color")]
    color: [f32; 3],
}

#[derive(Deserialize, Clone, Debug)]
struct EnemySpawnConfig {
    archetype: String,
    position: [f32; 3],
}

#[derive(Deserialize, Clone, Copy, Debug, Default)]
enum EnemyPathMode {
    #[default]
    Refined,
    Coarse,
    AStar,
    Waypoints,
    ThetaStar,
}

impl EnemyPathMode {
    fn into_pathfind_mode(self) -> PathfindMode {
        match self {
            Self::Refined => PathfindMode::Refined,
            Self::Coarse => PathfindMode::Coarse,
            Self::AStar => PathfindMode::AStar,
            Self::Waypoints => PathfindMode::Waypoints,
            Self::ThetaStar => PathfindMode::ThetaStar,
        }
    }

    fn supports_partial(self) -> bool {
        matches!(self, Self::AStar | Self::ThetaStar)
    }
}

#[derive(Component)]
struct Enemy;

#[derive(Component)]
struct EnemyMover {
    move_speed: f32,
}

#[derive(Clone, Component, Debug, ScorerBuilder)]
struct TargetInRangeScorer {
    chase_range: f32,
}

#[derive(Clone, Component, Debug, ActionBuilder)]
struct ChaseTargetAction {
    repath_interval_secs: f32,
    stopping_distance: f32,
    path_mode: EnemyPathMode,
    repath_elapsed: f32,
}

#[derive(Clone, Component, Debug, ActionBuilder)]
struct HoldPositionAction;

fn default_world_min() -> [i32; 3] {
    [-48, FLOOR_MIN_Y + 1, -48]
}

fn default_nav_dimensions() -> [u32; 3] {
    let nav_height = (CEILING_MAX_Y - FLOOR_MIN_Y - 1).max(1) as u32;
    [96, nav_height, 96]
}

fn default_chunk_size() -> u32 {
    8
}

fn default_chunk_depth() -> u32 {
    8
}

fn default_enemy_color() -> [f32; 3] {
    [0.86, 0.14, 0.18]
}

fn load_enemy_ai_config(mut state: ResMut<EnemyConfigState>, asset_server: Res<AssetServer>) {
    state.handle = asset_server.load(ENEMY_CONFIG_PATH);
}

fn initialize_enemy_ai_from_config(
    mut commands: Commands,
    mut state: ResMut<EnemyConfigState>,
    mut archetypes: ResMut<EnemyArchetypeLibrary>,
    configs: Res<Assets<EnemyAiConfig>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if state.initialized {
        return;
    }

    let Some(config) = configs.get(&state.handle) else {
        return;
    };

    let nav_dimensions = UVec3::new(
        config.nav.dimensions[0].max(3),
        config.nav.dimensions[1].max(1),
        config.nav.dimensions[2].max(3),
    );
    let nav_world_min = IVec3::new(
        config.nav.world_min[0],
        config.nav.world_min[1],
        config.nav.world_min[2],
    );

    let grid = build_navigation_grid(
        nav_world_min,
        nav_dimensions,
        config.nav.chunk_size.max(3),
        config.nav.chunk_depth.max(1),
    );
    let grid_entity = commands.spawn(grid).id();
    let nav_grid = EnemyNavigationGrid {
        entity: grid_entity,
        world_min: nav_world_min,
        dimensions: nav_dimensions,
    };
    commands.insert_resource(nav_grid);

    archetypes.0.clear();
    for archetype in &config.archetypes {
        archetypes.0.insert(archetype.id.clone(), archetype.clone());
    }

    for spawn in &config.spawns {
        spawn_enemy_from_config(
            &mut commands,
            &mut meshes,
            &mut materials,
            &archetypes.0,
            nav_grid,
            spawn,
        );
    }

    state.initialized = true;
    info!("Enemy AI prototype initialized from {ENEMY_CONFIG_PATH}");
}

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

fn spawn_enemy_from_config(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    archetypes: &HashMap<String, EnemyArchetypeConfig>,
    nav_grid: EnemyNavigationGrid,
    spawn: &EnemySpawnConfig,
) {
    let Some(archetype) = archetypes.get(&spawn.archetype) else {
        warn!(
            "Enemy spawn references unknown archetype '{}'",
            spawn.archetype
        );
        return;
    };

    let world_position = Vec3::new(spawn.position[0], spawn.position[1], spawn.position[2]);
    let Some(agent_pos) = world_to_grid_cell(world_position, nav_grid.world_min, nav_grid.dimensions)
    else {
        warn!(
            "Enemy spawn {:?} for '{}' is outside navigation bounds",
            spawn.position, spawn.archetype
        );
        return;
    };

    let thinker = Thinker::build()
        .label(format!("{}Thinker", archetype.id))
        .picker(HighestToScore::new(0.05))
        .when(
            TargetInRangeScorer {
                chase_range: archetype.chase_range.max(0.1),
            },
            ChaseTargetAction {
                repath_interval_secs: archetype.repath_interval.max(0.05),
                stopping_distance: archetype.stopping_distance.max(0.0),
                path_mode: archetype.path_mode,
                repath_elapsed: 0.0,
            },
        )
        .otherwise(HoldPositionAction);

    commands.spawn((
        Name::new(format!("Enemy::{}", archetype.id)),
        Enemy,
        EnemyMover {
            move_speed: archetype.move_speed.max(0.1),
        },
        Transform::from_translation(world_position),
        Mesh3d(meshes.add(Sphere::new(ENEMY_RENDER_RADIUS))),
        MeshMaterial3d(materials.add(Color::srgb(
            archetype.color[0],
            archetype.color[1],
            archetype.color[2],
        ))),
        AgentPos(agent_pos),
        AgentOfGrid(nav_grid.entity),
        thinker,
    ));
}

fn target_in_range_scorer_system(
    player: Query<&Transform, With<Player>>,
    enemies: Query<&Transform, With<Enemy>>,
    mut scorers: Query<(&Actor, &mut Score, &TargetInRangeScorer)>,
) {
    let Ok(player_transform) = player.single() else {
        return;
    };

    for (Actor(actor), mut score, scorer) in &mut scorers {
        let Ok(enemy_transform) = enemies.get(*actor) else {
            score.set(0.0);
            continue;
        };

        let distance = enemy_transform
            .translation
            .distance(player_transform.translation);
        let utility = (1.0 - distance / scorer.chase_range.max(0.001)).clamp(0.0, 1.0);
        score.set(utility);
    }
}

fn chase_target_action_system(
    time: Res<Time>,
    player: Query<&Transform, With<Player>>,
    enemies: Query<&Transform, With<Enemy>>,
    nav_grid: Option<Res<EnemyNavigationGrid>>,
    mut actions: Query<(&Actor, &mut ActionState, &mut ChaseTargetAction, &ActionSpan)>,
    mut commands: Commands,
) {
    let Some(nav_grid) = nav_grid else {
        return;
    };
    let Ok(player_transform) = player.single() else {
        return;
    };
    let Some(goal_cell) = world_to_grid_cell(
        player_transform.translation,
        nav_grid.world_min,
        nav_grid.dimensions,
    ) else {
        return;
    };

    for (Actor(actor), mut state, mut chase, span) in &mut actions {
        let _guard = span.span().enter();

        match *state {
            ActionState::Requested => {
                chase.repath_elapsed = chase.repath_interval_secs;
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                let Ok(enemy_transform) = enemies.get(*actor) else {
                    *state = ActionState::Failure;
                    continue;
                };

                let distance = enemy_transform
                    .translation
                    .distance(player_transform.translation);

                if distance <= chase.stopping_distance {
                    commands.entity(*actor).remove::<(Pathfind, NextPos)>();
                    continue;
                }

                chase.repath_elapsed += time.delta_secs();
                if chase.repath_elapsed >= chase.repath_interval_secs {
                    chase.repath_elapsed = 0.0;
                    let mut request = Pathfind::new(goal_cell).mode(chase.path_mode.into_pathfind_mode());
                    if chase.path_mode.supports_partial() {
                        request = request.partial();
                    }
                    commands.entity(*actor).insert(request);
                }
            }
            ActionState::Cancelled => {
                chase.repath_elapsed = 0.0;
                commands.entity(*actor).remove::<(Pathfind, NextPos)>();
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

fn hold_position_action_system(
    mut actions: Query<(&Actor, &mut ActionState, &ActionSpan), With<HoldPositionAction>>,
    mut commands: Commands,
) {
    for (Actor(actor), mut state, span) in &mut actions {
        let _guard = span.span().enter();

        match *state {
            ActionState::Requested => {
                commands.entity(*actor).remove::<(Pathfind, NextPos)>();
                *state = ActionState::Executing;
            }
            ActionState::Cancelled => {
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

fn move_enemy_agents(
    time: Res<Time>,
    nav_grid: Option<Res<EnemyNavigationGrid>>,
    mut query: Query<(Entity, &EnemyMover, &mut Transform, &mut AgentPos, &NextPos), With<Enemy>>,
    mut commands: Commands,
) {
    let Some(nav_grid) = nav_grid else {
        return;
    };

    for (entity, mover, mut transform, mut agent_pos, next_pos) in &mut query {
        let target_world = grid_cell_to_world_center(next_pos.0, nav_grid.world_min);
        let delta = target_world - transform.translation;
        let distance = delta.length();

        if distance <= ENEMY_POSITION_TOLERANCE {
            transform.translation = target_world;
            agent_pos.0 = next_pos.0;
            commands.entity(entity).remove::<NextPos>();
            continue;
        }

        let step = mover.move_speed * time.delta_secs();
        if distance <= step {
            transform.translation = target_world;
            agent_pos.0 = next_pos.0;
            commands.entity(entity).remove::<NextPos>();
            continue;
        }

        transform.translation += delta.normalize_or_zero() * step;
    }
}

fn clear_pathfinding_failures(
    failures: Query<Entity, (With<Enemy>, With<PathfindingFailed>)>,
    mut commands: Commands,
) {
    for entity in &failures {
        commands
            .entity(entity)
            .remove::<PathfindingFailed>()
            .remove::<NextPos>();
    }
}

fn world_to_grid_cell(position: Vec3, world_min: IVec3, dimensions: UVec3) -> Option<UVec3> {
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

fn grid_cell_to_world_center(cell: UVec3, world_min: IVec3) -> Vec3 {
    Vec3::new(
        world_min.x as f32 + cell.x as f32 + ENEMY_CELL_CENTER_OFFSET,
        world_min.y as f32 + cell.y as f32 + ENEMY_CELL_CENTER_OFFSET,
        world_min.z as f32 + cell.z as f32 + ENEMY_CELL_CENTER_OFFSET,
    )
}

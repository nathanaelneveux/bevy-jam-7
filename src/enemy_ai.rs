//! Enemy AI prototype module.
//!
//! This module combines three systems into a jam-friendly, data-driven enemy stack:
//! - `bevy_common_assets` for external archetype/spawn configuration (`*.enemy.ron`).
//! - `big-brain` for utility-style intent selection.
//! - `bevy_northstar` for voxel-grid pathfinding.
//!
//! The intent is to make enemy authoring fast:
//! - tune archetypes in `assets/enemies/descent.enemy.ron`
//! - hot-reload config while the game runs
//! - watch live "thinking" state in a lightweight debug HUD.

use std::{collections::HashMap, fmt::Display};

use bevy::{asset::AssetEvent, prelude::*};
use bevy_common_assets::ron::RonAssetPlugin;
use bevy_northstar::prelude::*;
use big_brain::prelude::*;
use serde::Deserialize;

use crate::{
    enemy_navigation::{
        EnemyChunkNavCache, EnemyNavigationAgent, EnemyNavigationConfig, EnemyNavigationGrid,
        EnemyNavigationGridMarker, EnemyNavigationRecenterState,
        apply_chunk_nav_on_spawn_or_remesh, apply_ready_navigation_grid_recenter,
        clear_chunk_nav_on_despawn, clear_pending_navigation_recenter, create_navigation_grid,
        grid_cell_to_world_center, request_navigation_grid_recenter, world_to_grid_cell,
    },
    player_controller::Player,
};

/// RON asset path for the enemy prototype config.
const ENEMY_CONFIG_PATH: &str = "enemies/descent.enemy.ron";
/// Render radius for the debug enemy sphere.
const ENEMY_RENDER_RADIUS: f32 = 0.35;
/// Distance threshold for snapping to the next path cell.
const ENEMY_POSITION_TOLERANCE: f32 = 0.05;
/// Font size for the on-screen enemy thought debug panel.
const ENEMY_DEBUG_FONT_SIZE: f32 = 14.0;

/// Plugin that wires config loading, utility AI, pathfinding, and debug UI.
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
        .init_resource::<EnemyChunkNavCache>()
        .init_resource::<EnemyNavigationRecenterState>()
        .init_resource::<EnemyDebugOverlay>()
        .add_systems(
            Startup,
            (load_enemy_ai_config, spawn_enemy_debug_overlay_ui),
        )
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
                initialize_or_reload_enemy_ai_from_config,
                ApplyDeferred,
                request_navigation_grid_recenter,
                apply_ready_navigation_grid_recenter,
                apply_chunk_nav_on_spawn_or_remesh,
                clear_chunk_nav_on_despawn,
                ApplyDeferred,
                toggle_enemy_debug_overlay,
                update_enemy_debug_overlay_ui,
                draw_enemy_thought_gizmos,
                move_enemy_agents.after(PathingSet),
                clear_pathfinding_failures,
            )
                .chain(),
        );
    }
}

/// Tracks load/reload lifecycle for the external enemy config asset.
#[derive(Resource, Default)]
struct EnemyConfigState {
    /// Asset handle for `EnemyAiConfig`.
    handle: Handle<EnemyAiConfig>,
    /// Whether initial spawn has been completed.
    initialized: bool,
    /// Whether a hot reload should be applied on next update.
    needs_reload: bool,
}

/// Runtime lookup table for archetypes by ID.
#[derive(Resource, Default)]
struct EnemyArchetypeLibrary(HashMap<String, EnemyArchetypeConfig>);

/// Runtime debug state written by scorers/actions and visualized in HUD/gizmos.
#[derive(Component, Default)]
struct EnemyThoughtState {
    /// Last utility score produced by the range scorer.
    utility: f32,
    /// Last measured distance from enemy to player.
    player_distance: f32,
    /// Current high-level intent.
    intent: EnemyIntent,
}

/// High-level intent labels used for debugging.
#[derive(Clone, Copy, Debug, Default)]
enum EnemyIntent {
    /// No active pursuit; waiting/idle behavior.
    #[default]
    Idle,
    /// Pursuit behavior currently active.
    Chasing,
}

impl Display for EnemyIntent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Idle => write!(f, "Idle"),
            Self::Chasing => write!(f, "Chasing"),
        }
    }
}

/// Toggleable overlay state for the enemy thought debug panel.
#[derive(Resource)]
struct EnemyDebugOverlay {
    enabled: bool,
}

impl Default for EnemyDebugOverlay {
    fn default() -> Self {
        Self { enabled: false }
    }
}

/// UI marker for the debug text panel.
#[derive(Component)]
struct EnemyDebugOverlayText;

/// Top-level asset loaded from `*.enemy.ron`.
#[derive(Asset, TypePath, Deserialize, Clone, Debug)]
struct EnemyAiConfig {
    /// Nav grid generation settings.
    #[serde(default)]
    nav: EnemyNavigationConfig,
    /// Enemy archetype templates.
    #[serde(default)]
    archetypes: Vec<EnemyArchetypeConfig>,
    /// Concrete spawn entries referencing archetype IDs.
    #[serde(default)]
    spawns: Vec<EnemySpawnConfig>,
}

/// Data-driven enemy archetype. Add more of these in config to create variants.
#[derive(Deserialize, Clone, Debug)]
struct EnemyArchetypeConfig {
    /// Stable archetype identifier used by spawn entries.
    id: String,
    /// World-space movement speed when following path cells.
    move_speed: f32,
    /// Max distance where chase utility ramps from 1 -> 0.
    chase_range: f32,
    /// Distance to player where chasing pauses.
    stopping_distance: f32,
    /// Time between path requests while chasing.
    repath_interval: f32,
    /// Pathfinding algorithm.
    #[serde(default)]
    path_mode: EnemyPathMode,
    /// Debug render color for the enemy sphere.
    #[serde(default = "default_enemy_color")]
    color: [f32; 3],
}

/// Concrete spawn entry.
#[derive(Deserialize, Clone, Debug)]
struct EnemySpawnConfig {
    /// Archetype id to instantiate.
    archetype: String,
    /// Initial world-space position.
    position: [f32; 3],
}

/// Serializable wrapper around Northstar pathfinding modes.
#[derive(Deserialize, Clone, Copy, Debug, Default)]
enum EnemyPathMode {
    /// HPA* refined path (default).
    #[default]
    Refined,
    /// HPA* coarse path.
    Coarse,
    /// Full-grid A*.
    AStar,
    /// Any-angle HPA* (waypoints).
    Waypoints,
    /// Theta* any-angle shortest-like path.
    ThetaStar,
}

impl EnemyPathMode {
    /// Converts config enum to Northstar enum.
    fn into_pathfind_mode(self) -> PathfindMode {
        match self {
            Self::Refined => PathfindMode::Refined,
            Self::Coarse => PathfindMode::Coarse,
            Self::AStar => PathfindMode::AStar,
            Self::Waypoints => PathfindMode::Waypoints,
            Self::ThetaStar => PathfindMode::ThetaStar,
        }
    }

    /// Returns whether this mode supports partial paths in Northstar.
    fn supports_partial(self) -> bool {
        matches!(self, Self::AStar | Self::ThetaStar)
    }
}

/// Enemy marker component.
#[derive(Component)]
struct Enemy;

/// Movement tuning carried by each spawned enemy.
#[derive(Component)]
struct EnemyMover {
    /// World units per second.
    move_speed: f32,
}

/// Utility scorer: how strongly should this enemy chase the player right now.
#[derive(Clone, Component, Debug, ScorerBuilder)]
struct TargetInRangeScorer {
    /// Distance where utility reaches 0.
    chase_range: f32,
}

/// Chase action: periodically request/re-request Northstar paths toward the player.
#[derive(Clone, Component, Debug, ActionBuilder)]
struct ChaseTargetAction {
    /// Seconds between path requests.
    repath_interval_secs: f32,
    /// Distance where enemy should stop pushing closer.
    stopping_distance: f32,
    /// Selected pathfinding mode.
    path_mode: EnemyPathMode,
    /// Internal timer accumulator.
    repath_elapsed: f32,
}

/// Idle fallback action.
#[derive(Clone, Component, Debug, ActionBuilder)]
struct HoldPositionAction;

/// Default debug enemy color.
fn default_enemy_color() -> [f32; 3] {
    [0.86, 0.14, 0.18]
}

/// Requests loading the enemy config asset.
fn load_enemy_ai_config(mut state: ResMut<EnemyConfigState>, asset_server: Res<AssetServer>) {
    state.handle = asset_server.load(ENEMY_CONFIG_PATH);
}

/// Spawns the on-screen debug panel listing enemy "thoughts".
fn spawn_enemy_debug_overlay_ui(mut commands: Commands) {
    commands.spawn((
        EnemyDebugOverlayText,
        Text::new("Enemy debug pending config load..."),
        TextFont {
            font_size: ENEMY_DEBUG_FONT_SIZE,
            ..default()
        },
        TextColor(Color::srgb(0.92, 0.96, 1.0)),
        Node {
            position_type: PositionType::Absolute,
            top: px(8),
            left: px(8),
            ..default()
        },
        BackgroundColor(Color::srgba(0.02, 0.03, 0.05, 0.66)),
        Visibility::Hidden,
    ));
}

/// Handles initial config load and hot reload rebuilds when the asset changes.
#[allow(clippy::too_many_arguments)]
fn initialize_or_reload_enemy_ai_from_config(
    mut commands: Commands,
    mut state: ResMut<EnemyConfigState>,
    mut archetypes: ResMut<EnemyArchetypeLibrary>,
    mut chunk_nav_cache: ResMut<EnemyChunkNavCache>,
    mut recenter_state: ResMut<EnemyNavigationRecenterState>,
    configs: Res<Assets<EnemyAiConfig>>,
    mut config_events: MessageReader<AssetEvent<EnemyAiConfig>>,
    existing_enemies: Query<Entity, With<Enemy>>,
    existing_grids: Query<Entity, With<EnemyNavigationGridMarker>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let watched_id = state.handle.id();
    for event in config_events.read() {
        if event.is_loaded_with_dependencies(watched_id)
            || event.is_modified(watched_id)
            || event.is_added(watched_id)
        {
            state.needs_reload = true;
        }
    }

    let Some(config) = configs.get(&state.handle) else {
        return;
    };

    if state.initialized && !state.needs_reload {
        return;
    }

    if state.initialized {
        chunk_nav_cache.0.clear();
        clear_pending_navigation_recenter(&mut recenter_state);
        for entity in &existing_enemies {
            commands.entity(entity).try_despawn();
        }
        for entity in &existing_grids {
            commands.entity(entity).try_despawn();
        }
    }

    let nav_grid = create_navigation_grid(&mut commands, &config.nav);
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
    state.needs_reload = false;
    info!("Enemy AI initialized from {ENEMY_CONFIG_PATH}");
}

/// Spawns a concrete enemy from one config spawn entry.
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
    let Some(agent_pos) =
        world_to_grid_cell(world_position, nav_grid.world_min, nav_grid.dimensions)
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
        EnemyNavigationAgent,
        EnemyThoughtState::default(),
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

/// Computes chase utility based on current enemy-player distance.
fn target_in_range_scorer_system(
    player: Query<&Transform, With<Player>>,
    mut enemies: Query<(&Transform, &mut EnemyThoughtState), With<Enemy>>,
    mut scorers: Query<(&Actor, &mut Score, &TargetInRangeScorer)>,
) {
    let Ok(player_transform) = player.single() else {
        return;
    };

    for (Actor(actor), mut score, scorer) in &mut scorers {
        let Ok((enemy_transform, mut thought)) = enemies.get_mut(*actor) else {
            score.set(0.0);
            continue;
        };

        let distance = enemy_transform
            .translation
            .distance(player_transform.translation);
        let utility = (1.0 - distance / scorer.chase_range.max(0.001)).clamp(0.0, 1.0);

        thought.utility = utility;
        thought.player_distance = distance;
        score.set(utility);
    }
}

/// Handles chase action state transitions and emits path requests.
fn chase_target_action_system(
    time: Res<Time>,
    player: Query<&Transform, With<Player>>,
    mut enemies: Query<(&Transform, &mut EnemyThoughtState), With<Enemy>>,
    nav_grid: Option<Res<EnemyNavigationGrid>>,
    mut actions: Query<(
        &Actor,
        &mut ActionState,
        &mut ChaseTargetAction,
        &ActionSpan,
    )>,
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
                if let Ok((_, mut thought)) = enemies.get_mut(*actor) {
                    thought.intent = EnemyIntent::Chasing;
                }
                chase.repath_elapsed = chase.repath_interval_secs;
                *state = ActionState::Executing;
            }
            ActionState::Executing => {
                let Ok((enemy_transform, mut thought)) = enemies.get_mut(*actor) else {
                    *state = ActionState::Failure;
                    continue;
                };

                thought.intent = EnemyIntent::Chasing;

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
                    let mut request =
                        Pathfind::new(goal_cell).mode(chase.path_mode.into_pathfind_mode());
                    if chase.path_mode.supports_partial() {
                        request = request.partial();
                    }
                    commands.entity(*actor).insert(request);
                }
            }
            ActionState::Cancelled => {
                if let Ok((_, mut thought)) = enemies.get_mut(*actor) {
                    thought.intent = EnemyIntent::Idle;
                }
                chase.repath_elapsed = 0.0;
                commands.entity(*actor).remove::<(Pathfind, NextPos)>();
                *state = ActionState::Failure;
            }
            _ => {}
        }
    }
}

/// Idle fallback action when chase utility is too low.
fn hold_position_action_system(
    mut enemies: Query<&mut EnemyThoughtState, With<Enemy>>,
    mut actions: Query<(&Actor, &mut ActionState, &ActionSpan), With<HoldPositionAction>>,
    mut commands: Commands,
) {
    for (Actor(actor), mut state, span) in &mut actions {
        let _guard = span.span().enter();

        match *state {
            ActionState::Requested => {
                if let Ok(mut thought) = enemies.get_mut(*actor) {
                    thought.intent = EnemyIntent::Idle;
                }
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

/// Converts `NextPos` path output into smooth world-space movement.
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

/// Clears transient pathfinding failures for prototype continuity.
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

/// Toggles enemy debug overlay and gizmos.
fn toggle_enemy_debug_overlay(
    keys: Res<ButtonInput<KeyCode>>,
    mut overlay: ResMut<EnemyDebugOverlay>,
) {
    if keys.just_pressed(KeyCode::F6) {
        overlay.enabled = !overlay.enabled;
        info!(
            "Enemy debug overlay {}",
            if overlay.enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
    }
}

/// Updates the text panel that summarizes enemy intent/score/pathing state.
fn update_enemy_debug_overlay_ui(
    overlay: Res<EnemyDebugOverlay>,
    mut ui_text: Single<(&mut Text, &mut Visibility), With<EnemyDebugOverlayText>>,
    enemies: Query<
        (
            Entity,
            &Name,
            &EnemyThoughtState,
            Option<&Pathfind>,
            Option<&Path>,
            Option<&NextPos>,
        ),
        With<Enemy>,
    >,
) {
    if !overlay.enabled {
        ui_text.0.clear();
        *ui_text.1 = Visibility::Hidden;
        return;
    }

    *ui_text.1 = Visibility::Visible;

    let mut lines = Vec::new();
    lines.push(format!("Enemy Debug [F6]  count={}", enemies.iter().len()));

    for (entity, name, thought, pathfind, path, next_pos) in &enemies {
        let path_len = path.map_or(0, Path::len);
        let has_request = pathfind.is_some();
        let next = next_pos
            .map(|next| format!("{:?}", next.0))
            .unwrap_or_else(|| "-".to_string());

        lines.push(format!(
            "{} ({:?}) | intent={} score={:.2} dist={:.1} req={} path_len={} next={}",
            name.as_str(),
            entity,
            thought.intent,
            thought.utility,
            thought.player_distance,
            has_request,
            path_len,
            next,
        ));
    }

    *ui_text.0 = lines.join("\n").into();
}

/// Draws a quick visual line from each enemy to the player, color-coded by intent.
fn draw_enemy_thought_gizmos(
    overlay: Res<EnemyDebugOverlay>,
    player: Query<&Transform, With<Player>>,
    enemies: Query<(&Transform, &EnemyThoughtState), With<Enemy>>,
    mut gizmos: Gizmos,
) {
    if !overlay.enabled {
        return;
    }

    let Ok(player_transform) = player.single() else {
        return;
    };

    for (enemy_transform, thought) in &enemies {
        let color = match thought.intent {
            EnemyIntent::Idle => Color::srgba(0.7, 0.7, 0.7, 0.45),
            EnemyIntent::Chasing => Color::srgba(0.95, 0.22, 0.2, 0.85),
        };
        gizmos.line(
            enemy_transform.translation,
            player_transform.translation,
            color,
        );
    }
}

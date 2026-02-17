use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::reflect::TypePath;
use bevy_common_assets::ron::RonAssetPlugin;
use serde::Deserialize;
use std::f32::consts::PI;

use crate::{
    cave_world::CaveWorld,
    ground_ik::{GroundIkSet, GroundedTwoBoneIkOwner},
    ground_ik_walk::GroundedTwoBoneIkWalk,
    mob_nav::{MobNavAgent, MobNavGoal, MobNavMovementMode, MobNavUpdateSet},
    player_controller::Player,
};

use super::ai::{EnemyAiBrain, EnemyAiPersonality, EnemyHealth};
use super::spider_ik::{SpiderVisualRoot, init_spider_leg_rig};

const GOLDEN_ANGLE: f32 = 2.399_963_1;
const SPIDER_MODEL_ASSET_PATH: &str = "Spider.glb#Scene0";
const SPIDER_ARCHETYPE_ASSET_PATH: &str = "enemies/spider.enemy.ron";

pub(crate) struct SpiderEnemyPlugin;

impl Plugin for SpiderEnemyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(RonAssetPlugin::<SpiderEnemyArchetype>::new(&["enemy.ron"]))
            .init_resource::<SpiderEnemyArchetypeHandle>()
            .init_resource::<SpiderEnemySceneHandle>()
            .init_resource::<SpiderEnemyArchetypeCache>()
            .init_resource::<SpiderEnemySpawnState>()
            .add_systems(Startup, load_spider_enemy_assets)
            .add_systems(
                Update,
                (
                    cache_loaded_spider_enemy_archetype,
                    spawn_spider_enemy,
                    face_spiders_toward_movement.after(MobNavUpdateSet::ApplyResults),
                    init_spider_leg_rig.before(GroundIkSet::Solve),
                ),
            );
    }
}

#[derive(Component)]
struct SpiderEnemy;

#[derive(Resource, Default)]
struct SpiderEnemyArchetypeHandle(Handle<SpiderEnemyArchetype>);

#[derive(Resource, Default)]
struct SpiderEnemySceneHandle(Handle<Scene>);

#[derive(Resource, Default)]
struct SpiderEnemyArchetypeCache {
    archetype: Option<SpiderEnemyArchetype>,
}

#[derive(Resource)]
struct SpiderEnemySpawnState {
    timer: Timer,
    spawn_index: u64,
}

impl Default for SpiderEnemySpawnState {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(1.5, TimerMode::Repeating),
            spawn_index: 0,
        }
    }
}

#[derive(Asset, TypePath, Deserialize, Clone, Debug)]
struct SpiderEnemyArchetype {
    max_alive: usize,
    spawn_interval_secs: f32,
    spawn_min_distance: f32,
    spawn_max_distance: f32,
    spawn_height_offset: f32,
    collider_radius: f32,
    collider_half_length: f32,
    move_speed: f32,
    arrival_tolerance: f32,
    #[serde(default)]
    ai: EnemyAiPersonality,
    hit_points: f32,
    body_turn_speed_rad_per_sec: f32,
    visual_y_offset: f32,
    visual_yaw_offset_degrees: f32,
}

impl SpiderEnemyArchetype {
    fn sanitized(&self) -> Self {
        Self {
            max_alive: self.max_alive.max(1),
            spawn_interval_secs: self.spawn_interval_secs.clamp(0.05, 60.0),
            spawn_min_distance: self.spawn_min_distance.max(1.0),
            spawn_max_distance: self
                .spawn_max_distance
                .max(self.spawn_min_distance.max(1.0)),
            spawn_height_offset: self.spawn_height_offset.clamp(0.2, 6.0),
            collider_radius: self.collider_radius.clamp(0.05, 2.5),
            collider_half_length: self.collider_half_length.clamp(0.05, 3.0),
            move_speed: self.move_speed.clamp(0.1, 30.0),
            arrival_tolerance: self.arrival_tolerance.clamp(0.05, 3.0),
            ai: self.ai.sanitized(),
            hit_points: self.hit_points.clamp(1.0, 500.0),
            body_turn_speed_rad_per_sec: self.body_turn_speed_rad_per_sec.clamp(0.1, 25.0),
            visual_y_offset: self.visual_y_offset.clamp(-8.0, 8.0),
            visual_yaw_offset_degrees: self.visual_yaw_offset_degrees,
        }
    }
}

fn load_spider_enemy_assets(
    mut archetype_handle: ResMut<SpiderEnemyArchetypeHandle>,
    mut spider_scene_handle: ResMut<SpiderEnemySceneHandle>,
    asset_server: Res<AssetServer>,
) {
    archetype_handle.0 = asset_server.load(SPIDER_ARCHETYPE_ASSET_PATH);
    spider_scene_handle.0 = asset_server.load(SPIDER_MODEL_ASSET_PATH);
}

fn cache_loaded_spider_enemy_archetype(
    archetype_handle: Res<SpiderEnemyArchetypeHandle>,
    spider_archetypes: Res<Assets<SpiderEnemyArchetype>>,
    mut cache: ResMut<SpiderEnemyArchetypeCache>,
) {
    let Some(loaded) = spider_archetypes.get(&archetype_handle.0) else {
        return;
    };

    cache.archetype = Some(loaded.sanitized());
}

#[allow(clippy::too_many_arguments)]
fn spawn_spider_enemy(
    mut commands: Commands,
    time: Res<Time>,
    cave_world: Res<CaveWorld>,
    mut spawn_state: ResMut<SpiderEnemySpawnState>,
    archetype_cache: Res<SpiderEnemyArchetypeCache>,
    spider_scene_handle: Res<SpiderEnemySceneHandle>,
    player: Query<&GlobalTransform, With<Player>>,
    existing_spiders: Query<(), With<SpiderEnemy>>,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };

    sync_spawn_timer_duration(&mut spawn_state.timer, archetype.spawn_interval_secs);
    if !spawn_state.timer.tick(time.delta()).just_finished() {
        return;
    }

    if existing_spiders.iter().len() >= archetype.max_alive {
        return;
    }

    let Some(player_transform) = player.iter().next() else {
        return;
    };

    spawn_state.spawn_index = spawn_state.spawn_index.wrapping_add(1);
    let player_position = player_transform.translation();
    let spawn_translation = next_spider_spawn_translation(
        spawn_state.spawn_index,
        player_position,
        archetype,
        &cave_world,
    );

    let spider = commands
        .spawn((
            Name::new("SpiderEnemy"),
            SpiderEnemy,
            RigidBody::Dynamic,
            Collider::capsule(archetype.collider_radius, archetype.collider_half_length),
            Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
            LinearVelocity::ZERO,
            LockedAxes::ROTATION_LOCKED,
            MobNavAgent {
                movement_mode: MobNavMovementMode::Ground,
                max_speed: archetype.move_speed,
                arrival_tolerance: archetype.arrival_tolerance,
            },
            MobNavGoal {
                position: player_position,
            },
            archetype.ai,
            EnemyAiBrain::seeded(spawn_state.spawn_index as u32),
            EnemyHealth::with_max(archetype.hit_points),
            GroundedTwoBoneIkOwner,
            GroundedTwoBoneIkWalk,
            Transform::from_translation(spawn_translation),
        ))
        .id();

    commands.entity(spider).with_children(|parent| {
        parent.spawn((
            Name::new("SpiderEnemyVisual"),
            SpiderVisualRoot { owner: spider },
            SceneRoot(spider_scene_handle.0.clone()),
            Transform {
                translation: Vec3::new(0.0, archetype.visual_y_offset, 0.0),
                rotation: Quat::from_rotation_y(archetype.visual_yaw_offset_degrees.to_radians()),
                ..default()
            },
        ));
    });
}

fn face_spiders_toward_movement(
    time: Res<Time>,
    archetype_cache: Res<SpiderEnemyArchetypeCache>,
    mut spiders: Query<(&LinearVelocity, &MobNavGoal, &mut Transform), With<SpiderEnemy>>,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };

    for (velocity, goal, mut transform) in &mut spiders {
        let mut direction = Vec3::new(velocity.x, 0.0, velocity.z);
        if direction.length_squared() <= 0.0001 {
            direction = goal.position - transform.translation;
            direction.y = 0.0;
        }

        if direction.length_squared() <= 0.0001 {
            continue;
        }

        let target_rotation = Transform::default()
            .looking_to(direction.normalize(), Vec3::Y)
            .rotation;
        let turn_lerp = (archetype.body_turn_speed_rad_per_sec * time.delta_secs()).clamp(0.0, 1.0);
        transform.rotation = transform.rotation.slerp(target_rotation, turn_lerp);
    }
}

fn sync_spawn_timer_duration(timer: &mut Timer, spawn_interval_secs: f32) {
    if timer.duration().as_secs_f32() == spawn_interval_secs {
        return;
    }

    *timer = Timer::from_seconds(spawn_interval_secs, TimerMode::Repeating);
}

fn next_spider_spawn_translation(
    spawn_index: u64,
    player_position: Vec3,
    archetype: &SpiderEnemyArchetype,
    cave_world: &CaveWorld,
) -> Vec3 {
    let radius_factor = (spawn_index as f32 * 0.618_034).fract();
    let radius = archetype.spawn_min_distance
        + (archetype.spawn_max_distance - archetype.spawn_min_distance) * radius_factor;
    let angle = GOLDEN_ANGLE * spawn_index as f32 + PI;

    let x = player_position.x + angle.cos() * radius;
    let z = player_position.z + angle.sin() * radius;
    let (floor_y, _) = cave_world.sample_column_bounds(x.round() as i32, z.round() as i32);

    Vec3::new(x, floor_y as f32 + archetype.spawn_height_offset, z)
}

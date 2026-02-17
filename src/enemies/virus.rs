use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::reflect::TypePath;
use bevy_common_assets::ron::RonAssetPlugin;
use serde::Deserialize;

use crate::{
    cave_world::CaveWorld,
    ground_ik::{GroundIkSet, GroundedTwoBoneIkOwner},
    ground_ik_walk::GroundedTwoBoneIkWalk,
    mob_nav::{MobNavAgent, MobNavGoal, MobNavMovementMode},
    player_controller::Player,
};

use super::ai::{EnemyAiBrain, EnemyAiPersonality, EnemyAiState, EnemyHealth, desired_engage_goal};
use super::virus_ik::{VirusVisualRoot, init_virus_leg_rig};

const VIRUS_MODEL_ASSET_PATH: &str = "virus.glb#Scene0";
const VIRUS_ARCHETYPE_ASSET_PATH: &str = "enemies/virus.enemy.ron";
const HEAD_NAME_MATCHES: [&str; 5] = ["Head", "head", "VirusHead", "head_joint", "HeadBone"];

pub(crate) struct VirusEnemyPlugin;

impl Plugin for VirusEnemyPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(RonAssetPlugin::<VirusEnemyArchetype>::new(&["enemy.ron"]))
            .init_resource::<VirusEnemyArchetypeHandle>()
            .init_resource::<VirusEnemySceneHandle>()
            .init_resource::<VirusEnemyArchetypeCache>()
            .init_resource::<VirusEnemySpawnState>()
            .init_resource::<VirusLaserVisual>()
            .add_systems(Startup, load_virus_enemy_assets)
            .add_systems(
                Update,
                (
                    cache_loaded_virus_enemy_archetype,
                    spawn_virus_enemy,
                    face_virus_toward_player,
                    attach_virus_head_markers,
                    fire_virus_lasers,
                    tick_virus_lasers,
                    init_virus_leg_rig.before(GroundIkSet::Solve),
                ),
            );
    }
}

#[derive(Component)]
struct VirusEnemy;

#[derive(Component)]
struct VirusHeadAttachment(Entity);

#[derive(Component)]
struct VirusVisualScanned;

#[derive(Component)]
struct VirusLaser {
    velocity: Vec3,
    lifetime: Timer,
}

#[derive(Component)]
struct VirusAttackState {
    cooldown: Timer,
}

#[derive(Resource, Default)]
struct VirusEnemyArchetypeHandle(Handle<VirusEnemyArchetype>);

#[derive(Resource, Default)]
struct VirusEnemySceneHandle(Handle<Scene>);

#[derive(Resource, Default)]
struct VirusEnemyArchetypeCache {
    archetype: Option<VirusEnemyArchetype>,
}

#[derive(Resource)]
struct VirusEnemySpawnState {
    timer: Timer,
    spawn_index: u64,
}

impl Default for VirusEnemySpawnState {
    fn default() -> Self {
        Self {
            timer: Timer::from_seconds(2.5, TimerMode::Repeating),
            spawn_index: 0,
        }
    }
}

#[derive(Resource)]
struct VirusLaserVisual {
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
}

impl FromWorld for VirusLaserVisual {
    fn from_world(world: &mut World) -> Self {
        let mesh = {
            let mut meshes = world.resource_mut::<Assets<Mesh>>();
            meshes.add(Sphere::new(0.06))
        };
        let material = {
            let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
            materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.12, 0.12),
                unlit: true,
                ..default()
            })
        };
        Self { mesh, material }
    }
}

#[derive(Asset, TypePath, Deserialize, Clone, Debug)]
struct VirusEnemyArchetype {
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
    attack_range: f32,
    attack_cooldown_secs: f32,
    laser_speed: f32,
    laser_lifetime_secs: f32,
    laser_hit_radius: f32,
    visual_y_offset: f32,
    visual_yaw_offset_degrees: f32,
}

impl VirusEnemyArchetype {
    fn sanitized(&self) -> Self {
        let ai = self.ai.sanitized();
        Self {
            max_alive: self.max_alive.max(1),
            spawn_interval_secs: self.spawn_interval_secs.clamp(0.1, 60.0),
            spawn_min_distance: self.spawn_min_distance.max(1.0),
            spawn_max_distance: self
                .spawn_max_distance
                .max(self.spawn_min_distance.max(1.0)),
            spawn_height_offset: self.spawn_height_offset.clamp(0.2, 8.0),
            collider_radius: self.collider_radius.clamp(0.05, 2.5),
            collider_half_length: self.collider_half_length.clamp(0.05, 3.0),
            move_speed: self.move_speed.clamp(0.2, 40.0),
            arrival_tolerance: self.arrival_tolerance.clamp(0.1, 8.0),
            ai,
            hit_points: self.hit_points.clamp(1.0, 500.0),
            body_turn_speed_rad_per_sec: self.body_turn_speed_rad_per_sec.clamp(0.1, 30.0),
            attack_range: self
                .attack_range
                .max(ai.attack_enter_distance)
                .clamp(3.0, 100.0),
            attack_cooldown_secs: self.attack_cooldown_secs.clamp(0.05, 10.0),
            laser_speed: self.laser_speed.clamp(1.0, 300.0),
            laser_lifetime_secs: self.laser_lifetime_secs.clamp(0.05, 10.0),
            laser_hit_radius: self.laser_hit_radius.clamp(0.02, 3.0),
            visual_y_offset: self.visual_y_offset.clamp(-10.0, 10.0),
            visual_yaw_offset_degrees: self.visual_yaw_offset_degrees,
        }
    }
}

fn load_virus_enemy_assets(
    mut archetype_handle: ResMut<VirusEnemyArchetypeHandle>,
    mut scene_handle: ResMut<VirusEnemySceneHandle>,
    asset_server: Res<AssetServer>,
) {
    archetype_handle.0 = asset_server.load(VIRUS_ARCHETYPE_ASSET_PATH);
    scene_handle.0 = asset_server.load(VIRUS_MODEL_ASSET_PATH);
}

fn cache_loaded_virus_enemy_archetype(
    archetype_handle: Res<VirusEnemyArchetypeHandle>,
    archetypes: Res<Assets<VirusEnemyArchetype>>,
    mut cache: ResMut<VirusEnemyArchetypeCache>,
) {
    let Some(loaded) = archetypes.get(&archetype_handle.0) else {
        return;
    };

    cache.archetype = Some(loaded.sanitized());
}

#[allow(clippy::too_many_arguments)]
fn spawn_virus_enemy(
    mut commands: Commands,
    time: Res<Time>,
    cave_world: Res<CaveWorld>,
    mut spawn_state: ResMut<VirusEnemySpawnState>,
    archetype_cache: Res<VirusEnemyArchetypeCache>,
    scene_handle: Res<VirusEnemySceneHandle>,
    player: Query<&GlobalTransform, With<Player>>,
    existing_viruses: Query<(), With<VirusEnemy>>,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };

    sync_spawn_timer_duration(&mut spawn_state.timer, archetype.spawn_interval_secs);
    if !spawn_state.timer.tick(time.delta()).just_finished() {
        return;
    }
    if existing_viruses.iter().len() >= archetype.max_alive {
        return;
    }

    let Some(player_transform) = player.iter().next() else {
        return;
    };
    let player_position = player_transform.translation();

    spawn_state.spawn_index = spawn_state.spawn_index.wrapping_add(1);
    let spawn_translation = next_virus_spawn_translation(
        spawn_state.spawn_index,
        player_position,
        archetype,
        &cave_world,
    );

    let initial_goal = desired_engage_goal(
        spawn_translation,
        player_position,
        archetype.ai.preferred_distance,
    );

    let virus = commands
        .spawn((
            Name::new("VirusEnemy"),
            VirusEnemy,
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
                position: initial_goal,
            },
            archetype.ai,
            EnemyAiBrain::seeded((spawn_state.spawn_index as u32) ^ 0xA511_E9B3),
            VirusAttackState {
                cooldown: Timer::from_seconds(archetype.attack_cooldown_secs, TimerMode::Repeating),
            },
            GroundedTwoBoneIkOwner,
            GroundedTwoBoneIkWalk,
            Transform::from_translation(spawn_translation),
        ))
        .id();

    commands
        .entity(virus)
        .insert(EnemyHealth::with_max(archetype.hit_points));

    commands.entity(virus).with_children(|parent| {
        parent.spawn((
            Name::new("VirusEnemyVisual"),
            VirusVisualRoot { owner: virus },
            SceneRoot(scene_handle.0.clone()),
            Transform {
                translation: Vec3::new(0.0, archetype.visual_y_offset, 0.0),
                rotation: Quat::from_rotation_y(archetype.visual_yaw_offset_degrees.to_radians()),
                ..default()
            },
        ));
    });
}

fn face_virus_toward_player(
    time: Res<Time>,
    archetype_cache: Res<VirusEnemyArchetypeCache>,
    player: Query<&GlobalTransform, With<Player>>,
    mut viruses: Query<&mut Transform, With<VirusEnemy>>,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };
    let Some(player_transform) = player.iter().next() else {
        return;
    };
    let player_position = player_transform.translation();

    for mut virus_transform in &mut viruses {
        let mut to_player = player_position - virus_transform.translation;
        to_player.y = 0.0;
        if to_player.length_squared() <= 0.0001 {
            continue;
        }

        let target_rotation = Transform::default()
            .looking_to(to_player.normalize(), Vec3::Y)
            .rotation;
        let lerp = (archetype.body_turn_speed_rad_per_sec * time.delta_secs()).clamp(0.0, 1.0);
        virus_transform.rotation = virus_transform.rotation.slerp(target_rotation, lerp);
    }
}

fn attach_virus_head_markers(
    mut commands: Commands,
    visual_roots: Query<(Entity, &VirusVisualRoot), Without<VirusVisualScanned>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
) {
    for (visual_root, visual) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut matched_head: Option<Entity> = None;
        let mut saw_named_node = false;

        while let Some(entity) = stack.pop() {
            if let Ok(name) = names.get(entity) {
                saw_named_node = true;
                if HEAD_NAME_MATCHES
                    .iter()
                    .any(|needle| name.as_str().contains(needle))
                {
                    matched_head = Some(entity);
                    break;
                }
            }

            if let Ok(children) = children_query.get(entity) {
                for child in children.iter() {
                    stack.push(child);
                }
            }
        }

        if !saw_named_node {
            continue;
        }

        let head_entity = matched_head.unwrap_or(visual_root);
        commands
            .entity(visual.owner)
            .insert(VirusHeadAttachment(head_entity));
        commands.entity(visual_root).insert(VirusVisualScanned);
    }
}

fn fire_virus_lasers(
    mut commands: Commands,
    time: Res<Time>,
    archetype_cache: Res<VirusEnemyArchetypeCache>,
    player: Query<&GlobalTransform, With<Player>>,
    head_transforms: Query<&GlobalTransform>,
    laser_visual: Res<VirusLaserVisual>,
    mut attacks: Query<
        (
            &GlobalTransform,
            &mut VirusAttackState,
            Option<&VirusHeadAttachment>,
            &EnemyAiBrain,
        ),
        With<VirusEnemy>,
    >,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };
    let Some(player_transform) = player.iter().next() else {
        return;
    };
    let player_position = player_transform.translation();

    for (virus_transform, mut attack_state, head_attachment, ai_brain) in &mut attacks {
        if ai_brain.state != EnemyAiState::Attack {
            continue;
        }

        if !attack_state.cooldown.tick(time.delta()).just_finished() {
            continue;
        }

        let distance = virus_transform.translation().distance(player_position);
        if distance > archetype.attack_range {
            continue;
        }

        let muzzle = head_attachment
            .and_then(|head| head_transforms.get(head.0).ok())
            .map(|transform| transform.translation())
            .unwrap_or_else(|| virus_transform.translation() + Vec3::Y * 0.6);

        let mut direction = player_position - muzzle;
        if direction.length_squared() <= 0.0001 {
            continue;
        }
        direction = direction.normalize();

        commands.spawn((
            Name::new("VirusLaser"),
            VirusLaser {
                velocity: direction * archetype.laser_speed,
                lifetime: Timer::from_seconds(archetype.laser_lifetime_secs, TimerMode::Once),
            },
            Mesh3d(laser_visual.mesh.clone()),
            MeshMaterial3d(laser_visual.material.clone()),
            Transform {
                translation: muzzle,
                scale: Vec3::splat((archetype.laser_hit_radius * 2.0).max(0.02)),
                ..default()
            },
        ));
    }
}

fn tick_virus_lasers(
    mut commands: Commands,
    time: Res<Time>,
    archetype_cache: Res<VirusEnemyArchetypeCache>,
    player: Query<&GlobalTransform, With<Player>>,
    mut gizmos: Gizmos,
    mut lasers: Query<(Entity, &mut VirusLaser, &mut Transform)>,
) {
    let Some(archetype) = archetype_cache.archetype.as_ref() else {
        return;
    };

    let player_position = player
        .iter()
        .next()
        .map(|transform| transform.translation());

    for (entity, mut laser, mut transform) in &mut lasers {
        let prev = transform.translation;
        transform.translation += laser.velocity * time.delta_secs();

        gizmos.line(prev, transform.translation, Color::srgb(1.0, 0.1, 0.1));

        if let Some(player_position) = player_position
            && transform.translation.distance(player_position) <= archetype.laser_hit_radius
        {
            info!("Virus laser hit player.");
            commands.entity(entity).despawn();
            continue;
        }

        if laser.lifetime.tick(time.delta()).is_finished() {
            commands.entity(entity).despawn();
        }
    }
}

fn sync_spawn_timer_duration(timer: &mut Timer, spawn_interval_secs: f32) {
    if timer.duration().as_secs_f32() == spawn_interval_secs {
        return;
    }
    *timer = Timer::from_seconds(spawn_interval_secs, TimerMode::Repeating);
}

fn next_virus_spawn_translation(
    spawn_index: u64,
    player_position: Vec3,
    archetype: &VirusEnemyArchetype,
    cave_world: &CaveWorld,
) -> Vec3 {
    let angle = spawn_index as f32 * 1.357_91;
    let radius_factor = (spawn_index as f32 * 0.754_877).fract();
    let radius = archetype.spawn_min_distance
        + (archetype.spawn_max_distance - archetype.spawn_min_distance) * radius_factor;
    let x = player_position.x + angle.cos() * radius;
    let z = player_position.z + angle.sin() * radius;
    let (floor_y, _) = cave_world.sample_column_bounds(x.round() as i32, z.round() as i32);

    Vec3::new(x, floor_y as f32 + archetype.spawn_height_offset, z)
}

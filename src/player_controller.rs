use avian3d::prelude::*;
use bevy::asset::RenderAssetUsages;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy_enhanced_input::prelude::*;
use bevy_northstar::prelude::Blocking;
use bevy_voxel_world::{custom_meshing::CHUNK_SIZE_I, prelude::VoxelWorldCamera};

use crate::{
    InspectorMode,
    cave_world::{CAVE_WORLD_SPAWNING_DISTANCE, CaveWorld},
    enemies::{EnemyDebrisPiece, EnemyHealth, EnemyKilledEvent},
};

const LOOK_SENSITIVITY: f32 = 0.002;
const PLAYER_WALK_SPEED: f32 = 8.5;
const PLAYER_SPRINT_MULTIPLIER: f32 = 1.6;
const PLAYER_GRAVITY: f32 = 28.0;
const PLAYER_JUMP_SPEED: f32 = 10.0;
const PLAYER_CAPSULE_RADIUS: f32 = 0.4;
const PLAYER_CAPSULE_LENGTH: f32 = 1.1;
const PLAYER_CAMERA_HEIGHT: f32 = 0.55;
const GROUND_NORMAL_Y_THRESHOLD: f32 = 0.65;
const DEPTH_FOG_START_OFFSET_CHUNKS: f32 = 6.0;
const DEPTH_FOG_END_OFFSET_CHUNKS: f32 = 0.75;
const PLAYER_BOLT_FIRE_INTERVAL_SECS: f32 = 0.09;
const PLAYER_BOLT_SPEED: f32 = 90.0;
const PLAYER_BOLT_DAMAGE: f32 = 4.0;
const PLAYER_BOLT_LIFETIME_SECS: f32 = 1.2;
const PLAYER_BOLT_MUZZLE_OFFSET: f32 = 0.65;
const PLAYER_BOLT_BILLBOARD_SIZE: f32 = 0.34;
const PLAYER_BOLT_SPRITE_SIZE: u32 = 64;

pub struct PlayerControllerPlugin;

impl Plugin for PlayerControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_input_context::<Player>()
            .init_resource::<PlayerEnergyBoltVisual>()
            .add_systems(Startup, spawn_player)
            .add_systems(
                Update,
                (
                    look_camera,
                    fire_player_energy_bolts,
                    tick_player_energy_bolts,
                    billboard_player_energy_bolts.after(tick_player_energy_bolts),
                ),
            )
            .add_systems(FixedUpdate, player_move_and_slide);
    }
}

#[derive(Component)]
pub(crate) struct Player;

#[derive(Component)]
struct PlayerCamera;

#[derive(InputAction)]
#[action_output(Vec2)]
struct MoveAction;

#[derive(InputAction)]
#[action_output(bool)]
struct JumpAction;

#[derive(InputAction)]
#[action_output(bool)]
struct SprintAction;

#[derive(InputAction)]
#[action_output(bool)]
struct FireAction;

#[derive(Component)]
struct ControllerState {
    pitch: f32,
    grounded: bool,
}

#[derive(Component)]
struct PlayerWeaponState {
    cooldown_remaining_secs: f32,
}

#[derive(Component)]
struct PlayerEnergyBolt {
    velocity: Vec3,
    remaining_life_secs: f32,
    damage: f32,
}

#[derive(Resource)]
struct PlayerEnergyBoltVisual {
    mesh: Handle<Mesh>,
    material: Handle<StandardMaterial>,
}

impl FromWorld for PlayerEnergyBoltVisual {
    fn from_world(world: &mut World) -> Self {
        let texture = {
            let mut images = world.resource_mut::<Assets<Image>>();
            images.add(make_energy_bolt_texture(PLAYER_BOLT_SPRITE_SIZE))
        };
        let mesh = {
            let mut meshes = world.resource_mut::<Assets<Mesh>>();
            meshes.add(Rectangle::new(1.0, 1.0))
        };
        let material = {
            let mut materials = world.resource_mut::<Assets<StandardMaterial>>();
            materials.add(StandardMaterial {
                base_color: Color::srgb(1.0, 0.95, 0.7),
                base_color_texture: Some(texture),
                alpha_mode: AlphaMode::Add,
                unlit: true,
                cull_mode: None,
                ..default()
            })
        };

        Self { mesh, material }
    }
}

fn spawn_player(mut commands: Commands) {
    let chunk_world_size = CHUNK_SIZE_I as f32;
    let spawn_radius_world = CAVE_WORLD_SPAWNING_DISTANCE as f32 * chunk_world_size;
    let fog_start =
        (spawn_radius_world - DEPTH_FOG_START_OFFSET_CHUNKS * chunk_world_size).max(32.0);
    let fog_end =
        (spawn_radius_world - DEPTH_FOG_END_OFFSET_CHUNKS * chunk_world_size).max(fog_start + 1.0);

    commands
        .spawn((
            Player,
            actions!(Player[
                (
                    Action::<MoveAction>::new(),
                    Bindings::spawn(Cardinal::wasd_keys()),
                ),
                (
                    Action::<JumpAction>::new(),
                    bindings![KeyCode::Space],
                ),
                (
                    Action::<SprintAction>::new(),
                    bindings![KeyCode::ShiftLeft, KeyCode::ShiftRight],
                ),
                (
                    Action::<FireAction>::new(),
                    bindings![MouseButton::Left],
                ),
            ]),
            ControllerState {
                pitch: 0.0,
                grounded: false,
            },
            PlayerWeaponState {
                cooldown_remaining_secs: 0.0,
            },
            RigidBody::Kinematic,
            Blocking,
            Collider::capsule(PLAYER_CAPSULE_RADIUS, PLAYER_CAPSULE_LENGTH),
            LinearVelocity::ZERO,
            CustomPositionIntegration,
            Transform::from_xyz(0.0, 10.0, 0.0),
        ))
        .with_children(|parent| {
            parent.spawn((
                Camera3d::default(),
                DistanceFog {
                    color: Color::BLACK,
                    falloff: FogFalloff::Linear {
                        start: fog_start,
                        end: fog_end,
                    },
                    ..default()
                },
                Transform::from_xyz(0.0, PLAYER_CAMERA_HEIGHT, 0.0),
                PlayerCamera,
                VoxelWorldCamera::<CaveWorld>::default(),
            ));
        });
}

fn fire_player_energy_bolts(
    mut commands: Commands,
    time: Res<Time>,
    inspector_mode: Res<InspectorMode>,
    fire_actions: Query<&Action<FireAction>>,
    player: Single<(&mut PlayerWeaponState, &Actions<Player>), With<Player>>,
    player_camera: Single<&GlobalTransform, (With<PlayerCamera>, Without<Player>)>,
    bolt_visual: Res<PlayerEnergyBoltVisual>,
) {
    if inspector_mode.enabled {
        return;
    }

    let (mut weapon_state, actions) = player.into_inner();
    let Some(fire) = fire_actions.iter_many(actions).next() else {
        return;
    };

    weapon_state.cooldown_remaining_secs =
        (weapon_state.cooldown_remaining_secs - time.delta_secs()).max(0.0);
    if !**fire || weapon_state.cooldown_remaining_secs > 0.0 {
        return;
    }

    weapon_state.cooldown_remaining_secs = PLAYER_BOLT_FIRE_INTERVAL_SECS;

    let camera = player_camera.compute_transform();
    let direction = camera.rotation * -Vec3::Z;
    let spawn_position = camera.translation + direction * PLAYER_BOLT_MUZZLE_OFFSET;

    commands.spawn((
        Name::new("PlayerEnergyBolt"),
        PlayerEnergyBolt {
            velocity: direction * PLAYER_BOLT_SPEED,
            remaining_life_secs: PLAYER_BOLT_LIFETIME_SECS,
            damage: PLAYER_BOLT_DAMAGE,
        },
        Mesh3d(bolt_visual.mesh.clone()),
        MeshMaterial3d(bolt_visual.material.clone()),
        Transform {
            translation: spawn_position,
            scale: Vec3::splat(PLAYER_BOLT_BILLBOARD_SIZE),
            ..default()
        },
    ));
}

fn tick_player_energy_bolts(
    mut commands: Commands,
    time: Res<Time>,
    spatial_query: SpatialQuery,
    mut enemy_killed_events: MessageWriter<EnemyKilledEvent>,
    player: Single<Entity, With<Player>>,
    mut enemies: Query<(&mut EnemyHealth, &GlobalTransform)>,
    children_query: Query<&Children>,
    enemy_meshes: Query<(&Mesh3d, &MeshMaterial3d<StandardMaterial>, &GlobalTransform)>,
    mut bolts: Query<(Entity, &mut PlayerEnergyBolt, &mut Transform)>,
) {
    let player_entity = player.into_inner();
    let filter = SpatialQueryFilter::from_excluded_entities([player_entity]);
    let dt = time.delta_secs();

    for (entity, mut bolt, mut transform) in &mut bolts {
        bolt.remaining_life_secs -= dt;
        if bolt.remaining_life_secs <= 0.0 {
            commands.entity(entity).despawn();
            continue;
        }

        let step = bolt.velocity * dt;
        let distance = step.length();
        if distance <= 0.0001 {
            continue;
        }

        let Ok(direction) = Dir3::new(step / distance) else {
            continue;
        };

        if let Some(hit) =
            spatial_query.cast_ray(transform.translation, direction, distance, true, &filter)
        {
            transform.translation += direction.as_vec3() * hit.distance.min(distance);

            if let Ok((mut health, enemy_transform)) = enemies.get_mut(hit.entity)
                && health.apply_damage(bolt.damage)
            {
                let debris_pieces =
                    collect_enemy_debris_pieces(hit.entity, &children_query, &enemy_meshes);
                enemy_killed_events.write(EnemyKilledEvent {
                    position: enemy_transform.translation(),
                    debris_pieces,
                });
                commands.entity(hit.entity).despawn();
            }

            commands.entity(entity).despawn();
            continue;
        }

        transform.translation += step;
    }
}

fn collect_enemy_debris_pieces(
    enemy: Entity,
    children_query: &Query<&Children>,
    enemy_meshes: &Query<(&Mesh3d, &MeshMaterial3d<StandardMaterial>, &GlobalTransform)>,
) -> Vec<EnemyDebrisPiece> {
    const MAX_DEBRIS_PIECES: usize = 12;

    let mut pieces = Vec::with_capacity(MAX_DEBRIS_PIECES);
    let mut stack = vec![enemy];
    while let Some(entity) = stack.pop() {
        if pieces.len() >= MAX_DEBRIS_PIECES {
            break;
        }

        if let Ok((mesh, material, global_transform)) = enemy_meshes.get(entity) {
            pieces.push(EnemyDebrisPiece {
                mesh: mesh.0.clone(),
                material: material.0.clone(),
                transform: global_transform.compute_transform(),
            });
        }

        if let Ok(children) = children_query.get(entity) {
            for child in children.iter() {
                stack.push(child);
            }
        }
    }

    pieces
}

fn billboard_player_energy_bolts(
    player_camera: Single<&GlobalTransform, (With<PlayerCamera>, Without<Player>)>,
    mut bolts: Query<&mut Transform, With<PlayerEnergyBolt>>,
) {
    let camera_position = player_camera.translation();

    for mut transform in &mut bolts {
        let to_camera = camera_position - transform.translation;
        if to_camera.length_squared() <= 0.0001 {
            continue;
        }

        transform.rotation = Transform::default()
            .looking_to(to_camera.normalize(), Vec3::Y)
            .rotation;
    }
}

fn look_camera(
    mouse_motion: Res<AccumulatedMouseMotion>,
    inspector_mode: Res<InspectorMode>,
    player: Single<(&mut Transform, &mut ControllerState), With<Player>>,
    player_camera: Single<&mut Transform, (With<PlayerCamera>, Without<Player>)>,
) {
    if inspector_mode.enabled {
        return;
    }

    let look = mouse_motion.delta * -LOOK_SENSITIVITY;
    if look == Vec2::ZERO {
        return;
    }

    let (mut player_transform, mut controller) = player.into_inner();
    let mut camera_transform = player_camera.into_inner();

    let (mut yaw, _pitch, _roll) = player_transform.rotation.to_euler(EulerRot::YXZ);
    yaw += look.x;
    controller.pitch = (controller.pitch + look.y).clamp(
        -core::f32::consts::FRAC_PI_2 + 0.01,
        core::f32::consts::FRAC_PI_2 - 0.01,
    );

    player_transform.rotation = Quat::from_rotation_y(yaw);
    camera_transform.rotation = Quat::from_rotation_x(controller.pitch);
}

fn player_move_and_slide(
    time: Res<Time>,
    move_and_slide: MoveAndSlide,
    movement_actions: Query<&Action<MoveAction>>,
    jump_actions: Query<&Action<JumpAction>>,
    jump_events: Query<&ActionEvents, With<Action<JumpAction>>>,
    sprint_actions: Query<&Action<SprintAction>>,
    player: Single<
        (
            Entity,
            &Collider,
            &mut Transform,
            &mut LinearVelocity,
            &mut ControllerState,
            &Actions<Player>,
        ),
        With<Player>,
    >,
) {
    let (entity, collider, mut transform, mut linear_velocity, mut controller, actions) =
        player.into_inner();

    let Some(movement) = movement_actions.iter_many(actions).next() else {
        return;
    };
    let Some(jump) = jump_actions.iter_many(actions).next() else {
        return;
    };
    let Some(jump_events) = jump_events.iter_many(actions).next() else {
        return;
    };
    let Some(sprint) = sprint_actions.iter_many(actions).next() else {
        return;
    };

    let mut move_speed = PLAYER_WALK_SPEED;
    if **sprint {
        move_speed *= PLAYER_SPRINT_MULTIPLIER;
    }

    let movement = **movement;
    let wish = Vec3::new(movement.x, 0.0, -movement.y).normalize_or_zero();
    let desired_horizontal = transform.rotation * (wish * move_speed);
    let mut desired_velocity = Vec3::new(
        desired_horizontal.x,
        linear_velocity.y,
        desired_horizontal.z,
    );

    if controller.grounded {
        if **jump && jump_events.contains(ActionEvents::START) {
            desired_velocity.y = PLAYER_JUMP_SPEED;
            controller.grounded = false;
        } else {
            desired_velocity.y = desired_velocity.y.max(0.0);
        }
    }
    desired_velocity.y -= PLAYER_GRAVITY * time.delta_secs();

    let mut grounded = false;
    let MoveAndSlideOutput {
        position,
        projected_velocity,
    } = move_and_slide.move_and_slide(
        collider,
        transform.translation,
        transform.rotation,
        desired_velocity,
        time.delta(),
        &MoveAndSlideConfig::default(),
        &SpatialQueryFilter::from_excluded_entities([entity]),
        |hit| {
            if !hit.intersects() && hit.normal.as_vec3().y >= GROUND_NORMAL_Y_THRESHOLD {
                grounded = true;
            }
            true
        },
    );

    transform.translation = position;
    let mut projected_velocity = projected_velocity;
    if grounded && projected_velocity.y < 0.0 {
        projected_velocity.y = 0.0;
    }
    controller.grounded = grounded;
    linear_velocity.0 = projected_velocity;
}

fn make_energy_bolt_texture(size: u32) -> Image {
    let mut data = Vec::with_capacity((size * size * 4) as usize);
    let center = (size as f32 - 1.0) * 0.5;
    let radius = center.max(1.0);

    for y in 0..size {
        for x in 0..size {
            let dx = x as f32 - center;
            let dy = y as f32 - center;
            let normalized_distance = (dx * dx + dy * dy).sqrt() / radius;
            let falloff = (1.0 - normalized_distance).clamp(0.0, 1.0);
            let core = falloff * falloff;

            let red = (150.0 + 105.0 * core).round() as u8;
            let green = (90.0 + 130.0 * core).round() as u8;
            let blue = (40.0 + 75.0 * core).round() as u8;
            let alpha = (falloff.powf(2.4) * 255.0).round() as u8;

            data.extend_from_slice(&[red, green, blue, alpha]);
        }
    }

    Image::new(
        Extent3d {
            width: size,
            height: size,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    )
}

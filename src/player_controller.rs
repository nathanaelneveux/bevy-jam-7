use avian3d::prelude::*;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_enhanced_input::prelude::*;
use bevy_northstar::prelude::Blocking;
use bevy_voxel_world::{custom_meshing::CHUNK_SIZE_I, prelude::VoxelWorldCamera};

use crate::{
    InspectorMode,
    cave_world::{CAVE_WORLD_SPAWNING_DISTANCE, CaveWorld},
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

pub struct PlayerControllerPlugin;

impl Plugin for PlayerControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_input_context::<Player>()
            .add_systems(Startup, spawn_player)
            .add_systems(Update, look_camera)
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

#[derive(Component)]
struct ControllerState {
    pitch: f32,
    grounded: bool,
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
            ]),
            ControllerState {
                pitch: 0.0,
                grounded: false,
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

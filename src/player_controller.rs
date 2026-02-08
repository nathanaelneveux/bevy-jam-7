use avian3d::prelude::*;
use bevy::input::mouse::AccumulatedMouseMotion;
use bevy::prelude::*;
use bevy_voxel_world::prelude::VoxelWorldCamera;

use crate::cave_world::CaveWorld;

const LOOK_SENSITIVITY: f32 = 0.002;
const PLAYER_WALK_SPEED: f32 = 8.5;
const PLAYER_SPRINT_MULTIPLIER: f32 = 1.6;
const PLAYER_GRAVITY: f32 = 28.0;
const PLAYER_JUMP_SPEED: f32 = 10.0;
const PLAYER_CAPSULE_RADIUS: f32 = 0.4;
const PLAYER_CAPSULE_LENGTH: f32 = 1.1;
const PLAYER_CAMERA_HEIGHT: f32 = 0.55;
const GROUND_NORMAL_Y_THRESHOLD: f32 = 0.65;

pub struct PlayerControllerPlugin;

impl Plugin for PlayerControllerPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_player)
            .add_systems(Update, look_camera)
            .add_systems(FixedUpdate, player_move_and_slide);
    }
}

#[derive(Component)]
struct Player;

#[derive(Component)]
struct PlayerCamera;

#[derive(Component)]
struct ControllerState {
    pitch: f32,
    grounded: bool,
}

fn spawn_player(mut commands: Commands) {
    commands
        .spawn((
            Player,
            ControllerState {
                pitch: 0.0,
                grounded: false,
            },
            RigidBody::Kinematic,
            Collider::capsule(PLAYER_CAPSULE_RADIUS, PLAYER_CAPSULE_LENGTH),
            LinearVelocity::ZERO,
            CustomPositionIntegration,
            Transform::from_xyz(0.0, 10.0, 0.0),
        ))
        .with_children(|parent| {
            parent.spawn((
                Camera3d::default(),
                Transform::from_xyz(0.0, PLAYER_CAMERA_HEIGHT, 0.0),
                PlayerCamera,
                VoxelWorldCamera::<CaveWorld>::default(),
            ));
        });
}

fn look_camera(
    mouse_motion: Res<AccumulatedMouseMotion>,
    player: Single<(&mut Transform, &mut ControllerState), With<Player>>,
    player_camera: Single<&mut Transform, (With<PlayerCamera>, Without<Player>)>,
) {
    let look = mouse_motion.delta * -LOOK_SENSITIVITY;
    if look == Vec2::ZERO {
        return;
    }

    let (mut player_transform, mut controller) = player.into_inner();
    let mut camera_transform = player_camera.into_inner();

    let (mut yaw, _pitch, _roll) = player_transform.rotation.to_euler(EulerRot::YXZ);
    yaw += look.x;
    controller.pitch = (controller.pitch + look.y)
        .clamp(-core::f32::consts::FRAC_PI_2 + 0.01, core::f32::consts::FRAC_PI_2 - 0.01);

    player_transform.rotation = Quat::from_rotation_y(yaw);
    camera_transform.rotation = Quat::from_rotation_x(controller.pitch);
}

fn player_move_and_slide(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    move_and_slide: MoveAndSlide,
    player: Single<
        (
            Entity,
            &Collider,
            &mut Transform,
            &mut LinearVelocity,
            &mut ControllerState,
        ),
        With<Player>,
    >,
) {
    let (entity, collider, mut transform, mut linear_velocity, mut controller) =
        player.into_inner();

    let mut wish = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        wish.z -= 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        wish.z += 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        wish.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        wish.x += 1.0;
    }

    let mut move_speed = PLAYER_WALK_SPEED;
    if keys.pressed(KeyCode::ShiftLeft) {
        move_speed *= PLAYER_SPRINT_MULTIPLIER;
    }

    let desired_horizontal = transform.rotation * (wish.normalize_or_zero() * move_speed);
    let mut desired_velocity = Vec3::new(
        desired_horizontal.x,
        linear_velocity.y,
        desired_horizontal.z,
    );

    if controller.grounded {
        if keys.just_pressed(KeyCode::Space) {
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

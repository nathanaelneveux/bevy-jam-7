mod cave_noise;
mod cave_world;

use bevy::asset::AssetMetaCheck;
use bevy::input::{common_conditions::input_toggle_active, mouse::AccumulatedMouseMotion};
use bevy::prelude::*;
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};
use bevy_voxel_world::prelude::VoxelWorldCamera;

use cave_world::{CaveWorld, CaveWorldPlugin};

const LOOK_SENSITIVITY: f32 = 0.002;
const FLY_SPEED: f32 = 18.0;
const FAST_FLY_SPEED: f32 = 36.0;

#[derive(Component)]
struct FlyCam;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(
            WorldInspectorPlugin::default().run_if(input_toggle_active(false, KeyCode::Escape)),
        )
        .add_plugins(CaveWorldPlugin)
        .add_systems(Startup, setup)
        .add_systems(Update, fly_camera)
        .run();
}

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, 0.0).looking_at(Vec3::new(1.0, 10.0, 1.0), Vec3::Y),
        FlyCam,
        VoxelWorldCamera::<CaveWorld>::default(),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 4_500.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(0.0, 0.0, 0.0).looking_at(Vec3::new(0.3, -1.0, 0.2), Vec3::Y),
    ));

    commands.insert_resource(GlobalAmbientLight {
        color: Color::srgb(0.62, 0.68, 0.78),
        brightness: 260.0,
        affects_lightmapped_meshes: true,
    });
}

fn fly_camera(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse_motion: Res<AccumulatedMouseMotion>,
    mut camera: Query<&mut Transform, With<FlyCam>>,
) {
    let Ok(mut transform) = camera.single_mut() else {
        return;
    };

    let mut motion = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyW) {
        motion.z -= 1.0;
    }
    if keys.pressed(KeyCode::KeyS) {
        motion.z += 1.0;
    }
    if keys.pressed(KeyCode::KeyA) {
        motion.x -= 1.0;
    }
    if keys.pressed(KeyCode::KeyD) {
        motion.x += 1.0;
    }
    if keys.pressed(KeyCode::Space) {
        motion.y += 1.0;
    }
    if keys.pressed(KeyCode::ShiftLeft) {
        motion.y -= 1.0;
    }

    let speed = if keys.pressed(KeyCode::ControlLeft) {
        FAST_FLY_SPEED
    } else {
        FLY_SPEED
    };

    let step = motion.normalize_or_zero() * speed * time.delta_secs();
    let local_x = transform.local_x();
    let local_z = transform.local_z();
    transform.translation += local_x * step.x;
    transform.translation += local_z * step.z;
    transform.translation += Vec3::Y * step.y;

    let look = mouse_motion.delta * -LOOK_SENSITIVITY;
    let (mut yaw, mut pitch, _roll) = transform.rotation.to_euler(EulerRot::YXZ);
    yaw += look.x;
    pitch = (pitch + look.y).clamp(-core::f32::consts::FRAC_PI_2, core::f32::consts::FRAC_PI_2);
    transform.rotation =
        Quat::from_axis_angle(Vec3::Y, yaw) * Quat::from_axis_angle(Vec3::X, pitch);
}

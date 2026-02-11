mod cave_noise;
mod cave_world;
mod chunk_colliders;
mod mob_nav;
mod mob_nav_northstar;
mod player_controller;

use avian3d::prelude::*;
use bevy::asset::AssetMetaCheck;
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, PrimaryWindow};
use bevy_enhanced_input::prelude::EnhancedInputPlugin;
use bevy_inspector_egui::{bevy_egui::EguiPlugin, quick::WorldInspectorPlugin};

use cave_world::CaveWorldPlugin;
use chunk_colliders::ChunkColliderPlugin;
use mob_nav::{
    MobNavAgent, MobNavGoal, MobNavMovementMode, MobNavPlugin, MobNavStatus, MobNavUpdateSet,
};
use mob_nav_northstar::MobNavNorthstarPlugin;
use player_controller::PlayerControllerPlugin;

#[derive(Resource, Default)]
pub(crate) struct InspectorMode {
    pub(crate) enabled: bool,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            meta_check: AssetMetaCheck::Never,
            ..default()
        }))
        .init_resource::<InspectorMode>()
        .add_plugins(EnhancedInputPlugin)
        .add_plugins(PhysicsPlugins::default())
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldInspectorPlugin::default().run_if(inspector_mode_active))
        .add_plugins(CaveWorldPlugin)
        .add_plugins(PlayerControllerPlugin)
        .add_plugins(ChunkColliderPlugin)
        .add_plugins(MobNavPlugin)
        .add_plugins(MobNavNorthstarPlugin)
        .add_systems(Startup, spawn_nav_test_mobs)
        .add_systems(
            Update,
            (
                advance_nav_test_patrols.after(MobNavUpdateSet::ApplyResults),
                sync_nav_goal_markers,
            ),
        )
        .add_systems(Update, toggle_inspector_mode)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut cursor_options: Single<&mut CursorOptions, With<PrimaryWindow>>,
) {
    apply_cursor_mode(&mut cursor_options, false);

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

fn inspector_mode_active(inspector_mode: Res<InspectorMode>) -> bool {
    inspector_mode.enabled
}

fn toggle_inspector_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mut inspector_mode: ResMut<InspectorMode>,
    mut cursor_options: Single<&mut CursorOptions, With<PrimaryWindow>>,
) {
    if keys.just_pressed(KeyCode::Escape) {
        inspector_mode.enabled = !inspector_mode.enabled;
        apply_cursor_mode(&mut cursor_options, inspector_mode.enabled);
    }
}

fn apply_cursor_mode(cursor_options: &mut CursorOptions, inspector_enabled: bool) {
    if inspector_enabled {
        cursor_options.visible = true;
        cursor_options.grab_mode = CursorGrabMode::None;
    } else {
        cursor_options.visible = false;
        cursor_options.grab_mode = CursorGrabMode::Locked;
    }
}

#[derive(Component)]
struct NavTestPatrol {
    points: [Vec3; 2],
    next_target_index: usize,
}

#[derive(Component)]
struct NavGoalMarker {
    tracked_entity: Entity,
}

#[derive(Component)]
struct NavTestGround;

fn spawn_nav_test_mobs(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let ground_points_xz = [Vec2::new(-10.0, -6.0), Vec2::new(10.0, -6.0)];
    let ground_points = ground_points_xz.map(|point| Vec3::new(point.x, 1.5, point.y));
    let flyer_points = [Vec3::new(-12.0, 8.0, 8.0), Vec3::new(12.0, 8.0, 8.0)];

    let ground = commands
        .spawn((
            Name::new("NavTestGround"),
            NavTestGround,
            RigidBody::Kinematic,
            Collider::capsule(0.35, 0.7),
            LinearVelocity::ZERO,
            LockedAxes::ROTATION_LOCKED,
            MobNavAgent {
                movement_mode: MobNavMovementMode::Ground,
                max_speed: 4.0,
                arrival_tolerance: 0.4,
            },
            MobNavGoal {
                position: ground_points[1],
            },
            NavTestPatrol {
                points: ground_points,
                next_target_index: 0,
            },
            Mesh3d(meshes.add(Capsule3d::new(0.35, 0.7))),
            MeshMaterial3d(materials.add(Color::srgb(0.85, 0.35, 0.3))),
            Transform::from_translation(ground_points[0]),
        ))
        .id();

    let flyer = commands
        .spawn((
            Name::new("NavTestFlyer"),
            MobNavAgent {
                movement_mode: MobNavMovementMode::FlyingLineOfSight,
                max_speed: 6.5,
                arrival_tolerance: 0.6,
            },
            MobNavGoal {
                position: flyer_points[1],
            },
            NavTestPatrol {
                points: flyer_points,
                next_target_index: 0,
            },
            Mesh3d(meshes.add(Sphere::new(0.45))),
            MeshMaterial3d(materials.add(Color::srgb(0.25, 0.75, 0.9))),
            Transform::from_translation(flyer_points[0]),
        ))
        .id();

    commands.spawn((
        Name::new("NavGoalMarkerGround"),
        NavGoalMarker {
            tracked_entity: ground,
        },
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.8, 0.2))),
        Transform::from_translation(ground_points[1] + Vec3::Y * 0.2),
    ));

    commands.spawn((
        Name::new("NavGoalMarkerFlyer"),
        NavGoalMarker {
            tracked_entity: flyer,
        },
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 1.0, 0.9))),
        Transform::from_translation(flyer_points[1] + Vec3::Y * 0.2),
    ));
}

fn advance_nav_test_patrols(mut mobs: Query<(&MobNavStatus, &mut MobNavGoal, &mut NavTestPatrol)>) {
    for (status, mut goal, mut patrol) in &mut mobs {
        if !matches!(*status, MobNavStatus::Arrived | MobNavStatus::Blocked) {
            continue;
        }

        let target = patrol.points[patrol.next_target_index];
        if goal.position.distance_squared(target) <= 0.0001 {
            continue;
        }

        goal.position = target;
        patrol.next_target_index = 1 - patrol.next_target_index;
    }
}

fn sync_nav_goal_markers(
    goals: Query<&MobNavGoal>,
    mut markers: Query<(&NavGoalMarker, &mut Transform)>,
) {
    for (marker, mut transform) in &mut markers {
        if let Ok(goal) = goals.get(marker.tracked_entity) {
            transform.translation = goal.position + Vec3::Y * 0.2;
        }
    }
}

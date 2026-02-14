use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use std::f32::consts::PI;
use std::time::Duration;

use crate::mob_nav::{
    MobNavAgent, MobNavGoal, MobNavMovementMode, MobNavRepath, MobNavStatus, MobNavUpdateSet,
};

const NAV_TEST_GROUND_VISUAL_Y_OFFSET: f32 = -0.5;
const NAV_TEST_GROUND_VISUAL_YAW_OFFSET: f32 = PI;
const NAV_TEST_BODY_TURN_SPEED_RAD_PER_SEC: f32 = 5.0;

pub struct NavSandboxPlugin;

impl Plugin for NavSandboxPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_nav_test_mobs).add_systems(
            Update,
            (
                retry_blocked_nav_test_mobs
                    .after(MobNavUpdateSet::ApplyResults)
                    .run_if(on_timer(Duration::from_secs_f32(0.75))),
                advance_nav_test_patrols.after(MobNavUpdateSet::ApplyResults),
                face_nav_test_ground_toward_movement.after(MobNavUpdateSet::ApplyResults),
                sync_nav_goal_markers,
            ),
        );
    }
}

#[derive(Component)]
struct NavTestPatrol {
    points: Vec<Vec3>,
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
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let ground_points_xz = [
        Vec2::new(-10.0, -6.0),
        Vec2::new(6.0, -10.0),
        Vec2::new(6.0, 10.0),
        Vec2::new(10.0, -6.0),
    ];
    let ground_points: Vec<Vec3> = ground_points_xz
        .into_iter()
        .map(|point| Vec3::new(point.x, 1.5, point.y))
        .collect();
    let flyer_points = vec![Vec3::new(-12.0, 8.0, 8.0), Vec3::new(12.0, 8.0, 8.0)];

    let (ground_spawn, ground_goal, ground_next_target_index) =
        patrol_spawn_goal_and_next_index(&ground_points, Vec3::new(0.0, 1.5, 0.0));
    let (flyer_spawn, flyer_goal, flyer_next_target_index) =
        patrol_spawn_goal_and_next_index(&flyer_points, Vec3::new(0.0, 8.0, 0.0));

    let spider_scene: Handle<Scene> = asset_server.load("Spider.glb#Scene0");

    let ground = commands
        .spawn((
            Name::new("NavTestGround"),
            NavTestGround,
            RigidBody::Dynamic,
            Collider::capsule(0.35, 0.7),
            Friction::ZERO.with_combine_rule(CoefficientCombine::Min),
            LinearVelocity::ZERO,
            LockedAxes::ROTATION_LOCKED,
            MobNavAgent {
                movement_mode: MobNavMovementMode::Ground,
                max_speed: 4.0,
                arrival_tolerance: 0.4,
            },
            MobNavGoal {
                position: ground_goal,
            },
            NavTestPatrol {
                points: ground_points,
                next_target_index: ground_next_target_index,
            },
            Transform::from_translation(ground_spawn),
        ))
        .id();
    commands.entity(ground).with_children(|parent| {
        parent.spawn((
            Name::new("NavTestGroundVisual"),
            SceneRoot(spider_scene),
            Transform {
                translation: Vec3::new(0.0, NAV_TEST_GROUND_VISUAL_Y_OFFSET, 0.0),
                rotation: Quat::from_rotation_y(NAV_TEST_GROUND_VISUAL_YAW_OFFSET),
                ..default()
            },
        ));
    });

    let flyer = commands
        .spawn((
            Name::new("NavTestFlyer"),
            MobNavAgent {
                movement_mode: MobNavMovementMode::FlyingLineOfSight,
                max_speed: 6.5,
                arrival_tolerance: 0.6,
            },
            MobNavGoal {
                position: flyer_goal,
            },
            NavTestPatrol {
                points: flyer_points,
                next_target_index: flyer_next_target_index,
            },
            Mesh3d(meshes.add(Sphere::new(0.45))),
            MeshMaterial3d(materials.add(Color::srgb(0.25, 0.75, 0.9))),
            Transform::from_translation(flyer_spawn),
        ))
        .id();

    commands.spawn((
        Name::new("NavGoalMarkerGround"),
        NavGoalMarker {
            tracked_entity: ground,
        },
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.8, 0.2))),
        Transform::from_translation(ground_goal + Vec3::Y * 0.2),
    ));

    commands.spawn((
        Name::new("NavGoalMarkerFlyer"),
        NavGoalMarker {
            tracked_entity: flyer,
        },
        Mesh3d(meshes.add(Cuboid::new(0.3, 0.3, 0.3))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 1.0, 0.9))),
        Transform::from_translation(flyer_goal + Vec3::Y * 0.2),
    ));
}

fn retry_blocked_nav_test_mobs(
    mut commands: Commands,
    mobs: Query<(Entity, &MobNavStatus), With<NavTestGround>>,
) {
    for (entity, status) in &mobs {
        if *status == MobNavStatus::Blocked {
            commands.entity(entity).insert(MobNavRepath);
        }
    }
}

fn advance_nav_test_patrols(mut mobs: Query<(&MobNavStatus, &mut MobNavGoal, &mut NavTestPatrol)>) {
    for (status, mut goal, mut patrol) in &mut mobs {
        if *status != MobNavStatus::Arrived {
            continue;
        }

        if patrol.points.is_empty() {
            continue;
        }

        let target_index = patrol.next_target_index % patrol.points.len();
        let target = patrol.points[target_index];
        if goal.position.distance_squared(target) <= 0.0001 {
            patrol.next_target_index = (target_index + 1) % patrol.points.len();
            continue;
        }

        goal.position = target;
        patrol.next_target_index = (target_index + 1) % patrol.points.len();
    }
}

fn patrol_spawn_goal_and_next_index(points: &[Vec3], fallback: Vec3) -> (Vec3, Vec3, usize) {
    let spawn = points.first().copied().unwrap_or(fallback);
    let goal = points.get(1).copied().unwrap_or(spawn);
    let next_target_index = if points.len() >= 2 {
        2 % points.len()
    } else {
        0
    };
    (spawn, goal, next_target_index)
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

fn face_nav_test_ground_toward_movement(
    time: Res<Time>,
    mut mobs: Query<(&LinearVelocity, &MobNavGoal, &mut Transform), With<NavTestGround>>,
) {
    for (velocity, goal, mut transform) in &mut mobs {
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
        let turn_lerp = (NAV_TEST_BODY_TURN_SPEED_RAD_PER_SEC * time.delta_secs()).clamp(0.0, 1.0);
        transform.rotation = transform.rotation.slerp(target_rotation, turn_lerp);
    }
}

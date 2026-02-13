use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::time::common_conditions::on_timer;
use std::f32::consts::{PI, TAU};
use std::time::Duration;

use crate::mob_nav::{
    MobNavAgent, MobNavGoal, MobNavMovementMode, MobNavRepath, MobNavStatus, MobNavUpdateSet,
};

const NAV_TEST_GROUND_VISUAL_Y_OFFSET: f32 = -1.05;
const NAV_TEST_GROUND_VISUAL_YAW_OFFSET: f32 = PI;
const NAV_TEST_GAIT_BASE_FREQ_HZ: f32 = 2.4;
const NAV_TEST_GAIT_FREQ_PER_SPEED_HZ: f32 = 0.35;
const NAV_TEST_GAIT_SPEED_EPSILON: f32 = 0.1;
const NAV_TEST_GAIT_HIP_SWING: f32 = 0.32;
const NAV_TEST_GAIT_HIP_LIFT: f32 = 0.16;
const NAV_TEST_GAIT_KNEE_BEND: f32 = 0.36;
const NAV_TEST_GAIT_PHASE_GROUP_A: f32 = 0.0;
const NAV_TEST_GAIT_PHASE_GROUP_B: f32 = PI;
const NAV_TEST_IK_FOOT_RAY_ORIGIN_UP: f32 = 0.6;
const NAV_TEST_IK_FOOT_RAY_DISTANCE: f32 = 2.4;
const NAV_TEST_IK_STANCE_CLEARANCE: f32 = 0.05;
const NAV_TEST_IK_SWING_LIFT: f32 = 0.22;
const NAV_TEST_IK_KNEE_GAIN: f32 = 1.6;
const NAV_TEST_IK_KNEE_MAX_DELTA: f32 = 0.55;

pub struct NavSandboxPlugin;

impl Plugin for NavSandboxPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_nav_test_mobs).add_systems(
            Update,
            (
                retry_blocked_nav_test_mobs
                    .after(MobNavUpdateSet::ApplyResults)
                    .run_if(on_timer(Duration::from_secs_f32(0.75))),
                init_nav_test_spider_leg_rig,
                advance_nav_test_patrols.after(MobNavUpdateSet::ApplyResults),
                face_nav_test_ground_toward_movement.after(MobNavUpdateSet::ApplyResults),
                animate_nav_test_spider_legs.after(MobNavUpdateSet::ApplyResults),
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

#[derive(Component)]
struct NavTestGroundVisualRoot {
    owner: Entity,
}

#[derive(Component)]
struct NavTestSpiderRigReady;

#[derive(Component, Clone, Copy)]
struct NavTestSpiderLegBone {
    owner: Entity,
    leg: SpiderLegId,
    side_sign: f32,
    phase_offset: f32,
    joint: SpiderLegJoint,
}

#[derive(Component, Clone, Copy)]
struct NavTestSpiderFootBone {
    owner: Entity,
    leg: SpiderLegId,
    phase_offset: f32,
}

#[derive(Component, Clone, Copy)]
struct NavTestSpiderBindPose {
    local_rotation: Quat,
}

#[derive(Clone, Copy)]
enum SpiderLegJoint {
    Hip,
    Knee,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SpiderLegId {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

#[derive(Clone, Copy)]
enum SpiderRigBoneDefinition {
    Leg {
        leg: SpiderLegId,
        joint: SpiderLegJoint,
        side_sign: f32,
        phase_offset: f32,
    },
    Foot {
        leg: SpiderLegId,
        phase_offset: f32,
    },
}

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
            NavTestGroundVisualRoot { owner: ground },
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

        transform.look_to(direction.normalize(), Vec3::Y);
    }
}

fn init_nav_test_spider_leg_rig(
    mut commands: Commands,
    visual_roots: Query<(Entity, &NavTestGroundVisualRoot), Without<NavTestSpiderRigReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    transforms: Query<&Transform>,
    existing_leg_bones: Query<(), Or<(With<NavTestSpiderLegBone>, With<NavTestSpiderFootBone>)>>,
) {
    for (visual_root, visual_info) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut matched_bones = 0usize;

        while let Some(entity) = stack.pop() {
            if let Ok(children) = children_query.get(entity) {
                for child in children.iter() {
                    stack.push(child);
                }
            }

            let Ok(name) = names.get(entity) else {
                continue;
            };
            let Some(definition) = spider_leg_bone_definition(name.as_str()) else {
                continue;
            };

            if existing_leg_bones.contains(entity) {
                matched_bones += 1;
                continue;
            }

            let Ok(transform) = transforms.get(entity) else {
                continue;
            };

            matched_bones += 1;
            match definition {
                SpiderRigBoneDefinition::Leg {
                    leg,
                    joint,
                    side_sign,
                    phase_offset,
                } => {
                    commands.entity(entity).insert((
                        NavTestSpiderLegBone {
                            owner: visual_info.owner,
                            leg,
                            side_sign,
                            phase_offset,
                            joint,
                        },
                        NavTestSpiderBindPose {
                            local_rotation: transform.rotation,
                        },
                    ));
                }
                SpiderRigBoneDefinition::Foot { leg, phase_offset } => {
                    commands.entity(entity).insert(NavTestSpiderFootBone {
                        owner: visual_info.owner,
                        leg,
                        phase_offset,
                    });
                }
            }
        }

        if matched_bones >= 12 {
            commands.entity(visual_root).insert(NavTestSpiderRigReady);
        }
    }
}

fn animate_nav_test_spider_legs(
    time: Res<Time>,
    spatial_query: SpatialQuery,
    movers: Query<&LinearVelocity, With<NavTestGround>>,
    foot_bones: Query<(&NavTestSpiderFootBone, &GlobalTransform)>,
    mut leg_bones: Query<(&NavTestSpiderLegBone, &NavTestSpiderBindPose, &mut Transform)>,
) {
    let elapsed = time.elapsed_secs();

    for (bone, bind_pose, mut transform) in &mut leg_bones {
        let Ok(velocity) = movers.get(bone.owner) else {
            continue;
        };

        let speed = Vec2::new(velocity.x, velocity.z).length();
        if speed <= NAV_TEST_GAIT_SPEED_EPSILON {
            transform.rotation = bind_pose.local_rotation;
            continue;
        }

        let gait_weight = ((speed - NAV_TEST_GAIT_SPEED_EPSILON) / 2.0).clamp(0.0, 1.0);
        let stride_frequency = NAV_TEST_GAIT_BASE_FREQ_HZ + speed * NAV_TEST_GAIT_FREQ_PER_SPEED_HZ;
        let phase = elapsed * TAU * stride_frequency + bone.phase_offset;
        let delta = match bone.joint {
            SpiderLegJoint::Hip => {
                let swing = phase.sin() * NAV_TEST_GAIT_HIP_SWING * gait_weight;
                let lift = phase.cos().max(0.0) * NAV_TEST_GAIT_HIP_LIFT * gait_weight;
                Quat::from_euler(EulerRot::XYZ, lift, 0.0, bone.side_sign * swing)
            }
            SpiderLegJoint::Knee => {
                let knee_ik_delta = sample_knee_ik_delta(
                    bone.owner,
                    bone.leg,
                    speed,
                    elapsed,
                    &spatial_query,
                    &foot_bones,
                ) * gait_weight;
                let base_bend = ((phase.sin() + 1.0) * 0.5) * NAV_TEST_GAIT_KNEE_BEND * gait_weight;
                let bend =
                    (base_bend + knee_ik_delta).clamp(0.0, NAV_TEST_GAIT_KNEE_BEND + NAV_TEST_IK_KNEE_MAX_DELTA);
                Quat::from_rotation_x(-bend)
            }
        };

        transform.rotation = bind_pose.local_rotation * delta;
    }
}

fn sample_knee_ik_delta(
    owner: Entity,
    leg: SpiderLegId,
    speed: f32,
    elapsed: f32,
    spatial_query: &SpatialQuery,
    foot_bones: &Query<(&NavTestSpiderFootBone, &GlobalTransform)>,
) -> f32 {
    let Some((foot_bone, foot_global_transform)) = foot_bones
        .iter()
        .find(|(foot_bone, _)| foot_bone.owner == owner && foot_bone.leg == leg)
    else {
        return 0.0;
    };

    let stride_frequency = NAV_TEST_GAIT_BASE_FREQ_HZ + speed * NAV_TEST_GAIT_FREQ_PER_SPEED_HZ;
    let phase = elapsed * TAU * stride_frequency + foot_bone.phase_offset;
    let swing_lift = phase.sin().max(0.0) * NAV_TEST_IK_SWING_LIFT;

    let foot_world = foot_global_transform.translation();
    let ray_origin = foot_world + Vec3::Y * NAV_TEST_IK_FOOT_RAY_ORIGIN_UP;
    let filter = SpatialQueryFilter::from_excluded_entities([owner]);
    let Some(hit) = spatial_query.cast_ray(
        ray_origin,
        Dir3::NEG_Y,
        NAV_TEST_IK_FOOT_RAY_DISTANCE,
        true,
        &filter,
    ) else {
        return 0.0;
    };

    let ground_height = ray_origin.y - hit.distance;
    let current_clearance = foot_world.y - ground_height;
    let target_clearance = NAV_TEST_IK_STANCE_CLEARANCE + swing_lift;
    let clearance_error = current_clearance - target_clearance;

    (-clearance_error * NAV_TEST_IK_KNEE_GAIN).clamp(-NAV_TEST_IK_KNEE_MAX_DELTA, NAV_TEST_IK_KNEE_MAX_DELTA)
}

fn spider_leg_bone_definition(name: &str) -> Option<SpiderRigBoneDefinition> {
    match name {
        "HipFront.L" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::FrontLeft,
            joint: SpiderLegJoint::Hip,
            side_sign: -1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        "KneeFront.L" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::FrontLeft,
            joint: SpiderLegJoint::Knee,
            side_sign: -1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        "FootFront.L" => Some(SpiderRigBoneDefinition::Foot {
            leg: SpiderLegId::FrontLeft,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        "HipFront.R" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::FrontRight,
            joint: SpiderLegJoint::Hip,
            side_sign: 1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "KneeFront.R" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::FrontRight,
            joint: SpiderLegJoint::Knee,
            side_sign: 1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "FootFront.R" => Some(SpiderRigBoneDefinition::Foot {
            leg: SpiderLegId::FrontRight,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "HipRear.L" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::RearLeft,
            joint: SpiderLegJoint::Hip,
            side_sign: -1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "KneeRear.L" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::RearLeft,
            joint: SpiderLegJoint::Knee,
            side_sign: -1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "FootRear.L" => Some(SpiderRigBoneDefinition::Foot {
            leg: SpiderLegId::RearLeft,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_B,
        }),
        "HipRear.R" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::RearRight,
            joint: SpiderLegJoint::Hip,
            side_sign: 1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        "KneeRear.R" => Some(SpiderRigBoneDefinition::Leg {
            leg: SpiderLegId::RearRight,
            joint: SpiderLegJoint::Knee,
            side_sign: 1.0,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        "FootRear.R" => Some(SpiderRigBoneDefinition::Foot {
            leg: SpiderLegId::RearRight,
            phase_offset: NAV_TEST_GAIT_PHASE_GROUP_A,
        }),
        _ => None,
    }
}

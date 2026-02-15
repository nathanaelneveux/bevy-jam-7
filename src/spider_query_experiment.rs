//! Experimental spider spawner and spider-specific grounded IK rig init.

use bevy::prelude::*;
use std::f32::consts::PI;

use crate::ground_ik::{GroundIkSet, GroundedTwoBoneIkOwner, GroundedTwoBoneIkRig};

const EXPERIMENT_MODEL_X: f32 = -2.0;
const EXPERIMENT_MODEL_Y: f32 = -5.0;
const EXPERIMENT_VISUAL_Y_OFFSET: f32 = -0.5;
const EXPERIMENT_VISUAL_YAW_OFFSET: f32 = PI;
const SPIDER_RIG_EPSILON: f32 = 0.0001;

pub struct SpiderQueryExperimentPlugin;

impl Plugin for SpiderQueryExperimentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_spider_query_experiment)
            .add_systems(Update, init_spider_leg_rig.before(GroundIkSet::Solve));
    }
}

#[derive(Component)]
pub(crate) struct ExperimentSpiderVisualRoot {
    pub owner: Entity,
}

#[derive(Component)]
struct ExperimentRigReady;

#[derive(Clone, Copy)]
enum SpiderLegJoint {
    Hip,
    Knee,
    Foot,
}

#[derive(Clone, Copy)]
struct SpiderRigBoneDefinition {
    leg: SpiderLegId,
    joint: SpiderLegJoint,
    side_sign: f32,
    fore_sign: f32,
}

#[derive(Clone, Copy, Default)]
struct SpiderRigLegMatch {
    hip: Option<Entity>,
    knee: Option<Entity>,
    foot: Option<Entity>,
    side_sign: f32,
    fore_sign: f32,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum SpiderLegId {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

fn spawn_spider_query_experiment(mut commands: Commands, asset_server: Res<AssetServer>) {
    let spider_scene: Handle<Scene> = asset_server.load("Spider.glb#Scene0");

    let spider = commands
        .spawn((
            Name::new("SpiderQueryExperiment"),
            GroundedTwoBoneIkOwner,
            Transform::from_xyz(EXPERIMENT_MODEL_X, EXPERIMENT_MODEL_Y, 0.0),
        ))
        .id();

    commands.entity(spider).with_children(|parent| {
        parent.spawn((
            Name::new("SpiderQueryExperimentVisual"),
            ExperimentSpiderVisualRoot { owner: spider },
            SceneRoot(spider_scene),
            Transform {
                translation: Vec3::new(0.0, EXPERIMENT_VISUAL_Y_OFFSET, 0.0),
                rotation: Quat::from_rotation_y(EXPERIMENT_VISUAL_YAW_OFFSET),
                ..default()
            },
        ));
    });
}

fn init_spider_leg_rig(
    mut commands: Commands,
    visual_roots: Query<(Entity, &ExperimentSpiderVisualRoot), Without<ExperimentRigReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    local_transforms: Query<&Transform>,
    global_transforms: Query<&GlobalTransform>,
    existing_leg_rigs: Query<(), With<GroundedTwoBoneIkRig>>,
) {
    for (visual_root, visual_info) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut leg_matches = [SpiderRigLegMatch::default(); 4];
        let Ok(owner_global_transform) = global_transforms.get(visual_info.owner) else {
            continue;
        };
        let owner_inverse_affine = owner_global_transform.affine().inverse();

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

            let leg_slot = &mut leg_matches[spider_leg_index(definition.leg)];
            match definition.joint {
                SpiderLegJoint::Hip => {
                    leg_slot.hip = Some(entity);
                    leg_slot.side_sign = definition.side_sign;
                    leg_slot.fore_sign = definition.fore_sign;
                }
                SpiderLegJoint::Knee => leg_slot.knee = Some(entity),
                SpiderLegJoint::Foot => leg_slot.foot = Some(entity),
            }
        }

        let mut complete_leg_count = 0usize;
        for (index, rig_match) in leg_matches.iter().enumerate() {
            let (Some(hip), Some(knee), Some(foot)) =
                (rig_match.hip, rig_match.knee, rig_match.foot)
            else {
                continue;
            };
            complete_leg_count += 1;

            if existing_leg_rigs.contains(hip) {
                continue;
            }

            let Ok(hip_local_transform) = local_transforms.get(hip) else {
                continue;
            };
            let Ok(knee_local_transform) = local_transforms.get(knee) else {
                continue;
            };
            let Ok(foot_local_transform) = local_transforms.get(foot) else {
                continue;
            };
            let Ok(hip_global_transform) = global_transforms.get(hip) else {
                continue;
            };
            let Ok(knee_global_transform) = global_transforms.get(knee) else {
                continue;
            };
            let Ok(foot_global_transform) = global_transforms.get(foot) else {
                continue;
            };

            let upper_len = hip_global_transform
                .translation()
                .distance(knee_global_transform.translation())
                .max(SPIDER_RIG_EPSILON);
            let lower_len = knee_global_transform
                .translation()
                .distance(foot_global_transform.translation())
                .max(SPIDER_RIG_EPSILON);

            let hip_to_knee_local = safe_normalize(knee_local_transform.translation, Vec3::Y);
            let knee_to_foot_local = safe_normalize(foot_local_transform.translation, Vec3::Y);
            let hip_rest_dir_parent_space = safe_normalize(
                hip_local_transform.rotation * hip_to_knee_local,
                Vec3::NEG_Y,
            );
            let knee_rest_dir_parent_space = safe_normalize(
                knee_local_transform.rotation * knee_to_foot_local,
                Vec3::NEG_Y,
            );
            let foot_rest_owner_space =
                owner_inverse_affine.transform_point3(foot_global_transform.translation());
            let leg = spider_leg_from_index(index);

            commands.entity(hip).insert(GroundedTwoBoneIkRig {
                owner: visual_info.owner,
                debug_color: spider_leg_debug_color(leg),
                side_sign: rig_match.side_sign,
                fore_sign: rig_match.fore_sign,
                hip,
                knee,
                foot,
                upper_len,
                lower_len,
                hip_bind_rotation: hip_local_transform.rotation,
                knee_bind_rotation: knee_local_transform.rotation,
                hip_rest_dir_parent_space,
                knee_rest_dir_parent_space,
                foot_rest_owner_space,
            });
        }

        if complete_leg_count == 4 {
            commands.entity(visual_root).insert(ExperimentRigReady);
        }
    }
}

fn safe_normalize(input: Vec3, fallback: Vec3) -> Vec3 {
    let normalized = input.normalize_or_zero();
    if normalized.length_squared() > SPIDER_RIG_EPSILON {
        normalized
    } else {
        fallback.normalize_or_zero()
    }
}

fn spider_leg_index(leg: SpiderLegId) -> usize {
    match leg {
        SpiderLegId::FrontLeft => 0,
        SpiderLegId::FrontRight => 1,
        SpiderLegId::RearLeft => 2,
        SpiderLegId::RearRight => 3,
    }
}

fn spider_leg_from_index(index: usize) -> SpiderLegId {
    match index {
        0 => SpiderLegId::FrontLeft,
        1 => SpiderLegId::FrontRight,
        2 => SpiderLegId::RearLeft,
        _ => SpiderLegId::RearRight,
    }
}

fn spider_leg_bone_definition(name: &str) -> Option<SpiderRigBoneDefinition> {
    match name {
        "HipFront.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontLeft,
            joint: SpiderLegJoint::Hip,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "KneeFront.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontLeft,
            joint: SpiderLegJoint::Knee,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "FootFront.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontLeft,
            joint: SpiderLegJoint::Foot,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "HipFront.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontRight,
            joint: SpiderLegJoint::Hip,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "KneeFront.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontRight,
            joint: SpiderLegJoint::Knee,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "FootFront.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::FrontRight,
            joint: SpiderLegJoint::Foot,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "HipRear.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearLeft,
            joint: SpiderLegJoint::Hip,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "KneeRear.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearLeft,
            joint: SpiderLegJoint::Knee,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "FootRear.L" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearLeft,
            joint: SpiderLegJoint::Foot,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "HipRear.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearRight,
            joint: SpiderLegJoint::Hip,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        "KneeRear.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearRight,
            joint: SpiderLegJoint::Knee,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        "FootRear.R" => Some(SpiderRigBoneDefinition {
            leg: SpiderLegId::RearRight,
            joint: SpiderLegJoint::Foot,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        _ => None,
    }
}

fn spider_leg_debug_color(leg: SpiderLegId) -> Color {
    match leg {
        SpiderLegId::FrontLeft => Color::srgb(1.0, 0.35, 0.35),
        SpiderLegId::FrontRight => Color::srgb(0.35, 1.0, 0.35),
        SpiderLegId::RearLeft => Color::srgb(0.35, 0.6, 1.0),
        SpiderLegId::RearRight => Color::srgb(1.0, 0.8, 0.3),
    }
}

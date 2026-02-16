use bevy::prelude::*;

use crate::ground_ik::{GroundedTwoBoneIkLegInit, GroundedTwoBoneIkRig, init_leg_rigs};

#[derive(Component)]
pub(crate) struct SpiderVisualRoot {
    pub(crate) owner: Entity,
}

#[derive(Component)]
pub(crate) struct SpiderRigReady;

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

pub(crate) fn init_spider_leg_rig(
    mut commands: Commands,
    visual_roots: Query<(Entity, &SpiderVisualRoot), Without<SpiderRigReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    local_transforms: Query<&Transform>,
    global_transforms: Query<&GlobalTransform>,
    existing_leg_rigs: Query<(), With<GroundedTwoBoneIkRig>>,
) {
    for (visual_root, visual_info) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut leg_matches = [SpiderRigLegMatch::default(); 4];

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
        let mut leg_inits = Vec::new();
        for (index, rig_match) in leg_matches.iter().enumerate() {
            let (Some(hip), Some(knee), Some(foot)) =
                (rig_match.hip, rig_match.knee, rig_match.foot)
            else {
                continue;
            };
            complete_leg_count += 1;
            let leg = spider_leg_from_index(index);
            leg_inits.push(GroundedTwoBoneIkLegInit {
                debug_color: spider_leg_debug_color(leg),
                side_sign: rig_match.side_sign,
                fore_sign: rig_match.fore_sign,
                hip,
                knee,
                foot,
            });
        }

        init_leg_rigs(
            &mut commands,
            visual_info.owner,
            leg_inits,
            &local_transforms,
            &global_transforms,
            &existing_leg_rigs,
        );

        if complete_leg_count == 4 {
            commands.entity(visual_root).insert(SpiderRigReady);
        }
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

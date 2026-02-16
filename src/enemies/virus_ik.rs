use bevy::prelude::*;

use crate::ground_ik::{GroundedTwoBoneIkLegInit, GroundedTwoBoneIkRig, init_leg_rigs};

#[derive(Component)]
pub(crate) struct VirusIkVisualRoot {
    pub(crate) owner: Entity,
}

#[derive(Component)]
pub(crate) struct VirusIkRigReady;

#[derive(Clone, Copy)]
enum VirusLegJoint {
    Hip,
    Knee,
    Foot,
}

#[derive(Clone, Copy)]
struct VirusRigBoneDefinition {
    leg: VirusLegId,
    joint: VirusLegJoint,
    side_sign: f32,
    fore_sign: f32,
}

#[derive(Clone, Copy, Default)]
struct VirusRigLegMatch {
    hip: Option<Entity>,
    knee: Option<Entity>,
    foot: Option<Entity>,
    side_sign: f32,
    fore_sign: f32,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum VirusLegId {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
}

pub(crate) fn init_virus_leg_rig(
    mut commands: Commands,
    visual_roots: Query<(Entity, &VirusIkVisualRoot), Without<VirusIkRigReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    local_transforms: Query<&Transform>,
    global_transforms: Query<&GlobalTransform>,
    existing_leg_rigs: Query<(), With<GroundedTwoBoneIkRig>>,
) {
    for (visual_root, visual_info) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut leg_matches = [VirusRigLegMatch::default(); 4];

        while let Some(entity) = stack.pop() {
            if let Ok(children) = children_query.get(entity) {
                for child in children.iter() {
                    stack.push(child);
                }
            }

            let Ok(name) = names.get(entity) else {
                continue;
            };
            let Some(definition) = virus_leg_bone_definition(name.as_str()) else {
                continue;
            };

            let leg_slot = &mut leg_matches[virus_leg_index(definition.leg)];
            match definition.joint {
                VirusLegJoint::Hip => {
                    leg_slot.hip = Some(entity);
                    leg_slot.side_sign = definition.side_sign;
                    leg_slot.fore_sign = definition.fore_sign;
                }
                VirusLegJoint::Knee => leg_slot.knee = Some(entity),
                VirusLegJoint::Foot => leg_slot.foot = Some(entity),
            }
        }

        // virus.glb has no explicit Foot* bones in this rig; infer endpoint from knee descendants.
        for rig_match in &mut leg_matches {
            if rig_match.foot.is_none()
                && let Some(knee) = rig_match.knee
            {
                rig_match.foot = find_leg_end_fallback(knee, &children_query, &local_transforms);
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
            let leg = virus_leg_from_index(index);
            leg_inits.push(GroundedTwoBoneIkLegInit {
                debug_color: virus_leg_debug_color(leg),
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
            commands.entity(visual_root).insert(VirusIkRigReady);
        }
    }
}

fn virus_leg_index(leg: VirusLegId) -> usize {
    match leg {
        VirusLegId::FrontLeft => 0,
        VirusLegId::FrontRight => 1,
        VirusLegId::RearLeft => 2,
        VirusLegId::RearRight => 3,
    }
}

fn virus_leg_from_index(index: usize) -> VirusLegId {
    match index {
        0 => VirusLegId::FrontLeft,
        1 => VirusLegId::FrontRight,
        2 => VirusLegId::RearLeft,
        _ => VirusLegId::RearRight,
    }
}

fn virus_leg_bone_definition(name: &str) -> Option<VirusRigBoneDefinition> {
    match name {
        "HipFront.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontLeft,
            joint: VirusLegJoint::Hip,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "KneeFront.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontLeft,
            joint: VirusLegJoint::Knee,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "FootFront.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontLeft,
            joint: VirusLegJoint::Foot,
            side_sign: -1.0,
            fore_sign: 1.0,
        }),
        "HipFront.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontRight,
            joint: VirusLegJoint::Hip,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "KneeFront.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontRight,
            joint: VirusLegJoint::Knee,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "FootFront.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::FrontRight,
            joint: VirusLegJoint::Foot,
            side_sign: 1.0,
            fore_sign: 1.0,
        }),
        "HipBack.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearLeft,
            joint: VirusLegJoint::Hip,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "KneeBack.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearLeft,
            joint: VirusLegJoint::Knee,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "FootBack.L" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearLeft,
            joint: VirusLegJoint::Foot,
            side_sign: -1.0,
            fore_sign: -1.0,
        }),
        "HipBack.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearRight,
            joint: VirusLegJoint::Hip,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        "KneeBack.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearRight,
            joint: VirusLegJoint::Knee,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        "FootBack.R" => Some(VirusRigBoneDefinition {
            leg: VirusLegId::RearRight,
            joint: VirusLegJoint::Foot,
            side_sign: 1.0,
            fore_sign: -1.0,
        }),
        _ => None,
    }
}

fn virus_leg_debug_color(leg: VirusLegId) -> Color {
    match leg {
        VirusLegId::FrontLeft => Color::srgb(0.95, 0.3, 0.35),
        VirusLegId::FrontRight => Color::srgb(0.35, 0.95, 0.55),
        VirusLegId::RearLeft => Color::srgb(0.35, 0.55, 0.95),
        VirusLegId::RearRight => Color::srgb(0.95, 0.75, 0.25),
    }
}

fn find_leg_end_fallback(
    knee: Entity,
    children_query: &Query<&Children>,
    local_transforms: &Query<&Transform>,
) -> Option<Entity> {
    let mut best: Option<(f32, Entity)> = None;
    let mut stack = vec![knee];

    while let Some(entity) = stack.pop() {
        if let Ok(children) = children_query.get(entity) {
            for child in children.iter() {
                stack.push(child);
            }
        }

        if entity == knee {
            continue;
        }

        let Ok(local_transform) = local_transforms.get(entity) else {
            continue;
        };
        let dist2 = local_transform.translation.length_squared();
        if best.is_none_or(|(best_dist2, _)| dist2 > best_dist2) {
            best = Some((dist2, entity));
        }
    }

    best.map(|(_, entity)| entity)
}

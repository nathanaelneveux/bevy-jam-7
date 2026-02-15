//! Procedural foot planting/stepping for grounded two-bone IK rigs.

use avian3d::prelude::*;
use bevy::prelude::*;
use std::f32::consts::PI;

use crate::ground_ik::{
    GroundIkSet, GroundedTwoBoneIkOwner, GroundedTwoBoneIkRig, GroundedTwoBoneIkSettings,
    GroundedTwoBoneIkTargetOverride, sample_ground_target,
};

const WALK_DEFAULT_STEP_DURATION: f32 = 0.22;
const WALK_DEFAULT_STEP_HEIGHT: f32 = 1.0;
const WALK_DEFAULT_MAX_PLANT_DISTANCE: f32 = 0.7;
const WALK_DEFAULT_MIN_STEP_INTERVAL: f32 = 0.12;

const WALK_MIN_STEP_DURATION: f32 = 0.02;
const WALK_MIN_STEP_HEIGHT: f32 = 0.0;
const WALK_MIN_MAX_PLANT_DISTANCE: f32 = 0.05;
const WALK_MIN_STEP_INTERVAL: f32 = 0.0;

pub(crate) struct GroundIkWalkPlugin;

impl Plugin for GroundIkWalkPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<GroundedTwoBoneIkWalkSettings>()
            .configure_sets(
                Update,
                (GroundIkWalkSet::Sanitize, GroundIkWalkSet::UpdateTargets).chain(),
            )
            .add_systems(
                Update,
                sanitize_grounded_two_bone_ik_walk_settings.in_set(GroundIkWalkSet::Sanitize),
            )
            .add_systems(
                Update,
                update_grounded_two_bone_ik_walk_targets
                    .in_set(GroundIkWalkSet::UpdateTargets)
                    .after(GroundIkSet::Sanitize)
                    .before(GroundIkSet::Solve),
            );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum GroundIkWalkSet {
    Sanitize,
    UpdateTargets,
}

#[derive(Component)]
#[require(GroundedTwoBoneIkWalkSettings)]
/// Opt-in marker for procedural foot planting/stepping.
pub(crate) struct GroundedTwoBoneIkWalk;

#[derive(Component, Reflect, Clone, Copy, Debug)]
#[reflect(Component)]
/// Per-owner procedural walk tuning.
pub(crate) struct GroundedTwoBoneIkWalkSettings {
    /// Enable/disable procedural walk target overrides for this owner.
    enabled: bool,
    /// Duration of one foot swing phase in seconds.
    step_duration: f32,
    /// Vertical lift applied during a step arc.
    step_height: f32,
    /// Distance from planted target to freshly sampled ground target required to trigger a step.
    max_plant_distance: f32,
    /// Cooldown between completed steps for the same leg.
    min_step_interval: f32,
}

impl Default for GroundedTwoBoneIkWalkSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            step_duration: WALK_DEFAULT_STEP_DURATION,
            step_height: WALK_DEFAULT_STEP_HEIGHT,
            max_plant_distance: WALK_DEFAULT_MAX_PLANT_DISTANCE,
            min_step_interval: WALK_DEFAULT_MIN_STEP_INTERVAL,
        }
    }
}

impl GroundedTwoBoneIkWalkSettings {
    fn sanitize(self) -> Self {
        Self {
            enabled: self.enabled,
            step_duration: sanitize_f32(self.step_duration, WALK_DEFAULT_STEP_DURATION, WALK_MIN_STEP_DURATION, 3.0),
            step_height: sanitize_f32(self.step_height, WALK_DEFAULT_STEP_HEIGHT, WALK_MIN_STEP_HEIGHT, 3.0),
            max_plant_distance: sanitize_f32(
                self.max_plant_distance,
                WALK_DEFAULT_MAX_PLANT_DISTANCE,
                WALK_MIN_MAX_PLANT_DISTANCE,
                10.0,
            ),
            min_step_interval: sanitize_f32(
                self.min_step_interval,
                WALK_DEFAULT_MIN_STEP_INTERVAL,
                WALK_MIN_STEP_INTERVAL,
                2.0,
            ),
        }
    }
}

#[derive(Component, Clone, Copy, Debug)]
/// Optional per-leg phase override for initial stagger.
///
/// Value is interpreted as `[0..1)` fraction of `min_step_interval`.
pub(crate) struct GroundedTwoBoneIkWalkLegPhase {
    pub(crate) phase_offset: f32,
}

#[derive(Component, Clone, Copy, Debug)]
struct GroundedTwoBoneIkWalkLegState {
    planted_target: Vec3,
    step_start: Vec3,
    step_end: Vec3,
    step_elapsed: f32,
    cooldown_remaining: f32,
    stepping: bool,
}

fn sanitize_grounded_two_bone_ik_walk_settings(
    mut owners: Query<&mut GroundedTwoBoneIkWalkSettings, With<GroundedTwoBoneIkWalk>>,
) {
    for mut settings in &mut owners {
        *settings = settings.sanitize();
    }
}

#[allow(clippy::type_complexity)]
fn update_grounded_two_bone_ik_walk_targets(
    mut commands: Commands,
    time: Res<Time>,
    spatial_query: SpatialQuery,
    owners: Query<
        (
            &GroundedTwoBoneIkSettings,
            &GroundedTwoBoneIkWalkSettings,
            &GlobalTransform,
        ),
        (With<GroundedTwoBoneIkOwner>, With<GroundedTwoBoneIkWalk>),
    >,
    mut rigs: Query<
        (
            Entity,
            &GroundedTwoBoneIkRig,
            Option<&GroundedTwoBoneIkWalkLegPhase>,
            Option<&mut GroundedTwoBoneIkWalkLegState>,
            Option<&mut GroundedTwoBoneIkTargetOverride>,
        ),
    >,
) {
    let dt = time.delta_secs();

    for (rig_entity, rig, leg_phase_override, leg_state, target_override) in &mut rigs {
        let Ok((ik_settings, walk_settings, owner_global_transform)) = owners.get(rig.owner) else {
            commands
                .entity(rig_entity)
                .remove::<(GroundedTwoBoneIkWalkLegState, GroundedTwoBoneIkTargetOverride)>();
            continue;
        };

        let walk_settings = walk_settings.sanitize();
        if !walk_settings.enabled {
            commands
                .entity(rig_entity)
                .remove::<(GroundedTwoBoneIkWalkLegState, GroundedTwoBoneIkTargetOverride)>();
            continue;
        }

        let Some(sampled_target) = sample_ground_target(
            &spatial_query,
            rig.owner,
            owner_global_transform,
            rig,
            ik_settings,
        ) else {
            commands
                .entity(rig_entity)
                .remove::<(GroundedTwoBoneIkWalkLegState, GroundedTwoBoneIkTargetOverride)>();
            continue;
        };

        if let Some(mut leg_state) = leg_state {
            let effective_target =
                tick_walk_leg_state(&mut leg_state, sampled_target, walk_settings, dt);
            if let Some(mut target_override) = target_override {
                target_override.world_target = effective_target;
            } else {
                commands.entity(rig_entity).insert(GroundedTwoBoneIkTargetOverride {
                    world_target: effective_target,
                });
            }
            continue;
        }

        let phase = leg_phase_override
            .map(|phase| phase.phase_offset)
            .unwrap_or_else(|| default_phase_offset(rig))
            .rem_euclid(1.0);
        let state = GroundedTwoBoneIkWalkLegState {
            planted_target: sampled_target,
            step_start: sampled_target,
            step_end: sampled_target,
            step_elapsed: walk_settings.step_duration,
            cooldown_remaining: walk_settings.min_step_interval * phase,
            stepping: false,
        };

        commands.entity(rig_entity).insert(state);
        if let Some(mut target_override) = target_override {
            target_override.world_target = sampled_target;
        } else {
            commands.entity(rig_entity).insert(GroundedTwoBoneIkTargetOverride {
                world_target: sampled_target,
            });
        }
    }
}

fn tick_walk_leg_state(
    state: &mut GroundedTwoBoneIkWalkLegState,
    sampled_target: Vec3,
    settings: GroundedTwoBoneIkWalkSettings,
    dt: f32,
) -> Vec3 {
    state.cooldown_remaining = (state.cooldown_remaining - dt).max(0.0);

    if !state.stepping {
        let plant_error = state.planted_target.distance(sampled_target);
        if plant_error >= settings.max_plant_distance && state.cooldown_remaining <= 0.0 {
            state.stepping = true;
            state.step_elapsed = 0.0;
            state.step_start = state.planted_target;
            state.step_end = sampled_target;
        }
    }

    if !state.stepping {
        return state.planted_target;
    }

    state.step_elapsed += dt;
    let t = (state.step_elapsed / settings.step_duration).clamp(0.0, 1.0);
    let eased_t = smoothstep(t);
    let base_target = state.step_start.lerp(state.step_end, eased_t);
    let lift = (eased_t * PI).sin().max(0.0) * settings.step_height;
    let stepped_target = base_target + Vec3::Y * lift;

    if t >= 1.0 {
        state.stepping = false;
        state.planted_target = state.step_end;
        state.cooldown_remaining = settings.min_step_interval;
    }

    stepped_target
}

fn default_phase_offset(rig: &GroundedTwoBoneIkRig) -> f32 {
    if rig.side_sign.signum() == rig.fore_sign.signum() {
        0.5
    } else {
        0.0
    }
}

fn smoothstep(t: f32) -> f32 {
    t * t * (3.0 - 2.0 * t)
}

fn sanitize_f32(value: f32, fallback: f32, min_value: f32, max_value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(min_value, max_value)
    } else {
        fallback
    }
}

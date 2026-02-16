//! Procedural foot planting/stepping for grounded two-bone IK rigs.

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::ground_ik::{
    GroundIkSet, GroundedTwoBoneIkOwner, GroundedTwoBoneIkRig, GroundedTwoBoneIkSettings,
    GroundedTwoBoneIkTargetOverride, sample_ground_target,
    sample_ground_target_with_world_offset,
};

const WALK_DEFAULT_STEP_DURATION: f32 = 0.22;
const WALK_DEFAULT_STEP_HEIGHT: f32 = 1.0;
const WALK_DEFAULT_MAX_PLANT_DISTANCE: f32 = 0.7;
const WALK_DEFAULT_MIN_STEP_INTERVAL: f32 = 0.12;
const WALK_DEFAULT_GAIT_CYCLE_DURATION: f32 = 0.5;
const WALK_DEFAULT_PHASE_START_WINDOW: f32 = 0.22;
const WALK_DEFAULT_STEP_FORWARD_DISTANCE: f32 = 0.25;
const WALK_DEFAULT_STEP_FORWARD_SPEED_SCALE: f32 = 0.08;
const WALK_DEFAULT_STEP_FORWARD_MAX_DISTANCE: f32 = 1.25;

const WALK_MIN_STEP_DURATION: f32 = 0.02;
const WALK_MIN_STEP_HEIGHT: f32 = 0.0;
const WALK_MIN_MAX_PLANT_DISTANCE: f32 = 0.05;
const WALK_MIN_STEP_INTERVAL: f32 = 0.0;
const WALK_MIN_GAIT_CYCLE_DURATION: f32 = 0.05;
const WALK_MIN_PHASE_START_WINDOW: f32 = 0.01;
const WALK_MIN_STEP_FORWARD_DISTANCE: f32 = 0.0;
const WALK_MIN_STEP_FORWARD_SPEED_SCALE: f32 = 0.0;
const WALK_MIN_STEP_FORWARD_MAX_DISTANCE: f32 = 0.0;

const WALK_MIN_STEP_DURATION_SCALE: f32 = 0.35;
const WALK_PHASE_BREAK_DISTANCE_MULTIPLIER: f32 = 1.75;
const WALK_EMERGENCY_REPLANT_MULTIPLIER: f32 = 2.5;
const WALK_MIN_TRAVEL_SPEED_FOR_FORWARD: f32 = 0.05;

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
                advance_grounded_two_bone_ik_walk_owner_state
                    .in_set(GroundIkWalkSet::UpdateTargets)
                    .after(GroundIkWalkSet::Sanitize)
                    .after(GroundIkSet::Sanitize)
                    .before(update_grounded_two_bone_ik_walk_targets),
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

#[derive(Component, Reflect, Clone, Copy, Debug, PartialEq)]
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
    /// Global gait cycle duration in seconds for phase scheduling.
    gait_cycle_duration: f32,
    /// Fractional phase window where a leg is allowed to start stepping.
    phase_start_window: f32,
    /// Base landing lead distance in movement direction.
    step_forward_distance: f32,
    /// Extra lead distance per unit owner speed.
    step_forward_speed_scale: f32,
    /// Maximum total lead distance for forward landing placement.
    step_forward_max_distance: f32,
}

impl Default for GroundedTwoBoneIkWalkSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            step_duration: WALK_DEFAULT_STEP_DURATION,
            step_height: WALK_DEFAULT_STEP_HEIGHT,
            max_plant_distance: WALK_DEFAULT_MAX_PLANT_DISTANCE,
            min_step_interval: WALK_DEFAULT_MIN_STEP_INTERVAL,
            gait_cycle_duration: WALK_DEFAULT_GAIT_CYCLE_DURATION,
            phase_start_window: WALK_DEFAULT_PHASE_START_WINDOW,
            step_forward_distance: WALK_DEFAULT_STEP_FORWARD_DISTANCE,
            step_forward_speed_scale: WALK_DEFAULT_STEP_FORWARD_SPEED_SCALE,
            step_forward_max_distance: WALK_DEFAULT_STEP_FORWARD_MAX_DISTANCE,
        }
    }
}

impl GroundedTwoBoneIkWalkSettings {
    fn sanitize(self) -> Self {
        Self {
            enabled: self.enabled,
            step_duration: sanitize_f32(
                self.step_duration,
                WALK_DEFAULT_STEP_DURATION,
                WALK_MIN_STEP_DURATION,
                3.0,
            ),
            step_height: sanitize_f32(
                self.step_height,
                WALK_DEFAULT_STEP_HEIGHT,
                WALK_MIN_STEP_HEIGHT,
                3.0,
            ),
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
            gait_cycle_duration: sanitize_f32(
                self.gait_cycle_duration,
                WALK_DEFAULT_GAIT_CYCLE_DURATION,
                WALK_MIN_GAIT_CYCLE_DURATION,
                5.0,
            ),
            phase_start_window: sanitize_f32(
                self.phase_start_window,
                WALK_DEFAULT_PHASE_START_WINDOW,
                WALK_MIN_PHASE_START_WINDOW,
                0.99,
            ),
            step_forward_distance: sanitize_f32(
                self.step_forward_distance,
                WALK_DEFAULT_STEP_FORWARD_DISTANCE,
                WALK_MIN_STEP_FORWARD_DISTANCE,
                5.0,
            ),
            step_forward_speed_scale: sanitize_f32(
                self.step_forward_speed_scale,
                WALK_DEFAULT_STEP_FORWARD_SPEED_SCALE,
                WALK_MIN_STEP_FORWARD_SPEED_SCALE,
                2.0,
            ),
            step_forward_max_distance: sanitize_f32(
                self.step_forward_max_distance,
                WALK_DEFAULT_STEP_FORWARD_MAX_DISTANCE,
                WALK_MIN_STEP_FORWARD_MAX_DISTANCE,
                10.0,
            ),
        }
    }
}

#[derive(Component, Clone, Copy, Debug)]
/// Optional per-leg phase override for initial stagger.
///
/// Value is interpreted as normalized `[0..1)` gait phase.
pub(crate) struct GroundedTwoBoneIkWalkLegPhase {
    pub(crate) phase_offset: f32,
}

#[derive(Component, Clone, Copy, Debug)]
struct GroundedTwoBoneIkWalkLegState {
    planted_target: Vec3,
    step_start: Vec3,
    step_end: Vec3,
    step_elapsed: f32,
    step_duration: f32,
    cooldown_remaining: f32,
    stepping: bool,
}

#[derive(Component, Clone, Copy, Debug)]
struct GroundedTwoBoneIkWalkOwnerState {
    gait_elapsed: f32,
    last_position: Vec3,
    travel_direction: Vec3,
    travel_speed: f32,
}

fn sanitize_grounded_two_bone_ik_walk_settings(
    mut owners: Query<
        &mut GroundedTwoBoneIkWalkSettings,
        (With<GroundedTwoBoneIkWalk>, Changed<GroundedTwoBoneIkWalkSettings>),
    >,
) {
    for mut settings in &mut owners {
        let sanitized = settings.sanitize();
        if *settings != sanitized {
            *settings = sanitized;
        }
    }
}

fn advance_grounded_two_bone_ik_walk_owner_state(
    mut commands: Commands,
    time: Res<Time>,
    mut owners: Query<
        (
            Entity,
            &GroundedTwoBoneIkWalkSettings,
            &GlobalTransform,
            Option<&mut GroundedTwoBoneIkWalkOwnerState>,
        ),
        With<GroundedTwoBoneIkWalk>,
    >,
) {
    let dt = time.delta_secs();

    for (owner, settings, owner_global_transform, owner_state) in &mut owners {
        let cycle = settings.gait_cycle_duration;
        let owner_world_position = owner_global_transform.translation();
        let owner_forward = planar_direction_or(
            owner_global_transform.forward().as_vec3(),
            Vec3::Z,
        );
        match owner_state {
            Some(mut owner_state) => {
                owner_state.gait_elapsed += dt;
                if cycle > 0.0 {
                    owner_state.gait_elapsed = owner_state.gait_elapsed.rem_euclid(cycle);
                }

                let delta_world = owner_world_position - owner_state.last_position;
                let planar_delta = Vec3::new(delta_world.x, 0.0, delta_world.z);
                let planar_delta_len_sq = planar_delta.length_squared();
                if dt > 0.00001 && planar_delta_len_sq > 0.000001 {
                    let planar_delta_len = planar_delta_len_sq.sqrt();
                    owner_state.travel_speed = planar_delta_len / dt;
                    owner_state.travel_direction = planar_delta / planar_delta_len;
                } else {
                    owner_state.travel_speed = 0.0;
                    if owner_state.travel_direction.length_squared() <= 0.000001 {
                        owner_state.travel_direction = owner_forward;
                    }
                }
                owner_state.last_position = owner_world_position;
            }
            None => {
                commands.entity(owner).insert(GroundedTwoBoneIkWalkOwnerState {
                    gait_elapsed: 0.0,
                    last_position: owner_world_position,
                    travel_direction: owner_forward,
                    travel_speed: 0.0,
                });
            }
        }
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
            Option<&GroundedTwoBoneIkWalkOwnerState>,
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
        let has_leg_state = leg_state.is_some();
        let has_target_override = target_override.is_some();
        let Ok((ik_settings, walk_settings, owner_state, owner_global_transform)) =
            owners.get(rig.owner)
        else {
            clear_walk_components(
                &mut commands,
                rig_entity,
                has_leg_state,
                has_target_override,
            );
            continue;
        };

        if !walk_settings.enabled {
            clear_walk_components(
                &mut commands,
                rig_entity,
                has_leg_state,
                has_target_override,
            );
            continue;
        }

        let travel_direction = owner_state
            .map(|state| state.travel_direction)
            .unwrap_or_else(|| {
                planar_direction_or(owner_global_transform.forward().as_vec3(), Vec3::Z)
            });
        let travel_speed = owner_state.map(|state| state.travel_speed).unwrap_or(0.0);
        let forward_distance = compute_forward_distance(*walk_settings, travel_speed);
        let use_forward_probe =
            travel_speed >= WALK_MIN_TRAVEL_SPEED_FOR_FORWARD && forward_distance > 0.0001;
        let ray_anchor_world_offset = if use_forward_probe {
            travel_direction * forward_distance
        } else {
            Vec3::ZERO
        };

        let sampled_target = if use_forward_probe {
            sample_ground_target_with_world_offset(
                &spatial_query,
                rig.owner,
                owner_global_transform,
                rig,
                ik_settings,
                ray_anchor_world_offset,
            )
            .or_else(|| {
                sample_ground_target(
                    &spatial_query,
                    rig.owner,
                    owner_global_transform,
                    rig,
                    ik_settings,
                )
            })
        } else {
            sample_ground_target(
                &spatial_query,
                rig.owner,
                owner_global_transform,
                rig,
                ik_settings,
            )
        };
        let Some(sampled_target) = sampled_target else {
            clear_walk_components(
                &mut commands,
                rig_entity,
                has_leg_state,
                has_target_override,
            );
            continue;
        };

        let leg_phase = leg_phase_override
            .map(|phase| phase.phase_offset)
            .unwrap_or_else(|| default_phase_offset(rig))
            .rem_euclid(1.0);
        let gait_phase = owner_state
            .map(|state| (state.gait_elapsed / walk_settings.gait_cycle_duration).fract())
            .unwrap_or(0.0);
        let phase_ready =
            phase_distance(gait_phase, leg_phase) <= (0.5 * walk_settings.phase_start_window);

        if let Some(mut leg_state) = leg_state {
            let effective_target = tick_walk_leg_state(
                &mut leg_state,
                sampled_target,
                *walk_settings,
                dt,
                phase_ready,
            );
            if let Some(mut target_override) = target_override {
                target_override.world_target = effective_target;
            } else {
                commands.entity(rig_entity).insert(GroundedTwoBoneIkTargetOverride {
                    world_target: effective_target,
                });
            }
            continue;
        }

        let state = GroundedTwoBoneIkWalkLegState {
            planted_target: sampled_target,
            step_start: sampled_target,
            step_end: sampled_target,
            step_elapsed: walk_settings.step_duration,
            step_duration: walk_settings.step_duration,
            cooldown_remaining: 0.0,
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
    phase_ready: bool,
) -> Vec3 {
    state.cooldown_remaining = (state.cooldown_remaining - dt).max(0.0);

    let max_plant_distance_sq = settings.max_plant_distance * settings.max_plant_distance;
    let phase_break_distance_sq = max_plant_distance_sq
        * WALK_PHASE_BREAK_DISTANCE_MULTIPLIER
        * WALK_PHASE_BREAK_DISTANCE_MULTIPLIER;
    let emergency_replant_distance_sq = max_plant_distance_sq
        * WALK_EMERGENCY_REPLANT_MULTIPLIER
        * WALK_EMERGENCY_REPLANT_MULTIPLIER;
    let planted_drift_sq = state.planted_target.distance_squared(sampled_target);
    let phase_break = planted_drift_sq >= phase_break_distance_sq;

    if !state.stepping {
        let cooldown_ready = state.cooldown_remaining <= 0.0 || phase_break;
        let can_start = phase_ready || phase_break;
        if planted_drift_sq >= max_plant_distance_sq && cooldown_ready && can_start {
            start_step(state, sampled_target, settings);
        } else if phase_break {
            replant_immediately(state, sampled_target, settings);
        }
    }

    if !state.stepping {
        return state.planted_target;
    }

    // Keep the destination live so large owner movement doesn't induce long drag tails.
    state.step_end = sampled_target;
    update_step_duration(state, settings);

    state.step_elapsed += dt;
    let t = (state.step_elapsed / state.step_duration).clamp(0.0, 1.0);
    let eased_t = smoothstep(t);
    let base_target = state.step_start.lerp(state.step_end, eased_t);
    let centered_t = 2.0 * eased_t - 1.0;
    let lift = (1.0 - centered_t * centered_t).max(0.0) * settings.step_height;
    let stepped_target = base_target + Vec3::Y * lift;

    if stepped_target.distance_squared(sampled_target) >= emergency_replant_distance_sq {
        replant_immediately(state, sampled_target, settings);
        return sampled_target;
    }

    if t >= 1.0 {
        state.stepping = false;
        state.planted_target = state.step_end;
        if state.planted_target.distance_squared(sampled_target) > max_plant_distance_sq {
            state.cooldown_remaining = 0.0;
        } else {
            state.cooldown_remaining = settings.min_step_interval;
        }
    }

    stepped_target
}

fn start_step(
    state: &mut GroundedTwoBoneIkWalkLegState,
    sampled_target: Vec3,
    settings: GroundedTwoBoneIkWalkSettings,
) {
    state.stepping = true;
    state.step_elapsed = 0.0;
    state.step_start = state.planted_target;
    state.step_end = sampled_target;
    update_step_duration(state, settings);
}

fn replant_immediately(
    state: &mut GroundedTwoBoneIkWalkLegState,
    sampled_target: Vec3,
    settings: GroundedTwoBoneIkWalkSettings,
) {
    state.stepping = false;
    state.step_elapsed = settings.step_duration;
    state.step_duration = settings.step_duration;
    state.step_start = sampled_target;
    state.step_end = sampled_target;
    state.planted_target = sampled_target;
    state.cooldown_remaining = 0.0;
}

fn update_step_duration(
    state: &mut GroundedTwoBoneIkWalkLegState,
    settings: GroundedTwoBoneIkWalkSettings,
) {
    let step_distance = state.step_start.distance(state.step_end);
    let duration_scale = if step_distance <= 0.0001 {
        1.0
    } else {
        (settings.max_plant_distance / step_distance).clamp(WALK_MIN_STEP_DURATION_SCALE, 1.0)
    };
    state.step_duration = (settings.step_duration * duration_scale).max(WALK_MIN_STEP_DURATION);
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

fn phase_distance(a: f32, b: f32) -> f32 {
    let delta = (a - b).abs();
    delta.min(1.0 - delta)
}

fn compute_forward_distance(settings: GroundedTwoBoneIkWalkSettings, travel_speed: f32) -> f32 {
    let dynamic_distance =
        settings.step_forward_distance + travel_speed * settings.step_forward_speed_scale;
    dynamic_distance.clamp(0.0, settings.step_forward_max_distance)
}

fn clear_walk_components(
    commands: &mut Commands,
    rig_entity: Entity,
    has_leg_state: bool,
    has_target_override: bool,
) {
    if has_leg_state || has_target_override {
        commands
            .entity(rig_entity)
            .remove::<(GroundedTwoBoneIkWalkLegState, GroundedTwoBoneIkTargetOverride)>();
    }
}

fn planar_direction_or(input: Vec3, fallback: Vec3) -> Vec3 {
    let planar = Vec3::new(input.x, 0.0, input.z);
    if planar.length_squared() > 0.000001 {
        planar.normalize()
    } else {
        let fallback_planar = Vec3::new(fallback.x, 0.0, fallback.z);
        if fallback_planar.length_squared() > 0.000001 {
            fallback_planar.normalize()
        } else {
            Vec3::Z
        }
    }
}

fn sanitize_f32(value: f32, fallback: f32, min_value: f32, max_value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(min_value, max_value)
    } else {
        fallback
    }
}

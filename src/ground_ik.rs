//! Grounded two-bone IK runtime.
//!
//! This module owns the generic path:
//! - `GroundedTwoBoneIkOwner`: marks an entity as a grounded IK owner.
//! - `GroundedTwoBoneIkSettings`: runtime tuning for grounded target acquisition.
//! - `GroundedTwoBoneIkRig`: per-leg runtime data consumed by the solve pass.
//! - `GroundedTwoBoneIkTargetOverride`: optional per-leg world target override.
//! - `GroundedTwoBoneIkLegInit`: generic one-leg init descriptor.
//! - `init_leg_rigs`: generic rig initializer that computes rest data and inserts rigs.
//! - `solve_two_bone_ik`: pure geometric two-bone solver used by grounded IK.

use avian3d::prelude::*;
use bevy::prelude::*;

const IK_DEFAULT_RAY_ORIGIN_UP: f32 = 1.0;
const IK_DEFAULT_RAY_DISTANCE: f32 = 3.1;
const IK_DEFAULT_TARGET_FOOT_OFFSET: f32 = 0.0;
const IK_DEFAULT_POLE_SIDE_OFFSET: f32 = 0.0;
const IK_DEFAULT_POLE_FORWARD_OFFSET: f32 = 0.0;
const IK_DEFAULT_POLE_UP_OFFSET: f32 = 0.3;
const IK_DEFAULT_MAX_REACH_RATIO: f32 = 0.995;
const IK_DEFAULT_GIZMO_MARKER_SIZE: f32 = 0.2;

const IK_MIN_RAY_DISTANCE: f32 = 0.05;
const IK_MIN_GIZMO_MARKER_SIZE: f32 = 0.01;
const IK_MIN_REACH_RATIO: f32 = 0.1;
const IK_MAX_REACH_RATIO: f32 = 0.9999;
const IK_SOLVER_EPSILON: f32 = 0.0001;

pub(crate) struct GroundIkPlugin;

impl Plugin for GroundIkPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<GroundedTwoBoneIkSettings>()
            .configure_sets(Update, (GroundIkSet::Sanitize, GroundIkSet::Solve).chain())
            .add_systems(
                Update,
                sanitize_grounded_two_bone_ik_settings.in_set(GroundIkSet::Sanitize),
            )
            .add_systems(
                Update,
                solve_grounded_two_bone_ik.in_set(GroundIkSet::Solve),
            );
    }
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum GroundIkSet {
    Sanitize,
    Solve,
}

#[derive(Component)]
#[require(GroundedTwoBoneIkSettings)]
/// Marks an entity as an owner for grounded two-bone IK.
///
/// Inserting this component automatically inserts `GroundedTwoBoneIkSettings`
/// via required components.
pub(crate) struct GroundedTwoBoneIkOwner;

#[derive(Component, Clone, Copy, Debug)]
/// Internal runtime rig data for one two-bone leg chain.
pub(crate) struct GroundedTwoBoneIkRig {
    /// Entity that owns this leg rig and provides grounded IK settings.
    pub(crate) owner: Entity,
    /// Debug gizmo color for this leg.
    pub(crate) debug_color: Color,
    /// Per-leg lateral sign used when building the pole point from settings.
    pub(crate) side_sign: f32,
    /// Per-leg forward sign used when building the pole point from settings.
    pub(crate) fore_sign: f32,
    /// Upper-joint entity (root joint of the two-bone chain).
    pub(crate) hip: Entity,
    /// Middle-joint entity.
    pub(crate) knee: Entity,
    /// End-effector entity for this leg.
    pub(crate) foot: Entity,
    /// World-space rest length from hip to knee.
    pub(crate) upper_len: f32,
    /// World-space rest length from knee to foot.
    pub(crate) lower_len: f32,
    /// Hip local rotation captured at bind/rest pose.
    pub(crate) hip_bind_rotation: Quat,
    /// Knee local rotation captured at bind/rest pose.
    pub(crate) knee_bind_rotation: Quat,
    /// Hip rest direction in the hip parent space.
    pub(crate) hip_rest_dir_parent_space: Vec3,
    /// Knee rest direction in the knee parent space.
    pub(crate) knee_rest_dir_parent_space: Vec3,
    /// Foot rest position in owner-local space used as grounded ray anchor.
    pub(crate) foot_rest_owner_space: Vec3,
}

#[derive(Component, Clone, Copy, Debug)]
/// Optional world-space foot target override for one leg rig.
///
/// Insert on the leg rig entity (typically the hip). When present, the grounded
/// solve uses this target instead of acquiring one from a downward raycast.
pub(crate) struct GroundedTwoBoneIkTargetOverride {
    pub(crate) world_target: Vec3,
}

#[derive(Clone, Copy, Debug)]
/// Generic init data for one grounded two-bone leg.
pub(crate) struct GroundedTwoBoneIkLegInit {
    /// Debug gizmo color for this leg.
    pub(crate) debug_color: Color,
    /// Per-leg lateral sign used when building the pole point from settings.
    pub(crate) side_sign: f32,
    /// Per-leg forward sign used when building the pole point from settings.
    pub(crate) fore_sign: f32,
    /// Upper-joint entity (root joint of the two-bone chain).
    pub(crate) hip: Entity,
    /// Middle-joint entity.
    pub(crate) knee: Entity,
    /// End-effector entity.
    pub(crate) foot: Entity,
}

#[derive(Clone, Copy, Debug, Default)]
/// Summary for one `init_leg_rigs` call.
pub(crate) struct GroundedTwoBoneIkInitSummary {
    /// Count of leg descriptors provided to the initializer.
    pub(crate) requested_count: usize,
    /// Count of rigs inserted this run.
    pub(crate) inserted_count: usize,
    /// Count skipped because a rig already exists on the hip entity.
    pub(crate) skipped_existing_count: usize,
    /// Count skipped due to missing transforms on owner/leg entities.
    pub(crate) skipped_missing_transform_count: usize,
}

/// Initializes grounded IK rigs from generic leg descriptors.
///
/// The rig component is inserted on each leg's hip entity.
pub(crate) fn init_leg_rigs<I>(
    commands: &mut Commands,
    owner: Entity,
    leg_inits: I,
    local_transforms: &Query<&Transform>,
    global_transforms: &Query<&GlobalTransform>,
    existing_leg_rigs: &Query<(), With<GroundedTwoBoneIkRig>>,
) -> GroundedTwoBoneIkInitSummary
where
    I: IntoIterator<Item = GroundedTwoBoneIkLegInit>,
{
    let mut summary = GroundedTwoBoneIkInitSummary::default();
    let Ok(owner_global_transform) = global_transforms.get(owner) else {
        return summary;
    };
    let owner_inverse_affine = owner_global_transform.affine().inverse();

    for leg in leg_inits {
        summary.requested_count += 1;

        if existing_leg_rigs.contains(leg.hip) {
            summary.skipped_existing_count += 1;
            continue;
        }

        let Ok(
            [
                hip_local_transform,
                knee_local_transform,
                foot_local_transform,
            ],
        ) = local_transforms.get_many([leg.hip, leg.knee, leg.foot])
        else {
            summary.skipped_missing_transform_count += 1;
            continue;
        };
        let Ok(
            [
                hip_global_transform,
                knee_global_transform,
                foot_global_transform,
            ],
        ) = global_transforms.get_many([leg.hip, leg.knee, leg.foot])
        else {
            summary.skipped_missing_transform_count += 1;
            continue;
        };

        let upper_len = hip_global_transform
            .translation()
            .distance(knee_global_transform.translation())
            .max(IK_SOLVER_EPSILON);
        let lower_len = knee_global_transform
            .translation()
            .distance(foot_global_transform.translation())
            .max(IK_SOLVER_EPSILON);

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

        commands.entity(leg.hip).insert(GroundedTwoBoneIkRig {
            owner,
            debug_color: leg.debug_color,
            side_sign: leg.side_sign,
            fore_sign: leg.fore_sign,
            hip: leg.hip,
            knee: leg.knee,
            foot: leg.foot,
            upper_len,
            lower_len,
            hip_bind_rotation: hip_local_transform.rotation,
            knee_bind_rotation: knee_local_transform.rotation,
            hip_rest_dir_parent_space,
            knee_rest_dir_parent_space,
            foot_rest_owner_space,
        });
        summary.inserted_count += 1;
    }

    summary
}

#[derive(Component, Reflect, Clone, Copy, Debug)]
#[reflect(Component)]
/// Tunable settings for grounded two-bone IK target acquisition and debug draw.
pub(crate) struct GroundedTwoBoneIkSettings {
    /// Enable/disable the grounded IK pass for this owner.
    enabled: bool,
    /// Draw tuning gizmos: ray, target marker, pole hint, and solved leg segments.
    draw_gizmos: bool,
    /// Vertical offset above the current foot position where the ground ray starts.
    ray_origin_up: f32,
    /// Maximum distance for the downward foot-to-ground ray cast.
    ray_distance: f32,
    /// Final world-space Y offset added to the hit point (useful to keep feet above/below ground).
    target_foot_offset: f32,
    /// Sideways pole offset from hip in owner local-space (controls outward knee bend).
    pole_side_offset: f32,
    /// Forward pole offset from hip in owner local-space (per-leg fore sign is applied).
    pole_forward_offset: f32,
    /// Vertical pole offset from hip in owner local-space.
    pole_up_offset: f32,
    /// Maximum solver reach as a ratio of (upper_len + lower_len); below 1.0 avoids full extension instability.
    max_reach_ratio: f32,
    /// Cross marker half-size for hit, target, and pole gizmos.
    gizmo_marker_size: f32,
}

impl Default for GroundedTwoBoneIkSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            draw_gizmos: false,
            ray_origin_up: IK_DEFAULT_RAY_ORIGIN_UP,
            ray_distance: IK_DEFAULT_RAY_DISTANCE,
            target_foot_offset: IK_DEFAULT_TARGET_FOOT_OFFSET,
            pole_side_offset: IK_DEFAULT_POLE_SIDE_OFFSET,
            pole_forward_offset: IK_DEFAULT_POLE_FORWARD_OFFSET,
            pole_up_offset: IK_DEFAULT_POLE_UP_OFFSET,
            max_reach_ratio: IK_DEFAULT_MAX_REACH_RATIO,
            gizmo_marker_size: IK_DEFAULT_GIZMO_MARKER_SIZE,
        }
    }
}

impl GroundedTwoBoneIkSettings {
    fn sanitize(self) -> Self {
        Self {
            enabled: self.enabled,
            draw_gizmos: self.draw_gizmos,
            ray_origin_up: sanitize_f32(self.ray_origin_up, IK_DEFAULT_RAY_ORIGIN_UP, 0.0, 10.0),
            ray_distance: sanitize_f32(
                self.ray_distance,
                IK_DEFAULT_RAY_DISTANCE,
                IK_MIN_RAY_DISTANCE,
                20.0,
            ),
            target_foot_offset: sanitize_f32(
                self.target_foot_offset,
                IK_DEFAULT_TARGET_FOOT_OFFSET,
                -1.0,
                1.0,
            ),
            pole_side_offset: sanitize_f32(
                self.pole_side_offset,
                IK_DEFAULT_POLE_SIDE_OFFSET,
                -3.0,
                3.0,
            ),
            pole_forward_offset: sanitize_f32(
                self.pole_forward_offset,
                IK_DEFAULT_POLE_FORWARD_OFFSET,
                -3.0,
                3.0,
            ),
            pole_up_offset: sanitize_f32(self.pole_up_offset, IK_DEFAULT_POLE_UP_OFFSET, -3.0, 3.0),
            max_reach_ratio: sanitize_f32(
                self.max_reach_ratio,
                IK_DEFAULT_MAX_REACH_RATIO,
                IK_MIN_REACH_RATIO,
                IK_MAX_REACH_RATIO,
            ),
            gizmo_marker_size: sanitize_f32(
                self.gizmo_marker_size,
                IK_DEFAULT_GIZMO_MARKER_SIZE,
                IK_MIN_GIZMO_MARKER_SIZE,
                1.0,
            ),
        }
    }
}

#[derive(Clone, Copy)]
/// Result of solving a two-bone chain in world-space.
struct TwoBoneIkSolution {
    target: Vec3,
    knee: Vec3,
    target_clamped: bool,
}

fn sanitize_grounded_two_bone_ik_settings(
    mut owners: Query<&mut GroundedTwoBoneIkSettings, With<GroundedTwoBoneIkOwner>>,
) {
    for mut settings in &mut owners {
        *settings = settings.sanitize();
    }
}

/// Acquires a grounded target for a leg via downward raycast from its owner-space
/// rest anchor, offset in world-space before raycasting.
pub(crate) fn sample_ground_target_with_world_offset(
    spatial_query: &SpatialQuery,
    owner: Entity,
    owner_global_transform: &GlobalTransform,
    rig: &GroundedTwoBoneIkRig,
    settings: &GroundedTwoBoneIkSettings,
    ray_anchor_world_offset: Vec3,
) -> Option<Vec3> {
    let ray_anchor_world =
        owner_global_transform.transform_point(rig.foot_rest_owner_space) + ray_anchor_world_offset;
    let ray_origin = ray_anchor_world + Vec3::Y * settings.ray_origin_up;
    let filter = SpatialQueryFilter::from_excluded_entities([owner]);
    let hit = spatial_query.cast_ray(
        ray_origin,
        Dir3::NEG_Y,
        settings.ray_distance,
        true,
        &filter,
    )?;
    let hit_point = ray_origin + Vec3::NEG_Y * hit.distance;
    Some(hit_point + Vec3::Y * settings.target_foot_offset)
}

/// Acquires a grounded target for a leg via downward raycast from its owner-space
/// rest anchor, using the provided grounded IK settings.
pub(crate) fn sample_ground_target(
    spatial_query: &SpatialQuery,
    owner: Entity,
    owner_global_transform: &GlobalTransform,
    rig: &GroundedTwoBoneIkRig,
    settings: &GroundedTwoBoneIkSettings,
) -> Option<Vec3> {
    sample_ground_target_with_world_offset(
        spatial_query,
        owner,
        owner_global_transform,
        rig,
        settings,
        Vec3::ZERO,
    )
}

/// Grounded solve pass:
/// - raycasts down from each leg's owner-space rest anchor
/// - builds a world-space target and pole point
/// - runs `solve_two_bone_ik`
/// - applies solved hip/knee local rotations
/// - optionally draws debug gizmos
fn solve_grounded_two_bone_ik(
    spatial_query: SpatialQuery,
    mut gizmos: Gizmos,
    owners: Query<(&GroundedTwoBoneIkSettings, &GlobalTransform), With<GroundedTwoBoneIkOwner>>,
    rigs: Query<&GroundedTwoBoneIkRig>,
    target_overrides: Query<&GroundedTwoBoneIkTargetOverride>,
    global_transforms: Query<&GlobalTransform>,
    mut local_transforms: Query<&mut Transform>,
) {
    for rig in &rigs {
        let Ok((settings, owner_global_transform)) = owners.get(rig.owner) else {
            continue;
        };
        if !settings.enabled {
            continue;
        }
        let settings = settings.sanitize();

        let Ok(hip_global_transform) = global_transforms.get(rig.hip) else {
            continue;
        };
        let Ok(knee_global_transform) = global_transforms.get(rig.knee) else {
            continue;
        };
        let Ok(foot_global_transform) = global_transforms.get(rig.foot) else {
            continue;
        };

        let leg_color = rig.debug_color;
        let hip_world = hip_global_transform.translation();
        let foot_world = foot_global_transform.translation();
        let ray_anchor_world = owner_global_transform.transform_point(rig.foot_rest_owner_space);
        let ray_origin = ray_anchor_world + Vec3::Y * settings.ray_origin_up;
        let ray_end = ray_origin + Vec3::NEG_Y * settings.ray_distance;

        if settings.draw_gizmos {
            gizmos.line(ray_origin, ray_end, leg_color);
            draw_cross_marker(
                &mut gizmos,
                ray_anchor_world,
                settings.gizmo_marker_size,
                leg_color,
            );
            gizmos.line(
                hip_world,
                knee_global_transform.translation(),
                Color::srgb(0.35, 0.35, 0.35),
            );
            gizmos.line(
                knee_global_transform.translation(),
                foot_world,
                Color::srgb(0.35, 0.35, 0.35),
            );
        }

        let mut hit_point = None;
        let target = if let Ok(target_override) = target_overrides.get(rig.hip) {
            target_override.world_target
        } else {
            let Some(target) = sample_ground_target(
                &spatial_query,
                rig.owner,
                owner_global_transform,
                rig,
                &settings,
            ) else {
                if let Ok([mut hip_local_transform, mut knee_local_transform]) =
                    local_transforms.get_many_mut([rig.hip, rig.knee])
                {
                    hip_local_transform.rotation = rig.hip_bind_rotation;
                    knee_local_transform.rotation = rig.knee_bind_rotation;
                }
                continue;
            };
            hit_point = Some(target - Vec3::Y * settings.target_foot_offset);
            target
        };
        let owner_rotation = owner_global_transform.rotation();
        let pole_point = hip_world
            + owner_rotation
                * Vec3::new(
                    settings.pole_side_offset * rig.side_sign,
                    settings.pole_up_offset,
                    settings.pole_forward_offset * rig.fore_sign,
                );
        let solution = solve_two_bone_ik(
            hip_world,
            target,
            pole_point,
            rig.upper_len,
            rig.lower_len,
            settings.max_reach_ratio,
        );

        let Ok([mut hip_local_transform, mut knee_local_transform]) =
            local_transforms.get_many_mut([rig.hip, rig.knee])
        else {
            continue;
        };

        let parent_world_rotation = safe_quat_or(
            hip_global_transform.rotation() * hip_local_transform.rotation.inverse(),
            Quat::IDENTITY,
        );
        let desired_upper_world_dir = safe_normalize(solution.knee - hip_world, Vec3::NEG_Y);
        let desired_upper_parent_dir = safe_normalize(
            parent_world_rotation.inverse() * desired_upper_world_dir,
            rig.hip_rest_dir_parent_space,
        );
        let hip_delta = safe_rotation_arc(rig.hip_rest_dir_parent_space, desired_upper_parent_dir);
        let solved_hip_local_rotation =
            safe_quat_or(hip_delta * rig.hip_bind_rotation, rig.hip_bind_rotation);
        hip_local_transform.rotation = solved_hip_local_rotation;

        let solved_hip_world_rotation = safe_quat_or(
            parent_world_rotation * solved_hip_local_rotation,
            parent_world_rotation,
        );
        let desired_lower_world_dir = safe_normalize(solution.target - solution.knee, Vec3::NEG_Y);
        let desired_lower_parent_dir = safe_normalize(
            solved_hip_world_rotation.inverse() * desired_lower_world_dir,
            rig.knee_rest_dir_parent_space,
        );
        let knee_delta =
            safe_rotation_arc(rig.knee_rest_dir_parent_space, desired_lower_parent_dir);
        let solved_knee_local_rotation =
            safe_quat_or(knee_delta * rig.knee_bind_rotation, rig.knee_bind_rotation);
        knee_local_transform.rotation = solved_knee_local_rotation;

        if settings.draw_gizmos {
            let solved_color = if solution.target_clamped {
                Color::srgb(1.0, 0.6, 0.2)
            } else {
                Color::WHITE
            };
            if let Some(hit_point) = hit_point {
                gizmos.line(foot_world, hit_point, Color::srgb(0.7, 0.7, 0.7));
                draw_cross_marker(
                    &mut gizmos,
                    hit_point,
                    settings.gizmo_marker_size,
                    Color::srgb(0.8, 0.8, 0.8),
                );
            }
            gizmos.line(hip_world, solution.knee, solved_color);
            gizmos.line(solution.knee, solution.target, solved_color);
            draw_cross_marker(
                &mut gizmos,
                solution.target,
                settings.gizmo_marker_size,
                solved_color,
            );
            draw_cross_marker(
                &mut gizmos,
                pole_point,
                settings.gizmo_marker_size,
                Color::srgb(0.9, 0.2, 1.0),
            );

            if solution.target_clamped {
                gizmos.line(solution.target, target, Color::srgb(1.0, 0.6, 0.2));
            }
        }
    }
}

/// Solves a two-bone chain in world-space against a target and pole hint.
///
/// The target distance is clamped to a stable reachable range:
/// - minimum reach: `abs(upper_len - lower_len) + epsilon`
/// - maximum reach: `(upper_len + lower_len) * max_reach_ratio`
///
/// Returns the clamped target and the solved knee position.
fn solve_two_bone_ik(
    hip: Vec3,
    target: Vec3,
    pole: Vec3,
    upper_len: f32,
    lower_len: f32,
    max_reach_ratio: f32,
) -> TwoBoneIkSolution {
    let hip_to_target = target - hip;
    let direction = safe_normalize(hip_to_target, Vec3::NEG_Y);
    let raw_distance = hip_to_target.length();
    let min_reach = (upper_len - lower_len).abs() + IK_SOLVER_EPSILON;
    let max_reach = ((upper_len + lower_len) * max_reach_ratio).max(min_reach + IK_SOLVER_EPSILON);
    let clamped_distance = raw_distance.clamp(min_reach, max_reach);
    let clamped_target = hip + direction * clamped_distance;
    let target_clamped = (clamped_distance - raw_distance).abs() > 0.0005;

    let along = ((upper_len * upper_len - lower_len * lower_len
        + clamped_distance * clamped_distance)
        / (2.0 * clamped_distance))
        .clamp(IK_SOLVER_EPSILON, upper_len);
    let height_sq = (upper_len * upper_len - along * along).max(0.0);
    let height = height_sq.sqrt();

    let pole_offset = pole - hip;
    let bend_hint = pole_offset - direction * pole_offset.dot(direction);
    let bend_normal = if bend_hint.length_squared() > IK_SOLVER_EPSILON {
        bend_hint.normalize()
    } else {
        any_perpendicular(direction)
    };
    let knee = hip + direction * along + bend_normal * height;

    TwoBoneIkSolution {
        target: clamped_target,
        knee,
        target_clamped,
    }
}

fn draw_cross_marker(gizmos: &mut Gizmos, center: Vec3, half: f32, color: Color) {
    gizmos.line(
        center + Vec3::new(-half, 0.0, 0.0),
        center + Vec3::new(half, 0.0, 0.0),
        color,
    );
    gizmos.line(
        center + Vec3::new(0.0, 0.0, -half),
        center + Vec3::new(0.0, 0.0, half),
        color,
    );
}

fn safe_rotation_arc(from: Vec3, to: Vec3) -> Quat {
    let from = safe_normalize(from, Vec3::Y);
    let to = safe_normalize(to, from);
    safe_quat_or(Quat::from_rotation_arc(from, to), Quat::IDENTITY)
}

fn safe_normalize(input: Vec3, fallback: Vec3) -> Vec3 {
    let normalized = input.normalize_or_zero();
    if normalized.length_squared() > IK_SOLVER_EPSILON {
        normalized
    } else {
        fallback.normalize_or_zero()
    }
}

fn safe_quat_or(input: Quat, fallback: Quat) -> Quat {
    if input.x.is_finite() && input.y.is_finite() && input.z.is_finite() && input.w.is_finite() {
        input.normalize()
    } else {
        fallback
    }
}

fn sanitize_f32(value: f32, fallback: f32, min_value: f32, max_value: f32) -> f32 {
    if value.is_finite() {
        value.clamp(min_value, max_value)
    } else {
        fallback
    }
}

fn any_perpendicular(direction: Vec3) -> Vec3 {
    let axis = if direction.y.abs() < 0.99 {
        Vec3::Y
    } else {
        Vec3::X
    };
    safe_normalize(direction.cross(axis), Vec3::X)
}

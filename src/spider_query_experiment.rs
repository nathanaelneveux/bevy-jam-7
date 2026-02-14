use avian3d::prelude::*;
use bevy::prelude::*;
use std::f32::consts::PI;

const EXPERIMENT_MODEL_X: f32 = -2.0;
const EXPERIMENT_MODEL_Y: f32 = -5.0;
const EXPERIMENT_VISUAL_Y_OFFSET: f32 = -0.5;
const EXPERIMENT_VISUAL_YAW_OFFSET: f32 = PI;
const IK_DEFAULT_RAY_ORIGIN_UP: f32 = 1.0;
const IK_DEFAULT_RAY_DISTANCE: f32 = 3.0;
const IK_DEFAULT_TARGET_FOOT_OFFSET: f32 = 0.0;
const IK_DEFAULT_POLE_SIDE_OFFSET: f32 = 0.55;
const IK_DEFAULT_POLE_FORWARD_OFFSET: f32 = 0.22;
const IK_DEFAULT_POLE_UP_OFFSET: f32 = 0.3;
const IK_DEFAULT_MAX_REACH_RATIO: f32 = 0.995;
const IK_DEFAULT_GIZMO_MARKER_SIZE: f32 = 0.08;

const IK_MIN_RAY_DISTANCE: f32 = 0.05;
const IK_MIN_GIZMO_MARKER_SIZE: f32 = 0.01;
const IK_MIN_REACH_RATIO: f32 = 0.1;
const IK_MAX_REACH_RATIO: f32 = 0.9999;
const IK_SOLVER_EPSILON: f32 = 0.0001;

pub struct SpiderQueryExperimentPlugin;

impl Plugin for SpiderQueryExperimentPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<ExperimentSpiderIkSettings>()
            .add_systems(Startup, spawn_spider_query_experiment)
            .add_systems(
                Update,
                (
                    init_spider_leg_rig,
                    sanitize_experiment_spider_ik_settings,
                    solve_spider_two_bone_ik,
                )
                    .chain(),
            );
    }
}

#[derive(Component)]
struct ExperimentSpider;

#[derive(Component)]
struct ExperimentSpiderVisualRoot {
    owner: Entity,
}

#[derive(Component)]
struct ExperimentRigReady;

#[derive(Component, Clone, Copy, Debug)]
struct ExperimentSpiderLegRig {
    owner: Entity,
    leg: SpiderLegId,
    side_sign: f32,
    fore_sign: f32,
    hip: Entity,
    knee: Entity,
    foot: Entity,
    upper_len: f32,
    lower_len: f32,
    hip_bind_rotation: Quat,
    knee_bind_rotation: Quat,
    hip_rest_dir_parent_space: Vec3,
    knee_rest_dir_parent_space: Vec3,
}

#[derive(Component, Reflect, Clone, Copy, Debug)]
#[reflect(Component)]
struct ExperimentSpiderIkSettings {
    /// Enable/disable the IK pass for this spider.
    enabled: bool,
    /// Draw tuning gizmos: ray, target marker, pole hint, and solved leg segments.
    draw_gizmos: bool,
    /// Vertical offset above the current foot position where the ground ray starts.
    ray_origin_up: f32,
    /// Maximum distance for the downward foot-to-ground ray cast.
    ray_distance: f32,
    /// Final world-space Y offset added to the hit point (useful to keep feet above/below ground).
    target_foot_offset: f32,
    /// Sideways pole offset from hip in spider local-space (controls outward knee bend).
    pole_side_offset: f32,
    /// Forward pole offset from hip in spider local-space (front/rear legs apply this with sign).
    pole_forward_offset: f32,
    /// Vertical pole offset from hip in spider local-space.
    pole_up_offset: f32,
    /// Maximum solver reach as a ratio of (upper_len + lower_len); below 1.0 avoids full extension instability.
    max_reach_ratio: f32,
    /// Cross marker half-size for hit, target, and pole gizmos.
    gizmo_marker_size: f32,
}

impl Default for ExperimentSpiderIkSettings {
    fn default() -> Self {
        Self {
            enabled: true,
            draw_gizmos: true,
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

impl ExperimentSpiderIkSettings {
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

#[derive(Clone, Copy)]
struct TwoBoneIkSolution {
    target: Vec3,
    knee: Vec3,
    target_clamped: bool,
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
            ExperimentSpider,
            Transform::from_xyz(EXPERIMENT_MODEL_X, EXPERIMENT_MODEL_Y, 0.0),
            ExperimentSpiderIkSettings::default(),
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

fn sanitize_experiment_spider_ik_settings(
    mut spiders: Query<&mut ExperimentSpiderIkSettings, With<ExperimentSpider>>,
) {
    for mut settings in &mut spiders {
        *settings = settings.sanitize();
    }
}

fn init_spider_leg_rig(
    mut commands: Commands,
    visual_roots: Query<(Entity, &ExperimentSpiderVisualRoot), Without<ExperimentRigReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    local_transforms: Query<&Transform>,
    global_transforms: Query<&GlobalTransform>,
    existing_leg_rigs: Query<(), With<ExperimentSpiderLegRig>>,
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

            commands.entity(hip).insert(ExperimentSpiderLegRig {
                owner: visual_info.owner,
                leg: spider_leg_from_index(index),
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
            });
        }

        if complete_leg_count == 4 {
            commands.entity(visual_root).insert(ExperimentRigReady);
        }
    }
}

fn solve_spider_two_bone_ik(
    spatial_query: SpatialQuery,
    mut gizmos: Gizmos,
    spiders: Query<(&ExperimentSpiderIkSettings, &GlobalTransform), With<ExperimentSpider>>,
    rigs: Query<&ExperimentSpiderLegRig>,
    global_transforms: Query<&GlobalTransform>,
    mut local_transforms: Query<&mut Transform>,
) {
    for rig in &rigs {
        let Ok((settings, spider_global_transform)) = spiders.get(rig.owner) else {
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

        let leg_color = spider_leg_color(rig.leg);
        let hip_world = hip_global_transform.translation();
        let foot_world = foot_global_transform.translation();
        let ray_origin = foot_world + Vec3::Y * settings.ray_origin_up;
        let ray_end = ray_origin + Vec3::NEG_Y * settings.ray_distance;

        if settings.draw_gizmos {
            gizmos.line(ray_origin, ray_end, leg_color);
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

        let filter = SpatialQueryFilter::from_excluded_entities([rig.owner]);
        let Some(hit) = spatial_query.cast_ray(
            ray_origin,
            Dir3::NEG_Y,
            settings.ray_distance,
            true,
            &filter,
        ) else {
            continue;
        };

        let hit_point = ray_origin + Vec3::NEG_Y * hit.distance;
        let target = hit_point + Vec3::Y * settings.target_foot_offset;
        let spider_rotation = spider_global_transform.rotation();
        let pole_point = hip_world
            + spider_rotation
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
            gizmos.line(foot_world, hit_point, Color::srgb(0.7, 0.7, 0.7));
            gizmos.line(hip_world, solution.knee, solved_color);
            gizmos.line(solution.knee, solution.target, solved_color);
            draw_cross_marker(
                &mut gizmos,
                hit_point,
                settings.gizmo_marker_size,
                Color::srgb(0.8, 0.8, 0.8),
            );
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

fn spider_leg_color(leg: SpiderLegId) -> Color {
    match leg {
        SpiderLegId::FrontLeft => Color::srgb(1.0, 0.35, 0.35),
        SpiderLegId::FrontRight => Color::srgb(0.35, 1.0, 0.35),
        SpiderLegId::RearLeft => Color::srgb(0.35, 0.6, 1.0),
        SpiderLegId::RearRight => Color::srgb(1.0, 0.8, 0.3),
    }
}

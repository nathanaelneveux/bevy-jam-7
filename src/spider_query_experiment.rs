use avian3d::prelude::*;
use bevy::prelude::*;
use std::f32::consts::PI;

const EXPERIMENT_MODEL_Y: f32 = 1.5;
const EXPERIMENT_VISUAL_Y_OFFSET: f32 = -0.5;
const EXPERIMENT_VISUAL_YAW_OFFSET: f32 = PI;
const FOOT_RAY_ORIGIN_UP: f32 = 0.6;
const FOOT_RAY_DISTANCE: f32 = 2.4;

pub struct SpiderQueryExperimentPlugin;

impl Plugin for SpiderQueryExperimentPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_spider_query_experiment)
            .add_systems(Update, (init_spider_foot_bones, debug_spider_foot_queries));
    }
}

#[derive(Component)]
struct ExperimentSpider;

#[derive(Component)]
struct ExperimentSpiderVisualRoot {
    owner: Entity,
}

#[derive(Component)]
struct ExperimentFeetReady;

#[derive(Component, Clone, Copy)]
struct ExperimentSpiderFootBone {
    owner: Entity,
    leg: SpiderLegId,
}

#[derive(Clone, Copy, PartialEq, Eq)]
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
            Transform::from_xyz(0.0, EXPERIMENT_MODEL_Y, 0.0),
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

fn init_spider_foot_bones(
    mut commands: Commands,
    visual_roots: Query<(Entity, &ExperimentSpiderVisualRoot), Without<ExperimentFeetReady>>,
    children_query: Query<&Children>,
    names: Query<&Name>,
    existing_foot_bones: Query<(), With<ExperimentSpiderFootBone>>,
) {
    for (visual_root, visual_info) in &visual_roots {
        let mut stack = vec![visual_root];
        let mut matched_feet = 0usize;

        while let Some(entity) = stack.pop() {
            if let Ok(children) = children_query.get(entity) {
                for child in children.iter() {
                    stack.push(child);
                }
            }

            let Ok(name) = names.get(entity) else {
                continue;
            };
            let Some(leg) = spider_foot_definition(name.as_str()) else {
                continue;
            };

            matched_feet += 1;
            if existing_foot_bones.contains(entity) {
                continue;
            }

            commands.entity(entity).insert(ExperimentSpiderFootBone {
                owner: visual_info.owner,
                leg,
            });
        }

        if matched_feet >= 4 {
            commands.entity(visual_root).insert(ExperimentFeetReady);
        }
    }
}

fn debug_spider_foot_queries(
    spatial_query: SpatialQuery,
    mut gizmos: Gizmos,
    spiders: Query<Entity, With<ExperimentSpider>>,
    foot_bones: Query<(&ExperimentSpiderFootBone, &GlobalTransform)>,
) {
    for spider in &spiders {
        for (foot_bone, foot_transform) in &foot_bones {
            if foot_bone.owner != spider {
                continue;
            }

            let ray_color = spider_leg_color(foot_bone.leg);
            let foot_world = foot_transform.translation();
            let ray_origin = foot_world + Vec3::Y * FOOT_RAY_ORIGIN_UP;
            let ray_end = ray_origin + Vec3::NEG_Y * FOOT_RAY_DISTANCE;
            gizmos.line(ray_origin, ray_end, ray_color);

            let filter = SpatialQueryFilter::from_excluded_entities([spider]);
            let Some(hit) =
                spatial_query.cast_ray(ray_origin, Dir3::NEG_Y, FOOT_RAY_DISTANCE, true, &filter)
            else {
                continue;
            };

            let hit_point = ray_origin + Vec3::NEG_Y * hit.distance;
            gizmos.line(foot_world, hit_point, Color::WHITE);

            let marker_half = 0.08;
            gizmos.line(
                hit_point + Vec3::new(-marker_half, 0.0, 0.0),
                hit_point + Vec3::new(marker_half, 0.0, 0.0),
                Color::WHITE,
            );
            gizmos.line(
                hit_point + Vec3::new(0.0, 0.0, -marker_half),
                hit_point + Vec3::new(0.0, 0.0, marker_half),
                Color::WHITE,
            );
        }
    }
}

fn spider_foot_definition(name: &str) -> Option<SpiderLegId> {
    match name {
        "FootFront.L" => Some(SpiderLegId::FrontLeft),
        "FootFront.R" => Some(SpiderLegId::FrontRight),
        "FootRear.L" => Some(SpiderLegId::RearLeft),
        "FootRear.R" => Some(SpiderLegId::RearRight),
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

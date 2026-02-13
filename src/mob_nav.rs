use avian3d::prelude::{Collisions, LinearVelocity};
use bevy::prelude::*;
use bevy_voxel_world::prelude::VoxelWorld;
use std::collections::HashSet;

use crate::cave_world::CaveWorld;

const GROUND_NORMAL_Y_THRESHOLD: f32 = 0.65;
const GROUND_JUMP_TRIGGER_DESIRED_Y: f32 = 0.9;
const GROUND_JUMP_SPEED: f32 = 5.0;
const WALL_NORMAL_MAX_Y: f32 = 0.15;

pub struct MobNavPlugin;

impl Plugin for MobNavPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MobNavRequestCounter>()
            .init_resource::<MobNavDebugConfig>()
            .register_type::<MobNavAgent>()
            .register_type::<MobNavGoal>()
            .register_type::<MobNavPath>()
            .register_type::<MobNavSteering>()
            .register_type::<MobNavStatus>()
            .register_type::<MobNavMovementMode>()
            .register_type::<MobNavRepath>()
            .register_type::<MobNavDebugConfig>()
            .add_message::<MobNavPlanRequest>()
            .add_message::<MobNavPlanResult>()
            .configure_sets(
                Update,
                (
                    MobNavUpdateSet::EnsureState,
                    MobNavUpdateSet::EnqueueRequests,
                    MobNavUpdateSet::PlanPaths,
                    MobNavUpdateSet::ApplyResults,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                ensure_nav_status.in_set(MobNavUpdateSet::EnsureState),
            )
            .add_systems(
                Update,
                enqueue_nav_requests.in_set(MobNavUpdateSet::EnqueueRequests),
            )
            .add_systems(
                Update,
                built_in_flying_los_planner.in_set(MobNavUpdateSet::PlanPaths),
            )
            .add_systems(
                Update,
                apply_plan_results.in_set(MobNavUpdateSet::ApplyResults),
            )
            .add_systems(
                Update,
                draw_mob_nav_path_gizmos
                    .after(MobNavUpdateSet::ApplyResults)
                    .run_if(mob_nav_debug_enabled),
            )
            .add_systems(Update, toggle_mob_nav_path_debug)
            .add_systems(
                FixedUpdate,
                (follow_mob_nav_paths, stop_non_following_agents),
            );

        #[cfg(feature = "northstar_debug")]
        app.insert_resource(MobNavDebugConfig { draw_paths: true });
    }
}

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MobNavUpdateSet {
    EnsureState,
    EnqueueRequests,
    PlanPaths,
    ApplyResults,
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MobNavAgent {
    pub movement_mode: MobNavMovementMode,
    pub arrival_tolerance: f32,
    pub max_speed: f32,
}

impl Default for MobNavAgent {
    fn default() -> Self {
        Self {
            movement_mode: MobNavMovementMode::Ground,
            arrival_tolerance: 0.35,
            max_speed: 5.0,
        }
    }
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MobNavMovementMode {
    #[default]
    Ground,
    FlyingLineOfSight,
}

#[derive(Component, Reflect, Debug, Clone, Copy)]
#[reflect(Component)]
pub struct MobNavGoal {
    pub position: Vec3,
}

impl MobNavGoal {
    #[allow(dead_code)]
    pub fn new(position: Vec3) -> Self {
        Self { position }
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct MobNavRepath;

#[derive(Component, Reflect, Debug, Clone, PartialEq)]
#[reflect(Component)]
pub struct MobNavPath {
    pub waypoints: Vec<Vec3>,
    pub next_waypoint: usize,
}

impl MobNavPath {
    #[allow(dead_code)]
    pub fn new(waypoints: Vec<Vec3>) -> Self {
        Self {
            waypoints,
            next_waypoint: 0,
        }
    }

    #[allow(dead_code)]
    pub fn next(&self) -> Option<Vec3> {
        self.waypoints.get(self.next_waypoint).copied()
    }

    #[allow(dead_code)]
    pub fn is_complete(&self) -> bool {
        self.next_waypoint >= self.waypoints.len()
    }

    #[allow(dead_code)]
    pub fn advance_if_reached(&mut self, position: Vec3, tolerance: f32) -> bool {
        let Some(target) = self.next() else {
            return false;
        };
        if position.distance(target) <= tolerance {
            self.next_waypoint += 1;
            return true;
        }
        false
    }
}

#[derive(Component, Reflect, Debug, Clone, Copy, Default)]
#[reflect(Component)]
pub struct MobNavSteering {
    pub desired_velocity: Vec3,
}

#[derive(Component, Reflect, Debug, Clone, Copy, PartialEq, Eq, Default)]
#[reflect(Component)]
pub enum MobNavStatus {
    #[default]
    Idle,
    Planning,
    FollowingPath,
    Arrived,
    Blocked,
}

#[derive(Resource, Reflect, Debug, Clone, Default)]
#[reflect(Resource)]
pub struct MobNavDebugConfig {
    pub draw_paths: bool,
}

#[derive(Reflect, Debug, Clone, Copy, PartialEq, Eq)]
pub enum MobNavPlanReason {
    GoalChanged,
    AgentChanged,
    ExplicitRepath,
}

#[derive(Message, Debug, Clone, Copy)]
pub struct MobNavPlanRequest {
    pub request_id: u64,
    pub entity: Entity,
    pub from: Vec3,
    pub to: Vec3,
    pub movement_mode: MobNavMovementMode,
    #[allow(dead_code)]
    pub reason: MobNavPlanReason,
}

#[derive(Debug, Clone)]
pub enum MobNavPlanResultKind {
    Path(Vec<Vec3>),
    Blocked,
}

#[derive(Message, Debug, Clone)]
pub struct MobNavPlanResult {
    pub request_id: u64,
    pub entity: Entity,
    pub result: MobNavPlanResultKind,
}

#[derive(Component, Debug, Clone, Copy)]
struct MobNavPendingRequest {
    request_id: u64,
}

#[derive(Resource, Default)]
struct MobNavRequestCounter(u64);

fn ensure_nav_status(
    mut commands: Commands,
    entities: Query<(Entity, Option<&MobNavStatus>, Option<&MobNavSteering>), With<MobNavAgent>>,
) {
    for (entity, status, steering) in &entities {
        let mut entity_commands = commands.entity(entity);
        if status.is_none() {
            entity_commands.insert(MobNavStatus::Idle);
        }
        if steering.is_none() {
            entity_commands.insert(MobNavSteering::default());
        }
    }
}

fn enqueue_nav_requests(
    mut commands: Commands,
    mut request_counter: ResMut<MobNavRequestCounter>,
    mut requests: MessageWriter<MobNavPlanRequest>,
    changed_query: Query<
        (Entity, &GlobalTransform, Ref<MobNavAgent>, Ref<MobNavGoal>),
        Or<(
            Added<MobNavAgent>,
            Changed<MobNavAgent>,
            Added<MobNavGoal>,
            Changed<MobNavGoal>,
        )>,
    >,
    repath_query: Query<(Entity, &GlobalTransform, &MobNavAgent, &MobNavGoal), Added<MobNavRepath>>,
) {
    let mut queued: HashSet<Entity> = HashSet::new();

    for (entity, transform, agent, goal) in &changed_query {
        let reason = if goal.is_added() || goal.is_changed() {
            MobNavPlanReason::GoalChanged
        } else {
            MobNavPlanReason::AgentChanged
        };
        queue_nav_request(
            &mut commands,
            &mut request_counter,
            &mut requests,
            entity,
            transform.translation(),
            goal.position,
            agent.movement_mode,
            reason,
        );
        queued.insert(entity);
    }

    for (entity, transform, agent, goal) in &repath_query {
        commands.entity(entity).remove::<MobNavRepath>();
        if queued.contains(&entity) {
            continue;
        }
        queue_nav_request(
            &mut commands,
            &mut request_counter,
            &mut requests,
            entity,
            transform.translation(),
            goal.position,
            agent.movement_mode,
            MobNavPlanReason::ExplicitRepath,
        );
    }
}

fn queue_nav_request(
    commands: &mut Commands,
    request_counter: &mut MobNavRequestCounter,
    requests: &mut MessageWriter<MobNavPlanRequest>,
    entity: Entity,
    from: Vec3,
    to: Vec3,
    movement_mode: MobNavMovementMode,
    reason: MobNavPlanReason,
) {
    request_counter.0 = request_counter.0.wrapping_add(1);
    let request_id = request_counter.0;
    requests.write(MobNavPlanRequest {
        request_id,
        entity,
        from,
        to,
        movement_mode,
        reason,
    });
    commands
        .entity(entity)
        .remove::<MobNavPath>()
        .insert((
            MobNavPendingRequest { request_id },
            MobNavSteering::default(),
            MobNavStatus::Planning,
        ));
}

fn built_in_flying_los_planner(
    mut requests: MessageReader<MobNavPlanRequest>,
    mut results: MessageWriter<MobNavPlanResult>,
    voxel_world: VoxelWorld<CaveWorld>,
) {
    for request in requests.read() {
        if request.movement_mode != MobNavMovementMode::FlyingLineOfSight {
            continue;
        }

        let displacement = request.to - request.from;
        let distance = displacement.length();
        if distance <= f32::EPSILON {
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Path(Vec::new()),
            });
            continue;
        }

        let Ok(direction) = Dir3::new(displacement) else {
            results.write(MobNavPlanResult {
                request_id: request.request_id,
                entity: request.entity,
                result: MobNavPlanResultKind::Blocked,
            });
            continue;
        };

        let ray = Ray3d::new(request.from, direction);
        let blocked = voxel_world.raycast(ray, &|(_pos, _vox)| true).is_some();

        let result = if blocked {
            MobNavPlanResultKind::Blocked
        } else {
            MobNavPlanResultKind::Path(vec![request.to])
        };

        results.write(MobNavPlanResult {
            request_id: request.request_id,
            entity: request.entity,
            result,
        });
    }
}

fn apply_plan_results(
    mut commands: Commands,
    mut results: MessageReader<MobNavPlanResult>,
    pending: Query<&MobNavPendingRequest>,
) {
    for result in results.read() {
        let Ok(pending_request) = pending.get(result.entity) else {
            continue;
        };
        if pending_request.request_id != result.request_id {
            continue;
        }

        match &result.result {
            MobNavPlanResultKind::Path(waypoints) => {
                let status = if waypoints.is_empty() {
                    MobNavStatus::Arrived
                } else {
                    MobNavStatus::FollowingPath
                };
                commands
                    .entity(result.entity)
                    .remove::<MobNavPendingRequest>()
                    .insert((
                        MobNavPath::new(waypoints.clone()),
                        MobNavSteering::default(),
                        status,
                    ));
            }
            MobNavPlanResultKind::Blocked => {
                commands
                    .entity(result.entity)
                    .remove::<(MobNavPendingRequest, MobNavPath)>()
                    .insert((MobNavSteering::default(), MobNavStatus::Blocked));
            }
        }
    }
}

fn follow_mob_nav_paths(
    time: Res<Time>,
    collisions: Collisions,
    mut movers: Query<
        (
            Entity,
            &MobNavAgent,
            &mut MobNavPath,
            &mut MobNavStatus,
            &mut MobNavSteering,
            &mut Transform,
            Option<&mut LinearVelocity>,
        ),
        With<MobNavAgent>,
    >,
) {
    for (entity, agent, mut path, mut status, mut steering, mut transform, linear_velocity) in
        &mut movers
    {
        let mut linear_velocity = linear_velocity;

        if *status != MobNavStatus::FollowingPath {
            continue;
        }

        let position = transform.translation;
        while let Some(next_waypoint) = path.next() {
            let reached = match agent.movement_mode {
                MobNavMovementMode::Ground => {
                    let delta_xz = Vec2::new(
                        next_waypoint.x - position.x,
                        next_waypoint.z - position.z,
                    );
                    delta_xz.length() <= agent.arrival_tolerance
                }
                MobNavMovementMode::FlyingLineOfSight => {
                    position.distance(next_waypoint) <= agent.arrival_tolerance
                }
            };
            if !reached {
                break;
            }

            path.next_waypoint += 1;
            if path.is_complete() {
                *status = MobNavStatus::Arrived;
                steering.desired_velocity = Vec3::ZERO;
                if let Some(linear_velocity) = linear_velocity.as_mut() {
                    match agent.movement_mode {
                        MobNavMovementMode::Ground => {
                            linear_velocity.0.x = 0.0;
                            linear_velocity.0.z = 0.0;
                        }
                        MobNavMovementMode::FlyingLineOfSight => {
                            linear_velocity.0 = Vec3::ZERO;
                        }
                    }
                }
                break;
            }
        }

        if *status != MobNavStatus::FollowingPath {
            continue;
        }

        let Some(next_waypoint) = path.next() else {
            *status = MobNavStatus::Arrived;
            steering.desired_velocity = Vec3::ZERO;
            if let Some(linear_velocity) = linear_velocity.as_mut() {
                match agent.movement_mode {
                    MobNavMovementMode::Ground => {
                        linear_velocity.0.x = 0.0;
                        linear_velocity.0.z = 0.0;
                    }
                    MobNavMovementMode::FlyingLineOfSight => {
                        linear_velocity.0 = Vec3::ZERO;
                    }
                }
            }
            continue;
        };

        let mut desired = next_waypoint - position;
        desired = desired.normalize_or_zero() * agent.max_speed;
        steering.desired_velocity = desired;

        if let Some(linear_velocity) = linear_velocity.as_mut() {
            match agent.movement_mode {
                MobNavMovementMode::Ground => {
                    let desired = project_horizontal_off_walls(entity, desired, &collisions);
                    linear_velocity.0.x = desired.x;
                    linear_velocity.0.z = desired.z;
                    if desired.y > GROUND_JUMP_TRIGGER_DESIRED_Y
                        && linear_velocity.0.y <= 0.1
                        && is_grounded(entity, &collisions)
                    {
                        linear_velocity.0.y = linear_velocity.0.y.max(GROUND_JUMP_SPEED);
                    }
                }
                MobNavMovementMode::FlyingLineOfSight => {
                    linear_velocity.0 = desired;
                }
            }
        } else {
            transform.translation += desired * time.delta_secs();
        }
    }
}

fn project_horizontal_off_walls(entity: Entity, desired: Vec3, collisions: &Collisions) -> Vec3 {
    let mut horizontal = Vec2::new(desired.x, desired.z);

    for pair in collisions.collisions_with(entity) {
        if !pair.generates_constraints() {
            continue;
        }

        for manifold in &pair.manifolds {
            let normal_from_entity = if pair.collider1 == entity {
                manifold.normal
            } else {
                -manifold.normal
            };
            if normal_from_entity.y.abs() > WALL_NORMAL_MAX_Y {
                continue;
            }

            let wall_normal = Vec2::new(normal_from_entity.x, normal_from_entity.z).normalize_or_zero();
            if wall_normal == Vec2::ZERO {
                continue;
            }

            let into_wall = horizontal.dot(wall_normal);
            if into_wall > 0.0 {
                horizontal -= wall_normal * into_wall;
            }
        }
    }

    Vec3::new(horizontal.x, desired.y, horizontal.y)
}

fn is_grounded(entity: Entity, collisions: &Collisions) -> bool {
    collisions.collisions_with(entity).any(|pair| {
        if !pair.generates_constraints() {
            return false;
        }

        pair.manifolds.iter().any(|manifold| {
            let normal = if pair.collider1 == entity {
                -manifold.normal
            } else {
                manifold.normal
            };
            normal.y >= GROUND_NORMAL_Y_THRESHOLD
        })
    })
}

fn stop_non_following_agents(
    mut query: Query<
        (
            &MobNavAgent,
            &MobNavStatus,
            &mut MobNavSteering,
            Option<&mut LinearVelocity>,
        ),
        (With<MobNavAgent>, Without<MobNavPath>),
    >,
) {
    for (agent, status, mut steering, linear_velocity) in &mut query {
        if *status == MobNavStatus::FollowingPath {
            continue;
        }

        steering.desired_velocity = Vec3::ZERO;
        if let Some(mut linear_velocity) = linear_velocity {
            match agent.movement_mode {
                MobNavMovementMode::Ground => {
                    linear_velocity.0.x = 0.0;
                    linear_velocity.0.z = 0.0;
                }
                MobNavMovementMode::FlyingLineOfSight => {
                    linear_velocity.0 = Vec3::ZERO;
                }
            }
        }
    }
}

fn draw_mob_nav_path_gizmos(
    mut gizmos: Gizmos,
    paths: Query<(&GlobalTransform, &MobNavPath, &MobNavStatus), With<MobNavAgent>>,
) {
    let active_color = Color::srgba(0.15, 0.85, 1.0, 0.95);
    let remaining_color = Color::srgba(1.0, 0.7, 0.2, 0.85);
    let waypoint_color = Color::srgba(1.0, 0.95, 0.25, 0.9);
    let lift = Vec3::Y * 0.08;
    let cross_half = 0.12;

    for (transform, path, status) in &paths {
        if path.next_waypoint >= path.waypoints.len() {
            continue;
        }

        let mut from = transform.translation() + lift;
        for (index, waypoint) in path.waypoints.iter().enumerate().skip(path.next_waypoint) {
            let to = *waypoint + lift;
            let color = if index == path.next_waypoint && *status == MobNavStatus::FollowingPath {
                active_color
            } else {
                remaining_color
            };
            gizmos.line(from, to, color);
            gizmos.line(
                to + Vec3::new(-cross_half, 0.0, 0.0),
                to + Vec3::new(cross_half, 0.0, 0.0),
                waypoint_color,
            );
            gizmos.line(
                to + Vec3::new(0.0, 0.0, -cross_half),
                to + Vec3::new(0.0, 0.0, cross_half),
                waypoint_color,
            );
            from = to;
        }
    }
}

fn mob_nav_debug_enabled(debug_config: Res<MobNavDebugConfig>) -> bool {
    debug_config.draw_paths
}

fn toggle_mob_nav_path_debug(
    keys: Res<ButtonInput<KeyCode>>,
    mut debug_config: ResMut<MobNavDebugConfig>,
) {
    if keys.just_pressed(KeyCode::F7) {
        debug_config.draw_paths = !debug_config.draw_paths;
        info!(
            "MobNav path debug {} (toggle: F7)",
            if debug_config.draw_paths {
                "enabled"
            } else {
                "disabled"
            }
        );
    }
}

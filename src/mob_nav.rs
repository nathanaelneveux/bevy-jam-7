use avian3d::prelude::LinearVelocity;
use bevy::prelude::*;
use bevy_voxel_world::prelude::VoxelWorld;
use std::collections::HashSet;

use crate::cave_world::CaveWorld;

pub struct MobNavPlugin;

impl Plugin for MobNavPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<MobNavRequestCounter>()
            .register_type::<MobNavAgent>()
            .register_type::<MobNavGoal>()
            .register_type::<MobNavPath>()
            .register_type::<MobNavSteering>()
            .register_type::<MobNavStatus>()
            .register_type::<MobNavMovementMode>()
            .register_type::<MobNavRepath>()
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
                FixedUpdate,
                (follow_mob_nav_paths, stop_non_following_agents),
            );
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
        .insert((MobNavPendingRequest { request_id }, MobNavStatus::Planning));
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
    mut movers: Query<
        (
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
    for (agent, mut path, mut status, mut steering, mut transform, linear_velocity) in &mut movers {
        let mut linear_velocity = linear_velocity;

        if *status != MobNavStatus::FollowingPath {
            continue;
        }

        let position = transform.translation;

        while path.advance_if_reached(position, agent.arrival_tolerance) {
            if path.is_complete() {
                *status = MobNavStatus::Arrived;
                steering.desired_velocity = Vec3::ZERO;
                if let Some(linear_velocity) = linear_velocity.as_mut() {
                    linear_velocity.0 = Vec3::ZERO;
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
                linear_velocity.0 = Vec3::ZERO;
            }
            continue;
        };

        let mut desired = next_waypoint - position;
        desired = desired.normalize_or_zero() * agent.max_speed;
        steering.desired_velocity = desired;

        if let Some(linear_velocity) = linear_velocity.as_mut() {
            linear_velocity.0 = desired;
        } else {
            transform.translation += desired * time.delta_secs();
        }
    }
}

fn stop_non_following_agents(
    mut query: Query<
        (
            &MobNavStatus,
            &mut MobNavSteering,
            Option<&mut LinearVelocity>,
        ),
        (With<MobNavAgent>, Without<MobNavPath>),
    >,
) {
    for (status, mut steering, linear_velocity) in &mut query {
        if *status == MobNavStatus::FollowingPath {
            continue;
        }

        steering.desired_velocity = Vec3::ZERO;
        if let Some(mut linear_velocity) = linear_velocity {
            linear_velocity.0 = Vec3::ZERO;
        }
    }
}

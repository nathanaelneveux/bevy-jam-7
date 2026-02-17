use bevy::prelude::*;
use serde::Deserialize;
use std::f32::consts::TAU;

use crate::{
    mob_nav::{MobNavAgent, MobNavGoal, MobNavRepath, MobNavStatus, MobNavUpdateSet},
    player_controller::Player,
};

const AI_MIN_SECONDS: f32 = 0.01;
const AI_MIN_DISTANCE: f32 = 0.05;
const AI_MAX_DISTANCE: f32 = 256.0;
const AI_DEFAULT_SEED: u32 = 0x9E37_79B9;

pub(crate) struct EnemyAiPlugin;

impl Plugin for EnemyAiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, update_enemy_ai.after(MobNavUpdateSet::ApplyResults));
    }
}

#[derive(Component, Clone, Copy, Debug, Deserialize)]
pub(crate) struct EnemyAiPersonality {
    pub(crate) blocked_to_idle_secs: f32,
    pub(crate) idle_to_patrol_secs: f32,
    pub(crate) engage_distance: f32,
    pub(crate) attack_enter_distance: f32,
    pub(crate) attack_exit_distance: f32,
    pub(crate) patrol_radius_min: f32,
    pub(crate) patrol_radius_max: f32,
    pub(crate) patrol_retarget_secs: f32,
    pub(crate) goal_update_distance: f32,
    pub(crate) preferred_distance: f32,
}

impl Default for EnemyAiPersonality {
    fn default() -> Self {
        Self {
            blocked_to_idle_secs: 0.75,
            idle_to_patrol_secs: 1.4,
            engage_distance: 42.0,
            attack_enter_distance: 1.8,
            attack_exit_distance: 2.4,
            patrol_radius_min: 3.0,
            patrol_radius_max: 9.0,
            patrol_retarget_secs: 2.2,
            goal_update_distance: 1.0,
            preferred_distance: 0.0,
        }
    }
}

impl EnemyAiPersonality {
    pub(crate) fn sanitized(self) -> Self {
        let blocked_to_idle_secs = self.blocked_to_idle_secs.clamp(AI_MIN_SECONDS, 20.0);
        let idle_to_patrol_secs = self.idle_to_patrol_secs.clamp(AI_MIN_SECONDS, 30.0);
        let engage_distance = self.engage_distance.clamp(AI_MIN_DISTANCE, AI_MAX_DISTANCE);
        let attack_enter_distance = self
            .attack_enter_distance
            .clamp(AI_MIN_DISTANCE, AI_MAX_DISTANCE);
        let attack_exit_distance = self
            .attack_exit_distance
            .max(attack_enter_distance)
            .clamp(AI_MIN_DISTANCE, AI_MAX_DISTANCE);
        let patrol_radius_min = self.patrol_radius_min.clamp(0.0, AI_MAX_DISTANCE);
        let patrol_radius_max = self
            .patrol_radius_max
            .max(patrol_radius_min)
            .clamp(0.0, AI_MAX_DISTANCE);
        let patrol_retarget_secs = self.patrol_retarget_secs.clamp(AI_MIN_SECONDS, 30.0);
        let goal_update_distance = self
            .goal_update_distance
            .clamp(AI_MIN_DISTANCE, AI_MAX_DISTANCE);
        let preferred_distance = self.preferred_distance.clamp(0.0, AI_MAX_DISTANCE);

        Self {
            blocked_to_idle_secs,
            idle_to_patrol_secs,
            engage_distance: engage_distance.max(attack_exit_distance),
            attack_enter_distance,
            attack_exit_distance,
            patrol_radius_min,
            patrol_radius_max,
            patrol_retarget_secs,
            goal_update_distance,
            preferred_distance,
        }
    }
}

#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Default)]
pub(crate) enum EnemyAiState {
    #[default]
    Idle,
    Patrol,
    Chase,
    Attack,
    Blocked,
}

#[derive(Component, Clone, Copy, Debug)]
pub(crate) struct EnemyAiBrain {
    pub(crate) state: EnemyAiState,
    state_elapsed_secs: f32,
    idle_elapsed_secs: f32,
    patrol_elapsed_secs: f32,
    rng: u32,
}

impl Default for EnemyAiBrain {
    fn default() -> Self {
        Self::seeded(AI_DEFAULT_SEED)
    }
}

impl EnemyAiBrain {
    pub(crate) fn seeded(seed: u32) -> Self {
        let seed = if seed == 0 { AI_DEFAULT_SEED } else { seed };
        Self {
            state: EnemyAiState::Idle,
            state_elapsed_secs: 0.0,
            idle_elapsed_secs: 0.0,
            patrol_elapsed_secs: 0.0,
            rng: seed,
        }
    }

    fn advance(&mut self, dt: f32) {
        self.state_elapsed_secs += dt;
        if self.state == EnemyAiState::Idle {
            self.idle_elapsed_secs += dt;
        }
        if self.state == EnemyAiState::Patrol {
            self.patrol_elapsed_secs += dt;
        }
    }

    fn transition(&mut self, next: EnemyAiState) {
        if self.state == next {
            return;
        }

        self.state = next;
        self.state_elapsed_secs = 0.0;
        if next != EnemyAiState::Idle {
            self.idle_elapsed_secs = 0.0;
        }
        if next != EnemyAiState::Patrol {
            self.patrol_elapsed_secs = 0.0;
        }
    }
}

pub(crate) fn desired_engage_goal(
    current_position: Vec3,
    player_position: Vec3,
    preferred_distance: f32,
) -> Vec3 {
    if preferred_distance <= AI_MIN_DISTANCE {
        return player_position;
    }

    let mut away = Vec2::new(
        current_position.x - player_position.x,
        current_position.z - player_position.z,
    );
    if away.length_squared() <= 0.0001 {
        away = Vec2::X;
    }
    let offset = away.normalize() * preferred_distance;
    Vec3::new(
        player_position.x + offset.x,
        player_position.y,
        player_position.z + offset.y,
    )
}

fn update_enemy_ai(
    mut commands: Commands,
    time: Res<Time>,
    player: Query<&GlobalTransform, With<Player>>,
    mut enemies: Query<
        (
            Entity,
            &EnemyAiPersonality,
            &MobNavStatus,
            &GlobalTransform,
            &mut MobNavGoal,
            &mut EnemyAiBrain,
        ),
        With<MobNavAgent>,
    >,
) {
    let Some(player_position) = player.iter().next().map(|player| player.translation()) else {
        return;
    };
    let dt = time.delta_secs();

    for (entity, personality, status, transform, mut goal, mut brain) in &mut enemies {
        brain.advance(dt);
        let position = transform.translation();
        let to_player_sq = planar_distance_squared(position, player_position);
        let engage_sq = sqr(personality.engage_distance);
        let attack_sq = if brain.state == EnemyAiState::Attack {
            sqr(personality.attack_exit_distance)
        } else {
            sqr(personality.attack_enter_distance)
        };

        if *status == MobNavStatus::Blocked {
            if brain.state != EnemyAiState::Blocked {
                brain.transition(EnemyAiState::Blocked);
            }
            if brain.state_elapsed_secs >= personality.blocked_to_idle_secs {
                commands.entity(entity).insert(MobNavRepath);
                brain.transition(EnemyAiState::Idle);
            }
            continue;
        }
        if brain.state == EnemyAiState::Blocked {
            brain.transition(EnemyAiState::Idle);
        }

        if to_player_sq <= attack_sq {
            brain.transition(EnemyAiState::Attack);
            continue;
        }
        if brain.state == EnemyAiState::Attack {
            brain.transition(EnemyAiState::Chase);
        }

        if to_player_sq <= engage_sq {
            if brain.state != EnemyAiState::Chase {
                brain.transition(EnemyAiState::Chase);
            }
            let chase_goal =
                desired_engage_goal(position, player_position, personality.preferred_distance);
            set_goal_if_far(&mut goal, chase_goal, personality.goal_update_distance);
            continue;
        }

        if brain.state != EnemyAiState::Patrol && brain.state != EnemyAiState::Idle {
            brain.transition(EnemyAiState::Idle);
        }

        if brain.state == EnemyAiState::Idle
            && brain.idle_elapsed_secs >= personality.idle_to_patrol_secs
        {
            brain.transition(EnemyAiState::Patrol);
        }

        if brain.state != EnemyAiState::Patrol {
            continue;
        }

        let should_retarget_patrol = *status == MobNavStatus::Arrived
            || *status == MobNavStatus::Idle
            || brain.patrol_elapsed_secs >= personality.patrol_retarget_secs;
        if !should_retarget_patrol {
            continue;
        }

        let patrol_goal = next_patrol_goal(position, personality, &mut brain.rng);
        set_goal_if_far(&mut goal, patrol_goal, personality.goal_update_distance);
        brain.patrol_elapsed_secs = 0.0;
    }
}

fn set_goal_if_far(goal: &mut MobNavGoal, target: Vec3, min_delta: f32) {
    if goal.position.distance_squared(target) >= sqr(min_delta) {
        goal.position = target;
    }
}

fn next_patrol_goal(origin: Vec3, personality: &EnemyAiPersonality, rng: &mut u32) -> Vec3 {
    let angle = rand01(rng) * TAU;
    let radius = personality.patrol_radius_min
        + (personality.patrol_radius_max - personality.patrol_radius_min) * rand01(rng);

    Vec3::new(
        origin.x + angle.cos() * radius,
        origin.y,
        origin.z + angle.sin() * radius,
    )
}

fn rand01(state: &mut u32) -> f32 {
    *state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    ((*state >> 8) as f32) * (1.0 / ((u32::MAX >> 8) as f32))
}

fn planar_distance_squared(a: Vec3, b: Vec3) -> f32 {
    let dx = a.x - b.x;
    let dz = a.z - b.z;
    dx * dx + dz * dz
}

fn sqr(value: f32) -> f32 {
    value * value
}

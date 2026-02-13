use bevy::prelude::*;
use bevy_northstar::prelude::{AgentOfGrid, AgentPos, Blocking};

use crate::mob_nav::{MobNavAgent, MobNavMovementMode};

use super::grid::{MobNavNorthstarRollingGrid, world_to_local};

pub(crate) fn sync_ground_agents_to_northstar(
    mut commands: Commands,
    rolling: Res<MobNavNorthstarRollingGrid>,
    agents: Query<
        (
            Entity,
            &MobNavAgent,
            &GlobalTransform,
            Option<&AgentOfGrid>,
            Option<&AgentPos>,
            Has<Blocking>,
        ),
        With<MobNavAgent>,
    >,
    mut removed_agents: RemovedComponents<MobNavAgent>,
) {
    for entity in removed_agents.read() {
        if let Ok(mut entity_commands) = commands.get_entity(entity) {
            entity_commands.remove::<(AgentOfGrid, AgentPos, Blocking)>();
        }
    }

    let Some(grid_entity) = rolling.grid_entity else {
        for (entity, _, _, _, _, _) in &agents {
            if let Ok(mut entity_commands) = commands.get_entity(entity) {
                entity_commands.remove::<(AgentOfGrid, AgentPos, Blocking)>();
            }
        }
        return;
    };

    for (entity, agent, transform, agent_of_grid, agent_pos, has_blocking) in &agents {
        if agent.movement_mode != MobNavMovementMode::Ground || !rolling.initialized {
            if let Ok(mut entity_commands) = commands.get_entity(entity) {
                entity_commands.remove::<(AgentOfGrid, AgentPos, Blocking)>();
            }
            continue;
        }

        let world = transform.translation().floor().as_ivec3();
        let Some(local) = world_to_local(world, rolling.min_world, rolling.max_world) else {
            if let Ok(mut entity_commands) = commands.get_entity(entity) {
                entity_commands.remove::<(AgentOfGrid, AgentPos, Blocking)>();
            }
            continue;
        };

        let same_grid = agent_of_grid.is_some_and(|current| current.0 == grid_entity);
        let same_pos = agent_pos.is_some_and(|current| current.0 == local);
        if same_grid && same_pos && has_blocking {
            continue;
        }

        commands
            .entity(entity)
            .insert((AgentOfGrid(grid_entity), AgentPos(local), Blocking));
    }
}

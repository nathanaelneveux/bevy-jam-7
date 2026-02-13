mod config;
#[cfg(feature = "northstar_debug")]
mod debug;
mod grid;
mod planner;
mod sync;

use bevy::prelude::*;
use bevy_northstar::prelude::{
    CardinalIsoNeighborhood, NorthstarPlugin, NorthstarPluginSettings, PathingSet,
};

use crate::mob_nav::MobNavUpdateSet;

pub use config::MobNavNorthstarConfig;

pub struct MobNavNorthstarPlugin;

impl Plugin for MobNavNorthstarPlugin {
    fn build(&self, app: &mut App) {
        if app
            .world()
            .get_resource::<NorthstarPluginSettings>()
            .is_none()
        {
            app.insert_resource(NorthstarPluginSettings {
                // We consume full paths ourselves instead of using NextPos stepping.
                max_collision_avoidance_agents_per_frame: 0,
                ..default()
            });
        }

        app.init_resource::<MobNavNorthstarConfig>()
            .init_resource::<grid::MobNavNorthstarRollingGrid>()
            .add_plugins(NorthstarPlugin::<CardinalIsoNeighborhood>::default())
            .add_systems(
                Update,
                sync::sync_ground_agents_to_northstar.before(MobNavUpdateSet::PlanPaths),
            )
            .add_systems(
                Update,
                planner::plan_ground_paths_with_northstar
                    .in_set(MobNavUpdateSet::PlanPaths)
                    .before(PathingSet),
            )
            .add_systems(
                Update,
                planner::collect_northstar_plan_results
                    .after(PathingSet)
                    .before(MobNavUpdateSet::ApplyResults),
            );

        #[cfg(feature = "northstar_debug")]
        app.init_resource::<debug::MobNavNorthstarDebugConfig>()
            .init_resource::<debug::MobNavNorthstarDebugCache>()
            .register_type::<debug::MobNavNorthstarDebugConfig>()
            .add_systems(
                Update,
                (
                    debug::toggle_northstar_grid_debug,
                    debug::refresh_northstar_grid_debug_cache.after(MobNavUpdateSet::PlanPaths),
                    debug::draw_northstar_grid_debug,
                )
                    .chain(),
            );
    }
}

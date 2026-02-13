use bevy::prelude::*;
use bevy_northstar::prelude::PathfindMode;

use crate::cave_noise::{CEILING_MAX_Y, FLOOR_MIN_Y};

#[derive(Resource, Debug, Clone)]
pub struct MobNavNorthstarConfig {
    pub grid_width: u32,
    pub grid_height: u32,
    pub grid_depth: u32,
    pub recenter_margin_voxels: i32,
    pub chunk_size: u32,
    pub chunk_depth: u32,
    pub agent_height_voxels: i32,
    pub snap_search_radius_voxels: i32,
    pub min_world_y: i32,
    pub max_world_y: i32,
    pub waypoint_center_offset: Vec3,
    pub pathfind_mode: PathfindMode,
    pub allow_partial_paths: bool,
}

impl Default for MobNavNorthstarConfig {
    fn default() -> Self {
        Self {
            grid_width: 128,
            grid_height: 32,
            grid_depth: 128,
            recenter_margin_voxels: 20,
            chunk_size: 8,
            chunk_depth: 8,
            agent_height_voxels: 2,
            snap_search_radius_voxels: 24,
            min_world_y: FLOOR_MIN_Y - 4,
            max_world_y: CEILING_MAX_Y + 8,
            waypoint_center_offset: Vec3::splat(0.5),
            pathfind_mode: PathfindMode::Refined,
            allow_partial_paths: false,
        }
    }
}

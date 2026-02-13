use bevy::prelude::*;
use bevy_voxel_world::prelude::VoxelWorld;

use crate::cave_world::CaveWorld;

use super::MobNavNorthstarConfig;
use super::grid::{MobNavNorthstarRollingGrid, is_walkable_cell};

#[derive(Resource, Reflect, Debug, Clone)]
#[reflect(Resource)]
pub(crate) struct MobNavNorthstarDebugConfig {
    pub enabled: bool,
    pub show_passable: bool,
    pub show_blocked: bool,
    pub marker_half_size: f32,
    pub passable_color: Color,
    pub blocked_color: Color,
}

impl Default for MobNavNorthstarDebugConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            show_passable: true,
            show_blocked: false,
            marker_half_size: 0.16,
            passable_color: Color::srgba(0.15, 0.95, 0.25, 0.8),
            blocked_color: Color::srgba(0.95, 0.15, 0.15, 0.25),
        }
    }
}

#[derive(Resource, Default)]
pub(crate) struct MobNavNorthstarDebugCache {
    passable_centers: Vec<Vec3>,
    blocked_centers: Vec<Vec3>,
    min_world: IVec3,
    max_world: IVec3,
    has_grid: bool,
}

pub(crate) fn toggle_northstar_grid_debug(
    keys: Res<ButtonInput<KeyCode>>,
    mut debug_config: ResMut<MobNavNorthstarDebugConfig>,
) {
    if keys.just_pressed(KeyCode::F8) {
        debug_config.enabled = !debug_config.enabled;
        info!(
            "Northstar debug grid {} (toggle: F8)",
            if debug_config.enabled {
                "enabled"
            } else {
                "disabled"
            }
        );
    }
}

pub(crate) fn refresh_northstar_grid_debug_cache(
    rolling: Res<MobNavNorthstarRollingGrid>,
    config: Res<MobNavNorthstarConfig>,
    voxel_world: VoxelWorld<CaveWorld>,
    mut cache: ResMut<MobNavNorthstarDebugCache>,
) {
    if !rolling.is_changed() && !config.is_changed() {
        return;
    }

    cache.passable_centers.clear();
    cache.blocked_centers.clear();

    if !rolling.initialized {
        cache.has_grid = false;
        return;
    }

    cache.has_grid = true;
    cache.min_world = rolling.min_world;
    cache.max_world = rolling.max_world;

    let agent_height_voxels = config.agent_height_voxels.max(1);
    for y in cache.min_world.y..=cache.max_world.y {
        for z in cache.min_world.z..=cache.max_world.z {
            for x in cache.min_world.x..=cache.max_world.x {
                let world = IVec3::new(x, y, z);
                let center = Vec3::new(x as f32 + 0.5, y as f32 + 0.5, z as f32 + 0.5);
                if is_walkable_cell(&voxel_world, world, agent_height_voxels) {
                    cache.passable_centers.push(center);
                } else {
                    cache.blocked_centers.push(center);
                }
            }
        }
    }
}

pub(crate) fn draw_northstar_grid_debug(
    mut gizmos: Gizmos,
    debug: Res<MobNavNorthstarDebugConfig>,
    cache: Res<MobNavNorthstarDebugCache>,
) {
    if !debug.enabled || !cache.has_grid {
        return;
    }

    let full_min = Vec3::new(
        cache.min_world.x as f32,
        cache.min_world.y as f32,
        cache.min_world.z as f32,
    );
    let full_max = Vec3::new(
        (cache.max_world.x + 1) as f32,
        (cache.max_world.y + 1) as f32,
        (cache.max_world.z + 1) as f32,
    );
    draw_debug_aabb(&mut gizmos, full_min, full_max, Color::srgba(0.2, 0.8, 1.0, 0.35));

    let marker_half = debug.marker_half_size.max(0.02);
    if debug.show_passable {
        for &center in &cache.passable_centers {
            draw_debug_cross(&mut gizmos, center, marker_half, debug.passable_color);
        }
    }
    if debug.show_blocked {
        for &center in &cache.blocked_centers {
            draw_debug_cross(&mut gizmos, center, marker_half, debug.blocked_color);
        }
    }
}

fn draw_debug_cross(gizmos: &mut Gizmos, center: Vec3, marker_half: f32, color: Color) {
    gizmos.line(
        center + Vec3::new(-marker_half, 0.0, 0.0),
        center + Vec3::new(marker_half, 0.0, 0.0),
        color,
    );
    gizmos.line(
        center + Vec3::new(0.0, 0.0, -marker_half),
        center + Vec3::new(0.0, 0.0, marker_half),
        color,
    );
    gizmos.line(
        center + Vec3::new(0.0, -marker_half, 0.0),
        center + Vec3::new(0.0, marker_half, 0.0),
        color,
    );
}

fn draw_debug_aabb(gizmos: &mut Gizmos, min: Vec3, max: Vec3, color: Color) {
    let p000 = min;
    let p001 = Vec3::new(min.x, min.y, max.z);
    let p010 = Vec3::new(min.x, max.y, min.z);
    let p011 = Vec3::new(min.x, max.y, max.z);
    let p100 = Vec3::new(max.x, min.y, min.z);
    let p101 = Vec3::new(max.x, min.y, max.z);
    let p110 = Vec3::new(max.x, max.y, min.z);
    let p111 = max;

    gizmos.line(p000, p001, color);
    gizmos.line(p000, p010, color);
    gizmos.line(p000, p100, color);
    gizmos.line(p111, p110, color);
    gizmos.line(p111, p101, color);
    gizmos.line(p111, p011, color);
    gizmos.line(p001, p011, color);
    gizmos.line(p001, p101, color);
    gizmos.line(p010, p011, color);
    gizmos.line(p010, p110, color);
    gizmos.line(p100, p101, color);
    gizmos.line(p100, p110, color);
}

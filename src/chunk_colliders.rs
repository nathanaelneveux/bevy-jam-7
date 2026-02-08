use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy_voxel_world::prelude::{Chunk, VoxelWorld, WorldVoxel};

use crate::cave_world::CaveWorld;

const CHUNK_COLLIDER_VOXEL_SIZE: f32 = 1.0;

pub struct ChunkColliderPlugin;

impl Plugin for ChunkColliderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ChunkColliderCache>()
            .add_systems(Update, (sync_chunk_colliders, clear_chunk_colliders));
    }
}

#[derive(Resource, Default)]
struct ChunkColliderCache(HashMap<u64, Collider>);

fn sync_chunk_colliders(
    mut commands: Commands,
    mut chunk_collider_cache: ResMut<ChunkColliderCache>,
    voxel_world: VoxelWorld<CaveWorld>,
    chunk_meshes: Query<
        (Entity, &Chunk<CaveWorld>),
        (
            With<Mesh3d>,
            Or<(Added<Mesh3d>, Changed<Mesh3d>, Added<Chunk<CaveWorld>>)>,
        ),
    >,
) {
    for (entity, chunk) in &chunk_meshes {
        let Some(chunk_data) = voxel_world.get_chunk_data(chunk.position) else {
            continue;
        };

        if chunk_data.is_empty() {
            continue;
        }

        let voxels_hash = chunk_data.voxels_hash();
        let collider = if let Some(cached) = chunk_collider_cache.0.get(&voxels_hash) {
            cached.clone()
        } else {
            let shape = chunk_data.data_shape();
            let [sx, sy, sz] = shape.to_array();
            if sx < 3 || sy < 3 || sz < 3 {
                continue;
            }

            let voxel_coords = if let Some(voxels) = chunk_data.voxels_arc() {
                collect_solid_voxel_coordinates(voxels.as_ref(), sx, sy, sz)
            } else if chunk_data.is_full() {
                full_chunk_coordinates(sx, sy, sz)
            } else {
                continue;
            };

            if voxel_coords.is_empty() {
                continue;
            }

            let collider =
                Collider::voxels(Vec3::splat(CHUNK_COLLIDER_VOXEL_SIZE), &voxel_coords);
            chunk_collider_cache.0.insert(voxels_hash, collider.clone());
            collider
        };

        commands.entity(entity).insert((RigidBody::Static, collider));
    }
}

fn collect_solid_voxel_coordinates<I: Copy + PartialEq>(
    voxels: &[WorldVoxel<I>],
    sx: u32,
    sy: u32,
    sz: u32,
) -> Vec<IVec3> {
    let mut coords = Vec::new();
    let yz_stride = sx * sy;

    // Skip the padding layer around each chunk; only interior voxels are actual chunk content.
    for z in 1..(sz - 1) {
        for y in 1..(sy - 1) {
            for x in 1..(sx - 1) {
                let index = (x + sx * y + yz_stride * z) as usize;
                if matches!(voxels[index], WorldVoxel::Solid(_)) {
                    coords.push(IVec3::new(x as i32, y as i32, z as i32));
                }
            }
        }
    }

    coords
}

fn full_chunk_coordinates(sx: u32, sy: u32, sz: u32) -> Vec<IVec3> {
    let mut coords = Vec::new();

    for z in 1..(sz - 1) {
        for y in 1..(sy - 1) {
            for x in 1..(sx - 1) {
                coords.push(IVec3::new(x as i32, y as i32, z as i32));
            }
        }
    }

    coords
}

fn clear_chunk_colliders(
    mut commands: Commands,
    chunks_without_mesh: Query<
        Entity,
        (
            With<Chunk<CaveWorld>>,
            Without<Mesh3d>,
            With<Collider>,
            With<RigidBody>,
        ),
    >,
) {
    for entity in &chunks_without_mesh {
        commands.entity(entity).remove::<(Collider, RigidBody)>();
    }
}

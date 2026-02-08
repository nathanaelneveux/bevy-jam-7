use std::collections::HashMap;

use avian3d::prelude::*;
use bevy::asset::AssetId;
use bevy::prelude::*;
use bevy_voxel_world::prelude::Chunk;

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
struct ChunkColliderCache(HashMap<AssetId<Mesh>, Collider>);

fn sync_chunk_colliders(
    mut commands: Commands,
    mut chunk_collider_cache: ResMut<ChunkColliderCache>,
    meshes: Res<Assets<Mesh>>,
    chunk_meshes: Query<
        (Entity, &Mesh3d),
        (
            With<Chunk<CaveWorld>>,
            Or<(Added<Mesh3d>, Changed<Mesh3d>)>,
        ),
    >,
) {
    for (entity, mesh3d) in &chunk_meshes {
        let mesh_id = mesh3d.id();
        let collider = if let Some(cached) = chunk_collider_cache.0.get(&mesh_id) {
            cached.clone()
        } else {
            let Some(mesh) = meshes.get(&mesh3d.0) else {
                continue;
            };
            let Some(collider) = Collider::voxelized_trimesh_from_mesh(
                mesh,
                CHUNK_COLLIDER_VOXEL_SIZE,
                FillMode::SurfaceOnly,
            ) else {
                continue;
            };
            chunk_collider_cache.0.insert(mesh_id, collider.clone());
            collider
        };

        commands.entity(entity).insert((RigidBody::Static, collider));
    }
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

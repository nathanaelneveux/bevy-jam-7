use std::collections::VecDeque;

use avian3d::prelude::*;
use bevy::platform::collections::HashMap;
use bevy::prelude::*;
use bevy_voxel_world::prelude::{
    Chunk, ChunkWillChangeLod, ChunkWillDespawn, NeedsDespawn, VoxelWorld, WorldVoxel,
};

use crate::cave_world::CaveWorld;

const CHUNK_COLLIDER_VOXEL_SIZE: f32 = 1.0;
const CHUNK_COLLIDER_CACHE_MAX_ENTRIES: usize = 512;

pub struct ChunkColliderPlugin;

impl Plugin for ChunkColliderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ChunkColliderCache>()
            .add_systems(PreUpdate, cleanup_chunk_assets)
            .add_systems(
                PostUpdate,
                (invalidate_chunk_colliders, ensure_chunk_colliders).chain(),
            );
    }
}

#[derive(Component, Default)]
struct ChunkColliderReady;

#[derive(Resource, Default)]
struct ChunkColliderCache {
    colliders: HashMap<u64, Collider>,
    access_order: VecDeque<u64>,
}

impl ChunkColliderCache {
    fn get(&mut self, key: u64) -> Option<Collider> {
        let collider = self.colliders.get(&key).cloned();
        if collider.is_some() {
            self.touch(key);
        }
        collider
    }

    fn insert(&mut self, key: u64, collider: Collider) {
        self.insert_with_capacity(key, collider, CHUNK_COLLIDER_CACHE_MAX_ENTRIES);
    }

    fn insert_with_capacity(&mut self, key: u64, collider: Collider, capacity: usize) {
        if capacity == 0 {
            return;
        }

        let is_new = !self.colliders.contains_key(&key);
        self.colliders.insert(key, collider);
        self.touch(key);

        if is_new
            && self.colliders.len() > capacity
            && let Some(oldest) = self.access_order.pop_front()
        {
            self.colliders.remove(&oldest);
        }
    }

    fn touch(&mut self, key: u64) {
        if let Some(pos) = self
            .access_order
            .iter()
            .position(|existing| *existing == key)
        {
            self.access_order.remove(pos);
        }
        self.access_order.push_back(key);
    }
}

fn ensure_chunk_colliders(
    mut commands: Commands,
    mut chunk_collider_cache: ResMut<ChunkColliderCache>,
    voxel_world: VoxelWorld<CaveWorld>,
    chunk_meshes: Query<
        (Entity, &Chunk<CaveWorld>),
        (
            With<Mesh3d>,
            Without<ChunkColliderReady>,
            Without<NeedsDespawn>,
        ),
    >,
) {
    for (entity, chunk) in &chunk_meshes {
        if chunk.lod_level != 0 {
            commands
                .entity(entity)
                .remove::<(RigidBody, Collider)>()
                .insert(ChunkColliderReady);
            continue;
        }

        let Some(chunk_data) = voxel_world.get_chunk_data(chunk.position) else {
            continue;
        };

        if chunk_data.is_empty() {
            commands
                .entity(entity)
                .remove::<(RigidBody, Collider)>()
                .insert(ChunkColliderReady);
            continue;
        }

        let voxels_hash = chunk_data.voxels_hash();
        let collider = if let Some(cached) = chunk_collider_cache.get(voxels_hash) {
            cached
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

            let collider = Collider::voxels(Vec3::splat(CHUNK_COLLIDER_VOXEL_SIZE), &voxel_coords);
            chunk_collider_cache.insert(voxels_hash, collider.clone());
            collider
        };

        commands
            .entity(entity)
            .insert((RigidBody::Static, collider, ChunkColliderReady));
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

fn cleanup_chunk_assets(
    mut commands: Commands,
    mut events: MessageReader<ChunkWillDespawn<CaveWorld>>,
) {
    for event in events.read() {
        if let Ok(mut entity_commands) = commands.get_entity(event.entity) {
            entity_commands.remove::<(ChunkColliderReady, Collider, RigidBody)>();
        }
    }
}

fn invalidate_chunk_colliders(
    mut commands: Commands,
    mut lod_changes: MessageReader<ChunkWillChangeLod<CaveWorld>>,
    changed_meshes: Query<Entity, (With<Chunk<CaveWorld>>, Changed<Mesh3d>)>,
) {
    let mut dirty_entities = std::collections::HashSet::new();

    for event in lod_changes.read() {
        dirty_entities.insert(event.entity);
    }

    for entity in &changed_meshes {
        dirty_entities.insert(entity);
    }

    for entity in dirty_entities {
        commands
            .entity(entity)
            .remove::<(ChunkColliderReady, Collider, RigidBody)>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cache_evicts_oldest_when_capacity_is_reached() {
        let mut cache = ChunkColliderCache::default();
        let collider = Collider::cuboid(1.0, 1.0, 1.0);

        cache.insert_with_capacity(1, collider.clone(), 2);
        cache.insert_with_capacity(2, collider.clone(), 2);
        cache.insert_with_capacity(3, collider, 2);

        assert!(cache.get(1).is_none());
        assert!(cache.get(2).is_some());
        assert!(cache.get(3).is_some());
    }

    #[test]
    fn cache_updates_duplicate_key_and_preserves_single_entry() {
        let mut cache = ChunkColliderCache::default();
        let collider_a = Collider::cuboid(1.0, 1.0, 1.0);
        let collider_b = Collider::cuboid(2.0, 2.0, 2.0);

        cache.insert_with_capacity(1, collider_a, 2);
        cache.insert_with_capacity(1, collider_b.clone(), 2);

        assert_eq!(cache.colliders.len(), 1);
        assert_eq!(cache.access_order.len(), 1);
        assert!(cache.get(1).is_some());
    }

    #[test]
    fn cache_touch_promotes_recent_key() {
        let mut cache = ChunkColliderCache::default();
        let collider = Collider::cuboid(1.0, 1.0, 1.0);

        cache.insert_with_capacity(1, collider.clone(), 2);
        cache.insert_with_capacity(2, collider.clone(), 2);
        let _ = cache.get(1);
        cache.insert_with_capacity(3, collider, 2);

        assert!(cache.get(1).is_some());
        assert!(cache.get(2).is_none());
        assert!(cache.get(3).is_some());
    }
}

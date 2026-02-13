use std::sync::Arc;

use bevy::prelude::*;
use bevy_voxel_world::custom_meshing::CHUNK_SIZE_I;
use bevy_voxel_world::prelude::*;

use crate::cave_noise::{CEILING_MAX_Y, CaveNoise, FLOOR_MIN_Y};

const FLOOR_MATERIAL: u8 = 0;
const CEILING_MATERIAL: u8 = 1;
pub(crate) const CAVE_WORLD_SPAWNING_DISTANCE: u32 = 10;
pub(crate) const CAVE_WORLD_MIN_DESPAWN_DISTANCE: u32 = 2;

pub struct CaveWorldPlugin;

impl Plugin for CaveWorldPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(VoxelWorldPlugin::with_config(CaveWorld::default()));
    }
}

#[derive(Resource, Clone)]
pub struct CaveWorld {
    noise: Arc<CaveNoise>,
}

impl Default for CaveWorld {
    fn default() -> Self {
        Self {
            noise: Arc::new(CaveNoise::new()),
        }
    }
}

impl VoxelWorldConfig for CaveWorld {
    type MaterialIndex = u8;
    type ChunkUserBundle = ();

    fn spawning_distance(&self) -> u32 {
        CAVE_WORLD_SPAWNING_DISTANCE
    }

    fn min_despawn_distance(&self) -> u32 {
        CAVE_WORLD_MIN_DESPAWN_DISTANCE
    }

    fn voxel_lookup_delegate(&self) -> VoxelLookupDelegate<Self::MaterialIndex> {
        let cave_noise = Arc::clone(&self.noise);

        Box::new(move |chunk_pos, _lod, _previous| {
            let chunk_min_y = chunk_pos.y * CHUNK_SIZE_I;
            let chunk_max_y = chunk_min_y + CHUNK_SIZE_I - 1;

            if chunk_max_y < FLOOR_MIN_Y {
                return Box::new(|_, _| WorldVoxel::Solid(FLOOR_MATERIAL));
            }

            if chunk_min_y > CEILING_MAX_Y {
                return Box::new(|_, _| WorldVoxel::Solid(CEILING_MATERIAL));
            }

            let mut cache = std::collections::HashMap::<(i32, i32), (i32, i32)>::new();
            let noise = Arc::clone(&cave_noise);

            Box::new(move |pos: IVec3, _previous| {
                let (floor_y, ceiling_y) = match cache.get(&(pos.x, pos.z)) {
                    Some(sample) => *sample,
                    None => {
                        let sample = noise.sample_column(pos.x, pos.z);
                        cache.insert((pos.x, pos.z), sample);
                        sample
                    }
                };

                if pos.y <= floor_y {
                    WorldVoxel::Solid(FLOOR_MATERIAL)
                } else if pos.y >= ceiling_y {
                    WorldVoxel::Solid(CEILING_MATERIAL)
                } else {
                    WorldVoxel::Air
                }
            })
        })
    }
}

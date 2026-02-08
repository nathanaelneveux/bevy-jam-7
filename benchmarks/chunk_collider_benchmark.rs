use std::{sync::Arc, time::Instant};

use avian3d::prelude::{Collider, FillMode};
use bevy::prelude::*;
use bevy_voxel_world::custom_meshing::generate_chunk_mesh_for_shape;
use bevy_voxel_world::prelude::{TextureIndexMapperFn, WorldVoxel};

const PADDED_CHUNK_EDGE: u32 = 34;
const INTERIOR_MIN: u32 = 1;
const INTERIOR_MAX_EXCL: u32 = PADDED_CHUNK_EDGE - 1;

fn main() {
    let voxels = build_chunk_voxels();

    let mapper: TextureIndexMapperFn<u8> = Arc::new(|_| [0, 0, 0]);
    let mesh = generate_chunk_mesh_for_shape(
        voxels.clone(),
        IVec3::ZERO,
        UVec3::splat(PADDED_CHUNK_EDGE),
        UVec3::splat(PADDED_CHUNK_EDGE),
        mapper,
    );

    let warmup_iters = 10;
    let benchmark_iters = 100;

    for _ in 0..warmup_iters {
        let coords = collect_solid_voxel_coordinates(
            voxels.as_ref(),
            PADDED_CHUNK_EDGE,
            PADDED_CHUNK_EDGE,
            PADDED_CHUNK_EDGE,
        );
        let _ = Collider::voxels(Vec3::ONE, &coords);
    }
    for _ in 0..warmup_iters {
        let _ = Collider::voxelized_trimesh_from_mesh(&mesh, 1.0, FillMode::SurfaceOnly);
    }

    let direct_start = Instant::now();
    let mut direct_coord_count = 0usize;
    for _ in 0..benchmark_iters {
        let coords = collect_solid_voxel_coordinates(
            voxels.as_ref(),
            PADDED_CHUNK_EDGE,
            PADDED_CHUNK_EDGE,
            PADDED_CHUNK_EDGE,
        );
        direct_coord_count += coords.len();
        let _ = Collider::voxels(Vec3::ONE, &coords);
    }
    let direct_duration = direct_start.elapsed();

    let mesh_start = Instant::now();
    let mut mesh_success = 0usize;
    for _ in 0..benchmark_iters {
        if Collider::voxelized_trimesh_from_mesh(&mesh, 1.0, FillMode::SurfaceOnly).is_some() {
            mesh_success += 1;
        }
    }
    let mesh_duration = mesh_start.elapsed();

    println!("iters={benchmark_iters}");
    println!(
        "direct_voxel_path: total={:?}, avg={:.3} ms, avg_coords={}",
        direct_duration,
        direct_duration.as_secs_f64() * 1000.0 / benchmark_iters as f64,
        direct_coord_count / benchmark_iters
    );
    println!(
        "voxelized_trimesh_from_mesh: total={:?}, avg={:.3} ms, ok_iters={}",
        mesh_duration,
        mesh_duration.as_secs_f64() * 1000.0 / benchmark_iters as f64,
        mesh_success
    );

    println!(
        "ratio(mesh/direct)={:.2}x",
        mesh_duration.as_secs_f64() / direct_duration.as_secs_f64()
    );
}

fn build_chunk_voxels() -> Arc<[WorldVoxel<u8>]> {
    let size = (PADDED_CHUNK_EDGE * PADDED_CHUNK_EDGE * PADDED_CHUNK_EDGE) as usize;
    let mut voxels = vec![WorldVoxel::Air; size];

    // Cave-like data: solid floor and ceiling with slight deterministic undulation.
    for z in INTERIOR_MIN..INTERIOR_MAX_EXCL {
        for x in INTERIOR_MIN..INTERIOR_MAX_EXCL {
            let xw = (x - 1) as i32;
            let zw = (z - 1) as i32;
            let floor = 8 + ((xw * 13 + zw * 7) & 3);
            let ceil = 24 - ((xw * 5 + zw * 11) & 3);

            for y in INTERIOR_MIN..INTERIOR_MAX_EXCL {
                let yw = (y - 1) as i32;
                if yw <= floor || yw >= ceil {
                    let idx = linearize(x, y, z, PADDED_CHUNK_EDGE) as usize;
                    voxels[idx] = WorldVoxel::Solid(0);
                }
            }
        }
    }

    voxels.into()
}

fn collect_solid_voxel_coordinates<I: Copy + PartialEq>(
    voxels: &[WorldVoxel<I>],
    sx: u32,
    sy: u32,
    sz: u32,
) -> Vec<IVec3> {
    let mut coords = Vec::new();

    for z in 1..(sz - 1) {
        for y in 1..(sy - 1) {
            for x in 1..(sx - 1) {
                let index = linearize(x, y, z, sx) as usize;
                if matches!(voxels[index], WorldVoxel::Solid(_)) {
                    coords.push(IVec3::new(x as i32, y as i32, z as i32));
                }
            }
        }
    }

    coords
}

fn linearize(x: u32, y: u32, z: u32, sx: u32) -> u32 {
    x + sx * y + sx * sx * z
}

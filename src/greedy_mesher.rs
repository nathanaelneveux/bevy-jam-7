//! greedy_mesher.rs  – BGM-based replacement for block_mesh mesher
//! Works on Bevy 0.15, bevy_voxel_world 0.12, binary-greedy-meshing 0.5.0

use std::collections::BTreeSet;

use bevy::{
    asset::RenderAssetUsages,
    math::IVec3,
    mesh::{Indices, Mesh, VertexAttributeValues},
    render::render_resource::PrimitiveTopology,
};
use bevy_voxel_world::{
    custom_meshing::*,
    prelude::{TextureIndexMapperFn, WorldVoxel},
    rendering::ATTRIBUTE_TEX_INDEX,
};
use binary_greedy_meshing::{self as bgm, Face};
use ndshape::ConstShape;

use tracing::trace_span;
const CHUNK_SIZE: usize = 32;

// ---------------------------------------------------------------------------
// 1. Public entry point ------------------------------------------------------
pub fn generate_greedy_chunk_mesh<I: PartialEq + Copy>(
    // use this signature because MaterialIndex is user defined and can be different per voxel world so we can't make any assumptions about it
    voxels: VoxelArray<I>,
    _pos: IVec3,
    texture_index_mapper: TextureIndexMapperFn<I>,
) -> Mesh {
    let _span = trace_span!("generate_greedy_chunk_mesh").entered();
    let (mesher, material_lookup) = greedy_quads(&voxels);

    mesh_from_greedy_quads(mesher, material_lookup, texture_index_mapper)
}

// ---------------------------------------------------------------------------
// 2.   Binary-Greedy-Meshing invocation -------------------------------------
fn greedy_quads<I: PartialEq + Copy>(voxels: &VoxelArray<I>) -> (bgm::Mesher<CHUNK_SIZE>, Vec<I>) {
    let _span = trace_span!("greedy_quads_fill_buffer").entered();
    // build the padded Z-X-Y buffer BGM expects
    let mut buf = [0u16; bgm::Mesher::<CHUNK_SIZE>::CS_P3];
    let mut material_lookup: Vec<I> = Vec::new();

    for i in 0..PaddedChunkShape::SIZE {
        // found this pattern in bevy_voxel_world chunk.rs generate function
        let chunk_block = PaddedChunkShape::delinearize(i);
        let src = voxels[i as usize];
        if let WorldVoxel::Solid(mat) = src {
            let dst = linearize(
                chunk_block[0] as usize,
                chunk_block[1] as usize,
                chunk_block[2] as usize,
            );
            let material_id = match material_lookup.iter().position(|existing| *existing == mat) {
                Some(index) => index as u16 + 1,
                None => {
                    assert!(
                        material_lookup.len() < u16::MAX as usize,
                        "greedy mesher exceeded material id capacity"
                    );
                    material_lookup.push(mat);
                    material_lookup.len() as u16
                }
            };

            buf[dst] = material_id; // non-zero id marks the voxel solid and carries material info
        }
    }

    let mut mesher = bgm::Mesher::<CHUNK_SIZE>::new();
    let transparents = BTreeSet::new();
    mesher.mesh(&buf, &transparents);
    (mesher, material_lookup)
}

/// Helper function similar to bgm::pad_linearize without the padding since PaddedChunkShape is already padded
pub fn linearize(x: usize, y: usize, z: usize) -> usize {
    z + (x) * bgm::Mesher::<CHUNK_SIZE>::CS_P + (y) * bgm::Mesher::<CHUNK_SIZE>::CS_P2
}

// ---------------------------------------------------------------------------
// 3.   Convert BGM quads → Bevy Mesh ----------------------------------------
fn mesh_from_greedy_quads<I: PartialEq + Copy>(
    mesher: bgm::Mesher<CHUNK_SIZE>,
    material_lookup: Vec<I>,
    texture_index_mapper: TextureIndexMapperFn<I>,
) -> Mesh {
    let _span = trace_span!("build_mesh_attributes").entered();
    let bgm::Mesher { quads, .. } = mesher;
    let quad_count: usize = quads.iter().map(|v| v.len()).sum();
    let num_vertices = quad_count * 4;

    let indices = bgm::indices(quad_count);
    let mut positions = Vec::with_capacity(num_vertices);
    let mut normals = Vec::with_capacity(num_vertices);
    let mut tex_coords = Vec::with_capacity(num_vertices);
    let mut material_types = Vec::with_capacity(num_vertices);
    let mut colors: Vec<[f32; 4]> = Vec::with_capacity(num_vertices);
    let mut material_indices = Vec::with_capacity(material_lookup.len() + 1);
    material_indices.push([0, 0, 0]);
    for material in &material_lookup {
        material_indices.push(texture_index_mapper(*material));
    }

    for (face_id, quads_per_face) in quads.into_iter().enumerate() {
        let face = Face::from(face_id as u8);
        let normal = face.n();

        for quad in quads_per_face {
            let verts = face.vertices_packed(quad);
            let material_type = material_indices
                .get(quad.voxel_id() as usize)
                .copied()
                .unwrap_or([0, 0, 0]);

            for vertex in verts.into_iter() {
                let [x, y, z] = vertex.xyz();
                positions.push([x as f32, y as f32, z as f32]);
                normals.push(normal);
                tex_coords.push([vertex.u() as f32, vertex.v() as f32]);

                colors.push([1.0, 1.0, 1.0, 1.0]);

                material_types.push(material_type);
            }
        }
    }

    let mut render_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );

    render_mesh.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        VertexAttributeValues::Float32x3(positions),
    );
    render_mesh.insert_attribute(
        Mesh::ATTRIBUTE_NORMAL,
        VertexAttributeValues::Float32x3(normals),
    );
    render_mesh.insert_attribute(
        Mesh::ATTRIBUTE_UV_0,
        VertexAttributeValues::Float32x2(tex_coords),
    );
    render_mesh.insert_attribute(
        ATTRIBUTE_TEX_INDEX,
        VertexAttributeValues::Uint32x3(material_types),
    );

    render_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);

    render_mesh.insert_indices(Indices::U32(indices));

    render_mesh
}
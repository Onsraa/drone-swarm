use bevy::asset::{io::Reader, AssetLoader, LoadContext};
use bevy::math::UVec3;
use bevy::reflect::TypePath;

use super::asset::MapAsset;

/// AssetLoader for MagicaVoxel `.vox` files. Reads the first model in
/// the file, swaps from MagicaVoxel's Z-up frame to Bevy's Y-up frame,
/// and rasterizes occupied voxels (any non-zero palette index) into a
/// `MapAsset` bitset. `voxel_size` defaults to 1.0 since `.vox` doesn't
/// carry a world scale.
#[derive(Default, TypePath)]
pub struct VoxAssetLoader;

#[derive(Debug)]
pub enum VoxLoadError {
    Io(std::io::Error),
    Parse(String),
    Empty,
}

impl std::fmt::Display for VoxLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "io error: {}", e),
            Self::Parse(msg) => write!(f, ".vox parse error: {}", msg),
            Self::Empty => write!(f, ".vox contained no models"),
        }
    }
}

impl std::error::Error for VoxLoadError {}

impl From<std::io::Error> for VoxLoadError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

impl AssetLoader for VoxAssetLoader {
    type Asset = MapAsset;
    type Settings = ();
    type Error = VoxLoadError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _ctx: &mut LoadContext<'_>,
    ) -> Result<MapAsset, VoxLoadError> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        let data = dot_vox::load_bytes(&bytes).map_err(|e| VoxLoadError::Parse(e.to_string()))?;
        let model = data.models.first().ok_or(VoxLoadError::Empty)?;

        // MagicaVoxel: X-Y plane horizontal, Z up.
        // Bevy:        X-Z plane horizontal, Y up.
        // Swap MV's Y -> our Z and MV's Z -> our Y so the model
        // stands upright in our world.
        let dims_x = model.size.x;
        let dims_y = model.size.z;
        let dims_z = model.size.y;
        let cells = (dims_x * dims_y * dims_z) as usize;
        let words = cells.div_ceil(32);
        let mut bitset = vec![0u32; words];
        for v in &model.voxels {
            let x = v.x as u32;
            let y = v.z as u32;
            let z = v.y as u32;
            if x >= dims_x || y >= dims_y || z >= dims_z {
                continue;
            }
            let flat = (x + y * dims_x + z * dims_x * dims_y) as usize;
            bitset[flat / 32] |= 1u32 << (flat % 32);
        }

        Ok(MapAsset {
            dims: UVec3::new(dims_x, dims_y, dims_z),
            voxel_size: 1.0,
            bitset,
        })
    }

    fn extensions(&self) -> &[&str] {
        &["vox"]
    }
}

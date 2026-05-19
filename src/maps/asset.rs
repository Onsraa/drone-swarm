use bevy::asset::{io::Reader, Asset, AssetLoader, LoadContext};
use bevy::math::UVec3;
use bevy::reflect::TypePath;

const MAGIC: &[u8; 4] = b"DVM1";
const HEADER_BYTES: usize = 4 + 4 * 3 + 4 + 4; // magic + dims + voxel_size + bitset_len

/// In-memory representation of a `.dvm` ground-truth map.
///
/// `bitset` is a packed occupancy grid: 1 bit per cell, 32 cells per
/// `u32` (flat-index `i` -> bit `i % 32` of word `i / 32`). This matches
/// the shape `GroundTruthMap::pack_bitset` already produces, so loading
/// a map into the existing GPU lidar pipeline is a memcpy.
#[derive(Asset, TypePath, Clone, Debug)]
pub struct MapAsset {
    pub dims: UVec3,
    pub voxel_size: f32,
    pub bitset: Vec<u32>,
}

#[derive(Debug)]
pub enum MapLoadError {
    Io(std::io::Error),
    BadMagic,
    Truncated,
    BitsetLenMismatch { expected: usize, found: usize },
}

impl std::fmt::Display for MapLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "io error: {}", e),
            Self::BadMagic => write!(f, ".dvm bad magic (expected DVM1)"),
            Self::Truncated => write!(f, ".dvm truncated"),
            Self::BitsetLenMismatch { expected, found } => {
                write!(f, ".dvm bitset len mismatch: header={}, dims-derived={}", found, expected)
            }
        }
    }
}

impl std::error::Error for MapLoadError {}

impl From<std::io::Error> for MapLoadError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

#[derive(Default, TypePath)]
pub struct MapAssetLoader;

impl AssetLoader for MapAssetLoader {
    type Asset = MapAsset;
    type Settings = ();
    type Error = MapLoadError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &(),
        _ctx: &mut LoadContext<'_>,
    ) -> Result<MapAsset, MapLoadError> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;
        parse_dvm(&bytes)
    }

    fn extensions(&self) -> &[&str] {
        &["dvm"]
    }
}

fn parse_dvm(bytes: &[u8]) -> Result<MapAsset, MapLoadError> {
    if bytes.len() < HEADER_BYTES {
        return Err(MapLoadError::Truncated);
    }
    if &bytes[..4] != MAGIC {
        return Err(MapLoadError::BadMagic);
    }
    let dx = u32_le(&bytes[4..8]);
    let dy = u32_le(&bytes[8..12]);
    let dz = u32_le(&bytes[12..16]);
    let voxel_size = f32::from_le_bytes(bytes[16..20].try_into().unwrap());
    let bitset_len = u32_le(&bytes[20..24]) as usize;

    let needed = HEADER_BYTES + bitset_len * 4;
    if bytes.len() < needed {
        return Err(MapLoadError::Truncated);
    }
    let cells = (dx as usize) * (dy as usize) * (dz as usize);
    let expected_len = cells.div_ceil(32);
    if bitset_len != expected_len {
        return Err(MapLoadError::BitsetLenMismatch {
            expected: expected_len,
            found: bitset_len,
        });
    }

    let mut bitset = Vec::with_capacity(bitset_len);
    for i in 0..bitset_len {
        let off = HEADER_BYTES + i * 4;
        bitset.push(u32_le(&bytes[off..off + 4]));
    }
    Ok(MapAsset {
        dims: UVec3::new(dx, dy, dz),
        voxel_size,
        bitset,
    })
}

#[inline]
fn u32_le(b: &[u8]) -> u32 {
    u32::from_le_bytes(b.try_into().unwrap())
}

/// Serialize a packed-bitset ground-truth map into the `.dvm` byte
/// layout used by `MapAssetLoader`. Used by the exporter helper and
/// also by the in-process bootstrap that seeds `assets/maps/` on a
/// fresh checkout.
pub fn encode_dvm(dims: UVec3, voxel_size: f32, bitset: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(HEADER_BYTES + bitset.len() * 4);
    out.extend_from_slice(MAGIC);
    out.extend_from_slice(&dims.x.to_le_bytes());
    out.extend_from_slice(&dims.y.to_le_bytes());
    out.extend_from_slice(&dims.z.to_le_bytes());
    out.extend_from_slice(&voxel_size.to_le_bytes());
    out.extend_from_slice(&(bitset.len() as u32).to_le_bytes());
    for w in bitset {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

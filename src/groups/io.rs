use std::path::Path;

use super::resources::{DroneGroupPresets, Preset};

/// Each line: `name|mask_lo|mask_hi`. Names are trimmed; pipes inside a
/// name are not allowed.
pub fn parse(text: &str) -> Vec<Preset> {
    let mut out = Vec::new();
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let mut parts = trimmed.splitn(3, '|');
        let (Some(name), Some(lo), Some(hi)) = (parts.next(), parts.next(), parts.next()) else {
            continue;
        };
        let (Ok(lo), Ok(hi)) = (u32::from_str_radix(lo.trim(), 16), u32::from_str_radix(hi.trim(), 16)) else {
            continue;
        };
        out.push(Preset {
            name: name.trim().to_string(),
            mask: [lo, hi],
        });
    }
    out
}

pub fn serialize(presets: &DroneGroupPresets) -> String {
    let mut out = String::new();
    out.push_str("# drone-visibility presets — name|mask_lo|mask_hi (hex u32 halves)\n");
    for p in &presets.entries {
        out.push_str(&format!("{}|{:08X}|{:08X}\n", p.name, p.mask[0], p.mask[1]));
    }
    out
}

pub fn load_from_disk(path: &Path) -> Option<Vec<Preset>> {
    std::fs::read_to_string(path).ok().map(|s| parse(&s))
}

pub fn save_to_disk(path: &Path, presets: &DroneGroupPresets) -> std::io::Result<()> {
    std::fs::write(path, serialize(presets))
}

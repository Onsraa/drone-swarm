use bevy::prelude::*;

/// Sampled past positions of a drone, drawn as a gizmo trail. Bounded
/// ring of `TRAIL_MAX_POINTS`; older samples fall off the front when
/// new ones push in. Sampled at a small interval so the line stays
/// readable at swarm scale.
#[derive(Component, Default, Debug)]
pub struct Trail {
    pub points: std::collections::VecDeque<Vec3>,
    pub last_sample_secs: f32,
}

/// One observation: peer's last-known position + time the anchor saw
/// it. Anchors use this to compute relay positioning even when peers
/// drop out of comms range.
#[derive(Clone, Copy, Debug)]
pub struct GhostPeer {
    pub last_pos: Vec3,
    pub last_seen_secs: f32,
}

/// Per-drone memory of recently-seen peers. Only Anchors read it — but
/// every drone carries one (cheap, empty by default) so the query in
/// `apply_role_steering` doesn't need an Option filter. Ghosts older
/// than `GHOST_FORGET_SECS` get dropped each frame.
#[derive(Component, Default, Debug)]
pub struct GhostMemory {
    pub peers: bevy::platform::collections::HashMap<u32, GhostPeer>,
}

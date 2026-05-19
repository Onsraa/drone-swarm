/// Default radio range in meters. A drone outside this distance from
/// both the base and every connected peer is "isolated" and won't
/// contribute to the central map.
pub const DEFAULT_COMMS_RANGE_M: f32 = 200.0;

pub const MIN_COMMS_RANGE_M: f32 = 10.0;
pub const MAX_COMMS_RANGE_M: f32 = 2000.0;

/// Height (above floor) at which the virtual base station sits when
/// `CommsSettings.base_offset` isn't explicitly set. Just for the
/// connectivity check / gizmo rendering — there's no actual base entity.
pub const BASE_DEFAULT_HEIGHT_M: f32 = 1.0;

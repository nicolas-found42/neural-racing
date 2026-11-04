//! Small allowlisted snapshots. No observations, weights, paths, or input text.
use neural_racing_core::{Env, Track};
use neural_racing_diagnostics::{json, Value};
pub fn track(track: &Track) -> Value {
    // Non-cryptographic identity of the actual geometry, including procedural tracks.
    let mut hash = 0xcbf29ce484222325u64;
    for point in &track.centerline {
        for value in [point.x, point.y] {
            for byte in value.to_bits().to_le_bytes() {
                hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
            }
        }
    }
    for width in &track.half_width {
        for byte in width.to_bits().to_le_bytes() {
            hash = (hash ^ u64::from(byte)).wrapping_mul(0x100000001b3);
        }
    }
    json!({"circuit":track.circuit.map(|c|c.slug()).unwrap_or("procedural"),
        "geometry_id":format!("{hash:016x}"),"lap_length":track.lap_length})
}
pub fn state(env: &Env) -> Value {
    json!({"step":env.steps,"x":env.x,"y":env.y,"speed":env.speed,"distance":env.distance,
        "heading":env.heading,"steer_angle":env.steer_angle,"pedal":env.pedal,
        "slip_deg":env.slip_angle_deg,"lateral_offset":env.lateral_offset,"road_half_width":env.road_half_width,
        "min_ray":env.min_ray,"laps":env.credited_laps(),"off_track":env.off_track,"goal":env.lap_done})
}

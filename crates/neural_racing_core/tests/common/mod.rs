use neural_racing_core::{
    vehicle::steering_limit, wrap_angle, Action, Env, Vec2, MAX_LATERAL_ACCEL, UNITS_PER_METRE,
    WHEELBASE,
};

/// Independent rear-axle pure pursuit, with braking distance and a grip margin.
/// Used only to establish physical track feasibility, never to train the model.
pub fn pursuit(env: &Env, cruise: f64) -> Action {
    let rear = env.position()
        - Vec2 {
            x: WHEELBASE * 0.5,
            y: 0.0,
        }
        .rotated(env.heading);
    let hit = env.track.nearest_on_centerline(rear);
    let lookahead = 2.5 * UNITS_PER_METRE + env.speed * 0.65;
    let target_arc = hit.arc + lookahead;
    // An open finish is a crossing plane. Continue the test driver's target
    // along its final tangent, so it never turns back toward a clamped endpoint.
    let target = if !env.track.closed && target_arc > env.track.lap_length {
        env.track.arc_point(env.track.lap_length)
            + env.track.arc_tangent(env.track.lap_length) * (target_arc - env.track.lap_length)
    } else {
        env.track.arc_point(target_arc)
    };
    let delta = target - rear;
    let alpha = wrap_angle(delta.angle() - env.heading);
    let wheel_angle = (2.0 * WHEELBASE * alpha.sin() / delta.norm()).atan();
    let steer = (wheel_angle / steering_limit(env.vx)).clamp(-1.0, 1.0);
    let horizon = (env.speed * env.speed / (2.0 * 8.0 * UNITS_PER_METRE) + 30.0 * UNITS_PER_METRE)
        .max(35.0 * UNITS_PER_METRE);
    let radius = env.track.tightest_corner_ahead(hit.arc, horizon);
    let desired_speed = cruise
        .min(20.0 * UNITS_PER_METRE)
        .min((0.45 * MAX_LATERAL_ACCEL * radius).sqrt());
    let throttle = ((desired_speed - env.speed) / (4.0 * UNITS_PER_METRE)).clamp(-0.9, 0.8);
    Action { steer, throttle }
}

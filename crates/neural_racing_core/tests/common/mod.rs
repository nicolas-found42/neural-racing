use neural_racing_core::{
    wrap_angle, Action, Env, Vec2, MAX_LATERAL_ACCEL, MAX_SPEED, MAX_STEER_ANGLE, UNITS_PER_METRE,
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
    let target = env.track.arc_point(hit.arc + lookahead);
    let delta = target - rear;
    let alpha = wrap_angle(delta.angle() - env.heading);
    let wheel_angle = (2.0 * WHEELBASE * alpha.sin() / delta.norm()).atan();
    let steer = (wheel_angle * (1.0 + 0.45 * (env.vx / MAX_SPEED).powi(2)) / MAX_STEER_ANGLE)
        .clamp(-1.0, 1.0);
    let horizon = (env.speed * env.speed / (2.0 * 8.0 * UNITS_PER_METRE) + 30.0 * UNITS_PER_METRE)
        .max(35.0 * UNITS_PER_METRE);
    let radius = env.track.tightest_corner_ahead(hit.arc, horizon);
    let desired_speed = cruise
        .min(20.0 * UNITS_PER_METRE)
        .min((0.45 * MAX_LATERAL_ACCEL * radius).sqrt());
    let throttle = ((desired_speed - env.speed) / (4.0 * UNITS_PER_METRE)).clamp(-0.9, 0.8);
    Action { steer, throttle }
}

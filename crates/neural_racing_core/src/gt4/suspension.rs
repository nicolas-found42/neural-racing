//! Recovered suspension numeric routines 0x00369b48 and 0x00369de8.
//! Parameter offsets and global values are explicit; no generic spring model is
//! substituted for the source branches or its square-root damper response.

use super::eefloat::{add, div, mul, sqrt};

/// Original 20 f32 fields at parameter offsets 0x00 through 0x4c.
#[derive(Clone, Copy, Debug)]
pub struct SuspensionParameters {
    pub values: [f32; 20],
}

/// Global damper settings, resolved for the original axle index argument.
#[derive(Clone, Copy, Debug, Default)]
pub struct DamperGlobals {
    pub mode_at_620dd9: u8,
    pub blend_enabled_at_620cb0: bool,
    pub negative_speed_at_620cb8: f32,
    pub negative_blend_at_620cc0: f32,
    pub positive_speed_at_620cc8: f32,
    pub positive_blend_at_620cd0: f32,
}

/// Full scalar arithmetic and branches of 0x00369b48.
pub fn damper_response(
    mut velocity: f32,
    parameters: &SuspensionParameters,
    globals: &DamperGlobals,
) -> f32 {
    let p = &parameters.values;
    if globals.mode_at_620dd9 == 0 {
        if velocity < 0.0 {
            if velocity < -1.0 {
                velocity = -1.0;
            }
            let mut response = add(p[7], mul(p[8], sqrt(add(-velocity, -p[6]))));
            if globals.blend_enabled_at_620cb0 {
                let threshold = -globals.negative_speed_at_620cb8;
                if threshold < velocity {
                    let edge = add(
                        p[7],
                        mul(p[8], sqrt(add(globals.negative_speed_at_620cb8, -p[6]))),
                    );
                    let quadratic = div(
                        mul(mul(edge, velocity), velocity),
                        mul(threshold, threshold),
                    );
                    response = add(
                        mul(add(1.0, -globals.negative_blend_at_620cc0), response),
                        mul(globals.negative_blend_at_620cc0, quadratic),
                    );
                }
            }
            response
        } else {
            if velocity > 1.0 {
                velocity = 1.0;
            }
            let mut response = add(-p[10], -mul(p[11], sqrt(add(velocity, -p[9]))));
            if globals.blend_enabled_at_620cb0 {
                let threshold = globals.positive_speed_at_620cc8;
                if threshold <= velocity {
                    return response;
                }
                let edge = add(-p[10], -mul(p[11], sqrt(add(threshold, -p[9]))));
                let quadratic = div(
                    mul(mul(edge, velocity), velocity),
                    mul(threshold, threshold),
                );
                response = add(
                    mul(add(1.0, -globals.positive_blend_at_620cd0), response),
                    mul(globals.positive_blend_at_620cd0, quadratic),
                );
            }
            response
        }
    } else if velocity < 0.0 {
        if velocity <= -p[8] {
            return p[18];
        }
        if -p[6] <= velocity {
            return div(mul(-velocity, p[7]), p[6]);
        }
        add(
            p[7],
            div(
                mul(add(-velocity, -p[6]), add(p[18], -p[7])),
                add(p[8], -p[6]),
            ),
        )
    } else if velocity > 0.0 {
        if p[11] <= velocity {
            return -p[19];
        }
        if p[9] < velocity {
            return -add(
                p[10],
                div(
                    mul(add(velocity, -p[9]), add(p[19], -p[10])),
                    add(p[11], -p[9]),
                ),
            );
        }
        div(mul(-velocity, p[10]), p[9])
    } else {
        0.0
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SuspensionState {
    pub normal_at_48: f32,
    pub travel_at_94: f32,
    pub velocity_at_98: f32,
    pub spring_at_9c: f32,
    pub damper_at_a0: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SuspensionContext {
    pub dt_at_54c: f32,
    pub inverse_dt_at_550: f32,
    pub alternate_limit_at_5be: bool,
    pub external_force_enabled_at_1076: bool,
}

/// Numeric body of 0x00369de8. The optional source callback 0x003f2650 remains
/// an explicit caller boundary and is invoked only in its original contact
/// branch. A false source flag does not invoke or substitute a callback result.
pub fn update_suspension<F: FnMut(f32) -> f32>(
    position_input: f32,
    mut spring_rate: f32,
    state: &mut SuspensionState,
    parameters: &SuspensionParameters,
    globals: &DamperGlobals,
    context: &SuspensionContext,
    mut external_force: F,
) {
    let p = &parameters.values;
    let limit = if context.alternate_limit_at_5be {
        p[17]
    } else {
        p[2]
    };
    let previous = state.travel_at_94;
    let acceleration = div(add(state.spring_at_9c, state.damper_at_a0), p[14]);
    let predicted = add(
        previous,
        mul(
            add(
                state.velocity_at_98,
                mul(mul(acceleration, 0.5), context.dt_at_54c),
            ),
            context.dt_at_54c,
        ),
    );
    let mut travel = limit;
    if predicted <= limit {
        travel = predicted;
        if predicted < p[1] {
            travel = p[1];
        }
    }
    if travel < position_input {
        state.spring_at_9c = mul(-travel, spring_rate);
        if travel == limit {
            state.velocity_at_98 = 0.0;
        } else {
            let mut velocity = mul(context.inverse_dt_at_550, add(travel, -previous));
            if velocity > 1.0 {
                velocity = 1.0;
            } else if velocity < -1.0 {
                velocity = -1.0;
            }
            state.velocity_at_98 = velocity;
        }
        state.damper_at_a0 = damper_response(state.velocity_at_98, parameters, globals);
        state.travel_at_94 = travel;
        state.normal_at_48 = 0.0;
        return;
    }
    travel = p[1];
    if p[1] <= position_input {
        travel = position_input;
    }
    let bump = add(travel, -p[0]);
    spring_rate = mul(-travel, spring_rate);
    if bump < 0.0 {
        spring_rate = add(spring_rate, mul(mul(bump, bump), p[13]));
    }
    let mut velocity = mul(context.inverse_dt_at_550, add(position_input, -previous));
    state.velocity_at_98 = velocity;
    let mut damping = damper_response(velocity, parameters, globals);
    if velocity < 0.0 && bump < 0.0 {
        if velocity < -1.0 {
            velocity = -1.0;
        }
        damping = add(damping, -mul(p[16], velocity));
    }
    if context.external_force_enabled_at_1076 {
        damping = add(damping, external_force(travel));
    }
    state.travel_at_94 = travel;
    state.spring_at_9c = spring_rate;
    state.damper_at_a0 = damping;
    state.normal_at_48 = add(add(spring_rate, damping), p[15]);
    if state.normal_at_48 < 0.0 {
        state.normal_at_48 = 0.0;
        state.damper_at_a0 = 0.0;
        state.spring_at_9c = -p[15];
    }
}

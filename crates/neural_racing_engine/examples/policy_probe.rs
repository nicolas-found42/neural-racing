//! Deterministic and sampled behaviour of a saved policy on the real circuits.

use neural_racing_core::{Action, Circuit, Env, Rng, Track, UNITS_PER_METRE};
use neural_racing_ml::Policy;

fn rollout(
    policy: &Policy,
    track: &Track,
    mut rng: Option<&mut Rng>,
) -> (f64, f64, f64, f64, bool) {
    let mut env = Env::new(track.clone(), 1);
    let (mut throttle, mut speed, mut steer, mut steps) = (0.0, 0.0, 0.0, 0u32);
    let mut crashed = false;
    while env.steps < env.max_steps() {
        let (ts, tt) = match rng.as_deref_mut() {
            Some(rng) => {
                let s = policy.sample(&env.obs, rng);
                (s.action[0], s.action[1])
            }
            None => {
                let a = policy.mean_action(&env.obs);
                (a[0], a[1])
            }
        };
        let out = env.step(Action {
            steer: ts as f64,
            throttle: tt as f64,
        });
        throttle += tt as f64;
        steer += (ts as f64).abs();
        speed += env.speed;
        steps += 1;
        if out.done {
            crashed = out.info.off_track;
            break;
        }
    }
    let n = f64::from(steps.max(1));
    (
        env.distance / (env.track.lap_length * f64::from(env.laps)),
        throttle / n,
        speed / n / UNITS_PER_METRE,
        steer / n,
        crashed,
    )
}

fn main() {
    let _diagnostics = neural_racing_diagnostics::init("policy_probe");
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "models/policy.bin".into());
    let policy = Policy::load(std::path::Path::new(&path)).expect("policy");
    println!("{path}: {} params", policy.num_params());
    for circuit in [Circuit::Monza, Circuit::Silverstone, Circuit::RedBullRing] {
        let track = circuit.build();
        let (progress, throttle, speed, steer, crashed) = rollout(&policy, &track, None);
        println!(
            "{:14} det progress {progress:.3} throttle {throttle:+.3} speed {speed:.1} m/s |steer| {steer:.3} crash {crashed}",
            format!("{circuit:?}"),
        );
        let mut rng = Rng::new(5);
        let mut sum = (0.0, 0.0, 0.0);
        for _ in 0..16 {
            let (p, t, s, _, _) = rollout(&policy, &track, Some(&mut rng));
            sum.0 += p;
            sum.1 += t;
            sum.2 += s;
        }
        println!(
            "{:14} sampled progress {:.3} throttle {:+.3} speed {:.1} m/s",
            "",
            sum.0 / 16.0,
            sum.1 / 16.0,
            sum.2 / 16.0
        );
    }
}

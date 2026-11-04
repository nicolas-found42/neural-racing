# Five-lap learning and vehicle validation

Measured on 4 November 2026 on the user's Apple M5 Pro Mac: 18 CPU cores,
20 GPU cores, 48 GB memory. These results replace the earlier single-lap,
smaller-policy measurements. Throughput depends on current system load and
training phase; the app displays achieved speed alongside the requested speed.

## Steering regression

The previous rate-limited actuator could still snap its angular velocity to zero
when a changing target crossed its angle. A deterministic test with the same
20 Hz Gaussian exploration pattern measured **86.98 rad/s²**, exceeding its
6 rad/s² acceleration limit. A filtered target and critically damped rack servo
remove that snap. The same test now passes the 6 rad/s² bound.

The trail now records real 120 Hz integration positions, rather than connecting
only the 20 Hz decision endpoints. It is not a cosmetic spline concealing the
car's motion. Smoothness rewards penalize repeated steering/pedal corrections,
excessive lateral jerk and sideslip. The policy sees its previous commands and
actuator state, so it can learn control continuity. Exploration and mistakes
remain visible while learning; this is not a claim that a fresh policy drives
like a trained professional.

## Five-lap goal and policy

Each live attempt targets five consecutive laps. Laps one through four continue
without resetting. Consecutive laps earn an extra endurance bonus and finishing
all five earns an additional 30. A dense low-speed cost, a terminal stall penalty
and a timeout penalty that exceeds the off-track penalty make parking before a
difficult corner strictly worse than attempting it. Paid checkpoint/lap
milestones cannot be farmed by reversing and recrossing.

The dashboard separates total laps, completed five-lap goals, best consecutive
streak and individual best lap time. History measures progress toward the full
five-lap goal. Circuit changes preserve learning and separate statistics.

The residual policy has **350,725 parameters**: five 256-unit residual blocks with
layer normalization, 67 inputs and three heads for steer, throttle/brake and
state value. Inputs cover previous controls, steering target, accelerations, grip
usage, wheel loads, braking distance, stint progress, and signed road headings
from 10 to 200 metres ahead. Live training uses Adam at `1e-4` with PPO: three
epochs over minibatches of 256, advantage normalization, a clipped surrogate, an
entropy bonus and gradient clipping at 1.0. A minibatch is not applied when the
measured policy KL exceeds 0.02, and any step that pushes KL above 0.05 restores
the entire update (parameters and Adam moments) instead of accepting it. Completed
attempts accumulate until the batch holds at least 1024 rows, so short crashes
cannot reduce an update to a few dozen noisy samples. GAE uses `lambda = 0.95`,
the usual PPO setting. Every 200 updates
a deterministic 1200-decision audit keeps the best mean policy; a stall, or a
drop below 40% of that best progress, restores the snapshot and a fresh Adam
state. No expert driver is used to generate training samples.

## Physics

The model uses four separate tire contact patches, each with its own velocity,
front Ackermann angle, normal load, combined friction limit and yaw lever arm.
Longitudinal transfer and lagged lateral transfer redistribute load; sublinear
load sensitivity reduces total grip under unequal loading. Tire lateral force
builds over a representative 0.45 m relaxation length. Power-limited drive,
braking, drag, downforce and the low-speed rear-axle rolling transition remain.

The transient tire/load concepts are described in the primary
[tire-dynamics study](https://pmc.ncbi.nlm.nih.gov/articles/PMC9459800/) and
[MathWorks combined-slip wheel documentation](https://www.mathworks.com/help/vdynblks/ref/combinedslipwheel2dof.html).
These sources inform the model structure; the tuning is representative, not
measured BMW tire data or a Magic Formula calibration. This remains planar:
no wheel-spin/ABS model, suspension travel, temperature, camber or gearbox.

## Live rendering and learning measurements

A release-mode integration probe renders the full 1264 × 900 UI for 30 seconds
per circuit, requesting Max, with four actual training cars and fresh weights.
Live refreshes count changing training states, not repeated paint frames.
Multiplier means simulated time **per car** divided by real time.

| Circuit | Achieved | Live refreshes/s | Render p95 | Attempts | Updates |
| --- | ---: | ---: | ---: | ---: | ---: |
| Monza | 224.4× | 39.4 | 8.89 ms | 5,099 | 1,118 |
| Silverstone | 344.4× | 41.7 | 8.99 ms | 1,426 | 260 |
| Red Bull Ring | 205.1× | 38.2 | 9.04 ms | 5,507 | 1,135 |

None completed a lap in this 30-second run. The earlier table reported laps for
a different update stack; the current three-epoch PPO update, trust-region
re-forward and policy audit cost roughly 4–6× throughput, and the learned policy
still crashes at each circuit's first complex corner. These are live exploratory
attempts, not proof that the mean policy has mastered a track. See
[current capacity](speed-capacity.md).

An independent test-only pursuit driver completes five laps on all three real
circuits. This verifies physical feasibility separately from neural learning.

## Learning evidence

Headless PPO on the small oval (16 episodes per generation, 1000 generations,
about 2.5 minutes of wall time): from generation 760 through 1000 the deterministic
policy completes the lap with progress `0.99–1.006`, no crash, roughly 200
decisions per lap at 12–13 m/s, and an explained value variance of `0.97–0.99`.
Earlier generations crash.

Live learning on Monza (seed 42, 60 seconds of wall time, 1,004 updates) moved the
deterministic policy from `0.009` to `0.107` of a lap — about eleven times further
— and the sampled cars reached 36.8 m/s mean speed with the critic already at
`0.99` explained variance. A 300-second run kept the sampled cars near 31 m/s and
restored the best mean policy once after a low-speed collapse; the deterministic
policy ended at `0.107`.

The remaining limit is control, not just training time: the policy reaches Monza's
first chicane (about 10% of the lap) and crashes there, and a separate 400-generation
headless Monza run (6,400 episodes) did not pass that corner either. Complete-lap
learning is demonstrated on the small oval; the real circuits need a curriculum or
more training than a live session provides.

## Verification

`cargo test --workspace`: **59 passed**, two explicit performance probes ignored.
The separate 90-second release performance probe also passes. Coverage includes
steering acceleration, tire-force buildup and load sensitivity, four-wheel grip
limits, no stationary pivot, rear-axle rolling, realistic acceleration/braking,
five-lap continuation and bonuses, gate replay prevention, individual lap timing,
analytic model gradients, the clipped PPO update, the oversized-step rollback,
sigma clamping, timeout and stall penalties, best-snapshot restoration, live
pause/resume/reset, circuit switching, and live rendering at 0.25×/1×/16×/1000×. Changed Rust modules pass formatting; `git diff --check`
passes. The rebuilt native app opens with `0 / 5` laps and `0 / 40` checkpoints.

```sh
cargo test --workspace
cargo test --release -p neural_racing_gui --test screenshots sustained_live_render_at_1000x -- --ignored --nocapture
cargo run --release -p neural_racing_engine --example live_benchmark -- 30 49597221076428566 monza
```

Live learning writes no policy/session files. The expanded observation requires
fresh weights; older weights fail the compatibility check. Test screenshots are
verification artifacts, not a save feature.

[Steering](screenshots/front-wheel-steering.png) ·
[Monza](screenshots/learning-monza-1000x.png) ·
[Silverstone](screenshots/learning-silverstone-1000x.png) ·
[Red Bull Ring](screenshots/learning-red-bull-ring-1000x.png)

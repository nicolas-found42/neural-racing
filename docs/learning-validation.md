# Five-lap learning and vehicle validation

## Corner curriculum and braking credit — 4 November 2026

The live learner now uses three coordinated changes:

- **Bounded control exploration:** the entropy bonus integrates the tanh Jacobian
  using five-point normal quadrature, with gradients for both mean and scale.
  A saturated pedal loses entropy even if its latent Gaussian sigma is large.
  PPO still scores the original pre-tanh draw: the action transform cancels in
  the likelihood ratio. One draw is held for four 20 Hz environment decisions
  (200 ms), rather than independently resampling every 50 ms. Manual input and
  native 60 Hz physics retain their cadence. Live training, generic rollouts,
  deterministic audits and Watch mode use the same neural control cadence.
- **Success-driven curriculum:** real track geometry identifies corner sections.
  Five difficulty levels extend the approach and increase initial speed. Each
  level requires at least 16 successful exits in a window of 20 attempts. Starts
  jitter by up to 10 m and entry speed varies. One in four section starts reviews
  a mastered corner; poor review results return it to practice. Repeated failure
  also reduces approach difficulty. All corners must qualify before full-lap
  practice; 16/20 completed full laps unlock five-lap stints. A section must exit
  on-road, moving, within its speed envelope, with controlled heading and offset.
  Section completions receive their own bonus and counters; they cannot earn
  checkpoint, lap or five-lap credit. No expert actions enter training.
- **Earlier braking feedback:** progress reward stops increasing above the safe
  profile, the existing overspeed cost follows squared speed (stopping energy),
  and the underspeed target leaves a 15% margin below the profile. GAE lambda
  rises from .95 to .99 at unchanged gamma .999. The direct trace weight three
  seconds back rises from .0434 to approximately .515; this is a credit trace,
  not a hard horizon on the critic. Held-control rewards are discounted at each
  environment decision; bootstrap and trace discounts use actual duration,
  including a partial hold at a terminal or update boundary.

Policy format **four** binds weights to the 200 ms neural cadence. Version-two
and version-three files are rejected with a retraining message; input count and
network shape remain 100 / 359,173 parameters. Vehicle parameters and integration
are unchanged. Fresh training is required for the new control contract.

Normal-start audits now allow up to 900 simulated seconds, also bounded by the
real one-lap environment budget, so a full beginner-paced lap can be assessed.
They use the same 200 ms cadence. A straight-line progress snapshot cannot restore
and erase a corner-practice policy; automatic restore waits for full-lap practice.
The assessment separately evaluates fixed corner starts outside training jitter,
at three difficulties, and records brake usage, crash speed and full-lap results.
The UI separates actual control entropy from latent Gaussian sigma, and displays
section success and curriculum stage alongside genuine lap counts.

The original entropy-only saturation regression failed with pedal .9950548
unchanged; the corrected update reduces saturation. The three-second crash-credit
regression failed at -1.301568 for a -30 terminal cost, then passed after changing
lambda. Numerical derivatives check the new entropy gradients. Tests also check
partial held controls, terminal/truncation bootstrap, curriculum promotion and
regression, no false lap credit, and early braking feedback before a crash.
An independent test-only driver completes every generated validation lesson at
all five levels on all three circuits. This proves lesson feasibility separately
from learned driving.

A fresh seed-42 final recipe trained on Monza for **180 seconds**, then its saved
checkpoint was reloaded and evaluated without exploration:

| Measurement | Result |
| --- | ---: |
| Actual full training laps / five-lap stints | 289 / 0 |
| Section attempts / safe exits | 11,330 / 6,443 |
| Held-out short / medium / long corner successes | 13/13 · 12/13 · 10/13 |
| Deterministic Monza normal-start progress | 44.66% of a lap, then crash |
| Deterministic Silverstone | 2 laps, crash at 2.85 laps |
| Deterministic Red Bull Ring | 7.99%, then crash |
| Applied braking fraction during training | 12.75% |

The unchanged baseline, seed 42 after a separate 120-second run, completed zero
training laps and crashed at 10.37% on Monza, 6.33% on Silverstone and 6.97% on Red
Bull Ring. An intermediate recipe before review/regression was added completed
five deterministic laps on all three circuits after 120 seconds, but stalled from
one alternate Monza start. The final recipe's mixed results show meaningful corner
and sampled-lap learning, **not consistent deterministic lap or stint competence**.
Wall budgets and concurrent load differ; this is not a controlled attribution of
improvement to any one change, a throughput benchmark, or a multi-seed result.

Final assessment run `1793835878018-58347-1`, source fingerprint
`7433979e5993bd2b`, has clean shutdown and zero drops, I/O errors or sequence gaps.
Local baseline/intermediate/final executables, saved weights, results, summaries,
source identity and retained diagnostics are under `.audit/training-improvements/`
(gitignored). These are explicit assessment saves; the live window still has no
persistence control. The new Learning window starts fresh. The previous session
was not stopped by this work; its worker-stop event predates the baseline build,
and it was absent from the running-app inventory at final verification.

`cargo test --workspace` passes **125 tests**, zero failures, two intentionally
ignored performance probes. The subsequently added forgotten-corner review test
also passes, as do the strengthened version-two/three rejection checks and three
Python retrieval tests. Generated screenshots were saved locally and all 23
existing files restored byte-for-byte. Changed standalone Rust formatting,
whitespace checks and release GUI build pass. A live native window was observed
training on Monza at requested 2000×, displaying curriculum stage and genuine
section/lap counters. Achieved speed is machine/load dependent.

```sh
cargo test --workspace
cargo test -p neural_racing_engine --lib reviews_detect_forgotten_corners
cargo run --release -p neural_racing_engine --example learning_assessment -- \
  180 42 monza .audit/new-learning-assessment/policies
```


## Recovered GT4 M3 integration — 4 November 2026

Manual driving and training now share the recovered stock GT4 M3 normal-road
model; [the port record](gt4-physics-port.md) documents source identity, original
instruction comparisons, configuration and limitations. The earlier independent
model and its measurements below are historical. The native 60 Hz physics runs
three updates per 20 Hz decision. Policy format version three rejects old weights
even when their observation dimensions match. Fresh training is required.

At the current `Env::step` seam, 0–100 km/h takes 3.60 seconds, full steering at an
initial 30 m/s produces about 1.45g in the controlled turning scenario, and a 0.2
steering command preserves straight-line acceleration from 10 m/s. The original
one-second keyboard launch peaks at 7.69 degrees rear slip in either direction.
The keyboard test driver completes all three circuits with binary controls at
200 ms intervals. The continuous test driver completes five laps on all three
circuits. GUI key-event and manual-speed isolation checks pass. These establish
control feasibility; no human playtest or newly learned lap competence is claimed.

Sustained full lock under coasting or hard braking can cause a recoverable slide
in the original model. Across 5–80 m/s, the controlled probes measured maximum
rear slip of 13.0 degrees while coasting, 21.7 degrees under full braking, and
9.4 degrees under throttle. Every steering-release probe returned below
0.0003 rad/s yaw. The old independent model's universal 12-degree/friction-circle
assertions are not native GT4 invariants: acceptance now distinguishes launch
stability, recoverable braking slides, and release control. No force clamp was
added to make those checks pass. Steering/clutch/tire physics has separate
original-instruction fixtures.

Final workspace verification passes **115 Rust tests**, zero failures, with two
performance probes intentionally ignored. The 21 original-code motion/contact
cases compare 232 outputs each at at most 8 ULP, with exact flags and integer
mileage. The GUI key-event regression and rendered UI checks pass. Existing
screenshot files were restored byte-for-byte; new diagnostic captures remain in
the local audit area. No throughput benchmark or fresh learning assessment was
run for this physics change.

## Earlier responsive handling iteration — superseded 4 November 2026

The first manual repair below prevented launch spinouts but restricted steering
too much. Its road-speed replay produced only **0.656g** at 30 m/s. After the
[GT4 REA investigation](gt4-physics-investigation.md), the shared vehicle now
combines independently requested tire forces, includes tire compliance in the
steering range, delivers power through a representative torque curve and six
automatic ratios, and uses actual/predicted rear slip for stability assistance.
The blanket cornering power reduction is removed. These are independently
implemented equations and tuning; GT4's complete tire model was not recovered.

Measured at the real `Env::step` seam: full steering in either direction sustains
**1.087g** at an initial 30 m/s; a modest 0.2 steering command retains **97.8%**
of straight-line acceleration (7.13 versus 7.29 m/s gained in one second from
10 m/s). Straight 0–100 km/h remains **3.55 s**. These are game acceptance
measurements, not GT4/BMW calibration claims. The original keyboard launch now
peaks at **7.70° rear slip**. The held-turn/release and correction checks still
pass across 5–80 m/s, including coasting and braking; the circuit drivers still
complete their one- and five-lap checks.

This iteration's full workspace suite passes **84 tests**, with zero failures
and two ignored performance probes. The release GUI rebuild passes; pre-existing
screenshot files were restored after testing. The rebuilt game was launched in
manual mode for the next human evaluation.

```sh
cargo test -p neural_racing_core --test handling_response -- --nocapture
cargo test -p neural_racing_core -- --nocapture
```

No observation fields or policy file format changed. Handling changed, so old
weight quality and historical learning/throughput measurements remain unverified
on this revision. No claim of learned lap completion or human playtesting is
made by the automated driving checks.

## Earlier manual handling repair — superseded 4 November 2026

The reported manual spinout was reproduced through both `Env::step` and real
egui key events passed through `App::manual_action` / `advance_race`. Starting at
5 m/s, holding throttle and either turn key made rear-axle slip reach **13.63°
after 0.55 s**. Rear-axle slip distinguishes a skid from the normal CG sideslip
of a tight, low-speed rolling turn. The retained manual races in run
`1793821302706-94010-1` ended off-track at steps 67 and 88 with CG slip of
16.17° and −17.47°; inputs are not retained, so those records alone cannot
reconstruct the user's exact maneuver.

Two coupled causes were isolated: full pedal force was allocated before lateral
force, leaving the driven rear tires no cornering capacity, while keyboard input
requested almost full road-wheel lock even at road speed. Reserving longitudinal
grip alone delayed the regression but did not prevent it. At that revision, the shared vehicle
used speed-sensitive steering with a nominal 60% lateral-grip demand and combined
slip control that reduces drive/braking to retain lateral capacity. It reserves
grip for requested and actual steering, including rack return, and responds to
each tire's slip demand. No yaw clamp, trajectory correction or manual-only
physics is applied. Straight-line engine/brake force limits are retained.

Regression coverage includes the original one-second held-key launch in both
directions, arrow keys and A/D through the GUI input path, real-time manual
stepping with a previously selected 1000× learning speed, and three-second held
turns followed by two-second release across 5/10/20/40/60/80 m/s under throttle,
coasting and braking. The latter checks the real integrator without track walls
prematurely terminating the maneuver. Rear slip stays below 12°, yaw recovers
below 0.05 rad/s, and all four friction limits remain respected. Repeated
left/right corrections every 250 ms also remain stable. The original one-second
launch now peaks at **7.32° rear slip** in either direction.

A separate test-only driver completes one lap on each real circuit using only
−1/0/+1 steering and pedal commands updated every 200 ms, at a 15 m/s cruise
target with corner braking. Steering uses accumulated taps to approximate the
requested turn. The existing continuous test driver also completes five laps
on each circuit. These are control-feasibility checks, not human playtest results
or evidence of learned-policy competence. Both launch regressions were verified
to fail on the original vehicle source and pass with the repair. Local logs and
preserved screenshot artifacts are in `.audit/manual-handling/` (gitignored).
The workspace run exposed a worker-test assumption that any attempted PPO update
changes weights; a rejected update can leave them unchanged. Its wait now requires
the actual parameter change, and the worker control test passes.

Final validation: `cargo test --workspace` passes **81 tests**, zero failures,
with the two 90-second performance probes intentionally ignored. Changed Rust
modules pass rustfmt and `git diff --check`; the release GUI rebuild passes.
Screenshot-test output was saved locally and the existing `docs/screenshots/`
contents restored byte-for-byte. No new throughput benchmark or learned-policy
assessment was run.

```sh
cargo test -p neural_racing_core --test vehicle_dynamics keyboard_turn_from_start_does_not_spin_out -- --exact --nocapture
cargo test -p neural_racing_core held_keys_and_release -- --nocapture
cargo test -p neural_racing_core --test real_circuits
cargo test -p neural_racing_gui --lib held_keyboard_turn -- --nocapture
```

Observation layout and policy file format remain unchanged; old weights can be
structurally compatible but their driving quality under the changed steering
mapping is unverified. Start a fresh learning session. All learning/throughput
measurements below predate this handling repair and have not been re-established
for the new dynamics.

## Earlier learning measurements

The historical measurements starting at “Steering regression” predate the
increase from 11 to 44 sensor rays. The current policy has 100 inputs and 359,173
parameters. Its separate five-minute assessment is recorded below.

Measured on 4 November 2026 on the user's Apple M5 Pro Mac: 18 CPU cores,
20 GPU cores, 48 GB memory. These results replace the earlier single-lap,
smaller-policy measurements. Throughput depends on current system load and
training phase; the app displays achieved speed alongside the requested speed.

## Five-minute assessment with 44 rays

On 4 November 2026, a fresh seed-42 policy trained headlessly on Monza for
**300.023 seconds**, using the same `LiveTrainer` as the UI, four practice cars,
five-lap attempts, 44 rays and unchanged physics/rewards/PPO. The release build
ran uncapped, with no GUI or other benchmark running. Baseline and final
evaluations were outside the training timer; four checkpoint audits and policy
saves inside it cost 0.044 seconds altogether.

| Result | Observed |
| --- | ---: |
| Achieved simulated speed per car | 241.26× |
| Simulated hours per car | 20.11 |
| Decisions across all four cars | 5,790,668 |
| Model updates | 4,429 |
| Completed attempts / crashes | 23,735 / 23,735 |
| Completed laps / five-lap goals | 0 / 0 |
| Rejected updates / restored snapshots | 56 / 0 |

Noise-free evaluation from the normal starting line, measured as a fraction of
**one lap**, shows a rapid initial improvement followed by a plateau:

| Training wall time | Monza lap progress | Outcome |
| --- | ---: | --- |
| Before training | 0.53% | Crash |
| 60 seconds | 10.75% | Crash |
| 120 seconds | 10.71% | Crash |
| 180 seconds | 10.72% | Crash |
| 240 seconds | 10.75% | Crash |
| 300 seconds | 10.78% | Crash |

The final checkpoint also crashes on Silverstone at 6.69% and the Red Bull Ring
at 7.56%; those circuits were evaluated without training on them. From the three
other Monza practice starts, it reaches 6.68%, 12.87% and 8.45% of a lap before
crashing. No deterministic evaluation completes a lap. At Monza's normal start,
the minute audits crash at 67–69 m/s. These observations establish a failure to
learn lap completion in this one run; they do not isolate its cause or establish
that more rays help or hurt relative to an otherwise identical 11-ray run.

Run `1793812891390-50292-1` has source fingerprint `b31cb544907a5c06`, revision
`ce51d49995081ae1323ebcda8a5bd54cfab6622e` plus local changes, and Accelerate math
support. Retained records have a clean shutdown, zero dropped events, zero I/O
errors and no sequence gaps. Attempt/update detail remains sampled; cumulative
outcomes are exact. Local evidence is in
`.audit/learning-5min-20261104T172131Z/`: raw results, metadata and binary hash,
working-tree patch, diagnostic records, a JSON summary and six policy snapshots.
The saved final policy was reloaded and checked against the in-memory parameters
before evaluation. This directory is gitignored and is not available in every
clone; these are explicitly saved assessment weights, not live-session persistence.

Reproduce with a new output directory:

```sh
NEURAL_RACING_DIAGNOSTICS_DIR="$PWD/.audit/assessment/diagnostics" \
  cargo run --release -p neural_racing_engine --example learning_assessment -- \
  300 42 monza .audit/assessment/policies
```

The observed throughput is well below 10,000×. Separate profiling records throughput and learning quality with their own acceptance criteria.

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

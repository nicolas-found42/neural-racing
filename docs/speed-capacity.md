# Measured simulation capacity

4 November 2026 · Apple M5 Pro · 18 CPU cores · 48 GiB memory.
No thermal/performance warning was reported during the initial hardware check.
These are sustained measurements of this build under the observed system load,
not a theoretical upper bound on what the hardware could do with a different engine.

## Uncapped results

The same four-car, five-lap training session, 67-input / 350,725-parameter policy,
120 Hz vehicle physics, three-epoch PPO updates and the deterministic policy audit
run in both probes. Speed means simulated seconds **per car** per wall second,
including training overhead. The four cars are not added together to inflate the
multiplier.

| Circuit | No-render loop, 15 s | Live Max, 30 s | Live state refreshes/s | Render p95 |
| --- | ---: | ---: | ---: | ---: |
| Monza | 289.3× | 224.4× | 39.4 | 8.89 ms |
| Silverstone | 218.8× | 344.4× | 41.7 | 8.99 ms |
| Red Bull Ring | 176.2× | 205.1× | 38.2 | 9.04 ms |

Live rendering uses the actual UI at 1264 × 900, with latest actual positions of
all four training cars. Native Retina rendering, competing applications, model
update size and the learned policy can change the result. The 15-second and
30-second runs cover different amounts of learning and different update counts,
so their difference is not an exact isolated estimate of rendering overhead.

These figures replace the earlier 1,200–1,300× measurements on the previous
update stack. The current trainer spends three epochs over every batch,
re-forwards each minibatch to enforce the trust region and runs a 1200-decision
audit every 200 updates. That work costs roughly 4–6× per
simulated second; physics, rendering and the number of cars are unchanged. The
useful measured answer is **about 180–345× per car**, which is still far above
real time. The 2000× numeric slider limit is headroom, not a claimed ceiling.

## What changed

- Numeric slider increased from 0.25–1000× to **0.25–2000×**. The upper end provides
  headroom above the measured 180–345×; it does not claim 2000× is achieved.
- **Max** runs uncapped: the worker bypasses wall-time pacing and advances as fast
  as available computation allows. It still checks controls between short work
  slices and publishes current states approximately every 16 ms when work allows.
- Removed the unconditional 1 ms sleep after each 3 ms work slice when work is
  queued. The earlier scheduler left useful CPU time idle under heavy demand.
- Numeric speed selection or 1× exits Max mode. Clicking the selected Max button
  returns to numeric pacing. CLI: `--speed max`.
- Actual speed remains measured from completed simulation decisions. Physics dt
  and the number of cars are unchanged by these changes.

The main limit is the serial live coordinator: inference, four environment steps,
trajectory bookkeeping and model updates run in sequence. Dense matrix operations
use Accelerate, but the 18 CPU cores do not each independently advance a car.
The rendering benchmark was observed at about 129% aggregate CPU (roughly 1.3
cores in macOS accounting), not 18 saturated cores. Higher whole-machine throughput
would require a different parallel rollout/inference design and another benchmark;
raising a number cannot establish that capacity.

## Verification

The uncapped rendering probe passes on all three circuits, with continuing model
updates and live states. Its minimum refresh and responsiveness checks remain
active. Unit tests cover numeric speeds above 1000, switching between Max and
numeric pacing, invalid inputs, pause/resume, track changes and reset. The full
workspace regression suite passes 55 tests; the two sustained performance probes
are separate ignored tests.

```sh
cargo run --release -p neural_racing_engine --example live_benchmark -- 15 49597221076428566 monza
cargo test --release -p neural_racing_gui --test screenshots sustained_live_render_at_max_speed -- --ignored --nocapture
cargo run --release -p neural_racing_gui -- --speed max
```

Run timed benchmarks separately from other benchmarks for comparable results.
No model or session is saved by these probes.

[Monza Max](screenshots/learning-monza-max.png) ·
[Silverstone Max](screenshots/learning-silverstone-max.png) ·
[Red Bull Ring Max](screenshots/learning-red-bull-ring-max.png)

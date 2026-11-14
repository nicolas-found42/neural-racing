# Observability verification — 4 November 2026

All scenarios here are local. Test evidence is in `.audit/observability/evidence`
(gitignored) and is queryable with the same production retrieval script. This
folder is not a remote service or a guarantee of future retention. Each run embeds
its own source fingerprint; successive audit builds are deliberately distinguishable.
The application is not automatically upgraded by editing the repository: rebuild
and launch it to collect future sessions.

## Previously visible

The initial source inventory covered 39 files under the original four crates.
`jg files . --list` also inventoried docs/assets; the initial semantic search hit
its 12-request limit. Its returned source excerpts were read, and the missing areas
were inspected directly rather than treating incomplete search results as coverage.
The final [coverage inventory](observability.md) includes every application component
and explicitly identifies the external boundaries that were not exercised.

The only existing structured historical artifact found was
`.audit/perf-shared-comparison.jsonl`. A real parse returned **10 rows, zero reported
failures, no timestamps, and no commit/revision field**. This supports only the
recorded baseline/shared benchmark comparison. It cannot establish what happened
yesterday. The default diagnostics directory did not exist before this work.
The shipped policy and documentation are historical artifacts, not state history.

## Reconstructed incidents

### Numerical failure and recovery (synthetic)

```sh
python3 scripts/diagnostics.py --dir .audit/observability/evidence \
  --run 1793808044780-17673-1 --operation 2 --limit 100
```

This release probe's retained sequence shows session start with seed 42 and Monza,
three successful updates, a switch to Silverstone (epoch 1), then `live_error` with
`reason="non-finite inference"` at **3,592 decisions**. `probe_recovered` follows at
**3,596 decisions**. The fixture injected a NaN parameter, asserted the rejected
step did not advance the decision counter, restored the valid policy and asserted
that the next step advanced. All four cars had just been reset by the track switch;
the empty recent-decision history in the error record is therefore expected.
There were **zero drops, I/O errors or sequence gaps**, and a process end was retained.

Supported: the failure reached inference after the track switch; the new guard
prevented that invalid inference from entering physics; valid weights allowed work
to continue. The synthetic cause is known from the fixture. For an unexplained
production NaN, this evidence alone cannot identify which computation corrupted
parameters or recover unsaved weights. See
[diagnostics_probe](../crates/neural_racing_engine/examples/diagnostics_probe.rs).

### Policy collapse, restore and continued stepping (synthetic)

```sh
python3 scripts/diagnostics.py --dir .audit/observability/evidence \
  --run 1793808222310-18959-1 --operation 2
```

Retained `policy_audit` reports progress **0.0111260705**, `stopped=true`, previous
best **0.2**. The next event is `policy_restored`, epoch **1**, restores **1**.
The following `live_progress` shows **four decisions**, all cars at step **1**,
and no terminal outcome. Zero losses/gaps; normal shutdown retained.
The existing behavioral test was extended to step after restoration; it also
asserts exact parameter equality with the saved snapshot. This validates recovery,
not the claim that the restored policy has learned a real circuit. The fixture's
remembered best is synthetic.

### Worker control sequence (real background thread)

```sh
python3 scripts/diagnostics.py --dir .audit/observability/evidence \
  --run 1793807855508-15629-1 --limit 100
```

Five command requests each have receipt and application records. The test paused
after a real update, changed circuits without changing decisions or weights,
reset to a new session with zero decisions, resumed, then stopped/joined the worker.
The retained reset links old and new session IDs. The test retrieved the files
only **after shutdown**, invoked the Python run summary in another process and
checked normal end/loss counters. No drops or sequence gaps.

### Stall and timeout (synthetic terminal state)

Run **1793808222650-18969-1** records the first four stalled attempts. Its final
`live_progress` has `end_counts=[0,0,4,4]`: four stalls and four timeouts. The test
forced the progress/step-budget boundaries, asserted both reasons, then stepped
again and checked all four cars restarted at step 1. Timeout detail is unsampled
in this fixture; the cumulative totals and behavioral assertions establish it.
No claim is made that these forced conditions explain a real historical stall.

### Panic, abrupt exit, file failures and saturation

- Run **1793808045036-17701-1**: a synthetic panic retains the source location and
  `payload="redacted"`; the canary payload is absent from all retained files.
  A process end records `unwinding=true`, which the query distinguishes from a
  clean shutdown. Malformed end records cannot falsely mark a run as clean.
  Normal stderr still uses Rust's pre-existing panic hook.
- Run **1793808045126-17703-1**: the probe flushed `probe_waiting`, then the test
  controller sent SIGKILL. Retrieval finds the waiting event and **no process end**.
  Without the controller's record this means interrupted/unfinished evidence,
  not proof of a kill or a specific cause.
- Missing/invalid policy CLI tests retain `policy_load_finished`, safe error kind,
  panic location and process end across separate processes. The shipped legacy file
  also fails loading in a real CLI run. Later instrumentation adds an allowlisted
  format-failure category to distinguish unsupported versions, bad magic, truncated
  data, shape mismatches and trailing bytes without retaining raw I/O messages.
  Final run **1793808911101-22035-1** explicitly records `unsupported_version` for
  the shipped model, `InvalidData`, 2 ms duration and operation 2.
- A real one-generation headless run (one worker/episode/evaluation track) retained
  rollout, update, evaluation and three policy saves. The final implementation also
  records checkpoint roles (`evaluation`, `best`, `final`) and generations, with
  child load/save operation and content fingerprints. Final run
  **1793808911176-22036-1** was retrieved and checked for all three successful roles.
- An initial 100,000-event flood recorded **89,435 drops** but lost `process_end`.
  This exposed an actionable shutdown ordering gap. A second saturation run found
  that a nonblocking flush request could itself be rejected. Control messages now
  retry within a fixed deadline, and shutdown drains before enqueueing its end.
  Final run **1793808910864-22033-1** records **88,676 drops**, exactly **88,676
  sequence gaps**, and a **clean process end**. The new full-queue shutdown test
  prevents regression. Normal event producers still never wait for queue space.
- Blocked-directory tests check the application's exit result is unchanged, then
  replace the blocker and retrieve a subsequent event carrying nonzero I/O/drop
  counters. Rotation tests exceed the configured segment count, check bounded
  file sizes/count, private Unix permissions and active-file lock protection.
  Expiry uses a backdated file timestamp. Oversized events and full queues are
  counted; malformed/truncated query input yields an explicit incomplete result.

## Fresh review and repairs

A second pass challenged storage and shutdown rather than live training:

- Normal shutdown after a saturated queue now records its end **after** draining;
  flush/shutdown control messages retry within their deadline when the queue is full.
- `train --help` returns through the guard instead of calling `process::exit`.
- Policy operations include byte/content identity and safe format rejection reason;
  headless checkpoint operations include role and generation.
- Manual/Watch races have distinct operation IDs; geometry IDs include road width.
- Query summaries distinguish unwind from clean return and flag absent earlier
  segments, sequence gaps, drops and partial input.
- The first full debug suite failed a UI test's 10-second wait for two updates.
  An independent seed-42 debug probe showed update 1 at **5.024 s** and update 2
  at **10.128 s**; it was not a retained trace of the failed test itself. UI tests now wait for the required worker acknowledgment/state
  with a 30-second bound and print the last state on timeout; screenshot tests no
  longer infer readiness from a fixed sleep. Performance assertions remain in the
  separate release benchmark. This supports a throughput-sensitive test timeout,
  not a claim that all asynchronous timeouts are harmless.

## Regression and performance results

`cargo test --workspace`: **67 passed, 0 failed, 2 ignored performance probes**.
The full suite was repeated after the shutdown and correlation changes. Later
narrow CLI/error-category changes were checked with the relevant engine, ML and
recorder suites. `python3 -m unittest discover -s scripts -p test_diagnostics.py`:
**3 passed**. Changed Rust modules pass rustfmt; `git diff --check` passes.
Generated screenshots were retained in the audit folder, then the user's previous
screenshot files were restored.

The separate **90-second release Max-speed render probe**, with diagnostics enabled,
passed its actual-state refresh and rendering latency acceptance checks:

| Circuit | Achieved speed per car | Live state refresh/s | Render p95 |
| --- | ---: | ---: | ---: |
| Monza | 318.2× | 43.2 | 8.47 ms |
| Silverstone | 271.0× | 38.5 | 9.20 ms |
| Red Bull Ring | 286.2× | 39.5 | 8.18 ms |

Run **1793808225576-19052-1** retained **82,579 bytes**; largest line **1,810 bytes**.
It contains 23 sampled attempts, 21 sampled updates, 6 policy audits, 8 UI health
records, 9 worker health records and 9 complete command triplets. All three workers
stopped/joined. **Zero drops, I/O errors or sequence gaps**. One observed UI health
record reports snapshot age 11 ms, max frame interval 34 ms and worker still alive.
The probe completed no laps; this does not establish driving competence.

These measurements show healthy local performance with instrumentation enabled.
They are not a controlled before/after overhead estimate, a slow-disk test, or a
portable performance guarantee. Seven real days of retention, power loss,
network filesystems, non-macOS locking/permissions, native window initialization,
GPU/Accelerate crashes, deliberate deadlocks and remote deployment remain untested.

## Repeatable verification commands

```sh
cargo test --workspace
python3 -m unittest discover -s scripts -p test_diagnostics.py

# Isolated synthetic inference failure/recovery; no existing weights changed.
NEURAL_RACING_DIAGNOSTICS_DIR="$PWD/.audit/observability/evidence" \
  cargo run --release -p neural_racing_engine --example diagnostics_probe -- live

NEURAL_RACING_DIAGNOSTICS_DIR="$PWD/.audit/observability/evidence" \
  NEURAL_RACING_CAPTURE_TEST=restore cargo test --release -p neural_racing_engine \
  live::tests::a_collapsed_mean_policy_restores_the_best_snapshot -- --exact

NEURAL_RACING_DIAGNOSTICS_DIR="$PWD/.audit/observability/evidence" \
  NEURAL_RACING_CAPTURE_TEST=endings cargo test --release -p neural_racing_engine \
  live::tests::stalled_and_timed_out_cars_restart_without_ending_training -- --exact

# Runs 30 seconds per circuit and writes screenshot artifacts.
NEURAL_RACING_DIAGNOSTICS_DIR="$PWD/.audit/observability/evidence" \
  NEURAL_RACING_CAPTURE_TEST=render cargo test --release -p neural_racing_gui \
  --test screenshots sustained_live_render_at_max_speed -- --ignored --exact --nocapture

python3 scripts/diagnostics.py --dir .audit/observability/evidence --summary
```

## Next iteration targets

External crash reports/watchdog integration, deliberate slow-disk and native-hang
experiments, and longer retention validation need separate environments or
infrastructure. Exact replay requires a product decision about checkpoint/input
retention and storage cost. No credentials, deployment settings, or remote services
were changed, and no historical events that were never retained were recovered.

## Commit preparation and check ledger

The November 4 changes are committed in dependency order, without backdating or
pretending the intermediate snapshots were the original capture-time builds:

1. `docs(circuits)`: existing circuit reference provenance and visual evidence.
   All 48 manifest asset hashes were verified. Nine temporary signed download
   queries were removed before publication; stable source URLs, local files and
   hashes remain. Three GeoJSON files had trailing whitespace normalized; parsed
   JSON was checked against the originals and is identical.
2. `feat(racing)!`: the previously uncommitted, coupled physics, 67-input model,
   real circuits and live-learning UI. This split was reconstructed in an isolated
   directory while preserving the final working source. Its entire workspace and
   all targets compile independently of diagnostics. Runtime tests were **not
   rerun on this intermediate snapshot**; its inherited timing assumptions are
   fixed in the later instrumentation commit. Earlier capacity/learning reports
   remain dated historical records; their 55/59-test counts are not current totals.
3. `feat(diagnostics)`: shared bounded store and retrieval tool. The isolated
   staged snapshot passes all-target workspace compilation, five recorder tests
   and three Python retrieval tests. Application entry points are intentionally
   connected in the following commit.
4. `feat(observability)`: workflow correlation, safe failure context, recovery
   evidence, failure probes and state-based UI test waits. The final workspace
   checks below validate the combined implementation.
5. `docs(observability)`: coverage, priorities, runbook, measured evidence,
   chronology and remaining limits. Local `.audit` logs and raw event files are
   gitignored; they are **not shipped with a clone**. Reproduction commands above
   create new evidence and do not reproduce the original event timestamps.

Failed checks were repaired, not omitted from the record:

| Stage | Failure | Resolution / later evidence |
| --- | --- | --- |
| Implementation | GUI command-channel element type could not be inferred (`E0282`) after adding the command envelope | Explicit tuple channel type; workspace compiles and worker integration tests pass |
| Recorder test | Recovery test expected exactly one I/O error, although failed write and failed flush can each count | Assert nonzero I/O errors and the exact dropped-event count; recovery test passes |
| First full debug suite | UI state wait expired after 10 seconds | State-based bounded waits with last-state failure details; final suite passes. The independent timing probe does not prove the exact cause of that earlier timeout |
| Saturation review, first pass | Queue flood lost the shutdown marker | Drain before recording end; retest exposed a further flush-enqueue problem |
| Saturation review, second pass | Full queue also rejected the flush control message | Bounded retries for control messages, dedicated full-queue shutdown test, and retained end marker in final 100,000-event probe |
| Publication whitespace check | Three imported GeoJSON lines had trailing whitespace | Normalize trailing whitespace; verify parsed data unchanged; whole-series diff check passes |
| Temporary prerequisite split | Two remaining diagnostic references caused unresolved-crate errors | Remove those calls only from the intermediate snapshot; independently compile all targets again. Final working implementation was never reverted |
| Temporary formatting invocation | Included coordinate-expression files were passed to rustfmt as standalone modules | Exclude `circuit_data/*.rs` from standalone formatting; Cargo compiles them through their actual `include!` callers. All standalone Rust files pass formatting |

Expected negative-test exits (missing/invalid policy, deliberately injected panic,
SIGKILL and non-finite input rejection) are successful scenario checks when their
assertions and retained evidence match, not unexplained failed regressions.

Unexecuted or unfinished checks/work are explicitly separate from passed checks:

- **Unexecuted on intermediate feature commits:** runtime regression suites; only
  compilation was repeated for the prerequisite split. The combined final source
  receives the full workspace suite.
- **Not repeated during commit preparation:** the 90-second Max render probe. Its
  actual earlier run is recorded above. The separate ignored 1000× probe was not
  rerun in this observability pass; earlier documents describe earlier runs.
- **Unexecuted:** seven actual days of retention; physical power loss; deliberately
  slow or network-backed disks; non-macOS locking/permissions; deliberate native
  window/GPU/Accelerate crashes or deadlocks; controlled before/after overhead
  measurement; deployed or remote-service integration tests.
- **Unfinished product work:** learned-policy completion of the real circuits,
  live model/optimizer checkpointing and exact input replay. Deterministic test
  drivers completing laps do not establish learned-policy competence.
- **Unfinished observability integration:** OS crash-report collection, independent
  watchdog/remote archival and durable-history guarantees. Local bounded telemetry
  is implemented and tested; none of these external capabilities is implied.

The empty untracked file `8` is left untouched and excluded from publication.

Commit-preparation rerun: `cargo test --workspace` passed **67 tests**, with
**0 failures** and **2 ignored performance probes**. The three Python retrieval
tests, standalone Rust formatting check, and whole-series whitespace check also
passed. Generated screenshots were copied into the local audit area and their
pre-existing committed contents restored. No new performance claim is inferred
from these regression checks.

## Native Metal throughput experiment — later 4 November 2026

The optional Metal inference component's full release workspace run passed
80 tests, zero failures and two ignored performance probes in
`.audit/speed-10000/native-metal-workspace-rxh7e_6h/test.log`; screenshot hashes
were restored. The default-feature ML run passed 18 tests and the default
release workspace/all-targets check passed. These are later checks of the dirty
working tree, separate from the earlier committed observability verification.

Fresh process `1793820453294-90066-1`, operation 2, retained
`metal_inference_created` for the matrix-fused kernel: 359,173 logical parameters,
360,965 padded storage values, f32 normalization, fast math disabled, version 0.
The run ended cleanly with zero losses/I/O errors/sequence gaps. Native Objective-C
and Metal files are now included in the source fingerprint. A separate probe
with diagnostics directed at `/dev/null` returned the same three parity results,
exit 0 and the expected storage warning. Retrieved evidence is in
`.audit/speed-10000/native-metal-control-20261104T192721Z/`.

The tests include actual shader-compilation failure and rejection of invalid
shapes/non-finite input; none substitutes a CPU result for failed GPU execution.
Full-PPO native parity, GPU physics, live integration and the actual 10,000× gate
remain unachieved. Forward-only and minimal-dispatch measurements are recorded
with their scope in [capacity measurements](speed-capacity.md).

## 4 November 2026 — recovered GT4 M3 integration

Manual driving and learning now use the same recovered M3/flat-asphalt model.
`cargo test --workspace` passed **115 tests**, zero failures, with two performance
probes intentionally ignored. The release GUI rebuild, changed-module formatting
and `git diff --check` passed. Existing23 screenshot files were restored
byte-for-byte after UI tests; diagnostic renders remain local. A real manual
window on Monza was opened and exercised with a throttle key; this is an automated
UI check, not a human handling assessment.

Original-code proof is in [the validation receipt](reference/gt4/validation-receipt.json):
2319 main groups,21 post-frame cases checking232 outputs each (max8ULP, exact
flags/mileage), and24 whole contact-tail cases. Constructor/source/profile
identities and emulator limitations are retained with that proof. Earlier
intermediate comparisons exposed arithmetic-order errors and a missing free-wheel
update; those were fixed at their original operations. Invalid harness ABI/proxy
reports remain explicitly marked local diagnostics and are not accepted evidence.

The diagnostic source fingerprint now includes embedded binary parameter assets.
An isolated copy of the build fingerprint program changed only one parameter
byte and produced distinct fingerprints; local proof is
`.audit/gt4-port/fingerprint-parameter-check.json`. Policy version three rejects
previous-vehicle weights, and the load rejection plus new-format round trip pass.
The final manual bundle launch and executable SHA256 are retained in
`.audit/gt4-port/final-launch.json`; workspace output, native comparisons and
render captures are in that same gitignored audit directory. No new throughput
benchmark or learned-policy competence claim is made.

## 4 November 2026 — exploration and corner curriculum

The final 180-second seed-42 Monza assessment uses retained run
`1793835878018-58347-1`, fingerprint `7433979e5993bd2b`, release Accelerate backend.
Retrieval after shutdown reports zero drops, I/O errors or sequence gaps, with a
clean process end. It contains 215 curriculum changes, 304 sampled terminal
records, 19 progress records and successful checkpoint save/reload operations.
Totals and learning limits are in [learning validation](learning-validation.md).
Section completions are separate from real laps; update rows now cover held
controls, with explicit duration discounting. Latent sigma and bounded-control
entropy have distinct fields. Earlier records keep their original four-outcome
counter layout; new records append section completion as outcome five.

The full workspace suite passes 125 tests, two performance probes ignored. The
new forgotten-corner review regression passed separately after its addition.
The ML suite, all-circuit lesson-feasibility test, three Python retrieval tests,
changed-module formatting, whitespace and release GUI build pass. Screenshot
outputs were retained in the local audit directory and 23 existing images restored
byte-for-byte. Two newly introduced regression tests were first observed failing:
saturated throttle received no entropy mean gradient, and a three-second crash
trace was too weak. Both pass with the corrections. A held-control test initially
used exact equality across batched/scalar f32 inference, then jumped the clock and
accidentally triggered the stall path; numeric tolerance and progress-clock setup
were corrected. An initial format check found the newly added review test unformatted;
formatting and the final check pass. None of these fixture issues is omitted from
verification history.

Local proof is `.audit/training-improvements/`: baseline/intermediate/final
assessment JSON, policies, final diagnostic export/summary, test/build logs,
render captures, snapshot hashes and a bundle-launch receipt. The updated native
Learning window was observed running real curriculum training. Existing unsaved
weights were not recovered: the earlier worker had already stopped before the
baseline build, and its app was absent from the final running inventory. No new
throughput or universally reliable learned-policy claim is made.

## GT4 course integration checkpoints — 4 November 2026

The first frozen 95-resource course probe used input receipt
`330f87e69f26971eb5aa4b02b117ae1d4115f7ff89311ecef34f0a9ddc3a7854`.
Its before/after SHA-256 verification passed. All 95 source-distance/finite-launch
checks passed; the continuous independent driver completed 85 race routes and
six manual area launches. Four Sarthe variants failed near native progress
13,503 m. The derived guide left only one metre of right clearance on a measured
23.25 m connected road component. These failures are retained in
`.audit/gt4-tracks/final-all-course-driving.json` and the checkpoint copies; a
later repair must carry a new input receipt rather than rewriting their meaning.

The representative 200 ms binary-keyboard probe passed High Speed Ring,
wet Tsukuba and Suzuka at its 15 m/s target. Yosemite Dirt Short failed at both
15 m/s and a separately recorded 12 m/s target. Smooth-control feasibility is
not keyboard-controller competence or a human playtest. Lowering the target
is recorded as a separate experiment, not a replacement for the failed result.

A workspace checkpoint passed 152 tests, zero failures, with four long/performance
probes ignored by default. The first attempt had failed three screenshot tests:
a new dashboard label used a missing arrow glyph, causing a font panic. Replacing
the wording fixed all seven ordinary screenshot tests; the following workspace
run passed. All 23 existing screenshot files were restored byte for byte; new
captures remain in `.audit/gt4-tracks/rendered-final/`. Build fingerprint checks
also establish that changing only a copied `.mesh`, then only a `.stations`
input produces distinct source fingerprints. Neither check applies retroactively
to a previously running binary.

The isolated REA session exported 37 evidence records with zero reported unknowns
and closed successfully. Its local bundle SHA-256 is
`a169dc26302fe88017bfe1c1278a2d74ced8e6de3baa6f9be979579bc2767b01`;
the saved analysis snapshot SHA-256 is
`f785ee8d58f642b7ff73d2a3a60765d5d71a24aafc0453af8b83771c1524e9cc`.
The owned MCP client exited and no owned headless Ghidra process remained.
This is an evidence-retention and cleanup result, not a whole-game parity claim.

The unchanged 12 m/s dirt keyboard controller's last 40 decisions explain that
probe's departure: all 39 preceding body-centre queries hit material 5, with
height changes below 5.7 cm per decision. It commanded full lock on 24 decisions,
full throttle on 17, and no braking. Heading error reached 65.65 degrees, CG slip
27.93 degrees and yaw rate 1.558 rad/s before crossing the road boundary; the
first missing body-centre contact was at the departure. The retained trace is
`.audit/gt4-tracks/keyboard-traces/gt4-yosemite_dirt_short-12mps.json`.
This supports a controller-overshoot diagnosis for that run, not an abrupt
body-centre mesh hole; it does not establish a human dirt playtest or excuse the
failed controller result. Rear slip in this trace is explicitly a kinematic
estimate from public telemetry, not original native wheel-slip output.

## GT4 ground depth and tile rendering — 4 November 2026

The retained negative control
`.audit/gt4-road-boundaries/visual-prototype/depth-order-negative-control.json`
re-renders the same source packets, UVs, texture and colors in two orders.
15,352 of 808,920 pixels change; at Suzuka physics (-572.5, -327.5) m the
mean-height sort paints [100, 98, 94, 255] where per-pixel depth paints
[0, 0, 0, 128]. The paired DAG evidence contains blocked ordering cycles within
0.5–1 mm, so this is not a tolerance problem. Mean-height face order is removed
from the source record (`SourceVisual`) and the prepared visual; it is not a
fallback path.

Core now rasterises prepared source faces per pixel into world-aligned tiles
(`source_raster`), compositing back to front with interpolated height, UV and
vertex color, and the GUI caches tiles at cached power-of-two densities and asks
one background worker. `source_raster` tests cover translucent-over-opaque
compositing, tile seams, crossing faces, malformed texture records and
degenerate input. The bundled-asset test now also builds the rasterizer for all
95 visuals: every prepared face has a usable texture record and every source
face is either rasterisable or counted as zero-area. The production test
`imported_courses_draw_depth_composited_ground_tiles_over_the_course`
waits for the visible tile set with a deadline and asserts that `draw_track`
actually draws textured tile images over Suzuka follow and Nürburgring overview,
with no source-visual error text. `tile_cache_converges_across_zoom_viewport_and_course_switch`
changes tile level without waiting, alternates follow and overview four times
while requests are in flight, resizes the viewport and switches course in one
context; it asserts that requests drain to zero and resident tiles stay within
the documented bound. Removing the per-level request cleanup fails that test
(244 s timeout with stale keys), so it covers the stale-level path rather than
passing vacuously.
Retained follow captures also cover New York and the free-roam gymkhana area
(`.audit/track-visual-fix/*-follow-source-render.png`); New York's authored
translucent shadow decals keep their source triangular silhouettes and are not
evidence of mean-height ordering.

Measured on this revision in a release build with `App` plus wgpu readback at
1264×900 (`cargo test --release --test gt4_visual_audit -- --ignored`,
`.audit/track-visual-fix/app-follow-frame-cost.json`, 120 frames per view):
follow p95 7.9 ms (Suzuka) and 6.7 ms (Nürburgring); overview p95 7.5 ms and
6.1 ms; driving with tile-cache misses p95 7.2 ms and 9.3 ms, maximum 13.2 ms.
This is frame cost with synchronous readback on this machine, not a GPU-command
capture or a perceptual image-difference metric; the audit renders under
`.audit/track-visual-fix/` are visual inspection aids.

The 15 m/s 5 Hz binary keyboard probe fails three of four representative routes
on this revision (`.audit/track-visual-fix/keyboard-15mps-current.log`,
`.audit/gt4-tracks/keyboard-course-driving.json`). The failure rows are
byte-identical to the pre-existing `.../track-visual-fix/final-keyboard.log`
checkpoint (Tsukuba 1403.25 m/2103 steps, Suzuka 832.76 m/1169 steps, Yosemite
254.61 m/398 steps), and this change touches no physics, reward or guide code,
so these are baseline failures rather than visual regressions. The earlier
checkpoint above recorded wet Tsukuba and Suzuka passing at 15 m/s, so between
that receipt and now they became road regressions; this revision neither
introduces nor repairs them. All three are marginal crossings of the guide's
side clearance: Suzuka 1.122 m of 1.057 m (ratio 1.062), wet Tsukuba 2.073 m of
2.051 m (1.011), Yosemite 4.376 m of 4.115 m (1.063); none is a large
excursion, a missing contact or a lap-termination error.

The retained speed sweep (`keyboard_speed_sweep_retains_lower_speed_results`,
`.audit/track-visual-fix/keyboard-speed-sweep.json`) shows the same controller
passing wet Tsukuba at 8/10/12 m/s, Yosemite at 8/10 m/s, High Speed Ring at
10/15 m/s, while failing High Speed Ring at 8/12 m/s and Suzuka at 8/10/12/15.
The non-monotonic pattern is a 5 Hz bang-bang limit cycle, not a corridor-width
law: every failure is a small overshoot past a guide that hugs one boundary by
1.06–4.12 m. That limits this test driver; it does not establish human
playability, and the failing checkpoint is retained rather than relaxed.

The full workspace suite passed on this revision: 179 tests passed, zero
failed, and eight long, performance and local-capture probes were ignored by
default. The first screenshot test in that run
overwrote the documentation captures as documented; all 23 files under
`docs/screenshots/` were restored from HEAD and verified byte for byte.

## 13–14 November 2026 local GT4 and final publication checks

The final published head of [PR 2](https://github.com/nicolas-found42/neural-racing/pull/2)
was `923df5e84f9f6fd7e39634ff7f7db1be2a27b354`. That revision passed
68 workspace tests, with two ignored, and five query tests. The GitHub
Sourcery check succeeded. An earlier check on the PR was skipped because
the review budget was exhausted. The merge to the existing
`codex/retained-diagnostics` branch was verified at `9e5f98e` with tree
`eb6bdecdc7e9ae99aaa0de37432e2aed377cd5a1` matching the tested head.

A fresh build of the exact PR head retained four processes with source
fingerprint `8093732ccd7c3b0a`. The failed checkpoint, completed training, and
failed verifier were runs `1794632586628-20278-1`, `1794632587630-20281-1` and
`1794632588738-20282-1`. Exit codes remained 101, 0 and 1 respectively.
Run `1794632589328-20284-1` retained the waiting marker after SIGKILL and had
no shutdown row. With only the saved rows, the inference is that the process
last recorded the waiting marker and its final state is unknown. The signal
is known from the test driver's separate process status. The run summaries
reported zero drops or I/O errors on their retained rows.

On the larger local checkout, a separate Egui run drew Monza and imported
Suzuka using the production render path. Source preparation and visible
readiness were retrieved from `1794632349840-18146-1`. Operation 2 recorded
`no_source_visual` and worker 3 acknowledged the absence of a raster. Operation
4 prepared 61,725 faces, worker 3 acknowledged generation 2 with a raster, and
the GUI reported 24 accepted and resident tiles with zero pending, invalid or
discarded results. The recorder reported zero drops or I/O errors. This
establishes fallback and textured tile readiness. It does not establish
physical display completion, delayed recovery, or behavior after a GPU reset.

A separate controlled off-mesh pose marked the simulator off track with
`road query has no contact`. Reset cleared the reason. This exercised the
production state adapter; the reason is exported through the shared state
serializer and retained terminal event paths. The fixture alone cannot
establish why a real car lost road contact. The original ground test fixture
failed because it dropped unapplied egui texture deltas. Explicitly clearing the
headless test deltas fixed the fixture. The tracked and untracked documentation
screenshots were backed up and restored byte-for-byte.

The larger workspace passed 184 tests with nine ignored. `cargo check
--workspace --all-targets` and the optional Metal suite passed (27 tests).
Both Python suites passed (five query and five offline GT4 tests). A historical
GT4 course probe receipt was readable but its input verification reported
changed files, including stations, manifests and Rust inputs. Its test outcome
remains attributable to the historical input set only. No new all-route native
oracle or sustained performance qualification was run.

Local evidence and exact-source PR probes are under `.audit/observability-nov13/`.
Use `python3 scripts/diagnostics.py --dir .audit/observability-nov13/ground
--run 1794632349840-18146-1` or replace the directory with `published-evidence`
and a PR run ID above. The GT4-specific source changes stayed in the local
checkout because its imported assets and runtime owners are unpublished.

Two additional targeted fixtures passed after that workspace run. The terminal
attempt probe initially failed to compile because it used a JSON crate name not
in the test's dependencies. It now uses the shared diagnostics value parser.
The fresh query for run `1794632721270-22183-1`, operation 2, returned car 0,
attempt 1, `Crash`, step 1 and `departure_reason="road query has no contact"`.
This verifies the adapter-to-retained-event path, including retrieval in a
separate process. The pose was deliberately moved off the mesh.

The isolated tile disconnect fixture uses closed worker channels and a
synthetic three-second pending interval. Run `1794632784124-22767-1`, operation
2, retained one `ground_worker_disconnected` at `result_receive` and one
`ground_visible_delayed` with one pending request and zero accepted tiles.
Ten observations of the same condition emitted no duplicates. It establishes
the evidence contract for an unresolved view after channel loss. Automatic
worker restart, a genuine GPU stall and the complete outage/recovery cycle
were not exercised by this fixture.

```sh
python3 scripts/diagnostics.py --dir .audit/observability-nov13/contact \
  --run 1794632721270-22183-1 --operation 2
python3 scripts/diagnostics.py --dir .audit/observability-nov13/tile-loss \
  --run 1794632784124-22767-1 --operation 2
```

A fresh local live probe, run `1794632799445-22789-1`, retained three updates,
a circuit change, one `live_error`, the subsequent `probe_recovered` and the
recorder end. The error was non-finite inference and the driver restored a
valid policy. No queue drops were observed. Use
`python3 scripts/diagnostics.py --dir .audit/observability-nov13/live --run
1794632799445-22789-1 --event live_error --event probe_recovered` to retrieve
the before/after decisions. The results verify rejection and subsequent
progress, with the original NaN source known only from the synthetic setup.

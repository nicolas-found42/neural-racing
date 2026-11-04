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

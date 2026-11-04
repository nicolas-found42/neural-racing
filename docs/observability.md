# Retained diagnostic evidence

Audited 4 November 2026 across all four application crates, executable entry points,
examples, test harnesses, manifests, existing diagnostic artifacts and reference
assets. This is a local desktop application; it has no application server,
database, remote API, distributed queue, or network retry workflow. Native windowing,
wgpu and Accelerate are external dependency boundaries. Existing user edits were
preserved; this pass adds a fifth, shared diagnostics crate.

A newly built executable now writes bounded JSONL evidence automatically. Existing
binaries and past sessions do not gain this instrumentation retroactively. Live
weights, optimizer state and complete trajectories still are **not saved**.

## Coverage inventory

“Inspected” means the application source and its relevant failure/retention
boundaries were inspected; it does not imply every possible incident was exercised.
Runtime and historical evidence are distinguished below. Source links name the
implementation, not proof that it ran in a deployed session.

| Area / status | Previously visible and retention | Added evidence / retrieval | Actual local verification |
| --- | --- | --- | --- |
| Startup and shutdown — inspected: [GUI main](../crates/neural_racing_gui/src/main.rs), [train](../crates/neural_racing_engine/src/bin/train.rs), [verify](../crates/neural_racing_engine/src/bin/verify.rs), engine examples | Console errors and panic payloads; retention depended on caller redirection, no run identity | `process_start/end`, safe panic location, GUI exit result, CLI config; every segment carries build/platform identity | Separate process exits, panic and SIGKILL; headless CLI run. Native `eframe::run_native` initialization failure not injected |
| Controls, Manual/Watch races — inspected: [app](../crates/neural_racing_gui/src/app.rs), [sim](../crates/neural_racing_gui/src/sim.rs) | Current UI state, trails/skids and outcomes only; all lost at exit | `ui_control/config/health`, `race_restarted/ended`, distinct race operation IDs; mode, seed, geometry, terminal state | Headless rendered UI regression tests; live render telemetry probe. Complete manual input sequence is intentionally absent |
| Background coordinator — inspected: [learning](../crates/neural_racing_gui/src/learning.rs) | Latest snapshot replaces previous frame; errors were only in snapshot; send/join failures discarded | Requested/received/applied command IDs, queue latency, revision, worker/session IDs, reset linkage, disconnect/join outcome, 10-second worker health | Real worker pause, track switch, reset, resume, stop; retrieved and checked all acknowledgments after shutdown |
| Live learning — inspected: [live](../crates/neural_racing_engine/src/live.rs) | Last 160 attempts **per circuit**, current counters, in-memory best policy; all lost on exit | Session seed/config, track epochs, sampled terminal state with eight recent sparse decisions, exact outcome totals, update metrics, audit/restore events, non-finite inference/environment errors | Real updates, synthetic non-finite inference and recovery, forced stall/timeout/restart, collapsed policy restore; retrieved disk evidence |
| Parallel training/evaluation — inspected: [engine](../crates/neural_racing_engine/src/lib.rs) | Stdout summaries; optional `train --log PATH` truncates file on startup, no time/run correlation, write errors ignored | Generation operation propagated into rollout threads; worker/episode/track seeds and variants; start/end, evaluation sample rows and totals; checkpoint roles; legacy log write failures | One real generation, rollout, evaluation and three checkpoint writes; failed verification process retains result |
| ML update and persistence — inspected: [train](../crates/neural_racing_ml/src/train.rs), [policy](../crates/neural_racing_ml/src/policy.rs), [Adam](../crates/neural_racing_ml/src/adam.rs), [matrix](../crates/neural_racing_ml/src/matrix.rs) | KL/error results passed upward; direct binary save overwrites; files have format version but no incident timeline | Sampled update KL/scale/norm/duration plus exact rollback count; load/save start/end, error kind, byte count/content fingerprint; checkpoint generation and role | Existing numerical/rollback tests, actual missing/invalid file loads, successful save/load, real restore fixture. Power-loss-safe model writes are not implemented |
| Physics, sensors, tracks, RNG — inspected: [env](../crates/neural_racing_core/src/env.rs), [vehicle](../crates/neural_racing_core/src/vehicle.rs), [track](../crates/neural_racing_core/src/track.rs), [circuits](../crates/neural_racing_core/src/circuit.rs), [RNG](../crates/neural_racing_core/src/rng.rs) | Deterministic seeded simulation, assertions/tests, current env; generator's 512-rejection fallback silent | State captured at workflow boundaries; geometry ID includes widths; rare `track_generation_fallback` captures seed/rejection count | Regression tests on all real circuits, procedural geometry, tire/steering dynamics; fallback branch inspected, not forced end to end |
| Painting/dashboard and bundled assets — inspected: GUI render/widgets/palette, circuit coordinate arrays, docs/reference | Visual gauges are not a persistent telemetry source; PNGs and published benchmark reports survive until overwritten/deleted | UI health records max frame interval, frame count, snapshot age, stale revision count and worker liveness | Actual egui/wgpu render probe; no GPU command capture. Reference images/GeoJSON/licenses have no runtime logging requirement (not applicable) |
| Diagnostic transport/storage/query — inspected: [recorder](../crates/neural_racing_diagnostics/src/lib.rs), [build identity](../crates/neural_racing_diagnostics/build.rs), [query](../scripts/diagnostics.py) | Absent | Bounded async local store; sequence/loss counters; query by time/run/operation/event, explicit partial/missing-data notices | Rotation, expiry, active-file locking, disk-unavailable/recovery, queue overflow, oversized event, partial line, redaction, fresh-process retrieval |
| Other repository artifacts — inspected | README, validation/capacity reports, screenshots, shipped policy, old `.audit/perf-shared-comparison.jsonl`, empty root file `8`; no additional executable source found in scratch/audit inventory | Existing artifacts preserved; reports are claims from past runs, not incident traces | Old JSONL contains 10 performance rows with empty failure lists, but **no timestamps or commit IDs**. The default diagnostic directory did not exist before this pass |
| OS crash logs, GPU driver internals, external monitoring — inaccessible/not established | No exported crash report or connected historical telemetry source was supplied | Panic location and health narrow application boundaries only | OS-level native crashes, OOM, power loss, suspend/resume and driver hangs not injected; remote retention/access is unknown |
| Network services, database history, distributed retries — not applicable | No such application boundary exists in the inspected code/manifests | None added | No claim about infrastructure outside this repository |

## Priorities and acceptance checks

| Priority; impact / likelihood / diagnostic value | Gap or scenario | Acceptance / outcome |
| --- | --- | --- |
| P0; high / high / high | Yesterday's live failure vanishes at exit; build/config unknown | New process can query prior run, identify build, session, operation, sequence, state and shutdown. **Implemented and exercised** |
| P0; high / medium / high | Pause/reset/track change appears ignored, duplicated or out of order | Match command request → receipt → application with worker, revision and reset session lineage; missing acknowledgment remains unknown. **Implemented and exercised** |
| P0; high / medium / high | Non-finite inference poisons state; worker error disappears | Reject non-finite inference before advancing that car; persist reason/counters/state; valid restored policy advances. **Implemented and exercised**. This does not identify the original cause of a numerical failure |
| P1; high / medium / high | Late policy collapse or rejected update looks like unexplained regression | Retain audit observation, previous best, restoration and following decisions; retain rollback totals and sampled update details. **Implemented; restoration and continued stepping exercised** |
| P1; high / medium / high | Missing/incompatible/truncated policy or checkpoint partial failure | File operation start/end, safe error kind and byte/content identity; save role/generation; never record raw path. **Implemented and exercised**. A missing end cannot distinguish kill from stalled I/O |
| P1; high / medium / high | Telemetry unavailable or overloaded creates false confidence | Application result unchanged; losses/partial records/missing run made explicit; store recovers; shutdown marker survives drained flood. **Implemented and exercised** |
| P1; medium / medium / high | UI stalls while worker continues, or worker stalls inside optimizer | Compare UI snapshot age/frame intervals with worker publish age and update duration. **Instrumented; healthy rendering exercised**. A driver/optimizer deadlock was not injected |
| P2; high / low / medium | Arbitrary native crash or power cut | Need OS crash report/backtrace or external watchdog and independent durable collection. **Remaining integration work**; absent `process_end` alone is not a diagnosis |
| P2; medium / high at Max / medium | Sampling/rotation loses exact actions or an old session | Tool warns; sampled fields and caps documented. Exact replay would need deliberate checkpoint/input retention. **Intentional limit**, not promised by this pass |

The documented historical steering regression and late learning collapse in
[learning-validation.md](learning-validation.md) informed the scenarios. They do
not establish the cause of a new incident. Missing file, NaN injection, forced
stall/timeout, panic, queue saturation and SIGKILL probes are **synthetic**.

## Store, limits and privacy

- Default: `$HOME/.local/state/neural-racing/diagnostics/nr-<run>-<segment>.jsonl`.
  Set `NEURAL_RACING_DIAGNOSTICS_DIR` to another local directory. Set
  `NEURAL_RACING_DIAGNOSTICS=off` to opt out. Libraries stay silent unless their
  executable/test explicitly calls `init` once and retains its guard until workers stop.
- One background writer, nonblocking producer sends, queue of 256 events. Records
  above 8 KiB are dropped and counted. No per-physics-step disk I/O. Current callers
  emit bounded numeric snapshots and code-defined labels, not arbitrary user text.
- Segments are at most 2 MiB, last 32 per run (64 MiB). Age limit is seven days
  since last file modification; global target is 256 MiB. Cleanup runs on opening,
  rotation, and after 60 seconds of event activity. Active files are locked and
  excluded from deletion, so the global limit is **soft** (up to active segments,
  or more if filesystem deletion fails). No retention SLA: high volume can evict
  yesterday before seven days. Copy incident files to an approved archive promptly.
- Live attempt details: first four and every 1,000th, plus all five-lap goals.
  Recent history is eight samples per car at 20-decision intervals, plus terminal
  state; cleared on attempt restart/track change/restore. Live updates: first three,
  each 100th, and updates taking at least one second. Audits/restores/errors and
  commands are unsampled before queue limits. Live/worker/UI health every ten seconds
  while their respective loop runs; reset/stop also capture live totals. Evaluation
  detail: first four and every 100th track. Headless rollout start/end is unsampled.
- `live_progress.end_counts` = `[goal, crash, stalled, timeout]`, cumulative per live
  session; updates/attempts continue over circuit changes, reset creates a new session.
  History tuples = `[step, speed, distance, lateral_offset, steer_command, throttle_command]`.
  Distances/speeds use core world units; divide by `UNITS_PER_METRE` for metres/m/s.
  A decision is 0.05 simulated seconds. Requested infinite speed is encoded as
  `null` plus `uncapped:true`; other non-finite numbers serialize as `null`.
- Every record has wall-clock Unix milliseconds, monotonic elapsed milliseconds,
  schema, run, sequence, operation, and cumulative dropped/I/O-error counters.
  Sequence numbers are assigned before enqueueing; concurrent threads can arrive
  out of sequence. Use IDs and request/ack pairs for causality, not file order.
  Source fingerprints cover workspace Rust source, manifests and Cargo.lock;
  fingerprints are comparison keys, **not security hashes**, and do not recover code.
- Each segment repeats revision, source fingerprint, version, build profile,
  OS/architecture, math backend and retention/sampling policy. A git revision alone
  cannot identify this dirty working tree. Keep the corresponding source/artifact.
- Writes are synced approximately once per second; normal shutdown drains/syncs
  with bounded waits. Panic hook tries a 300 ms flush and retains only source
  basename/line and operation, never panic payload. SIGKILL/abort/power failure can
  lose queued records and the tail; successful write is not proof of physical-media
  durability. No network-backed filesystem durability testing was performed.
- New Unix directories/files request 0700/0600. Existing directory access controls
  remain the owner's responsibility. No raw CLI, environment dump, user identity,
  path, observation vector, model parameters, or panic payload is persisted.
  Existing stderr/optional legacy logs have their previous behavior and protections.
  Telemetry I/O failure warns once on stderr and is retried in the writer; it never
  changes the race/training result. If *all* writes fail, no disk record can prove it.

## Investigation runbook

Run from the repository root; Python uses only the standard library.

```sh
# Discover retained runs/builds, losses, sequence gaps and missing shutdowns.
python3 scripts/diagnostics.py --summary

# Incident window: always include a timezone. Epoch milliseconds also work.
python3 scripts/diagnostics.py --since 2026-11-03T00:00:00-05:00 \
  --until 2026-11-04T00:00:00-05:00 --limit 10000

# Replace RUN and OP with values returned by the query.
python3 scripts/diagnostics.py --run RUN --operation OP --limit 10000
python3 scripts/diagnostics.py --run RUN --event live_error --event panic

# Evidence created during this audit (local, gitignored; not production history).
python3 scripts/diagnostics.py --dir .audit/observability/evidence --summary
```

1. Confirm the directory, time window, build fingerprint and first retained sequence.
   Exit 2 means no matching evidence, unreadable files or malformed/partial records;
   stderr explains which. Valid records are still returned. `--summary` covers entire
   retained runs intersecting the time window. Event queries emit at most 200 rows
   by default, max 10,000; narrow the window when output is limited.
2. Follow operation IDs within one **run**. For live learning, use `live_started`'s
   operation as the session ID; its `parent` is the worker. Commands use their own
   operation and reference that worker; applied events reference the live session.
   `--operation` matches the operation and direct parent/session/worker/command
   references, not an unlimited transitive trace. Follow the next ID explicitly.
3. Reconstruct request → acknowledgment → state epoch → update/outcome → recovery.
   Check loss counters, sequence gaps and sampled-detail limits before concluding
   that an event never happened. A missing end could mean still running, kill, lost
   tail or deletion. A `process_end` during unwind is not a clean exit.
4. Compare last UI and worker health and optimizer durations for a stall. Compare
   checkpoint role/generation, policy content IDs and load result for persistence
   problems. Match procedural seeds and geometry IDs to source/build context.
5. State supported facts, hypotheses and unknowable facts separately. Preserve raw
   segment files before cleanup; record exact query/timezone. Reproduce on an isolated
   model/output directory, then verify the behavioral acceptance check and retrieve
   its evidence. An emitted event alone proves neither root cause nor a fix.

See [observed verification and incident reconstructions](observability-verification.md).

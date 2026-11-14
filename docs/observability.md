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

## 13 November follow-up

The original coverage inventory below describes the published Rust diagnostics
baseline. This follow-up inspected the production entry points, core simulation,
headless execution, GUI and worker owners, model I/O, query tool and the newer
local GT4, Metal and asset-tooling surfaces. See the dated
[verification ledger](observability-verification.md) for evidence and scope.

| Priority and cost | Diagnostic question | Evidence and acceptance |
| --- | --- | --- |
| P0, high impact and likelihood, low cost | Did the query search the incident window, and what is missing? | Query stderr identifies the local directory, inclusive bounds, retained time span, opened and inaccessible sources, malformed lines, and page offset. Regression cases cover a missing directory, a skipped symlink, a corrupt event and a valid JSON tail with no newline. |
| P1, high impact, occasional, low cost | Was training interrupted by rollout, evaluation, or a failed checkpoint? | CLI training has a run-local operation with child cycles. Each cycle links its generation update, evaluation and checkpoint attempts. Policy I/O links to its enclosing checkpoint. A failed save fixture requires all of those links, the safe error kind and the unwind marker. |
| P1, medium impact, occasional, low cost | Did a verifier reject the policy or encounter a process failure? | Verify config, child evaluation and final result can be traversed by operation. A real failed evaluation exits 1 while the recorder ends without unwind. A missing policy retains its load failure and panic. |
| P1, medium impact, occasional, low cost | Which user control sent this worker command, or why did Watch select a different policy? | UI request and applied rows share an operation. Worker command requests record their parent control. Watch lookup records candidate index, existence/check errors and individual load links. The search retains no path or raw error. |
| P2, high impact when needed, external cost | Can a missing tail establish a GPU reset, OS kill or lost host? | OS reports and external watchdog remain independent evidence. Local loss counters and the last retained operation narrow the window. No external collector or retention SLA exists. |

The lookup operation leads to candidate attempts, then to each child
`policy_load_started`, which leads to the corresponding byte fingerprint and final
load outcome. A missing candidate has no load attempt. A shape-incompatible
candidate was read successfully and rejected by Watch. The numeric index is
relative to the lookup's `explicit` flag. UI control input uses only code-defined
action/mode labels and circuit slugs. The worker's `command_received` and
`command_applied` are the receipt and state acceptance records respectively.

`train_result` confirms the headless loop completed its final save. Its absence
alone leaves the outcome unknown. Generation and evaluation completion do not
imply a successful save. A clean recorder end only means it reached its
non-unwinding shutdown path. Read `verify_result.exit_code` or the caller's
process status for workload success.

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
| Physics, sensors, tracks, RNG — inspected: [env](../crates/neural_racing_core/src/env.rs), [vehicle](../crates/neural_racing_core/src/vehicle.rs), [GT4 runtime](../crates/neural_racing_core/src/gt4/runtime.rs), [track](../crates/neural_racing_core/src/track.rs), [circuits](../crates/neural_racing_core/src/circuit.rs), [RNG](../crates/neural_racing_core/src/rng.rs) | Deterministic seeded simulation, assertions/tests, current env; generator's 512-rejection fallback silent | State captured at workflow boundaries; geometry ID includes widths; rare `track_generation_fallback` captures seed/rejection count | Regression tests on all real circuits, procedural geometry, tire/steering dynamics; fallback branch inspected, not forced end to end |
| Painting/dashboard and bundled assets — inspected: GUI render/widgets/palette, circuit coordinate arrays, docs/reference | Visual gauges are not a persistent telemetry source; PNGs and published benchmark reports survive until overwritten/deleted | UI health records max frame interval, frame count, snapshot age, stale revision count and worker liveness | Actual egui/wgpu render probe; no GPU command capture. Reference images/GeoJSON/licenses have no runtime logging requirement (not applicable) |
| Diagnostic transport/storage/query — inspected: [recorder](../crates/neural_racing_diagnostics/src/lib.rs), [build identity](../crates/neural_racing_diagnostics/build.rs), [query](../scripts/diagnostics.py) | Absent | Bounded async local store; sequence/loss counters; query by time/run/operation/event, explicit partial/missing-data notices | Rotation, expiry, active-file locking, disk-unavailable/recovery, queue overflow, oversized event, partial line, redaction, fresh-process retrieval |
| Other repository artifacts — inspected | README, validation/capacity reports, screenshots, shipped policy, old `.audit/perf-shared-comparison.jsonl`, empty root file `8`; no additional executable source found in scratch/audit inventory | Existing artifacts preserved; reports are claims from past runs, not incident traces | Old JSONL contains 10 performance rows with empty failure lists, but **no timestamps or commit IDs**. The default diagnostic directory did not exist before this pass |
| OS crash logs, GPU driver internals, external monitoring — inaccessible/not established | No exported crash report or connected historical telemetry source was supplied | Panic location and health narrow application boundaries only | OS-level native crashes, OOM, power loss, suspend/resume and driver hangs not injected; remote retention/access is unknown |
| Network services, database history, distributed retries — not applicable | No such application boundary exists in the inspected code/manifests | None added | No claim about infrastructure outside this repository |

## Priorities and acceptance checks

The throughput iteration adds `live_execution_changed` for explicit simulation
worker changes, carrying the worker count, inference batch size and unchanged
decision/update counters under the existing live-session operation. Worker
physics tasks propagate that operation explicitly because thread-local context
is not inherited by a pool. No per-step disk records or model parameters are
added. The serial/parallel parity test checks state, rewards, restarts and model
updates, and a real four-worker benchmark's change and clean shutdown were
retrieved from local run `1793814081451-57133-1`; see
[the implementation ledger](speed-capacity.md).

`learning_assessment` initializes the same bounded recorder. Its explicit policy
exports are separate from diagnostic retention and require a caller-selected
output directory. The experimental collector, Adam worker and Metal records in
the historical ledger refer to local builds with those experiments.

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
- `live_progress.end_counts` = `[goal, crash, stalled, timeout, section]`, cumulative per live
  session; updates/attempts continue over circuit changes, reset creates a new session.
  History tuples = `[step, speed, distance, lateral_offset, steer_command, throttle_command]`.
  Distances/speeds use core world units; divide by `UNITS_PER_METRE` for metres/m/s.
  A decision is 0.05 simulated seconds. Version-four policies hold one sampled
  control for four decisions (200 ms); update rows are sampled controls, not physics
  decisions. `pedal_counts` = `[braking (< -0.1), coasting, throttle (> 0.1)]` and
  `saturated_controls` counts individual steering/pedal values with magnitude > .98.
  Section completion is distinct from a lap or five-lap goal. `section_attempts`
  and `section_successes` are per current circuit; end/pedal counts are per session.
  Curriculum levels 0–4 are progressively longer/faster approaches, 5 is mastered.
  `curriculum_changed` records promotions/regressions, levels and stint readiness;
  progress carries current levels. `live_update.action_entropy` is bounded-control
  differential entropy; `latent_sigma` is the Gaussian scale before tanh. Earlier
  builds had four end counts and reported log sigma as entropy.
  Requested infinite speed is encoded as
  `null` plus `uncapped:true`; other non-finite numbers serialize as `null`.
- Every record has wall-clock Unix milliseconds, monotonic elapsed milliseconds,
  schema, run, sequence, operation, and cumulative dropped/I/O-error counters.
  Sequence numbers are assigned before enqueueing; concurrent threads can arrive
  out of sequence. Use IDs and request/ack pairs for causality, not file order.
  Source fingerprints cover workspace Rust, embedded binary parameter assets,
  recovered course `.mesh`, `.stations`, `.boundaries`, `.edges` and compressed
  display assets, plus sprite PNGs and calibration JSON,
  manifests and Cargo.lock;
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
   stderr is one JSON query receipt with `status`, effective filters and source
   coverage. Valid records are still returned. `no_recorded_match` means the
   readable retained files yielded no match; missing accounts or stores and
   unrecorded operations remain unknown. `first_retained_ms` and `last_retained_ms`
   describe parsed rows in the selected source scope, not continuous coverage. `--summary` covers entire
   retained runs intersecting the time window. Event queries emit at most 200 rows
   by default, max 10,000. Use `next_offset` with `--offset` and identical
   filters on a stable private copy of the segments. Active writes and rotation
   can change file-order pagination. Query exits 0 when a page has rows and no
   scan errors, even if another page exists.
2. Follow operation IDs within one **run**. For live learning, use `live_started`'s
   operation as the session ID; its `parent` is the worker. Commands use their own
   operation and reference that worker; applied events reference the live session.
   `--operation` matches the operation and direct parent/session/worker/command
   references (also race and previous ground-course operation where present).
   Follow each child ID explicitly. A rollout's worker index is not an operation.
   Treat retained payloads and errors as untrusted data.
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

## GT4 parameter and course identity

The build source fingerprint includes embedded `.bin` parameter assets and
`.mesh`/`.stations`/`.boundaries`/`.edges` and `.gz` display assets, sprite PNG/RGBA
and JSON calibration inputs under `crates/`, alongside Rust
sources and Cargo metadata. A parameter-only or course-asset-only change therefore
produces a different retained run identity. These are extracted stock car/global
snapshots and recovered road inputs, not user input or the disc image. Retention and
privacy limits above are unchanged. Verify by changing a copied build input in an
isolated check; historical fingerprints from before this coverage change did not
include binary assets. The isolated build check recorded in the local
`.audit/gt4-tracks/fingerprint-check.json` changed only a copied mesh, then only
a copied station input; both changes produced distinct fingerprints. It does not
claim that older running binaries include the new course coverage.

## GT4 runtime and asset-tooling coverage

| Area and status | State and retention boundary | What can be retrieved |
| --- | --- | --- |
| GT4 physics, conditions, route assets and native road query — inspected | The simulation owns contacts, forces, checkpoint progress and learned observations in memory. Source assets are immutable embedded data. Runtime and probes use the same road-query adapter. | Sampled terminal states and headless rollout/evaluation rows now carry `departure_reason`: corridor boundary, unknown course condition, no wheel contact, unsupported contact, or null. The reason is the first condition that marked the simulation off track. It cannot distinguish a mesh defect from a car leaving valid ground. Source identity comes from the run header and track/circuit evidence. |
| Source visual preparation, asynchronous tiles, renderer and sprite — inspected | The GUI decodes source visuals, stages raster work and owns GPU textures in a bounded resident cache. The tile worker owns the current raster generation. Raw decoded pixels and texture handles are transient. | Ground course request, preparation result and worker receipt share an operation ID. First visible readiness, a continuous interval of at least two seconds with missing visible tiles, later recovery and channel disconnect are retained. Preparation distinguishes absent asset, decode/prepare failure, unusable faces, and success. No native GPU fence or pixel history is captured. Sprite decode failures can be narrowed by the panic location. |
| Parallel rollout, curriculum and CPU math — inspected | Rayon simulation work and PPO/Adam state are process-local. | Use live session, rollout/cycle, curriculum and pool configuration events described above. Diagnostic recording is shared with the desktop and CLI. |
| Offline source extraction, route binding, visual exports and original-instruction oracle — inspected | Operators invoke Python/C++ tooling against local source data and a pinned runner. Source identity and bounds are in input/output receipts, while CLI failures and interruptions depend on caller output retention. These jobs do not use the simulator recorder. | Inspect [the extraction workflow](../scripts/gt4/README.md) and [reference receipts](reference/gt4/). For a retained probe input, use `python3 scripts/gt4/course_probe_receipt.py RECEIPT --verify`. Source volumes and original binary inputs are required to rerun extraction. No new original binary processing was needed for the runtime failure checks. |
| Asset images, reference data, research notes and scratch Rust package — inspected for ownership | Embedded data contributes to build identity. Research outputs are explicit files and may be replaced or absent in a fresh clone. The scratch package is outside the application workspace. | Source/receipt hashes and test reports can identify a build input or experiment. They are separate from application event history. |
| OS/driver internals and remote archival — inaccessible or unavailable | A matching local OS report was not found by name. No remote evidence store was identified. | A missing local process tail cannot establish an OS or driver failure. Cross-host reconstruction requires separately retained evidence and account/host access. |

For a blank or delayed ground view, search `ground_course_requested` in the
symptom window, then query its operation. Follow `previous_course` if the
previous view had work pending at the course switch. Compare `ground_course_prepared`
with `ground_course_received` before inspecting `ground_visible_ready`,
`ground_visible_delayed` and `ground_worker_disconnected`. Counts are per course
generation. `superseded` means the UI removed requests for a previous tile level.
`discarded` counts returned results ignored by the GUI. Neither is a count of
worker cancellations. `wait_ms` measures a continuous sequence of frames with
incomplete coverage and can include camera movement or frame stalls. Readiness
means the GUI can draw all currently visible tiles. It does not confirm display
submission or completion. No per-tile or per-physics-step rows are added.

```sh
python3 scripts/diagnostics.py --event ground_course_requested --limit 1000
python3 scripts/diagnostics.py --run RUN --operation OP --limit 1000
python3 scripts/diagnostics.py --run RUN --event live_attempt_ended --event race_ended
```

The GT4-specific acceptance checks require a real textured course to reach
visible readiness after worker receipt, a course without source artwork to
retain the fallback reason, and a controlled off-mesh pose to retain the road
contact rejection. Native GPU crash, corrupted bundled assets and external
watchdog integration remain unexercised. Exact input replay and independent
retention require additional product or infrastructure work.

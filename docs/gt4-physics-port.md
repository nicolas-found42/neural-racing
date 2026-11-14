# GT4 physics extraction and port

Status: selected vehicle port integrated and verified, 4 November 2026. Manual
and training controls now use the recovered stock BMW M3 GTR ALMS 2001 model.
The earlier independently tuned vehicle has been replaced by the adapter in
`crates/neural_racing_core/src/vehicle.rs`; native state and frame sequencing live
in `gt4/runtime.rs`. The separate local integration prototype checked generic
body API was incorporated and verified, but generic `0x3e2b28` is not the normal
car's integrator.

## Scope and evidence

The selected configuration is the original four-wheel FR model, native manual
car mode zero, automatic gearbox, analog controller packet type two, initialized
secondary steering mode one, stock equipment and asphalt surface zero. Both
human and policy controls become the same 12-bit packet. The original flag
`car+0x5b5` bit zero disables original-course track-centering assistance on this
game's courses. Optional wear, pit control and damage extensions are off; this
is an explicit race configuration. The initial port used an explicit horizontal
asphalt plane. Recovered GT4 courses now supply the original road mesh, elevation,
contact normals, numeric surfaces and flags through the [road-query adapter](reference/gt4/road-contact.md).
The three geographic circuits retain their plane configuration. Circuit
boundaries and race scoring remain this game's responsibility.

Native routines provide wheel slip, the compound's five tire tables and combined
force coupling, normal loads, spring/damper travel, camber, anti-roll response,
pitch/roll, tire steering compliance, engine/clutch, differential, wheel rotation,
automatic gears, TCS/ASC/ABS, brake heat and force-based chassis motion. No
kinematic yaw blend, velocity damping, invented friction circle, or representative
torque curve is added by the adapter. Source identities and coverage are in
[the port ledger](../crates/neural_racing_core/src/gt4/PORT_LEDGER.md) and
[frame coverage](reference/gt4/frame-coverage.md).

The selected equipment, expanded parts, CarData and globals were constructed by
unchanged original instructions. An independent original database/constructor
execution using upstream Play! kernel HLE matched the offline builder's complete
outputs byte-for-byte. The production snapshots and exact initialization recipe
are documented in [the data README](../crates/neural_racing_core/src/gt4/data/README.md)
and [constructor proof](reference/gt4/native-constructor.md). This is a verified
selected numeric initialization, not a claim of a complete GT4 boot.

The ISO SHA-256 is
`67b6c0075837f3ae1132d608acf2858bf13b2dd62d6eae83dff76df02e4e824f`.
Its decompressed CORE segment bytes match GT4 NTSC-U v1.01. The reconstructed
analysis ELF SHA-256 is
`e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`;
its headers/BSS layout are reconstructed metadata, while loaded source bytes are
unchanged. Real REA MCP R5900 inspection, original C/assembly views, request/reply
Evidence and a bundle are retained under `.audit/gt4-port/run-01/`.

## Original-instruction comparisons

The [final validation receipt](reference/gt4/validation-receipt.json) binds the
2,319/2,319 passing main comparison groups, 21 complete motion/contact cases and
24 contact-tail cases to their target, emulator, parameter and source identities.

The reference executes unchanged CORE instructions and original callees using
upstream Play! commit `1f27232772e484bfb79364b751b84f41f4c87481`. It includes
EE COP1/COP2/MMI behavior, GP initialization, toward-zero/flush-to-zero arithmetic
and scalar ADD/SUB guard alignment. A separately hashed negative control changes
lookup MUL to ADD and changes the result from 35 to 31.5; the canonical target is
unchanged. This is controlled emulator execution, not physical PS2 measurement.

Portable fixtures cover scalar/curve helpers, all 14 compounds and five force
volumes, tire force chains, drivetrain/control, normal-load and attitude stages,
steering, gearing, engine preparation and whole force frames. Inputs are frozen
with their parameter bytes so later initialization fixes cannot silently change
the meaning of old goldens. Most numeric comparisons use a 12-ULP bound; flags,
packed fields and configuration bytes remain exact. Near-zero cancellation
requires output-specific bounds derived from original operation inputs and
per-instruction rounding, documented with the corresponding fixtures and
[the yaw error proof](reference/gt4/yaw-forward-error.json).

The assembled motion/contact path passes 21 original-code cases, including
penetration edges, comparing 232 outputs per case with a maximum observed
difference of 8 ULP and exact contact flags, free-wheel flags and integer mileage.
Its portable fixture is `gt4_original_post_pipeline.json`. The original contact
tail has 24 additional whole-call cases, including free-wheel/carrier refresh,
mileage and filters. These are controlled source-frame comparisons, not a full
GT4 game boot or whole-game equivalence claim. The later original-course query
work has separate evidence in [course resources](reference/gt4/course-resources.md)
and [road inputs](reference/gt4/road-contact.md). Additional cars, scene collisions,
motorcycles, dynamic weather, wear and full-game runtime behavior remain outside
this selected vehicle port.

## Game adapter and acceptance

Native positions use metres, forward body velocity `+0x640`, right body velocity
`+0x644` and heading `+0x674` with forward `[-sin(theta),cos(theta)]`. The adapter
uses a local origin to preserve track placement, converts right velocity to the
game's signed lateral velocity, and publishes the actual native position at each
60-Hz update. Three updates retain the existing 20-Hz decision cadence. Tire
telemetry is a diagnostic view; its estimated demand ratio never constrains the
native forces. Rendering retains its established wheel ordering.

Policy file version four rejects version-two and version-three weights, which
share observation shapes but used a different car or control cadence. Fresh
training holds each sampled action for 200 ms and uses the new model;
no claim of learned lap competence follows from a feasible test driver.

Current integration checks: 0–100 km/h in 3.60 seconds, approximately 1.45g in the
30 m/s full-input turning scenario, 7.69 degrees maximum rear slip in the original
one-second keyboard launch regression, keyboard-only test-driver lap completion
on all three real circuits at 200 ms control intervals, and five-lap continuous
test-driver completion on all three circuits. Full-lock sustained coasting or hard
braking can produce a recoverable slide; the 5–80 m/s release checks return below
0.05 rad/s. These are automated feasibility results, not human playtest results or
manufacturer calibration claims. At the initial vehicle-port checkpoint, the workspace suite passed **115 Rust tests**, zero failures, with two
performance probes intentionally ignored. The release GUI rebuild passes and
the actual app was opened in manual mode and exercised with a throttle key.
Existing screenshots were restored byte-for-byte after rendered UI testing.

## Real parameter data

The supplied `GT4.VOL` was extracted read-only from the ISO. Its RoFS 3.1 index
contains **18,562 entries**. A bounded parser recovered **1,801 SpecDB files**,
preserving original packed ranges and SHA-256 hashes. DBT Huffman/template-delta
decoding and IDI label joining recovered these tables:

| Table | Rows | Bytes per row |
| --- | ---: | ---: |
| TIRECOMPOUND | 14 | 90 |
| TIREFORCEVOL | 5 | 18 |
| STEER | 3 | 38 |
| TCSC | 3 | 10 |
| ENGINE | 723 | 113 |
| CHASSIS | 722 | 37 |
| DRIVETRAIN | 845 | 21 |
| GEAR | 2,270 | 45 |
| SUSPENSION | 2,573 | 111 |
| FRONTTIRE / REARTIRE | 9,012 each | 59 / 57 |

The real compound labels include `gt4_ec`, `gt4_st`, `gt4_sp`, `gt4_ch`,
`gt4_cm`, `gt4_cs`, `gt4_rh`, `gt4_rm`, `gt4_rs`, `gt4_rq`, `gt4_rsh`,
dirt and snow profiles. Raw database bytes alone are not initialized physics parameters. The native
byte-to-float scaling, curve construction and selected M3 car/part relationships
now have the independent constructor proof linked above.

The extraction/parser work follows the documented layouts in
[GT4FS](https://github.com/Razer2015/GT4FS) and
[PDTools SpecDB](https://github.com/Nenkai/PDTools/tree/2a9637d8c36100e4c211d73cd8140a9d690578d2/PDTools.SpecDB).
Those mappings are reading leads; native initializer code and independent
execution provide the port's semantic checks.

## Retained work and repeatable checks

Local, gitignored `.audit/gt4-port/` contains:

* `run-01/`: the current REA MCP requests, full replies, native dossiers and maps.
* `data/volume-index.json`: all original RoFS records and data ranges.
* `data/extracted/extraction-manifest.json`: selected packed/output hashes.
* `data/decoded/`: decoded tables with original DBT/IDI identities and raw rows.
* `data/rofs_extract.py`, `data/decode_specdb.py`: bounded read-only decoders.
* `differential/execute_original.py`: original-instruction execution harness,
  results, source-byte mutation check and explicit floating-point limitations.

The helpers are local evidence under active development, not portable artifacts
guaranteed to be present in another clone. Their outputs will be promoted into
reviewable tooling/fixtures as each validated port boundary is connected.

The selected vehicle scope above has executable comparisons. Broader GT4
equivalence is not established by these checks. Later course and learning
acceptance results have their own receipts and do not retroactively broaden the
initial vehicle-port validation.

# GT4 evidence and responsive handling — 4 November 2026

The earlier handling repair prevented launch spinouts but made road-speed
steering too weak. This iteration measures both responsiveness and stability,
inspects the supplied GT4 core through a real REA MCP session, and implements
simulator-style force allocation and power delivery in the shared game physics.
It does not claim that the complete GT4 tire model or its driving feel has been
reproduced.

## Artifact identity and REA recovery

Input: `raw/gt4/gran-turidmo-4.iso`, 5,314,478,080 bytes, SHA-256
`67b6c0075837f3ae1132d608acf2858bf13b2dd62d6eae83dff76df02e4e824f`.
The UDF/ISO container exposes `SYSTEM.CNF`, `SCUS_973.28`, `CORE.GT4` and the
game volume. The original ISO and executable bytes were not changed.

REA's installed container reader rejected the ISO, and its installed Ghidra
adapter rejected the PS2-specific MIPS flags. The system `bsdtar` extracted the
three named boot/core files. The CORE header declares raw DEFLATE at offset 6;
decompression produced exactly the declared 6,119,116 bytes. Following the
[documented CORE layout](https://nenkai.github.io/gt-modding-hub/formats/ps2_core/)
and [PDTools builder](https://github.com/Nenkai/PDTools/tree/2a9637d8c36100e4c211d73cd8140a9d690578d2/PDTools.GT4ElfBuilderTool),
the three segments were wrapped in an ELF with their original bytes/addresses,
entry `0x100008`, and original flags `0x20924001`. ELF section names and an 8 MiB
BSS allocation are analysis metadata; the BSS extent is representative, not a
recovered CORE segment. Container signature authentication was not established.

| Artifact | SHA-256 |
| --- | --- |
| Boot `SCUS_973.28` | `f8f10823160e2b5cef5c7032628134632b291b1df87a9aee3c144794dd8019fa` |
| Packed `CORE.GT4` | `85d26aa8430154967b2633eede929286694ac39e99762527edcec365fd642ff9` |
| Reconstructed `CORE.ps2.elf` | `e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804` |

The canonical memory image has SHA-1
`38cc7f0d7841d1b75e76ad3678efd813386f65ce`, exactly matching the named NTSC-U
SCUS-97328 v1.01 target in the public
[decompilation configuration](https://github.com/gustavocanepa/gt4-decomp/blob/3b1f2ae5b896201dfca6bb0cebcf024ad2986acf/config/gt4.yaml).
This identifies the comparison target; it does not certify disc provenance.

An isolated copy of [REA's dedicated PS2 support branch](https://github.com/morluto/rea/pull/6422)
at commit `61f7b8d526c58db7db0f9729a18fc7fd133af75e`, with the matching
Ghidra 12.1.4 Emotion Engine decoder, supplied `r5900:LE:32:default`.
The branch's Linux host admission needed a local Darwin arm64 extension; this
analysis path is experimental. Its profile/limitations retain that fact.
The globally installed REA/Ghidra and client registration were left unchanged.

A local stdio MCP client called actual REA `open_binary`, `binary_overview`,
`search_strings`, `xrefs`, `analyze_function`, instruction and byte readers.
The first cold query exceeded a 60-second client deadline. Its owned processes
were cleaned up before correcting the client deadline to 600 seconds; the
successful cold import took about 82 seconds. This was not solved by changing
the target's architecture flags or using a generic MIPS decoder.

The native overview reported 24,616 procedures and 17,410 defined strings;
mirrored regions can duplicate inventory entries. PS2-specific `clear`, `lq`
and `sq` instructions were independently checked against their original bytes.
Overview Evidence:
`ev_2bafd0390bdc25e89a93f28a57ea7eb7e42a5633d2763e1c3bf5346bbbae3c80`.

## Verified GT4 findings and limits

These are static observations of the supplied core, not execution or emulator
measurements. Decompiled types and names remain analyst interpretations.

* **Engine curves:** native registration ties `buildEngineCurveData` to
  `0x148b60`, then `0x34b960`, `0x34f3f8`, `0x34e9a0` and `0x359568`.
  The final helper clamps outside a bounded table, searches the enclosing knots
  and linearly interpolates between them. The caller converts angular speed
  to RPM using `60 / 2π`, samples 100-RPM intervals and computes torque/power
  values. This establishes a real engine-curve path, not a claim based on the
  word “engine” alone. Curve Evidence:
  `ev_2aa4df6dfa5fa07f1f3ce1801bb66bd39ff47c3cf00daa4257d3108369fff3c0`;
  numeric caller:
  `ev_1e22f010692c97fc0ae63b07192dc7972940311cbbf17c20ba98d0319f2ee8b3`;
  binding path:
  `ev_93aeed1eb44da1e21b5df14e12a2f376df16c8204dd9d13983a565e10a8c28f2`.
* **Rigid-body integration:** `0x3e2b28` applies gravity near 9.81, force divided
  by mass, principal-inertia angular equations and orientation wrapping. It
  uses a multi-stage **angular** update. Read angular coefficients are consistent
  with a fourth-order symplectic composition; that algorithm identification is
  an inference. Its translational branch explicitly advances position by
  velocity times timestep and then velocity by force/mass times timestep.
  A linked-list manager calls the routine. This is a 3D solver;
  copying its stage constants into this dissipative planar model would not
  establish equivalent dynamics. Evidence:
  `ev_e2d8dfd48c84d3d90e65d8439293bf6029dcffe16e45adaee896bb41b3428cad`.
* **Controls/configuration:** the core contains separate analog/digital steering
  labels, active-steering options and TCSC-setting bindings. Inspected input
  dispatch uses virtual controller calls. The actual speed-sensitive steering
  map and the exact intervention laws were not recovered.
* **Tires/suspension:** selected curve callers and numerical loops suggest
  stateful corrections, springs and dampers, but their physical fields are not
  sufficiently mapped to assert a complete tire equation. No GT4 tire curve,
  friction coefficients, per-car torque knots or suspension calibration were
  imported into this game. Collision-plane and animation/IK routines were
  excluded from tire claims after inspecting their code.

## Implemented model

The changes are independent code in `neural_racing_core::vehicle`, used by both
manual driving and training:

1. **Useful steering authority:** geometric curvature plus tire-compliance
   understeer compensation replaces the previous 60%-grip geometric request.
   Parking lock, rack speed/acceleration bounds and tire-force limits remain.
2. **Combined tire demand:** longitudinal and lateral requests are calculated
   independently, then their vector is normalized to each contact patch's
   load-sensitive capacity. This follows the structure described by the
   [PhysX vehicle documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/docs/Vehicles.html#pxvehicletireforceparams).
   It removes both the blanket 65% longitudinal cap and unconditional priority
   for either force axis. The compact pure lateral curve remains `tanh`, not
   a claim of GT4's or VDrift's exact tire implementation.
3. **Power delivery:** bounded linear interpolation of an independently tuned
   torque curve feeds a quasi-static six-ratio automatic gearbox and launch
   clutch approximation. Wheel power remains bounded at 390 kW and launch
   demand at 9 m/s². The curve structure is supported by the GT4 observations;
   all torque knots, ratios, efficiencies and shift tuning here are chosen
   parameters, not recovered GT4/BMW data. There is no engine rotational inertia
   or shift-time simulation.
4. **Slip-based assistance:** actual rear contact slip reduces drive/rear brake
   demand only as oversteer develops. Front brake bias follows axle loading.
   Active steering uses rear-slip rate to anticipate the rack's response delay
   and countersteers through actual tire forces. It never writes chassis heading
   or yaw velocity directly. These are explicit controller design choices;
   their gains/deadbands are not claimed to be GT4's. With no wheel-angular-speed
   states, these aids are not wheelspin TCS or wheel-lock ABS.

The [VDrift controller](https://raw.githubusercontent.com/VDrift/vdrift/master/src/carcontrolmap.cpp)
and [tire implementation](https://raw.githubusercontent.com/VDrift/vdrift/master/src/physics/cartire1.cpp)
provide additional primary examples of separate input conditioning and combined
slip. Their source was researched, not copied into this project. The planar
model still omits suspension travel, camber, tire temperature and full 3D motion.

## Measured acceptance

`cargo test -p neural_racing_core --test handling_response -- --nocapture` was
deterministically red on the earlier source: only 0.656g for a held turn at
30 m/s. A steering-only change reached 1.195g but failed held-throttle stability;
this prevented accepting response at the expense of spin recovery.

| Shared-physics outcome | Earlier handling | Final handling |
| --- | ---: | ---: |
| Sustained full-input turn, initial 30 m/s, either direction | 0.656g | 1.087g |
| Straight 0–100 km/h | 3.55 s | 3.55 s |
| One-second speed gain from 10 m/s, steering 0.2 | 6.64 m/s | 7.13 m/s |
| Same turn's gain relative to straight acceleration | 91.1% | 97.8% |

These are broad gameplay acceptance measures, not comparisons against GT4
telemetry. The steering test widens only the terminal track-width boundary so
ordinary walls cannot terminate the force measurement; it uses real `Env::step`,
sensors and integration. Open-skidpad integrator tests cover three-second held
turns and two-second release at 5/10/20/40/60/80 m/s under drive/coast/braking,
plus repeated 250-ms corrections. Rear slip stays below 12° and release yaw
below 0.05 rad/s. The original one-second launch peaks at 7.70° rear slip.

The actual GUI arrow/WASD path passes its launch regression. The binary-input
test driver, acting every 200 ms, completes all three circuits, and the separate
continuous driver completes five laps on each. These prove control feasibility,
not human playtesting or learned-policy competence.

`cargo test --workspace`: **84 passed, zero failed, two ignored performance
probes**. Changed Rust formatting and `git diff --check` pass. The release GUI
build passes. Existing screenshot files were restored byte-for-byte after the
suite. No performance benchmark or fresh learned-policy assessment was run.
Observation layout and policy file format stay unchanged; older weights'
handling competence is unverified after these dynamics changes.

## Retained local evidence

`.audit/gt4-handling/` is gitignored and contains the extraction/reconstruction
script, artifact hashes, before/after physics, red/green test logs, saved screenshot
outputs, primary-source research and the isolated REA environment. Its
`rea-support/responses/` records actual MCP requests/results;
`selected-pseudocode/` contains selected native dossiers, and
`evidence-bundle.json` retains the complete REA Evidence. Those game-derived
artifacts are not shipped with this repository. The report's hashes and Evidence
IDs identify this inspection, not files guaranteed to exist in another clone.
The final bundle contains 59 records (16,236,871 bytes), SHA-256
`1f70dfcee76f7eb3c6b93db14b423a429c9d27016d47141e23935c0a4b88cada`.
The replay snapshot SHA-256 is
`82399dffa4b6aedad8eb11e939359459bec9c4da5e6512c3ac82b05beac2042f`.
The isolated support build and its 66 focused boundary/parser checks pass;
that is separate from the game's 84-test suite. The native session was exported
and closed, and its owned client/server/Ghidra processes were verified absent.
Local reproduction instructions are in `rea-support/FINDINGS.md`.

The exact GT4 tire formula, controller steering map, runtime calibration and a
human comparison of driving feel remain unverified. The implemented handling
improvement is independently validated against the outcomes above.

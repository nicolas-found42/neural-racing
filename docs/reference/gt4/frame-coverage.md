# Selected native frame coverage

This records the recovered standard four-wheel FR M3, manual mode, asphalt-zero
integration boundary. It does not establish a complete GT4 boot, collision engine,
course geometry, audio renderer or peripheral implementation. Original source
is CORE.ps2.elf SHA-256
`e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`.
REA used Ghidra12.1.4 with `r5900:LE:32:default`. Retained complete dossiers are
in the local audit run `run-01`; its source-address record numbers below identify
the actual shipped instructions, rather than inferred function names.

The custom-course input configuration explicitly sets native car byte `5b5`
bit zero. This disables GT4's original-course centering lookup, consistently for
human and policy inputs. It does not change the recovered physical steering
map, secondary torsion, compliance, TCS or ASC/ABS. Packet device type two is
independent of car byte `546`: original input cases `546=0/1/2` share the same
selected pedal/clutch path. The automatic controller uses `546=0` throughout.

| Native routine | Recovered selected behavior | Treatment/evidence |
| --- | --- | --- |
| `355e10` | Clear `78d` only for drive layout seven. | Inactive M3 layout zero; records392,399. |
| `35ca60`, `35d1d0` | Special-vehicle state machines gated by car-data byte32 equal one. | Inactive M3 byte32 zero; records393,394,397. |
| `350308` | Clear car70c unless car710 equals two. | Scalar state update; record395. |
| `3f7ea8` | Wheel map, feedback768/76c/770, mode-one secondary torsion, force compliance. | `steering_frame::map_wheels`; records350,353,354,366. |
| `36a490` | Body/road-normal dot, wheel contact projection, compression targets514, contact-mask5b9. | Numeric contact stage; records113,125,332,333,400. `36a1d8` computes a mask, not position integration. |
| `345a20` | `3457b0` alignment/vertical geometry, then `3458d0` wheelXY offsets. | Physical geometry; records376,383. Runs before the final, redundant basis rebuild in344530;36b0c8 and36ba88 already refresh the basis. |
| `36cee8` | Vibration intensity from forces/slip; writes conductorca58+car544*16+1. | Peripheral output; records384,388. |
| `353ee8` | Steering-force peripheral packet and car737 countdown. | Peripheral output; record396. Virtual input-device access requires the original peripheral context. |
| `367148` | Wheel feedbackb4/b8/bc; rough-terrain engine-correction increment. | Records386,387. Asphalt group zero contributes no increment. If car6f0 is nonzero,3438c8 bypasses the engine-state write entirely. Non-asphalt groups require the original auxiliary tables; they are outside the selected road boundary. |
| `350080` | For545 zero, gate366678 returns three: update haptic bytes708/709/70a and decay608. | Records381,398. It does **not** take the clear608 branch. Recovered NA3554a0 does not use608 as physical torque; its turbo branch writes that excitation state. |
| `366718` | Clears wheelb4/b8/bc and708..70a only when366678 is not three. | Inactive for selected545 zero; records374,381. |
| `345658`, `34a668`, `34a788` | Camera smoothing in conductorcab8..cae4 and car700/704. | Camera outputs; records375,378,379. |
| `35de20` tail | After contact projection: signed speed6c8, distance/mileage6d4/6d8/6dc, speed/acceleration filters6d0/660/664, free-wheel20 and carrier630/634 refresh. | `contact_finish::finish`; records158,417–420. Source flag46 zero projects free rims onto the contact velocity; rear driven rims retain their force-integrated state. Guard6f0 one bypasses the three optional engine-correction updates. |
| `344040` | Heading/history TAU wrapping; rim-angle24/history910 wrapping; renderpose and outputfilters780/784. | Record385; helpers389–391. No road query. Normal M3 special-vehicle branch is false. |
| `3668f0` | Emits wheel packets through3667a8→395c70/395c98. | External output boundary; records377,382. |
| `3f38d8` | Brake heat wheel+a4 evolves from brake work and cooling. | Record380 and initializer359. No heat read occurs in the recovered selected force/coupling/TCS/ASC/suspension pipeline; this is bounded evidence, not a whole-program claim. |
| `352ea0` | Tire temperature+a8 and wear+ac/+b0 lifecycle. | Records408,411–413. Explicit controlled conductor6d/6c zero disable both;356ef0 returns zero and357418 skips the lifecycle multiplier. Enabling either requires the original race-configuration coefficients. |
| `353218` | Race pit-needed status6c6 and5b5 bit2/3, gated by racevtable+184. | Record409. Race/pit policy is outside the selected custom-course controller boundary; no direct force writes. |
| `35a678` | Random filtered wheelc0/c4 shake. | Record410. No read in the recovered selected force/contact/attitude pipeline; bounded consumer evidence. |

Original344530 orders:36a490→36a758→36b0c8→36ba88→36cee8→353ee8→367148→
350080→366718→345658→345a20→36c9f8(redundant basis rebuild)→35d350(road basis)→
35de20(contact velocities)→344040→3668f0→34a668→34a788. Dossier109 preserves
this order. WorldXY/yaw pose comes from the earlier collision/free-motion stage;
it is an explicit input boundary when executing the suspension subchain.

Initialized profile outputs and source initializer limitations are documented in
[native constructor evidence](native-constructor.md). The verified response-global
capture at843538 is904 bytes, SHA-256
`28759d691b70d91da61673f08bdf0df9116e94ea80aca3177271a6c4ac8f8bb6`,
including the actual347a00→353938 controller initialization. The source-initialized
mode at845bf0 is one. Zero-valued globals are preserved; no replacement tuning
coefficients are synthesized.

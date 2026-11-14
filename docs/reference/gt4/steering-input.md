# Native steering and input boundary

The normal M3 adapter uses the recovered original `3f7ea8` wheel map and
`349f48` → `349418` → `348850` analog-input path. The portable fixture
`gt4_original_steering_frame.json` retains unchanged-opcode Play! outputs for
36 complete wheel-map cases and24 complete analog-input cases. The source
unit test `original_complete_steering_and_analog_input` applies the same
12-ULP float gate and exact flag comparison used by the independent audit.
No cancellation-specific tolerance was added. The port preserves scalar
instruction order with bounded EE arithmetic and original initialized response
coefficients; physical R5900 hardware identity is not claimed.

Wheel mapping includes Ackermann/toe/rear steering, fixed-step feedback at
car768/76c/770, initialized mode-one secondary torsion at wheel18/1c,
previous-frame wheel history atcar8e0, and force-driven compliance atwheelcc/d0.
Mandatory tables620b24/620b40/620b54 come from original366100, and actual
initialized mode one comes from original3f7be0. Input handling includes
speed-dependent steering caps, original response filtering, nonzero-pedal
bias, asymmetric pedal smoothing, reverse arbitration, shift byte, clutch
input, throttle598 and per-wheel brake requests64.

Both human and policy controls use the same native type-two analog packet,
with steering−4096..4096 and pedals0..4096. Car byte546 is a separate native
selector: ordinary values0/1/2 share this selected input path. The automatic
controller uses zero throughout; there is no temporary selector change.

The custom-course adapter explicitly sets original car5b5 bit0 to disable
GT4-course centering assistance. The source otherwise enables a lookup into
original track geometry at zero steering above8.333332m/s. That external
course lookup is not substituted with a fabricated answer. Other supported
entry conditions are manual56a/56b zero, no optional5e9 bits0x18, analog
latch568 zero, and no remembered digital mode569 one. These conditions are
checked. Native physical secondary steering, compliance, TCS and ASC/ABS
remain active.

The904-byte response capture at843538 has SHA256
`28759d691b70d91da61673f08bdf0df9116e94ea80aca3177271a6c4ac8f8bb6`.
It was produced by unmodified347a00 and353938 with the controlled normal-road
conductor and independently matched byte for byte. See
[native constructor evidence](native-constructor.md) and
[frame coverage](frame-coverage.md) for initialization and external-context
limits. Local complete REA dossiers are records168,169,176,350,353,354,366,
371,372,373 of run-01, including assembly and immutable target identity.

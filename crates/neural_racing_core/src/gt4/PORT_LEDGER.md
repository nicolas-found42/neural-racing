# GT4 routine port ledger

These are address-specific ports of routines recovered from the user's supplied
GT4 image. They are not a claim that every physics subsystem or the complete game
has been reconstructed. Engine correction fields and wheel fields retain their
source offsets where physical types or units have not yet been established.

`SOURCE_IDENTITIES.json` joins each port to the exact source function's inclusive
Ghidra body ranges, SHA-256 of those original bytes, REA Evidence ID, recovered
signature and scope. The original compressed `CORE.GT4` SHA-256 is
`85d26aa8430154967b2633eede929286694ac39e99762527edcec365fd642ff9`.
The analyzed ELF SHA-256 is
`e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`.
Its ELF headers were reconstructed; the loaded segment bytes were preserved.
Original source bytes, complete REA replies and emulator artifacts are retained
locally under `.audit/gt4-handling/` and `.audit/gt4-port/`, not shipped with clones.

## Recovered routines

| Address | Port | Scope |
| --- | --- | --- |
| `0x359568` | `curve::Curve::sample` | Endpoint clamp, original binary-search cutoff, exact-knot return, f32 interpolation order |
| `0x359830` | `curve::Curve::sample_linear` | Original tire-table linear-scan variant |
| `0x3598f8` | `curve::PairedCurve::sample` | Two ordinate tables sampled with the first table's abscissas |
| `0x350678` | `engine::EngineCorrection::from_tuning_bytes` | Original nine-byte conversion with exact `0x3a83126e` scale; an explicit object replaces the process-global lazy guard |
| `0x350868` | `engine::EngineCorrection::factor` | Original piecewise correction, bypass flag and multiplication; no invented [0,1] clamp |
| `0x3433e8` / `0x34e9a0` | `CorrectionState::default` / `corrected_curve_value` | Zero state plus bypass flag, and curve-times-correction wrapper |
| `0x351358` / `0x3513f8` | `slip::SlipParameters::gain` / `update_slip` | Both signed-slip modes, original thresholds, flags, divisors and low-speed fade |
| `0x352218` | `tire::fold_slip_angle` | Single angle fold and separate direction sign |
| `0x3518e8` | `tire::split_longitudinal_force` | One wheel's numeric stores; wheel selection/count remain with the caller |
| `0x442490` | `tire::expand_compound_into` | Exact 90-byte compound / 18-byte volume expansion; untouched destination bytes remain unchanged |
| `0x361008` / `0x361220` / `0x3613d8` | `tire::TireTables::from_runtime` | Slip Mu A/B, side, corner-drag, load and terrain tables, source endpoint mutations and peak outputs |
| `0x352408` | `tire::update_contact_force` | One wheel's branches and stores; metadata selection and optional per-car statistic average remain with the caller |
| `0x351fa8` | `drivetrain::update_longitudinal` | Both numeric force phases and original stop-on-crossing branch, with explicit wheel metadata values |
| `0x36d230` | `drivetrain::update_reaction_torque` | One wheel's reaction/brake torque |
| `0x36e4b8` | `drivetrain::update_wheel_rotation` | Numeric axle/differential modes and brake/lock transitions; engine torque and carrier speed must be supplied by the real caller |
| `0x350fb8` / `0x3511a0` | `control::prepare_standard_control` / `prepare_brakes` | Standard FR clutch/control preparation and original wheel brake/flag stores |
| `0x3519b8` / `0x351dc8` | `control::finish_standard_control` / `apply_desired_gear` | Standard FR clutch/RPM followthrough and desired-gear application |
| `0x3535b0` | `conductor::ForceConductor::step` | Complete selected M3 normal-human force conductor, with exact stage ordering |
| `0x357700` | `preparation::prepare` | Normal M3 timer/clock/mass/aero/engine/load/heading preparation with wear disabled |
| `0x3f79a0` / `0x3f71e8` / `0x3f9d00` / `0x3f9870` | `aids` | Selected standard-manual FR metrics, traction, stability and ABS; AI and pit/course limiter policy excluded |
| `0x34ea08` | `engine::drag_torque` | Original linear/quadratic drag query; selected initialized M3 uses linear mode |


The `math` and `body` modules have separate source-specific function comments.
A generic rigid-body update being recovered does not establish it is the normal
GT4 car path; it must not be substituted silently for the car-specific routines.

## Data and supported contracts

The scalar fixtures use real initialized curve descriptors `0x620c80/0x620c90`,
real correction bytes at `0x620da8`, and real slip settings at `0x620d8c`.
Controlled enabled-gain settings explicitly exercise branches inactive in those
original slip settings. They are test inputs, not falsely attributed car tuning.

The 90-byte `TIRECOMPOUND` database row is not the runtime compound layout.
`0x442490` performs the verified conversion. In particular, negative Slip Mu B
uses raw bytes 57–62 and positive Slip Mu B uses raw bytes 81–86; raw bytes 51–56
and 75–80 are not copied by this routine. Post-copy equipment defaults from
`0x3f3b68` belong to a separate caller stage and are not implicitly invented by
`expand_compound_into` or `from_runtime`.

The safe Rust table boundary rejects empty, malformed, non-finite or non-increasing
tables and out-of-layout counts; the original pointer routines do not validate
these inputs. Those checks define the supported port domain, not extra recovered
GT4 behavior. Non-finite scalar queries and arbitrary corrupted game memory are
outside the portable numerical contract. All arithmetic remains `f32`; replacing
intermediates with `f64`, FMA or reordered division can change source results.

## Differential evidence and limits

Tracked original-opcode fixtures use the canonical CORE bytes executed by pinned
upstream Play! R5900 EE JIT commit
`1f27232772e484bfb79364b751b84f41f4c87481`. Fixture metadata records target,
runner identity, execution limits and provenance. This is emulator execution,
not physical PS2 measurement or a full game boot. Primary execution substitutes
no SDK numeric callee and patches no original instruction. A separate negative
control changes the lookup multiply to addition and changes its output.

| Tracked test | Original-execution cases | Comparison |
| --- | --- | --- |
| `gt4_scalars` | 280 | Primary Play outputs bounded in ULP; independent IEEE-mode control exact |
| `gt4_tire_tables` | 210 | All 14 compound rows by 5 volume rows; expansion/mutations byte-exact, table outputs <=12 ULP |
| `gt4_suspension` | 144 | Damper modes and suspension states <=12 ULP |
| `gt4_driveline_heading` | 241 | 99 native rear-driveline cases and 142 velocity-heading threshold/FTZ cases; floats <=12 ULP, all state flags exact |
| `gt4_control` | 200 | Original 3511a0 / 3519b8 calls, standard FR configuration; floats <=12 ULP, brake/gear/clutch/controller flags exact |
| `gt4_tire_forces` | 126 | Original table-load through load/fade/slip/force chain across all 14 compound rows; floats <=12 ULP, flags exact |
| internal `conductor::original_execution_tests` | 60 | Whole original 3535b0, including initialized angle mode1; 115 output words, exact flags and scoped yaw roundoff proof |
| internal `preparation::original_execution_tests` | 36 | Whole original 357700 calls, 28 outputs; clocks/mass/aero/engine/load/fade <=12 ULP, timer/flags exact |
| internal `preparation::composition_tests` | 48 | Original357700→3535b0 composition, including initialized angle mode1; exact flags and scoped yaw proof |
| internal `aids::tests` | 72 | Original metrics/TCS/ASC/ABS leaf calls, <=12 ULP and exact flags |
| internal `steering_frame::tests` | 60 | Whole original analog input and wheel-angle calls, <=12 ULP and exact flags |
| internal `contact_finish::tests` | 24 | Whole original35de20 projection, free rims/carriers, filters and distance, <=12 ULP and exact integer mileage |
| internal `brake_heat::tests` | 150 | Original3f3808→3f38d8 initialization/update,8 outputs per case, <=12 ULP |

The heading vectors include nonzero lateral velocities around the low-speed
threshold and tiny subnormal-producing velocities. Its norm and fade use bounded
per-instruction EE rounding and signed-zero flushing, rather than an empirical
threshold epsilon. `eefloat` uses f64 only for exact f32 operand arithmetic before
rounding, not to integrate higher-precision physics. Nonfinite values and overflow
remain outside that helper's bounded domain.

The common identity ledger also includes root-owned math, chassis, attitude,
calibration and seven steering identities, including canonical procedure identity
for shared-tail angle helpers. Their tracked tests retain their own fixture
provenance. The steering port covers the default digital branch; optional track,
nondefault countersteer and other controller policies remain unsupported.

The user's `raw/patches/gt4-integration` checked API was adapted into `body.rs`
using recovered GT4 math. Root reports 244 original fixture cases and nine atomic
invalid-input tests passing after adaptation; the unchanged standalone patch
passed 22 tests in a private workspace copy. This checked generic kernel does
not establish it is the normal car update path.

The coupled tire fixtures execute the unmodified guest call chain
442490 →361008/361220/3613d8 →357418 →357690 →351850 →3518e8 →352408.
They cover road and dirt force flags, zero/partial/full fade, signed velocities,
slip, terrain, steering correction modes and wheel-kind-two corrections.
The whole-conductor fixtures execute the complete original 3535b0 routine,
including brake preparation, longitudinal forces, reaction/differential torque,
standard engine/clutch followthrough, automatic-controller boundary and chassis.
Twelve extra frames trigger the metric fuel-economy sample through original
354ce8→472ae0→472a78 and check the converted integer exactly. They compare 115 output words per controlled frame, including exact
integer and
packed flag words. They are stored as crate-internal tests so the actual private
integration API is tested without duplicate module compilation.

The M3 carData4352-byte and initialized-global snapshots are independently byte-equal
to actual original constructors/loaders executed on the pinned Play EE/HLE core.
See `docs/reference/gt4/m3-native-constructor.json` for hashes, input identity,
constructor provenance and controlled-runtime limitations. Controlled numeric
frames are not original race-state captures or playability measurements.

`native::NativeCar` preserves source offsets in a fixed 0x1100-byte safe buffer;
`ForceConductor` keeps its four wheel flags allocated once and uses stack arrays
for each step. Native control/gearbox fields preserve their original stores and
call order. The admitted complete conductor is the selected M3 drive mode 0,
gearbox mode 0, human mode 0 path. Other vehicle/controller configurations remain
unsupported. Debug statistics are explicitly disabled and economy unit mode is
explicitly metric zero in the game binding; these are supplied runtime inputs,
not inferred factory defaults. Fuel accounting and economy-sample state stores
are included. The per-frame engine query, contact/suspension preparation and
optional driving aids are earlier caller stages, not silently synthesized by
3535b0. The complete normal 357700 preparation has separate whole-call fixtures,
including the source clock factor, fuel mass, drafting/drag/downforce branches,
engine torque/drag, resistance and effective tire grip/fade. Wear-enabled and
special aero-extension configurations are explicitly rejected by that adapter.

The shared `vehicle.rs` manual/training adapter now uses `runtime.rs` for the
selected M3 on the game's asphalt plane, with 60 Hz physics and 20 Hz policy
commands. Recovered pose/contact/suspension, input, preparation, driving-aid and
force stages are wired into that runtime. Independent test-driver circuit runs
check route feasibility; policy VERSION3 separates the new physics/model format.
These checks do not establish learned-policy competence or complete GT4 race-state
equivalence. Other car configurations, GT4 track-collision systems and unsupported
wear/AI/pit-policy contexts remain outside this selected integration.

The selected driving-aid source identities include the REA owner's 13-record
fragment, and 72 original metrics/TCS/ASC/ABS leaf cases pass the portable tests.
The steering-frame source identities and 60 complete original input/wheel-angle
fixtures are also retained. AI and pit/course-limiter conductor policies remain
outside the selected manual road path. Packet device byte1=2 is distinct from
car546; the admitted runtime consistently uses car546=0 without switching modes.

Native initializer3f7be0 selects845bf0=1. The initial mode0 force checkpoints are
preserved as controlled inputs; twelve additional whole-force frames and twelve
compositions use actual mode1 with nonzero secondary angles.

The whole-force/composition gate normally allows <=12 ULP, with flags exact.
Two exact input buffers have source-derived forward-error bounds for only yaw
fields680/68c/740/744, documented in `docs/reference/gt4/yaw-forward-error.json`.
The fixture gate matches the complete input buffer, angle mode, all four parameter
blob hashes, canonical target hash and original captured output word before using
an offset-specific bound. This handles cancellation of nearly equal yaw moments;
it is not a general absolute tolerance for tire force or chassis outputs.

Whole-frame fixtures instantiate `NativeParameters::fixture_parameters()` from
embedded exact carData/global/steering/response bytes. Production defaults are
not silently substituted for historical reference inputs. Earlier snapshots
with uninitialized compliance descriptors remain local historical artifacts;
the strict production decoder has not been weakened to accept them.

`brake_heat` ports original3f3808 initialization and3f38d8 wheel+a4 stores without
inventing a force fade multiplier. No wheel+a4 consumer was identified in the
bounded force/TCS/ASC graph. Its 150 original initialization/update cases pass the portable fixture test;
the runtime preserves the heat state after forces.


Attitude scalar arithmetic follows the retained36af30/36b0c8/36c9f8 assembly.
These functions use ordinary COP1 MUL/ADD/SUB/DIV, not ACC instructions. The
mode0 vertical update computes acceleration×dt before multiplying each up-axis
component, matching36b74c. The existing244-case chassis/math/basis/attitude test
passes after this precision correction; extended composed post-step validation
is being rerun independently.


`contact_finish` recovers the physical35de20 tail after contact projection:
35dd00/343518 distance and integer/fractional mileage,35dc08 speed and lateral
acceleration filters, and35d990 free-wheel/carrier correction. Original wheel
flag46 zero snaps rim velocity20 to the steering-projected contact velocity,
then eligible carriers average the corrected rims before multiplying inverse
radius. This update runs after35d920 and before344040 rim-angle history. The
selected reset's6f0 one bypasses the three optional engine-correction updates;
zero is rejected. All24 original whole35de20 cases pass the portable test
`original_full_contact_projection_and_finish` under the unchanged12-ULP/exact
integer gate. References execute original GT4 instructions on the existing
Play! EE executor; they are not calculated by the Rust port.

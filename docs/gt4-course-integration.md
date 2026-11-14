# GT4 course integration

The supplied NTSC-U 1.01 disc provides original collision-road geometry, native
track progress, elevation, numeric surfaces and course identities. The importer
does not resize roads to fit map illustrations. See the [resource provenance](reference/gt4/course-resources.md),
[course and event bindings](reference/gt4/course-catalog.md), and
[road-query implementation](reference/gt4/road-contact.md).

## Source coverage

The volume contains 110 top-level course resources and 110 companion `.ad`
resources. All were extracted and decoded. Joining the original COURSE table
and RACE events distinguishes 83 closed routes, six open routes, and 21 areas
without lap progression. Six of the latter are Gymkhana resources. The remaining
15 nonlap scene resources retain their catalog identities; a filename alone is
not evidence for an activity or a driveable race.

Forward, reverse, wet and alternate layouts retain distinct original resource
IDs. Five acceleration/slalom variants select actual original event sections,
not the full collision mesh. Las Vegas uses its native open-course progression.
Manual practice areas have no lap reward, checkpoint reward, finish condition or
race timeout. Selecting one preserves and pauses the learner. They cannot be
used as lap-training courses.

No verified original GT4 resource matches this simulator's Monza, Silverstone or
Red Bull Ring layouts. Those geographic courses remain available. An exact
verified alias would select GT4 values first; fuzzy names do not silently replace
one circuit with another.

## Native evidence and construction choices

REA inspected the original R5900 executable and its course loader, mesh queries,
progress projection, event section selection, condition fields and contact-cache
branches. The canonical analysis ELF SHA-256 is
`e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`.
The registered provider rejected the PS2 processor, so the investigation reused
the isolated local REA/Ghidra setup with the PS2 extension; the user's global
registration and original executable were unchanged.

The independent reference executes unchanged original functions and callees on
upstream Play EE commit `1f27232772e484bfb79364b751b84f41f4c87481`.
This is controlled emulator execution, not a physical PS2 measurement or a full
GT4 boot. The native road-query harness does not substitute host geometry
callbacks for the original BSP, cluster traversal, triangle hit or quad projection.

Source checks include 21,784 authored-guide queries over 88 course resources,
12,147 checkpoint/deck queries over all 110, and a frozen set of 189,816 runtime
station queries over 93 resources, extended by 2,063 queries for the last two
resources (191,879 queries across all 95 driving resources). All station ground
queries succeeded. Seventeen
native progress results were fallback zero with no valid cluster; they must not
be treated as measured progress knots. Of 11,938 distinct source checkpoints,
none differed from original stored progress by more than 1 cm; the maximum was
1.954 mm. The runtime station comparison measured interpolation errors before
rebinding, not a claim that the initial interpolation was original GT4 behavior.

The original mesh records retain their coordinates, triangle winding, numeric
material/flag bytes and vertex attributes. The runtime driving guide and width
cross-sections are derived inputs. Checkpoint gates and minimap strips are not
road edges, and checkpoint Y is not a reliable road height. Desired centre
clearance is a simulator route-construction choice, never a recovered road-width
constant. Original scene textures, buildings, barriers, race opponents, event
medals and camera activities are outside this road integration.

## Regressions found during integration

An early uniformly indexed minimap guide assigned large progress changes to tiny
physical movements on Autumn Ring. Original native progress queries replaced
that assumption. An open Fisco section had a one-float-ULP endpoint discrepancy;
wrapping it generated a microscopic backward finish tangent. Open endpoint
sampling now clamps to actual stations, and a real environment finish-plane test
checks route completion with zero lap credit.

The road adapter initially ended an attempt when any wheel query missed. The
original partial-contact branch preserves that wheel's prior height, material
and normal, suppresses its roughness, and lets valid neighbors continue. It also
prepares queries using wheel geometry and body up, rather than body position
alone. The recovered per-frame search casts down from five metres above each
prepared query and selects the highest reachable positive-winding face.
Nearest-height selection gives different results at stacked roads. Unit tests
include original Monte Carlo and Suzuka backface geometry and native query
receipts; a controlled nearest-height negative control fails the new stacked-road
regression while the production implementation passes.

The first frozen 95-entry driving audit completed 85 race routes and six manual
area launches. Four Sarthe variants failed near native progress 13,503 m where a
derived guide cut a chicane and left only one metre of clearance on its right.
The connected physical road component there measures about 23.25 m across. This is a
guide-reconstruction failure, not evidence that the original course or car is
unplayable; it remains separate from the finite-launch check that passed all 95.

## Validation boundaries

Finite launches and independent-driver traversals establish physical feasibility;
they are not human playtest results or proof of a competent learned policy. The
new learning cadence, curriculum and reward checks are recorded separately in
[learning validation](learning-validation.md). Old policy files are rejected by
format version four. Existing running sessions do not acquire new geometry or
physics code until launched from an updated executable.

The all-course report is bound to source and asset hashes using
`scripts/gt4/course_probe_receipt.py`. Local `.audit/gt4-tracks/` artifacts retain
failed intermediate audits as well as later results; their dates and input
identities distinguish them. The final acceptance receipt records the exact
admitted set and current checks. Diagnostic source fingerprints also include
`.mesh` and `.stations`, so an asset-only change changes a new run's identity.

The representative 5 Hz binary-keyboard test passes the three selected road/wet
routes. Its dirt probe fails at both 15 and 12 m/s. The last 40 slower-run decisions
show 24 full-lock commands, 17 full-throttle commands, no braking and a growing
heading/slip error before boundary departure. All 39 preceding body-centre mesh
queries hit. That trace supports test-controller overshoot, while the smooth
12 m/s driver completes the route; neither result replaces a human dirt playtest.

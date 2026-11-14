# GT4 course identities and route selection

The catalog joins all 110 extracted top-level `crs` resources to the original
110-row COURSE table by exact resource label. An index is an in-process handle;
the persisted/CLI identity is `gt4-<resource-id>`. Forward, reverse and alternate
layouts retain separate source identities. Existing geographic circuits remain
unless an explicitly evidenced exact alias gives a GT4 resource precedence.

`COURSE_NAME_american` provides only 23 localized names. The other entries display
the exact resource ID. The pinned PDTools `Course.cs` field called `NameEng` is
**not** a GT4 display name: actual values resolve to attribute masks such as
`0ff0` and `1ff0`. The importer deliberately avoids that mapping.

COURSE byte `0x10` is dry/wet/dirt/snow for values 0/1/2/3. The matching original
`MCourseData::GetType` at `0x1b3030` calls the COURSE reader `0x447f48` (table
`0x22`) and selects literal strings at `0x693280`, `0x693288`, `0x693290` and
`0x693298`. These strings were read from the unchanged canonical CORE ELF.

COURSE byte `0x12` is copied to conductor `0xcbde` by `0x34c7e0`. The retained
native source uses value 1 for endpoint-controlled progression (`0x34c5e0` and
`DynamicsConductor::ProcessGoalLine`) and skips lap tracking for value 2
(`0x359008`). Catalog values distinguish 83 closed routes, six open routes and
21 non-lap areas. Areas retain their identity without fabricated closed races. Geometry-validated
areas support manual practice with no lap or checkpoint rewards, race finish or
time limit. Selecting one pauses and preserves the learner; learning/watching
requires a race route. Other area resources remain visible and unavailable until
their driving geometry or activity is established.

## Dedicated sprint sections

A resource can contain a full circuit mesh while its original event selects a
short section. Original RACE byte `0x89` is signed. `0x35e108` copies it, bounded
below by -1, to packed settings `0x21`; `0x34d418` places those settings at
conductor `0xf818`. `SetupGlobalStatusPre` reads `0xf839`, validates the section
index and calls `RaceCourse::setSectionNumber`. Native getters `0x395528` and
`0x395588` read section start/goal values when that index is nonnegative.

The importer examines every original RACE event referring to each course ID.
If they all select the same nonnegative section, that source-bound section is
the catalog route. This binds `fisco_slalom` and all four dedicated acceleration
test resources to section 0. Ordinary Suzuka events include both full-course
and several section selections, so the importer preserves its full course.
There is no filename-based distance or topology rule.

The 100 m resource's mesh has TrackV 10342.2626953125 m; its selected section
runs from 10242.2578125 to 10342.2626953125 m. Treating the whole mesh as its race
would incorrectly turn the sprint into a 10.3 km lap.

## Reproduction and admission

Run `scripts/gt4/export_course_catalog.py --help` for the source paths and
options. `--native-classification` applies the recovered field/section data flow.
`--geometry-manifest` admits only independently validated published geometry IDs.
An explicit `--bindings` manifest can carry additional evidenced identities or
route selection. Availability is separate from resource presence and metadata.

The local `.audit/gt4-tracks/decoded/course-metadata.json` records every raw COURSE
row, its RACE event references, selected section values, source hashes and
geometry-manifest hash. Generated `src/circuit_data/gt4_catalog.rs` is runtime
metadata; `gt4_files.rs` binds the separate validated geometry assets. Missing
geometry remains visible and unavailable rather than receiving placeholder roads.

This proves a source-derived catalog and selected route inputs. It does not
establish GT4's complete event controller, every license rule, original scenery,
or every free-area activity.

## Legacy precedence audit

The complete source join contains no verified identity alias for the existing
Monza GP, current Silverstone GP or Red Bull Ring GP layouts. All three geographic
maps therefore remain available. `legacy-course-alias-audit.json` binds that
bounded finding to the original COURSE DBT/IDI hashes. No similarly named GT4
resource is substituted: High Speed Ring and Nürburgring are separate identities,
not aliases for Red Bull Ring. Any future overlap must carry an explicit alias
with evidence; GT4 wins that exact lookup and replaces that legacy menu entry.

The chooser displays the source ID beside the available label, supports text
search and bounded scrolling, and identifies circuit/open/manual-practice mode
and native dry/wet/dirt/snow condition. Open routes count completed routes and
never promote to five-lap stints. Area selection preserves the paused learner
and permits manual practice only. Unavailable resources stay visible without
placeholder racing geometry.

## Independent driving receipts

`tests/gt4_courses.rs` checks all available routes against original RNW TrackV
or selected section distances, finite observations, four native loaded wheels
and a moderate-pedal launch. Its ignored release probe attempts a traversal of
each race route using the independent pursuit test driver; areas receive manual
launch checks without fabricated finishes. A separate ignored probe uses the
same 200 ms binary keyboard-command controller as the legacy circuit tests on
representative imported dry/wet/grade-separated/dirt routes. These measure test-driver
feasibility; failed drives remain recorded failures. They do not establish human
or learned-policy competence.

After the importer freezes a coherent publication, use
`scripts/gt4/course_probe_receipt.py RECEIPT.json` to record SHA-256 identities of
all core sources, native parameter bytes, road assets, catalog/asset bindings,
probe/driver sources and original distance fixtures, plus the Rust compiler.
Area distance fixtures come from published native-V station extents and are
explicitly navigation-only; they do not create race goals from header fields.
Set `GT4_COURSE_INPUT_RECEIPT` to its absolute path when running the ignored
release tests. The report embeds the receipt and rejects compiled catalog or
asset-binding text that differs from it. Run the receipt script with `--verify`
before and after execution to detect any input change during the audit. The
ordinary launch test remains fast; no screenshots are overwritten by these probes.

## Final driving checkpoint

The frozen 95-resource publication passed on 2026-11-05 UTC: 89 complete race
traversals with the independent 12 m/s pursuit driver and six finite manual-area
launches, with zero failures. The ordinary test also checks that all 95 resources
remain available, rather than allowing a failing route to disappear from the
catalog. All 245 source/asset identities verified before and after execution.

- Geometry manifest SHA-256: `f44e4179c3a88210bdc15965abe3b7ef1c7d22a197f2f776de565381702bf2e1`.
- Input receipt SHA-256: `f700c8d0aa35f26d0544cfd62222d091b116caead777bc28dff1a10fcb199bf8`.
- Driving report SHA-256: `918cc8ff8773147226a7643e08754e62284dbe61664cb35c15b5637ae28fa957`.

The local report is `.audit/gt4-tracks/final-all-course-driving.json`; its embedded
receipt records the actual compiled inputs. The summary and before/after check
are retained beside it. Earlier failed checkpoints remain separate. Four late
Sarthe failures were repaired by moving the derived guide inside the unchanged
original road edges; no vehicle-physics source changed between the failed and
passing 95-resource checkpoints.

The separate 200 ms binary keyboard controller completed High Speed Ring, wet
Tsukuba and Suzuka at a 15 m/s target. It failed Yosemite Dirt Short at both
15 m/s and 12 m/s targets. Those failures remain recorded: the last-two-second
trace shows large yaw/slip, delayed full-lock pulses and no braking before the
car departs the road. Continuous 12 m/s pursuit completes the same route. This
is a test-controller limitation, not a claim that all imported routes have been
validated by human players or a learned policy.

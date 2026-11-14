# GT4 course resources

The source is the user's GT4 NTSC-U 1.01 disc, not a reconstruction from a map image. The local extraction manifest records the exact archive offsets, compressed and decompressed SHA-256 hashes for all 110 `crs/<id>` resources and their 110 `.ad` companions. The original geometry contains 886,303 vertices and 1,630,614 triangles in total.

The retained manifest is `.audit/gt4-tracks/extracted/manifest.json`; its source volume-index SHA-256 is `561476eff4047b34f40758432eb39160b894ad9d1daed4fe35d796191a092d78`. Local audit files are retained evidence, not files guaranteed in another checkout.

## Original geometry

The course container's pointer at `+0xdc` locates `RNW4`. Its unsigned length at `+8` bounds the resource. Native track progress length, start and goal are the original floats at `+0x14`, `+0x18`, `+0x1c`. Vertex and triangle arrays use the pointers at `+0x74` and `+0x78`.

A vertex is 16 bytes: three float coordinates, a signed 16-bit index, and two attribute bytes. A triangle is 12 bytes: three unsigned 16-bit vertex indices, two material/flag bytes and a final unchanged word. Material ID is byte 6 masked with 31. Native collision code interpolates vertex byte 14 divided by 255 and byte 15 masked with 127 divided by 127. These attributes are retained; no new friction interpretation is assigned.

RNW coordinates are X, Y elevation, Z. The application conversion is `(x, y, z_elevation) = (RNW.X, -RNW.Z, RNW.Y)`. Mesh coordinates, triangle winding, materials, flags and attributes are preserved; no map-size fitting or geometric smoothing is applied.

The runtime mesh envelope is little-endian `u32 vertex_count, u32 triangle_count`, followed by the unchanged 16-byte vertex and 12-byte triangle records. The generated source includes identify each exact resource by ID.

## Checkpoints and progress

The checkpoint count at `RNW+0x3c` describes 40-byte records: three XYZ vectors followed by native TrackV. Their left and right vectors are validation gates, **not physical road edges**. Their middle Y values also cannot be treated as physical ground height: the source Tahiti checkpoint heights lie around 72–127 m while its collision mesh lies around −2–54 m. The original relocation function changes pointers, not these coordinates.

The GTCM minimap contains an authored two-dimensional guide. Its chunk index is not native RNW progress: Autumn's first chunks correspond to native TrackV around 2883 m. Uniformly spreading chunks over the circuit length caused a reproducible progress discontinuity and is rejected.

Progress bindings are measured by executing unchanged original GT4 code: `0x3e5818` relocates an isolated original RNW copy, `0x395760` performs its native road query, and `0x3e6858` uses the original BSP, cluster-limited checkpoint lookup and `0x3e6610` quad projection. This runs on the pinned Play EE execution harness; the game query functions have no host geometry stubs.

The immutable local results are:

- `.audit/gt4-tracks/native-v/native-gtcm-progress-results.json`: 21,784 actual mesh-height queries across 88 resources, SHA-256 `ac0523fba9021004fa87029f6ef0a4fa9a5cd0322639a4b84ab4dfa2e4e8fa92`.
- `.audit/gt4-tracks/native-v/native-rnw-checkpoint-progress-results.json`: 12,147 queries across all 110 resources, SHA-256 `e0ab47367d1ed4e9717b59bfa94ed8631d05112cd0cf8e971f49d910c837e445`.

For 11,938 distinct checkpoints, selecting the actual mesh deck by native V reproduces the stored checkpoint progress within 1.954 mm; none differs by more than 1 cm. Physical heights come from original road queries, never a guessed checkpoint-height translation. A successful ground query alone does not prove valid progress: five GTCM queries return native fallback zero with cluster zero. Those observations remain recorded and are excluded from authoritative progress knots.

## Derived driving guide

The collision mesh is original. The runtime driving guide and cross-section widths are derived geometry. Authored GTCM guide points, or original checkpoint quads for dedicated open sections, supply route leads. Their progress and mesh deck are bound to native query results. Cross-sections intersect the original triangles; widths are measured from connected physical intervals, never from checkpoint gate widths or the minimap strip width. The source native material classifier distinguishes IDs 2, 3, 4 and 6 from class zero; the corridor derivation records this numeric classification without inventing surface names.

A desired centre clearance is a route-construction choice and is recorded separately from source road dimensions. The extractor preserves original source positions and hashes so this derived guide can be replaced without altering the original mesh.

Open sections are bounded by actual RNW/RACE section bindings. They are not wrapped into loops. In particular, the source test-course sprint variants cover only their selected section; their full RNW length is not the sprint distance. Las Vegas uses its actual native grid lane rather than treating the middle checkpoint divider as a drivable lane.

Admission checks include finite, monotonic progress; mesh coverage along the guide; and lateral samples at ±0.95 m every at most 1 m of physical guide distance. This is a geometric corridor check, not a complete car trajectory proof. Native four-wheel contact and driving feasibility are checked separately in the application. Photo/area resources and unresolved paths remain explicit catalog entries rather than fabricated closed circuits.

## Reproduction and source identities

`scripts/gt4/decode_courses.py EXTRACTED_CRS OUTPUT` performs bounded original RNW/ADLN decoding. The local extraction and native-query recipes, raw responses and per-resource diagnostics are in `.audit/gt4-tracks/`. `published-geometry.json` records the exact runtime bytes and their acceptance diagnostics.

Format leads were checked against PDTools commit `2a9637d8c36100e4c211d73cd8140a9d690578d2`; native behavior was inspected through REA on unchanged CORE SHA-256 `e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`. REA evidence `ev_735e39a10b806bba675f3e12b6726edcf8e60adc90770dcd64ac58aa03eb19c8` covers native checkpoint progress, and `ev_836cdd3a4daf546a2484ae22d7929e8b0c415eee029d60c1e924d679eb238a08` covers original collision mesh/material/attribute decoding. The execution receipts identify the exact emulator, runner, inputs and RNW hashes.

## Authored-route and edge-frame correction

The later visual audit found that valid contacts and full driving traversals did
not prove the old ribbon's visual scale. Independently recentering coarse guides
and then rebuilding edges with a different tangent produced folds and merged
aprons. Suzuka's old start cross-section was 35.77675 m; original directed boundary
clipping in the authored route frame measures 15.97407 m there. This is a physical
corridor measurement, not a claim that a collision wall is painted road edge.

Recovered original ADLN route 6 supplies line/arc geometry for 86 race resources.
Nine explicit RNW/manual-navigation baselines cover six practice Areas, Fisco
Slalom, the legacy 100 m sprint, and Las Vegas. The three legacy 40-byte source
formats and four nonascending type-6 arrays are not passed to the ordinary
64-byte ascending sampler. Returning from a minimal-context native initializer
was insufficient for the legacy records; their diagnostic output was quarantined.

Original source identities:

| Routine | Source-backed behavior |
| --- | --- |
| `0x46a510` | Original ADLN initialization, including native ground/progress probes |
| `0x46b248` | Original coordinate/pointer relocation |
| `0x46b480` | Native-V line/circular-arc sampler |
| `0x46bb48` / `0x46b108` | Driving-route fallback / nearest selection among 6/2/1 |
| `0x3e6a58` | Per-triangle boundary-span lookup and signed-counter loop closure |
| `0x45d550` | Directed physical segment/corner collision, independently run on EE |
| `0x3e5818→0x395760→0x3e6858` | Unchanged relocation, contact query, native progress binding |

Boundary flags 0/1 and ordered loops are preserved without assigning universal
pit/paint semantics. Original callbacks are two-dimensional; per-triangle and
height-connected-deck selection prevents globally clipping an unrelated overpass.
Upward ground-face selection follows the original contact query. Native surface
classes select physical ground components, not visual racing-strip categories.

The corrected payloads preserve explicit world-space edge points. Navigation may
move minimally to accommodate the selected M3's 1.914 m mirror width and a recorded
0.10 m per-side margin; this changes neither source edges nor widths. Genuine
wide aprons must be drawn from original source faces/textures, not a potentially
folded corridor fill. Native query outputs take precedence over host-derived
progress, hit height, and material.

The local final-source proof covers 191,879 candidate contacts plus 202 explicitly
changed positions, all successful. Twelve original cluster-zero progress fallbacks
have an explicit neighbor-interpolation ledger. The final cyclic-source endpoint
normalization leaves 191,792 aligned stations/edge pairs across all 95 resources.
The public generator and binder reproduced all 190 installed payloads byte-for-byte.
The 289 retained original sampler outputs check host geometry within 0.5 mm;
actual-car acceptance remains a separate dated run in the verification ledger.
Local receipts are under `.audit/gt4-road-boundaries/`; they are historical evidence,
not files guaranteed to exist in every clone. Reproduction tools and frozen source
fixtures are documented in [the extraction README](../../../scripts/gt4/README.md).

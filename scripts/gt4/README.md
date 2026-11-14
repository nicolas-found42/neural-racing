# GT4 source-data extraction

The extraction commands below read the user's own extracted `GT4.VOL`; they
do not execute original game code or download assets. The separately documented
native oracle does execute bounded original functions on Play! EE. It supplies
the initialized source inputs used by the authored-route tools.
Keep volume files and extracted database contents under gitignored `.audit/`.

From the repository root, using a new output directory:

```sh
python3 scripts/gt4/rofs_extract.py .audit/gt4-port/data/GT4.VOL \
  --index .audit/gt4-port/recheck-index.json \
  --out .audit/gt4-port/recheck-extracted --prefix specdb/GT4_US2560/
python3 scripts/gt4/decode_specdb.py \
  .audit/gt4-port/recheck-extracted/specdb/GT4_US2560/TIRECOMPOUND \
  .audit/gt4-port/recheck-compounds.json
```

Decode each DBT/IDI pair into a directory before using `select_car.py`:

```sh
python3 scripts/gt4/select_car.py .audit/gt4-port/data/decoded \
  m3_gtr_alms_01 .audit/gt4-port/recheck-m3-source.json
```

The RoFS reader supports version 3.1, 2,048-byte pages and the observed encrypted
index framing. Independently framed index pages can omit DEFLATE BFINAL; exact
page output length remains mandatory. Selected files have a 64 MiB per-file
bound, checked paths and exclusive creation. Extraction rejects symlink paths
and preserves packed ranges plus packed/output SHA-256 values. It does not
support concurrent adversarial changes to the destination filesystem.

The database decoder reads GTDB/GTID headers, the original Huffman codebook,
template/delta rows and ID/label relationships. Table schemas, physical units,
equipment defaults and auto-gearing are separate native-code recovery steps.
`select_car.py` follows actual table IDs and preserves all-ones NULL keys; it
does not infer equipment slot names from newer schema versions.

Validation on the supplied NTSC-U v1.01 image reproduced all 18,562 RoFS entries
and the retained 14-row compound table exactly. The selected M3 source joins
contain 26 default-part slots. A symlink escape probe was rejected before writes.
These source-data checks do not validate a complete car simulation.

Original authored routes can be initialized by the original EE instructions,
using the runner built from `native_oracle/` and the pinned Play! checkout:

```sh
python3 scripts/gt4/initialize_native_adln.py \
  --audit-root .audit/gt4-tracks --runner .audit/gt4-tracks/native-oracle-build/gt4-native-ee-runner \
  --elf .audit/gt4-handling/extracted/CORE.ps2.elf \
  --out .audit/gt4-native-adln --all-available
```

The explicit host cycle budget defaults to 100 million for these initialization
calls; ordinary runner invocations retain their one-million default. The runner
accepts `--cycle-budget=100000000` without changing guest instructions or callees.
The controlled RaceCourse owner binds CourseData at `+4` and the actual ADLN
resource at `+0x9c`. Each receipt pins source inputs, CORE, runner, initialized
bytes and the original `0x46bb48` default-route result. Legacy headers are
reported separately; successful execution alone does not establish usable route
geometry. Types 1/2/6 are driving-route alternatives, not painted road edges.

Static course artwork uses the original CRS MDLS/VIF/Tex1 data. This workflow
requires Pillow in addition to Python's standard library:

```sh
python3 scripts/gt4/export_visual_ground.py --all-available --jobs 2 \
  --audit-root .audit/gt4-tracks \
  --out crates/neural_racing_core/src/circuit_data/gt4_visual \
  --receipts .audit/gt4-visual-export
```

The exporter retains literal model matrices, packed VIF position/UV deltas,
packet texture events, native vertex colors, TEX0 and CLAMP registers. Unknown
dynamic or instance commands and unsupported texture formats remain explicit
omissions in per-course receipts. Highest LOD and texture group zero are used.
The display filter keeps triangles whose absolute up-normal component is at
least 0.7 and whose centroid is within 0.4 metres of an original front-facing
RNW ground triangle. This selects a display layer; it does not define physical
contact or classify painted racing boundaries. Physics coordinates are exactly
original `[X,-Z,Y]` metres, without route fitting or width scaling.

Each `.gtv.gz` contains little-endian indexed GTV1. Its 32-byte header is magic
`GTV1`, vertex/triangle/texture counts, strides `24/28/40`, and reserved zero.
Vertices contain five f32 values `(X,Y,height,U,V)` and four native RGBA bytes
(128 is unity). Triangles contain seven u32 values: three vertex indices,
texture index, source model, source shape and source packet file offset.
Texture records contain u32 width/height, u64 CLAMP/TEX0, then u32 absolute RGBA
payload offset, payload length, source Tex1 file offset and source texture
index. Decoded texture RGB remains native; GS palette alpha 128 maps to 255.
Texture row zero and UV V zero retain the source orientation. Gzip uses a zero
timestamp for deterministic output.

GS address permutations follow the pinned upstream Play! commit
`1f27232772e484bfb79364b751b84f41f4c87481`; its license is retained in
`Play-GS-LICENSE.txt`. These tools use original game bytes and do not depend on
third-party model-exporter repositories.

Layout references:

* [GT4FS](https://github.com/Razer2015/GT4FS/tree/b938f71fd939e0ea44c38839a50ffb66cfd2067a)
* [PDTools SpecDB](https://github.com/Nenkai/PDTools/tree/2a9637d8c36100e4c211d73cd8140a9d690578d2/PDTools.SpecDB)

Native REA Evidence and original-opcode execution remain the semantic authority
for the physics port; the external schema implementations are layout references.

## Source-authored routes and physical corridor edges

`adln_route.py` reads **original-initialized ADLN**, not raw attack-line records.
The original sampler uses line/circular-arc segments parameterized by native
TrackV. Route types 1/2/6 are alternative driving routes; they are not road edges.
Type 6 uses the native preference order 6→1→2. The host float64 construction is
checked against 289 original-EE outputs within 0.5 mm, including controlled
positive/negative arcs and wrapping. It is not a bit-exact EE port.

`source_corridors.cpp` intersects original, upward-facing RNW ground triangles
in the authored tangent frame. It selects the source route's dominant native
surface class and gathers collision-boundary spans from the same connected
height deck. Original directed segment blockers clip the physical corridor.
This does **not** classify paint, racing asphalt, pits, or runoff. Broad source
aprons remain broad. Render original source faces rather than filling these
cross-sections as an assumed continuous racing ribbon.

The navigation reference moves only enough to fit the selected M3's 1.914 m
mirror width plus an explicit 0.10 m margin on each side. This does not inflate,
smooth, or clamp source road widths. Edges retain their measured world positions;
reconstructing them from the displaced guide's later tangent was the old bug.

Initialize the 92 supported ADLN resources with the original code first (legacy
formats are reported as unsupported rather than relabeled):

```sh
python3 scripts/gt4/initialize_native_adln.py --audit-root .audit/gt4-tracks \
  --runner /path/to/gt4-native-ee-runner \
  --elf .audit/gt4-handling/extracted/CORE.ps2.elf \
  --out .audit/gt4-road-boundaries/adln --all-available
```

Its explicit host cycle budget is 100 million; the game bytes and callees stay
unchanged. The default generic runner budget remains one million.

Build candidate payloads without changing runtime files:

```sh
c++ -std=c++17 -O3 scripts/gt4/source_corridors.cpp -o /tmp/gt4-source-corridors
python3 scripts/gt4/build_physical_corridors.py \
  --audit-root .audit/gt4-tracks \
  --initialized-adln .audit/gt4-road-boundaries/adln \
  --baseline scripts/gt4/fixtures/rnw_navigation \
  --boundaries crates/neural_racing_core/src/circuit_data/gt4 \
  --retain-navigation lasvegas \
  --corridor-executable /tmp/gt4-source-corridors --out /tmp/gt4-candidates
```

The nine frozen navigation baselines have their own hashes/provenance. They
cover six manual practice Areas, the legacy 100 m sprint, nonascending Fisco
Slalom, and Las Vegas (whose ADLN wrap failed original native-progress ordering
at the start). Their original whole-course collision meshes remain unchanged.
Legacy 40-byte records are not read as initialized 64-byte ADLN. A returning
initializer alone does not prove those records are usable.

Verify every candidate XYZ using the [original native oracle](native_oracle/README.md).
Then bind the returned native TrackV, height, and material to the **same** candidate
positions, keeping station/edge pairs aligned:

```sh
python3 scripts/gt4/bind_physical_corridors.py \
  --candidates /tmp/gt4-candidates --out /tmp/gt4-native-bound \
  --metadata .audit/gt4-tracks/decoded/course-metadata.json \
  --mesh-dir .audit/gt4-tracks/decoded/meshes \
  --runway-dir .audit/gt4-tracks/decoded/runways \
  --results /path/to/original-results.json
python3 -m unittest discover -s scripts/gt4/tests -v
```

A later `--results` file can override an explicitly re-queried changed subset.
Original cluster-zero/getV fallbacks are recorded and interpolated between
proven neighbors by physical distance; they are never silently interpreted as a
start line. Full cyclic source loops normalize native V and drop paired duplicate
endpoints even when a native race section is publicly exposed as a sprint.
Partial open navigation remains partial. Neither tool installs assets or declares
a candidate playable; construction/contact and actual-car acceptance are separate.

Payloads: `.stations` is `u32 count` plus `<6fII>` records (V, X, physics Y,
height, left/right clearances, source index, material). `.edges` is `u32 count`
plus `<4f>` (left X/Y, right X/Y), aligned row-for-row. Core left is positive
`[-tangent.y,+tangent.x]`. `.boundaries` is `u32 vertex_count,index_count`, followed
by unchanged 16-byte original vertices and u16 lookup indices. Its lookup ranges
come from each triangle's packed final word; lookup indices are segment starts,
not an ordered line strip. All binary tools currently require little-endian hosts.

# Queried road inputs

The vehicle adapter accepts external world queries in metres, with X/Y horizontal
and Z up. The query includes reference height: a two-dimensional nearest-triangle
choice cannot distinguish an overpass from the road below it. The vehicle bridge
rotates local positions into absolute track coordinates and rotates normals back.
Original `0x344458` calls the four-query wrapper `0x3957f0`, which clears input
hints and performs fresh downward searches. Each ray starts five metres above
the prepared query height and extends to the mesh's lower bound. The adapter
selects the highest positive-winding face below that start; choosing the closest
height would give different results on stacked roads. Its vertical origin comes from the selected centerline height at reset, preserving
native body clearance without placing an elevated course at world zero.

The result supplies height, unit normal, original surface index, flags and two interpolated attributes. Height
populates wheel `+8c`; cached course-normal storage is `{nx,nz,-ny}` at car
`+7d4/+7d8/+7dc`, repeated every `0x34`. Original `0x3f4e40` remaps wet-flag surfaces: raw 0/7 to20, 1/10/11/12 to21, and9 to22. The resulting surface index populates wheel `+89` and
feeds the existing recovered tire terrain and rolling-resistance tables. There is
no additional empirical grip multiplier. Indices outside the 24 recovered terrain
slots report failure. A missing single-wheel query follows the original cached
miss behavior described below; it does not substitute asphalt or end the race.

Original `0x3697a8` prepares every query using `0x3696c8` and `0x369760`:
body position plus the stored wheel geometry plus body-up times the original
suspension/radius offset. This applies on all three axes, including query height;
the query is not simply the car's centre height. The cached input Y at `+4`
(car `+7bc`) is populated too. Twenty-four original instruction fixtures compare
all twelve prepared wheel coordinates bit for bit.

Original `0x344458` replaces contact normals whose vertical component is below
0.5 with the upright normal. This guard is separate from the front-face winding
filter used by the collision query, and applies to retained cached normals too.

On a miss, `0x395760`/`0x3957f0` mark cached `+30` as one without overwriting
prior hit data. Original `0x369888` then clears only wheel bump `+90`, retaining
wheel height `+8c`, surface `+89` and cached normal. The adapter follows this
behavior for partial wheel misses. Twenty-one original contact-prefix fixtures
verify retained heights/materials and valid-neighbour roughness bit for bit.
The exact Yosemite point that exposed premature race termination also misses
under the unchanged native query, so replacing it with asphalt would conceal the
source behavior. Losing all four external road queries remains an explicit
adapter termination boundary; unsupported material IDs still fail explicitly.

Original `0x35d350`, model zero, derives road pitch and bank from the four wheel
heights, body attitude and original wheelbase/track parameters, then constructs
the basis through `0x36c9f8`. Original `0x36a490` projects each cached contact
normal onto body up. These are separate inputs: averaging the normals is not a
substitute for the four-height construction. Normal forces and attitude receive
the individual queried normals. The penetration pass retains the original road
tangent velocity floor from `0x36ba88`.

The saved original C/assembly records are `308-contact-aids-frame-0x35d350`,
`113-load-pipeline-0x36a490`, `125-suspension-0x36a080`, and
`116-load-pipeline-0x36ba88`, under the local REA evidence directory. The tracked
`gt4_original_road_basis.json` contains 27 controlled combinations of nonflat
heights and body pitch/roll. All 12 selected height/angle/basis outputs match
unchanged original instructions executed on upstream Play EE exactly. This is
emulator evidence, not a physical PS2 hardware claim.

Original `0x369888` adds material roughness to wheel `+90`, using the captured
amplitude/frequency/shape arrays and `0x369640` SDK sine/cosine kernel. It uses
original course-global X/Z, so resetting the vehicle origin cannot move bumps.
Raw-surface `0x3f4ea0` classification controls the original mixed-wheel halving;
the optional wheel `+40` scaling follows the captured flag array. The 72 original
contact-prefix cases cover all24 table slots in normal, mixed and wet states:
maximum1ULP, with packed material/flags exact. The 128 original wet-remap cases
cover all32 raw IDs and four flag combinations with exact integer output.

The RNW importer and query own triangle selection, attribute interpolation and
course-axis conversion. Native `0x3956f8` preserves interpolated attributes at
cached `+14/+18` (car `+7cc/+7d0`); neither becomes an extra friction multiplier.
Original spawn slope alignment requires a separate source-backed binding;
the adapter does not invent values for it. Existing flat-road query
support remains an explicit world configuration and regression path.

## Material interpretation limits

The captured native M3 tables do not establish English terrain names or a
universal road/not-road predicate. For example, raw indices0,9,11 all have class0
at `0x622430` and select tire slot0. Their decoded terrain factors are
approximately1.70,1.87,1.70, respectively; index11 cannot be excluded as "grass"
based on these coefficients. Raw2/3/4/6 have classification1, used for the
specific mixed-wheel roughness behavior. Calling that classification a general
road boundary would change its meaning. Route reconstruction must establish its
corridor from geometry/progress evidence instead of assigning unsupported names.

A controlled mesh bridge regression checks absolute/local position and normal
rotation, elevated spawn origin, original course-global noise coordinates, wet
material selection, and an actual environment step. It supplements the original
instruction fixtures; its synthetic geometry is explicitly a test input.

Two additional fixtures retain actual Monte Carlo and Suzuka triangles with
negative and positive winding. Unchanged original native queries select the
positive face even when the negative face is closer. The runtime regression
matches those heights within 0.05 mm and matches their material IDs. Provenance
and original query receipts are in `fixtures/*-native-backface.json`; only the
fixture vertex indices were remapped.

## Course condition initialization

Dry, wet, dirt and snow use separate immutable parameter instances where the
original code requires a difference. Original `0x34c998` maps dry/wet to kind0,
dirt to kind1 and snow to kind2. The checked-in off-road car-data and snow global
snapshots are outputs of unchanged `0x35f808`, the original aid-disable helpers,
and the original `0x363e14..0x363efc` constructor block including `0x361908`.
The byte-exact initialization fixture and source identities are recorded in
`course-condition-initialization.json`.

For this selected M3, dry and wet initialization exactly reproduce the existing
car and global bytes. Wet triangle flags still select the native wet material
indices. Dirt and snow clear the original stock ASC/TCS equipment settings;
this changes thirteen initialized car-data bytes. Snow also initializes material5
from raw table row15: its roughness amplitude becomes approximately0.002m instead
of0.005m. The original temporary snow tire-selector/terrain override is executed
before construction and restored afterward. Its selected compound and terrain
entries happen to agree for this M3, so no extra snow grip multiplier is added.
This equality is not a claim about other cars or equipment.

The recovered dirt/snow `0x3f9870` branch also blends axle braking with steering
angle using the original captured fifteen-degree threshold and percentages.
One hundred twenty-four original instruction fixtures cover all four conditions,
including adjacent F32 threshold values, at at most9ULP with exact flags. The
legacy dry flat-road fixtures retain their existing acceptance gates. No shared
global M3 state is mutated when a different course is selected.

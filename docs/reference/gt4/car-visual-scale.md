# Selected M3 visual frame

The selected original GT4 M3 GTR ALMS model replaces the earlier M4 dimensions
in the renderer. The world scale remains `28 / 2.917` units per metre; changing
that scale would also resize every imported road. Dimensions are applied in
metres through that same scale.

| Quantity | Recovered native value | Display value |
| --- | --- | --- |
| Body length | 4.618999243 m | 4.619 m |
| Body width | 1.878999829 m | 1.879 m |
| Forward body extent | 2.219999790 m | 2.220 m |
| Rear body extent | 2.398999453 m | 2.399 m |
| Wheelbase | 2.730999708 m | 2.731 m |
| Front / rear track | 1.607999802 / 1.602999806 m | 1.608 / 1.603 m |
| Front / rear tire radius | 0.323099971 / 0.336599976 m | 0.3231 / 0.3366 m |
| Tire width | 0.269999981 m | 0.270 m |

The body origin is not the envelope midpoint. A texture quad therefore runs
from -2.399 m to +2.220 m along the car, while the native axle positions are
approximately symmetric at +/-1.3655 m. Front and rear wheel diameters and
tracks remain separate.

Source: original model-info loading through `0x4458d0`, `0x4430f0` and
`0x361908`, checked against the independently executed original constructor.
The model resource is bound by original VARIATIONamerican row 275,
`bmw_0003_0`, to `/car/lod/bmw_0003` and `/car/menu/bmw_0003`; this is not a
filename guess. The extracted LOD SHA-256 is
`ed99160100a7ba1d8de0d93ea4ed4478f1a8fcb50ba5c8feddbd81f7a7d0dc81`.
The local extraction and binding receipts are under
`.audit/gt4-sprite/native-model/`.

## Existing collection artwork

The application uses the user-identified existing collection under
`oversteer/artifacts/carmodels-research/sprites/viewer-bakes`: model
`GT4_BMW_0003-source`, the M3 GTR Race Car '01 matching the selected physics car.
The example `GT4_BMW_0010` is a different road M3. The donor repository is read
only. Only the 13 needed source steering frames, saved bake metadata and a
hash/projection receipt are copied to `crates/neural_racing_gui/assets/gt4/viewer-bakes/`.

The collection GLB identifies the same original menu-model SHA-256
`183c4ab90b650d167ff020e309c8a196f562cca5abde220843d29fc843c339da` and
`m3_gtr_alms_01` source. Its existing PNGs face image-up and already contain the
original wheel/tire meshes. No separate tire overlays are drawn. The nearest
10-degree pose from -60 through +60 degrees follows the actual rack angle,
with the sign converted from the bake's positive-left convention to the
simulator's positive-right image convention. Pose selection is visual and does
not alter physics.

All 13 actual files are 512 x 512. The saved orthographic frame spans
4.769063949584961 m, giving 107.358593932 source pixels per metre. The model-bounds
centre is (0, -0.0895102024) m, giving native-origin pixel
(256, 246.390310527). The camera-centre inference comes from saved frame/model
bounds and is corroborated by projected mirror/front/rear alpha extents; it is
not a fit to the alpha box. Transparent cropping preserves that original pixel
calibration and origin. Mirrors are not squeezed into the nominal 1.879 m body
width, and steering-frame changes do not recenter the car.

The freshly decoded native orthographic body remains an independent audit
reference under `.audit/gt4-sprite/`; it is not a second selectable car or the
default artwork. The existing collection is the installed user-facing asset.

## Markings and width display

The start decoration spans actual `inner[0]` to `outer[0]`, preserving asymmetric
road edges, and has a fixed 0.4 m depth. It is simulator paint, not a claim to
reproduce the original GT4 texture. A regression first measured the old stripe
at 5.49 m deep, then checked its corrected depth and both edge endpoints.

The HUD uses the sensed `Env::road_width` (left + right) for full road width.
`road_half_width` remains side-dependent control/clearance data and is not doubled
into a misleading full-width label. An asymmetric-width HUD regression checks
20 m full width with a 3 m nearest-side clearance.

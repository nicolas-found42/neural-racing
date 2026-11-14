# GT4 road display and scale

The earlier display treated a sequence of collision cross-sections as a painted
road ribbon. That assumption was wrong in two ways. GT4 collision ground includes
aprons and adjoining surfaces, and measuring an edge in one normal frame before
reconstructing it in another moves its endpoint. The actual old renderer emitted
87 folded ribbon quads at Nürburgring and 20 at Suzuka. At the Suzuka spawn it
painted 35.777 m of combined ground while the HUD reported 10.535 m by doubling
only the nearer clearance. Those values described different surfaces.

The current display uses independently decoded original static MDLS ground
faces, textures, UVs and vertex colors where available. It does not infer a
painted white line from a collision material identifier. Coordinates remain
original metres, transformed `[X,Y-up,Z] → [X,-Z,Y-up]`, with the same world scale
as the calibrated [M3 sprite](car-visual-scale.md). Original road markings replace
synthetic kerbs and the oversized generated start stripe on these display meshes.

Source ground overlaps itself. Sloped faces cross in height and translucent
shadow planes sit above opaque ground, so no single face order is correct. The
(retained, gitignored) negative control
`.audit/gt4-road-boundaries/visual-prototype/depth-order-negative-control.json`
changes only paint order on the same packets, UVs, textures and colors: a
mean-height sort paints [100, 98, 94, 255] where the per-pixel depth result is
[0, 0, 0, 128] at physics (-572.5, -327.5) m on Suzuka, and 15,352 of 808,920
pixels differ (maximum RGB L1 difference 292). The matching DAG reports
for Suzuka and Nürburgring show genuinely crossing overlapping height planes and
blocked ordering cycles even at 0.5–1 mm tolerance, so min/max sorting or any
other whole-face heuristic is still wrong.

Core therefore rasterises the prepared source into world-aligned tiles
(`SourceGroundRasterizer`). Each pixel interpolates the authored face height,
UV and vertex color, samples the prepared GS-addressed texture, and composites
the covered fragments back to front. Translucent faces blend over the ground
beneath them instead of replacing it. The GUI chooses a power-of-two tile
density from the current view scale and pixels-per-point, keeps a bounded
LRU-style tile cache (128 tiles, 32 MiB), and rasterises visible tiles on one
background worker with generation and level cancellation. A coarser level is
chosen rather than exceeding the cache, so an overview converges; the tile
count is bounded by a budget, not by the course size. Tiles are drawn at their
exact world coordinates, so the car and the ground share one scale. While the
first tiles load, the neutral collision ground is shown beneath them; the old
mean-height face stream is no longer a rendering path.

Authored ground decals remain part of the original ground set. New York, for
example, contains translucent black shadow faces (16×16 texture, vertex alpha
128) with authored triangular silhouettes; they composite at their authored
alpha over road and grass. Such hard decal edges are source content, not
depth-order artifacts, and are not filtered out or softened.

The tile level is the coarsest density that is at least as fine as the
displayed pixels per metre, then coarsened further when the visible tile count
would exceed the budget. Sampling is never coarser than the display, so a
12.8 m road is at least three texels wide in an overview instead of dropping
between pixel centres; the budget keeps texture memory bounded instead of
baking an entire Nürburgring at follow-camera resolution. At the default 100 m
follow view the budget settles on 8 or 16 samples/m; a 6 km overview uses
0.25 samples/m. Beyond the exported static ground the canvas shows the ordinary
grass background; nothing is invented to fill it.

GS texture addressing can differ on each axis. The runtime prepares Repeat,
Clamp, Region Clamp and Region Repeat for egui's clamp sampler by preserving
the original pixel-address rule, splitting faces at UV boundaries and padding
texture seams. New points interpolate within the original face and height plane;
the resource acceptance check verifies each source face's projected area is
preserved. Raw vertex color 128 represents unity modulation. Unsupported texture
effects fail explicitly instead of being silently treated as ordinary modulation.

Display-only filtering retains static faces near positive-up original RNW ground.
It omits dynamic/nested model callbacks and distant scenery; this is a top-down
ground renderer, not a reproduction of GT4's entire scene, lighting or shaders.
The source export records unsupported models and packet failures separately.
Suzuka and Nürburgring decode without packet failures, and their original visual
faces cover all 13,330 checked authored/native-contact route positions within
0.4 m. Empty scenery outside those surfaces is not classified as a road hole.

When no supported textured display exists, the fallback paints original RNW
faces, clips its neutral road shading to the recovered corridor, and orders
native finite boundary segments together with faces. It never stretches a
folded quad across missing source ground. A practice area without a source
visual keeps its original area mesh, now also as the progressive base under
loading tiles. Physical boundaries are scoped by
their owning source triangle, as required by the original game's directed
collision callback; their roles are not automatically equated with painted edges.

Driving geometry remains separate. Authored ADLN line/circular-arc routes provide
navigation, with source-backed RNW/manual fallbacks for unsupported or unsuitable
resources. The guide moves only enough to fit the actual car footprint. Exact
cross-section endpoints are retained rather than reconstructed in a later normal
frame. The station/edge arrays are aligned through sorting, cropping and source
distance wrapping. A sprint on a closed source road interpolates across that
source's distance wrap while still ending at its real race-section finish.

All 191,792 final station/edge pairs are backed by original contact-query results
(192,081 successful original-code queries including changed-row rechecks).
Twelve cluster-zero distance results use explicitly recorded neighboring-distance
fallbacks. This establishes finite ground/contact and route correspondence;
collision clearance is still distinct from the width between painted white lines.
The HUD reports the full recovered cross-section and labels side clearance
separately. It does not double the nearer edge.

Reproduction tools live under `scripts/gt4/`: `export_visual_ground.py`,
`visual_vif.py`, `visual_tex1.py`, `adln_route.py`,
`build_physical_corridors.py` and `bind_physical_corridors.py`. Extraction uses
the user's original image; no perspective photograph or arbitrary width clamp
rescales the game geometry. Local audit receipts retain source hashes,
original-code evidence, red reproductions and before/after checks under
`.audit/gt4-road-boundaries/` and `.audit/track-visual-fix/`.

Rendering acceptance is now split. `source_raster` unit tests prove per-pixel
crossing order, translucent-over-opaque compositing, seamless adjacent tiles
and malformed/degenerate input handling. The production render test waits for
the visible tile set (deadline plus last state) and asserts that the actual
`draw_track` stream covers the course with textured tile images and reports no
source-visual failure. The previous paired-face/area-count renderer test was
removed: matching the old mean-height face stream did not detect the pixel
artifact it could not see.

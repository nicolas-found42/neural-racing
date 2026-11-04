# Real circuit reference vault

Captured 4 November 2026. Open [the gallery](index.html) or [contact sheet](contact-sheet.jpg).
The vault retains 9 full-page screenshots, 12 unretouched detail crops, and 27
source images. [manifest.json](manifest.json) records source URLs, original image
URLs, SHA-256 hashes, and crop coordinates. Thumbnails are navigation aids, not
additional evidence. Some full-page captures retain cookie overlays; the separate
source images and dimension detail crops provide unobstructed references. The
MotoGP comparison screenshot is labeled separately from the F1 layout used.
Source photographs retain their publishers' copyright;
they are reference material, not runtime game textures.

## Scale decisions

| Reference | Published dimensions | Implementation |
| --- | --- | --- |
| [BMW M4 GT3 EVO](https://www.bmw-m.com/en/fastlane/motorsport/race-cars/bmw-m4-gt3-evo.html) | 5.020 m long, 2.040 m wide, 2.917 m wheelbase | One world scale, 28 / 2.917 units per metre; body, tires and wing share it |
| [Monza operator](https://www.monzanet.it/en/circuit/) | 5,793 m; width 10–12 m | GP circuit, nominal 12 m road; car width 17.0% of road |
| [Silverstone / Formula 1](https://www.formula1.com/en/information/great-britain-silverstone-circuit.2DtFVI1FjkYgLVdGhbAIv0) | 5.891 km | Current GP layout; start on Hamilton Straight |
| [Tech3 Silverstone track data](https://motogp.tech3racing.fr/index.php/pages-gp/grand-prix-of-united-kingdom) | 15 m width | Nominal 15 m road; car width 13.6% of road. The rounded MotoGP length is not used. |
| [Red Bull Ring operator](https://www.redbullring.com/en/experiences/tracks/) | 4.326 km GP circuit | F1 layout, without the MotoGP chicane |
| [Puresport circuit operator experience](https://www.puresport.net/us/red-bull-ring-race-track) | 12–13 m width | Nominal 12.5 m road; car width 16.32% of road |

The Red Bull Ring geographic file labels its length 4,318 m. The current circuit
operator gives 4,326 m. The implementation uses the operator's figure and keeps
the discrepancy visible here. It does not accidentally use the shorter national
layout or the motorcycle chicane.

## What the pictures changed

- `monza-map-02` and `monza-map-03` show the two opening chicanes, Lesmos,
  Ascari and Alboreto/Parabolica. The game preserves their order and relative
  positions rather than substituting a random oval. `monza-map-05` shows GT cars
  across the first chicane: several car widths fit across the asphalt, with the
  green runoff outside it. `monza-map-06` shows the closed-wheel body and rear
  wing fitting within that footprint.
- `silverstone-02` shows the Arena loop, Brooklands/Luffield, Copse,
  Maggotts/Becketts, Hangar Straight and Stowe/Vale/Club. The game uses a north-up
  geographic projection; the publisher's map is rotated. The British GT race
  gallery shows GT cars on the start/finish surface and corner approaches;
  `silverstone-gallery-02` through `-06` establish the broad road relative to
  the cars. Perspective images support the proportions qualitatively; they are
  not treated as an orthographic ruler.
- `ring-map-01` identifies the F1 course in the aerial view. `ring-racing-01`,
  `-02` and `-04` show the cars approaching/rounding Turn 1, asphalt boundaries,
  and the distinction between track, kerb and runoff. The implemented circuit
  preserves the long uphill runs, tight upper hairpin and infield sweepers.
- `car-05` gives an elevated view of the BMW body; `car-01` and `-03` show the
  wing and closed wheels. The renderer now uses an elongated GT silhouette,
  enclosed tires and a rear wing. Fixed-pixel wheels no longer enlarge the car
  when zoomed out. Kerbs sit outside the asphalt, rather than reducing its width.

## Geometry provenance and accuracy

The three original [GeoJSON files](data/) come from
[Tomislav Bacinger's f1-circuits](https://github.com/bacinger/f1-circuits), under
the included [MIT license](data/geojson-license.txt). Runtime Rust coordinate
arrays retain the original vertices. Projection converts longitude using the
local latitude, flips latitude for north-up screen coordinates, and uses the same
scale on both axes. Segments are subdivided to at most 5 m before two local
smoothing passes. A final uniform factor makes the **measured closed centerline**
equal the published length; the length is not just a UI label.

This is a map-based 2D recreation, not a laser scan. Nominal width is constant on
each real circuit; local widening/narrowing, elevation, camber, pit lanes, exact
kerb placement and individual runoff areas are not reproduced. The existing
vehicle dimensions are retained; handling now uses a mass-and-inertia four-contact-patch model with front-wheel steering. The photographic evidence supports a
realistic overall car/road ratio; it cannot establish centimetre-level geometry.

## Implementation evidence

| Circuit | Follow camera | Full layout |
| --- | --- | --- |
| Monza | [capture](../screenshots/monza-follow.png) | [capture](../screenshots/monza-overview.png) |
| Silverstone | [capture](../screenshots/silverstone-follow.png) | [capture](../screenshots/silverstone-overview.png) |
| Red Bull Ring | [capture](../screenshots/red-bull-ring-follow.png) | [capture](../screenshots/red-bull-ring-overview.png) |

`real_circuits.rs` measures closed-loop distances and road/car ratios and drives
two laps on each layout. GUI tests render all three circuits, switch cameras,
restart, advance via the keyboard, cycle tracks and run the shipped neural
policy for two laps. Camera tests check centering, bounds and scale ratios at
large and minimum-sized canvases. The overview locator is a map symbol; the
actual car sprite is never enlarged independently of the road.

Publication note: temporary signed download query parameters were removed from
the manifest before committing. Original source-page URLs, local captures and
SHA-256 hashes remain. GeoJSON trailing whitespace was normalized without
changing parsed geometry. The implementation-evidence narrative above records
the earlier capture stage; current learning and observability results are in
[learning validation](../learning-validation.md) and the
[observability verification record](../observability-verification.md).

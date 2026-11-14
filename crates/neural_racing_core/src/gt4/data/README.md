# Initialized M3 parameter snapshot

These files contain numeric configuration and native curve-pointer values,
not executable instructions. The loader resolves those pointers against checked
captured regions and copies curve values into Rust-owned tables.

The original stock M3 GTR ALMS 2001 equipment constructor, expanded conversion,
global initializers `0x35f808(normal-road context)`, `0x3f7be0`,
`0x366100` (steering compliance) and `0x347a00` (controller response),
and car-data initializer `0x361908` produced these
bytes. Independent execution using unchanged GT4 instructions on upstream Play!
with its official kernel HLE matched the offline builder byte for byte.

* `m3_gtr_alms_01.bin`: 4,352 bytes at native base `0x01d02000`, SHA-256
  `572905793b61abb6a71810c4bc2e021520796d1686418fe25645befe1cac278e`.
* `global_parameters.bin`: 9,472 bytes at `0x00620000`, SHA-256
  `7e3756b3b5b980f0db9e9348efaa11d2607cad42af60edb2b249359a4da38c6a`.
* `steering_globals.bin`: 32 bytes at `0x00845bf0`, SHA-256
  `e2fb876a7a1256d1734d3ba31a68ae878564de42f9c9f1daec825ee61cd662de`.
  The original initialized steering mode is one.
* `steering_response_globals.bin`: 904 bytes at `0x00843538`, SHA-256
  `28759d691b70d91da61673f08bdf0df9116e94ea80aca3177271a6c4ac8f8bb6`.
  Original `0x347a00` and its `0x353938` tail initialize these controller
  response arrays; independent native execution matches all 904 bytes.

Inputs selected normal manual mode, zero-filled runtime configuration and zero
tail argument. These are explicit integration inputs; this does not establish
every factory race setting or general game-boot global initialization.
The road/controller context bytes are explicitly zero. Controller BSS initialized
by `0x347a00` also has its own recovered port and independent captured proof.

Full source identities, raw output hex, input row hashes, bootstrap conditions,
limitations and independent comparisons are in
[the constructor evidence](../../../../../docs/reference/gt4/native-constructor.md).


`m3_gtr_alms_01_offroad.bin` and `global_parameters_snow.bin` are separately
initialized outputs for dirt/snow course conditions, not hand-adjusted tuning.
The original constructor block clears stock aid settings for both conditions;
snow additionally selects raw surface-table row15 for material5. Complete
source/input/output identities and byte comparisons are in
`docs/reference/gt4/course-condition-initialization.json`. Dry/wet retain the
original snapshots; different courses never mutate a shared parameter instance.

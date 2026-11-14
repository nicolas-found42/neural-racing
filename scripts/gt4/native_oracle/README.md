# Original EE progress oracle

This analyst harness runs the supplied GT4 CORE's unchanged runway relocation,
contact query, BSP traversal, triangle collision and progress projection routines.
It does not boot the game and needs no Sony BIOS. Its query runner uses only
Python's standard library (Python3.11+) and the separately built upstream Play EE
engine. This is emulator execution evidence, not physical PS2 bit equality.

Use the recorded upstream source and recursive submodules:

```sh
git clone https://github.com/jpd002/Play- .audit/gt4-native-oracle/Play
git -C .audit/gt4-native-oracle/Play checkout 1f27232772e484bfb79364b751b84f41f4c87481
git -C .audit/gt4-native-oracle/Play submodule update --init --recursive
cmake -S scripts/gt4/native_oracle -B .audit/gt4-native-oracle/build -G Ninja \
  -DPLAY_SOURCE="$PWD/.audit/gt4-native-oracle/Play" \
  -DCMAKE_BUILD_TYPE=Release -DENABLE_AMAZON_S3=OFF -DDEBUGGER_INCLUDED=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build .audit/gt4-native-oracle/build --target gt4-native-ee-runner -j 2
```

CMake, Ninja, Git and a C++ compiler are host prerequisites. On the recorded
Darwin/ARM64 environment, upstream Play and its submodules supplied the remaining
libraries. This workflow does not install global software or alter emulator core
source. CMake verifies the pinned source commit. The target CORE is checked
against SHA256`e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804`;
addresses from another build are not interchangeable.

After the extraction/decoder produces `decoded/<course>.json` and the referenced
original `decoded/runways/*.rnw`, supply a JSON array such as:

```json
[{"course":"suzuka","index":0,"query":[529.1676,-9.7670,-518.6332]}]
```

Coordinates are original RNW XYZ, with Y up, in metres. Extra fields are preserved
in the results. Height and route/deck selection are explicit caller inputs:
query success alone does not prove that a candidate belongs to the driving route.

```sh
python3 scripts/gt4/run_native_progress.py .audit/gt4-tracks/native-v-all-inputs.json \
  --audit-root .audit/gt4-tracks \
  --runner .audit/gt4-native-oracle/build/gt4-native-ee-runner \
  --elf .audit/gt4-handling/extracted/CORE.ps2.elf \
  --output-prefix native-anchor-check --batch-size 32
```

Each batch executes original `0x3e5818`, then original `0x395760` and `0x3e6858`
for each point. The original contact wrapper starts its downward ray five metres
above the supplied query Y. Returned progress belongs to the native selected
triangle/cluster, which can differ from the caller's nearest-height candidate.
A minimal declared owner/course structure supplies the original runway pointer.
The trampoline performs ABI argument loads and output stores, with no host
geometry callback or substituted game callee. Executable and mutable buffers
occupy separate16KiB pages.

The output directory is `<audit-root>/native-v/`: binary inputs, raw stdout/stderr,
and a results JSON with original bits, contact status, hit height, packed hint,
source hashes, runner/ELF hashes and script identity. Native progress zero can be
a fallback (especially cluster0), not necessarily the start line. Compare it with
route continuity and preserve exceptions explicitly. The default32query batch is
conservative; larger batches up to128 can exceed the bounded runner's one-million
cycle budget on repeated misses. A failed batch is reported, never fabricated.

Prior retained runs under `.audit/gt4-tracks/native-v/` establish actual execution.
Their recorded binary hashes remain historical identities; rebuilding can change
the binary hash without changing the pinned original instructions or emulator
source. No large emulator build or private game ELF is checked into this folder.

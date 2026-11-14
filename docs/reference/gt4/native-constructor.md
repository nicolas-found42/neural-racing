# Original M3 constructor execution

The supplied CORE ELF’s unmodified database readers, cache/mutex functions, equipment constructor, converters and selected numeric initializers ran on upstream Play!’s R5900 ARM64 JIT and HLE kernel. An independent opcode interpreter produced byte-identical outputs.

The [machine-readable evidence](m3-native-constructor.json) records target, emulator and harness identities, input hashes, complete output bytes and explicit context limitations.

| Output | Bytes | SHA-256 |
| --- | ---: | --- |
| Equipment | 376 | `baacfdd365fd7e9440039334b21bf40f47a28c6513aa393ce92c10a744008500` |
| Expanded equipment | 1536 | `cf842a38d73103304e2da2b9316e099adf0ea85ab6411e7118189b3bcae88bdf` |
| Initialized carData | 4352 | `572905793b61abb6a71810c4bc2e021520796d1686418fe25645befe1cac278e` |
| Main globals | 9472 | `7e3756b3b5b980f0db9e9348efaa11d2607cad42af60edb2b249359a4da38c6a` |
| Steering globals | 32 | `e2fb876a7a1256d1734d3ba31a68ae878564de42f9c9f1daec825ee61cd662de` |
| Steering-response globals | 904 | `28759d691b70d91da61673f08bdf0df9116e94ea80aca3177271a6c4ac8f8bb6` |

The database inputs are regenerated uncompressed American SpecDB views that preserve original decoded row IDs and bytes. Addresses and cache state are controlled inputs. Original searches, copies, locks and syscalls execute unchanged; no game callee supplies a host-calculated result.

The numeric initialization order is `35f808(CONDUCTOR)`, `3f7be0()`, `366100(620b00,620af0,4,2)`, `347a00(CONDUCTOR)`, then `361908`. A zero-filled 64 KiB conductor explicitly selects normal-road/controller context zero. The manual carData initializer receives a controlled zero-filled runtime configuration and `t1=0`. This establishes behavior under those inputs; it is not a captured factory race state or a claim that every game-context initializer is recovered.

The main global window is `0x620000..0x622500`; steering windows are `0x845bf0..0x845c10` and `0x843538..0x8438c0`. Initialized steering mode is **1**. Earlier BSS-only snapshots lacked mandatory steering compliance and course-dependent surface initialization; those are historical checkpoints, not current defaults.

The upstream kernel initializes a legitimate main thread through SetupThread. The ELF’s GP is `0x6dddf0`; upstream Mach exception handling manages JIT-protected guest pages. No Sony BIOS is needed. Emulator results do not establish physical PS2 bit equality.

Local reproduction with the supplied target, decoded inputs and retained emulator source:

```sh
cmake --build .audit/gt4-port/differential/play-build --target gt4-hle-runner -j 2
python3 .audit/gt4-port/differential/run_native_m3.py equipment
python3 .audit/gt4-port/differential/run_native_m3.py expanded
python3 .audit/gt4-port/differential/run_native_m3.py car_data
```

The audit directory retains source, native outputs, failed-run diagnoses and byte comparisons. A fresh clone does not include the game; the retained JSON makes the results reviewable without those private artifacts.

# Cross-path capture fixtures

Chain-B captures of the drone's own PA A transmitting the cross-path probe
frame (HT MCS1, one stream), used by `tests/test_cross_evm.cpp`. They come
from the archive `spike-selfcal2-captures.tar.zst` of the cross-path
self-calibration spike
([finding](../../../docs/findings/2026-11-26-drone-self-calibration-cross-path.md)).
Each archive file is named `<dir>/i<TXAGC index>_<listener gain word>_<take>.bin`.

| Fixture | Archive path | Width |
|---|---|---|
| `c144_20_i028.hi32` | `x144_20_r/i028_0x3000e_1.bin` | 20 |
| `c144_20_i060.hi32` | `x144_20_r/i060_0x3000e_1.bin` | 20 |
| `c144_20_i124.hi32` | `x144_20_r/i124_0x3000e_1.bin` | 20 |
| `c40_20_i060.hi32` | `x40_20_r/i060_0x300c0_1.bin` | 20 |
| `c40_20_i060_weak.hi32` | `x40_20_r/i060_0x3000e_1.bin` | 20 |
| `c144_20_i028_clip.hi32` | `x144_20_r/i028_0x300c0_1.bin` | 20 |
| `c40_20_noise.hi32` | `x40_20_r/i060_0x30c00_2.bin` | 20 |
| `c144_40_i028.hi32` | `x144_40_r/i028_0x3000e_1.bin` | 40 |
| `c144_40_i076.hi32` | `x144_40_r/i076_0x3000e_2.bin` | 40 |
| `c144_40_i124.hi32` | `x144_40_r/i124_0x3000e_2.bin` | 40 |
| `c144_40_noframe.hi32` | `x144_40_r/i076_0x3000e_1.bin` | 40 |
| `c40_20_i040_edge.hi32` | `x40_20_r/i040_0x30800_1.bin` | 20 |
| `c40_40_i072_edge.hi32` | `x40_40_r/i072_0x3000e_1.bin` | 40 |
| `c112_40_i064_two.hi32` | `x112_40_r/i064_0x3000e_2.bin` | 40 |
| `c40_20_i112_two.hi32` | `x40_20_r/i112_0x3000e_1.bin` | 20 |

`c40_20_noise` was taken with nothing on the air; `c144_40_noframe` holds no
complete frame. The `_edge` captures have their best symbol timing just past
the edge of the EVM floor; the `_two` captures hold two frames, the later one
the better.

Conversion: an archive `.bin` is a 32-byte header followed by one
little-endian `uint64` logic-analyzer word per sample. A `.hi32` keeps only
the high dword of each word (bytes 4..7 of every 8) as a little-endian
`uint32`: 8192 samples at 20 MHz, 16384 at 40. Path-B I is bits 23:12, Q is
bits 11:0 (12-bit two's complement), and bit 29 clear marks a bad I sample;
the test rebuilds the word as `uint64_t{hi} << 32` for
`devourer::cross_path_sample`.

The `matches_the_reference` values in the test are the spike's numpy
measurement on the first eleven files; `keeps_the_exhaustive_search_answer`
pins every file to what the exhaustive timing search returned.

## Probe pairs for `test_pa_fit`

The twelve `d…` / `h…` files are the survey's weakest-word (`0x3000e`) takes
1 and 2 of the bench drone (`d…`) and of a second RTL8812EU (`h…`), named
`<unit><channel>_<width>_i<TXAGC index>_<a|b>.hi32` (take 1 is `a`, take 2 is
`b`). The expected values in the test come from
`pafit_ref.py <width> <a> <b>`, the numpy reference the C++ fit ports
(`nix-shell -p 'python3.withPackages(ps: [ps.numpy])'`).

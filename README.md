<div align="center">

# cbirds

**A flock of birds in your terminal.** Real sprites, real flocking, no dependencies.

![Language](https://img.shields.io/badge/Language-C99-blue.svg)
![Dependencies](https://img.shields.io/badge/Dependencies-libc%20%2B%20libm-brightgreen.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS-lightgrey.svg)

![The flock spelling its own name](docs/hero.png)

*That is a terminal. Those are PNG sprites. It wrote its own name, and then took
its own photograph with its own PNG encoder.*

</div>

## What this is

Craig Reynolds' boids, from 1986, rendered as rotated PNG sprites through the
Kitty graphics protocol. Eight hundred birds at sixty frames a second, in a
terminal, in about a third of a millisecond of CPU per frame.

There is no zlib, no libpng, no ncurses and no SDL. The PNG decoder, the PNG
encoder, the DEFLATE compressor *and* decompressor, the CRC and Adler checksums,
the rotation, the resampling and the 5×7 font are all in this repository, in
about 7,500 lines of C99 that link against libc and libm and nothing else. The bird is one PNG
compiled into the binary; every rotation and every colour of it is built at
startup, in memory, by the project's own code.

```bash
git clone https://github.com/clainstone/cbirds && cd cbirds && make && ./cbirds
```

## Try these

The flock writes whatever you pipe into it:

```bash
fortune | cbirds --spell -
echo "SHIP IT" | cbirds --spell -
```

It wears your terminal's own colours, because it asks:

```bash
cbirds                    # already does, that is the default
cbirds --color ember      # or pick a ramp
```

<div align="center"><img src="docs/murmuration.png" alt="A murmuration" width="90%"></div>

```bash
cbirds --preset murmuration --trails    # the starling look, with tails
cbirds --hawks 2                        # give the clip a story
cbirds --flocks 3 --color ice           # three flocks that will not merge
cbirds --clock                          # the flock is the time
cbirds --screensaver                    # for a terminal left open
cbirds --matrix                         # it is raining birds
cbirds --shape fish --wrap              # or a school, off one edge and onto the other
```

<table>
<tr>
<td width="50%"><img src="docs/hawks.png" alt="Two hawks scattering the flock"></td>
<td width="50%"><img src="docs/matrix.png" alt="Matrix rain, as birds"></td>
</tr>
<tr>
<td align="center"><code>--hawks 2 --color acid</code></td>
<td align="center"><code>--matrix</code></td>
</tr>
</table>

And then move your mouse. The flock parts around the pointer.

## The panel

Every adjustable parameter is a slider in the corner, with its value and the two
keys that move it. Lowercase lowers, uppercase raises, and **one keypress is
exactly one notch of bar** — the notch is the state the keys move and the value
is derived from it, so the number and the bar cannot disagree.

```
╭────────────────────────────────────╮
│ boundary   ▓▓▓▓░░░░░░░░  0.20  b/B │
│ separation ▓▓▓▓░░░░░░░░ 0.005  s/S │
│ cohesion   ▓▓▓▓░░░░░░░░ 0.010  c/C │
│ alignment  ▓▓▓▓░░░░░░░░  1.50  a/A │
│ perception ▓▓▓▓▓▓░░░░░░  36px  p/P │
│ rate       ▓▓▓▓░░░░░░░░    60  r/R │
│                                    │
│ quit       q                       │
╰────────────────────────────────────╯
```

The flock cannot fly through it. The panel's rectangle carries an edge force of
100000, far above every other term in the model, aimed at whichever of its two
open sides is nearer — and the force acts on the panel grown by one frame of
travel, which is what makes it unreachable rather than merely unwelcoming. Over
**784,000 placements across seven configurations**, not one landed on it.

| key | | key | |
|---|---|---|---|
| `b`/`B` `s`/`S` `c`/`C` `a`/`A` `p`/`P` `r`/`R` | one notch down / up | `h` | hide the panel |
| `space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+`/`-` | more or fewer birds | `k`/`K` | summon or dismiss a hawk |
| `e` | trails | `w` | wrap |
| `M` | cycle the pointer's mode | `L` | cycle what picks a colour |
| `q` | quit, with a fly-away | | |

There is also the Konami code. It is in `--help`, under **Oddities**, because an
easter egg nobody finds is wasted.

## Options

`cbirds -h` is one screen of the dozen that matter. `cbirds --help` is all
forty, grouped. Both come off the same table that drives the parser, so they
cannot drift apart, and `--completion bash|zsh|fish` walks it too.

It accepts what people actually type: `-n 800`, `-n800`, `--birds 800`,
`--birds=800`, clustered flags, `--no-NAME`, `--` to end the options. A typo is
answered with the name you probably meant. Usage errors exit 2, so a script can
tell a mistyped command from a run that went wrong.

```
$ cbirds --colour ember
cbirds: unknown option '--colour', did you mean '--color'?
Try 'cbirds --help'.
```

## How it works

**The sprite.** One PNG, compiled into the binary as a C array. At startup it is
decoded, scaled up 8×, then rotated into 90 frames at 4° apart and re-encoded as
90 PNGs in memory — 50 ms. Colour is a second set of images, because Kitty has
no per-placement tint; the rotation is the expensive half and does not depend on
the colour, so each angle is rotated **once** and then tinted and encoded per
shade. Five shades cost 35 ms, not 250.

**The protocol.** Each frame is one synchronized update (`CSI ? 2026 h`), one
global placement clear, one placement per bird, then the panel, then a
non-blocking flush that keeps only the unsent suffix if the terminal cannot
swallow it. The screen is **never** cleared after the sprites are uploaded:
clearing deletes them, and every later placement would point at an image that no
longer exists. A test asserts that no frame ever carries a screen erase.

**The compression.** `png_encode` runs LZ77 with a hash chain and the fixed
Huffman codes of RFC 1951, which the inflater in the same file has always been
able to read. Fixed rather than dynamic because there is no tree to build and no
second pass, and on this data — long runs of one colour, long runs of
transparency — it lands within a few percent of what a dynamic tree would.
Stored blocks remain the fallback, so incompressible data cannot come out larger
than it went in. It earns its place twice: `--size 64` used to push 10 MB of
base64 before drawing anything and now pushes 726 KB, and the images in this
README came straight out of `--snapshot` at 50–100 KB instead of 4.15 MB each.

**The neighbours.** A uniform 12×12 pixel grid, rebuilt every frame with counting
and prefix sums. The cell size is exactly the perception quantum, so the
`(2v+1)²` block a bird sweeps is provably sufficient with no slop — and a test
compares the grid against a brute-force scan to 1e-11, across every radius and
every flock count.

**The flocking.** Separation, alignment and cohesion, from the same immutable
snapshot for every bird, so the order they are updated in cannot matter. Flocks
are social, not physical: separation applies to every bird in reach, alignment
and cohesion only to your own flock, which is why three flocks interpenetrate
and refuse to merge.

**The writing.** A 5×7 font, authored as rows of `#` so it can be corrected by
eye. A target per lit cell, a bird per target round robin, and a bird with a
target steers at it and moves the *smaller of its speed and the distance left* —
which is what makes a letter crisp instead of a cloud orbiting one.

## Numbers

`--bench N` runs N frames with no terminal at all and prints these, so you can
check them rather than take them on trust. Ryzen-class laptop, 1600×800 viewport:

| birds | CPU per frame | ceiling | bytes per frame |
|---|---|---|---|
| 400 | 0.170 ms | 5900 fps | 11.4 KB |
| 800 | 0.353 ms | 2832 fps | 22.3 KB |
| 2000 | 1.003 ms | 997 fps | 45.4 KB |
| 4096 | 2.734 ms | 366 fps | 108.3 KB |

The simulation is not the bottleneck and has not been since the spatial grid
landed. The limit is terminal bandwidth: 1.3 MB/s at the default, 6.5 MB/s at
four thousand birds.

## Requirements

A terminal that speaks the Kitty graphics protocol: **Kitty**, **WezTerm**,
**Ghostty**, recent **Konsole**. Alacritty and GNOME Terminal cannot show it.

You will not get a black screen finding out. cbirds asks the terminal whether it
can draw, before taking the screen, and says so plainly if it cannot:

```
$ cbirds
cbirds draws with the Kitty graphics protocol, and this terminal did not
answer for it. Kitty, WezTerm, Ghostty and recent Konsole all do.
Run it under one of those, or pass --force to try anyway.
```

Build needs GCC 7+ or Clang 10+ and `make`. `make test` runs four suites.

## Layout

| | |
|---|---|
| `boids.c` | the simulation, the panel, the terminal |
| `options.c` `options.h` | the option table that drives both the parser and `--help` |
| `kitty_graphics.c` `.h` | the buffered protocol, with flow control |
| `spatial_grid.c` `.h` | the uniform grid and its contiguous cell buckets |
| `png.c` `png.h` | the PNG library: decode, encode, DEFLATE, rotate, resize, tint |
| `font.c` `font.h` | the 5×7 font the flock writes with |
| `sprite_png.h` | the bird, generated from `matrix.png` by `mkasset.c` |
| `tests/` | four suites, run by `make test` |

## Contributing

Genuinely open, and one of these is nearly free:

**A dynamic Huffman encoder.** `png_encode` uses fixed codes, which is within a
few percent on sprite data and further off on photographs. The decoder already
reads dynamic blocks, so the tests are waiting for it.

**A canvas renderer.** One image a frame instead of one placement a bird, which
is the road to twenty thousand birds: composite into one RGBA canvas — the
`--snapshot` path already does exactly this — compress it, and send it as a
single `a=T`. The flocking update is trivially parallel thanks to the snapshot.

**More sprites.** `--sprite FILE` takes any PNG through the project's own
decoder, and `--shape` draws five of them from triangles. A sixth is a pull
request with one function in it.

**The ideas not taken.** Music reactivity, obstacles the flock must fly around,
perching along the bottom edge, V formations, a config file, tmux passthrough.
Fifty were proposed and twenty were built; the rest are listed in `plan.md`.

## License

MIT. See [LICENSE](LICENSE).

## Acknowledgements

**Craig Reynolds** for the [boids algorithm](http://www.red3d.com/cwr/boids/),
1986. **Kovid Goyal** for the
[Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/),
without which none of this could be in a terminal at all.

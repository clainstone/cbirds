# The clips in the README

Every one of them was recorded by cbirds itself, with its own GIF encoder.
Nothing outside this repository touched them. To regenerate:

```sh
make

# Every clip opens with the three seconds in which the flock writes BOIDS, so
# eight seconds is five of flocking. Each flies slower than the tuned pace, at
# its own notch: --speed 0 is 0.2x, 1 is 0.4x (the default), 2 is 0.6x.

# The clip at the top: the murmuration preset with tails, at the default pace.
./cbirds --record docs/demo.gif --record-fps 25 --record-seconds 8 \
         --record-size 120x32 -n 250 -s 12 --color ember --trails \
         --preset murmuration --speed 1 --seed 11

# The gallery. The sprite cells are recorded at 64x18 cells, 25 frames a
# second, so that GitHub shows the birds near the size they were drawn at.
./cbirds --record docs/hawks.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 14 --color ice --hawks 2 --speed 0 --seed 5
./cbirds --record docs/flocks.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 14 --color ember --flocks 3 --speed 1 --seed 3
./cbirds --record docs/matrix.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 12 --matrix --speed 0 --seed 1
./cbirds --record docs/depth.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 160 -s 14 --color ice --depth --trails --speed 1 --seed 9

# The text ones are what a terminal with no graphics protocol shows: under
# --render braille or sextants the GIF is of the cells, not of the pixels.
# 80 columns is an honest text terminal; dots are cheap, so they get more
# frames a second.
./cbirds --record docs/braille.gif --record-fps 33 --record-seconds 8 \
         --record-size 80x22 -n 360 -s 14 --color ice --hawks 2 --render braille --speed 1 --seed 5
./cbirds --record docs/sextants.gif --record-fps 50 --record-seconds 7 \
         --record-size 80x22 -n 360 -s 12 --color acid --render sextants --speed 2 --seed 5

# And the asciinema cast, at the default pace.
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 --color ember --hawks 2 --seed 5
```

The ceiling is 50 frames a second, and the reason is the format, not the
program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
treat anything under two hundredths as a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

A GIF frame costs about three bits per bird pixel, so the size of a clip is set
by how many birds are on the screen and how big they are, not by the
resolution. The clip at the top, 250 birds at twelve pixels for eight seconds
at 25 frames a second, is 2.9 megabytes; the text clips are a fraction of
that, because dots compress. The cost is birds times sprite area times frames,
so a slower, smoother clip pays for its frames with birds, not with
resolution.

A recording whose name ends in `.cast` is an asciinema file instead: the flock
as the braille renderer sends it, one line of escape sequences per frame,
playable in any terminal with `asciinema play docs/demo.cast`.

`--record` needs no terminal at all. `--snapshot FILE` does, because it
photographs a live frame. The picture is what the terminal was showing, at its
size in pixels: sprites under Kitty, dots or blocks anywhere else.

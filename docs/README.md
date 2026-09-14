# The clips in this README

Every one of them was produced by cbirds itself, with its own GIF encoder.
Nothing outside this repository touched them. To regenerate:

```sh
make

# The animation at the top: the flock writes its name, lets go, and murmurates.
./cbirds --record docs/demo.gif --record-fps 20 --record-seconds 5 \
         --record-size 120x32 -n 500 -s 12 --color ember --trails \
         --seed 11 --preset murmuration

# The gallery, all at 100 by 28 cells, twenty frames a second, five seconds.
r() { ./cbirds --record "docs/$1" --record-fps 20 --record-seconds 5 \
               --record-size 100x28 -s 12 "${@:2}"; }
r hawks.gif       -n 380 --color acid   --hawks 2 --seed 5
r flocks.gif      -n 380 --color ember  --flocks 3 --seed 3
r murmuration.gif -n 400 --color ice    --preset murmuration --trails --seed 7
r depth.gif       -n 420 --color ice    --depth --trails --seed 9
r storm.gif       -n 380 --color acid   --preset storm --shape plane --seed 9
r matrix.gif      -n 350 --matrix
r ember.gif       -n 400 --color ember  --seed 2
r arrows.gif      -n 380 --color matrix --flocks 2 --shape arrow --seed 13
# The text ones are what a terminal with no graphics protocol shows: under
# --render braille or sextants the GIF is of the cells, not of the pixels.
r braille.gif     -n 500 --color ember  --hawks 2 --render braille --seed 5
r sextants.gif    -n 500 --color ice    --render sextants --seed 5

# And the asciinema cast.
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 --color ember --hawks 2 --seed 5
```

Fifty frames a second is the ceiling, and the reason is the format rather than
the program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
clamp anything under two hundredths up to a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

A GIF frame costs about three bits per bird pixel, so the size of a clip is set
by how many birds are on the screen and how big they are, not by the
resolution. Four hundred birds at twelve pixels for five seconds is about a
megabyte and a half; the text clips are a fraction of that, because dots
compress.

A recording named `.cast` is an asciinema file instead: the same flock as the
braille renderer sends it, one line of escape text per frame, playable with
`asciinema play docs/demo.cast` in any terminal.

`--record` needs no terminal at all. `--snapshot FILE` does, because it
photographs a live frame: the picture is of whatever that terminal was shown,
sprites under Kitty, dots or blocks anywhere else, at the terminal's size in
pixels.

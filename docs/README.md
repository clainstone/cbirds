# The images in this README

Every one of them was produced by cbirds itself, with its own PNG and GIF
encoders. Nothing outside this repository touched them. To regenerate:

```sh
make

# The animation at the top: the flock writes its name, lets go, and murmurates.
./cbirds --record docs/demo.gif --record-fps 20 --record-seconds 5 \
         --record-size 120x32 -n 500 -s 12 --color ember --trails \
         --seed 11 --preset murmuration
# The two shorter ones in the gallery.
./cbirds --record docs/hawks.gif  --record-fps 20 --record-seconds 5 \
         --record-size 120x32 -n 500 -s 13 --color acid --hawks 2 --seed 5
./cbirds --record docs/flocks.gif --record-fps 20 --record-seconds 5 \
         --record-size 120x32 -n 500 -s 13 --color ember --flocks 3 --seed 3
# And the cast.
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 --color ember --hawks 2 --seed 5

# The stills. --snapshot writes the last frame, so --frames chooses the moment;
# the first three seconds are the intro, so none is taken before frame 400.
# --no-panel because the panel is terminal text: it is not in the PNG, and left
# on it would hold the flock out of a corner for no visible reason. These were
# taken in a 120 by 34 cell terminal of 1200 by 680 pixels.
./cbirds --snapshot docs/hero.png        --frames 420 -n 1100 --color ember --seed 5 --no-panel
./cbirds --snapshot docs/murmuration.png --frames 440 -n 900  --color ice \
         --preset murmuration --trails --seed 7 --no-panel
./cbirds --snapshot docs/depth.png       --frames 440 -n 1000 --color ice \
         --depth --trails --seed 9 --no-panel
./cbirds --snapshot docs/storm.png       --frames 440 -n 800  --color acid \
         --preset storm --shape plane --seed 9 --no-panel
./cbirds --snapshot docs/arrows.png      --frames 440 -n 900  --color matrix \
         --flocks 2 --shape arrow --seed 13 --no-panel
./cbirds --snapshot docs/matrix.png      --frames 420 -n 700  --matrix --no-panel
# The text ones are what a terminal with no graphics protocol shows: under
# --render braille or sextants a snapshot is a picture of the cells, not of
# the pixels.
./cbirds --snapshot docs/braille.png     --frames 460 -n 700  --color ember \
         --hawks 2 --seed 5 --render braille --no-panel
./cbirds --snapshot docs/sextants.png    --frames 460 -n 700  --color ice \
         --seed 5 --render sextants --no-panel
```

Fifty frames a second is the ceiling, and the reason is the format rather than
the program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
clamp anything under two hundredths up to a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

A few megabytes is a lot for a README, and the birds are why: a GIF frame costs
about three bits per bird pixel, so the size is set by how many birds are on the
screen and how big they are, not by the resolution. Twenty frames a second and
five hundred birds keep each clip under two.

A recording named `.cast` is an asciinema file instead: the same flock as the
braille renderer sends it, one line of escape text per frame, playable with
`asciinema play docs/demo.cast` in any terminal, at a fraction of the size of
the GIF.

```sh
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 --color ember --hawks 2 --seed 5
```

`--record` needs no terminal at all. The `--snapshot` runs do, because they
photograph a live frame; any terminal will do, and the picture is of whatever
that terminal was shown — sprites under Kitty, dots or blocks anywhere else. The
size of the picture is the size of the terminal in pixels.

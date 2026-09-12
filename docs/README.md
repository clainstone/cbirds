# The images in this README

Every one of them was produced by cbirds itself, with its own PNG and GIF
encoders. Nothing outside this repository touched them. To regenerate:

```sh
make

# The animation at the top: a band of birds, its own name, then a murmuration.
./cbirds --record docs/demo.gif --record-fps 50 --record-seconds 6 \
         --record-size 104 --record-rows 28 \
         -n 900 --color ember --trails --spell CBIRDS --spell-hold 2 \
         --seed 11 --preset murmuration

# The stills. --snapshot writes the last frame, so --frames chooses the moment.
./cbirds --snapshot docs/hero.png        --frames 150 -n 1100 --color ember \
         --spell CBIRDS --spell-hold 0 --no-intro
./cbirds --snapshot docs/murmuration.png --frames 300 -n 900  --color ice \
         --preset murmuration --trails --no-intro
./cbirds --snapshot docs/matrix.png      --frames 200 -n 700  --matrix --no-intro
./cbirds --snapshot docs/hawks.png       --frames 300 -n 800  --color acid \
         --hawks 2 --no-intro
```

Fifty frames a second is the ceiling, and the reason is the format rather than
the program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
clamp anything under two hundredths up to a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

`--record` needs no terminal at all. The `--snapshot` runs do, because they
photograph a live frame; run them in Kitty, WezTerm or Ghostty.

# The images in this README

Every one of them was produced by cbirds itself, with its own PNG and GIF
encoders. Nothing outside this repository touched them. To regenerate:

```sh
make

# The animation at the top: a band of birds, its own name, then a murmuration.
./cbirds --record docs/demo.gif --record-size 120 --record-rows 32 \
         --record-every 4 --record-delay 5 --frames 400 \
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

`--record` needs no terminal at all. The `--snapshot` runs do, because they
photograph a live frame; run them in Kitty, WezTerm or Ghostty.

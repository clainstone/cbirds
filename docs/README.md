# The clips in the README

Every one of them was recorded by cbirds itself, with its own GIF encoder.
Nothing outside this repository touched them. To regenerate:

```sh
make

# Every clip opens with the three seconds in which the flock writes BOIDS, so
# eight seconds is five of flocking. Each flies slower than the tuned pace, at
# its own notch: --speed 0 is 0.2x, 1 is 0.4x (the default), 2 is 0.6x.

# The clip at the top: two hawks through an ice-blue flock, at a fifth of the
# pace and at 50 frames a second, the most a GIF can carry. 96x32 cells is
# 768x512 pixels, which GitHub shows at full size on a laptop.
./cbirds --record docs/demo.gif --record-fps 50 --record-seconds 8 \
         --record-size 96x32 -n 300 -s 14 --color ice --hawks 2 --speed 0 --seed 33

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

# Code that flies: docs/rules.c is the loop in flock_direction(), in boids.c,
# that sums the three rules, with the rest of it left out and its long lines
# broken so that it fits sixteen lines of 48 columns and none wraps; pygmentize
# colours it. A GIF of text is drawn in cells of 12 by 20 pixels, so 48x16 cells
# is 576x320. It opens on the code at rest. At seed 64 the wave starts in the
# middle of it and the last letter is home 21.5 seconds in, the soonest of 64
# seeds, so 23 seconds end on the code at rest again and the GIF loops without a
# jump. Ten frames a second, as for the letters further down.
pygmentize -f 16m -O style=monokai docs/rules.c | ./cbirds --record docs/code.gif \
         --record-fps 10 --record-seconds 23 --record-size 48x16 --seed 64

# The murmuration at dusk, beside the code: 72x20 cells is 576x320, the same
# size. No hawk, and the default 2000 birds. A space has no BOIDS to write, so
# it opens on the flock. Seed 96 of 120 that were recorded: the nearest birds
# pour out in a stream that sweeps round in front of the camera, and the flock
# folds back into one body much as it opened, so the loop barely jumps. Half the
# seeds are over 2.5 MB at this size; this one is 2.1.
./cbirds --record docs/dusk.gif --record-fps 25 --record-seconds 8 \
         --record-size 72x20 --3d --color dusk --seed 96

# Two signs, at the gallery's size and rate, each in a ramp the gallery had not
# shown: the colour runs along the text from one end of the ramp to the other,
# and the flock writes it from the first frame. A sign sizes its birds to its
# letters, so there is no -s. 400 birds write HELLO in strokes that read at half
# the width of the page; a clock with its seconds is eight characters, and takes
# 500. 12:34:56 reads the same on a twelve and a twenty-four hour clock, and four
# seconds in the minute changes while the seconds hop.
./cbirds --record docs/hello.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 400 --say "hello" --color prism --seed 5
./cbirds --record docs/seconds.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 500 --clock-at 12:34:56 --seconds --color aurora --seed 5

# The text ones are what a terminal shows by default: under
# --render braille or sextants the GIF is of the cells, not of the pixels.
# 80 columns is an honest text terminal; dots are cheap, so they get more
# frames a second.
./cbirds --record docs/braille.gif --record-fps 33 --record-seconds 8 \
         --record-size 80x22 -n 360 -s 14 --color ice --hawks 2 --render braille --speed 1 --seed 5
./cbirds --record docs/sextants.gif --record-fps 50 --record-seconds 7 \
         --record-size 80x22 -n 360 -s 12 --color acid --render sextants --speed 2 --seed 5

# Text as the flock: a screenful like the one fastfetch prints, from a file kept
# here so that the clip can be made again. It opens with the text at rest, a
# wave lifts the letters, they fly as a flock, and they land on the cells they
# left. A GIF of text is drawn in cells of 12 by 20 pixels, so 64x16 cells is
# 768x320, wider than the others, which is what it takes to read it. The
# letters move slowly, so ten frames a second will do.
cat docs/neofetch.txt | ./cbirds --record docs/letters.gif --record-fps 10 \
         --record-size 64x16 --seed 5

# A sign. The flock writes it from the first frame and holds it for thirty
# seconds at least, so eight seconds show the writing, the hovering and the
# flock wheeling round it. 400 birds are enough for ten letters on two lines.
./cbirds --record docs/say.gif --record-fps 20 --record-seconds 8 \
         --record-size 64x18 -n 400 --say "back in five" --seed 5

# The clock, started eight seconds before the next minute so that the change
# of minute is in the clip: at 8 seconds the two digits that change let go and
# other birds write the new ones, and the rest of the time does not move.
./cbirds --record docs/clock.gif --record-fps 20 --record-seconds 14 \
         --record-size 64x18 -n 300 --clock-at 10:09:52 --seed 5

# The murmuration, in ember and with a hawk. With no terminal to ask, ink falls
# back to ash and the birds are grey, so the colour is named. The size of the
# flock is the default 2000. Seeds differ a good deal in twelve seconds, which
# is a tenth of the camera's orbit: this is one of 48 that were recorded and
# looked at, chosen because the flock closes into a ring around the hawk
# between the third and the sixth second, and folds into arcs and sheets after
# it. Most of the seeds are over 3 MB at this size; this one is just under.
./cbirds --record docs/3d.gif --record-fps 25 --record-seconds 12 \
         --record-size 64x18 --3d --color ember --hawks 1 --seed 45

# And the asciinema cast, at the default pace, with the braille birds at the
# 60 pixels they were drawn at before 30 became the default everywhere.
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 -s 60 --color ember --hawks 2 --seed 5
```

The ceiling is 50 frames a second, and the reason is the format, not the
program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
treat anything under two hundredths as a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

A GIF frame costs about three bits per bird pixel, so the size of a clip is set
by how many birds are on the screen and how big they are, not by the
resolution. The clip at the top, 300 birds at fourteen pixels for eight
seconds at 50 frames a second, with two hawks, is 5.3 megabytes; the text clips are a fraction of
that, because dots compress. The cost is birds times sprite area times frames,
so a slower, smoother clip pays for its frames with birds, not with
resolution.

A recording whose name ends in `.cast` is an asciinema file instead: the flock
as the braille renderer sends it, one line of escape sequences per frame,
playable in any terminal with `asciinema play docs/demo.cast`.

`--record` needs no terminal at all. `--snapshot FILE` does, because it
photographs a live frame. The picture is what the terminal was showing, at its
size in pixels: sprites under Kitty, dots or blocks anywhere else.

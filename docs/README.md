# The clips in the README

Every one of them was recorded by cbirds itself, with its own GIF encoder.
Nothing outside this repository touched them. To regenerate:

```sh
make

# The flock clips open with the three seconds in which the flock writes BOIDS,
# so eight seconds is five of flocking. Each flies slower than the tuned pace,
# at its own notch: --speed 0 is 0.2x, 1 is 0.4x (the default), 2 is 0.6x.

# The clip at the top: two hawks through an ice-blue flock, at a fifth of the
# pace and at 50 frames a second, the most a GIF can carry. 96x32 cells is
# 768x512 pixels, which GitHub shows at full size on a laptop. Where a dive
# turns the birds in its way they light up gold: an escape wave.
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

# The text ones are what a terminal shows by default: under
# --render braille or sextants the GIF is of the cells, not of the pixels.
# 80 columns is an honest text terminal; dots are cheap, so they get more
# frames a second.
./cbirds --record docs/braille.gif --record-fps 33 --record-seconds 8 \
         --record-size 80x22 -n 360 -s 14 --color ice --hawks 2 --render braille --speed 1 --seed 5
./cbirds --record docs/sextants.gif --record-fps 50 --record-seconds 7 \
         --record-size 80x22 -n 360 -s 12 --color acid --render sextants --speed 2 --seed 5

# And the asciinema cast, at the default pace, with the braille birds at the
# 60 pixels they were drawn at before 30 became the default everywhere.
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 -s 60 --color ember --hawks 2 --seed 5
```

The clips of the features sit in the README's sections for them, not in the
gallery, whose cells are half width. They are recorded at the same 64x18 cells
as the gallery, at 10, 20 or 25 frames a second, three of the rates a GIF has
exactly. None of them opens with BOIDS: a sign and a picture write what they
were asked for from the first frame, a text opens as the text, and a night and
a space are not flocks that write.

```sh
# The murmuration, in ember and with a hawk: the 3D mode at its most striking.
# With no terminal to ask, ink falls back to ash and the birds are grey, so the
# colour is named. The size of the flock is the default 2000. Seeds differ a
# good deal in twelve seconds, which is a tenth of the camera's orbit: this is
# one of 48 that were recorded and looked at, chosen because the flock closes
# into a ring around the hawk between the third and the sixth second, and folds
# into arcs and sheets after it. Most of the seeds are over 3 MB at this size;
# this one is 2.9.
./cbirds --record docs/3d.gif --record-fps 25 --record-seconds 12 \
         --record-size 64x18 --3d --color ember --hawks 1 --seed 45

# Text taking flight. docs/neofetch.txt is a sample of what neofetch and
# fastfetch print, written by hand in plain bytes: colour escape sequences, and
# a cursor that goes up and across to put the information beside the logo, as
# those tools do. Nothing in it is a real machine; it is cbirds' own settings.
# A text records a whole cycle, 34 seconds, unless told otherwise: it rests, a
# wave lifts the letters, they fly as a flock, and they land on the cells they
# left. A GIF of text is drawn in cells of 12 by 20 pixels, so 64x16 cells is
# 768x320, wider than the others, which is what it takes to read it. The
# letters move slowly, so ten frames a second will do.
cat docs/neofetch.txt | ./cbirds --record docs/letters.gif --record-fps 10 \
         --record-size 64x16 --seed 5

# A sign. The flock writes it from the first frame and holds it for thirty
# seconds at least, so eight seconds show the writing, the hovering and the
# flock flying round it. 400 birds are enough for ten letters on two lines.
./cbirds --record docs/say.gif --record-fps 20 --record-seconds 8 \
         --record-size 64x18 -n 400 --say "back in five" --seed 5

# The clock, started eight seconds before the next minute so that the change
# of minute is in the clip: at 8 seconds the two digits that change let go and
# other birds write the new ones, and the rest of the time does not move.
./cbirds --record docs/clock.gif --record-fps 20 --record-seconds 14 \
         --record-size 64x18 -n 300 --clock-at 10:09:52 --seed 5

# A picture. docs/scene.png is a picture made for this clip: a sunset over pine
# trees, 320 by 180 pixels, generated and not taken from anywhere. The flock cuts
# it down to eight colours and every bird takes a place in it. A picture is held
# as long as a sign, thirty seconds at least, and the gathering takes a fifth of
# a second, so six seconds show it gather and then hold. 1500 birds are enough to
# read it at 64x18 cells. At the default size a dot stands alone and the six
# seconds cost 4.1 megabytes; at 16 pixels the dots touch and cost 2.8, and the
# pines and the sun read as well. At 20 the dots are a solid fill, 1.5, and no
# longer birds.
./cbirds --record docs/picture.gif --record-fps 25 --record-seconds 6 \
         --record-size 64x18 -n 1500 -s 16 --shape dot --picture docs/scene.png --seed 5

# The fireflies, from a random start to unison. A flash rises and dies in half
# a second, so twenty frames a second. With this seed and 200 fireflies on a
# screen this small the swarm is in step at about nine seconds, sooner than
# the twenty-four the README gives for the default 400 on 96x26 cells, and
# flashes together for the seven seconds after.
./cbirds --record docs/fireflies.gif --record-fps 20 --record-seconds 16 \
         --record-size 64x18 --fireflies -n 200 -s 12 --seed 6
```

There is no clip of `--link`. A shared sky is made of several windows, each a
terminal of its own, and `--record` is refused with it: a recording has one
window and no neighbour to cross to.

Run again, every command gives the same bytes. Built with clang instead of
gcc, every GIF here comes out the same except `3d.gif`, and `demo.cast` differs
too: their floating point is not the same to the last bit.

The ceiling is 50 frames a second, and the reason is the format, not the
program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
treat anything under two hundredths as a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending. Ask for 12 and you get 12.5, and it
says that too; 10, 20, 25 and 50 are exact.

A GIF frame costs about three bits per bird pixel, so the size of a clip is set
by how many birds are on the screen and how big they are, not by the
resolution. The clip at the top, 300 birds at fourteen pixels for eight
seconds at 50 frames a second, with two hawks, is 5.4 megabytes; the text clips
are a fraction of that, because dots compress. The cost is birds times sprite
area times frames, so a slower, smoother clip pays for its frames with birds,
not with resolution. The six clips of the features come to 13 megabytes
together, from 1.4 for the text to 2.9 for the murmuration; the clip at the top
and the gallery are 15.

A recording whose name ends in `.cast` is an asciinema file instead: the flock
as the braille renderer sends it, one line of escape sequences per frame,
playable in any terminal with `asciinema play docs/demo.cast`.

`--record` needs no terminal at all. `--snapshot FILE` does, because it
photographs a live frame. The picture is what the terminal was showing, at its
size in pixels: sprites under Kitty, dots or blocks anywhere else.

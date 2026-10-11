# The clips

Every clip in the README, and every clip that used to be in it, was recorded by
cbirds itself, with its own GIF encoder. Nothing outside this repository touched
them. Each is shown here above the command that made it, run from a clone after
`make`. Run again with the same build, every command gives the same bytes; a
cast also writes in its first line the time it was made.

The flock clips open with the three seconds in which the flock writes BOIDS, so
eight seconds is five of flocking. Each flies slower than the tuned pace, at its
own notch: `--speed 0` is 0.2x, 1 is 0.4x (the default), 2 is 0.6x. A sign, a
picture and a text open as themselves, from the first frame.

## The six at the top of the README

The sprite clips are recorded at 64x18 cells, 512x288 pixels, so that GitHub
shows the birds near the size they were drawn at. The six come to 10.6
megabytes together.

### Two hawks

<img src="hawks.gif" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, ice blue and in slow motion">

```sh
./cbirds --record docs/hawks.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 14 --color ice --hawks 2 --speed 0 --seed 5
```

### A picture

docs/scene.png is a picture made for this clip: a sunset over pine trees, 320 by
180 pixels, generated and not taken from anywhere. The flock cuts it down to
eight colours and every bird takes a place in it. A picture is held as long as a
sign, thirty seconds at least, and the gathering takes a fifth of a second, so
six seconds show it gather and then hold. 1500 birds are enough to read it at
64x18 cells. At the default size a dot stands alone and the six seconds cost 4.1
megabytes; at 16 pixels the dots touch and cost 2.8, and the pines and the sun
read as well. At 20 the dots are a solid fill, 1.5, and no longer birds.

<img src="picture.gif" alt="Fifteen hundred dots gather into a sunset: bands of orange, pink and purple sky, and an orange sun low over pine trees in two greens">

```sh
./cbirds --record docs/picture.gif --record-fps 25 --record-seconds 6 \
         --record-size 64x18 -n 1500 -s 16 --shape dot --picture docs/scene.png --seed 5
```

### Text as the flock

docs/neofetch.txt is a sample of what neofetch and fastfetch print, written by
hand in plain bytes: colour escape sequences, and a cursor that goes up and
across to put the information beside the logo, as those tools do. Nothing in it
is a real machine; it is cbirds' own settings. A text records a whole cycle, 34
seconds, unless told otherwise: it rests, a wave lifts the letters, they fly as
a flock, and they land on the cells they left. A GIF of text is drawn in cells
of 12 by 20 pixels, so 64x16 cells is 768x320, wider than the others, which is
what it takes to read it. The letters move slowly, so ten frames a second will
do.

<img src="letters.gif" alt="A bird drawn in letters, a list of settings beside it and rows of colour blocks: the letters take flight as a wave runs through them, fly as a flock, and land on the cells they left">

```sh
cat docs/neofetch.txt | ./cbirds --record docs/letters.gif --record-fps 10 \
         --record-size 64x16 --seed 5
```

### The clock

Started eight seconds before the next minute so that the change of minute is in
the clip: at 8 seconds the two digits that change let go and other birds write
the new ones, and the rest of the time does not move. Without `--color` a
recording has no terminal to take its colours from, and flies in ember.

<img src="clock.gif" alt="The flock writes 10:09, and a few seconds later the two digits that change let go and other birds write 10:10 while the rest of the time stays where it is">

```sh
./cbirds --record docs/clock.gif --record-fps 20 --record-seconds 14 \
         --record-size 64x18 -n 300 --clock-at 10:09:52 --seed 5
```

### Three flocks in prism

The flight of flocks.gif, below, seed and all, in prism, the ramp that goes
round the rainbow. Several flocks take their colours from the ends of the ramp
inwards, so in prism they are red, violet and green, further apart than ember's
gold, red and orange.

<img src="prism.gif" alt="Three flocks in red, green and violet write BOIDS together, let go, and cross one another, each keeping to its own kind">

```sh
./cbirds --record docs/prism.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 14 --color prism --flocks 3 --speed 1 --seed 3
```

### A murmuration in braille

What a terminal shows by default: under `--render braille` the GIF is of the
cells, not of the pixels. 80 columns is an honest text terminal. The
murmuration preset, in dusk; dots are cheap, and eight seconds at 25 frames a
second come to half a megabyte.

<img src="murmuration.gif" alt="A murmuration in braille dots, peach through to indigo, as a terminal without a graphics protocol shows it">

```sh
./cbirds --record docs/murmuration.gif --record-fps 25 --record-seconds 8 \
         --record-size 80x22 -n 360 -s 14 --color dusk --render braille --preset murmuration --speed 1 --seed 5
```

## More clips

### The clip that was at the top

Two hawks through an ice-blue flock, at a fifth of the pace and at 50 frames a
second, the most a GIF can carry. 96x32 cells is 768x512 pixels, which GitHub
shows at full size on a laptop.

<img src="demo.gif" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks in slow motion">

```sh
./cbirds --record docs/demo.gif --record-fps 50 --record-seconds 8 \
         --record-size 96x32 -n 300 -s 14 --color ice --hawks 2 --speed 0 --seed 33
```

### Three flocks in ember

<img src="flocks.gif" alt="Three flocks in gold, orange and red, each keeping to its own kind and crossing the others">

```sh
./cbirds --record docs/flocks.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 14 --color ember --flocks 3 --speed 1 --seed 3
```

### Matrix

<img src="matrix.gif" alt="Matrix rain, as green birds with tails falling through the screen">

```sh
./cbirds --record docs/matrix.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 200 -s 12 --matrix --speed 0 --seed 1
```

### Two skies

<img src="depth.gif" alt="Two skies: birds with tails in front, smaller and dimmer birds drifting slower behind them">

```sh
./cbirds --record docs/depth.gif --record-fps 25 --record-seconds 8 \
         --record-size 64x18 -n 160 -s 14 --color ice --depth --trails --speed 1 --seed 9
```

### Braille and sextants

The text ones are what a terminal shows: under `--render braille` or
`sextants` the GIF is of the cells, not of the pixels. 80 columns is an honest
text terminal; dots are cheap, so they get more frames a second.

<img src="braille.gif" alt="The flock and two hawks in braille dots, as a terminal without a graphics protocol shows it">

```sh
./cbirds --record docs/braille.gif --record-fps 33 --record-seconds 8 \
         --record-size 80x22 -n 360 -s 14 --color ice --hawks 2 --render braille --speed 1 --seed 5
```

<img src="sextants.gif" alt="The flock in sextant blocks, as a text terminal with a font from 2020 or later shows it">

```sh
./cbirds --record docs/sextants.gif --record-fps 50 --record-seconds 7 \
         --record-size 80x22 -n 360 -s 12 --color acid --render sextants --speed 2 --seed 5
```

### A sign

The flock writes it from the first frame and holds it for thirty seconds at
least, so eight seconds show the writing, the hovering and the flock wheeling
round it. 400 birds are enough for ten letters on two lines.

<img src="say.gif" alt="Birds gather into the words BACK IN FIVE on two lines, and hover in their places while the rest of the flock wheels round the sign">

```sh
./cbirds --record docs/say.gif --record-fps 20 --record-seconds 8 \
         --record-size 64x18 -n 400 --say "back in five" --seed 5
```

### The cast

The asciinema cast, at the default pace, with the braille birds at the 60
pixels they were drawn at before 30 became the default everywhere. It plays in
any terminal with `asciinema play docs/demo.cast`.

```sh
./cbirds --record docs/demo.cast --record-fps 30 --record-seconds 8 \
         --record-size 96x26 -n 700 -s 60 --color ember --hawks 2 --seed 5
```

There is no clip of `--link`. A shared sky is made of several windows, each a
terminal of its own, and `--record` is refused with it: a recording has one
window and no neighbour to cross to.

## Rates and sizes

The ceiling is 50 frames a second, and the reason is the format, not the
program: a GIF carries the delay between frames as whole hundredths of a
second, so the only rates it has are 100/1, 100/2, 100/3 and so on, and viewers
treat anything under two hundredths as a tenth. Ask for 60 and you get 50,
and cbirds says so rather than pretending.

A GIF frame costs about three bits per bird pixel, so the size of a clip is set
by how many birds are on the screen and how big they are, not by the
resolution. The clip that was at the top, 300 birds at fourteen pixels for
eight seconds at 50 frames a second, with two hawks, is 5.5 megabytes; the text
clips are a fraction of that, because dots compress. The cost is birds times
sprite area times frames, so a slower, smoother clip pays for its frames with
birds, not with resolution.

A recording whose name ends in `.cast` is an asciinema file instead: the flock
as the braille renderer sends it, one line of escape sequences per frame.
Eight seconds of the default flock, asked for at 30 frames a second, come to
2.0 MB as a cast, 8.3 MB as a GIF with sprites and 1.6 MB as a GIF in braille;
the GIFs are at 33.33 frames a second, the nearest rate a GIF has.

`--record` needs no terminal at all. `--snapshot FILE` does, because it
photographs a live frame. The picture is what the terminal was showing, at its
size in pixels: sprites under Kitty, dots or blocks anywhere else.

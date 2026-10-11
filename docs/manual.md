# cbirds in detail

The [README](../README.md) says what cbirds does. This is the rest of it: the
panel, text as the flock, signs, the clock and pictures, a sky shared between
terminals, and two options.

## The panel

`h` opens a panel of sliders in the corner, and `--panel` opens it from the
start. Lowercase lowers, uppercase raises, one press is one notch. `speed`
flies the same flock slower or faster, from a fifth of its pace to thirteen
fifths. With two flocks or more the panel grows one more row, `avoidance` on
`g`/`G`: at the bottom the flocks mix into one flock of two or three colours,
in the middle, where it starts, each keeps to its own kind and flies where it
likes, and at the top they keep well apart.

`--unlock-fps` removes the frame delay and renders as fast as the terminal accepts
frames. The simulation still advances in real time, so unlocking it does not make
the birds fly faster. It is useful for profiling; normal runs are capped at 60
fps.

## Anything can fly

Pipe text into cbirds and the text is the flock.

```
fastfetch | cbirds
figlet -f big hello | cbirds
ls --color=always -la | cbirds
git log --oneline --graph --color=always | cbirds
cbirds --text poem.txt
```

It opens with the text exactly as the command printed it, in its own colours,
laid out as a terminal would have laid it out: cbirds reads the escape
sequences, so a logo with its information beside it comes out as a logo with its
information beside it. A few seconds later one letter leaves and the ones near
it follow, a wave that crosses the screen in about a second, and the letters
fly as a flock. After twenty seconds or so they are called home, and every
letter lands on its own cell: the screen is what the command printed again, to
the cell. After a pause it happens again.

`Enter` sends the letters off, or calls them home at once. Move the pointer
through the text at rest and the letters it touches fly up, and they come back
when it has gone. `q` sends them off the top, as it does the birds.

A letter is drawn as itself, in its own colours, with its bold and underline,
wherever it is; one with no colour of its own takes the flock's while it flies.
A background colour stays where it was printed. Wide characters take two cells.
Hawks work, and are arrows over text. `--birds`, `--flocks`, `--depth`,
`--matrix`, `--shape`, `--sprite`, `--size` and tails do not apply: the text
decides how many letters there are, and a letter has no sprite. `--render
kitty` draws text too. The panel is laid over the text, and the letters under it
still land there.

When standard input is not a terminal, the keys are read from the terminal
itself, its controlling one, `/dev/tty`, on Linux and on macOS alike; with no
terminal at all, as from a service or an editor's run button, cbirds says that it
needs one and stops. A pipe is read to its end, or until it has been quiet for a second and a
half, or has gone on for eight seconds, or has sent a megabyte, and what is kept
is the last screenful, as a terminal would keep it; `tail -f log | cbirds` shows
what the log had when it went quiet. A pipe that sends nothing for three seconds,
and text with nothing to see in it, give the ordinary flock. If the window
changes size, the text is laid out again on the new one. A sign is what the flock
writes, and text is the flock, so `--say`, `--clock`, `--seconds` and `--picture`
stop with a usage error beside `--text` or beside text on a pipe.

Recordings take text too, and then run a whole cycle, 34 seconds, unless
`--record-seconds` says otherwise. A recording never starts a wave it has no time
to finish, so a whole cycle ends on the text as it was printed, and a GIF of it
loops without a jump:

```
fastfetch | cbirds --record fetch.cast
cbirds --text poem.txt --record poem.gif --record-fps 20
```

A GIF of text is drawn with a 5 by 7 font in cells of 12 by 20 pixels, so it
shows the letters rather than dots. `--snapshot` takes text, and `--bench` takes
`--text` and never a pipe it happens to be in.

## Signs, the clock and pictures

`--say TEXT` has the flock write TEXT where it would write BOIDS, and keep it
up. A sign is for reading, so it is held for thirty to forty-five seconds, and
then the flock lets go for eight to twelve, flies as a murmuration, and writes
it again. The birds that write do not stand still: each hovers round its place
in a small loop of its own, so the strokes shimmer and stay sharp. The colour
runs along the text, from one end of the ramp to the other.

The rest of the flock wheels round the sign, not through it: all of it the same
way, on an ellipse round the text, as one river that bunches and thins, and the
other way round the next time the sign is written. Each bird of it wears the
colour of its heading, so the sky round the text turns like a wheel.

The letters are a seventh of the window's rows tall: four rows at 80 by 24,
five at 120 by 34, seven at 200 by 50. `--font-size ROWS` sets them from 4 to
10 rows, on any window. Under four the text is lost in the river, and over ten
the birds, which are as wide as the cells of the letters, are as heavy as a
flock with nothing to write and the river is two bands above and below the
text. A sign never takes more than half the width of the screen and not quite
half its height, or half and half on a small terminal, so that the river has
room; a size that does not fit there is made as large as fits, on up to three
lines, and a long text that would be too small to read takes up to two thirds
by three fifths. On a small terminal more of the flock writes, so that the
birds that are left have sky to fly in.

Lower case is written in capitals, and an accented letter as its plain one, so
that città is CITTA and Straße is STRASSE; a symbol, or a letter of another
alphabet, has no plain letter and is left out. A text the flock has too few
birds to write is said so on stderr and left unwritten, and one that turns out
too big for the screen once the run has started is said so there when the run
ends, after the terminal is given back. A key does not end a sign. Move the
pointer through it and the birds it reaches scatter, and come back when it has
gone. A hawk does the same to the places it flies over, for less time: a fifth
of a second to two fifths, and only the places within a hawk's own width of its
path. A hawk is turned from the text as the flock is, if less firmly than it is
drawn to its prey, so it hunts round the sign with the river and crosses it when
a chase takes it there. With one hawk up, 800 birds and a screen of 96 by 26
cells, about 3% of the writers are scattered at any moment, and about 7% with
two; a clock stays readable.

`--clock` writes the time, HH:MM, in local time, and follows the locale for the
hour: twelve hours, with no AM or PM, if `LC_TIME` has a time format that shows
the hour on a twelve hour clock, and twenty four otherwise. The colon rises and
settles once a second. At each new minute the digits that change let go and
other birds write the new ones, while the rest of the time stays where it is,
so the clock can be read at any moment. On the hour the whole of it lets go for
a few seconds, and the flock writes the next time.

`--seconds` shows the seconds too, HH:MM:SS, and is a clock on its own, as
`--clock-at` is. A digit of the seconds is not let go: its own birds hop over to
the next one, which takes them a fraction of a second, so the seconds can be
read as they tick. The minutes and the hour change as they do without it.

`--picture FILE` has the flock draw a PNG instead. Every bird takes a place in
the opaque part of the picture, where a pixel with an alpha above half is ink,
the places spread evenly over it, and wears the picture's colour there. The
picture is cut down to at most eight colours, which are the palette of the
run; with `--color` given they are not, and the light and dark of the picture
pick shades of that ramp. A bird keeps its colour while it flies, and the
picture is held, let go of and drawn again as a sign is. `--shape dot` and more
birds, `-n 2000`, draw it best. The file is read as `--sprite` reads one: a PNG
of up to 4 MB, in any of the colour types PNG has.

A sign picks a bird as wide as the distance between the cells of its letters
unless you give `--size`, so larger letters are written by larger birds, up to
the usual thirty pixels, and the whole flock flies at that size. `--shape dot`
is the crispest, and more birds make thicker strokes.

A recording runs on its own clock, not the wall's, so the time a recorded
`--clock` tells is the local time at which the recording started, moved on by
its frames. `--clock-at 10:09:50` starts it from a time you choose instead,
which is how to record a change of minute. On its own it starts a clock.

## One sky, several terminals

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the
next, at the same height and in the same direction. Two terminals side by side,
or two panes of a split, become one sky with a flock flowing across the gap.
Hawks cross as well.

The windows lie in a row in the order they were started, so start them from
left to right: the second joins on the right of the first, the third on the
right of the second. Only an edge that has a window behind it is open. At the
ends of the row the outer edge is a wall, as it is alone. A window that closes
leaves the row, and the two beside it become neighbours.

Each window keeps its own settings, its own `--birds` among them, and its birds
come and go: a window can empty and fill again. A window that holds 4080 birds
says so, and its neighbours treat that edge as a wall until it has room; the
few already on their way when it said so wait at its door and come in as places
free, so no bird is lost between two windows. A bird sees only the birds of its
own window, so a flock does not look across the gap; it follows its leaders
across. While a window is writing BOIDS, or is paused, it has no room and takes
nobody in, and nobody leaves it.

A sign goes with it, `--say`, `--clock` or `--picture`. The birds that write
hold their places and never cross, and the rest of the flock crosses as it does
without one, but a window with a sign keeps the birds it takes to write it.
`--screensaver` goes with it too: any key quits that window, and it leaves the
sky.

The windows talk through Unix sockets in a directory that is yours alone:
`$XDG_RUNTIME_DIR/cbirds`, or `$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`.
cbirds refuses to use one that belongs to somebody else or that others can write
in, and says which, and it looks again every second: a directory that has been
removed is made again and the windows find each other in it, and one that has
been made into anything else leaves each window alone. The sockets are removed
when a window quits or is interrupted, and the one a killed window leaves behind
is swept away by the others.

A letter has its home in its own window, so `--link` with `--text` or with text
piped in is refused, and so is `--link` with `--record` or `--bench`: it joins
the windows that are open now, so it needs a live terminal. No clip of it is in
the README for the same reason: a recording has one window.

## Options

`cbirds --help` prints every option, grouped, with examples and the keys.
`--sprite` takes any PNG up to 4 MB: palette, grayscale, RGB or RGBA, at any bit
depth, interlaced or not. `--seed` is the same flock on every system: the random
numbers are cbirds' own, not the C library's.

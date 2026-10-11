# The new clips

Every clip here was recorded by cbirds itself, with `--record` and its own GIF
encoder, from a build that has `--3d`. Two small programs in [make/](make) finish
some of them, and neither draws anything the program did not:

- **framegif.c** puts a recording in a terminal window: a title bar with three
  lights and the command, rounded corners, on the dark of a GitHub page. Every
  frame is copied pixel for pixel; only the colour table is made again, from the
  first frame.
- **linkgif.c** makes [link.gif](link.gif) out of three real `cbirds --link` runs.
  A recording has one window, so the three windows were run side by side in
  terminals of their own, sharing one sky, and what each wrote to its terminal
  was kept with the time it was written ([make/capture_link.py](make/capture_link.py)).
  Each is then played back through cbirds' own terminal emulator, `vt.c`, frame by
  frame, and painted as the terminal would show it.

To make them all again, from the top of the repository, with a cbirds that has
`--3d`:

```sh
CBIRDS=/path/to/cbirds docs/clips/make/build.sh
```

It needs a C compiler, python3 and pygmentize. What it builds, and the recordings
before they are framed, go to `docs/clips/build`, which git ignores.

## Text piped in

Each runs a whole cycle of the text, 34 seconds: at rest as it was printed, a
wave lifts the letters, they fly as a flock, and every one lands on its own cell.
Eight frames a second, which is all a letter needs.

| clip | the text | made by |
|---|---|---|
| [pipe-donut.gif](pipe-donut.gif) | one frame of the spinning donut | [donut.c](donut.c), after Andy Sloane's donut.c (2006): `cc donut.c -lm -o donut && ./donut \| cbirds` |
| [pipe-code.gif](pipe-code.gif) | cbirds' own `hawk_vector`, from `boids.c` | `pygmentize -f terminal16m -O style=monokai -l c`, monokai's colours |
| [pipe-git.gif](pipe-git.gif) | the history of the `next` branch where four tracks were merged | `git log --graph --color=always` ([git.ans](git.ans), kept, since it needs that branch) |
| [pipe-fetch.gif](pipe-fetch.gif) | a screen in the manner of fastfetch: a swallow in braille and cbirds' settings | [make_inputs.py](make/make_inputs.py) → [fetch.ans](fetch.ans) |
| [pipe-banner.gif](pipe-banner.gif) | CBIRDS in a block font with a rainbow across it | [make_inputs.py](make/make_inputs.py) → [banner.ans](banner.ans) |
| [pipe-sunset.gif](pipe-sunset.gif) | a sunset drawn in full blocks, one colour a cell | [make_inputs.py](make/make_inputs.py) → [sunset.ans](sunset.ans) |

The fetch screen, the banner and the sunset were made for the clips; nothing in
them is a real machine. `fastfetch | cbirds` flies yours.

[logo.png](logo.png) is the banner at rest, the first frame of its recording.

## Three dimensions

| clip | command |
|---|---|
| [3d-prism.gif](3d-prism.gif) | `cbirds --3d --color prism --hawks 1 --seed 3`, 80x26 cells, 20 frames a second, 10 seconds |
| [3d-aurora.gif](3d-aurora.gif) | `cbirds --3d --color aurora --hawks 1 --seed 7`, the same |
| [3d-ember.gif](3d-ember.gif) | `cbirds --3d --color ember --hawks 1 --seed 45`, 64x18 cells, 25 frames a second, 12 seconds |
| [3d-braille.gif](3d-braille.gif) | `cbirds --3d --render braille --color dusk --hawks 1 --seed 5`, 80x22 cells, 25 a second, 10 seconds |
| [3d-ice.gif](3d-ice.gif), [3d-potion.gif](3d-potion.gif), [3d-ash.gif](3d-ash.gif), [3d-dusk.gif](3d-dusk.gif) | `cbirds --3d --color RAMP --hawks 1`, 48x14 cells, 20 a second, 8 seconds |

A recording has no terminal to ask for its colours, so ink, the default ramp of
`--3d`, records as ash: the clips name a ramp.

## One sky

[link.gif](link.gif) is three windows of 44 by 22 cells, each running
`cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1` with a seed
of its own, started left to right, twelve seconds of them at 20 frames a second.
They are live, so no two takes are the same.

## Weight

The fifteen GIFs here weigh about 38 MB together; the text clips are 2.3 to 4 MB
each, since a letter in flight is a letter that changed.

<p align="center">
<a href="#anything-can-fly">Anything can fly</a> &nbsp;&nbsp;&nbsp;
<a href="#three-dimensions">Three dimensions</a> &nbsp;&nbsp;&nbsp;
<a href="#one-sky">One sky</a> &nbsp;&nbsp;&nbsp;
<a href="#above-all-a-flock">The flock</a> &nbsp;&nbsp;&nbsp;
<a href="#install">Install</a> &nbsp;&nbsp;&nbsp;
<a href="../../docs/manual.md">Manual</a>
</p>

<table>
<tr>
<td width="47%">

# cbirds

**A flock of birds in your terminal.**

Craig Reynolds' boids, in braille in any terminal and as real sprites in Kitty
and Ghostty. Pipe text into it and every letter is a bird. `--3d` flies a
murmuration of two thousand; `--link` makes several terminals one sky.

```
brew install clainstone/tap/cbirds
```

Linux and macOS. One C99 program, no dependencies.
[apt and source](#install).

<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version">
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>

</td>
<td width="53%" align="center">

<img src="../../docs/clips/pipe-banner.gif" width="400" alt="CBIRDS in a big block font with a rainbow across it, a tagline and a command under it: the blocks scatter as rainbow confetti and rebuild the word">

<sub><code>cat docs/clips/banner.ans | cbirds</code>, a banner made for the clip</sub>

</td>
</tr>
</table>

<table>
<tr>
<td width="53%" align="center">

<img src="../../docs/clips/pipe-donut.gif" width="400" alt="A donut drawn in characters from warm red to gold breaks into a flock of characters, swirls, and every character lands back on its own cell">

<sub><code>./donut | cbirds</code>, with <code>docs/clips/donut.c</code>, after Andy Sloane's donut.c</sub>

</td>
<td width="47%">

## Anything can fly

Pipe text into cbirds and the text is the flock: every letter is a bird.

A few seconds in, one letter leaves, the ones near it follow, and the letters fly
as a flock. After twenty seconds or so they are called home, and every letter
lands on its own cell, exactly. Then it begins again.

```
./donut | cbirds
```

</td>
</tr>
</table>

<table>
<tr>
<td width="47%">

### Your fetch, in flight

It opens with the text exactly as the command printed it, in its own colours:
24-bit, 256 and the 16. The layout of fastfetch and neofetch, the logo beside the
information, is kept, because cbirds reads the output through a small terminal
emulator of its own.

```
fastfetch | cbirds
```

</td>
<td width="53%" align="center">

<img src="../../docs/clips/pipe-fetch.gif" width="400" alt="A fastfetch-style screen, a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks: the letters and the dots fly off and come back, the colour blocks stay">

<sub><code>cat docs/clips/fetch.ans | cbirds</code>, a screen in the manner of fastfetch; <code>fastfetch | cbirds</code> does the same with yours</sub>

</td>
</tr>
</table>

<table>
<tr>
<td width="53%" align="center">

<img src="../../docs/clips/pipe-code.gif" width="400" alt="cbirds' own C source, the function that makes the flock flee a hawk, syntax-highlighted in monokai colours, takes flight letter by letter and lands back">

<sub><code>pygmentize boids.c | cbirds</code>, cbirds reading its own code</sub>

</td>
<td width="47%">

### Your code

This is the function in cbirds that makes the flock flee a hawk, in monokai
colours, taking flight letter by letter and landing back. Hawks work on text
too: they fly over it as arrows.

```
pygmentize boids.c | cbirds
```

</td>
</tr>
</table>

<table>
<tr>
<td width="50%">

### Your history

`Enter` sends the letters off, or calls them home at once. The pointer lifts
the letters it touches.

```
git log --graph --color=always | cbirds
```

`cbirds --text FILE` reads a file, `--text -` standard input. A pipe with nothing
in it gives the ordinary flock.

</td>
<td width="50%" align="center">

<img src="../../docs/clips/pipe-git.gif" width="380" alt="A git log graph of cbirds' own history, three coloured lanes and yellow hashes, takes flight letter by letter and lands back">

<sub><code>git log --graph --color=always | cbirds</code>, cbirds' own history</sub>

</td>
</tr>
</table>

<table>
<tr>
<td width="53%" align="center">

<img src="../../docs/clips/pipe-sunset.gif" width="400" alt="A pixel-art sunset drawn in coloured full blocks, purple sky, sun, sea and its road of light, shatters into a cloud of coloured pixels that flies and reassembles the picture exactly">

<sub><code>cat docs/clips/sunset.ans | cbirds</code>, a picture drawn in █ characters</sub>

</td>
<td width="47%">

### Pictures made of characters

A picture drawn in characters is text like any other: it breaks into a cloud of
coloured pixels and lands back exactly as it was printed.

A recording of text runs a whole cycle, 34 seconds, and ends on the text as it
was printed, where it began.

```
cbirds --text poem.txt
```

</td>
</tr>
</table>

<table>
<tr>
<td width="47%">

## Three dimensions

`--3d` takes the flock off the plane and into a sky: a murmuration of 2000 birds
over its roost, seen from a camera that goes once round it every two minutes, so
the depth shows even in a still frame.

A nearer bird is bigger and brighter. A bird flying at the camera is short, one
flying across it long.

```
cbirds --3d --color prism
```

</td>
<td width="53%" align="center">

<img src="../../docs/clips/3d-prism.gif" width="400" alt="Two thousand birds in three dimensions, coloured by depth from red near the camera to blue far from it, fold into a ribbon that bends round a hawk">

<sub><code>cbirds --3d --color prism --hawks 1</code></sub>

</td>
</tr>
</table>

<table>
<tr>
<td width="53%" align="center">

<img src="../../docs/clips/3d-braille.gif" width="400" alt="The three-dimensional flock drawn in braille dots in dusk colours, pink, violet and peach, with a cyan hawk">

<sub><code>cbirds --3d --render braille --color dusk</code>, as a terminal with no graphics protocol shows it</sub>

</td>
<td width="47%">

### As starlings fly

The same three rules, but a bird heeds its seven nearest neighbours however far
off they are, as starlings do (Ballerini et al., PNAS 2008), rather than
everything within a radius. It turns at a limited rate a second, so it banks. A
roost calls it home when it strays, and the air above the roost moves slowly,
which folds the flock into sheets and ribbons (Hildenbrandt, Carere and
Hemelrijk, Behavioral Ecology 2010).

By default the flock is in **ink**, a ramp from your terminal's own text colour
to its background. Every renderer draws it, braille included.

</td>
</tr>
</table>

<table>
<tr>
<td width="36%">

## One sky

Run it in two terminals side by side, or two panes of a split. Birds that fly out
of the edge of one window fly in through the edge of the next, at the same height
and in the same direction; hawks cross too.

```
cbirds --link
```

Start the windows left to right.

</td>
<td width="64%" align="center">

<img src="../../docs/clips/link.gif" width="480" alt="Three terminal windows side by side write BOIDS, then a flock in prism colours flows out of one window and into the next, and the hawks cross too">

<sub>three real windows, <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code> in each, recorded through cbirds' own terminal emulator</sub>

</td>
</tr>
</table>

<table>
<tr>
<td width="53%" align="center">

<img src="../../docs/demo.gif" width="400" alt="Three hundred ice-blue birds part around two scarlet hawks and close again behind them, in slow motion">

<sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0 -n 300 -s 14</code></sub>

</td>
<td width="47%">

## Above all, a flock

It opens by writing BOIDS, lets go, and flocks: keep your distance, fly the way
your neighbours fly, drift towards their middle. Move the pointer into it and it
scatters. Hawks: the flock parts around them and closes behind.

```
cbirds --hawks 2 --color ice
```

`q`, and it flies off the top.

</td>
</tr>
</table>

<table>
<tr>
<td width="47%">

### A sign, a clock, a lock screen

`--say TEXT` has the flock write a sign and hold it. `--clock` tells the time,
and at each new minute only the digits that change let go. `--picture FILE`
draws a PNG. `--screensaver` quits at once on any key, so this is a lock screen
that tells the time:

```
cbirds --screensaver --clock
```

</td>
<td width="53%" align="center">

<img src="../../docs/clock.gif" width="400" alt="The flock writes 10:09, and at the new minute only the changing digits let go while other birds write 10:10">

<sub><code>cbirds --clock-at 10:09:52</code></sub>

</td>
</tr>
</table>

## Install

Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64,
from https://clainstone.com/apt :

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

Both come with completions for bash, zsh and fish, and `brew upgrade` or
`sudo apt upgrade` keeps them up to date. The `.deb` files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest).

From source, with a C compiler and make and nothing else:

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

That installs one file, `/usr/local/bin/cbirds`; `make install PREFIX="$HOME/.local"`
puts it in your home. `make test` runs the tests.

<details>
<summary>Uninstall</summary>

```
brew uninstall cbirds
brew untap clainstone/tap
```

```
sudo apt remove cbirds
sudo rm /etc/apt/sources.list.d/cbirds.list /etc/apt/keyrings/cbirds.gpg
sudo apt update
```

From source: `sudo make uninstall` in the clone.

</details>

## Options

| | |
|---|---|
| `-n`, `--birds` | 1 to 4096 birds, 800 by default |
| `-s`, `--size` | 4 to 64 pixels, 30 by default |
| `-g`, `--flocks` | 1 to 3 flocks that keep to their own kind |
| `-k`, `--hawks` | 0 to 4 predators |
| `--preset` | `murmuration`, `swarm` or `storm` |
| `-c`, `--color` | `theme` (your terminal's own colours), `ember`, `ice`, `acid`, `matrix`, `aurora`, `prism`, `potion`, `dusk`, `ash`, `ink` |
| `--shape` | `bird`, `arrow`, `plane` or `dot` |
| `--sprite FILE` | any PNG up to 4 MB, in its own colours |
| `--depth`, `-e`/`--trails`, `--matrix` | a second sky further off; faint tails; it is raining birds |
| `--seed N` | the same flock on every system |
| `--render` | `braille` (the default), `sextants`, `blocks`, `kitty` |
| `--text FILE`, `--text -` | a file, or standard input, as the flock |
| `--say TEXT`, `--font-size ROWS` | a sign, its letters 4 to 10 rows tall |
| `--clock`, `--seconds`, `--clock-at HH:MM[:SS]` | the time, in local time |
| `--picture FILE` | a PNG drawn by the flock, in at most eight of its colours |
| `--3d`, `--link`, `--screensaver`, `--panel` | three dimensions; one sky; quit on any key; the sliders from the start |
| `--record FILE`, `--snapshot FILE`, `--bench N` | a GIF or an asciinema cast; a PNG of the last live frame; the numbers |

The [manual](../../docs/manual.md) has the details: the panel, how a pipe is
read, signs and the clock, and the sky shared between terminals.

## Keys

| key | | key | |
|---|---|---|---|
| `b`/`B` `s`/`S` `a`/`A` `t`/`T` `p`/`P` `v`/`V` `g`/`G` | a slider one notch down / up | `h` | the panel |
| `Space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+` `-` | more birds, fewer | `k` `K` | a hawk more, one fewer |
| `e` | tails | `Enter` | letters off, or home |
| `q` | quit | | |

The panel's sliders are boundary, separation, alignment, turning, perception and
speed (0.2x to 2.6x), each 0 to 12 notches; lowercase lowers, uppercase raises.
In `--3d`, `boundary` becomes `roost` and `perception` becomes `neighbours`,
1 to 13. Left alone for a minute, the flock starts moving the sliders itself, a
notch every four seconds; any key ends that.

## Terminals

Braille by default, in every terminal; no terminal is guessed at.
`--render sextants` and `--render blocks` are bolder text versions (sextants need
a font from 2020 or later; blocks work everywhere). Only the cells that changed
are sent, and the background is never painted, so the flock wears your theme.

`--render kitty` draws real sprites over the Kitty graphics protocol, made for
Kitty and Ghostty. WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol
and then draw too few birds, the wrong ones or none; inside tmux the sprites never
reach the terminal.

With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000
about three; braille, the default, about three and ten, since every frame is
painted and sampled down to dots. In 3D, 2000 birds cost about 3 ms a frame in
braille and 1.7 ms with Kitty sprites, measured on one desktop machine.
`cbirds --bench 300` prints the numbers on yours.

## Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --3d --record murmuration.gif --color prism
```

`--record` needs no terminal: it runs headless and writes the GIF with its own
encoder. A name ending in `.cast` gives an asciinema recording.
`--record-size COLSxROWS` (40x14 to 400x120, default 96x26), `--record-fps` (a GIF
carries up to 50), `--record-seconds` (default 6, 34 for text). Every clip on this
page was recorded by cbirds itself.

## Credits

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*,
  SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/)
- Ballerini et al., *Interaction ruling animal collective behavior depends on
  topological rather than metric distance: Evidence from a field study*, PNAS 2008:
  the seven nearest neighbours.
- Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of
  thousands of starlings: a model*, Behavioral Ecology 2010: the roost and the
  sheets and ribbons.
- The [Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/)
- The donut in the donut clip is after Andy Sloane's donut.c (2006).

Listed in [Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## License

MIT, by Alessandro Palliccia. See [LICENSE](../../LICENSE).

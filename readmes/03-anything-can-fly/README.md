<h1 align="center">Anything can fly</h1>

<p align="center"><b>cbirds</b> &nbsp;·&nbsp; a flock of birds in your terminal</p>

<p align="center">
<img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds. A donut drawn in the characters .,-~:;=!*#$@, warm red in the shade and gold in the light, breaks apart into a flock of characters that swirls across the window; then every character lands back on its own cell and the donut is whole again">
</p>

<p align="center">
<code>./donut | cbirds</code><br>
<sub>one frame of the spinning donut, the real output of <a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c (2006)</sub>
</p>

<p align="center">
Pipe text into cbirds and the text is the flock: every letter is a bird.<br>
It opens with the text exactly as the command printed it, in its own colours.<br>
A few seconds later one letter leaves, the ones near it follow, and the letters fly as a flock.<br>
After twenty seconds or so they are called home, and every letter lands on its own cell, exactly.<br>
Then it begins again.
</p>

<p align="center">
<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<a href="https://github.com/clainstone/cbirds/tags"><img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version"></a>
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux and macOS">
</p>

## The gallery

The donut is one of six. Every clip here is one whole cycle, 34 seconds,
recorded by cbirds itself: it starts and ends on the text as it was printed.

### `pygmentize boids.c | cbirds`

<p align="center">
<img src="../../docs/clips/pipe-code.gif" width="830" alt="A terminal window titled pygmentize boids.c | cbirds. C source in monokai colours, the function that makes the flock flee a hawk, comes apart letter by letter into a flock of characters, flies across the window, and lands back as the same code">
</p>

<p align="center">cbirds reading its own code: the function that makes the flock flee a hawk,<br>syntax-highlighted, takes flight letter by letter and lands back.</p>

### `cat docs/clips/banner.ans | cbirds`

<p align="center">
<img src="../../docs/clips/pipe-banner.gif" width="796" alt="A terminal window titled cat banner.ans | cbirds. CBIRDS in a big block font with a rainbow across it, a flock of birds in your terminal and $ fastfetch | cbirds under it; the blocks scatter as rainbow confetti, fly, and rebuild the word">
</p>

<p align="center">Not only letters: a banner made for the clip, in a block font with a rainbow across it.<br>The blocks scatter as rainbow confetti and rebuild the word.</p>

### `cat docs/clips/fetch.ans | cbirds`

<p align="center">
<img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A terminal window titled cat fetch.ans | cbirds. A swallow drawn in braille, cyan to pink, beside the lines flock@sky, Birds: 2000, Mode: --3d, over a roost, Camera: once round in 2 min, Neighbours: 7 nearest, Ramp: ink, Render: braille, Hawks: 1 and Seed: 45, above two rows of colour blocks. The letters and the bird's dots fly off as a flock and land back; the colour blocks stay">
</p>

<p align="center">A screen in the manner of fastfetch, made for the clip: the logo beside the information.<br>
cbirds reads the output through a small terminal emulator of its own, so that layout is kept.<br>
<code>fastfetch | cbirds</code> does the same with yours.</p>

### `cat docs/clips/sunset.ans | cbirds`

<p align="center">
<img src="../../docs/clips/pipe-sunset.gif" width="796" alt="A terminal window titled cat sunset.ans | cbirds. A pixel-art sunset in coloured full blocks, a purple sky, the sun, the sea and its road of light, shatters into a cloud of coloured pixels that flies and puts the picture back together exactly">
</p>

<p align="center">A picture drawn in █ characters, made for the clip:<br>it shatters into a cloud of coloured pixels, flies, and puts itself back exactly.</p>

### `git log --graph --color=always | cbirds`

<p align="center">
<img src="../../docs/clips/pipe-git.gif" width="796" alt="A terminal window titled git log --graph | cbirds. cbirds' own history, graph lanes in colour beside yellow commit hashes and their messages, takes flight letter by letter, flies as a flock and lands back line for line">
</p>

<p align="center">cbirds' own history, at the point where four feature tracks were merged:<br>three coloured lanes, yellow hashes, and every line of it flies.</p>

## How a screen flies

| At rest | Lift-off | Flight | Home |
|:--|:--|:--|:--|
| The text, exactly as the command printed it, in its own colours: 24-bit, 256 and the 16 colours. | A few seconds later one letter leaves, and the ones near it follow. | The letters fly as a flock. Hawks work, and fly over the text as arrows. | After twenty seconds or so they are called home, and every letter lands on its own cell, exactly. Then it begins again. |

While it flies:

| key or pointer | does |
|:--|:--|
| <kbd>Enter</kbd> | sends the letters off, or calls them home at once |
| the pointer | lifts the letters it touches |
| <kbd>k</kbd> <kbd>K</kbd> | a hawk more, one fewer |
| <kbd>q</kbd> | quit |

## Pipe your own

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

fastfetch's and neofetch's layout, with the logo beside the information, is
kept, because cbirds reads the output through a small terminal emulator of its
own. `--text FILE` reads a file and `--text -` reads standard input. A pipe with
nothing in it gives the ordinary flock.

The texts behind the clips are in the repository, so a clone can fly them as the
clips do, `cat docs/clips/sunset.ans | cbirds`:
[fetch.ans](../../docs/clips/fetch.ans), [banner.ans](../../docs/clips/banner.ans),
[sunset.ans](../../docs/clips/sunset.ans), and the donut's own
[donut.c](../../docs/clips/donut.c).

## Record your own

```
fastfetch | cbirds --record fetch.gif
```

`--record` needs no terminal: it runs headless and writes the GIF with its own
encoder, and a name ending in `.cast` gives an asciinema recording instead. A
recording of text runs a whole cycle, 34 seconds, unless `--record-seconds` says
otherwise, and ends on the text as it was printed. The six clips above were
recorded that way, by cbirds itself, and set in a drawn window whose title bar
names the command. The [manual](../../docs/manual.md#anything-can-fly)
has the rest: how a pipe is read, what goes with text, and how to record it.

## And it is also a flock

Without a pipe, cbirds is Craig Reynolds' boids, drawn in braille in any
terminal, and as real sprites over the Kitty graphics protocol in Kitty and
Ghostty.

<table>
<tr>
<td width="50%"><img src="../../docs/clips/3d-prism.gif" width="380" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in three dimensions, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk"></td>
<td width="50%"><img src="../../docs/hawks.gif" width="380" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, in ice blue and in slow motion"></td>
</tr>
<tr>
<td><b>Three dimensions.</b> <code>cbirds --3d</code> is a murmuration of 2000 birds over its roost, seen from a camera that goes once round it every two minutes. A bird heeds its seven nearest neighbours however far off they are, as starlings do. By default it wears <code>ink</code>, your terminal's own text colour.<br><sub><code>cbirds --3d --color prism --hawks 1</code></sub></td>
<td><b>The flock.</b> <code>cbirds</code> writes BOIDS, lets go, and flocks. Move the pointer into it and it scatters. Up to four hawks part it, and it closes behind them.<br><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
</tr>
</table>

<p align="center">
<img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each running cbirds --link: each writes BOIDS, then a prism-coloured flock drawn in sextant blocks flows out of one window and into the next, and hawks cross between them">
<br>
<sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub>
</p>

**One sky.** Run `cbirds --link` in two terminals side by side, or in two panes of
a split, and they become one sky: birds that fly out of the edge of one window
fly in through the edge of the next, at the same height and in the same
direction, and hawks cross too. Start them from left to right. Text stays at
home: `--link` is refused with text.

<table>
<tr>
<td width="33%"><img src="../../docs/say.gif" width="245" alt="The flock writes BACK IN FIVE on two lines and hovers on it while the rest of the flock wheels round the sign"></td>
<td width="33%"><img src="../../docs/clock.gif" width="245" alt="The flock writes 10:09; when the minute turns, only the last two digits let go and other birds write 10:10"></td>
<td width="33%"><img src="../../docs/picture.gif" width="245" alt="Fifteen hundred dots gather into a sunset over pine trees"></td>
</tr>
<tr>
<td align="center"><code>cbirds --say "back in five"</code></td>
<td align="center"><code>cbirds --clock-at 10:09:52</code></td>
<td align="center"><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></td>
</tr>
</table>

**Signs.** `--say TEXT` has the flock write TEXT and hold it as a sign for thirty
to forty-five seconds while the rest of the flock wheels round it; then it lets
go and writes it again. `--clock` writes the time, and at each new minute only
the digits that change let go. `--picture FILE` draws a PNG. More in the
[manual](../../docs/manual.md#signs-the-clock-and-pictures).

## Install

Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

<details>
<summary>apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64</summary>

From [clainstone.com/apt](https://clainstone.com/apt):

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

</details>

<details>
<summary>From source, with a C compiler and make and nothing else</summary>

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

That installs one file, `/usr/local/bin/cbirds`; `make install PREFIX="$HOME/.local"`
puts it in your home. `make test` runs the tests.

</details>

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

Both packages come with completions for bash, zsh and fish, and `brew upgrade` or
`sudo apt upgrade` keeps them up to date. The `.deb` files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest). One
program in C99, with no dependencies: the PNG, GIF and DEFLATE code is all in
the repository, and there is no zlib, no libpng, no ncurses.

## Use

```
cbirds                              the flock, in braille, in any terminal
cbirds --render kitty               real sprites, in Kitty and Ghostty
cbirds --3d                         a murmuration of 2000 birds, in three dimensions
cbirds --link                       one sky: run it in two terminals, the left one first
cbirds --preset murmuration         a preset; swarm and storm are the others
cbirds --hawks 2 --color ice        two hawks, in the ice ramp
cbirds --flocks 3 --color prism     three flocks that keep to their own kind
cbirds --depth --trails             a second sky further off, and faint tails
cbirds --matrix                     it is raining birds
cbirds --say "back in five"         the flock writes it, and holds it as a sign
cbirds --clock --seconds            the time, to the second
cbirds --picture photo.png          the flock draws a PNG
cbirds --screensaver --clock        a lock screen that tells the time; any key quits
```

It opens by writing BOIDS, lets go, and flocks. Move the pointer into the flock
and it scatters; `q` and it flies off the top. Left alone for a minute, it starts
moving the sliders itself, a notch every four seconds, and any key ends that.
`h` opens the panel, `--panel` from the start: sliders for boundary,
separation, alignment, turning, perception and speed (0.2x to 2.6x), each 0 to
12 notches; lowercase lowers, uppercase raises.

| key | does |
|:--|:--|
| <kbd>b</kbd>/<kbd>B</kbd> <kbd>s</kbd>/<kbd>S</kbd> <kbd>a</kbd>/<kbd>A</kbd> <kbd>t</kbd>/<kbd>T</kbd> <kbd>p</kbd>/<kbd>P</kbd> <kbd>v</kbd>/<kbd>V</kbd> <kbd>g</kbd>/<kbd>G</kbd> | a slider one notch down / up |
| <kbd>h</kbd> | the panel |
| <kbd>Space</kbd> | pause |
| <kbd>.</kbd> | one frame |
| <kbd>0</kbd> | back to the defaults |
| <kbd>Tab</kbd> | next preset |
| <kbd>+</kbd> <kbd>-</kbd> | more birds, fewer |
| <kbd>k</kbd> <kbd>K</kbd> | a hawk more, one fewer |
| <kbd>e</kbd> | tails |
| <kbd>Enter</kbd> | letters off, or home (with text) |
| <kbd>q</kbd> | quit |

**Terminals.** Braille is the default, in every terminal; no terminal is guessed
at. `--render sextants` and `--render blocks` are bolder text versions: sextants
need a font from 2020 or later, blocks work everywhere. Only the cells that
changed are sent, and the background is never painted, so the flock wears your
theme. `--render kitty` draws real sprites over the Kitty graphics protocol, made
for Kitty and Ghostty; WezTerm, Konsole, iTerm2, Warp and Rio answer for the
protocol and then draw too few birds, the wrong ones or none, and inside tmux
the sprites never reach the terminal.

The panel, text, signs and the shared sky in detail: [the manual](../../docs/manual.md).

## Credits

Much of cbirds' code, tests and documentation was written by large language
models (Claude, by Anthropic), under my direction.

- The flock is Craig Reynolds' boids: *Flocks, Herds, and Schools: A Distributed
  Behavioral Model*, SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/).
- Seven nearest neighbours in `--3d`: Ballerini et al., *Interaction ruling animal
  collective behavior depends on topological rather than metric distance:
  Evidence from a field study*, PNAS 2008.
- The roost and the slow air above it: Hildenbrandt, Carere and Hemelrijk,
  *Self-organized aerial displays of thousands of starlings: a model*,
  Behavioral Ecology 2010.
- [The Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/).
- The donut in the donut clip is after Andy Sloane's donut.c (2006).

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT, by Alessandro Palliccia ([clainstone](https://github.com/clainstone)). See [LICENSE](../../LICENSE).

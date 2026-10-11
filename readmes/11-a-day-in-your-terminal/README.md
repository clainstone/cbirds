<h1 align="center">A day in your terminal</h1>

<p align="center">
<b>cbirds</b> is a flock of birds in your terminal.<br>
Here is a day with one in it, from the first fetch of the morning to the lock screen at night.
</p>

<p align="center">
<a href="#morning">Morning</a> &nbsp;·&nbsp;
<a href="#work">Work</a> &nbsp;·&nbsp;
<a href="#lunch">Lunch</a> &nbsp;·&nbsp;
<a href="#afternoon">Afternoon</a> &nbsp;·&nbsp;
<a href="#evening">Evening</a> &nbsp;·&nbsp;
<a href="#night">Night</a> &nbsp;·&nbsp;
<a href="#keep-it">Keep it</a>
</p>

<p align="center">
<img src="../../docs/clips/pipe-banner.gif" width="796" alt="A terminal window titled cat banner.ans | cbirds. CBIRDS in a big block font with a rainbow across it, a flock of birds in your terminal and $ fastfetch | cbirds under it; the blocks scatter as rainbow confetti, fly, and rebuild the word">
<br>
<sub><code>cat docs/clips/banner.ans | cbirds</code>, a banner made for the clip</sub>
</p>

## Morning

New terminal, first coffee. Ask the machine how it is, and let the answer fly:

```
fastfetch | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A terminal window titled cat fetch.ans | cbirds. A swallow drawn in braille, cyan to pink, beside the lines flock@sky, Birds: 2000, Mode: --3d, over a roost, Camera: once round in 2 min, Neighbours: 7 nearest, Ramp: ink, Render: braille, Hawks: 1 and Seed: 45, above two rows of colour blocks. The letters and the bird's dots fly off as a flock and land back; the colour blocks stay">
<br>
<sub>The clip flies a screen made for it in the manner of fastfetch, <code>cat docs/clips/fetch.ans | cbirds</code>; <code>fastfetch | cbirds</code> does the same with yours.</sub>
</p>

It opens exactly as fastfetch printed it: the logo beside the information, in its
own colours, because cbirds reads the output through a small terminal emulator
of its own. A few seconds later one letter leaves, the ones near it follow, and
the letters fly as a flock. After twenty seconds or so they are called home, and
every one lands on its own cell, exactly. Then it begins again, all morning if
you let it.

## Work

Work is history and code, and both can fly. The log first:

```
git log --graph --color=always | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-git.gif" width="796" alt="A terminal window titled git log --graph | cbirds. cbirds' own history, graph lanes in colour beside yellow commit hashes and their messages, takes flight letter by letter, flies as a flock and lands back line for line">
<br>
<sub>cbirds' own history, where four feature tracks were merged: three coloured lanes, yellow hashes.</sub>
</p>

Then the code. Here cbirds reads its own, the function that makes the flock flee
a hawk:

```
pygmentize boids.c | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-code.gif" width="830" alt="A terminal window titled pygmentize boids.c | cbirds. C source in monokai colours comes apart letter by letter into a flock of characters, flies across the window, and lands back as the same code">
</p>

Somebody walks over to look? <kbd>Enter</kbd> calls the letters home at once,
and the next <kbd>Enter</kbd> sends them off again.

## Lunch

Lunch is a donut.

```
./donut | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds. A donut drawn in the characters .,-~:;=!*#$@, warm red in the shade and gold in the light, breaks apart into a flock of characters that swirls across the window; then every character lands back on its own cell and the donut is whole again">
<br>
<sub>One frame of the spinning donut, printed by <a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c (2006).</sub>
</p>

Run the pointer through it and the letters it touches lift off. Press <kbd>k</kbd>
for a hawk: hawks work on text too, and fly over it as arrows.

## Afternoon

Split the window. Run `cbirds --link` in each pane, the left one first, and the
two panes are one sky:

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

<p align="center">
<img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each running cbirds --link: each writes BOIDS, then a prism-coloured flock drawn in sextant blocks flows out of one window and into the next, and hawks cross between them">
<br>
<sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub>
</p>

Birds that fly out of the edge of one window fly in through the edge of the
next, at the same height and in the same direction, and hawks cross too. The
windows lie in a row in the order they were started; only an edge with a window
behind it is open, and the outer edges are walls. Each window keeps its own
settings: its own number of birds, its colours, its renderer.

Stepping out for a moment? Leave a note that flies:

```
cbirds --say "back in five"
```

<p align="center">
<img src="../../docs/say.gif" width="512" alt="The flock writes BACK IN FIVE on two lines and hovers on it while the rest of the flock wheels round the sign">
</p>

The flock writes it and holds it as a sign for thirty to forty-five seconds,
while the rest of the flock wheels round it; then it lets go and writes it
again. A key does not end a sign, and birds scattered by the pointer come back.

## Evening

The sun goes down in block characters:

```
cat docs/clips/sunset.ans | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-sunset.gif" width="796" alt="A terminal window titled cat sunset.ans | cbirds. A pixel-art sunset in coloured full blocks, a purple sky, the sun, the sea and its road of light, shatters into a cloud of coloured pixels that flies and puts the picture back together exactly">
<br>
<sub>A picture drawn in █ characters, made for the clip. It flies apart and lands back exactly.</sub>
</p>

Then the murmuration. Leave the plane for a sky:

```
cbirds --3d
```

<p align="center">
<img src="../../docs/clips/3d-prism.gif" width="668" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in three dimensions, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk">
<br>
<sub><code>cbirds --3d --color prism --hawks 1</code></sub>
</p>

Two thousand birds over their roost, seen from a camera that goes once round it
every two minutes, so the depth shows even in a still frame: a nearer bird is
bigger and brighter, and a bird flying at the camera is short, one flying across
it long. Each bird heeds its seven nearest neighbours however far off they are,
as starlings do, and turns at a limited rate, so it banks. The roost calls it
home when it strays, and the slow air above the roost folds the flock into
sheets and ribbons. A hawk hunts it through the air (`--hawks`), and the pointer
is a stick poked into the sky: birds near the line from the camera through it
get out of the way.

By default the flock wears **ink**, a ramp from your terminal's own text colour
to its background: grey on a dark terminal, near black on a light one. Any other
ramp is one option away, and a terminal with no graphics protocol gets the same
sky in braille:

<table>
<tr>
<td width="50%"><img src="../../docs/clips/3d-ember.gif" width="380" alt="An ember murmuration, gold to deep red, closes into a ring round a cyan hawk, then folds into arcs and sheets"></td>
<td width="50%"><img src="../../docs/clips/3d-braille.gif" width="380" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration in braille dots, pink, violet and peach, with a cyan hawk"></td>
</tr>
<tr>
<td align="center"><code>cbirds --3d --color ember --hawks 1</code></td>
<td align="center"><code>cbirds --3d --render braille --color dusk</code></td>
</tr>
</table>

## Night

Last thing: lock the screen, and let the flock keep the time.

```
cbirds --screensaver --clock
```

As the lock screen of tmux:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

Or of zsh:

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

<p align="center">
<img src="../../docs/clock.gif" width="512" alt="The flock writes 10:09; when the minute turns, only the last two digits let go and other birds write 10:10">
<br>
<sub><code>cbirds --clock-at 10:09:52</code>, started just before a minute turns</sub>
</p>

The flock writes the time, HH:MM, in local time, and `--seconds` adds the
seconds. At each new minute only the digits that change let go and are written
again. Any key, click or pointer movement quits at once, with no flight out;
input in the first half second is ignored. Good night.

## Keep it

One line, on macOS and Linux:

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

Both packages come with completions for bash, zsh and fish, and `brew upgrade`
or `sudo apt upgrade` keeps them up to date; the `.deb` files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest). It is one
program in C99, with no dependencies, for Linux and macOS: the PNG, GIF and
DEFLATE code is all in the repository, and there is no zlib, no libpng, no
ncurses.

<p>
<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<a href="https://github.com/clainstone/cbirds/tags"><img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version"></a>
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux and macOS">
</p>

## Any time of day

Without a pipe, cbirds is Craig Reynolds' boids: each bird keeps its distance,
flies the way its neighbours fly and drifts towards their middle. It opens by
writing BOIDS, lets go, and flocks. Move the pointer into the flock and it
scatters; `q` and it flies off the top. Left alone for a minute, it starts moving
the sliders itself, a notch every four seconds, and any key ends that.

```
cbirds                              the flock, in braille, in any terminal
cbirds --render kitty               real sprites, in Kitty and Ghostty
cbirds --preset murmuration         a preset; swarm and storm are the others
cbirds --hawks 2 --color ice        two hawks, in the ice ramp
cbirds --flocks 3 --color prism     three flocks that keep to their own kind
cbirds --depth --trails             a second sky further off, and faint tails
cbirds --matrix                     it is raining birds
cbirds --clock --seconds            the time, to the second
cbirds --picture photo.png          the flock draws a PNG
cbirds --text poem.txt              a file, flown like a pipe
```

`h` opens the panel, and `--panel` opens it from the start: sliders for
boundary, separation, alignment, turning, perception and speed (0.2x to 2.6x),
each 0 to 12 notches; lowercase lowers, uppercase raises.

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

**Your terminal.** Braille is the default, in every terminal; no terminal is
guessed at. `--render sextants` and `--render blocks` are bolder text versions:
sextants need a font from 2020 or later, blocks work everywhere. Only the cells
that changed are sent, and the background is never painted, so the flock wears
your theme. `--render kitty` draws real sprites over the Kitty graphics
protocol, made for Kitty and Ghostty; WezTerm, Konsole, iTerm2, Warp and Rio
answer for the protocol and then draw too few birds, the wrong ones or none, and
inside tmux the sprites never reach the terminal.

**Your own clips.** `--record` needs no terminal: it runs headless and writes the
GIF with its own encoder, and a name ending in `.cast` gives an asciinema
recording. A recording of text runs a whole cycle, 34 seconds, and ends on the
text as it was printed. Every clip in this day was recorded by cbirds itself.

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --3d --record murmuration.gif --color prism
fastfetch | cbirds --record fetch.gif
```

The rest, from the panel to the shared sky, is in [the manual](../../docs/manual.md).

## Credits

Much of cbirds' code, tests and documentation was written by large language
models (Claude, by Anthropic), under my direction.

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*,
  SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/).
- The seven nearest neighbours of the evening sky: Ballerini et al., *Interaction
  ruling animal collective behavior depends on topological rather than metric
  distance: Evidence from a field study*, PNAS 2008.
- Its roost and its slow air: Hildenbrandt, Carere and Hemelrijk,
  *Self-organized aerial displays of thousands of starlings: a model*,
  Behavioral Ecology 2010.
- [The Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/).
- Lunch is after Andy Sloane's donut.c (2006).

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT, by Alessandro Palliccia ([clainstone](https://github.com/clainstone)). See [LICENSE](../../LICENSE).

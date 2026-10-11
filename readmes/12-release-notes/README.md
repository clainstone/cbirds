# cbirds

**A flock of birds in your terminal.** This release takes the flock into three dimensions and
across your terminals, gives it your terminal's own ink, and comes with new clips of text piped
in, flying in its own colours.

```diff
+ --3d          a murmuration of 2000 birds over its roost, seen by a circling camera
+ --link        one sky across several terminals: birds fly from one window into the next
+ --color ink   your terminal's own text colour, fading into its background
+ new clips     text piped in, in its own colours: a donut, cbirds' own code, a fetch
+               screen, a rainbow banner, a pixel-art sunset, git log
```

[Three dimensions](#new-three-dimensions) &emsp;
[One sky across terminals](#new-one-sky-across-terminals) &emsp;
[Ink](#new-ink) &emsp;
[Anything can fly, in colour](#newly-filmed-anything-can-fly-in-colour) &emsp;
[Everything else](#everything-else) &emsp;
[Install or upgrade](#install-or-upgrade)

## New: three dimensions

<p align="center"><img src="../../docs/clips/3d-prism.gif" width="668" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in rainbow colours, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk"></p>

<p align="center"><code>cbirds --3d --color prism --hawks 1</code></p>

```
cbirds --3d                    2000 birds in ink, seen from a slow orbit
cbirds --3d --hawks 1          a murmuration, and a hawk through it
cbirds --3d --color prism      any ramp
cbirds --3d --render kitty     sprites, in Kitty or Ghostty
cbirds --3d --record murmuration.gif --color prism
```

- **A sky, not a plane.** The flock leaves the plane for a sky: a murmuration of 2000 birds
  over its roost, seen from a camera that goes once round it every two minutes, so the depth
  shows even in a still frame. A nearer bird is bigger and brighter; a bird flying at the
  camera is short, one flying across it long.
- **Seven neighbours.** The same three rules, but a bird heeds its seven nearest neighbours
  however far off they are, as starlings do (Ballerini et al., PNAS 2008), rather than
  everything within a radius. It turns at a limited rate a second, so it banks.
- **A roost, and moving air.** A roost calls a bird home when it strays, and the air above the
  roost moves slowly, which folds the flock into sheets and ribbons (Hildenbrandt, Carere and
  Hemelrijk, Behavioral Ecology 2010).
- **Hawks in the air.** Hawks hunt the flock through the air (`--hawks`, `k`/`K`). The pointer
  is a stick poked into the sky: birds near the line from the camera through it get out of
  the way.
- **The panel.** `boundary` becomes `roost`, and `perception` becomes `neighbours` (1 to 13).
- **Every renderer.** Braille, sextants, blocks, Kitty sprites. `--screensaver` goes with it,
  and so do the keys and recording.
- **Defaults.** 2000 birds, of 8 pixels (5 in the text renderers), in [ink](#new-ink). Any
  other ramp can be asked for.
- **Left out.** `--3d` replaces `--depth` and draws no tails. It is one flock: `--flocks` and
  `--matrix` are refused, and so are a text, a sign and `--link`.
- **Cost.** 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites,
  measured on one desktop machine.

<table>
<tr>
<td width="33%" align="center"><img src="../../docs/clips/3d-ice.gif" width="256" alt="An ice-blue murmuration, and a hawk"><br><sub><code>--color ice</code></sub></td>
<td width="33%" align="center"><img src="../../docs/clips/3d-potion.gif" width="256" alt="A murmuration in violet and green, and a red hawk"><br><sub><code>--color potion</code></sub></td>
<td width="33%" align="center"><img src="../../docs/clips/3d-dusk.gif" width="256" alt="A murmuration from peach to violet, and a cyan hawk"><br><sub><code>--color dusk</code></sub></td>
</tr>
</table>

<sub>Each is <code>cbirds --3d --color RAMP --hawks 1</code>.</sub>

Every renderer draws the sky. In braille, in a terminal with no graphics protocol at all:

<p align="center"><img src="../../docs/clips/3d-braille.gif" width="668" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration drawn in braille dots, pink, violet and peach, thick where the flock is dense, and a cyan hawk"></p>

<p align="center"><code>cbirds --3d --render braille --color dusk</code></p>

## New: one sky across terminals

<p align="center"><img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each titled cbirds --link --color prism: each writes BOIDS in coloured blocks, then the birds stream out of one window and in through the edge of the next, and hawks cross between them"></p>

<p align="center"><sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub></p>

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

- **Through the edge.** Birds that fly out of the edge of one window fly in through the edge
  of the next, at the same height and in the same direction; hawks cross too. Two terminals
  side by side, or two panes of a split, become one sky.
- **In a row.** The windows lie in a row in the order they were started: start them left to
  right. Only an edge with a window behind it is open; the outer edges are walls.
- **Each its own.** Each window keeps its own settings (its own `--birds`, colours,
  renderer). A sign goes with it; so does `--screensaver`.
- **Yours alone.** The windows talk through Unix sockets in a directory that is yours alone
  (`$XDG_RUNTIME_DIR/cbirds`, or `$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`).
- **What it refuses.** `--link` is refused with `--text`, text piped in, `--3d`, `--record`
  and `--bench`.

## New: ink

<p align="center"><img src="../../docs/clips/3d-ash.gif" width="384" alt="A murmuration in greys, as ink looks on a dark terminal, and a red hawk"></p>

<p align="center"><sub><code>cbirds --3d --color ash --hawks 1</code>: ash, which is how ink looks on a dark terminal</sub></p>

`ink` is a ramp from your terminal's own text colour to its background, asked of the terminal
at startup, so the flock is grey on a dark terminal and near black on a light one. It is the
default in 3D, and `--color ink` works in the flat sky as well. A terminal that does not
answer, or whose text is the colour of its ground, and every recording, get `ash`.

## Newly filmed: anything can fly, in colour

Pipe text into cbirds and the text is the flock: every letter is a bird. The six clips below
are new, each a recording by cbirds of the text named under it, in that text's own colours.

<p align="center"><img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds: a donut drawn in the characters .,-~:;=!*#$@, warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its own cell"></p>

<p align="center"><code>./donut | cbirds</code><br><sub>one frame of a spinning donut, <a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c</sub></p>

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clips/pipe-banner.gif" width="410" alt="A terminal window titled cat banner.ans | cbirds: the word CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti, flies as a flock and builds the word again"><br><sub><code>cat docs/clips/banner.ans | cbirds</code><br>a banner made for the clip</sub></td>
<td width="50%" align="center"><img src="../../docs/clips/pipe-sunset.gif" width="410" alt="A terminal window titled cat sunset.ans | cbirds: a pixel-art sunset in coloured blocks, a purple sky, a pale sun, a dark sea with a road of light, shatters into a cloud of coloured pixels that flies and puts the picture back exactly"><br><sub><code>cat docs/clips/sunset.ans | cbirds</code><br>a picture drawn in █ characters</sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-fetch.gif" width="410" alt="A terminal window titled cat fetch.ans | cbirds: a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks; the letters and the swallow's dots fly while the colour blocks stay"><br><sub><code>cat docs/clips/fetch.ans | cbirds</code><br>a screen in the manner of fastfetch; <code>fastfetch | cbirds</code> does the same with yours</sub></td>
<td align="center"><img src="../../docs/clips/pipe-git.gif" width="410" alt="A terminal window titled git log --graph | cbirds: the commit graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of characters and lands back"><br><sub><code>git log --graph --color=always | cbirds</code><br>cbirds' own history, where four feature tracks were merged</sub></td>
</tr>
</table>

<p align="center"><img src="../../docs/clips/pipe-code.gif" width="830" alt="A terminal window titled pygmentize boids.c | cbirds: the C function hawk_vector, syntax-highlighted, takes flight letter by letter, flies as a flock of characters and lands back as code"></p>

<p align="center"><code>pygmentize boids.c | cbirds</code><br><sub>cbirds reading its own code: the function that makes the flock flee a hawk</sub></p>

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

- **As printed.** It opens with the text exactly as the command printed it, in its own colours
  (24-bit, 256 and the 16 colours). fastfetch's and neofetch's layout, with the logo beside the
  information, is kept, because cbirds reads the output through a small terminal emulator of
  its own.
- **Away and home.** A few seconds later one letter leaves, the ones near it follow, and the
  letters fly as a flock. After twenty seconds or so they are called home, and every letter
  lands on its own cell, exactly. Then it begins again.
- **Hands on.** `Enter` sends the letters off, or calls them home at once. The pointer lifts the
  letters it touches. Hawks work, and fly over text as arrows.
- **Files too.** `--text FILE` reads a file; `--text -` reads standard input. A pipe with
  nothing in it gives the ordinary flock.
- **On record.** A recording of text runs a whole cycle, 34 seconds, unless told otherwise, and
  ends on the text as it was printed.
- **What the clips are.** The fetch screen, the banner and the sunset are texts made for the
  clips; the donut, the code and the git log are the real output of the commands named.

## Everything else

<table>
<tr>
<td width="33%" align="center"><img src="../../docs/hawks.gif" width="256" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, ice blue"><br><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
<td width="33%" align="center"><img src="../../docs/prism.gif" width="256" alt="Three flocks in red, green and violet write BOIDS together, let go, and cross one another, each keeping to its own kind"><br><sub><code>cbirds --render kitty --flocks 3 --color prism</code></sub></td>
<td width="33%" align="center"><img src="../../docs/matrix.gif" width="256" alt="Green birds with tails falling through the screen like rain"><br><sub><code>cbirds --matrix</code></sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/say.gif" width="256" alt="Ember birds write BACK IN FIVE on two lines and hover in their places while the rest of the flock wheels round the sign"><br><sub><code>cbirds --say "back in five"</code></sub></td>
<td align="center"><img src="../../docs/clock.gif" width="256" alt="Ember birds write 10:09; at the new minute the two digits that change let go and other birds write 10:10"><br><sub><code>cbirds --clock-at 10:09:52</code></sub></td>
<td align="center"><img src="../../docs/picture.gif" width="256" alt="Fifteen hundred dots gather into a sunset over pine trees"><br><sub><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub></td>
</tr>
</table>

| | command | what it does |
|---|---|---|
| **The flock** | `cbirds` | Reynolds' boids, in braille, in any terminal: it writes BOIDS, lets go, and flocks; the pointer scatters it; `q` and it flies off the top |
| **Rules** | | keep your distance, fly the way your neighbours fly, drift towards their middle, with a nudge away from the edges; turning is limited per frame, which gives curved fronts instead of a cloud snapping into shape |
| **Left alone** | | after a minute it starts moving the sliders itself, a notch every four seconds; any key ends that |
| **Birds** | `-n 2000` | 1 to 4096 birds (default 800) |
| **Size** | `-s 14` | 4 to 64 pixels (default 30) |
| **Flocks** | `-g 3` | 1 to 3 flocks that keep to their own kind |
| **Hawks** | `-k 2` | 0 to 4 predators: the flock parts around them and closes behind |
| **Presets** | `--preset storm` | murmuration, swarm, storm |
| **Seed** | `--seed 5` | the same flock on every system (cbirds' own random numbers) |
| **Colour** | `-c aurora` | theme (the terminal's own colours, asked of the terminal), ember, ice, acid, matrix, aurora, prism, potion, dusk, ash, ink |
| **Shape** | `--shape plane` | bird, arrow, plane, dot |
| **Sprite** | `--sprite bird.png` | any PNG up to 4 MB, kept in its own colours |
| **Depth** | `--depth` | a second sky further off: smaller, slower, dimmer birds |
| **Tails** | `-e` | faint tails |
| **Rain** | `--matrix` | it is raining birds: green, falling, with tails |
| **Panel** | `--panel`, `h` | sliders boundary, separation, alignment, turning, perception, speed (0.2x to 2.6x), each 0 to 12 notches; lowercase lowers, uppercase raises |
| **Text** | `fastfetch \| cbirds` | the text is the flock; `--text FILE` reads a file, `--text -` standard input |
| **Sign** | `--say "back in five"` | the flock writes it and holds it for thirty to forty-five seconds, lets go, and writes it again; `--font-size` 4 to 10 rows |
| **Clock** | `--clock --seconds` | the time, HH:MM, local, with the seconds if asked; at each new minute only the digits that change let go |
| **Clock at** | `--clock-at 10:09:52` | the clock, started at a time you choose |
| **Picture** | `--picture scene.png` | the flock draws a PNG, in at most eight of its colours, or in a ramp if `--color` is given; `--shape dot -n 2000` draw it best |
| **Lock screen** | `--screensaver` | quits at once on any key, click or pointer movement, with no flight out; input in the first half second is ignored |
| **Braille** | `--render braille` | the default, in every terminal; no terminal is guessed at |
| **Sextants, blocks** | `--render sextants` | bolder text versions: sextants need a font from 2020 or later, blocks work everywhere |
| **Sprites** | `--render kitty` | real sprites over the Kitty graphics protocol, made for Kitty and Ghostty |
| **Record** | `--record flock.gif` | needs no terminal: runs headless and writes the GIF with its own encoder; a `.cast` name gives an asciinema recording |
| **Shot** | `--record-size 96x26` | 40x14 to 400x120 cells; `--record-fps` (a GIF carries up to 50), `--record-seconds` (default 6, 34 for text) |
| **Snapshot** | `--snapshot frame.png` | the last live frame as a PNG |
| **Bench** | `--bench 300` | prints the numbers on your machine |

| key | does | key | does |
|---|---|---|---|
| `b`/`B` `s`/`S` `a`/`A` `t`/`T` `p`/`P` `v`/`V` `g`/`G` | a slider one notch down / up | `h` | the panel |
| `Space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+` `-` | more birds, fewer | `k` `K` | a hawk more, one fewer |
| `e` | tails | `Enter` | letters off, or home (with text) |
| `q` | quit | | |

**Terminals.** Only the cells that changed are sent, and the background is never painted, so
the flock wears your theme. WezTerm, Konsole, iTerm2, Warp and Rio answer for the Kitty
graphics protocol and then draw too few birds, the wrong ones or none; inside tmux the sprites
never reach the terminal. Braille works in all of them.

**The lock screen**, in tmux and in zsh:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

**Numbers.** With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000
about three; braille, the default, about three and ten, since every frame is painted and
sampled down to dots. `cbirds --bench 300` prints the numbers on your machine. At startup the
bird is rotated into sixty headings, three wing positions and every shade of the ramp: about
fifteen hundred small images. The wings beat six times a second, and now and then a bird
glides.

**Made of.** One program in C99, no dependencies: the PNG, GIF and DEFLATE code is all in the
repository; there is no zlib, no libpng, no ncurses. Linux and macOS. Every clip in this
repository was recorded by cbirds itself. The [manual](../../docs/manual.md) has the rest.

## Install or upgrade

Already have it? `brew upgrade` or `sudo apt upgrade` keeps it up to date.

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

That installs one file, `/usr/local/bin/cbirds`; `make install PREFIX="$HOME/.local"` puts it
in your home. `make test` runs the tests.

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

Homebrew and apt both come with completions for bash, zsh and fish. The `.deb` files alone are
on the [release page](https://github.com/clainstone/cbirds/releases/latest).

## Credits

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*, SIGGRAPH
  1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/)
- Ballerini et al., *Interaction ruling animal collective behavior depends on topological
  rather than metric distance: Evidence from a field study*, PNAS 2008.
- Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of thousands of
  starlings: a model*, Behavioral Ecology 2010.
- The Kitty graphics protocol:
  [sw.kovidgoyal.net/kitty/graphics-protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/)
- The donut in the donut clip is after Andy Sloane's donut.c (2006).

cbirds is listed in [Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT. See [LICENSE](../../LICENSE). By Alessandro Palliccia ([clainstone](https://github.com/clainstone)).

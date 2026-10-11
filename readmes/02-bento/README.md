# cbirds

A flock of birds in your terminal. Pipe anything into it and the letters take flight.

<table>
<tr>
<td colspan="2" rowspan="2" align="center"><img src="../../docs/clips/3d-prism.gif" width="366" alt="Two thousand birds in three dimensions, coloured by depth from red near the camera to blue far from it, fold into a ribbon that bends round a hawk"><br><sub><code>cbirds --3d --color prism --hawks 1</code><br>two thousand birds, red near, blue far</sub></td>
<td align="center"><img src="../../docs/clips/pipe-fetch.gif" width="170" alt="A fastfetch-style screen, a swallow drawn in braille beside a list of settings and two rows of colour blocks: the letters and the dots fly off and come back, the colour blocks stay"><br><sub><code>cat fetch.ans | cbirds</code><br>a fastfetch-style screen</sub></td>
<td align="center"><img src="../../docs/clips/pipe-sunset.gif" width="170" alt="A pixel-art sunset drawn in coloured full blocks shatters into a cloud of coloured pixels, flies, and reassembles the picture exactly"><br><sub><code>cat sunset.ans | cbirds</code><br>a sunset drawn in █</sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-banner.gif" width="170" alt="CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti and rebuilds the word"><br><sub><code>cat banner.ans | cbirds</code><br>a banner made for the clip</sub></td>
<td align="center"><img src="../../docs/clips/pipe-git.gif" width="170" alt="A git log graph of cbirds' own history, three coloured lanes and yellow hashes, takes flight and lands back"><br><sub><code>git log --graph | cbirds</code><br>cbirds' own history</sub></td>
</tr>
<tr>
<td colspan="2" align="center"><img src="../../docs/clips/pipe-donut.gif" width="366" alt="A donut drawn in characters from warm red to gold breaks into a flock of characters, swirls, and every character lands back on its own cell"><br><sub><code>./donut | cbirds</code>, after Andy Sloane's donut.c</sub></td>
<td colspan="2" align="center"><img src="../../docs/clips/pipe-code.gif" width="366" alt="cbirds' own C source, syntax-highlighted in monokai colours, takes flight letter by letter and lands back"><br><sub><code>pygmentize boids.c | cbirds</code>, cbirds reading its own code</sub></td>
</tr>
<tr>
<td colspan="4" align="center"><img src="../../docs/clips/link.gif" width="758" alt="Three terminal windows side by side write BOIDS, then a flock in prism colours flows out of one window and into the next, and the hawks cross too"><br><sub><code>cbirds --link</code> in three real terminals: a bird that leaves one window flies into the next</sub></td>
</tr>
<tr>
<td colspan="2" align="center"><img src="../../docs/hawks.gif" width="366" alt="The flock writes BOIDS in ice blue, lets go, and parts around two scarlet hawks"><br><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
<td colspan="2" align="center"><img src="../../docs/clips/3d-ember.gif" width="366" alt="An ember murmuration closes into a ring round a cyan hawk, then folds into arcs and sheets"><br><sub><code>cbirds --3d --color ember --hawks 1</code></sub></td>
</tr>
</table>

<p>
<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version">
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux and macOS">
<img src="https://img.shields.io/badge/sprites-Kitty%20%7C%20Ghostty-8A2BE2" alt="sprites in Kitty and Ghostty">
</p>

Craig Reynolds' boids, drawn in braille in any terminal and as real sprites over
the Kitty graphics protocol in Kitty and Ghostty. One program in C99 with no
dependencies, for Linux and macOS. Every clip on this page was recorded by
cbirds itself.

```
brew install clainstone/tap/cbirds
```

apt and source: [Install](#install).

## Anything can fly

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

Pipe text in and the text is the flock: every letter is a bird. It opens with
the text exactly as the command printed it, in its own colours (24-bit, 256 and
the 16), and fastfetch's and neofetch's layout, the logo beside the information,
is kept, because cbirds reads the output through a small terminal emulator of
its own. A few seconds later one letter leaves, the ones near it follow, and the
letters fly as a flock; after twenty seconds or so they are called home, and
every letter lands on its own cell, exactly. Then it begins again.

`Enter` sends the letters off, or calls them home at once. The pointer lifts the
letters it touches. Hawks work, and fly over text as arrows. `--text FILE` reads
a file, `--text -` standard input, and a pipe with nothing in it gives the
ordinary flock.

## Three dimensions

`--3d` takes the flock off the plane and into a sky: a murmuration of 2000 birds
over its roost, seen from a camera that goes once round it every two minutes, so
the depth shows even in a still frame. A nearer bird is bigger and brighter; a
bird flying at the camera is short, one flying across it long.

The three rules stay, but a bird heeds its seven nearest neighbours however far
off they are, as starlings do (Ballerini et al., PNAS 2008), rather than
everything within a radius. It turns at a limited rate a second, so it banks. A
roost calls it home when it strays, and the air above the roost moves slowly,
which folds the flock into sheets and ribbons (Hildenbrandt, Carere and
Hemelrijk, Behavioral Ecology 2010).

The default ramp is **ink**, from your terminal's own text colour to its
background, asked of the terminal at startup: grey on a dark terminal, near black
on a light one. A terminal that does not answer, and every recording, get `ash`.
Any other ramp can be asked for:

<table>
<tr>
<td align="center"><img src="../../docs/clips/3d-ice.gif" width="170" alt="A small ice-blue murmuration with a hawk"><br><sub><code>--color ice</code></sub></td>
<td align="center"><img src="../../docs/clips/3d-potion.gif" width="170" alt="A small murmuration in violet and green"><br><sub><code>--color potion</code></sub></td>
<td align="center"><img src="../../docs/clips/3d-ash.gif" width="170" alt="A small murmuration in greys, the way ink looks on a dark terminal"><br><sub><code>--color ash</code>, as ink looks on a dark terminal</sub></td>
<td align="center"><img src="../../docs/clips/3d-dusk.gif" width="170" alt="A small murmuration from peach to violet with a cyan hawk"><br><sub><code>--color dusk</code></sub></td>
</tr>
</table>

Hawks hunt the flock through the air. The pointer is a stick poked into the sky:
birds near the line from the camera through it get out of the way. In the panel
`boundary` becomes `roost` and `perception` becomes `neighbours`, 1 to 13. Every
renderer draws it, braille, sextants, blocks and Kitty sprites, and
`--screensaver`, the keys and recording go with it. It replaces `--depth` and
draws no tails, and it is one flock: `--flocks`, `--matrix`, a text, a sign and
`--link` are refused.

## One sky

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the next,
at the same height and in the same direction; hawks cross too. Two terminals side
by side, or two panes of a split, become one sky. The windows lie in a row in the
order they were started, so start them left to right; only an edge with a window
behind it is open, and the outer edges are walls. Each window keeps its own
settings (its own `--birds`, colours, renderer), and a sign goes with it, as does
`--screensaver`. The windows talk through Unix sockets in a directory that is
yours alone. `--link` is refused with `--text`, text piped in, `--3d`, `--record`
and `--bench`.

## The flock

It opens by writing BOIDS, lets go, and flocks. Move the pointer into the flock
and it scatters; `q`, and it flies off the top. Each bird keeps its distance,
flies the way its neighbours fly and drifts towards their middle, with a nudge
away from the edges, and its turning is limited per frame, which gives curved
fronts instead of a cloud snapping into shape. Left alone for a minute it starts
moving the sliders itself, a notch every four seconds; any key ends that.

| option | |
|---|---|
| `-n`, `--birds` | 1 to 4096 birds, 800 by default |
| `-s`, `--size` | 4 to 64 pixels, 30 by default |
| `-g`, `--flocks` | 1 to 3 flocks that keep to their own kind |
| `-k`, `--hawks` | 0 to 4 predators: the flock parts around them and closes behind |
| `--preset` | `murmuration`, `swarm` or `storm` |
| `-c`, `--color` | `theme` (your terminal's own colours), `ember`, `ice`, `acid`, `matrix`, `aurora`, `prism`, `potion`, `dusk`, `ash`, `ink` |
| `--shape` | `bird`, `arrow`, `plane` or `dot` |
| `--sprite FILE` | any PNG up to 4 MB, kept in its own colours |
| `--depth` | a second sky further off: smaller, slower, dimmer birds |
| `-e`, `--trails` | faint tails |
| `--matrix` | it is raining birds |
| `--seed N` | the same flock on every system |
| `--panel` | the panel of sliders from the start |

## Signs, the clock, pictures

<table>
<tr>
<td align="center"><img src="../../docs/say.gif" width="240" alt="The flock writes BACK IN FIVE and hovers on it while the rest of the flock wheels round"><br><sub><code>cbirds --say "back in five"</code></sub></td>
<td align="center"><img src="../../docs/clock.gif" width="240" alt="The flock writes 10:09, and at the new minute only the changing digits let go while other birds write 10:10"><br><sub><code>cbirds --clock-at 10:09:52</code></sub></td>
<td align="center"><img src="../../docs/picture.gif" width="240" alt="Fifteen hundred dots gather into a sunset over pine trees"><br><sub><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub></td>
</tr>
</table>

`--say TEXT` has the flock write TEXT and hold it as a sign for thirty to
forty-five seconds, let go, and write it again, while the rest of the flock wheels
round it; `--font-size ROWS` sets the letters from 4 to 10 rows. `--clock` writes
the time, HH:MM, in local time, and `--seconds` adds the seconds; at each new
minute only the digits that change let go and are written again, and
`--clock-at HH:MM[:SS]` starts it at a time you choose. `--picture FILE` draws a
PNG in at most eight of its colours, or in a ramp if `--color` is given;
`--shape dot -n 2000` draw it best. A key does not end a sign, and birds
scattered by the pointer or a hawk come back.

`--screensaver` quits at once on any key, click or pointer movement, with no
flight out (input in the first half second is ignored), and goes with everything
else, so `cbirds --screensaver --clock` is a lock screen that tells the time.

<details>
<summary>tmux and zsh</summary>

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

</details>

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
speed (0.2x to 2.6x), each 0 to 12 notches. More in the
[manual](../../docs/manual.md#the-panel).

## Terminals

Braille by default, in every terminal; no terminal is guessed at.
`--render sextants` and `--render blocks` are bolder text versions (sextants need
a font from 2020 or later; blocks work everywhere). Only the cells that changed
are sent, and the background is never painted, so the flock wears your theme.

`--render kitty` draws real sprites over the Kitty graphics protocol, made for
Kitty and Ghostty. WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol
and then draw too few birds, the wrong ones or none; inside tmux the sprites never
reach the terminal.

| CPU a frame | 800 birds | 4000 birds |
|---|---|---|
| sprites, `--render kitty` | about 0.5 ms | about 3 ms |
| braille, the default | about 3 ms | about 10 ms |

Braille costs more because every frame is painted and sampled down to dots. In
3D, 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites,
measured on one desktop machine. `cbirds --bench 300` prints the numbers on
yours.

## Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

`--record` needs no terminal: it runs headless and writes the GIF with its own
encoder, sprites or, with `--render braille | sextants | blocks`, the cells. A name
ending in `.cast` gives an asciinema recording. `--record-size COLSxROWS` (40x14
to 400x120, default 96x26), `--record-fps` (a GIF carries up to 50) and
`--record-seconds` (6 by default, 34 for text, a whole cycle that ends on the
text as it was printed). `--snapshot FILE` writes the last live frame as a PNG.

## Install

Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

<details>
<summary>apt, on Debian 12, Ubuntu 22.04 and later and their derivatives</summary>

On amd64 or arm64, from https://clainstone.com/apt :

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

Homebrew and apt come with completions for bash, zsh and fish, and
`brew upgrade` or `sudo apt upgrade` keeps them up to date. The `.deb` files
alone are on the [release page](https://github.com/clainstone/cbirds/releases/latest).

## Credits

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*,
  SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/)
- Ballerini et al., *Interaction ruling animal collective behavior depends on
  topological rather than metric distance: Evidence from a field study*, PNAS 2008
- Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of
  thousands of starlings: a model*, Behavioral Ecology 2010
- The [Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/)
- The donut in the donut clip is after Andy Sloane's donut.c (2006).

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## License

MIT, by Alessandro Palliccia. See [LICENSE](../../LICENSE).

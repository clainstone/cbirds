# cbirds

A flock of birds in your terminal, and the keys to play it with.

<table>
<tr>
<td align="center"><sub>1</sub></td>
<td align="center"><sub>2</sub></td>
<td align="center"><sub>3</sub></td>
<td align="center"><sub>4</sub></td>
<td align="center"><sub>5</sub></td>
<td align="center"><sub>6</sub></td>
<td align="center"><sub>7</sub></td>
<td align="center"><sub>8</sub></td>
<td align="center"><sub>9</sub></td>
<td align="center"><kbd>0</kbd><br><sub>defaults</sub></td>
<td align="center"><kbd>-</kbd><br><sub>fewer</sub></td>
<td align="center"><kbd>+</kbd><br><sub>more</sub></td>
</tr>
<tr>
<td align="center"><kbd>Tab</kbd><br><sub>preset</sub></td>
<td align="center"><kbd>q</kbd><br><sub>quit</sub></td>
<td align="center"><sub>w</sub></td>
<td align="center"><kbd>e</kbd><br><sub>tails</sub></td>
<td align="center"><sub>r</sub></td>
<td align="center"><kbd>t</kbd><br><sub>slider</sub></td>
<td align="center"><sub>y</sub></td>
<td align="center"><sub>u</sub></td>
<td align="center"><sub>i</sub></td>
<td align="center"><sub>o</sub></td>
<td align="center"><kbd>p</kbd><br><sub>slider</sub></td>
<td align="center"><sub>[</sub></td>
</tr>
<tr>
<td align="center"><sub>caps</sub></td>
<td align="center"><kbd>a</kbd><br><sub>slider</sub></td>
<td align="center"><kbd>s</kbd><br><sub>slider</sub></td>
<td align="center"><sub>d</sub></td>
<td align="center"><sub>f</sub></td>
<td align="center"><kbd>g</kbd><br><sub>slider</sub></td>
<td align="center"><kbd>h</kbd><br><sub>panel</sub></td>
<td align="center"><sub>j</sub></td>
<td align="center"><kbd>k</kbd><br><sub>hawk</sub></td>
<td align="center"><sub>l</sub></td>
<td align="center"><sub>;</sub></td>
<td align="center"><kbd>Enter</kbd><br><sub>letters</sub></td>
</tr>
<tr>
<td align="center"><kbd>Shift</kbd><br><sub>upper case</sub></td>
<td align="center"><sub>z</sub></td>
<td align="center"><sub>x</sub></td>
<td align="center"><sub>c</sub></td>
<td align="center"><kbd>v</kbd><br><sub>slider</sub></td>
<td align="center"><kbd>b</kbd><br><sub>slider</sub></td>
<td align="center"><sub>n</sub></td>
<td align="center"><sub>m</sub></td>
<td align="center"><sub>,</sub></td>
<td align="center"><kbd>.</kbd><br><sub>one frame</sub></td>
<td align="center"><sub>/</sub></td>
<td align="center"><sub>shift</sub></td>
</tr>
<tr>
<td colspan="12" align="center"><kbd>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Space&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</kbd><br><sub>pause</sub></td>
</tr>
</table>

A slider key in lower case lowers its slider one notch, and in upper case raises
it; <kbd>K</kbd> takes a hawk away. The pointer plays too: move it into the
flock and the flock scatters.

<table>
<tr>
<td align="center"><img src="../../docs/clips/3d-prism.gif" height="220" alt="Two thousand birds in three dimensions, coloured by depth from red near the camera to blue far from it, fold into a ribbon that bends round a hawk"><br><sub><kbd>k</kbd> calls a hawk like this one, from <code>cbirds --3d --color prism --hawks 1</code></sub></td>
<td align="center"><img src="../../docs/clips/pipe-donut.gif" height="220" alt="A donut drawn in characters from warm red to gold breaks into a flock of characters, swirls, and every character lands back on its own cell"><br><sub><code>./donut | cbirds</code>; <kbd>Enter</kbd> sends the letters off, or calls them home, at once</sub></td>
</tr>
</table>

## What each key does to the flock

### <kbd>k</kbd> <kbd>K</kbd> &nbsp; a hawk more, one fewer

<p align="center"><img src="../../docs/demo.gif" width="560" alt="Three hundred ice-blue birds part around two scarlet hawks and close again behind them, in slow motion"></p>
<p align="center"><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0 -n 300 -s 14</code></sub></p>

Up to four hawks. The flock parts around a hawk and closes behind it. In `--3d`
the hawks hunt the flock through the air; over text they fly as arrows.
`-k`/`--hawks` sets them from the start, 0 to 4.

### <kbd>Enter</kbd> &nbsp; the letters off, or home

Pipe text into cbirds and the text is the flock: every letter is a bird. It
opens with the text exactly as the command printed it, in its own colours (24-bit,
256 and the 16), with fastfetch's and neofetch's layout kept, because cbirds reads
the output through a small terminal emulator of its own. A few seconds later one
letter leaves, the ones near it follow, and the letters fly as a flock; after
twenty seconds or so they are called home, and every letter lands on its own
cell, exactly. Then it begins again. `Enter` does not wait: it sends the letters
off, or calls them home, at once. The pointer lifts the letters it touches.

<table>
<tr>
<td align="center"><img src="../../docs/clips/pipe-fetch.gif" width="380" alt="A fastfetch-style screen, a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks: the letters and the dots fly off and come back, the colour blocks stay"><br><sub><code>cat docs/clips/fetch.ans | cbirds</code>, a screen in the manner of fastfetch</sub></td>
<td align="center"><img src="../../docs/clips/pipe-code.gif" width="380" alt="cbirds' own C source, the function that makes the flock flee a hawk, syntax-highlighted in monokai colours, takes flight letter by letter and lands back"><br><sub><code>pygmentize boids.c | cbirds</code>, cbirds reading its own code</sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-git.gif" width="380" alt="A git log graph of cbirds' own history, three coloured lanes and yellow hashes, takes flight letter by letter and lands back"><br><sub><code>git log --graph --color=always | cbirds</code></sub></td>
<td align="center"><img src="../../docs/clips/pipe-sunset.gif" width="380" alt="A pixel-art sunset drawn in coloured full blocks shatters into a cloud of coloured pixels, flies, and reassembles the picture exactly"><br><sub><code>cat docs/clips/sunset.ans | cbirds</code>, a picture drawn in █ characters</sub></td>
</tr>
<tr>
<td colspan="2" align="center"><img src="../../docs/clips/pipe-banner.gif" width="560" alt="CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti and rebuilds the word"><br><sub><code>cat docs/clips/banner.ans | cbirds</code>, a banner made for the clip</sub></td>
</tr>
</table>

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

`--text FILE` reads a file and `--text -` standard input; a pipe with nothing in
it gives the ordinary flock.

### <kbd>h</kbd> &nbsp; the panel, and the slider keys

```
╭──────────────────────────────╮    ╭──────────────────────────────╮
│ boundary   ▓▓▓▓░░░░░░░░  b/B │    │ roost      ▓▓▓▓░░░░░░░░  b/B │
│ separation ▓▓▓▓░░░░░░░░  s/S │    │ separation ▓▓▓▓░░░░░░░░  s/S │
│ alignment  ▓▓▓▓░░░░░░░░  a/A │    │ alignment  ▓▓▓▓░░░░░░░░  a/A │
│ turning    ▓▓▓▓▓▓▓▓░░░░  t/T │    │ turning    ▓▓▓▓▓▓▓▓░░░░  t/T │
│ perception ▓▓▓▓▓▓░░░░░░  p/P │    │ neighbours ▓▓▓▓▓▓░░░░░░  p/P │
│ speed      ▓░░░░░░░░░░░  v/V │    │ speed      ▓░░░░░░░░░░░  v/V │
│ quit       q                 │    │ quit       q                 │
╰──────────────────────────────╯    ╰──────────────────────────────╯
             the flock                         with --3d
```

`h` brings up the panel, and `--panel` shows it from the start. Every slider
runs from 0 to 12 notches, one a key press: lowercase lowers, uppercase raises.

| keys | slider | what it changes |
|---|---|---|
| <kbd>b</kbd> <kbd>B</kbd> | boundary | the nudge away from the edges of the screen |
| <kbd>s</kbd> <kbd>S</kbd> | separation | how hard a bird keeps its distance |
| <kbd>a</kbd> <kbd>A</kbd> | alignment | how much it flies the way its neighbours fly |
| <kbd>t</kbd> <kbd>T</kbd> | turning | how far it may turn in a frame: the limit that gives curved fronts instead of a cloud snapping into shape |
| <kbd>p</kbd> <kbd>P</kbd> | perception | how far off a neighbour still counts |
| <kbd>v</kbd> <kbd>V</kbd> | speed | the pace, from 0.2x to 2.6x |
| <kbd>g</kbd> <kbd>G</kbd> | | one more slider, one notch down / up; see the [manual](../../docs/manual.md#the-panel) |

In `--3d`, `boundary` becomes `roost`, the pull that calls a stray bird home, and
`perception` becomes `neighbours`: how many of its nearest a bird heeds, 1 to 13.

### <kbd>Tab</kbd> &nbsp; the next preset

<p align="center"><img src="../../docs/murmuration.gif" width="560" alt="The murmuration preset drawn in braille dots in dusk colours, peach through red to violet"></p>
<p align="center"><sub><code>cbirds --preset murmuration --color dusk</code></sub></p>

The presets are `murmuration`, `swarm` and `storm`; `--preset` picks one from
the start.

### <kbd>e</kbd> &nbsp; tails

<p align="center"><img src="../../docs/depth.gif" width="560" alt="A second sky of smaller, dimmer birds further off, and faint tails, in ice blue"></p>
<p align="center"><sub><code>cbirds --depth --trails --color ice</code></sub></p>

Faint tails behind the birds; `-e`/`--trails` from the start. `--3d` draws no
tails, so there `e` has nothing to do.

### <kbd>+</kbd> <kbd>-</kbd> &nbsp; more birds, fewer

Anywhere from 1 bird to 4096. `-n`/`--birds` sets the number from the start, 800
by default and 2000 in `--3d`.

### <kbd>Space</kbd> <kbd>.</kbd> <kbd>0</kbd> &nbsp; pause, one frame, the defaults

`Space` pauses the flock. `.` moves it on by one frame. `0` goes back to the
defaults.

### <kbd>q</kbd> &nbsp; away

The flock flies off the top, and the terminal is yours again.

With `--screensaver` there is no flight: any key, click or movement of the pointer
quits at once (input in the first half second is ignored), which makes
`cbirds --screensaver --clock` a lock screen that tells the time.

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

### The pointer

Move it into the flock and the flock scatters. Over text it lifts the letters it
touches. In `--3d` it is a stick poked into the sky: birds near the line from the
camera through it get out of the way.

### No keys at all

Left alone for a minute, the flock starts moving the sliders itself, a notch
every four seconds, and any key ends that. A sign takes no orders: a key does not
end it, and birds scattered by the pointer or a hawk come back.

<table>
<tr>
<td align="center"><img src="../../docs/say.gif" width="380" alt="The flock writes BACK IN FIVE and hovers on it while the rest of the flock wheels round"><br><sub><code>cbirds --say "back in five"</code></sub></td>
<td align="center"><img src="../../docs/clock.gif" width="380" alt="The flock writes 10:09, and at the new minute only the changing digits let go while other birds write 10:10"><br><sub><code>cbirds --clock-at 10:09:52</code></sub></td>
</tr>
</table>

`--say TEXT` has the flock write TEXT and hold it as a sign for thirty to
forty-five seconds, let go, and write it again, while the rest of the flock wheels
round it; `--font-size ROWS` sets the letters from 4 to 10 rows. `--clock` writes
the time, HH:MM, in local time, `--seconds` adds the seconds, and at each new
minute only the digits that change let go; `--clock-at HH:MM[:SS]` starts it at a
time you choose. `--picture FILE` has the flock draw a PNG, in at most eight of
its colours, or in a ramp if `--color` is given; `--shape dot -n 2000` draw it
best.

## The rest

### Three dimensions

<p align="center"><img src="../../docs/clips/3d-braille.gif" width="560" alt="The three-dimensional flock drawn in braille dots in dusk colours, pink, violet and peach, with a cyan hawk"></p>
<p align="center"><sub><code>cbirds --3d --render braille --color dusk</code>, as a terminal with no graphics protocol shows it</sub></p>

`--3d` takes the flock off the plane and into a sky: a murmuration of 2000 birds
over its roost, seen from a camera that goes once round it every two minutes, so
the depth shows even in a still frame. A nearer bird is bigger and brighter; a
bird flying at the camera is short, one flying across it long. The three rules
stay, but a bird heeds its seven nearest neighbours however far off they are, as
starlings do (Ballerini et al., PNAS 2008), and turns at a limited rate a second,
so it banks. A roost calls it home when it strays, and the air above the roost
moves slowly, which folds the flock into sheets and ribbons (Hildenbrandt, Carere
and Hemelrijk, Behavioral Ecology 2010).

The birds are 8 pixels, 5 in the text renderers, and the ramp is **ink**, from
your terminal's own text colour to its background, asked of the terminal at
startup; a terminal that does not answer, and every recording, get `ash`. Any
other ramp: `cbirds --3d --color prism`. Every renderer draws it, and the keys,
`--screensaver` and recording go with it. It replaces `--depth`, and it is one
flock: `--flocks`, `--matrix`, a text, a sign and `--link` are refused.

### One sky, several terminals

<p align="center"><img src="../../docs/clips/link.gif" width="800" alt="Three terminal windows side by side write BOIDS, then a flock in prism colours flows out of one window and into the next, and the hawks cross too"></p>
<p align="center"><sub>three real windows, <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code> in each, recorded through cbirds' own terminal emulator</sub></p>

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the next,
at the same height and in the same direction; hawks cross too. The windows lie in
a row in the order they were started, so start them left to right; only an edge
with a window behind it is open, and the outer edges are walls. Each window keeps
its own settings (its own `--birds`, colours, renderer), and a sign goes with it,
as does `--screensaver`. The windows talk through Unix sockets in a directory that
is yours alone. `--link` is refused with `--text`, text piped in, `--3d`,
`--record` and `--bench`.

### Options

| option | |
|---|---|
| `-n`, `--birds` | 1 to 4096 birds, 800 by default |
| `-s`, `--size` | 4 to 64 pixels, 30 by default |
| `-g`, `--flocks` | 1 to 3 flocks that keep to their own kind |
| `-k`, `--hawks` | 0 to 4 predators |
| `--preset` | `murmuration`, `swarm` or `storm` |
| `-c`, `--color` | `theme` (your terminal's own colours), `ember`, `ice`, `acid`, `matrix`, `aurora`, `prism`, `potion`, `dusk`, `ash`, `ink` |
| `--shape` | `bird`, `arrow`, `plane` or `dot` |
| `--sprite FILE` | any PNG up to 4 MB, kept in its own colours |
| `--depth` | a second sky further off: smaller, slower, dimmer birds |
| `-e`, `--trails` | faint tails |
| `--matrix` | it is raining birds |
| `--seed N` | the same flock on every system |
| `--panel` | the panel from the start |

### Terminals

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

### Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

`--record` needs no terminal, so no keys: it runs headless and writes the GIF with
its own encoder, sprites or, with `--render braille | sextants | blocks`, the
cells. A name ending in `.cast` gives an asciinema recording.
`--record-size COLSxROWS` (40x14 to 400x120, default 96x26), `--record-fps` (a GIF
carries up to 50), `--record-seconds` (default 6, 34 for text). `--snapshot FILE`
writes the last live frame as a PNG. Every clip on this page was recorded by
cbirds itself.

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

cbirds is one program in C99 with no dependencies, for Linux and macOS: the PNG,
GIF and DEFLATE code is all in the repository; there is no zlib, no libpng, no
ncurses.

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

<div align="center">

# 🐦 cbirds

**A flock of birds in your terminal.**<br>
Pipe text into it and every letter becomes a bird. They all come home, too.

<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/birds-1%20to%204096-blue" alt="birds: 1 to 4096">
<img src="https://img.shields.io/badge/hawks-0%20to%204-red" alt="hawks: 0 to 4">
<img src="https://img.shields.io/badge/sprites-Kitty%20%7C%20Ghostty-8A2BE2" alt="sprites in Kitty and Ghostty">

<img src="../../docs/clips/pipe-code.gif" width="830" alt="cbirds' own C source, the function that makes the flock flee a hawk, syntax-highlighted in monokai colours, takes flight letter by letter, swirls as a flock and lands back exactly where it was">

<sub>cbirds reading its own source: <code>pygmentize boids.c | cbirds</code>. The function on screen makes the flock flee a hawk. Then the function itself takes off.</sub>

</div>

> [!TIP]
> **The short version:** `brew install clainstone/tap/cbirds`, then
> `fastfetch | cbirds`. Your fetch screen takes off, and twenty seconds or so
> later every letter lands back on its own cell, exactly.

## 🍩 Anything can fly

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

Pipe text into cbirds and the text is the flock. It opens with the text exactly
as the command printed it, colours and all (24-bit, 256 and the 16), and a fetch
screen keeps its logo beside its information, because cbirds reads the output
through a small terminal emulator of its own. A few seconds later one letter
leaves, its neighbours follow, and off they go. After twenty seconds or so they
are called home, and every letter lands on its own cell, exactly. Then it begins
again.

<p align="center">
  <img src="../../docs/clips/pipe-donut.gif" width="830" alt="One frame of the spinning donut, a torus drawn in the characters .,-~:;=!*#$@ from warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its cell"><br>
  <sub>🍩 <code>./donut | cbirds</code>: one frame of the spinning donut (<a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c) goes out for a fly, and every crumb comes back to its cell.</sub>
</p>

<table>
  <tr>
    <td align="center" width="50%"><img src="../../docs/clips/pipe-sunset.gif" width="380" alt="A pixel-art sunset drawn in coloured block characters shatters into a cloud of coloured pixels, flies, and reassembles the picture exactly"></td>
    <td align="center" width="50%"><img src="../../docs/clips/pipe-banner.gif" width="380" alt="CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti and rebuilds the word"></td>
  </tr>
  <tr>
    <td align="center"><sub>🌅 <code>cat docs/clips/sunset.ans | cbirds</code><br>a sunset drawn in █ characters falls apart and puts itself back together, exactly</sub></td>
    <td align="center"><sub>🎉 <code>cat docs/clips/banner.ans | cbirds</code><br>a banner, turned into confetti and back</sub></td>
  </tr>
  <tr>
    <td align="center"><img src="../../docs/clips/pipe-fetch.gif" width="380" alt="A fetch-style screen, a swallow drawn in braille from cyan to pink beside a list of flock settings and two rows of colour blocks: the letters and the bird's dots fly, the colour blocks stay"></td>
    <td align="center"><img src="../../docs/clips/pipe-git.gif" width="380" alt="A git log graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of letters and lands back"></td>
  </tr>
  <tr>
    <td align="center"><sub>🐦 <code>cat docs/clips/fetch.ans | cbirds</code><br>the colour blocks stay put; everything else flies</sub></td>
    <td align="center"><sub>🌳 <code>git log --graph --color=always | cbirds</code><br>cbirds' own history, migrating</sub></td>
  </tr>
</table>

> [!NOTE]
> The fetch screen, the banner and the sunset were made for these clips. The
> donut, the code and the git log are the real output of the commands named.
> `fastfetch | cbirds` does the same with your own fetch screen.

`Enter` sends the letters off, or calls them home at once. The pointer lifts the
letters it touches. Hawks work on text too, and fly over it as arrows.
`--text FILE` reads a file, `--text -` reads standard input, and a pipe with
nothing in it gives you the ordinary flock instead. More in the
[manual](../../docs/manual.md#anything-can-fly).

## 🌌 Three dimensions

<p align="center">
  <img src="../../docs/clips/3d-prism.gif" width="668" alt="Two thousand birds over their roost, seen from a circling camera and coloured by depth, red near and blue far, fold into a ribbon and bend round a pale hawk"><br>
  <sub><code>cbirds --3d --color prism --hawks 1</code>: two thousand birds, red near and blue far, folding round a hawk</sub>
</p>

`--3d` and the flock leaves the plane for a sky: a murmuration of 2000 birds
over its roost, seen from a camera that goes once round it every two minutes,
so the depth shows even in a still frame. A nearer bird is bigger and brighter;
a bird flying at the camera is short, one flying across it long.

Same three rules, with a starling's manners: a bird heeds its seven nearest
neighbours however far off they are, as starlings do (Ballerini et al., PNAS
2008), rather than everything within a radius. It turns at a limited rate a
second, so it banks. A roost calls it home when it strays, and the air above the
roost moves slowly, which folds the flock into sheets and ribbons (Hildenbrandt,
Carere and Hemelrijk, Behavioral Ecology 2010).

<table>
  <tr>
    <td align="center"><img src="../../docs/clips/3d-ice.gif" width="176" alt="An ice-blue murmuration with a red hawk"></td>
    <td align="center"><img src="../../docs/clips/3d-potion.gif" width="176" alt="A murmuration in violet and green"></td>
    <td align="center"><img src="../../docs/clips/3d-dusk.gif" width="176" alt="A murmuration from peach to violet with a cyan hawk"></td>
    <td align="center"><img src="../../docs/clips/3d-ash.gif" width="176" alt="A murmuration in greys with a red hawk"></td>
  </tr>
  <tr>
    <td align="center"><sub>🧊 <code>--color ice</code></sub></td>
    <td align="center"><sub>🧪 <code>--color potion</code></sub></td>
    <td align="center"><sub>🌇 <code>--color dusk</code></sub></td>
    <td align="center"><sub>🌫️ <code>--color ash</code></sub></td>
  </tr>
</table>

> [!TIP]
> The default in 3D is `ink`: a ramp from your terminal's own text colour to its
> background, asked of the terminal at startup. Grey on a dark terminal, near
> black on a light one: the flock dresses to match. A terminal that does not
> answer, or whose text is the colour of its ground, and every recording, get
> `ash`, the greys above. `--color ink` works in the flat sky as well.

Hawks hunt the flock through the air (`--hawks`, `k`/`K`). The pointer is a
stick poked into the sky: birds near the line from the camera through it get out
of the way. In the panel `boundary` becomes `roost` and `perception` becomes
`neighbours` (1 to 13). Every renderer draws it, braille dots included (birds of
5 pixels there, 8 with sprites), and `--screensaver`, the keys and recording all
come along. It replaces `--depth` and draws no tails. It is one flock, so
`--flocks` and `--matrix` are refused, and so are a text, a sign and `--link`.

## 🔗 One sky, several terminals

<p align="center">
  <img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each running cbirds --link: BOIDS is written in prism colours in each, then the flock flows out of one window and into the next, and pale hawks cross too"><br>
  <sub>three real windows, <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code> in each, recorded through cbirds' own terminal emulator</sub>
</p>

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the
next, at the same height and in the same direction, and hawks cross too. Two
terminals side by side, or the two panes of your split, become one sky.

> [!IMPORTANT]
> Start them left to right. The windows lie in a row in the order they were
> started; only an edge with a window behind it is open, and the outer edges are
> walls.

Each window keeps its own settings: its own `--birds`, colours, renderer. A sign
goes with it, and so does `--screensaver`. The windows talk through Unix sockets
in a directory that is yours alone (`$XDG_RUNTIME_DIR/cbirds`, or
`$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`). `--link` is refused with `--text`,
text piped in, `--3d`, `--record` and `--bench`. More in the
[manual](../../docs/manual.md#one-sky-several-terminals).

## 📦 Install

Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64,
from [clainstone.com/apt](https://clainstone.com/apt):

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

Both come with completions for bash, zsh and fish, and `brew upgrade` or
`sudo apt upgrade` keeps them up to date. The .deb files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest).

<details>
<summary>🛠️ From source, with a C compiler and make and nothing else</summary>

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
<summary>👋 Uninstall</summary>

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

## 🦅 The flock

<table>
  <tr>
    <td align="center" width="33%"><img src="../../docs/hawks.gif" width="240" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, in ice blue"></td>
    <td align="center" width="33%"><img src="../../docs/prism.gif" width="240" alt="Three flocks in red, green and violet write BOIDS together, then cross each other, each keeping to its own kind"></td>
    <td align="center" width="33%"><img src="../../docs/matrix.gif" width="240" alt="Green birds with tails raining down the window"></td>
  </tr>
  <tr>
    <td align="center"><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code><br>the flock parts around them and closes behind</sub></td>
    <td align="center"><sub><code>cbirds --render kitty --flocks 3 --color prism</code><br>three flocks that keep to their own kind</sub></td>
    <td align="center"><sub><code>cbirds --matrix</code><br>it is raining birds</sub></td>
  </tr>
</table>

It opens by writing BOIDS, lets go, and flocks. Move the pointer into the flock
and it scatters. Press `q` and it does not just quit: it flies off the top. Left
alone for a minute, it starts playing with its own sliders, a notch every four
seconds; any key ends that.

Every bird follows three rules: keep your distance, fly the way your neighbours
fly, drift towards their middle. Add a nudge away from the edges, and limit how
far it may turn in a frame, and you get curved fronts instead of a cloud
snapping into shape.

| option | |
|---|---|
| `-n`, `--birds` | 1 to 4096 (default 800) |
| `-s`, `--size` | 4 to 64 pixels (default 30) |
| `-g`, `--flocks` | 1 to 3 flocks that keep to their own kind |
| `-k`, `--hawks` | 0 to 4 predators |
| `--preset` | `murmuration`, `swarm` or `storm` |
| `--seed N` | the same flock on every system: reproducible birds |
| `-c`, `--color` | `theme` (the terminal's own colours, asked of the terminal), `ember`, `ice`, `acid`, `matrix`, `aurora`, `prism`, `potion`, `dusk`, `ash`, `ink` |
| `--shape` | `bird`, `arrow`, `plane` or `dot` |
| `--sprite FILE` | any PNG up to 4 MB, kept in its own colours: fly whatever you like |
| `--depth` | a second sky further off: smaller, slower, dimmer birds |
| `-e`, `--trails` | faint tails |
| `--matrix` | it is raining birds: green, falling, with tails |

## ⌨️ Keys

| key | does |
|---|---|
| <kbd>h</kbd> | the panel: sliders for boundary, separation, alignment, turning, perception and speed (0.2x to 2.6x), each 0 to 12 notches |
| <kbd>b</kbd> <kbd>s</kbd> <kbd>a</kbd> <kbd>t</kbd> <kbd>p</kbd> <kbd>v</kbd> <kbd>g</kbd> | a slider one notch down; the capitals put it one notch up |
| <kbd>Space</kbd> | pause |
| <kbd>.</kbd> | one frame, for the patient |
| <kbd>0</kbd> | back to the defaults |
| <kbd>Tab</kbd> | next preset |
| <kbd>+</kbd> <kbd>-</kbd> | more birds, fewer |
| <kbd>k</kbd> | a hawk more |
| <kbd>K</kbd> | one fewer, for when you start feeling sorry for the birds |
| <kbd>e</kbd> | tails |
| <kbd>Enter</kbd> | letters off, or home (with text) |
| <kbd>q</kbd> | quit, by way of the top of the screen |

`--panel` opens the panel from the start. More in the
[manual](../../docs/manual.md#the-panel).

## ⏰ Signs, the clock and pictures

<table>
  <tr>
    <td align="center" width="33%"><img src="../../docs/say.gif" width="240" alt="The flock writes BACK IN FIVE and hovers on it while the rest of the flock wheels round"></td>
    <td align="center" width="33%"><img src="../../docs/clock.gif" width="240" alt="The flock writes 10:09; at the new minute the changing digits let go and other birds write 10:10"></td>
    <td align="center" width="33%"><img src="../../docs/picture.gif" width="240" alt="Fifteen hundred dots gather into a sunset over pine trees"></td>
  </tr>
  <tr>
    <td align="center"><sub><code>cbirds --say "back in five"</code><br>an away message, written by birds</sub></td>
    <td align="center"><sub><code>cbirds --clock-at 10:09:52</code><br>only the digits that change let go</sub></td>
    <td align="center"><sub><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub></td>
  </tr>
</table>

`--say TEXT` writes TEXT and holds it as a sign for thirty to forty-five
seconds, lets go, and writes it again, while the rest of the flock wheels round
it. `--font-size ROWS`, 4 to 10. `--clock` writes the time, HH:MM, in local
time; `--seconds` adds the seconds; `--clock-at HH:MM[:SS]` starts it at a time
you choose. `--picture FILE` draws a PNG in at most eight of its colours, or in a
ramp if `--color` is given; `--shape dot -n 2000` draw it best. A key does not
end a sign, and birds that the pointer or a hawk scatters come back. More in the
[manual](../../docs/manual.md#signs-the-clock-and-pictures).

## 🔒 Screensaver

`--screensaver` quits at once on any key, click or twitch of the pointer, with no
flight out. Input in the first half second is ignored. It goes with everything
else, so `cbirds --screensaver --clock` tells the time until you are back. In
tmux:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

In zsh:

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

## 🖥️ Terminals

Braille by default, in every terminal; no terminal is guessed at.
`--render sextants` and `--render blocks` are bolder text versions: sextants need
a font from 2020 or later, blocks work everywhere. Only the cells that changed
are sent, and the background is never painted, so the flock wears your theme.

`--render kitty` draws real sprites over the Kitty graphics protocol, made for
Kitty and Ghostty.

> [!WARNING]
> WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol and then draw
> too few birds, the wrong ones or none. Inside tmux the sprites never reach the
> terminal. Braille works everywhere.

## 📼 Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

`--record` does not even need a terminal: it runs headless and writes the GIF
with its own encoder (sprites, or the cells with
`--render braille | sextants | blocks`). A name ending in `.cast` gives an
asciinema recording. `--record-size COLSxROWS` (40x14 to 400x120, default
96x26), `--record-fps` (a GIF carries up to 50), `--record-seconds` (default 6,
34 for text, so a whole cycle is recorded and it ends on the text as it was
printed). `--snapshot FILE` writes the last live frame as a PNG. Every clip on
this page was recorded by cbirds itself.

## 🔬 Inside

One program in C99 with no dependencies. No zlib, no libpng, no ncurses: it
brought its own PNG, GIF and DEFLATE code. Linux and macOS.

At startup the bird is rotated into sixty headings, three wing positions and
every shade of the ramp: about fifteen hundred small images. The wings beat six
times a second, and now and then a bird glides. With sprites, 800 birds cost
about half a millisecond of CPU a frame and 4000 about three; braille, the
default, about three and ten, since every frame is painted and sampled down to
dots. In 3D, 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty
sprites, measured on one desktop machine. `cbirds --bench 300` prints the
numbers on yours.

## 🙏 Credits

Much of cbirds' code, tests and documentation was written by large language
models (Claude, by Anthropic), under my direction.

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*,
  SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/)
- Ballerini et al., *Interaction ruling animal collective behavior depends on
  topological rather than metric distance: Evidence from a field study*, PNAS 2008.
- Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of
  thousands of starlings: a model*, Behavioral Ecology 2010.
- The Kitty graphics protocol:
  [sw.kovidgoyal.net/kitty/graphics-protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/)
- The donut in the donut clip is after Andy Sloane's donut.c (2006).

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## 📜 Licence

MIT, by Alessandro Palliccia ([clainstone](https://github.com/clainstone)). See
[LICENSE](../../LICENSE).

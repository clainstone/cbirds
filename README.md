# cbirds

A flock of birds in your terminal.

<p>
<a href="https://github.com/clainstone/cbirds/tags"><img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version"></a>
<a href="LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT license"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux, macOS">
<a href="#terminals"><img src="https://img.shields.io/badge/sprites-Kitty%20%7C%20Ghostty-8A2BE2" alt="sprites in Kitty and Ghostty"></a>
</p>

<p align="center"><img src="docs/demo.gif" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks in slow motion"></p>

<p align="center"><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></p>

Craig Reynolds' boids, drawn in braille in any terminal, and as sprites over
the Kitty graphics protocol in Kitty and Ghostty. One C99 program, no
dependencies. Every clip on this page was recorded by cbirds itself.

<table>
<tr>
<td width="50%"><img src="docs/hawks.gif" alt="Two scarlet hawks hunt an ice-blue flock, which parts around them and closes behind"></td>
<td width="50%"><img src="docs/flocks.gif" alt="Three flocks in gold, orange and red, each keeping to its own kind and crossing the others"></td>
</tr>
<tr>
<td align="center"><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></td>
<td align="center"><code>cbirds --render kitty --flocks 3 --color ember</code></td>
</tr>
<tr>
<td width="50%"><img src="docs/matrix.gif" alt="Matrix rain, as green birds with tails falling through the screen"></td>
<td width="50%"><img src="docs/depth.gif" alt="Two skies: birds with tails in front, smaller and dimmer birds drifting slower behind them"></td>
</tr>
<tr>
<td align="center"><code>cbirds --render kitty --matrix --speed 0</code></td>
<td align="center"><code>cbirds --render kitty --depth --trails --color ice</code></td>
</tr>
<tr>
<td width="50%"><img src="docs/braille.gif" alt="The same flock and hawks in braille dots, as a terminal without a graphics protocol shows it"></td>
<td width="50%"><img src="docs/sextants.gif" alt="The flock in sextant blocks, as a text terminal with a font from 2020 or later shows it"></td>
</tr>
<tr>
<td align="center"><code>cbirds --render braille --hawks 2 --color ice</code></td>
<td align="center"><code>cbirds --render sextants --color acid --speed 2</code></td>
</tr>
</table>

## Install

### Homebrew

macOS and Linux. Completions for bash, zsh and fish come with it, and
`brew upgrade` keeps it up to date.

<!-- test: brew-install -->
```
brew install clainstone/tap/cbirds
```

### apt

Debian 12, Ubuntu 22.04 and later, and their derivatives, on amd64 or arm64,
from its [apt repository](https://clainstone.com/apt). Completions come with
it, and `sudo apt upgrade` keeps it up to date.

<!-- test: apt-install -->
```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

Or just the package, without the repository and so without updates: the
`.deb` files are on the [release page](https://github.com/clainstone/cbirds/releases/latest),
for `sudo apt install ./cbirds_*.deb`.

### From source

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

That puts one file, `/usr/local/bin/cbirds`. Without sudo, in your home
instead (`~/.local/bin` has to be on your `PATH`):

```
make install PREFIX="$HOME/.local"
```

Linux and macOS. You need a C compiler and `make`, nothing else.
`make test` runs the tests. The build uses the system's `cc` and honours `CC`,
`CFLAGS`, `LDFLAGS`, `PREFIX` and `DESTDIR`, so `make CC=clang` and packaging
work as usual.

## Uninstall

### Homebrew

The second line removes the tap as well; leave it out to keep it.

<!-- test: brew-uninstall -->
```
brew uninstall cbirds
brew untap clainstone/tap
```

### apt

The last two lines remove the repository as well; leave them out to keep it.

<!-- test: apt-uninstall -->
```
sudo apt remove cbirds
sudo rm /etc/apt/sources.list.d/cbirds.list /etc/apt/keyrings/cbirds.gpg
sudo apt update
```

A `.deb` installed on its own goes with the first line alone.

### From source

In the same directory, and with the same `PREFIX` it was installed with:

```
sudo make uninstall                     # installed with sudo make install
make uninstall PREFIX="$HOME/.local"    # installed in your home
```

Without the clone it is the one file: `sudo rm /usr/local/bin/cbirds`, or
`rm ~/.local/bin/cbirds`.

## Use

```
cbirds                              a flock in braille, in your terminal's own colours
cbirds --render kitty               sprites, in Kitty or Ghostty
cbirds --preset murmuration         the starling look
cbirds --hawks 2 --color ice        something to watch
cbirds --flocks 3 --color ember     three flocks that keep to their own
cbirds --depth --trails             a second sky behind the first
cbirds --matrix                     it is raining birds
```

It opens by writing BOIDS, lets go, and flocks. Move the pointer into the
flock and it scatters. Press `q` and it flies off the top. Left alone for a
minute, it starts moving the sliders itself; any key takes them back.

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

```
╭────────────────────────────────────╮
│ boundary   ▓▓▓▓░░░░░░░░  0.20  b/B │
│ separation ▓▓▓▓░░░░░░░░ 0.005  s/S │
│ alignment  ▓▓▓▓░░░░░░░░  1.50  a/A │
│ turning    ▓▓▓▓▓▓▓▓░░░░   70°  t/T │
│ perception ▓▓▓▓▓▓░░░░░░  36px  p/P │
│ speed      ▓░░░░░░░░░░░  0.4×  v/V │
│ frame        0.6ms    31KB  60fps  │
│ quit       q                       │
╰────────────────────────────────────╯
```

| key | | key | |
|---|---|---|---|
| `b`/`B` `s`/`S` `a`/`A` `t`/`T` `p`/`P` `v`/`V` `g`/`G` | one notch down, one up | `h` | panel |
| `Space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+` `-` | more birds, fewer | `k` `K` | a hawk more, one fewer |
| `e` | tails | `q` | quit |

## Terminals

cbirds draws in braille by default, in every terminal: no terminal is guessed
at. `--render sextants` and `--render blocks` are bolder text versions;
sextants need a font from 2020 or later, blocks work everywhere. In text mode
only the cells that changed are sent, and the background is never painted, so
the flock wears your theme.

`--render kitty` draws real sprites over the Kitty graphics protocol, and is
yours to ask for: it is made for **Kitty** and **Ghostty**. In any other
terminal what it does is undefined. WezTerm, Konsole, iTerm2, Warp and Rio
answer for the protocol and then draw too few birds, the wrong ones or none,
and inside tmux the sprites never reach the terminal.

If braille does not look right in your terminal, open an issue and say which
terminal it is. That is the report that helps most.

## Options

```
Flock
  -n, --birds COUNT             how many birds (default 800)
  -s, --size PIXELS             sprite size in pixels (default 30, 60 in braille and blocks)
  -g, --flocks COUNT            flocks that keep to their own kind (default 1)
  -k, --hawks COUNT             predators hunting the flock (default 0)
      --preset NAME             murmuration, swarm, storm
      --seed N                  the same seed gives the same flock

Sliders   0 to 12, as the panel shows them
      --boundary NOTCH          how hard the edges push back (default 4)
      --separation NOTCH        how much a bird keeps its distance (default 4)
      --alignment NOTCH         how much a bird matches its neighbours (default 4)
      --turning NOTCH           sharpest turn a frame, 12 is instant (default 8)
      --perception PIXELS       how far a bird sees, 12 to 60 (default 36)
      --speed NOTCH             how fast the flock flies, 0.2x to 2.6x (default 1, 0.4x)
      --avoidance NOTCH         how much flocks keep out of each other's way (default 4)

Look
  -c, --color RAMP              theme, ember, ice, acid, matrix
      --shape NAME              bird, arrow, plane, dot
      --sprite FILE             a PNG you supply, kept in its own colours
  -e, --trails                  faint tails behind the flock
      --depth                   a second sky further off: smaller, slower, dimmer birds
  -l, --panel                   the sliders in the corner from the start; h toggles them
      --render HOW              braille by default; sextants, blocks, or kitty in Kitty and Ghostty

Oddities
      --matrix                  it is raining birds

Output
      --bench N                 run N frames with no terminal, print the numbers, quit
      --frames N                quit after N frames, for recording
      --snapshot FILE           write the last frame as a PNG
      --record FILE             record a GIF, or a .cast for asciinema, with no terminal, and quit
      --record-fps RATE         frames a second; a GIF can carry up to 50 (default 25)
      --record-seconds SECONDS  how long the recording runs (default 6)
      --record-size COLSxROWS   the size to record at, in cells (default 96x26)

General
      --unlock-fps              render as fast as the terminal allows
  -h, --help                    the one-screen help
      --completion SHELL        completions for bash, zsh or fish
  -V, --version                 print the version and quit
```

That is the options part of `cbirds --help`, as it prints it; the full help
adds usage, examples and the keys.

`--sprite` takes any PNG up to 4 MB: palette, grayscale, RGB or RGBA, at any bit
depth, interlaced or not. `--seed` is the same flock on every system: the random
numbers are cbirds' own, not the C library's.

## Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
```

`--record` needs no terminal: it runs the flock headless and writes the GIF
with its own encoder. If the file name ends in `.cast` you get an
[asciinema](https://asciinema.org) recording instead, which plays in any
terminal and is about half the size. A GIF is drawn with sprites unless
`--render braille` or `--render sextants` asks for the cells, as a text
terminal would show them. `--snapshot` saves a live frame as a PNG, so it wants a
terminal. The commands behind every clip here are in
[docs/README.md](docs/README.md).

## How it works

The bird is one PNG compiled into the binary. At startup it is rotated into
sixty headings, squashed into three wing positions, and tinted into every
shade on the ramp: about fifteen hundred small images, built in a fifth of a
second. They are sampled down into braille, sextants or blocks, or, with
`--render kitty`, uploaded once, and a frame is then one short command per
bird. The wings beat six times a second, and now and then
a bird glides.

Neighbours are found with a grid, so eight hundred birds cost about half a
millisecond of CPU a frame, and four thousand birds about four milliseconds.
The PNG, GIF and DEFLATE code is all in the repository; there is no
zlib, no libpng, no ncurses. `cbirds --bench 300` prints the numbers on your
machine.

## The algorithm

Each bird sees only its neighbours and follows three rules: keep your
distance, fly the way they fly, drift towards their middle. Add a nudge away
from the edges of the screen, sum the four pulls, and turn towards the result,
but only so far in one frame. That limit is what gives the flock curved fronts
instead of a cloud snapping into shape. Repeat sixty times a second and a
murmuration falls out of it; nothing in the code knows what a flock looks
like. Reynolds' paper, below, has the rest.

## Credits

The model is from Craig Reynolds' *Flocks, Herds, and Schools: A Distributed
Behavioral Model*, SIGGRAPH 1987; his page on boids is at
[red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/). The Kitty graphics
protocol is documented at
[sw.kovidgoyal.net/kitty/graphics-protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/).

## License

MIT. See [LICENSE](LICENSE).

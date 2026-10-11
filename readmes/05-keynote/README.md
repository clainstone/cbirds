<div align="center">

<sub><b>cbirds</b>&ensp;a flock of birds in your terminal</sub>

# Two thousand birds.<br>Three dimensions.<br>One terminal.

<img src="../../docs/clips/3d-prism.gif" width="668" alt="Two thousand birds over their roost, seen from a circling camera and coloured by depth, red near and blue far, fold into a ribbon and bend round a pale hawk">

<sub><code>cbirds --3d --color prism --hawks 1</code></sub>

Each bird heeds its seven nearest neighbours, as starlings do.<br>
A roost calls it home when it strays; the air above it moves slowly, which folds the flock into sheets and ribbons.<br>
The camera goes once round it every two minutes.

<br><br>

# Anything a terminal prints can fly.

<img src="../../docs/clips/pipe-donut.gif" width="830" alt="One frame of the spinning donut, a torus drawn in the characters .,-~:;=!*#$@ from warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its cell">

<sub><code>./donut | cbirds</code>&ensp;<a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c</sub>

Pipe text in, and every letter is a bird.

<br><br>

# Its own source code.

<img src="../../docs/clips/pipe-code.gif" width="830" alt="cbirds' own C source, the function that makes the flock flee a hawk, syntax-highlighted in monokai colours, takes flight letter by letter and lands back">

<sub><code>pygmentize boids.c | cbirds</code>&ensp;cbirds reading the function that makes the flock flee a hawk</sub>

<br><br>

# Its own history.

<img src="../../docs/clips/pipe-git.gif" width="796" alt="A git log graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of letters and lands back">

<sub><code>git log --graph --color=always | cbirds</code>&ensp;the point where four feature tracks were merged</sub>

<br><br>

# Its own name.

<img src="../../docs/clips/pipe-banner.gif" width="796" alt="CBIRDS in a big block font with a rainbow across it, over 'a flock of birds in your terminal' and '$ fastfetch | cbirds': the blocks scatter as rainbow confetti and rebuild the word">

<sub><code>cat docs/clips/banner.ans | cbirds</code>&ensp;a banner made for the clip</sub>

<br><br>

# Your fetch screen.

<img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A screen in the manner of fastfetch, a swallow drawn in braille from cyan to pink beside a list of flock settings and two rows of colour blocks: the letters and the bird's dots fly, the colour blocks stay">

<sub><code>cat docs/clips/fetch.ans | cbirds</code>&ensp;a screen in the manner of fastfetch; <code>fastfetch | cbirds</code> does the same with yours</sub>

Its logo stays beside its information, in its own colours.

<br><br>

# Every character lands on its own cell.<br>Exactly.

<img src="../../docs/clips/pipe-sunset.gif" width="796" alt="A pixel-art sunset drawn in coloured block characters, a purple sky, the sun, the sea and its road of light, shatters into a cloud of coloured pixels that flies and reassembles the picture exactly">

<sub><code>cat docs/clips/sunset.ans | cbirds</code>&ensp;a picture drawn in █ characters</sub>

After twenty seconds or so the letters are called home. Then it begins again.

<br><br>

# Three terminals. One sky.

<img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each running cbirds --link: BOIDS is written in prism colours in each, then the flock flows out of one window and into the next, and pale hawks cross too">

<sub>three real windows, <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code> in each, recorded through cbirds' own terminal emulator</sub>

Birds that fly out of one window fly in through the edge of the next.

<br><br>

# Any terminal. In braille.

<img src="../../docs/clips/3d-braille.gif" width="668" alt="The 3D flock drawn in braille dots in pink, violet and peach, with a cyan hawk">

<sub><code>cbirds --3d --render braille --color dusk</code>&ensp;as a terminal with no graphics protocol shows it</sub>

Braille by default, everywhere. Real sprites in Kitty and Ghostty.

<br><br>

# It tells the time.

<img src="../../docs/clock.gif" width="512" alt="The flock writes 10:09; at the new minute the changing digits let go and other birds write 10:10">

<sub><code>cbirds --clock-at 10:09:52</code></sub>

At each new minute, only the digits that change let go.

<br><br>

# One program. No dependencies.

C99, with its own PNG, GIF and DEFLATE code.<br>
No zlib, no libpng, no ncurses. Linux and macOS.

<br><br>

# And more.

</div>

```
cbirds                            a flock in braille, in any terminal
cbirds --render kitty             sprites, in Kitty or Ghostty
cbirds --3d                       a murmuration of 2000 birds over its roost
fastfetch | cbirds                the letters take flight, and land back
cbirds --link                     in two terminals: one sky
cbirds --preset murmuration       or swarm, or storm
cbirds --hawks 2 --color ice      two hawks in an ice-blue flock
cbirds --flocks 3                 three flocks that keep to their own kind
cbirds --depth --trails           a second sky further off, and tails
cbirds --matrix                   it is raining birds
cbirds --say "back in five"       the flock writes it, and holds it
cbirds --picture logo.png         the flock draws a PNG
cbirds --screensaver --clock      the time, until any key
cbirds --record flock.gif         a GIF, made without a terminal
```

<div align="center">

<kbd>h</kbd> the panel&emsp;<kbd>b</kbd> <kbd>s</kbd> <kbd>a</kbd> <kbd>t</kbd> <kbd>p</kbd> <kbd>v</kbd> <kbd>g</kbd> a slider one notch down; uppercase, up<br>
<kbd>Space</kbd> pause&emsp;<kbd>.</kbd> one frame&emsp;<kbd>0</kbd> back to the defaults&emsp;<kbd>Tab</kbd> next preset&emsp;<kbd>e</kbd> tails<br>
<kbd>+</kbd> <kbd>-</kbd> more birds, fewer&emsp;<kbd>k</kbd> <kbd>K</kbd> a hawk more, one fewer&emsp;<kbd>Enter</kbd> letters off, or home&emsp;<kbd>q</kbd> it flies off the top

<sub>More in the <a href="../../docs/manual.md">manual</a>.</sub>

<br><br>

# One more thing.

</div>

```
brew install clainstone/tap/cbirds
```

<div align="center">

<sub>Homebrew, on macOS and Linux. With completions for bash, zsh and fish; <code>brew upgrade</code> keeps it up to date.</sub>

</div>

<details>
<summary>apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64</summary>

From [clainstone.com/apt](https://clainstone.com/apt), with completions for bash, zsh and fish; `sudo apt upgrade` keeps it up to date:

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

The .deb files alone are on the [release page](https://github.com/clainstone/cbirds/releases/latest).

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

<br>

## The fine print

**The flock.** It opens by writing BOIDS, lets go, and flocks. Each bird keeps
its distance, flies the way its neighbours fly and drifts towards their middle,
with a nudge away from the edges; turning is limited per frame, which gives
curved fronts instead of a cloud snapping into shape. Move the pointer into the
flock and it scatters. Left alone for a minute it starts moving the sliders
itself, a notch every four seconds; any key ends that. `-n/--birds` 1 to 4096
(default 800), `-s/--size` 4 to 64 pixels (default 30), `-g/--flocks` 1 to 3
flocks that keep to their own kind, `-k/--hawks` 0 to 4 predators: the flock
parts around them and closes behind. `-c/--color` takes a ramp: theme (the
terminal's own colours, asked of the terminal), ember, ice, acid, matrix,
aurora, prism, potion, dusk, ash, ink. `--shape bird | arrow | plane | dot`;
`--sprite FILE`, any PNG up to 4 MB, kept in its own colours; `--seed N`, the
same flock on every system. The panel (`h`, or `--panel` from the start) has
sliders for boundary, separation, alignment, turning, perception and speed
(0.2x to 2.6x), each 0 to 12 notches.

**Terminals.** Braille is the default, in every terminal; no terminal is guessed
at. `--render sextants` and `--render blocks` are bolder text versions: sextants
need a font from 2020 or later, blocks work everywhere. `--render kitty` draws
real sprites over the Kitty graphics protocol, made for Kitty and Ghostty.
WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol and then draw too
few birds, the wrong ones or none; inside tmux the sprites never reach the
terminal. Only the cells that changed are sent, and the background is never
painted, so the flock wears your theme.

**Text.** The text opens exactly as the command printed it, in its own colours
(24-bit, 256 and the 16); fastfetch's and neofetch's layout is kept, because
cbirds reads the output through a small terminal emulator of its own. A few
seconds later one letter leaves, the ones near it follow, and the letters fly as
a flock. `Enter` sends the letters off or calls them home at once, the pointer lifts the letters
it touches, and hawks fly over text as arrows. `--text FILE` reads a file and
`--text -` standard input; a pipe with nothing in it gives the ordinary flock.

**Three dimensions.** By default, 2000 birds of 8 pixels (5 in the text renderers) in `ink`,
a ramp from your terminal's own text colour to its background, asked of the
terminal at startup: grey on a dark terminal, near black on a light one. A
terminal that does not answer, or whose text is the colour of its ground, and
every recording, get `ash`; any other ramp can be asked for. Hawks hunt through
the air, and the pointer is a stick poked into the sky: birds near the line from
the camera through it get out of the way. In the panel `boundary` becomes `roost`
and `perception` becomes `neighbours` (1 to 13). Every renderer draws it. It
replaces `--depth`, draws no tails, and refuses `--flocks`, `--matrix`, a text, a
sign and `--link`.

**One sky.** The windows lie in a row in the order they were started: start them
left to right. Only an edge with a window behind it is open; the outer edges are
walls. Hawks cross too. Each window keeps its own settings, and a sign goes with
it, as does `--screensaver`. The windows talk through Unix sockets in a
directory that is yours alone (`$XDG_RUNTIME_DIR/cbirds`, or
`$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`). `--link` is refused with `--text`,
text piped in, `--3d`, `--record` and `--bench`.

**Signs.** `--say TEXT` writes TEXT and holds it for thirty to forty-five seconds,
lets go, and writes it again, while the rest of the flock wheels round it
(`--font-size ROWS`, 4 to 10). `--clock` is HH:MM in local time, `--seconds` adds
the seconds, `--clock-at HH:MM[:SS]` starts it at a time you choose.
`--picture FILE` draws a PNG in at most eight of its colours, or in a ramp if
`--color` is given; `--shape dot -n 2000` draw it best. A key does not end a
sign, and birds scattered by the pointer or a hawk come back.

**Screensaver.** `--screensaver` quits at once on any key, click or pointer
movement, with no flight out (input in the first half second is ignored), and
goes with everything else. In tmux, and in zsh:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

**Recording.** `--record` needs no terminal: it runs headless and writes the GIF
with its own encoder; a name ending in `.cast` gives an asciinema recording.
`--record-size COLSxROWS` (40x14 to 400x120, default 96x26), `--record-fps` (a
GIF carries up to 50), `--record-seconds` (default 6, 34 for text, which ends on
the text as it was printed). `--snapshot FILE` writes the last live frame as a
PNG. Every clip on this page was recorded by cbirds itself.

**Numbers.** With sprites, 800 birds cost about half a millisecond of CPU a frame
and 4000 about three; braille, the default, about three and ten. In 3D, 2000
birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites, measured
on one desktop machine. `cbirds --bench 300` prints the numbers on yours.

## Credits

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

Listed in [Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT, by Alessandro Palliccia ([clainstone](https://github.com/clainstone)). See
[LICENSE](../../LICENSE).

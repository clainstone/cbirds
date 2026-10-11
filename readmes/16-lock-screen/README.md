<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clock.gif" width="410" alt="Ember birds write 10:09 while the rest of the flock wheels round; at the new minute the two digits that change let go and other birds write 10:10"></td>
<td width="50%" align="center"><img src="../../docs/say.gif" width="410" alt="Ember birds write BACK IN FIVE on two lines and hover in their places while the rest of the flock wheels round the sign"></td>
</tr>
<tr>
<td align="center"><sub><code>cbirds --clock-at 10:09:52</code></sub></td>
<td align="center"><sub><code>cbirds --say "back in five"</code></sub></td>
</tr>
</table>

<div align="center">

# cbirds

**Lock your terminal, and a flock of birds tells the time, or leaves a note, while you are away.**<br>
Any key, click or movement, and it is gone.

</div>

cbirds is a flock of birds in your terminal: Craig Reynolds' boids, in braille in any
terminal, and as real sprites in Kitty and Ghostty. With `--screensaver` it quits at once on
any key, click or pointer movement, with no flight out, which makes it a lock screen. Four
steps:

[1. Install](#1-install), [2. Set the lock](#2-set-the-lock),
[3. Walk away](#3-walk-away), [4. Come back](#4-come-back), and then,
[while you are here](#while-you-are-here), everything else it does.

## 1. Install

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

Homebrew and apt both come with completions for bash, zsh and fish, and `brew upgrade` or
`sudo apt upgrade` keeps them up to date. The `.deb` files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest). One program in C99, no
dependencies, for Linux and macOS.

## 2. Set the lock

<table>
<tr>
<th width="50%">tmux</th>
<th width="50%">zsh</th>
</tr>
<tr>
<td>

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

</td>
<td>

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

</td>
</tr>
<tr>
<td>In <code>~/.tmux.conf</code>: after 300 seconds with no input, tmux locks and runs the lock command.</td>
<td>In <code>~/.zshrc</code>: after 300 seconds idle at the prompt, zsh runs <code>TRAPALRM</code>.</td>
</tr>
</table>

`--clock` writes the time, HH:MM, in local time, and at each new minute only the digits that
change let go and are written again, so the time can be read at any moment. Add `--seconds`
for HH:MM:SS.

## 3. Walk away

`--screensaver` goes with everything else, so the lock command can be any of these:

```
cbirds --screensaver --clock --seconds       the time, to the second
cbirds --screensaver --say "back in five"    a note, held as a sign
cbirds --screensaver --3d                    a murmuration in your terminal's own ink
fastfetch | cbirds --screensaver             your machine's details, taking flight
cbirds --screensaver --picture photo.png     a PNG, drawn by the flock
```

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clips/3d-prism.gif" width="410" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in rainbow colours, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk"><br><sub>A murmuration of 2000 birds over its roost, seen from a camera that goes once round it every two minutes.<br><code>cbirds --3d --color prism --hawks 1</code></sub></td>
<td width="50%" align="center"><img src="../../docs/clips/pipe-fetch.gif" width="410" alt="A terminal window titled cat fetch.ans | cbirds: a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks; the letters and the swallow's dots fly while the colour blocks stay"><br><sub>A screen in the manner of fastfetch, made for the clip; <code>fastfetch | cbirds</code> flies yours.<br><code>cat docs/clips/fetch.ans | cbirds</code></sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-donut.gif" width="410" alt="A terminal window titled ./donut | cbirds: a donut drawn in the characters .,-~:;=!*#$@, warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its own cell"><br><sub>Anything a command prints: one frame of a spinning donut, after Andy Sloane's donut.c.<br><code>./donut | cbirds</code></sub></td>
<td align="center"><img src="../../docs/picture.gif" width="410" alt="Fifteen hundred dots gather into a sunset over pine trees: bands of orange, pink and purple sky and an orange sun low behind pines in two greens"><br><sub>A PNG, drawn by the flock in at most eight of its colours.<br><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub></td>
</tr>
</table>

In 3D the default ramp is **ink**: from your terminal's own text colour to its background,
asked of the terminal at startup, so the flock is grey on a dark terminal and near black on a
light one. A picture is drawn best with `--shape dot -n 2000`. Without `--screensaver` a key
does not end a sign; with it, any key ends cbirds itself.

## 4. Come back

Any key, click or pointer movement, and cbirds quits at once, with no flight out, and tmux or
zsh gives you back your terminal. Input in the first half second is ignored.

## While you are here

When nobody is away, cbirds is a flock to watch and to play with.

### The flock

<p align="center"><img src="../../docs/demo.gif" width="768" alt="Three hundred ice-blue birds part around two scarlet hawks in slow motion and close up behind them"></p>

<p align="center"><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0 -n 300 -s 14</code></sub></p>

It opens by writing BOIDS, lets go, and flocks. Each bird keeps its distance, flies the way
its neighbours fly and drifts towards their middle, with a nudge away from the edges; its
turning is limited per frame, which gives curved fronts instead of a cloud snapping into
shape. Move the pointer into the flock and it scatters; `q` and it flies off the top. Left
alone for a minute it starts moving the sliders itself, a notch every four seconds, and any
key ends that.

```
cbirds -n 2000 -s 14          1 to 4096 birds (800), 4 to 64 pixels (30)
cbirds --flocks 3             1 to 3 flocks that keep to their own kind
cbirds --hawks 2              0 to 4 hawks: the flock parts around them and closes behind
cbirds --preset murmuration   or swarm, or storm
cbirds --seed 5               the same flock on every system
cbirds --color aurora         theme, ember, ice, acid, matrix, aurora,
                              prism, potion, dusk, ash, ink
cbirds --shape dot            bird, arrow, plane, dot
cbirds --sprite bird.png      any PNG up to 4 MB, kept in its own colours
cbirds --depth --trails       a second sky further off, and faint tails
cbirds --matrix               it is raining birds
```

`theme` is the terminal's own colours, asked of the terminal. `h` opens the panel, and
`--panel` opens it from the start: sliders for boundary, separation, alignment, turning,
perception and speed (0.2x to 2.6x), each 0 to 12 notches, lowercase lowers and uppercase
raises.

| key | does | key | does |
|---|---|---|---|
| `b`/`B` `s`/`S` `a`/`A` `t`/`T` `p`/`P` `v`/`V` `g`/`G` | a slider one notch down / up | `h` | the panel |
| `Space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+` `-` | more birds, fewer | `k` `K` | a hawk more, one fewer |
| `e` | tails | `Enter` | letters off, or home (with text) |
| `q` | quit | | |

### Anything can fly

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clips/pipe-sunset.gif" width="410" alt="A terminal window titled cat sunset.ans | cbirds: a pixel-art sunset in coloured blocks, a purple sky, a pale sun, a dark sea with a road of light, shatters into a cloud of coloured pixels that flies and puts the picture back exactly"><br><sub><code>cat docs/clips/sunset.ans | cbirds</code>, a picture drawn in █ characters</sub></td>
<td width="50%" align="center"><img src="../../docs/clips/pipe-banner.gif" width="410" alt="A terminal window titled cat banner.ans | cbirds: the word CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti, flies as a flock and builds the word again"><br><sub><code>cat docs/clips/banner.ans | cbirds</code>, a banner made for the clip</sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-code.gif" width="410" alt="A terminal window titled pygmentize boids.c | cbirds: the C function hawk_vector, syntax-highlighted, takes flight letter by letter, flies as a flock of characters and lands back as code"><br><sub><code>pygmentize boids.c | cbirds</code>, cbirds reading its own code</sub></td>
<td align="center"><img src="../../docs/clips/pipe-git.gif" width="410" alt="A terminal window titled git log --graph | cbirds: the commit graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of characters and lands back"><br><sub><code>git log --graph --color=always | cbirds</code>, cbirds' own history</sub></td>
</tr>
</table>

Pipe text into cbirds and the text is the flock: every letter is a bird. It opens with the
text exactly as the command printed it, in its own colours (24-bit, 256 and the 16 colours);
fastfetch's and neofetch's layout, with the logo beside the information, is kept, because
cbirds reads the output through a small terminal emulator of its own. A few seconds later one
letter leaves, the ones near it follow, and the letters fly as a flock. After twenty seconds
or so they are called home, and every letter lands on its own cell, exactly. Then it begins
again.

`Enter` sends the letters off, or calls them home at once. The pointer lifts the letters it
touches. Hawks work, and fly over text as arrows. `--text FILE` reads a file and `--text -`
reads standard input; a pipe with nothing in it gives the ordinary flock. The fetch screen,
the banner and the sunset are texts made for these clips; the donut, the code and the git log
are the real output of the commands named.

### Three dimensions

`cbirds --3d` takes the flock off the plane and into a sky: 2000 birds over a roost, seen from
a camera that goes once round it every two minutes, so the depth shows even in a still frame.
A nearer bird is bigger and brighter; a bird flying at the camera is short, one flying across
it long. The same three rules, but a bird heeds its seven nearest neighbours however far off
they are, as starlings do (Ballerini et al., PNAS 2008), rather than everything within a
radius. It turns at a limited rate a second, so it banks. A roost calls it home when it
strays, and the air above the roost moves slowly, which folds the flock into sheets and
ribbons (Hildenbrandt, Carere and Hemelrijk, Behavioral Ecology 2010).

Birds are 8 pixels by default, 5 in the text renderers. A terminal that does not answer for
its colours, or whose text is the colour of its ground, and every recording, get `ash` instead
of ink; `--color ink` works in the flat sky as well, and any other ramp can be asked for, as in
`cbirds --3d --color prism`. Hawks hunt the flock through the air (`--hawks`, `k`/`K`). The
pointer is a stick poked into the sky: birds near the line from the camera through it get out
of the way. In the panel `boundary` becomes `roost` and `perception` becomes `neighbours` (1
to 13). Every renderer draws it, and `--screensaver`, the keys and recording go with it. It
replaces `--depth` and draws no tails; it is one flock, so `--flocks` and `--matrix` are
refused, and so are a text, a sign and `--link`.

### One sky, several terminals

<p align="center"><img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each titled cbirds --link --color prism: each writes BOIDS in coloured blocks, then the birds stream out of one window and in through the edge of the next, and hawks cross between them"></p>

<p align="center"><sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub></p>

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the next, at the same
height and in the same direction; hawks cross too. Two terminals side by side, or two panes of
a split, become one sky. The windows lie in a row in the order they were started, so start
them left to right; only an edge with a window behind it is open, and the outer edges are
walls. Each window keeps its own settings (its own `--birds`, colours, renderer). A sign goes
with it; so does `--screensaver`. The windows talk through Unix sockets in a directory that is
yours alone (`$XDG_RUNTIME_DIR/cbirds`, or `$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`).
`--link` is refused with `--text`, text piped in, `--3d`, `--record` and `--bench`.

### Signs, the clock and pictures

`--say TEXT` has the flock write TEXT and hold it as a sign for thirty to forty-five seconds;
then it lets go and writes it again, and the rest of the flock wheels round the sign.
`--font-size ROWS` sets the letters from 4 to 10 rows tall. `--clock` is the time and
`--seconds` adds the seconds; `--clock-at HH:MM[:SS]` starts the clock at a time you choose,
which is how the clip at the top was made. `--picture FILE` has the flock draw a PNG, in at
most eight of its colours, or in a ramp if `--color` is given; `--shape dot -n 2000` draw it
best. Birds scattered by the pointer or a hawk come back.

### Terminals

Braille by default, in every terminal; no terminal is guessed at. `--render sextants` and
`--render blocks` are bolder text versions (sextants need a font from 2020 or later; blocks
work everywhere). Only the cells that changed are sent, and the background is never painted,
so the flock wears your theme. `--render kitty` draws real sprites over the Kitty graphics
protocol, made for Kitty and Ghostty. WezTerm, Konsole, iTerm2, Warp and Rio answer for the
protocol and then draw too few birds, the wrong ones or none; inside tmux the sprites never
reach the terminal.

### Recording

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

`--record` needs no terminal: it runs headless and writes the GIF with its own encoder (sprites,
or the cells with `--render braille | sextants | blocks`). A name ending in `.cast` gives an
asciinema recording. `--record-size COLSxROWS` (40x14 to 400x120, default 96x26),
`--record-fps` (a GIF carries up to 50) and `--record-seconds` (default 6, 34 for text) set
its size, rate and length. `--snapshot FILE` writes the last live frame as a PNG. Every clip in this repository was
recorded by cbirds itself.

### Numbers

With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000 about three;
braille, the default, about three and ten, since every frame is painted and sampled down to
dots. In 3D, 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites,
measured on one desktop machine. `cbirds --bench 300` prints the numbers on yours. The PNG,
GIF and DEFLATE code is all in the repository; there is no zlib, no libpng, no ncurses.

The [manual](../../docs/manual.md) has the rest.

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

cbirds is listed among the screensavers of [Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers)
and [Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers), and in
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT. See [LICENSE](../../LICENSE). By Alessandro Palliccia ([clainstone](https://github.com/clainstone)).

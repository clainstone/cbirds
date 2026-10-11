<div align="center">

<img src="../../docs/clips/pipe-banner.gif" width="796" alt="A terminal window titled cat banner.ans | cbirds: the word CBIRDS in a big block font with a rainbow across it, and under it a flock of birds in your terminal and $ fastfetch | cbirds; the blocks scatter as rainbow confetti, fly as a flock and build the word again">

```
cat docs/clips/banner.ans | cbirds
```

<sub>SCENE 1, THE OPENING TITLES: a banner made for this clip scatters as rainbow confetti and builds the word again</sub>

# cbirds

**A flock of birds in your terminal, in fourteen scenes.**<br>
Every clip on this page was recorded by cbirds itself.

<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux and macOS">
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>

**Programme**<br>
1 the opening titles &emsp;
2 [the donut](#scene-2--the-donut) &emsp;
3 [the code reads itself](#scene-3--the-code-reads-itself) &emsp;
4 [the fetch](#scene-4--the-fetch) &emsp;
5 [the sunset](#scene-5--the-sunset) &emsp;
6 [the history](#scene-6--the-history)<br>
[intermission: install](#intermission)<br>
7 [the third dimension](#scene-7--the-third-dimension) &emsp;
8 [the hunt](#scene-8--the-hunt) &emsp;
9 [dots only](#scene-9--dots-only) &emsp;
10 [one sky](#scene-10--one-sky) &emsp;
11 [the sign](#scene-11--the-sign) &emsp;
12 [the clock](#scene-12--the-clock) &emsp;
13 [the picture](#scene-13--the-picture) &emsp;
14 [the flock](#scene-14--the-flock)<br>
[also showing](#also-showing) &emsp; [programme notes](#programme-notes) &emsp; [behind the scenes](#behind-the-scenes) &emsp; [credits](#credits)

</div>

<div align="center">

## Scene 2 — the donut

```
./donut | cbirds
```

<img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds: a donut drawn in the characters .,-~:;=!*#$@, warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its own cell">

One frame of a spinning donut ([docs/clips/donut.c](../../docs/clips/donut.c), after Andy Sloane's donut.c) breaks into a flock of characters, swirls, and every character lands back on its own cell.

</div>

<div align="center">

## Scene 3 — the code reads itself

```
pygmentize boids.c | cbirds
```

<img src="../../docs/clips/pipe-code.gif" width="830" alt="A terminal window titled pygmentize boids.c | cbirds: the C function hawk_vector, syntax-highlighted, takes flight letter by letter, flies as a flock of characters and lands back as code">

cbirds reads its own source, the function that makes the flock flee a hawk, and the code takes flight letter by letter in its own colours before it lands back.

</div>

<div align="center">

## Scene 4 — the fetch

```
cat docs/clips/fetch.ans | cbirds
```

<img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A terminal window titled cat fetch.ans | cbirds: a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks; the letters and the swallow's dots fly off as a flock while the colour blocks stay, then everything lands back">

A screen in the manner of fastfetch, made for this clip, opens as it was printed, then its letters and the swallow's dots fly while the colour blocks stay; `fastfetch | cbirds` does the same with yours.

</div>

<div align="center">

## Scene 5 — the sunset

```
cat docs/clips/sunset.ans | cbirds
```

<img src="../../docs/clips/pipe-sunset.gif" width="796" alt="A terminal window titled cat sunset.ans | cbirds: a pixel-art sunset in coloured blocks, a purple sky, a pale sun, a dark sea with a road of light, shatters into a cloud of coloured pixels that flies and puts the picture back exactly">

A picture drawn in █ characters for this clip shatters into a cloud of coloured pixels that flies and puts the picture back exactly.

</div>

<div align="center">

## Scene 6 — the history

```
git log --graph --color=always | cbirds
```

<img src="../../docs/clips/pipe-git.gif" width="796" alt="A terminal window titled git log --graph | cbirds: the commit graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of characters and lands back as the log">

cbirds' own history, at the point where four feature tracks were merged, takes flight in its own colours and lands back as it was printed.

</div>

<div align="center">

## Intermission

**Install it, and the show runs in your own terminal.**

</div>

Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

<details>
<summary><b>apt</b>, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64</summary>

From [clainstone.com/apt](https://clainstone.com/apt):

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

Both Homebrew and apt come with completions for bash, zsh and fish, and `brew upgrade` or
`sudo apt upgrade` keeps them up to date. The `.deb` files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest).

</details>

<details>
<summary><b>From source</b>, with a C compiler and make and nothing else</summary>

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
<summary><b>Uninstall</b></summary>

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

<div align="center">

## Scene 7 — the third dimension

```
cbirds --3d --color prism --hawks 1
```

<img src="../../docs/clips/3d-prism.gif" width="668" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in rainbow colours, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk">

Two thousand birds over their roost, seen from a camera that goes once round it every two minutes, the colour running red near to blue far through a ribbon that folds round a hawk.

</div>

<div align="center">

## Scene 8 — the hunt

```
cbirds --3d --color aurora --hawks 1
```

<img src="../../docs/clips/3d-aurora.gif" width="668" alt="A terminal window titled cbirds --3d --color aurora --hawks 1: a murmuration in greens and blues stretches into a ribbon, curls and thins while a scarlet hawk flies through it">

The hawk hunts through the air, here a scarlet one through a murmuration in aurora greens and blues.

</div>

<div align="center">

## Scene 9 — dots only

```
cbirds --3d --render braille --color dusk
```

<img src="../../docs/clips/3d-braille.gif" width="668" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration drawn in braille dots, pink, violet and peach, thick where the flock is dense, and a cyan hawk">

The same sky in braille dots, pink, violet and peach, as a terminal with no graphics protocol shows it.

</div>

<div align="center">

## Scene 10 — one sky

```
cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1
```

<img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each titled cbirds --link --color prism: each writes BOIDS in coloured blocks, then the birds stream out of one window and in through the edge of the next, and hawks cross between them">

Three real windows, each running the command above and started left to right, share one sky: birds that fly out of the edge of one fly in through the edge of the next, at the same height and in the same direction, and hawks cross too.

</div>

<div align="center">

## Scene 11 — the sign

```
cbirds --say "back in five"
```

<img src="../../docs/say.gif" width="512" alt="Ember birds write BACK IN FIVE on two lines and hover in their places while the rest of the flock wheels round the sign">

The flock writes it and hovers on it, and the rest of the flock wheels round the sign.

</div>

<div align="center">

## Scene 12 — the clock

```
cbirds --clock-at 10:09:52
```

<img src="../../docs/clock.gif" width="512" alt="Ember birds write 10:09 while the rest of the flock wheels round; at the new minute the two digits that change let go and other birds write 10:10">

The flock writes 10:09, and at the new minute only the digits that change let go while other birds write 10:10; `--clock` is the time now.

</div>

<div align="center">

## Scene 13 — the picture

```
cbirds --picture docs/scene.png --shape dot -n 1500 -s 16
```

<img src="../../docs/picture.gif" width="512" alt="Fifteen hundred dots gather into a sunset over pine trees: bands of orange, pink and purple sky and an orange sun low behind pines in two greens">

Fifteen hundred dots gather into a sunset over pine trees, a PNG drawn by the flock in at most eight of its colours.

</div>

<div align="center">

## Scene 14 — the flock

```
cbirds --render kitty --hawks 2 --color ice --speed 0 -n 300 -s 14
```

<img src="../../docs/demo.gif" width="768" alt="Three hundred ice-blue birds part around two scarlet hawks in slow motion and close up behind them">

Craig Reynolds' boids, drawn as sprites: three hundred birds part around two scarlet hawks and close up behind them, at a fifth of the pace.

<br>

<kbd>q</kbd> &nbsp;and the flock flies off the top.

<sub>THE END</sub>

</div>

## Also showing

<table>
<tr>
<td width="33%" align="center"><img src="../../docs/prism.gif" width="256" alt="Three flocks in red, green and violet write BOIDS together, let go, and cross one another, each keeping to its own kind"><br><sub><code>cbirds --render kitty --flocks 3 --color prism</code></sub></td>
<td width="33%" align="center"><img src="../../docs/matrix.gif" width="256" alt="Green birds with tails falling through the screen like rain"><br><sub><code>cbirds --matrix</code></sub></td>
<td width="33%" align="center"><img src="../../docs/depth.gif" width="256" alt="Two skies: ice-blue birds with tails in front, smaller and dimmer birds drifting slower behind them"><br><sub><code>cbirds --depth --trails --color ice</code></sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/murmuration.gif" width="256" alt="A murmuration in braille dots, peach through to indigo"><br><sub><code>cbirds --preset murmuration --color dusk</code></sub></td>
<td align="center"><img src="../../docs/sextants.gif" width="256" alt="The flock in acid-green sextant blocks"><br><sub><code>cbirds --render sextants --color acid</code></sub></td>
<td align="center"><img src="../../docs/braille.gif" width="256" alt="The flock and two hawks in ice-blue braille dots, as any terminal shows it"><br><sub><code>cbirds --hawks 2 --color ice</code></sub></td>
</tr>
</table>

## Programme notes

<details>
<summary><b>The flock</b>: what it does, its options and its keys</summary>

It opens by writing BOIDS, lets go, and flocks. Move the pointer into the flock and it
scatters; `q` and it flies off the top. Left alone for a minute it starts moving the
sliders itself, a notch every four seconds; any key ends that. Each bird keeps its
distance, flies the way its neighbours fly and drifts towards their middle, with a nudge
away from the edges; its turning is limited per frame, which gives curved fronts instead
of a cloud snapping into shape.

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
perception and speed (0.2x to 2.6x), each 0 to 12 notches.

| key | does |
|---|---|
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

</details>

<details>
<summary><b>Anything can fly</b>: text piped in (scenes 1 to 6)</summary>

```
fastfetch | cbirds
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

Pipe text into cbirds and the text is the flock: every letter is a bird. It opens with the
text exactly as the command printed it, in its own colours (24-bit, 256 and the 16
colours); fastfetch's and neofetch's layout, with the logo beside the information, is
kept, because cbirds reads the output through a small terminal emulator of its own. A few
seconds later one letter leaves, the ones near it follow, and the letters fly as a flock.
After twenty seconds or so they are called home, and every letter lands on its own cell,
exactly. Then it begins again.

`Enter` sends the letters off, or calls them home at once. The pointer lifts the letters
it touches. Hawks work, and fly over text as arrows. `--text FILE` reads a file and
`--text -` reads standard input; a pipe with nothing in it gives the ordinary flock.

</details>

<details>
<summary><b>--3d</b>: three dimensions (scenes 7 to 9)</summary>

The flock leaves the plane for a sky: a murmuration of 2000 birds over its roost, seen from
a camera that goes once round it every two minutes, so the depth shows even in a still
frame. A nearer bird is bigger and brighter; a bird flying at the camera is short, one
flying across it long.

The same three rules, but a bird heeds its seven nearest neighbours however far off they
are, as starlings do (Ballerini et al., PNAS 2008), rather than everything within a
radius. It turns at a limited rate a second, so it banks. A roost calls it home when it
strays and the air above the roost moves slowly, which folds the flock into sheets and
ribbons (Hildenbrandt, Carere and Hemelrijk, Behavioral Ecology 2010).

By default there are 2000 birds, of 8 pixels (5 in the text renderers), in **ink**: a ramp
from your terminal's own text colour to its background, asked of the terminal at startup,
so the flock is grey on a dark terminal and near black on a light one. A terminal that does
not answer, or whose text is the colour of its ground, and every recording, get `ash`.
`--color ink` works in the flat sky as well, and any other ramp can be asked for:
`cbirds --3d --color prism`.

Hawks hunt the flock through the air (`--hawks`, `k`/`K`). The pointer is a stick poked
into the sky: birds near the line from the camera through it get out of the way. In the
panel `boundary` becomes `roost` and `perception` becomes `neighbours` (1 to 13). Every
renderer draws it: braille, sextants, blocks, Kitty sprites. `--screensaver` goes with it,
and so do the keys and recording. It replaces `--depth` and draws no tails; it is one
flock, so `--flocks` and `--matrix` are refused, and so are a text, a sign and `--link`.

</details>

<details>
<summary><b>--link</b>: one sky, several terminals (scene 10)</summary>

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

Birds that fly out of the edge of one window fly in through the edge of the next, at the
same height and in the same direction; hawks cross too. Two terminals side by side, or two
panes of a split, become one sky. The windows lie in a row in the order they were started,
so start them left to right; only an edge with a window behind it is open, and the outer
edges are walls. Each window keeps its own settings (its own `--birds`, colours,
renderer). A sign goes with it; so does `--screensaver`. The windows talk through Unix
sockets in a directory that is yours alone (`$XDG_RUNTIME_DIR/cbirds`, or
`$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`). It is refused with `--text`, text piped in,
`--3d`, `--record` and `--bench`.

</details>

<details>
<summary><b>Signs, the clock, pictures</b> (scenes 11 to 13)</summary>

`--say TEXT`: the flock writes TEXT and holds it as a sign for thirty to forty-five
seconds, lets go, and writes it again; the rest of the flock wheels round the sign.
`--font-size ROWS` sets the letters from 4 to 10 rows tall.

`--clock`: the time, HH:MM, in local time; `--seconds` adds the seconds. At each new minute
only the digits that change let go and are written again. `--clock-at HH:MM[:SS]` starts it
at a time you choose.

`--picture FILE`: the flock draws a PNG, in at most eight of its colours, or in a ramp if
`--color` is given. `--shape dot -n 2000` draw it best.

A key does not end a sign; birds scattered by the pointer or a hawk come back.

</details>

<details>
<summary><b>The lock screen</b>: <code>--screensaver</code></summary>

`--screensaver` quits at once on any key, click or pointer movement, with no flight out
(input in the first half second is ignored). It goes with everything else. In tmux:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

In zsh:

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

</details>

<details>
<summary><b>Projection</b>: renderers and terminals</summary>

Braille by default, in every terminal; no terminal is guessed at. `--render sextants` and
`--render blocks` are bolder text versions (sextants need a font from 2020 or later; blocks
work everywhere). Only the cells that changed are sent, and the background is never
painted, so the flock wears your theme.

`--render kitty` draws real sprites over the Kitty graphics protocol, made for Kitty and
Ghostty. WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol and then draw too
few birds, the wrong ones or none; inside tmux the sprites never reach the terminal.

</details>

<details>
<summary><b>Running costs</b></summary>

With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000 about three;
braille, the default, about three and ten, since every frame is painted and sampled down to
dots. In 3D, 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites,
measured on one desktop machine. `cbirds --bench 300` prints the numbers on yours.

At startup the bird is rotated into sixty headings, three wing positions and every shade of
the ramp: about fifteen hundred small images. The wings beat six times a second, and now and
then a bird glides. One program in C99, no dependencies: the PNG, GIF and DEFLATE code is
all in the repository; there is no zlib, no libpng, no ncurses. Linux and macOS.

</details>

The [manual](../../docs/manual.md) has the rest: [the panel](../../docs/manual.md#the-panel),
[text](../../docs/manual.md#anything-can-fly),
[signs](../../docs/manual.md#signs-the-clock-and-pictures),
[one sky](../../docs/manual.md#one-sky-several-terminals) and [options](../../docs/manual.md#options).

## Behind the scenes

Every clip in this repository was recorded by cbirds itself. `--record` needs no terminal:
it runs headless and writes the GIF with its own encoder, with sprites, or the cells with
`--render braille | sextants | blocks`.

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

A name ending in `.cast` gives an asciinema recording. `--record-size COLSxROWS` (40x14 to
400x120, default 96x26), `--record-fps` (a GIF carries up to 50) and `--record-seconds`
(default 6, 34 for text) set the shot; a recording of text runs a whole cycle and ends on
the text as it was printed. `--snapshot FILE` writes the last live frame as a PNG.

The framed clips are the same frames inside a drawn terminal window whose title bar names
the command. Scene 10 is three real `cbirds --link` windows, recorded through cbirds' own
terminal emulator. The fetch screen, the banner and the sunset are texts made for their
clips; the donut, the code and the git log are the real output of the commands named.

## Credits

<div align="center">

<table>
<tr><td align="right"><b>The flock</b></td><td>Craig Reynolds, <i>Flocks, Herds, and Schools: A Distributed Behavioral Model</i>, SIGGRAPH 1987; <a href="https://www.red3d.com/cwr/boids/">red3d.com/cwr/boids</a></td></tr>
<tr><td align="right"><b>Seven neighbours, in 3D</b></td><td>Ballerini et al., <i>Interaction ruling animal collective behavior depends on topological rather than metric distance: Evidence from a field study</i>, PNAS 2008</td></tr>
<tr><td align="right"><b>The roost, in 3D</b></td><td>Hildenbrandt, Carere and Hemelrijk, <i>Self-organized aerial displays of thousands of starlings: a model</i>, Behavioral Ecology 2010</td></tr>
<tr><td align="right"><b>Sprites</b></td><td>the <a href="https://sw.kovidgoyal.net/kitty/graphics-protocol/">Kitty graphics protocol</a></td></tr>
<tr><td align="right"><b>The donut</b></td><td>after Andy Sloane's donut.c (2006)</td></tr>
<tr><td align="right"><b>Every clip</b></td><td>recorded by cbirds itself</td></tr>
<tr><td align="right"><b>Direction</b></td><td>Alessandro Palliccia (<a href="https://github.com/clainstone">clainstone</a>)</td></tr>
</table>

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

<sub>Listed in [Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).</sub>

</div>

## Licence

MIT. See [LICENSE](../../LICENSE).

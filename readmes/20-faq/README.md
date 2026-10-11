# cbirds

**A flock of birds in your terminal, and the questions people ask about it.**

<p align="center"><img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds: a donut drawn in the characters .,-~:;=!*#$@, warm red to gold, breaks into a flock of characters that swirls across the window, and every character lands back on its own cell"></p>

## Is that a donut?

Yes. It is one frame of a spinning donut, printed by
[docs/clips/donut.c](../../docs/clips/donut.c) (after Andy Sloane's donut.c) and piped into
cbirds:

```
./donut | cbirds
```

Every character is a bird. A few seconds in, one leaves, the ones near it follow, and the
characters fly as a flock; after twenty seconds or so they are called home, and every one lands
back on its own cell, exactly. Then it begins again. Anything a terminal prints can do this.

<table>
<tr>
<td width="50%">

- [So what is cbirds?](#so-what-is-cbirds)
- [How do I install it?](#how-do-i-install-it)
- [Will it run in my terminal?](#will-it-run-in-my-terminal)
- [Can it fly my fastfetch?](#can-it-fly-my-fastfetch)
- [What else can I pipe into it?](#what-else-can-i-pipe-into-it)
- [Does it really do 3D in a terminal?](#does-it-really-do-3d-in-a-terminal)
- [Can two terminals share it?](#can-two-terminals-share-it)
- [Can it lock my terminal?](#can-it-lock-my-terminal)
- [Can it write something, or draw a picture?](#can-it-write-something-or-draw-a-picture)

</td>
<td width="50%">

- [Can I steer the flock?](#can-i-steer-the-flock)
- [Can I change how it looks?](#can-i-change-how-it-looks)
- [How were these clips made?](#how-were-these-clips-made)
- [How much CPU does it take?](#how-much-cpu-does-it-take)
- [What does it need?](#what-does-it-need)
- [Who made it?](#who-made-it)
- [What is the licence?](#what-is-the-licence)
- [How do I uninstall it?](#how-do-i-uninstall-it)
- [Where is the rest?](#where-is-the-rest)

</td>
</tr>
</table>

## So what is cbirds?

A flock of birds in your terminal: Craig Reynolds' boids, drawn in braille in any terminal, and
as real sprites over the Kitty graphics protocol in Kitty and Ghostty. It opens by writing
BOIDS, lets go, and flocks. Each bird keeps its distance, flies the way its neighbours fly and
drifts towards their middle, with a nudge away from the edges; its turning is limited per
frame, which gives curved fronts instead of a cloud snapping into shape. Move the pointer into
the flock and it scatters; press `q` and it flies off the top.

<p align="center"><img src="../../docs/demo.gif" width="768" alt="Three hundred ice-blue birds part around two scarlet hawks in slow motion and close up behind them"></p>

<p align="center"><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0 -n 300 -s 14</code>: two hawks, at a fifth of the pace</sub></p>

## How do I install it?

With Homebrew, on macOS and Linux:

```
brew install clainstone/tap/cbirds
```

With apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64, from
[clainstone.com/apt](https://clainstone.com/apt):

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

Both come with completions for bash, zsh and fish, and `brew upgrade` or `sudo apt upgrade`
keeps them up to date. The `.deb` files alone are on the
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

That installs one file, `/usr/local/bin/cbirds`; `make install PREFIX="$HOME/.local"` puts it
in your home. `make test` runs the tests.

</details>

## Will it run in my terminal?

Yes. cbirds draws in braille by default, in every terminal; no terminal is guessed at.
`--render sextants` and `--render blocks` are bolder text versions: sextants need a font from
2020 or later, and blocks work everywhere. Only the cells that changed are sent, and the
background is never painted, so the flock wears your theme.

Real sprites, `--render kitty`, are drawn over the Kitty graphics protocol and made for **Kitty**
and **Ghostty**. WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol and then draw
too few birds, the wrong ones or none, and inside tmux the sprites never reach the terminal;
braille is the answer there.

<table>
<tr>
<td width="33%" align="center"><img src="../../docs/braille.gif" width="270" alt="The flock and two hawks in ice-blue braille dots, as any terminal shows it"><br><sub>braille, the default<br><code>cbirds --hawks 2 --color ice</code></sub></td>
<td width="33%" align="center"><img src="../../docs/sextants.gif" width="270" alt="The flock in acid-green sextant blocks"><br><sub>sextants<br><code>cbirds --render sextants --color acid</code></sub></td>
<td width="33%" align="center"><img src="../../docs/hawks.gif" width="270" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, ice blue"><br><sub>sprites, in Kitty and Ghostty<br><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
</tr>
</table>

## Can it fly my fastfetch?

Yes:

```
fastfetch | cbirds
```

cbirds reads what is piped in through a small terminal emulator of its own, so it opens with
the screen exactly as fastfetch printed it: the logo beside the information, every colour in
place (24-bit, 256 and the 16 colours). neofetch's layout is kept too. Then the letters take
flight, and after twenty seconds or so every one of them lands on its own cell, exactly.

<p align="center"><img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A terminal window titled cat fetch.ans | cbirds: a swallow drawn in braille in a cyan-to-pink gradient beside a list of settings and two rows of colour blocks; the letters and the swallow's dots fly while the colour blocks stay, then everything lands back"></p>

<p align="center"><sub><code>cat docs/clips/fetch.ans | cbirds</code>: a screen in the manner of fastfetch, made for this clip. <code>fastfetch | cbirds</code> does the same with yours.</sub></p>

## What else can I pipe into it?

Anything a terminal prints. Every letter is a bird, and it lands back where it was printed.

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clips/pipe-code.gif" width="410" alt="A terminal window titled pygmentize boids.c | cbirds: the C function hawk_vector, syntax-highlighted, takes flight letter by letter, flies as a flock of characters and lands back as code"><br><sub>cbirds reading its own code<br><code>pygmentize boids.c | cbirds</code></sub></td>
<td width="50%" align="center"><img src="../../docs/clips/pipe-git.gif" width="410" alt="A terminal window titled git log --graph | cbirds: the commit graph of cbirds' own history, coloured lanes and yellow hashes, takes flight as a flock of characters and lands back"><br><sub>cbirds' own history<br><code>git log --graph --color=always | cbirds</code></sub></td>
</tr>
<tr>
<td align="center"><img src="../../docs/clips/pipe-sunset.gif" width="410" alt="A terminal window titled cat sunset.ans | cbirds: a pixel-art sunset in coloured blocks, a purple sky, a pale sun, a dark sea with a road of light, shatters into a cloud of coloured pixels that flies and puts the picture back exactly"><br><sub>a picture drawn in █ characters, made for the clip<br><code>cat docs/clips/sunset.ans | cbirds</code></sub></td>
<td align="center"><img src="../../docs/clips/pipe-banner.gif" width="410" alt="A terminal window titled cat banner.ans | cbirds: the word CBIRDS in a big block font with a rainbow across it scatters as rainbow confetti, flies as a flock and builds the word again"><br><sub>a banner made for the clip<br><code>cat docs/clips/banner.ans | cbirds</code></sub></td>
</tr>
</table>

```
git log --oneline --graph --color=always | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

`--text FILE` reads a file, and `--text -` reads standard input; a pipe with nothing in it
gives the ordinary flock. `Enter` sends the letters off, or calls them home at once. The
pointer lifts the letters it touches. Hawks work, and fly over text as arrows.

## Does it really do 3D in a terminal?

Yes, even in braille.

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/clips/3d-prism.gif" width="410" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in rainbow colours, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk"><br><sub>sprites: the colour runs with depth, red near to blue far<br><code>cbirds --3d --color prism --hawks 1</code></sub></td>
<td width="50%" align="center"><img src="../../docs/clips/3d-braille.gif" width="410" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration drawn in braille dots, pink, violet and peach, thick where the flock is dense, and a cyan hawk"><br><sub>braille, in a terminal with no graphics protocol<br><code>cbirds --3d --render braille --color dusk</code></sub></td>
</tr>
</table>

`cbirds --3d` takes the flock off the plane and into a sky: a murmuration of 2000 birds over
its roost, seen from a camera that goes once round it every two minutes, so the depth shows
even in a still frame. A nearer bird is bigger and brighter; a bird flying at the camera is
short, one flying across it long.

The rules are the same three, but a bird heeds its seven nearest neighbours however far off
they are, as starlings do (Ballerini et al., PNAS 2008), rather than everything within a
radius. It turns at a limited rate a second, so it banks. A roost calls it home when it strays,
and the air above the roost moves slowly, which folds the flock into sheets and ribbons
(Hildenbrandt, Carere and Hemelrijk, Behavioral Ecology 2010).

By default the birds are 8 pixels (5 in the text renderers) and in **ink**, a ramp from your
terminal's own text colour to its background, asked of the terminal at startup: grey on a dark
terminal, near black on a light one. A terminal that does not answer, or whose text is the
colour of its ground, and every recording, get `ash`. `cbirds --3d --color prism` asks for any
other ramp, and `--color ink` works in the flat sky as well. Hawks hunt the flock through the
air (`--hawks`, `k`/`K`); the pointer is a stick poked into the sky, and birds near the line
from the camera through it get out of the way. In the panel `boundary` becomes `roost` and
`perception` becomes `neighbours` (1 to 13). `--screensaver`, the keys and recording go with
it. It replaces `--depth` and draws no tails; it is one flock, so `--flocks` and `--matrix` are
refused, and so are a text, a sign and `--link`.

## Can two terminals share it?

Yes. Start one in each, left to right:

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

<p align="center"><img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each titled cbirds --link --color prism: each writes BOIDS in coloured blocks, then the birds stream out of one window and in through the edge of the next, and hawks cross between them"></p>

<p align="center"><sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub></p>

Birds that fly out of the edge of one window fly in through the edge of the next, at the same
height and in the same direction; hawks cross too. Two terminals side by side, or two panes of
a split, become one sky. The windows lie in a row in the order they were started; only an edge
with a window behind it is open, and the outer edges are walls. Each window keeps its own
settings (its own `--birds`, colours, renderer); a sign goes with it, and so does
`--screensaver`. The windows talk through Unix sockets in a directory that is yours alone
(`$XDG_RUNTIME_DIR/cbirds`, or `$TMPDIR/cbirds-UID`, or `/tmp/cbirds-UID`). `--link` is
refused with `--text`, text piped in, `--3d`, `--record` and `--bench`.

## Can it lock my terminal?

Yes. `--screensaver` quits at once on any key, click or pointer movement, with no flight out
(input in the first half second is ignored), and it goes with everything else. With `--clock`
it is a lock screen that tells the time.

<p align="center"><img src="../../docs/clock.gif" width="512" alt="Ember birds write 10:09 while the rest of the flock wheels round; at the new minute the two digits that change let go and other birds write 10:10"></p>

<p align="center"><sub><code>cbirds --clock-at 10:09:52</code>: at the new minute only the digits that change let go</sub></p>

In tmux:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

In zsh:

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

## Can it write something, or draw a picture?

Yes. `--say TEXT` has the flock write TEXT and hold it as a sign for thirty to forty-five
seconds; then it lets go and writes it again, and the rest of the flock wheels round the sign.
`--font-size ROWS` sets the letters from 4 to 10 rows tall. `--clock` writes the time, HH:MM,
in local time; `--seconds` adds the seconds, and `--clock-at HH:MM[:SS]` starts it at a time
you choose. `--picture FILE` has the flock draw a PNG, in at most eight of its colours, or in a
ramp if `--color` is given; `--shape dot -n 2000` draw it best. A key does not end a sign, and
birds scattered by the pointer or a hawk come back.

<table>
<tr>
<td width="50%" align="center"><img src="../../docs/say.gif" width="410" alt="Ember birds write BACK IN FIVE on two lines and hover in their places while the rest of the flock wheels round the sign"><br><sub><code>cbirds --say "back in five"</code></sub></td>
<td width="50%" align="center"><img src="../../docs/picture.gif" width="410" alt="Fifteen hundred dots gather into a sunset over pine trees: bands of orange, pink and purple sky and an orange sun low behind pines in two greens"><br><sub><code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub></td>
</tr>
</table>

## Can I steer the flock?

Yes. The pointer scatters it, and the keys do the rest. `h` opens a panel of sliders, and
`--panel` opens it from the start: boundary, separation, alignment, turning, perception and
speed (0.2x to 2.6x), each 0 to 12 notches; lowercase lowers, uppercase raises. Left alone for
a minute, cbirds starts moving the sliders itself, a notch every four seconds, and any key ends
that.

| key | does | key | does |
|---|---|---|---|
| `b`/`B` `s`/`S` `a`/`A` `t`/`T` `p`/`P` `v`/`V` `g`/`G` | a slider one notch down / up | `h` | the panel |
| `Space` | pause | `.` | one frame |
| `0` | back to the defaults | `Tab` | next preset |
| `+` `-` | more birds, fewer | `k` `K` | a hawk more, one fewer |
| `e` | tails | `Enter` | letters off, or home (with text) |
| `q` | quit | | |

## Can I change how it looks?

Yes:

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

`theme` is your terminal's own colours, asked of the terminal.

<table>
<tr>
<td width="33%" align="center"><img src="../../docs/prism.gif" width="256" alt="Three flocks in red, green and violet write BOIDS together, let go, and cross one another, each keeping to its own kind"><br><sub><code>cbirds --render kitty --flocks 3 --color prism</code></sub></td>
<td width="33%" align="center"><img src="../../docs/depth.gif" width="256" alt="Two skies: ice-blue birds with tails in front, smaller and dimmer birds drifting slower behind them"><br><sub><code>cbirds --depth --trails --color ice</code></sub></td>
<td width="33%" align="center"><img src="../../docs/matrix.gif" width="256" alt="Green birds with tails falling through the screen like rain"><br><sub><code>cbirds --matrix</code></sub></td>
</tr>
</table>

## How were these clips made?

By cbirds itself. `--record` needs no terminal: it runs headless and writes the GIF with its
own encoder, with sprites, or the cells with `--render braille | sextants | blocks`. A name
ending in `.cast` gives an asciinema recording.

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --snapshot frame.png --frames 400
cbirds --3d --record murmuration.gif --color prism
```

`--record-size COLSxROWS` (40x14 to 400x120, default 96x26), `--record-fps` (a GIF carries up
to 50) and `--record-seconds` (default 6, 34 for text) set the shot; a recording of text runs a
whole cycle and ends on the text as it was printed. `--snapshot FILE` writes the last live frame
as a PNG. The framed clips are the same frames inside a drawn terminal window whose title bar
names the command. The fetch screen, the banner and the sunset are texts made for their clips;
the donut, the code and the git log are the real output of the commands named.

## How much CPU does it take?

With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000 about
three; braille, the default, about three and ten, since every frame is painted and sampled down
to dots. In 3D, 2000 birds cost about 3 ms a frame in braille and 1.7 ms with Kitty sprites,
measured on one desktop machine. `cbirds --bench 300` prints the numbers on yours.

## What does it need?

A terminal, on Linux or macOS. cbirds is one program in C99 with no dependencies: the PNG, GIF
and DEFLATE code is all in the repository, and there is no zlib, no libpng, no ncurses. To
build it you need a C compiler and make; it installs one file. At startup the bird is rotated
into sixty headings, three wing positions and every shade of the ramp, about fifteen hundred
small images; the wings beat six times a second, and now and then a bird glides.

## Who made it?

Alessandro Palliccia ([clainstone](https://github.com/clainstone)).

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

It stands on:

- Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*, SIGGRAPH 1987;
  [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/)
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

## What is the licence?

MIT. See [LICENSE](../../LICENSE).

## How do I uninstall it?

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

## Where is the rest?

In the [manual](../../docs/manual.md): [the panel](../../docs/manual.md#the-panel),
[text](../../docs/manual.md#anything-can-fly),
[signs, the clock and pictures](../../docs/manual.md#signs-the-clock-and-pictures),
[one sky](../../docs/manual.md#one-sky-several-terminals) and
[the options](../../docs/manual.md#options).

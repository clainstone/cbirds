```
CBIRDS(1)                   General Commands Manual                   CBIRDS(1)
```

## NAME

**cbirds** — a flock of birds in your terminal

## SYNOPSIS

**cbirds** [*option* ...]\
*command* | **cbirds** [*option* ...]\
**cbirds** **--text** *file* [*option* ...]\
**cbirds** **--3d** [*option* ...]\
**cbirds** **--link** [*option* ...]\
**cbirds** **--say** *text* | **--clock** | **--picture** *file* [*option* ...]\
**cbirds** **--record** *file* [*option* ...]

<p align="center">
<img src="../../docs/clips/pipe-code.gif" width="830" alt="A terminal window titled pygmentize boids.c | cbirds. C source in monokai colours, the function that makes the flock flee a hawk, comes apart letter by letter into a flock of characters, flies across the window, and lands back as the same code">
<br>
<sub><code>pygmentize boids.c | cbirds</code>: cbirds reading its own code</sub>
</p>

## DESCRIPTION

**cbirds** draws Craig Reynolds' boids: in braille in any terminal, and as real
sprites over the Kitty graphics protocol in Kitty and Ghostty. It is one program
in C99 with no dependencies, for Linux and macOS; the PNG, GIF and DEFLATE code
is all in the repository, and there is no zlib, no libpng, no ncurses.

It opens by writing BOIDS, lets go, and flocks. Each bird keeps its distance,
flies the way its neighbours fly and drifts towards their middle, with a nudge
away from the edges; its turning is limited per frame, which gives curved fronts
instead of a cloud snapping into shape. Move the pointer into the flock and it
scatters; **q** and it flies off the top. Left alone for a minute, it starts
moving the sliders itself, a notch every four seconds; any key ends that.

### Text

When text is piped in, the text is the flock: every letter is a bird. **cbirds**
opens with the text exactly as the command printed it, in its own colours
(24-bit, 256 and the 16 colours), and keeps the layout of fastfetch and
neofetch, the logo beside the information, because it reads the output through
a small terminal emulator of its own. A few seconds later one letter leaves, the
ones near it follow, and the letters fly as a flock. After twenty seconds or so
they are called home, and every letter lands on its own cell, exactly. Then it
begins again. A pipe with nothing in it gives the ordinary flock.

### Three dimensions

**--3d** takes the flock off the plane into a sky: a murmuration of 2000 birds
over its roost, seen from a camera that goes once round it every two minutes, so
the depth shows even in a still frame. A nearer bird is bigger and brighter; a
bird flying at the camera is short, one flying across it long. The rules are the
same three, but a bird heeds its seven nearest neighbours however far off they
are, as starlings do (Ballerini et al., 2008), rather than everything within a
radius, and it turns at a limited rate a second, so it banks. A roost calls it
home when it strays, and the air above the roost moves slowly, which folds the
flock into sheets and ribbons (Hildenbrandt, Carere and Hemelrijk, 2010).

### One sky

**--link** makes the windows that run it one sky. Birds that fly out of the edge
of one window fly in through the edge of the next, at the same height and in the
same direction, and hawks cross too: two terminals side by side, or two panes of
a split. The windows lie in a row in the order they were started, so start them
from left to right; only an edge with a window behind it is open, and the outer
edges are walls. Each window keeps its own settings: its own **--birds**, colours
and renderer.

### Signs

**--say** has the flock write a text and hold it as a sign, while the rest of the
flock wheels round it; **--clock** writes the time, and **--picture** draws a PNG.
A key does not end a sign, and birds scattered by the pointer or a hawk come
back.

### Rendering

Braille is the default, in every terminal; no terminal is guessed at.
**--render sextants** and **--render blocks** are bolder text versions. In these
three, only the cells that changed are sent, and the background is never
painted, so the flock wears your theme. **--render kitty** draws real sprites
over the Kitty graphics protocol, made for Kitty and Ghostty. At startup the bird
is rotated into sixty headings, three wing positions and every shade of the
ramp, about fifteen hundred small images. The wings beat six times a second, and
now and then a bird glides.

## INSTALLATION

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

Both come with completions for bash, zsh and fish; **brew upgrade** and
**sudo apt upgrade** keep them up to date. The .deb files alone are on the
[release page](https://github.com/clainstone/cbirds/releases/latest).

From source, with a C compiler and make and nothing else:

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

That installs one file, */usr/local/bin/cbirds*; **make install
PREFIX="$HOME/.local"** puts it in your home. **make test** runs the tests.

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

From source: **sudo make uninstall** in the clone.

</details>

## OPTIONS

### The flock

<dl>
<dt><b>-n</b>, <b>--birds</b> <i>n</i></dt>
<dd>How many birds, 1 to 4096; 800 by default.</dd>
<dt><b>-s</b>, <b>--size</b> <i>pixels</i></dt>
<dd>How big a bird is, 4 to 64 pixels; 30 by default.</dd>
<dt><b>-g</b>, <b>--flocks</b> <i>n</i></dt>
<dd>1 to 3 flocks, each keeping to its own kind.</dd>
<dt><b>-k</b>, <b>--hawks</b> <i>n</i></dt>
<dd>0 to 4 predators. The flock parts around them and closes behind.</dd>
<dt><b>--preset</b> <i>murmuration</i> | <i>swarm</i> | <i>storm</i></dt>
<dd>Start from a preset; <b>Tab</b> moves to the next.</dd>
<dt><b>--seed</b> <i>n</i></dt>
<dd>The same flock on every system: the random numbers are cbirds' own.</dd>
<dt><b>--depth</b></dt>
<dd>A second sky further off: smaller, slower, dimmer birds.</dd>
<dt><b>-e</b>, <b>--trails</b></dt>
<dd>Faint tails.</dd>
<dt><b>--matrix</b></dt>
<dd>It is raining birds: green, falling, with tails.</dd>
<dt><b>--panel</b></dt>
<dd>Open the panel of sliders from the start; see KEYS.</dd>
</dl>

### Colour and shape

<dl>
<dt><b>-c</b>, <b>--color</b> <i>ramp</i></dt>
<dd>One of <i>theme</i> (the terminal's own colours, asked of the terminal), <i>ember</i>, <i>ice</i>, <i>acid</i>, <i>matrix</i>, <i>aurora</i>, <i>prism</i>, <i>potion</i>, <i>dusk</i>, <i>ash</i> and <i>ink</i> (a ramp from the terminal's own text colour to its background, asked of the terminal at startup).</dd>
<dt><b>--shape</b> <i>bird</i> | <i>arrow</i> | <i>plane</i> | <i>dot</i></dt>
<dd>The shape of a bird.</dd>
<dt><b>--sprite</b> <i>file</i></dt>
<dd>Any PNG up to 4 MB as the bird, kept in its own colours.</dd>
</dl>

### Rendering

<dl>
<dt><b>--render</b> <i>braille</i> | <i>sextants</i> | <i>blocks</i> | <i>kitty</i></dt>
<dd>Braille, the default, in every terminal. Sextants and blocks are bolder text versions: sextants need a font from 2020 or later, blocks work everywhere. Kitty draws real sprites over the Kitty graphics protocol, made for Kitty and Ghostty.</dd>
</dl>

### Text

<dl>
<dt><b>--text</b> <i>file</i></dt>
<dd>Fly the text of a file; <b>--text -</b> reads standard input. Text piped in needs no option.</dd>
</dl>

### Three dimensions

<dl>
<dt><b>--3d</b></dt>
<dd>The murmuration, over its roost. Defaults: 2000 birds of 8 pixels (5 in the text renderers), in <i>ink</i>; a terminal that does not answer, or whose text is the colour of its ground, and every recording, get <i>ash</i>. Hawks hunt the flock through the air, and the pointer is a stick poked into the sky: birds near the line from the camera through it get out of the way. Every renderer draws it, and <b>--screensaver</b>, the keys and recording go with it.</dd>
</dl>

### One sky

<dl>
<dt><b>--link</b></dt>
<dd>Share one sky with the other windows running <b>cbirds --link</b>, in a row in the order they were started. A sign goes with it, and so does <b>--screensaver</b>.</dd>
</dl>

### Signs

<dl>
<dt><b>--say</b> <i>text</i></dt>
<dd>Write <i>text</i> and hold it as a sign for thirty to forty-five seconds, let go, and write it again; the rest of the flock wheels round the sign.</dd>
<dt><b>--font-size</b> <i>rows</i></dt>
<dd>The height of the letters of a sign, 4 to 10 rows.</dd>
<dt><b>--clock</b></dt>
<dd>The time, HH:MM, local time. At each new minute only the digits that change let go and are written again.</dd>
<dt><b>--seconds</b></dt>
<dd>Add the seconds to the clock.</dd>
<dt><b>--clock-at</b> <i>HH:MM</i>[<i>:SS</i>]</dt>
<dd>Start the clock at a time you choose.</dd>
<dt><b>--picture</b> <i>file</i></dt>
<dd>Draw a PNG, in at most eight of its colours, or in a ramp if <b>--color</b> is given. <b>--shape dot -n 2000</b> draw it best.</dd>
</dl>

### Screensaver

<dl>
<dt><b>--screensaver</b></dt>
<dd>Quit at once on any key, click or pointer movement, with no flight out; input in the first half second is ignored. It goes with everything else.</dd>
</dl>

### Recording and measuring

<dl>
<dt><b>--record</b> <i>file</i></dt>
<dd>Run headless, with no terminal, and write a GIF with cbirds' own encoder: sprites, or the cells with <b>--render braille</b>, <b>sextants</b> or <b>blocks</b>. A name ending in <i>.cast</i> gives an asciinema recording.</dd>
<dt><b>--record-size</b> <i>cols</i><b>x</b><i>rows</i></dt>
<dd>40x14 to 400x120; 96x26 by default.</dd>
<dt><b>--record-fps</b> <i>n</i></dt>
<dd>Frames a second; a GIF carries up to 50.</dd>
<dt><b>--record-seconds</b> <i>n</i></dt>
<dd>6 by default, and 34 for text: a whole cycle, which ends on the text as it was printed.</dd>
<dt><b>--snapshot</b> <i>file</i></dt>
<dd>Write the last live frame as a PNG.</dd>
<dt><b>--bench</b> <i>n</i></dt>
<dd>Print what a frame costs on this machine, as in <b>cbirds --bench 300</b>.</dd>
</dl>

## KEYS

| key | does |
|:--|:--|
| **b**/**B** **s**/**S** **a**/**A** **t**/**T** **p**/**P** **v**/**V** **g**/**G** | a slider one notch down / up |
| **h** | the panel |
| **Space** | pause |
| **.** | one frame |
| **0** | back to the defaults |
| **Tab** | next preset |
| **+** **-** | more birds, fewer |
| **k** **K** | a hawk more, one fewer |
| **e** | tails |
| **Enter** | letters off, or home (with text) |
| **q** | quit |

The panel has the sliders boundary, separation, alignment, turning, perception
and speed (0.2x to 2.6x), each 0 to 12 notches; lowercase lowers, uppercase
raises. Under **--3d**, boundary becomes roost and perception becomes neighbours
(1 to 13). With text, the pointer lifts the letters it touches.

## EXAMPLES

Fly what a command prints. The clip flies a screen in the manner of fastfetch,
made for it; **fastfetch | cbirds** does the same with yours, logo beside the
information:

```
fastfetch | cbirds
cat docs/clips/fetch.ans | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-fetch.gif" width="796" alt="A terminal window titled cat fetch.ans | cbirds. A swallow drawn in braille, cyan to pink, beside the lines flock@sky, Birds: 2000, Mode: --3d, over a roost, Camera: once round in 2 min, Neighbours: 7 nearest, Ramp: ink, Render: braille, Hawks: 1 and Seed: 45, above two rows of colour blocks. The letters and the bird's dots fly off as a flock and land back; the colour blocks stay">
</p>

Fly the history of a repository, here cbirds' own, at the point where four
feature tracks were merged:

```
git log --graph --color=always | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-git.gif" width="796" alt="A terminal window titled git log --graph | cbirds. cbirds' own history, graph lanes in colour beside yellow commit hashes and their messages, takes flight letter by letter, flies as a flock and lands back line for line">
</p>

Fly one frame of a spinning donut, printed by
[docs/clips/donut.c](../../docs/clips/donut.c), after Andy Sloane's donut.c (2006):

```
./donut | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-donut.gif" width="830" alt="A terminal window titled ./donut | cbirds. A donut drawn in the characters .,-~:;=!*#$@, warm red in the shade and gold in the light, breaks apart into a flock of characters that swirls across the window; then every character lands back on its own cell">
</p>

Fly a banner and a picture drawn in █, both made for the clips:

```
cat docs/clips/banner.ans | cbirds
cat docs/clips/sunset.ans | cbirds
```

<p align="center">
<img src="../../docs/clips/pipe-banner.gif" width="410" alt="A terminal window titled cat banner.ans | cbirds. CBIRDS in a big block font with a rainbow across it; the blocks scatter as rainbow confetti, fly, and rebuild the word">
<img src="../../docs/clips/pipe-sunset.gif" width="410" alt="A terminal window titled cat sunset.ans | cbirds. A pixel-art sunset in coloured full blocks, a purple sky, the sun, the sea and its road of light, shatters into coloured pixels that fly and put the picture back together exactly">
</p>

Fly cbirds' own code, as in the clip under SYNOPSIS; or a file, or standard
input:

```
pygmentize boids.c | cbirds
figlet -f big hello | cbirds
cbirds --text poem.txt
```

A murmuration in three dimensions, in prism, round a hawk:

```
cbirds --3d --color prism --hawks 1
```

<p align="center">
<img src="../../docs/clips/3d-prism.gif" width="668" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in three dimensions, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk">
</p>

The same sky in braille, as a terminal with no graphics protocol shows it:

```
cbirds --3d --render braille --color dusk
```

<p align="center">
<img src="../../docs/clips/3d-braille.gif" width="668" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration in braille dots, pink, violet and peach, with a cyan hawk">
</p>

One sky across terminals. Start them from left to right:

```
cbirds --link        # in one terminal
cbirds --link        # in another
```

<p align="center">
<img src="../../docs/clips/link.gif" width="830" alt="Three terminal windows side by side, each running cbirds --link: each writes BOIDS, then a prism-coloured flock drawn in sextant blocks flows out of one window and into the next, and hawks cross between them">
<br>
<sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub>
</p>

Part the flock round two hawks, at a fifth of its pace; and three flocks that
keep to their own kind:

```
cbirds --render kitty --hawks 2 --color ice --speed 0
cbirds --render kitty --flocks 3 --color prism
```

<p align="center">
<img src="../../docs/hawks.gif" width="410" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, in ice blue and in slow motion">
<img src="../../docs/prism.gif" width="410" alt="Three flocks in red, green and violet write BOIDS together, let go, and cross one another, each keeping to its own kind">
</p>

A sign, the time, a picture:

```
cbirds --say "back in five"
cbirds --clock-at 10:09:52
cbirds --picture docs/scene.png --shape dot -n 1500 -s 16
```

<p align="center">
<img src="../../docs/say.gif" width="270" alt="The flock writes BACK IN FIVE on two lines and hovers on it while the rest of the flock wheels round the sign">
<img src="../../docs/clock.gif" width="270" alt="The flock writes 10:09; when the minute turns, only the last two digits let go and other birds write 10:10">
<img src="../../docs/picture.gif" width="270" alt="Fifteen hundred dots gather into a sunset over pine trees">
</p>

Lock tmux with a clock:

```
set -g lock-after-time 300
set -g lock-command "cbirds --screensaver --clock"
```

The same in zsh:

```
TMOUT=300
TRAPALRM() { cbirds --screensaver --clock }
```

Record with no terminal: a GIF, an asciinema cast, a murmuration:

```
cbirds --record flock.gif --hawks 2 --seed 5
cbirds --record flock.cast --record-fps 30
cbirds --3d --record murmuration.gif --color prism
```

## FILES

<dl>
<dt>/usr/local/bin/cbirds</dt>
<dd>The program, as <b>sudo make install</b> puts it; <b>make install PREFIX="$HOME/.local"</b> puts it in your home instead.</dd>
<dt>$XDG_RUNTIME_DIR/cbirds, or $TMPDIR/cbirds-UID, or /tmp/cbirds-UID</dt>
<dd>A directory that is yours alone, with the Unix sockets through which <b>--link</b> windows talk.</dd>
<dt><a href="../../docs/manual.md">docs/manual.md</a></dt>
<dd>The manual, in the repository.</dd>
<dt><a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, <a href="../../docs/clips/fetch.ans">fetch.ans</a>, <a href="../../docs/clips/banner.ans">banner.ans</a>, <a href="../../docs/clips/sunset.ans">sunset.ans</a></dt>
<dd>The donut, and the texts made for the clips on this page.</dd>
</dl>

## ENVIRONMENT

<dl>
<dt>XDG_RUNTIME_DIR</dt>
<dd><b>--link</b> keeps its sockets in <i>$XDG_RUNTIME_DIR/cbirds</i>.</dd>
<dt>TMPDIR</dt>
<dd>Failing that, in <i>$TMPDIR/cbirds-UID</i>, and failing both, in <i>/tmp/cbirds-UID</i>.</dd>
</dl>

## NOTES

With sprites, 800 birds cost about half a millisecond of CPU a frame and 4000
about three; braille, the default, about three and ten, since every frame is
painted and sampled down to dots. In 3D, 2000 birds cost about 3 ms a frame in
braille and 1.7 ms with Kitty sprites, measured on one desktop machine.
**cbirds --bench 300** prints the numbers on yours.

Every clip on this page was recorded by cbirds itself.

## CAVEATS

Sprites are made for Kitty and Ghostty. WezTerm, Konsole, iTerm2, Warp and Rio
answer for the protocol and then draw too few birds, the wrong ones or none;
inside tmux the sprites never reach the terminal. Braille works in every
terminal.

**--link** is refused with **--text**, text piped in, **--3d**, **--record** and
**--bench**.

**--3d** replaces **--depth** and draws no tails. It is one flock: **--flocks**
and **--matrix** are refused, and so are a text, a sign and **--link**.

## SEE ALSO

[docs/manual.md](../../docs/manual.md): [the panel](../../docs/manual.md#the-panel),
[anything can fly](../../docs/manual.md#anything-can-fly),
[signs, the clock and pictures](../../docs/manual.md#signs-the-clock-and-pictures),
[one sky, several terminals](../../docs/manual.md#one-sky-several-terminals),
[options](../../docs/manual.md#options).

[docs/README.md](../../docs/README.md): the commands behind the older clips.

**tmux**(1), **zsh**(1).

Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*,
SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/).

Ballerini et al., *Interaction ruling animal collective behavior depends on
topological rather than metric distance: Evidence from a field study*, PNAS 2008.

Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of thousands
of starlings: a model*, Behavioral Ecology 2010.

[The Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/).

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## AUTHORS

Alessandro Palliccia ([clainstone](https://github.com/clainstone)),
[github.com/clainstone/cbirds](https://github.com/clainstone/cbirds).

Much of cbirds' code, tests and documentation was written by large language
models (Claude, by Anthropic), under my direction.

The donut in the donut clip is after Andy Sloane's donut.c (2006).

## COPYRIGHT

MIT licence; see [LICENSE](../../LICENSE).

```
cbirds                 a flock of birds in your terminal              CBIRDS(1)
```

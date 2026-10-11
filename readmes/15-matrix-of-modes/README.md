<h1 align="center">cbirds</h1>

<p align="center">
a flock of birds in your terminal<br>
<sub>modes &nbsp;×&nbsp; renderers &nbsp;×&nbsp; ramps</sub>
</p>

<p align="center">
<a href="https://github.com/clainstone/cbirds/actions/workflows/tests.yml"><img src="https://github.com/clainstone/cbirds/actions/workflows/tests.yml/badge.svg" alt="tests"></a>
<a href="https://github.com/clainstone/cbirds/tags"><img src="https://img.shields.io/github/v/tag/clainstone/cbirds?sort=semver&label=version" alt="version"></a>
<a href="../../LICENSE"><img src="https://img.shields.io/github/license/clainstone/cbirds" alt="MIT licence"></a>
<img src="https://img.shields.io/badge/C-99-00599C?logo=c&logoColor=white" alt="C99">
<img src="https://img.shields.io/badge/dependencies-none-brightgreen" alt="no dependencies">
<img src="https://img.shields.io/badge/platform-Linux%20%7C%20macOS-lightgrey" alt="Linux and macOS">
<img src="https://img.shields.io/badge/sprites-Kitty%20%7C%20Ghostty-8A2BE2" alt="sprites in Kitty and Ghostty">
</p>

Craig Reynolds' boids, drawn in braille in any terminal, and as real sprites
over the Kitty graphics protocol in Kitty and Ghostty. One program in C99, no
dependencies. Below, everything it does, crossed: what flies, how it is drawn,
and the colours it wears. Every clip was recorded by cbirds itself.

## Modes × renderers

<table>
<tr>
<th width="50%">the flock &nbsp; <code>cbirds</code></th>
<th width="50%">three dimensions &nbsp; <code>cbirds --3d</code></th>
</tr>
<tr>
<th colspan="2" align="left">sprites &nbsp; <sub><code>--render kitty</code>, in Kitty and Ghostty</sub></th>
</tr>
<tr>
<td><img src="../../docs/hawks.gif" width="380" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, in ice blue"></td>
<td><img src="../../docs/clips/3d-prism.gif" width="380" alt="A terminal window titled cbirds --3d --color prism --hawks 1: two thousand birds in three dimensions, red near the camera to blue far off, fold into a ribbon that bends round a pale hawk"></td>
</tr>
<tr>
<td><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
<td><sub><code>cbirds --3d --color prism --hawks 1</code></sub></td>
</tr>
<tr>
<th colspan="2" align="left">braille &nbsp; <sub>the default, in every terminal</sub></th>
</tr>
<tr>
<td><img src="../../docs/braille.gif" width="380" alt="The flock in braille dots, ice blue, with two red hawks"></td>
<td><img src="../../docs/clips/3d-braille.gif" width="380" alt="A terminal window titled cbirds --3d --render braille --color dusk: the murmuration in braille dots, pink, violet and peach, with a cyan hawk"></td>
</tr>
<tr>
<td><sub><code>cbirds --hawks 2 --color ice</code></sub></td>
<td><sub><code>cbirds --3d --render braille --color dusk</code></sub></td>
</tr>
<tr>
<th colspan="2" align="left">sextants &nbsp; <sub><code>--render sextants</code>, with a font from 2020 or later</sub></th>
</tr>
<tr>
<td><img src="../../docs/sextants.gif" width="380" alt="The flock drawn in sextant blocks, in acid greens"></td>
<td align="center"><sub>every renderer draws it;<br>no clip of this one here</sub></td>
</tr>
<tr>
<td><sub><code>cbirds --render sextants --color acid</code></sub></td>
<td><sub><code>cbirds --3d --render sextants</code></sub></td>
</tr>
</table>

`--render blocks`, a bolder text version that works everywhere, has no clip here.
In braille, sextants and blocks only the cells that changed are sent, and the
background is never painted, so the flock wears your theme.

## Text × where it came from

<table>
<tr>
<th width="50%">the real output of the command</th>
<th width="50%">a text made for the clip</th>
</tr>
<tr>
<td><img src="../../docs/clips/pipe-donut.gif" width="380" alt="A terminal window titled ./donut | cbirds. A donut drawn in the characters .,-~:;=!*#$@, warm red in the shade and gold in the light, breaks apart into a flock of characters that swirls across the window; then every character lands back on its own cell"><br><code>./donut | cbirds</code><br><sub>one frame of the spinning donut, <a href="../../docs/clips/donut.c">docs/clips/donut.c</a>, after Andy Sloane's donut.c (2006)</sub></td>
<td><img src="../../docs/clips/pipe-fetch.gif" width="380" alt="A terminal window titled cat fetch.ans | cbirds. A swallow drawn in braille, cyan to pink, beside the lines flock@sky, Birds: 2000, Mode: --3d, over a roost, Camera: once round in 2 min, Neighbours: 7 nearest, Ramp: ink, Render: braille, Hawks: 1 and Seed: 45, above two rows of colour blocks. The letters and the bird's dots fly off and land back; the colour blocks stay"><br><code>cat docs/clips/fetch.ans | cbirds</code><br><sub>a screen in the manner of fastfetch; <code>fastfetch | cbirds</code> does the same with yours</sub></td>
</tr>
<tr>
<td><img src="../../docs/clips/pipe-code.gif" width="380" alt="A terminal window titled pygmentize boids.c | cbirds. C source in monokai colours, the function that makes the flock flee a hawk, takes flight letter by letter and lands back as the same code"><br><code>pygmentize boids.c | cbirds</code><br><sub>cbirds reading its own code</sub></td>
<td><img src="../../docs/clips/pipe-banner.gif" width="380" alt="A terminal window titled cat banner.ans | cbirds. CBIRDS in a big block font with a rainbow across it; the blocks scatter as rainbow confetti and rebuild the word"><br><code>cat docs/clips/banner.ans | cbirds</code><br><sub>a banner made for the clip</sub></td>
</tr>
<tr>
<td><img src="../../docs/clips/pipe-git.gif" width="380" alt="A terminal window titled git log --graph | cbirds. cbirds' own history, graph lanes in colour beside yellow commit hashes and their messages, takes flight letter by letter and lands back line for line"><br><code>git log --graph --color=always | cbirds</code><br><sub>cbirds' own history, where four feature tracks were merged</sub></td>
<td><img src="../../docs/clips/pipe-sunset.gif" width="380" alt="A terminal window titled cat sunset.ans | cbirds. A pixel-art sunset in coloured full blocks shatters into a cloud of coloured pixels that flies and puts the picture back together exactly"><br><code>cat docs/clips/sunset.ans | cbirds</code><br><sub>a picture drawn in █ characters</sub></td>
</tr>
</table>

Every text goes through the same cycle:

| from the start | a few seconds in | then | after twenty seconds or so |
|:--|:--|:--|:--|
| the text exactly as the command printed it, in its own colours: 24-bit, 256 or the 16 | one letter leaves, and the ones near it follow | the letters fly as a flock | they are called home, and every letter lands on its own cell, exactly; then it begins again |

fastfetch's and neofetch's layout, with the logo beside the information, is
kept, because cbirds reads the output through a small terminal emulator of its
own. `--text FILE` reads a file and `--text -` standard input; a pipe with
nothing in it gives the ordinary flock. A recording of text runs a whole cycle,
34 seconds, and ends on the text as it was printed. The
[manual](../../docs/manual.md#anything-can-fly) has the rest.

## Signs and one sky × how they were drawn

<table>
<tr>
<th align="left">signs &nbsp; <code>--say</code> &nbsp;<code>--clock</code> &nbsp;<code>--picture</code> &nbsp; <sub>all three recorded with sprites</sub></th>
</tr>
<tr>
<td align="center">
<img src="../../docs/say.gif" width="250" alt="The flock writes BACK IN FIVE on two lines and hovers on it while the rest of the flock wheels round the sign">
<img src="../../docs/clock.gif" width="250" alt="The flock writes 10:09; when the minute turns, only the last two digits let go and other birds write 10:10">
<img src="../../docs/picture.gif" width="250" alt="Fifteen hundred dots gather into a sunset over pine trees">
<br>
<sub><code>cbirds --say "back in five"</code> &nbsp;·&nbsp; <code>cbirds --clock-at 10:09:52</code> &nbsp;·&nbsp; <code>cbirds --picture docs/scene.png --shape dot -n 1500 -s 16</code></sub>
</td>
</tr>
<tr>
<th align="left">one sky &nbsp; <code>cbirds --link</code>, in each window &nbsp; <sub>drawn here in sextants; each window keeps its own renderer</sub></th>
</tr>
<tr>
<td align="center">
<img src="../../docs/clips/link.gif" width="770" alt="Three terminal windows side by side, each running cbirds --link: each writes BOIDS, then a prism-coloured flock drawn in sextant blocks flows out of one window and into the next, and hawks cross between them">
<br>
<sub>Three real windows, each running <code>cbirds --link --color prism --render sextants -n 220 -s 11 --hawks 1</code>, recorded through cbirds' own terminal emulator.</sub>
</td>
</tr>
</table>

`--say` holds its text for thirty to forty-five seconds, lets go, and writes it
again; `--clock` lets go only of the digits that change. A key does not end a
sign, and birds scattered by the pointer or a hawk come back.
Birds that fly out of the edge of one `--link` window fly in through the edge of
the next, at the same height and in the same direction; start the windows from
left to right. More in the manual:
[signs](../../docs/manual.md#signs-the-clock-and-pictures),
[one sky](../../docs/manual.md#one-sky-several-terminals).

## Ramps × modes

`-c`, `--color`, with a swatch of each ramp, and the clips that wear it.

<table>
<tr>
<th width="24%">ramp</th>
<th width="38%">the flock</th>
<th width="38%"><code>--3d</code></th>
</tr>
<tr>
<td><b>theme</b><br><sub>the terminal's own colours, asked of the terminal</sub></td>
<td align="center">—</td>
<td align="center">—</td>
</tr>
<tr>
<td><b>ember</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FFD68A?style=flat-square" height="20" alt="pale gold"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FFB042?style=flat-square" height="20" alt="amber"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-F77A29?style=flat-square" height="20" alt="orange"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-E04A27?style=flat-square" height="20" alt="red-orange"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-AD2C33?style=flat-square" height="20" alt="deep red"></td>
<td><img src="../../docs/flocks.gif" width="300" alt="Three flocks in ember colours, gold, orange and deep red, each keeping to its own kind"><br><sub><code>cbirds --flocks 3 --color ember</code></sub></td>
<td><img src="../../docs/clips/3d-ember.gif" width="300" alt="An ember murmuration closes into a ring round a cyan hawk, then folds into arcs and sheets"><br><sub><code>cbirds --3d --color ember --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>ice</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-E2F6FF?style=flat-square" height="20" alt="ice white"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-A0DCFA?style=flat-square" height="20" alt="pale blue"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-60B0EC?style=flat-square" height="20" alt="sky blue"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-3A7ACE?style=flat-square" height="20" alt="blue"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-2C4A9E?style=flat-square" height="20" alt="deep blue"></td>
<td><img src="../../docs/hawks.gif" width="300" alt="The flock writes BOIDS, lets go, and parts around two scarlet hawks, in ice blue"><br><sub><code>cbirds --render kitty --hawks 2 --color ice --speed 0</code></sub></td>
<td><img src="../../docs/clips/3d-ice.gif" width="300" alt="A small ice-blue murmuration in three dimensions, with a hawk"><br><sub><code>cbirds --3d --color ice --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>acid</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-EEFFB0?style=flat-square" height="20" alt="pale yellow"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-BAF460?style=flat-square" height="20" alt="lime"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-76D64A?style=flat-square" height="20" alt="green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-36B06C?style=flat-square" height="20" alt="sea green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-1A7A6A?style=flat-square" height="20" alt="teal"></td>
<td><img src="../../docs/sextants.gif" width="300" alt="The flock drawn in sextant blocks, in acid greens"><br><sub><code>cbirds --render sextants --color acid</code></sub></td>
<td align="center">—</td>
</tr>
<tr>
<td><b>matrix</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-C6FFC6?style=flat-square" height="20" alt="pale green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-78F678?style=flat-square" height="20" alt="light green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-36D246?style=flat-square" height="20" alt="green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-1A9630?style=flat-square" height="20" alt="dark green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-0C5C1E?style=flat-square" height="20" alt="deep green"></td>
<td align="center">—</td>
<td align="center">—</td>
</tr>
<tr>
<td><b>aurora</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-CEFFDE?style=flat-square" height="20" alt="mint white"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-6EF0AA?style=flat-square" height="20" alt="mint"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-2CCCAA?style=flat-square" height="20" alt="teal"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-3C8CD2?style=flat-square" height="20" alt="blue"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-6E54C8?style=flat-square" height="20" alt="violet"></td>
<td align="center">—</td>
<td><img src="../../docs/clips/3d-aurora.gif" width="300" alt="A murmuration in aurora greens and blues, with a scarlet hawk cutting through it"><br><sub><code>cbirds --3d --color aurora --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>prism</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FF5C5C?style=flat-square" height="20" alt="red"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FFC440?style=flat-square" height="20" alt="gold"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-60DC6E?style=flat-square" height="20" alt="green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-50A0FF?style=flat-square" height="20" alt="blue"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-B06EFF?style=flat-square" height="20" alt="violet"></td>
<td><img src="../../docs/prism.gif" width="300" alt="Three flocks in red, green and violet write BOIDS together, then cross each other, each keeping to its own kind"><br><sub><code>cbirds --render kitty --flocks 3 --color prism</code></sub></td>
<td><img src="../../docs/clips/3d-prism.gif" width="300" alt="Two thousand birds in three dimensions, red near the camera to blue far off, folding into a ribbon round a pale hawk"><br><sub><code>cbirds --3d --color prism --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>potion</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-AAFF6E?style=flat-square" height="20" alt="lime"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-48D668?style=flat-square" height="20" alt="green"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-CEA0FF?style=flat-square" height="20" alt="lilac"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-A068F0?style=flat-square" height="20" alt="violet"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-7640CE?style=flat-square" height="20" alt="deep violet"></td>
<td align="center">—</td>
<td><img src="../../docs/clips/3d-potion.gif" width="300" alt="A small murmuration in violet and green"><br><sub><code>cbirds --3d --color potion --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>dusk</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FFD6AA?style=flat-square" height="20" alt="peach"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-FF9478?style=flat-square" height="20" alt="salmon"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-E85688?style=flat-square" height="20" alt="pink"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-A046AA?style=flat-square" height="20" alt="plum"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-5C40A8?style=flat-square" height="20" alt="indigo"></td>
<td><img src="../../docs/murmuration.gif" width="300" alt="The murmuration preset in braille dots, from peach through pink to indigo"><br><sub><code>cbirds --preset murmuration --color dusk</code></sub></td>
<td><img src="../../docs/clips/3d-dusk.gif" width="300" alt="A small murmuration from peach to violet, with a cyan hawk"><br><sub><code>cbirds --3d --color dusk --hawks 1</code></sub></td>
</tr>
<tr>
<td><b>ash</b><br><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-F4F4F6?style=flat-square" height="20" alt="white"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-CED0D6?style=flat-square" height="20" alt="light grey"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-A4A8B2?style=flat-square" height="20" alt="grey"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-7C808C?style=flat-square" height="20" alt="slate grey"><img src="https://img.shields.io/badge/%E2%A0%80%E2%A0%80-585C68?style=flat-square" height="20" alt="dark slate"></td>
<td align="center">—</td>
<td><img src="../../docs/clips/3d-ash.gif" width="300" alt="A small murmuration in greys, as ink looks on a dark terminal"><br><sub><code>cbirds --3d --color ash --hawks 1</code>: how <b>ink</b> looks on a dark terminal</sub></td>
</tr>
<tr>
<td><b>ink</b><br><sub>your terminal's own text colour, fading to its background, asked of the terminal at startup</sub></td>
<td align="center"><sub><code>--color ink</code> works in the flat sky as well</sub></td>
<td align="center"><sub>the default: grey on a dark terminal, near black on a light one; a terminal that does not answer, and every recording, get <b>ash</b></sub></td>
</tr>
</table>

<sub>— no clip of this pair here.</sub>

The flat flock has two more looks of its own:

<table>
<tr>
<td width="50%"><img src="../../docs/depth.gif" width="380" alt="A second sky of smaller, dimmer ice-blue birds behind the first, with faint tails"></td>
<td width="50%"><img src="../../docs/matrix.gif" width="380" alt="Green birds fall down the screen with tails: it is raining birds"></td>
</tr>
<tr>
<td align="center"><code>cbirds --depth --trails --color ice</code><br><sub>a second sky further off: smaller, slower, dimmer birds; and faint tails</sub></td>
<td align="center"><code>cbirds --matrix</code><br><sub>it is raining birds: green, falling, with tails</sub></td>
</tr>
</table>

## Modes × what goes with them

|  | hawks | the pointer | `--screensaver` | `--record` | `--link` |
|:--|:--|:--|:--|:--|:--|
| **the flock** | ✓ it parts round them and closes behind | ✓ it scatters | ✓ | ✓ | ✓ |
| **`--3d`** | ✓ they hunt it through the air | ✓ a stick poked into the sky | ✓ | ✓ | ✗ refused |
| **text** | ✓ they fly over it as arrows | ✓ it lifts the letters it touches | ✓ | ✓ a whole cycle, 34 s | ✗ refused |
| **signs** | ✓ scattered birds come back | ✓ scattered birds come back | ✓ | ✓ | ✓ the sign goes with it |

`--3d` replaces `--depth` and draws no tails; it is one flock, so `--flocks` and
`--matrix` are refused, and so are a text, a sign and `--link`. `--link` is
also refused with `--bench`. `--screensaver` quits at once on any key, click or
pointer movement, with no flight out; input in the first half second is
ignored.

## Renderers × what they need

| renderer | option | needs | notes |
|:--|:--|:--|:--|
| braille | the default | any terminal | no terminal is guessed at |
| sextants | `--render sextants` | a font from 2020 or later | a bolder text version |
| blocks | `--render blocks` | nothing: works everywhere | a bolder text version |
| sprites | `--render kitty` | Kitty or Ghostty | real sprites over the Kitty graphics protocol. WezTerm, Konsole, iTerm2, Warp and Rio answer for the protocol and then draw too few birds, the wrong ones or none; inside tmux the sprites never reach the terminal |

In `--3d` a bird is 8 pixels by default, and 5 in the text renderers.

## Renderers × birds: the cost of a frame

| | the flock, 800 birds | the flock, 4000 birds | `--3d`, 2000 birds <sub>(measured on one desktop machine)</sub> |
|:--|:--|:--|:--|
| **sprites** | about 0.5 ms | about 3 ms | 1.7 ms |
| **braille** | about 3 ms | about 10 ms | about 3 ms |

CPU a frame. Braille costs more because every frame is painted and sampled
down to dots. At startup the bird is rotated into sixty headings, three wing
positions and every shade of the ramp, about fifteen hundred small images; the
wings beat six times a second, and now and then a bird glides.
`cbirds --bench 300` prints the numbers on your machine.

## Options × what they take

<table>
<tr><th colspan="2" align="left">the flock</th></tr>
<tr><td><code>-n</code>, <code>--birds</code></td><td>1 to 4096 birds; 800 by default</td></tr>
<tr><td><code>-s</code>, <code>--size</code></td><td>4 to 64 pixels; 30 by default</td></tr>
<tr><td><code>-g</code>, <code>--flocks</code></td><td>1 to 3 flocks, each keeping to its own kind</td></tr>
<tr><td><code>-k</code>, <code>--hawks</code></td><td>0 to 4 predators</td></tr>
<tr><td><code>--preset</code></td><td><code>murmuration</code>, <code>swarm</code> or <code>storm</code></td></tr>
<tr><td><code>--seed N</code></td><td>the same flock on every system: cbirds' own random numbers</td></tr>
<tr><td><code>--depth</code>, <code>-e</code> <code>--trails</code>, <code>--matrix</code></td><td>a second sky further off; faint tails; it is raining birds</td></tr>
<tr><td><code>--panel</code></td><td>the panel of sliders, from the start</td></tr>
<tr><th colspan="2" align="left">the look</th></tr>
<tr><td><code>-c</code>, <code>--color</code></td><td><code>theme</code>, <code>ember</code>, <code>ice</code>, <code>acid</code>, <code>matrix</code>, <code>aurora</code>, <code>prism</code>, <code>potion</code>, <code>dusk</code>, <code>ash</code>, <code>ink</code></td></tr>
<tr><td><code>--shape</code></td><td><code>bird</code>, <code>arrow</code>, <code>plane</code> or <code>dot</code></td></tr>
<tr><td><code>--sprite FILE</code></td><td>any PNG up to 4 MB, kept in its own colours</td></tr>
<tr><td><code>--render</code></td><td><code>braille</code>, <code>sextants</code>, <code>blocks</code> or <code>kitty</code></td></tr>
<tr><th colspan="2" align="left">the modes</th></tr>
<tr><td><code>--3d</code></td><td>a murmuration of 2000 birds over its roost, seen from a camera that goes once round it every two minutes</td></tr>
<tr><td><code>--link</code></td><td>one sky across the windows that run it, in a row in the order they were started</td></tr>
<tr><td><code>--text FILE</code></td><td>a file as the flock; <code>--text -</code> reads standard input</td></tr>
<tr><td><code>--say TEXT</code>, <code>--font-size ROWS</code></td><td>a sign, held for thirty to forty-five seconds; letters 4 to 10 rows</td></tr>
<tr><td><code>--clock</code>, <code>--seconds</code>, <code>--clock-at HH:MM[:SS]</code></td><td>the time, HH:MM, local time; with the seconds; from a time you choose</td></tr>
<tr><td><code>--picture FILE</code></td><td>a PNG, in at most eight of its colours, or in a ramp if <code>--color</code> is given</td></tr>
<tr><td><code>--screensaver</code></td><td>quits on any key, click or pointer movement; goes with everything else</td></tr>
<tr><th colspan="2" align="left">recording and measuring</th></tr>
<tr><td><code>--record FILE</code></td><td>no terminal needed: a GIF, with sprites or, with <code>--render braille</code>, <code>sextants</code> or <code>blocks</code>, the cells; a name ending in <code>.cast</code> gives an asciinema recording</td></tr>
<tr><td><code>--record-size COLSxROWS</code></td><td>40x14 to 400x120; 96x26 by default</td></tr>
<tr><td><code>--record-fps</code></td><td>a GIF carries up to 50</td></tr>
<tr><td><code>--record-seconds</code></td><td>6 by default, 34 for text</td></tr>
<tr><td><code>--snapshot FILE</code></td><td>the last live frame, as a PNG</td></tr>
<tr><td><code>--bench</code></td><td><code>cbirds --bench 300</code> prints the numbers on your machine</td></tr>
</table>

## Keys × what they do

| key | does |
|:--|:--|
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
| <kbd>q</kbd> | quit: the flock flies off the top |

The panel's sliders are boundary, separation, alignment, turning, perception and
speed (0.2x to 2.6x), each 0 to 12 notches; lowercase lowers, uppercase raises.
Left alone for a minute, the flock moves them itself, a notch every four
seconds; any key ends that. More in [the manual](../../docs/manual.md#the-panel).

| mode | what changes |
|:--|:--|
| text | <kbd>Enter</kbd> sends the letters off, or calls them home at once; the pointer lifts the letters it touches; hawks fly over the text as arrows |
| `--3d` | in the panel, boundary becomes roost and perception becomes neighbours (1 to 13); hawks hunt through the air; the pointer is a stick poked into the sky; no tails |
| signs | a key does not end a sign; birds scattered by the pointer or a hawk come back |
| `--screensaver` | any key, click or pointer movement quits at once, with no flight out |

## Install × system

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

<details>
<summary>From source, and uninstalling</summary>

With a C compiler and make and nothing else:

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

That installs one file, `/usr/local/bin/cbirds`; `make install PREFIX="$HOME/.local"`
puts it in your home. `make test` runs the tests. To uninstall:

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

| | Homebrew | apt |
|:--|:--|:--|
| systems | macOS and Linux | Debian 12, Ubuntu 22.04 and later, and their derivatives; amd64 or arm64 |
| completions | bash, zsh and fish | bash, zsh and fish |
| updates | `brew upgrade` | `sudo apt upgrade` |

The `.deb` files alone are on the [release page](https://github.com/clainstone/cbirds/releases/latest).
No dependencies: the PNG, GIF and DEFLATE code is all in the repository, and
there is no zlib, no libpng, no ncurses.

## Credits

Much of cbirds' code, tests and documentation was written by large language
models (Claude, by Anthropic), under my direction.

| what | from |
|:--|:--|
| the three rules | Craig Reynolds, *Flocks, Herds, and Schools: A Distributed Behavioral Model*, SIGGRAPH 1987; [red3d.com/cwr/boids](https://www.red3d.com/cwr/boids/) |
| seven nearest neighbours, in `--3d` | Ballerini et al., *Interaction ruling animal collective behavior depends on topological rather than metric distance: Evidence from a field study*, PNAS 2008 |
| the roost and its slow air, in `--3d` | Hildenbrandt, Carere and Hemelrijk, *Self-organized aerial displays of thousands of starlings: a model*, Behavioral Ecology 2010 |
| sprites | [the Kitty graphics protocol](https://sw.kovidgoyal.net/kitty/graphics-protocol/) |
| the donut clip | after Andy Sloane's donut.c (2006) |

cbirds is listed in
[Awesome CLI Apps](https://github.com/agarrharr/awesome-cli-apps#screensavers),
[Awesome TUIs](https://github.com/rothgar/awesome-tuis#screensavers) and
[Awesome Ricing](https://github.com/fosslife/awesome-ricing#show-off-scripts).

## Licence

MIT, by Alessandro Palliccia ([clainstone](https://github.com/clainstone)). See [LICENSE](../../LICENSE).

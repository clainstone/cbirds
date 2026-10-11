<p align="center"><img src="../../docs/clips/pipe-donut.gif" width="820" alt="A donut drawn in characters from warm red to gold breaks into a flock of characters, swirls, and every character lands back on its own cell"></p>

# cbirds

A flock of birds in your terminal.

```
cbirds                          # a flock
fastfetch | cbirds              # the letters fly, and each lands back on its own cell
cbirds --3d                     # a murmuration of two thousand, in three dimensions
cbirds --link                   # in two terminals side by side: one sky
cbirds --screensaver --clock    # a lock screen that tells the time
```

<p align="center"><img src="../../docs/clips/3d-prism.gif" height="190" alt="Two thousand birds in three dimensions, coloured by depth from red near to blue far, bending round a hawk"> <img src="../../docs/clips/link.gif" height="190" alt="Three terminal windows side by side, a prism-coloured flock flowing out of one and into the next"></p>

Braille in any terminal; `--render kitty` draws sprites in Kitty and Ghostty.
`h` shows the sliders, `q` sends the flock off the top. The rest is in the
[manual](../../docs/manual.md).

## Install

```
brew install clainstone/tap/cbirds
```

<details>
<summary>apt, or from source</summary>

apt, on Debian 12, Ubuntu 22.04 and later and their derivatives, amd64 or arm64:

```
sudo apt install curl
curl -fsSL https://clainstone.com/apt/cbirds.gpg | sudo tee /etc/apt/keyrings/cbirds.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/cbirds.gpg] https://clainstone.com/apt stable main" | sudo tee /etc/apt/sources.list.d/cbirds.list
sudo apt update
sudo apt install cbirds
```

From source, with a C compiler and make and nothing else:

```
git clone https://github.com/clainstone/cbirds
cd cbirds
make
sudo make install
cbirds
```

</details>

## Credits

Much of cbirds' code, tests and documentation was written by large language models (Claude, by Anthropic), under my direction.

The flock is Craig Reynolds' boids: *Flocks, Herds, and Schools: A Distributed
Behavioral Model*, SIGGRAPH 1987. In three dimensions a bird heeds its seven
nearest neighbours, after Ballerini et al., PNAS 2008, and the slow air over its
roost folds the flock into sheets and ribbons, after Hildenbrandt, Carere and
Hemelrijk, Behavioral Ecology 2010. The donut is after Andy Sloane's donut.c (2006).

MIT, by Alessandro Palliccia: [LICENSE](../../LICENSE).

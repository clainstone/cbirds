# Cbirds: terminal high-Performance Boid Flocking Simulation in C

![Language](https://img.shields.io/badge/Language-C-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS-lightgrey.svg)

A high-performance implementation of Craig Reynolds' **Boids algorithm** in pure C, featuring real-time GPU-accelerated terminal graphics rendering. This flocking simulation brings autonomous agent behavior to life directly in your terminal using the Kitty Graphics Protocol.

Zero dependencies: no libraries beyond libc and libm, no asset files. The bird sprite is a PNG compiled into the binary, and every rotation frame is produced at startup by the PNG library included in the project.

![Cbirds Demo](./demo.gif)

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [How It Works](#how-it-works)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Technical Details](#technical-details)
- [Contributing](#contributing)
- [License](#license)

## Overview

**Cbirds** is a terminal-based flocking simulation that implements the classic Boids algorithm for simulating coordinated animal motion such as bird flocks or fish schools. Unlike traditional ASCII-based terminal animations, Cbirds leverages the **Kitty Graphics Protocol** to render hundreds of high-quality PNG sprites with full 360-degree rotation, achieving smooth, GPU-accelerated animation at 60+ FPS.

### What are Boids?

Boids is an artificial life program developed by Craig Reynolds in 1986 that simulates the flocking behavior of birds. Each "boid" (bird-oid object) follows three simple rules:

1. **Separation**: Avoid crowding neighbors
2. **Alignment**: Steer towards the average heading of neighbors
3. **Cohesion**: Move toward the average position of neighbors

These simple rules create surprisingly realistic emergent behavior resembling natural flocking patterns.

## Key Features

### Core Algorithm
- ✅ **Complete Boids Implementation**: Full implementation of Reynolds' three flocking rules
- ✅ **Boundary Avoidance**: Intelligent edge detection prevents boids from leaving the screen
- ✅ **Perception Radius**: Configurable neighbor detection for realistic local interactions
- ✅ **Dynamic Weight Adjustment**: Real-time tuning of behavioral parameters

### Graphics & Performance
- 🎨 **Native Terminal Graphics**: Direct PNG rendering via Kitty Graphics Protocol
- 🔄 **360° Sprite Animation**: 90 rotation frames generated at startup, no files involved
- 📦 **Self Contained**: own PNG decoder, encoder and DEFLATE implementation, no zlib, no libpng
- ⚡ **High Performance**: Handles 800+ boids at 60 FPS on modern hardware
- 📐 **Responsive Layout**: Automatic adaptation to terminal resizing
- 🎚️ **Parameter Panel**: every adjustable value as a slider in the top left corner, and the flock cannot fly through it
- 🎮 **Real-time Control**: Interactive parameter adjustment during runtime

### Customization
- 🎛️ **Adjustable Population**: Scale from tens to thousands of boids
- ⏱️ **Variable Frame Rate**: Configure FPS from 30 to 120
- 📏 **Sprite Size**: chosen at startup with `-s`, the frames are rendered for that size
- 🎚️ **Behavioral Tuning**: Fine-tune separation, alignment, cohesion, and boundary weights

## How It Works

Cbirds combines several technologies to achieve high-performance terminal graphics:

1. **Sprite Generation**: the embedded PNG is decoded once, then rotated and scaled into 90 frames, each re-encoded as a PNG in memory
2. **Kitty Graphics Protocol**: Binary image data is Base64-encoded and transmitted to the terminal using escape sequences
3. **Double Buffering**: State updates are computed on a separate copy to ensure consistency
4. **Spatial Grid**: boids are grouped into fixed 12×12 pixel cells so neighbor searches stay local
5. **Rotation Precomputation**: the 90 frames are built at startup so the main loop only sends positions
6. **Raw Terminal Mode**: Direct terminal control for responsive keyboard input

## Requirements

### Supported Terminal Emulators

Cbirds requires a terminal that supports the **Kitty Graphics Protocol**:

| Terminal | Status | Notes |
|----------|--------|-------|
| **Kitty** | ✅ Fully Supported | Original implementation |
| **WezTerm** | ✅ Fully Supported | Excellent performance |
| **Ghostty** | ✅ Fully Supported | Modern alternative |
| **Konsole** | ✅ Supported | Requires recent version  |
| Alacritty | ❌ Not Supported | No graphics protocol support |
| GNOME Terminal | ❌ Not Supported | No graphics protocol support |
| Windows Terminal | ⚠️ Partial | Newer versions only |

### System Requirements

- **Operating System**: Linux or macOS
- **Compiler**: GCC 7+ or Clang 10+
- **Libraries**:
  - `libm` (math library)
  - Standard C library
- **Image Assets**: none, the sprite is compiled into the binary

## Project Layout

The sources sit in the repository root, the tests in `tests/`:

| File | |
|---|---|
| `boids.c` | simulation and terminal handling |
| `kitty_graphics.c` / `kitty_graphics.h` | buffered Kitty graphics protocol API, plus the terminal text and erase it shares the buffer with |
| `spatial_grid.c` / `spatial_grid.h` | fixed-size spatial grid and contiguous cell buckets |
| `png.c` / `png.h` | the PNG library (decode, encode, rotate, resize, tint) |
| `sprite_png.h` | the bird PNG, generated, compiled into the binary |
| `mkasset.c` | regenerates that header from `matrix.png` |
| `matrix.png` | the original artwork |
| `tests/boids_test.c` | panel, obstacle, grid-versus-brute-force and input tests |
| `tests/kitty_graphics_test.c` | protocol formatting and chunking tests |
| `tests/spatial_grid_test.c` | spatial lookup and brute-force equivalence tests |

Each test reaches its subject with a `../` include and nothing else, so it needs
no search path and builds from wherever it is invoked. `tests/boids_test.c`
includes `boids.c` whole, with `main` renamed out of the way, which is what lets
it drive the real static functions rather than a copy of them.

## Installation

### Clone the Repository

```bash
git clone https://github.com/yourusername/cbirds.git
cd cbirds
```

### Build from Source

```bash
make
```

The compiled binary `cbirds` is created in the repository root. It needs
nothing else at runtime: copy it anywhere and run it.

Run the protocol, spatial grid and simulation tests with:

```bash
make test
```

### Changing the Artwork

The bird is `matrix.png`, embedded in the binary as `sprite_png.h`.
After editing the artwork, regenerate the header:

```bash
make asset
make
```

`make asset` builds `mkasset`, which validates the PNG with the project's
own decoder before writing the header.

## Usage

### Basic Usage

Run with default settings (800 boids at 60 FPS):

```bash
./cbirds
```

### Command-Line Options

```bash
./cbirds [OPTIONS]

Options:
  -n NUMBER    Set number of boids (default: 800, max: 4096)
  -f FPS       Set frame rate (default: 60, from 30 to 120)
  -s SIZE      Set bird size in pixels (default: 15, from 4 to 64)
  --no-legend  Hide the parameter panel, the flock keeps the corner
  -h           Show usage and exit

Examples:
  ./cbirds -n 1500 -f 75     # 1500 boids at 75 FPS
  ./cbirds -n 100            # 100 boids at default 60 FPS
  ./cbirds -f 30             # Default 800 boids at 30 FPS
```

### Runtime Controls

While the simulation is running, use these keyboard commands:

#### General Controls
- `q` or `Ctrl+C` - Quit the simulation (the terminal is always restored, crashes included)

#### Behavioral Parameters
- `B` / `b` - Increase/decrease **boundary avoidance** weight
- `S` / `s` - Increase/decrease **separation** weight
- `C` / `c` - Increase/decrease **cohesion** weight
- `A` / `a` - Increase/decrease **alignment** weight
- `P` / `p` - Increase/decrease **perception radius** by one notch, 4 pixels (12–60 pixels)

#### Performance
- `R` / `r` - Increase/decrease frame rate by one notch (limited to 30–120)

#### The Parameter Panel

The top left corner carries a panel of sliders, one per adjustable parameter,
each with its current value and the two keys that move it. The keys are written
lowercase first because that is the end of the bar each one works from, and the
values are right aligned in a column of their own so the numbers stack.

```
╭────────────────────────────────────╮
│ boundary   ▓▓▓▓░░░░░░░░  0.20  b/B │
│ separation ▓▓▓▓░░░░░░░░ 0.005  s/S │
│ cohesion   ▓▓▓▓░░░░░░░░ 0.010  c/C │
│ alignment  ▓▓▓▓░░░░░░░░  1.50  a/A │
│ perception ▓▓▓▓▓▓░░░░░░  36px  p/P │
│ rate       ▓▓▓▓░░░░░░░░    60  r/R │
│                                    │
│ quit       q                       │
╰────────────────────────────────────╯
```

Each number carries just enough decimals to tell one notch from the next:
hundredths for the boundary and alignment weights, thousandths for separation and
cohesion, whole pixels for the perception radius and whole frames a second for
the rate. The bar and the number come off the same notch, so they cannot disagree.

**One keypress is one notch of bar.** The bar has twelve cells, and every
parameter travels through exactly twelve steps from its floor to its ceiling, so
pressing a key always moves its slider by one cell and never by a fraction of
one. That holds because the notch is the state the keys move: the weight, the
radius and the frame rate are all derived from it, so a value and its bar cannot
drift apart. Each default sits on the fourth notch, a third along, except
perception which starts on the sixth.

The panel is 38 by 10 cells and never changes size: it follows the longest
parameter name, the bar and the value column, not the terminal. It is dropped
below 50 columns or 14 rows, where it would leave no corridor to fly in, and
`--no-legend` turns it off outright.

**The flock cannot enter it.** While the panel is up its rectangle carries an
edge force of magnitude 100000, far above every other term in the model, aimed
at whichever of the two open sides is nearer. The force acts on the panel grown
by one frame of travel, which is what makes the panel unreachable rather than
merely unwelcoming: a bird just outside that margin lands at worst a hair inside
it, still clear of the panel, and is turned away before the next step. The margin
is derived from the speed, so it follows the frame rate on its own. A run of 800
boids issues no placement over the panel at any frame rate.

Because the panel takes a corner rather than a row, the flyable area stays an L:
the flock keeps the full width below the panel and the full height beside it.
Note that `demo.gif` above predates the panel.

## Configuration

### Default Parameters

The simulation uses these default values (defined in source):

```c
BIRDS_N = 800              // Number of boids
FRAME_RATE = 60            // Frames per second
SPEED = 40                 // Movement speed (pixels/frame at 60 FPS)
BIRD_SIZE = 15             // Sprite size (pixels), see -s
SPATIAL_CELL_SIZE = 12     // Fixed grid cell size in pixels
VISION_RADIUS = 36         // Default perception radius in pixels, 12 to 60
LEGEND_BAR_CELLS = 12      // Bar cells, and the steps every parameter travels

// Behavioral weights, twelve notches from a floor to a ceiling, default on the 4th
SEPARATION_W = 0.005       // Avoidance strength, 0.001 to 0.013
ALIGNMENT_W = 1.5          // Direction matching strength, 0.1 to 4.3
COHESION_W = 0.01          // Grouping strength, 0.002 to 0.026
BOUNDARY_AV_W = 0.2        // Edge avoidance strength, 0.01 to 0.58

// Edge bands the flock turns away from, as a fraction of the viewport
TURN_BAND_DIVISOR = 3      // Sides and top: one third of width / height
BOTTOM_BAND_DIVISOR = 6    // Bottom: one sixth of the height
```

`SPEED` is derived from the requested frame rate so that the nominal distance
covered per second stays constant: changing FPS (`-f`, or `R`/`r` at runtime)
does not change the flock speed while the terminal sustains that rate.

The perception radius is tuned in pixels, from 12 to 60 in steps of 4, rather
than in whole grid cells: that is what lets it share the same twelve step travel
as everything else. The block of cells the neighbor search sweeps is derived from
it and rounds up, so it always reaches as far as the radius does; cells only
select candidates, and the final distance check remains circular on the exact
radius.

Each weight is bounded on both sides. The ceiling is three times the default,
which gives the panel's sliders a scale to fill against and keeps a keypress
worth between two and six cells of bar.

The turn bands are proportional to the viewport, so they follow a resize and
stay a band on a short terminal instead of covering it whole. The bottom one is
deliberately half the others: birds approaching the last rows get a later, and
therefore sharper, turn. The flock starts spread over the region no band covers.

### Optimizing Performance

**For smoother animation:**
- Reduce boid count: `./cbirds -n 400`
- Lower frame rate: `./cbirds -f 30`
- Use smaller sprites: `./cbirds -s 10`

**For more dramatic flocking:**
- Increase cohesion: Press `C` multiple times
- Decrease separation: Press `s` multiple times

**For more chaotic behavior:**
- Decrease alignment: Press `a` multiple times
- Increase separation: Press `S` multiple times

## Technical Details

### Architecture

The simulation follows this execution flow:

1. **Initialization**: Decode the embedded PNG, build the 90 rotation frames, Base64-encode them
2. **State Setup**: Initialize boid positions and velocities randomly
3. **Main Loop**:
   - Process keyboard input and drain any pending terminal output
   - Copy current state for consistent calculations
   - Rebuild the spatial grid from that immutable snapshot
   - Queue the current positions for rendering, then the panel on top
   - Calculate neighbor influences from nearby cells
   - Apply flocking rules and update positions
   - Update rotation frame IDs based on new directions
   - Flush the Kitty commands without blocking
   - Sleep for the remainder of the frame budget, measured with a monotonic clock

### The PNG Library

`png.c` / `png.h` are self contained, no zlib and no libpng:

| Function | What it does |
|---|---|
| `png_decode` | 8 bit non interlaced PNG (gray, RGB, with or without alpha) into RGBA, chunk CRCs verified |
| `png_encode` | RGBA back into a PNG kept in memory |
| `png_rotate` | rotation around the center, canvas preserved |
| `png_resize` | box filter when shrinking, bilinear when enlarging |
| `png_rotate_resize` | the two above in sequence, which is how the sprites are built |
| `png_tint` | recolors, multiply or replace, alpha untouched |

The DEFLATE decompressor supports all three block types (stored, fixed and
dynamic Huffman), so any real PNG can be read. Compression on the way out uses
stored blocks only: the sprites are a few hundred bytes each and are sent to the
terminal once, so the ratio does not matter. Filtering runs on premultiplied
alpha, otherwise the color of the transparent pixels bleeds into the wings.

### Key Algorithms

**Spatial Grid**: the screen is divided into fixed 12×12 pixel cells. Each
frame uses counting and prefix sums to group boid indices into contiguous cell
ranges. A boid visits only the cells covered by its current vision radius, then
applies the exact circular distance test. Building the grid is `O(n + cells)`;
neighbor lookup is proportional to the local candidates, with `O(n²)` retained
only as the worst case when the whole flock is densely clustered.

**Direction Calculation**: Weighted vector sum of all behavioral components:
```c
result = separation×W₁ + alignment×W₂ + cohesion×W₃ + boundary×W₄
```

### Graphics Protocol

Cbirds uses Kitty's graphics protocol with these commands:

- `\033_Ga=t,f=100,I=<id>;<base64_data>\033\\` - Upload image
- `\033_Ga=p,I=<id>,X=<x>,Y=<y>\033\\` - Display image
- `\033_Ga=d,d=a\033\\` - Delete all visible placements inside the synchronized frame

Image uploads are Base64 encoded and automatically split into protocol chunks
of at most 4096 bytes by `kitty_graphics.c`.

As in the original fast renderer, each frame uses one global placement clear
followed by all current placements, and then the panel. The whole operation
is wrapped in DEC synchronized-update mode (`CSI ? 2026 h` / `CSI ? 2026 l`), so
the terminal presents it atomically instead of displaying the empty intermediate
state.

The panel travels in the same buffer as the graphics commands, so it shares that
atomic frame and the flow control below. It is text rather than a placement,
which means the per frame placement clear does not remove it. Being anchored to
the origin and constant in cells, it never strands text by moving; the only rows
that ever need an erase are the ten it held when a shrinking viewport switches it
off, and those are erased one line at a time with `CSI K`. Never the whole
screen. Clearing the screen once the sprites are uploaded deletes them, and every
later placement then refers to an image that no longer exists, so the flock stops
being drawn altogether: that is also why the one full erase at startup happens
before the upload, not after. Redrawing the panel every frame costs about 500
bytes against the 29 KB a frame of 800 boids already spends.
Default placement and z-index IDs are omitted to keep every command compact;
`C=1` prevents cursor movement and accidental scrolling.

Runtime output is flow-controlled. If the terminal cannot consume a frame
immediately, Cbirds preserves only that frame's unsent suffix, keeps polling
both terminal input and output, and resumes as soon as either becomes ready. It
does not generate another frame until the pending output has drained. The
selected FPS is therefore an upper target: output saturation can lower the
effective rate, but cannot create an unbounded queue or starve the `R`, `r`,
and `q` input handling.

### Performance Characteristics

| Boid Count | Frame Rate | CPU Usage* | Memory Usage |
|------------|------------|------------|--------------|
| 400 | 60 FPS | ~8% | ~1.9 MB |
| 800 | 60 FPS | ~20% | ~2 MB |

*On an Intel i9-9880H with 8 cores

Startup, sprite generation included, is about 50 ms at the default size and
160 ms at `-s 64`.

## Contributing

Contributions are welcome! Areas for improvement:

- **Optimization**: multithreading or SIMD for dense flocks, a real DEFLATE compressor for the encoder
- **Features**: Predator-prey dynamics, obstacle avoidance, 3D visualization
- **Portability**: Windows support, additional terminal protocols

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Craig Reynolds** for the original Boids algorithm (1986)
- **Kovid Goyal** for the Kitty graphics protocol specification

## See Also

- [Original Boids Paper](http://www.red3d.com/cwr/boids/) by Craig Reynolds
- [Kitty Graphics Protocol Documentation](https://sw.kovidgoyal.net/kitty/graphics-protocol/)
- [Flocking Behavior on Wikipedia](https://en.wikipedia.org/wiki/Flocking_(behavior))

---

**Made with ❤️ and C**

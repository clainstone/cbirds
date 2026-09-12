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
- 📊 **Emacs Style Mode Line**: the active parameters and the key legend on the bottom row
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

Everything lives in the repository root, there are no subdirectories:

| File | |
|---|---|
| `boids.c` | simulation and terminal handling |
| `boids_test.c` | grid-versus-brute-force simulation and input tests |
| `kitty_graphics.c` / `kitty_graphics.h` | buffered Kitty graphics protocol API, plus the terminal text and erase it shares the buffer with |
| `kitty_graphics_test.c` | protocol formatting and chunking tests |
| `spatial_grid.c` / `spatial_grid.h` | fixed-size spatial grid and contiguous cell buckets |
| `spatial_grid_test.c` | spatial lookup and brute-force equivalence tests |
| `png.c` / `png.h` | the PNG library (decode, encode, rotate, resize, tint) |
| `sprite_png.h` | the bird PNG, generated, compiled into the binary |
| `mkasset.c` | regenerates that header from `matrix.png` |
| `matrix.png` | the original artwork |

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
- `P` / `p` - Increase/decrease **perception radius** by one 12-pixel cell (1–5 cells)

#### Performance
- `R` / `r` - Increase/decrease frame rate by 5 FPS (limited to 30–120)

#### The Mode Line

The bottom row carries a mode line in the Emacs manner, in reverse video, with
the live parameters and the keys that change them:

```
-:--- cbirds  800 boids  60fps  (Boids)  b/B 0.20  s/S 0.005  c/C 0.010  a/A 1.5  p/P 3  q quit
```

The leading sigil follows the Emacs convention for a modified buffer: `-:---`
while the four weights sit at their defaults, `-:**-` once any of them is
touched. Three layouts are used depending on the width, the widest from 100
columns and the most compact from 44; below 44 columns, or 6 rows, the bar is
dropped and the flock keeps the whole viewport.

The row is reserved: the flock gives it up along with the sprite height that
would otherwise spill into it, since a Kitty placement is not clipped to its
cell. At 24 rows of 16 pixels that costs about 8% of the vertical area, at
1080p about 4%. Note that `demo.gif` above predates the mode line.

## Configuration

### Default Parameters

The simulation uses these default values (defined in source):

```c
BIRDS_N = 800              // Number of boids
FRAME_RATE = 60            // Frames per second
SPEED = 40                 // Movement speed (pixels/frame at 60 FPS)
BIRD_SIZE = 15             // Sprite size (pixels), see -s
SPATIAL_CELL_SIZE = 12     // Fixed grid cell size in pixels
VISION_CELLS = 3           // Default radius: 3 cells = 36 pixels

// Behavioral weights
SEPARATION_W = 0.005       // Avoidance strength
ALIGNMENT_W = 1.5          // Direction matching strength
COHESION_W = 0.01          // Grouping strength
BOUNDARY_AV_W = 0.2        // Edge avoidance strength

// Edge bands the flock turns away from, as a fraction of the viewport
TURN_BAND_DIVISOR = 3      // Sides and top: one third of width / height
BOTTOM_BAND_DIVISOR = 6    // Bottom: one sixth of the height
```

`SPEED` is derived from the requested frame rate so that the nominal distance
covered per second stays constant: changing FPS (`-f`, or `R`/`r` at runtime)
does not change the flock speed while the terminal sustains that rate.

The perception radius ranges from 1 to 5 cells (12–60 pixels). Cells only
select neighbor candidates; the final distance check remains circular and
uses the exact radius in pixels.

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
   - Queue the current positions for rendering, then the mode line on top
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
followed by all current placements, and then the mode line. The whole operation
is wrapped in DEC synchronized-update mode (`CSI ? 2026 h` / `CSI ? 2026 l`), so
the terminal presents it atomically instead of displaying the empty intermediate
state.

The mode line travels in the same buffer as the graphics commands, so it shares
that atomic frame and the flow control below. It is text rather than a placement,
which means the per frame placement clear does not remove it: when a resize moves
the bar, the row it used to sit on is erased by hand (`CSI K` on that row alone),
otherwise the old bar stays stranded mid screen. It has to be that row alone.
Clearing the whole screen once the sprites are uploaded deletes them, and every
later placement then refers to an image that no longer exists, so the flock stops
being drawn altogether: that is also why the one full erase at startup happens
before the upload, not after. Redrawing the bar every frame costs about 110 bytes
against the 29 KB a frame of 800 boids already spends.
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

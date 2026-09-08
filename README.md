# Cbirds: terminal high-Performance Boid Flocking Simulation in C

![Language](https://img.shields.io/badge/Language-C-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20macOS-lightgrey.svg)

A high-performance implementation of Craig Reynolds' **Boids algorithm** in pure C, featuring real-time GPU-accelerated terminal graphics rendering. This flocking simulation brings autonomous agent behavior to life directly in your terminal using the Kitty Graphics Protocol.

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
- 🔄 **360° Sprite Animation**: Full rotational sprites for smooth directional changes
- ⚡ **High Performance**: Handles 800+ boids at 60 FPS on modern hardware
- 📐 **Responsive Layout**: Automatic adaptation to terminal resizing
- 🎮 **Real-time Control**: Interactive parameter adjustment during runtime

### Customization
- 🎛️ **Adjustable Population**: Scale from tens to thousands of boids
- ⏱️ **Variable Frame Rate**: Configure FPS from 1 to 200+
- 📏 **Dynamic Sizing**: Runtime adjustment of sprite dimensions
- 🎚️ **Behavioral Tuning**: Fine-tune separation, alignment, cohesion, and boundary weights

## How It Works

Cbirds combines several technologies to achieve high-performance terminal graphics:

1. **Kitty Graphics Protocol**: Binary image data is Base64-encoded and transmitted to the terminal using escape sequences
2. **Double Buffering**: State updates are computed on a separate copy to ensure consistency
3. **Rotation Precomputation**: 90 pre-rendered rotation frames per sprite size reduce CPU load
4. **Raw Terminal Mode**: Direct terminal control for responsive keyboard input

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
- **Image Assets**: PNG sprite resources (included in repository)

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

The compiled binary `cbirds` is created in the repository root.

### Verify Image Resources

Ensure the sprite images are properly located:

```bash
ls resources/dim5/  # Should contain bird_0.png through bird_89.png
```

Sprites are looked up in `./resources`, then `../resources`. To run the binary
from anywhere else, point it at the directory explicitly:

```bash
CBIRDS_RESOURCES=/path/to/cbirds/resources cbirds
```

The sprites can be regenerated from `resources/matrix.png` with
`python3 python/rotate.py` (run from the repository root; needs `opencv-python`
and `imutils`).

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
  -n NUMBER    Set number of boids (default: 800, max: 200000)
  -f FPS       Set frame rate (default: 60, max: 1000)
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

#### Visual Adjustments
- `=` - Increase bird sprite size
- `-` - Decrease bird sprite size

#### Behavioral Parameters
- `B` / `b` - Increase/decrease **boundary avoidance** weight
- `S` / `s` - Increase/decrease **separation** weight
- `C` / `c` - Increase/decrease **cohesion** weight
- `A` / `a` - Increase/decrease **alignment** weight
- `P` / `p` - Increase/decrease **perception radius** 

#### Performance
- `R` / `r` - Increase/decrease frame rate

## Configuration

### Default Parameters

The simulation uses these default values (defined in source):

```c
BIRDS_N = 800              // Number of boids
FRAME_RATE = 60            // Frames per second
SPEED = 40                 // Movement speed (pixels/frame at 60 FPS)
BIRD_SIZE = 15             // Sprite size (pixels)
PERCEPTION_RADIUS = 35     // Neighbor detection radius

// Behavioral weights
SEPARATION_W = 0.005       // Avoidance strength
ALIGNMENT_W = 1.5          // Direction matching strength
COHESION_W = 0.01          // Grouping strength
BOUNDARY_AV_W = 0.2        // Edge avoidance strength
```

`SPEED` is derived from the frame rate so that the distance covered per second
stays constant: changing FPS (`-f`, or `R`/`r` at runtime) does not make the
flock faster or slower.

### Optimizing Performance

**For smoother animation:**
- Reduce boid count: `./cbirds -n 400`
- Lower frame rate: `./cbirds -f 30`
- Decrease sprite size at runtime: Press `-` key

**For more dramatic flocking:**
- Increase cohesion: Press `C` multiple times
- Decrease separation: Press `s` multiple times

**For more chaotic behavior:**
- Decrease alignment: Press `a` multiple times
- Increase separation: Press `S` multiple times

## Technical Details

### Architecture

The simulation follows this execution flow:

1. **Initialization**: Load and Base64-encode all rotation sprites
2. **State Setup**: Initialize boid positions and velocities randomly
3. **Main Loop**:
   - Copy current state for consistent calculations
   - Calculate neighbor influences for each boid
   - Apply flocking rules and update positions
   - Update rotation frame IDs based on new directions
   - Render sprites using Kitty graphics commands
   - Process keyboard input
   - Sleep for the remainder of the frame budget, measured with a monotonic clock

### Key Algorithms

**Direction Calculation**: Weighted vector sum of all behavioral components:
```c
result = separation×W₁ + alignment×W₂ + cohesion×W₃ + boundary×W₄
```

### Graphics Protocol

Cbirds uses Kitty's graphics protocol with these commands:

- `\033_Ga=t,f=100,I=<id>;<base64_data>\033\\` - Upload image
- `\033_Ga=p,I=<id>,p=<placement>,X=<x>,Y=<y>\033\\` - Display image
- `\033_Ga=d,d=a\033\\` - Delete all visible placements

### Performance Characteristics

| Boid Count | Frame Rate | CPU Usage* | Memory Usage |
|------------|------------|------------|--------------|
| 400 | 60 FPS | ~8% | ~1.9 MB |
| 800 | 60 FPS | ~20% | ~2 MB |

*On an Intel i9-9880H with 8 cores

## Contributing

Contributions are welcome! Areas for improvement:

- **Optimization**: SIMD vectorization, spatial hashing for neighbor queries
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
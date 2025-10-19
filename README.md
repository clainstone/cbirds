# Cbirds: a C-Based Flocking Simulation

![Language](https://img.shields.io/badge/Language-C-blue.svg)

An a high-performance C implementation of Craig Reynolds' "Boids" flocking algorithm, visualized entirely within a modern terminal emulator.

It uses the **Kitty Graphics Protocol** to render and animate hundreds of PNG sprites, bypassing the limitations of traditional text-based (ASCII/Unicode) rendering.

![](./demo.gif)

##  Features

* **Full Boids Algorithm:** Implements the three classic rules:
    * **Separation:** Steer to avoid crowding local flockmates.
    * **Alignment:** Steer towards the average heading of local flockmates.
    * **Cohesion:** Steer to move toward the average position of local flockmates.
* **Native Graphics Rendering:** Uses the Kitty graphics protocol (`\033_G...`) to send Base64-encoded PNG images directly to the terminal for GPU-accelerated rendering.
* **360° Sprite Animation:** Loads 360 unique sprites (one for each degree of rotation) to realistically animate each boid's direction.
* **Responsive Design:** Detects terminal resizing and dynamically adjusts simulation boundaries.


##  Requirements

1.  **Compatible Terminal:** You *must* use a terminal emulator that supports the **Kitty Graphics Protocol**.
    * ✅ **Supported:** `kitty`, `WezTerm`, `Konsole` (recent versions), `Ghostty`.
    * ❌ **Not Supported:** `Alacritty`, `gnome-terminal`, `Terminator`, Windows Terminal (older versions), `st` (without patches).
2.  **Build Tools:** `gcc` (or `clang`) and standard C libraries (`libm`, `libpthread`).
3.  **Image Assets:** 360 PNG image files for the rotation sprites (see Resources).


## Compiling

The project is self-contained and can be compiled with `gcc`:

```bash
cd c
make main
```

## Executing

```bash
./cbirds [options]
```
Available options are:

```bash
-n          #Birds number, default is 800
-f          #Frame rate, default is 60 FPS
```
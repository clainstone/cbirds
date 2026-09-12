# cbirds: the twenty, built

Fable 5.1 audited the project and proposed fifty features. Twenty were chosen
from its own ranking — everything in its top ten, everything that makes the
first frame sell itself, and the handful of extravagances that give a second
post something to show — and all twenty are in. This document is the record of
that, and the list of what was not taken and why.

Every one of them keeps the rules the codebase already had: the screen is never
cleared after the sprites are uploaded, the flock never draws over the panel, one
keypress is one notch, and the terminal comes back exactly as it was.

## Built

| | what | how it shows up |
|---|---|---|
| 1 | A table driven option parser | `options.c`, one row per switch, drives the parser and both help screens |
| 2 | Colour ramps | `--color`, and the fifth shade costs nothing because each angle is rotated once |
| 3 | The terminal's own colours | the default: OSC 4 and 11, asked at startup, ember if unanswered |
| 4 | Shade by heading, density or flock | `--color-by`, a turn runs a ripple through the flock |
| 5 | Flocks that will not merge | `--flocks`, separation physical, alignment and cohesion social |
| 6 | The pointer as predator, feeder or cat | `--mouse`, the default, and the viewer is in the demo in a second |
| 7 | Hawks | `--hawks`, `k`/`K`, the split and reform that makes a clip loop |
| 8 | The flock writes | `--spell`, `font.c`, and `fortune \| cbirds --spell -` |
| 9 | The clock | `--clock`, re-formed when the minute turns |
| 10 | Sliders, presets and a seed on the command line | five weights as notches, `--preset`, `--seed` |
| 11 | Autopilot and idling | `--auto`, `--idle`, one notch every four seconds |
| 12 | Screensaver | `--screensaver`, and anything at all ends it |
| 13 | Intro, outro and a frame limit | `--intro`, `--outro`, `--frames`, so a recording ends by itself |
| 14 | Wrap and trails | `--wrap`, `--trails`, `w` and `e` |
| 15 | Wind | `--wind`, a slow wandering breeze |
| 16 | Shapes and your own sprite | `--shape` draws five from triangles, `--sprite` reads any PNG |
| 17 | The protocol probe | asked before the screen is taken, so no visitor gets a black rectangle |
| 18 | Stats, bench and snapshot | `--stats`, `--bench`, `--snapshot`, numbers anyone can check |
| 19 | Pause, step, reset, population, panel toggle | `space` `.` `0` `+` `-` `h`, the missing basics |
| 20 | Oddities | `--matrix`, and the Konami code, listed in `--help` |

Two more went in because the work asked for them:

- **A real DEFLATE compressor.** `--size 64` pushed ten megabytes of base64 and
  drew nothing for three seconds. LZ77 with fixed Huffman fixed it: 726 KB and
  drawing in 0.8 s, and a snapshot went from 4.15 MB to 50 KB.
- **A test suite for `png.c`.** The largest and most intricate module was the only
  one without one.

## Not taken, and why

- **A canvas renderer** for twenty thousand birds: the `--snapshot` path already
  composites, so the road is open, but deflating a 1080p canvas every frame is
  the whole frame budget and the result is 20 fps rather than 60.
- **Music reactivity**: needs an audio source to verify honestly, and a claim
  that cannot be tested is not one to ship.
- **Flappy Bird inside it**: the only proposal that repaints columns every frame,
  against a codebase whose one hard rule is about what it may repaint.
- **Obstacles, perching, V formations, depth layers, a config file, tmux
  passthrough, droppings, a countdown, the cursor riding a bird.** All viable,
  none of them the difference between a repository someone stars and one they
  scroll past.

## The fifty, as proposed

Fable's list, verbatim, for anyone who wants to pick one up. The numbers are its
own.

1. [VISUAL] Colour by heading — each of the 90 frames tinted by its own angle — `--color heading` **(built)**
2. [VISUAL] Colour by density — birds glow cool to hot as neighbours pile up — `--color density` **(built)**
3. [VISUAL] Terminal theme colours — the flock wears the terminal's own palette **(built, the default)**
4. [VISUAL] Bring your own sprite — any PNG becomes the bird — `--sprite FILE` **(built)**
5. [VISUAL] Built-in sprite set — fish, bat, plane, arrow, dot **(built, drawn from triangles)**
6. [VISUAL] Flapping wings — three wing poses synthesised from the one drawing
7. [VISUAL] Depth layers — three sizes, three speeds, parallax
8. [VISUAL] Leader trails — faint comet tails behind two dozen birds **(built)**
9. [VISUAL] Panel in your accent, in any corner
10. [BEHAVIOUR] Species — N groups that align and cohere only with their own kind **(built as --flocks)**
11. [BEHAVIOUR] The hawk — a predator that hunts the flock **(built)**
12. [BEHAVIOUR] Turning inertia — a maximum turn per frame, so birds bank
13. [BEHAVIOUR] Wind — a slow, wandering breeze the whole flock leans into **(built)**
14. [BEHAVIOUR] Velocity slider and speed jitter
15. [BEHAVIOUR] Wrap-around world — leave one edge, arrive from the other **(built)**
16. [BEHAVIOUR] Obstacles — boxes the flock must fly around
17. [BEHAVIOUR] Geese — V formations instead of a cloud
18. [BEHAVIOUR] The flock spells text — the formation engine **(built)**
19. [BEHAVIOUR] Perching — birds land along the bottom edge
20. [INTERACTION] The pointer is a predator — birds flee the mouse **(built, the default)**
21. [INTERACTION] The pointer is a feeder — birds gather on the mouse **(built)**
22. [INTERACTION] Click to scatter, drag to herd, resize to flinch
23. [INTERACTION] Cat mode — this is kitty, after all **(built)**
24. [INTERACTION] Type and they write it — a vim-style `:` prompt
25. [INTERACTION] Pause, step, reset — the missing basics **(built)**
26. [INTERACTION] Live population — grow or shrink the flock without restarting **(built)**
27. [INTERACTION] Panel toggle that clears its ground **(built)**
28. [INTERACTION] Presets — named slider sets, cycled with Tab **(built)**
29. [INTERACTION] The cursor rides a bird
30. [AUDIO-VISUAL] The flock dances to your music — raw PCM in, cohesion out
31. [AUDIO-VISUAL] Bird clock — the flock is the time **(built)**
32. [AUDIO-VISUAL] Bird countdown — a pomodoro that scatters at zero
33. [SELF-DEMO] Autopilot — the sliders wander by themselves **(built)**
34. [SELF-DEMO] The intro — birds stream in and write "cbirds" **(built)**
35. [SELF-DEMO] The outro — on q the flock flies off the top **(built)**
36. [POLISH] Protocol probe with a kind failure — no more black screens **(built)**
37. [INTEGRATION] Screensaver mode — a drop-in for omarchy/hypridle setups **(built)**
38. [INTEGRATION] `fortune | cbirds` — the flock writes whatever is piped in **(built)**
39. [INTEGRATION] Config file — `~/.config/cbirds/config`
40. [INTEGRATION] Completions and a man page — generated from the option table **(built, completions)**
41. [INTEGRATION] Self-screenshot — the program writes its own PNG **(built)**
42. [INTEGRATION] tmux passthrough
43. [PERFORMANCE] Stats, bench, seed, frames **(built)**
44. [POLISH] Adaptive rate — the rate slider lowers itself on a slow terminal
45. [PERFORMANCE] A real DEFLATE compressor **(built, and it fixed a real bug)**
46. [PERFORMANCE] Canvas renderer — one image a frame, for 20,000 boids
47. [EASTER EGG] Konami code **(built)**
48. [EASTER EGG] Droppings — what birds do
49. [EASTER EGG] Matrix rain — it is raining birds **(built)**
50. [EASTER EGG] Flappy — one bird, some pipes, the space bar

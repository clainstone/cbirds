# cbirds: the twenty

Fable 5.1 audited the project and proposed fifty features. These are the twenty
being built, chosen from its own ranking: everything in its top ten, everything
that makes the first frame sell itself, and the handful of extravagances that
give a second post something to show. This document replaces the panel plan,
which shipped in `31fae78`.

Ordered by the value each adds per hour of work, which is also the order they
are implemented in. Every one of them keeps the rules the codebase already has:
no screen clear after the sprites are uploaded, the flock never draws over the
panel, one keypress is one notch, and the terminal comes back exactly as it was.

## Already in

Three landed before the audit returned, as the foundation everything else needs:

- **A table driven option parser** (`options.c`), because a tool with forty
  switches cannot keep them in a chain of `strcmp`. Both the parser and `--help`
  come off one table.
- **Palette tints**, because Kitty has no per placement tint: a colour is a
  second set of images, and rotating once per angle then tinting per shade makes
  the fifth shade free.
- **Independent flocks**, Fable's #10: separation is physical and applies to
  everyone, alignment and cohesion are social and read only your own flock.
- **Pointer tracking**, mode 1003 with 1006 coordinates, with nothing steering
  by it yet.

## 1. Protocol probe (Fable #36)

A first run in the wrong terminal shows a black screen and the visitor leaves.
Send `\033_Gi=31,s=1,v=1,a=q,t=d,f=24;AAAA\033\\` plus a Primary Device
Attributes request, then poll stdin for 200 ms: a Kitty reply means yes, DA1
alone means no. On no, close the alt screen and print one sentence naming the
terminals that work. `--force` skips the check.

Touches `enter_terminal`. Tested by feeding both replies to the parser.

## 2. Colour from the terminal's own theme (Fable #3, new default)

Every screenshot then matches the poster's setup, which is what r/unixporn
upvotes. Query `OSC 4;i;?` for palette entries 1 to 6 and `OSC 10/11` for
foreground and background, parse `rgb:RRRR/GGGG/BBBB`, build a five step ramp
between the two most saturated answers. Falls back to `ember` when the terminal
does not answer. Becomes palette `theme`, the new default.

Touches the palette table, which already takes an arbitrary list of tints.

## 3. Colour by heading (Fable #1)

Shade from `bird->direction` instead of at birth, so every turn runs a ripple of
colour through the flock. One line in `update_birds`: `bird->shade = shade_for()`
where the mode picks heading, density, flock or fixed.

`--colour-by heading|density|flock|fixed`.

## 4. The pointer is a predator (Fable #20, new default)

The viewer becomes part of the demo inside a second. `mouse` already carries a
position; add a term to `flock_direction` that repels within a radius, the same
shape as the panel's repulsion but finite and weighted. Modes: `flee`, `follow`,
`cat` (flee, but the flock creeps back when the pointer holds still), `off`.

`--mouse MODE`.

## 5. The sliders on the command line

Six runtime rows with no flag between them means no dotfile can express a look.
Each weight takes its notch, 0 to 12, because the notch is the state; perception
and rate take real units and snap, as `-f` already does.

`--boundary N --separation N --cohesion N --alignment N --perception PX`.

## 6. The flock writes (Fable #18, #24, #38)

The shareable unit: birds that spell your name. A 5x7 bitmap font for 96
printable characters, laid out into target points across the free rectangle,
with a per bird target and a spring term that beats the flocking weights while
`spell` is active, then releases. `-` reads stdin, so `fortune | cbirds --spell -`
is the one liner.

New `font.c` and `spell.c`. The largest piece of work here.

## 7. Hawks (Fable #11)

A predator gives a clip a story: the flock splits and reforms. One to eight
hawks, larger sprites from the same PNG at a bigger `bird_size`, chasing the
nearest bird; every bird gets a flee term. Reuses the pointer predator's maths.

`--hawks N`.

## 8. Presets, seeds, and autopilot (the owner's round robin)

- `--preset murmuration|swarm|school|storm|calm` sets the six notches together.
- `--seed N` makes a run reproducible, which is what lets a bug report be shared.
- `--auto` wanders the notches by themselves, one notch every few seconds,
  smoothly, so a terminal left open keeps changing.

## 9. Screensaver (Fable #37)

The omarchy and DHH crowd installs what slots into what they already run.
`--screensaver` is `--auto --no-panel --idle 0` plus any key or pointer movement
quits. Small, given 8.

## 10. It demos itself (Fable #33, #34, #35)

The README GIF needs no hands. `--intro` writes the title as birds and lets go;
`--outro` flies the flock off the top on `q`; `--frames N` quits after N frames
so a recording ends by itself. All three on by default, each with a `--no-`.

## 11. Wrap and trails

`--wrap` takes a bird off one edge and back on the other, which turns the
boundary weight off and looks completely different. `--trails` keeps the last
few placements of every tenth bird at a dimmer shade, which is the cheapest way
to make a still frame look like motion.

## 12. Stats, bench and snapshot

The performance flex, and the numbers the README needs. `--stats` adds a row to
the panel with frame time, bytes a frame and the effective rate. `--bench N`
runs N frames with no terminal and prints the numbers. `--snapshot FILE` writes
the last frame as a PNG through `png_encode`, which the project already has.

## 13. The clock (Fable #31)

`--clock` spells the time with #6's machinery and re-forms it every minute. The
useful toy is the one that stays installed.

## 14. Bring your own sprite (Fable #4)

User made variants are free marketing. `--sprite FILE` decodes any PNG with the
project's own decoder; `--sprite bird|fish|bat|arrow|dot` picks a built-in.
Per flock when repeated.

## 15. Oddities (Fable #47, #49)

`--matrix` is green birds falling in columns, named after `matrix.png`, for the
crowd that runs cmatrix. The Konami code in `handle_input` spawns eight hawks
and spells NICE. Listing an easter egg in `--help` is itself a screenshot.

## 16. The CLI, finished (Fable section B)

- `--no-legend` becomes `--no-panel`, with the old name kept as a silent alias.
- `-h` is one screen, `--help` is everything, grouped, as ripgrep does it.
- Help to stdout and exit 0; usage errors to stderr and exit 2; runtime failures
  exit 1.
- Every default in parentheses in one template. A "did you mean" on a typo.
- An **Oddities** group, visible.
- `--completion bash|zsh|fish` walks the same table.

## Not doing, and why

- **A real DEFLATE compressor** (#45) and the **canvas renderer** (#46): both
  large, and neither shows up in a screenshot. The README already lists the
  compressor as an invitation to contribute, which is worth more than the code.
- **Music reactive** (#30): needs an audio source to verify honestly, and a
  claim I cannot test is not one to ship.
- **Flappy** (#50): the only feature that repaints columns every frame, against
  a codebase whose one hard rule is about what it may repaint.

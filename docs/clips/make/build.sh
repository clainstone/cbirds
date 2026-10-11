#!/bin/bash
# Makes every clip in docs/clips, from a cbirds that has --3d:
#
#     CBIRDS=/path/to/cbirds docs/clips/make/build.sh
#
# Needs a C compiler, python3 and pygmentize (for the code). What it builds and
# the intermediate recordings go to docs/clips/build, which git ignores. The
# recordings are cbirds' own, made with --record; the windows round some of them
# are drawn by framegif.c, which copies every frame pixel for pixel, and the
# three windows of link.gif are three real `cbirds --link` runs, played back
# through cbirds' terminal emulator by linkgif.c.
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT=$(cd ../.. && pwd)
B=${CBIRDS:-$ROOT/cbirds}
mkdir -p build

cc -std=c99 -O2 -I"$ROOT" make/framegif.c "$ROOT"/png.c "$ROOT"/font.c "$ROOT"/gif.c -lm -o build/framegif
cc -std=c99 -O2 -I"$ROOT" make/linkgif.c "$ROOT"/vt.c "$ROOT"/cells.c "$ROOT"/png.c "$ROOT"/gif.c \
    "$ROOT"/font.c -lm -o build/linkgif
cc -std=c99 -O2 -I"$ROOT" make/firstframe.c "$ROOT"/png.c "$ROOT"/font.c -lm -o build/firstframe
cc -std=c99 -O2 donut.c -lm -o build/donut

# The texts. Some are made by hand (make_inputs.py); the rest are the real output
# of a command.
python3 make/make_inputs.py
./build/donut > build/donut.ans
awk '/^static vector_t hawk_vector/,/^}$/' "$ROOT/boids.c" |
    pygmentize -f terminal16m -O style=monokai -l c > build/code.ans

pipe() { # name text size fps seed title
    cat "$2" | "$B" --record "build/$1.gif" --record-size "$3" --record-fps "$4" --seed "$5"
    ./build/framegif "build/$1.gif" "$1.gif" "$6"
}
pipe pipe-code build/code.ans 80x24 8 5 "pygmentize boids.c | cbirds"
pipe pipe-donut build/donut.ans 80x23 8 5 "./donut | cbirds"
pipe pipe-banner banner.ans 64x18 8 3 "cat banner.ans | cbirds"
pipe pipe-sunset sunset.ans 64x18 8 5 "cat sunset.ans | cbirds"
pipe pipe-fetch fetch.ans 64x16 8 5 "cat fetch.ans | cbirds"
pipe pipe-git git.ans 64x20 8 5 "git log --graph | cbirds"

# The banner at rest, as a picture for the top of a page.
cat logo.ans | "$B" --record build/logo.gif --record-size 64x14 --record-seconds 2 --record-fps 2
./build/firstframe build/logo.gif logo.png

# Three dimensions.
space() { # name size fps seconds framed-title args...
    local name=$1 size=$2 fps=$3 seconds=$4 title=$5
    shift 5
    "$B" --3d "$@" --record "build/$name.gif" --record-size "$size" --record-fps "$fps" \
        --record-seconds "$seconds"
    if [ -n "$title" ]; then ./build/framegif "build/$name.gif" "$name.gif" "$title"; else cp "build/$name.gif" "$name.gif"; fi
}
space 3d-prism 80x26 20 10 "cbirds --3d --color prism --hawks 1" --color prism --hawks 1 --seed 3
space 3d-aurora 80x26 20 10 "cbirds --3d --color aurora --hawks 1" --color aurora --hawks 1 --seed 7
space 3d-ember 64x18 25 12 "" --color ember --hawks 1 --seed 45
space 3d-braille 80x22 25 10 "cbirds --3d --render braille --color dusk" --color dusk --render braille --hawks 1 --seed 5
space 3d-ice 48x14 20 8 "" --color ice --hawks 1 --seed 4
space 3d-potion 48x14 20 8 "" --color potion --hawks 1 --seed 6
space 3d-ash 48x14 20 8 "" --color ash --hawks 1 --seed 2
space 3d-dusk 48x14 20 8 "" --color dusk --hawks 1 --seed 9

# One sky across three terminals: live, so never quite the same twice.
CBIRDS="$B" python3 make/capture_link.py build/link 15 44x22 -n 220 -s 11 --color prism --hawks 1 \
    --render sextants
./build/linkgif build/link 44 22 20 2 14 link.gif "cbirds --link --color prism" sextants

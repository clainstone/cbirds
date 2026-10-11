#!/usr/bin/env python3
"""Writes the texts the README clips pipe into cbirds, as ANSI text, into docs/clips.

banner.ans   CBIRDS in a block font, a rainbow across it, and a line under it
logo.ans     the same without the command under it, for a picture
sunset.ans   a picture drawn in full blocks, one colour a cell
fetch.ans    a system-information screen in the manner of fastfetch: a flock drawn
             in braille for the logo, and cbirds' own settings beside it
git.ans      git log --graph of the cbirds repository itself, in its colours
"""
import math
import os
import random
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.dirname(HERE)
REPO = os.path.dirname(os.path.dirname(OUT))
os.makedirs(OUT, exist_ok=True)
ESC = '\033['
RESET = ESC + '0m'


def fg(r, g, b):
    return f'{ESC}38;2;{int(r)};{int(g)};{int(b)}m'


def lerp(a, b, t):
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(3))


def ramp(stops, t):
    t = min(max(t, 0.0), 1.0) * (len(stops) - 1)
    i = min(int(t), len(stops) - 2)
    return lerp(stops[i], stops[i + 1], t - i)


# --- banner -----------------------------------------------------------------
SHADOW = [
    " ██████╗██████╗ ██╗██████╗ ██████╗ ███████╗",
    "██╔════╝██╔══██╗██║██╔══██╗██╔══██╗██╔════╝",
    "██║     ██████╔╝██║██████╔╝██║  ██║███████╗",
    "██║     ██╔══██╗██║██╔══██╗██║  ██║╚════██║",
    "╚██████╗██████╔╝██║██║  ██║██████╔╝███████║",
    " ╚═════╝╚═════╝ ╚═╝╚═╝  ╚═╝╚═════╝ ╚══════╝",
]


def rainbow(x, y):
    # lolcat's: three sines a third of a turn apart, along a slant.
    f = 0.11
    i = x + y * 2.2
    return (127 + 127 * math.sin(f * i + 0),
            127 + 127 * math.sin(f * i + 2 * math.pi / 3),
            127 + 127 * math.sin(f * i + 4 * math.pi / 3))


def banner():
    lines = ['', '', '']
    pad = ' ' * 10
    for y, row in enumerate(SHADOW):
        out = pad
        for x, ch in enumerate(row):
            out += (fg(*rainbow(x, y)) + ch) if ch != ' ' else ' '
        lines.append(out + RESET)
    lines.append('')
    sub = 'a flock of birds in your terminal'
    lines.append(' ' * (10 + (43 - len(sub)) // 2) + fg(170, 170, 185) + sub + RESET)
    lines.append('')
    cmd = '$ fastfetch | cbirds'
    lines.append(' ' * (10 + (43 - len(cmd)) // 2) + fg(110, 220, 160) + '$ ' + fg(230, 230, 235) +
                 cmd[2:] + RESET)
    return '\n'.join(lines) + '\n'


def logo():
    """The banner and the line under it, and nothing else: a picture for the top of
    a page."""
    return '\n'.join(banner().split('\n')[:13 - 2]) + '\n'


# --- sunset -------------------------------------------------------------------
def sunset(cols=46, rows=15):
    sky = [(28, 24, 74), (92, 42, 128), (196, 72, 112), (250, 140, 82), (255, 206, 120)]
    sea_top, sea_low = (60, 46, 110), (14, 18, 48)
    horizon = int(rows * 0.62)
    sun_x, sun_y, sun_r = cols * 0.62, horizon - 1.6, 4.2
    random.seed(7)
    hills = [horizon - 1.2 - 1.6 * math.sin(x * 0.11 + 1.3) - 0.9 * math.sin(x * 0.27)
             for x in range(cols)]
    lines = []
    for y in range(rows):
        out = ''
        for x in range(cols):
            # Cells are about twice as tall as they are wide.
            dx, dy = (x - sun_x) * 0.5, (y - sun_y)
            if y < horizon:
                c = ramp(sky, (y + 0.5) / horizon)
                if dx * dx + dy * dy < sun_r * sun_r * 0.42:
                    c = lerp((255, 236, 170), (255, 186, 92), max(0, dy + sun_r) / (2 * sun_r))
                elif dx * dx + dy * dy < sun_r * sun_r * 0.9:
                    c = lerp(c, (255, 200, 120), 0.45)
                if y >= hills[x] and x < cols * 0.42:
                    c = (34, 22, 52)  # far hills, on the left
                # Pines on the right edge.
                tree = (x * 7919) % 11
                if x > cols * 0.82 and y >= horizon - 2 - (tree % 5) and \
                        abs((x % 4) - 1.5) <= (y - (horizon - 2 - (tree % 5))) * 0.6 + 0.2:
                    c = (16, 26, 30)
            else:
                t = (y - horizon) / max(1, rows - horizon - 1)
                c = lerp(sea_top, sea_low, t)
                # The sun's road on the water, broken by the waves.
                if abs(x - sun_x) < 3.4 - t * 1.6 and random.random() < 0.72 - t * 0.4:
                    c = lerp((255, 190, 110), c, 0.25 + t * 0.35)
            out += fg(*c) + '█'
        lines.append(' ' * 9 + out + RESET)
    return '\n' + '\n'.join(lines) + '\n'


# --- fetch --------------------------------------------------------------------
def bird_logo(w=28, h=14):
    """A swallow in flight, in braille: wings swept up, a forked tail, and three
    small birds behind it. Inside is lit, the rest left blank."""
    W, H = w * 2, h * 4

    def inside(x, y):
        u, v = x / W, y / H
        # The body: a slim ellipse along a slant.
        bx, by = (u - 0.47) / 0.17, (v - 0.56 - (u - 0.47) * 0.18) / 0.075
        if bx * bx + by * by < 1:
            return True
        # The head, ahead of it.
        hx, hy = (u - 0.66) / 0.055, (v - 0.52) / 0.07
        if hx * hx + hy * hy < 1:
            return True
        # The beak.
        if 0.70 < u < 0.76 and abs(v - 0.53) < (0.76 - u) * 0.45:
            return True
        # The wings: two swept curves, thick at the shoulder and fine at the tip.
        for side in (-1, 1):
            for k in range(60):
                t = k / 59
                cx = 0.47 + side * 0.0 + (-0.36 if side < 0 else 0.30) * t - 0.10 * t * t * (side > 0)
                cx = 0.44 - 0.36 * t if side < 0 else 0.50 + 0.30 * t
                cy = 0.50 - 0.42 * t + 0.20 * t * t
                r = 0.075 * (1 - t) ** 0.8 + 0.008
                if ((u - cx) / 1.0) ** 2 + ((v - cy) * 0.55) ** 2 < r * r:
                    return True
        # The forked tail.
        for prong in (-1, 1):
            for k in range(30):
                t = k / 29
                cx, cy = 0.31 - 0.20 * t, 0.62 + prong * 0.10 * t + 0.05 * t
                if (u - cx) ** 2 + ((v - cy) * 0.6) ** 2 < (0.022 * (1 - t) + 0.006) ** 2:
                    return True
        return False

    flock = [(0.86, 0.16), (0.93, 0.27), (0.80, 0.30)]
    bits = [[0, 3], [1, 4], [2, 5], [6, 7]]
    rows = []
    for cy in range(h):
        line = ''
        for cx in range(w):
            v = 0
            for r in range(4):
                for c in range(2):
                    x, y = cx * 2 + c + 0.5, cy * 4 + r + 0.5
                    lit = inside(x, y)
                    for fx, fy in flock:  # a small v for each
                        dx, dy = x / W - fx, y / H - fy
                        if abs(dy + abs(dx) * 1.1) < 0.025 and abs(dx) < 0.035:
                            lit = True
                    if lit:
                        v |= 1 << bits[r][c]
            if v == 0:
                line += ' '
            else:
                col = ramp([(90, 210, 255), (130, 150, 255), (205, 110, 240), (255, 110, 160)],
                           (cx / w) * 0.7 + (cy / h) * 0.3)
                line += fg(*col) + chr(0x2800 + v)
        rows.append(line + RESET)
    return rows


def fetch():
    logo = bird_logo()
    key = fg(130, 200, 255)
    val = fg(225, 225, 232)
    title = fg(255, 130, 170) + 'flock' + fg(225, 225, 232) + '@' + fg(255, 130, 170) + 'sky'
    info = [
        title + RESET,
        fg(90, 90, 110) + '---------' + RESET,
        key + 'Birds' + val + ': 2000' + RESET,
        key + 'Mode' + val + ': --3d, over a roost' + RESET,
        key + 'Camera' + val + ': once round in 2 min' + RESET,
        key + 'Neighbours' + val + ': 7 nearest' + RESET,
        key + 'Ramp' + val + ': ink' + RESET,
        key + 'Render' + val + ': braille' + RESET,
        key + 'Hawks' + val + ': 1' + RESET,
        key + 'Seed' + val + ': 45' + RESET,
        '',
        ''.join(f'{ESC}4{i}m   ' for i in range(8)) + RESET,
        ''.join(f'{ESC}10{i}m   ' for i in range(8)) + RESET,
    ]
    out = ['']
    for i in range(max(len(logo), len(info))):
        left = logo[i] if i < len(logo) else ' ' * 28
        right = info[i] if i < len(info) else ''
        out.append('  ' + left + '   ' + right)
    return '\n'.join(out) + '\n'


# --- git ----------------------------------------------------------------------
def git_log():
    """Twenty three lines of the history of next, where four tracks were merged:
    the graph has three lanes there, each in its own colour."""
    fmt = '%C(yellow)%h%C(reset) %<(46,trunc)%s'
    for ref in ('origin/next', 'next'):
        log = subprocess.run(['git', '-C', REPO, 'log', '--graph', '--color=always', ref,
                              '--format=' + fmt, '-n', '400'], capture_output=True, text=True)
        if log.returncode == 0:
            break
    else:
        return None  # No next branch here: git.ans is kept as it was made.
    text = log.stdout.split('\n')
    first = next(i for i, l in enumerate(text) if '97026e6' in l)
    return '\n'.join(text[first:first + 19]) + '\n'


for name, text in [('banner.ans', banner()), ('logo.ans', logo()), ('sunset.ans', sunset()),
                   ('fetch.ans', fetch()),
                   ('git.ans', git_log())]:
    if text is None:
        continue
    with open(os.path.join(OUT, name), 'w', encoding='utf-8') as f:
        f.write(text)
    print(name, len(text.splitlines()), 'lines')

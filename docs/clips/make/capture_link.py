#!/usr/bin/env python3
"""capture_link.py OUTDIR SECONDS COLSxROWS ARGS...: three real `cbirds --link` windows,
started left to right in pseudo-terminals of their own that share a private sky
directory, and what each of them wrote, with when, as OUTDIR/window-N.log: lines of
"seconds hex-bytes". The windows are told to quit together at the end."""
import binascii, fcntl, os, select, signal, struct, sys, termios, time

OUT, SECONDS, SIZE, ARGS = sys.argv[1], float(sys.argv[2]), sys.argv[3], sys.argv[4:]
COLS, ROWS = (int(v) for v in SIZE.split('x'))
BIN = os.path.abspath(os.environ.get('CBIRDS', os.path.join(os.path.dirname(__file__), '..', '..', '..', 'cbirds')))
os.makedirs(OUT, exist_ok=True)
SKY = os.path.join(OUT, 'sky')
os.makedirs(SKY, mode=0o700, exist_ok=True)
os.chmod(SKY, 0o700)

windows = []
start = time.time()
for n in range(3):
    master, slave = os.openpty()
    fcntl.ioctl(slave, termios.TIOCSWINSZ, struct.pack('HHHH', ROWS, COLS, COLS * 8, ROWS * 16))
    pid = os.fork()
    if pid == 0:
        os.setsid()
        fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
        for fd in (0, 1, 2):
            os.dup2(slave, fd)
        os.close(master)
        env = dict(os.environ, TERM='xterm-256color', COLORTERM='truecolor', XDG_RUNTIME_DIR=SKY)
        os.execve(BIN, [BIN, '--link'] + ARGS + ['--seed', str(11 + n)], env)
    os.close(slave)
    windows.append({'pid': pid, 'fd': master, 'log': open(os.path.join(OUT, f'window-{n}.log'), 'w')})
    # Left to right: each joins after the one before it is in the sky.
    joined = time.time()
    while time.time() - joined < 0.6:
        for w in windows:
            r, _, _ = select.select([w['fd']], [], [], 0.01)
            if r:
                data = os.read(w['fd'], 65536)
                w['log'].write(f'{time.time() - start:.4f} {binascii.hexlify(data).decode()}\n')

quit_at = start + SECONDS
quit_sent = False
while True:
    now = time.time()
    if not quit_sent and now >= quit_at:
        for w in windows:
            os.write(w['fd'], b'q')
        quit_sent = True
    fds = [w['fd'] for w in windows if w['fd'] is not None]
    if not fds:
        break
    r, _, _ = select.select(fds, [], [], 0.01)
    for w in windows:
        if w['fd'] in r:
            try:
                data = os.read(w['fd'], 65536)
            except OSError:
                data = b''
            if data:
                w['log'].write(f'{now - start:.4f} {binascii.hexlify(data).decode()}\n')
            else:
                os.close(w['fd'])
                w['fd'] = None
    for w in windows:
        if w['fd'] is not None and os.waitpid(w['pid'], os.WNOHANG)[0]:
            # Drain what it had still to say.
            while True:
                rr, _, _ = select.select([w['fd']], [], [], 0.05)
                if not rr:
                    break
                try:
                    data = os.read(w['fd'], 65536)
                except OSError:
                    break
                if not data:
                    break
                w['log'].write(f'{time.time() - start:.4f} {binascii.hexlify(data).decode()}\n')
            os.close(w['fd'])
            w['fd'] = None
    if now > quit_at + 10:
        for w in windows:
            if w['fd'] is not None:
                os.kill(w['pid'], signal.SIGKILL)
        break
for w in windows:
    w['log'].close()
print('captured', [os.path.getsize(os.path.join(OUT, f'window-{n}.log')) for n in range(3)])

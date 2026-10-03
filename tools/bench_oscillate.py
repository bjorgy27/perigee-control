#!/usr/bin/env python3
"""Bench motor test for the PERIGEE mount: random oscillation on both axes.

Talks the fw4-stm32 `T4` protocol over the Nucleo's ST-LINK virtual COM port, the same
protocol perigee-control's `src/serial.rs` speaks. Stdlib only (termios), so there is
nothing to install.

This is a BARE-GEARBOX test. No boom arm, no dish, no counterweight: the two Stingrays
are screwed to the plate and nothing is bolted to their output shafts. It drives a
conservative sub-window well inside the firmware's soft limits, at a slow slew rate, so
the only thing to watch is that both shafts turn and reverse.

    python3 tools/bench_oscillate.py                 # 60 s, default sub-window
    python3 tools/bench_oscillate.py --seconds 20
    python3 tools/bench_oscillate.py --az 180 220 --el 40 50 --rate 10 8
    python3 tools/bench_oscillate.py --port /dev/ttyACM1
    python3 tools/bench_oscillate.py --dry-run       # print the session, open no port

HOLD THE BLUE USER BUTTON. The firmware drives the servos only while B1 is held down, so
this script sets up the link, declares a position and starts issuing moves, and nothing
turns until your thumb is on the board. Let go and the pulses stop within a millisecond.

THE FIRST PULSE SNAPS. These are positional servos with no feedback wire: after a power
cycle the firmware has no idea where the shafts are and refuses to move until a position
is declared. This script declares the park pose (AZ 200, EL 45), which is the centre of
both windows, so the moment you first press the button each shaft jumps from wherever it
physically sits to the centre, at full gearbox speed, once. With nothing attached that is
noise. Do not run this with the arm on until the axes have been calibrated against
docs/calibration.md.

Ctrl-C at any time: the script stops the axes, parks them, and leaves the pulses on
holding, unless --off is given.
"""

import argparse
import glob
import os
import random
import select
import sys
import termios
import time

BAUD = termios.B115200          # firmware: BAUD = 115_200 in src/main.rs
PROTO = "T4"                    # firmware: TEL_TAG in src/limits.rs
PARK = (200.0, 45.0)            # firmware: PARK in src/main.rs, the centre of both windows

# Firmware soft windows (src/limits.rs AZ_CAL / EL_CAL). The script never commands outside
# these; the defaults below keep a wide margin inside them on top of that.
FW_AZ = (0.0, 400.0)
FW_EL = (-2.0, 91.0)


class Mount:
    """Line-oriented serial link to the board, with a raw termios port underneath."""

    def __init__(self, path, dry_run=False):
        self.path = path
        self.dry_run = dry_run
        self.rx = b""
        self.log = []
        if dry_run:
            self.fd = None
            return
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        self.saved = termios.tcgetattr(self.fd)
        iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(self.fd)
        # 8N1, raw, no flow control, no modem-control lines, reads never block
        iflag = 0
        oflag = 0
        lflag = 0
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
        cc = list(cc)
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, BAUD, BAUD, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def close(self):
        if self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            os.close(self.fd)
            self.fd = None

    def send(self, line):
        self.log.append(">> " + line)
        print(f"  >> {line}", flush=True)
        if self.fd is not None:
            os.write(self.fd, (line + "\n").encode())

    def lines(self):
        """Every complete line received since the last call."""
        if self.fd is None:
            return []
        out = []
        while select.select([self.fd], [], [], 0)[0]:
            chunk = os.read(self.fd, 4096)
            if not chunk:
                break
            self.rx += chunk
        while b"\n" in self.rx:
            raw, self.rx = self.rx.split(b"\n", 1)
            text = raw.decode("utf-8", "replace").strip()
            if text:
                out.append(text)
        return out

    def ask(self, line, timeout=2.0):
        """Send a command and collect replies until the link goes quiet."""
        self.send(line)
        if self.fd is None:
            return []
        got, deadline = [], time.monotonic() + timeout
        while time.monotonic() < deadline:
            for text in self.lines():
                print(f"  << {text}", flush=True)
                got.append(text)
            if got:
                deadline = min(deadline, time.monotonic() + 0.15)
            time.sleep(0.02)
        return got


def find_port(explicit):
    if explicit:
        return explicit
    # Same search order as perigee-control's src/serial.rs
    for pattern in ("/dev/rfcomm*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[0]
    return None


def inside(name, lo, hi, fw_lo, fw_hi):
    if lo > hi:
        sys.exit(f"{name}: {lo} is above {hi}")
    if lo < fw_lo or hi > fw_hi:
        sys.exit(f"{name} range {lo}..{hi} is outside the firmware window {fw_lo}..{fw_hi}")


def main():
    p = argparse.ArgumentParser(description="Oscillate both mount axes on the bench.")
    p.add_argument("--port", help="serial port; default is the first rfcomm/ttyACM/ttyUSB")
    p.add_argument("--seconds", type=float, default=60.0, help="how long to oscillate (default 60)")
    p.add_argument("--az", nargs=2, type=float, default=[170.0, 230.0], metavar=("LO", "HI"),
                   help="azimuth sub-window, deg (default 170 230)")
    p.add_argument("--el", nargs=2, type=float, default=[30.0, 60.0], metavar=("LO", "HI"),
                   help="elevation sub-window, deg (default 30 60)")
    p.add_argument("--rate", nargs=2, type=float, default=[15.0, 10.0], metavar=("AZ", "EL"),
                   help="slew limit, deg/s (default 15 10)")
    p.add_argument("--dwell", nargs=2, type=float, default=[1.0, 3.0], metavar=("MIN", "MAX"),
                   help="seconds to hold between moves (default 1 3)")
    p.add_argument("--seed", type=int, help="seed the random walk, to repeat a run exactly")
    p.add_argument("--off", action="store_true", help="go limp at the end instead of holding park")
    p.add_argument("--dry-run", action="store_true", help="print the session without opening a port")
    args = p.parse_args()

    inside("azimuth", args.az[0], args.az[1], *FW_AZ)
    inside("elevation", args.el[0], args.el[1], *FW_EL)
    random.seed(args.seed)

    port = find_port(args.port)
    if port is None and not args.dry_run:
        sys.exit("no serial port found: plug the Nucleo in (ST-LINK USB) and look for /dev/ttyACM0")
    if not args.dry_run and not os.access(port, os.R_OK | os.W_OK):
        sys.exit(f"{port} is not readable/writable by this user: check the uucp group")

    print(f"PERIGEE bench oscillation on {port or '(dry run)'}")
    print(f"  azimuth   {args.az[0]:.0f} .. {args.az[1]:.0f} deg   at {args.rate[0]:.0f} deg/s")
    print(f"  elevation {args.el[0]:.0f} .. {args.el[1]:.0f} deg   at {args.rate[1]:.0f} deg/s")
    print(f"  {args.seconds:.0f} s, dwelling {args.dwell[0]:.1f}..{args.dwell[1]:.1f} s per move\n")

    m = Mount(port, dry_run=args.dry_run)
    parked = False
    try:
        # --- handshake -------------------------------------------------------------
        boot = m.lines()
        for text in boot:
            print(f"  << {text}", flush=True)
        if not m.ask("PING") and not args.dry_run:
            sys.exit("no answer to PING: wrong port, or the board is not running fw4")
        ident = " ".join(m.ask("ID"))
        if ident and f"PROTO {PROTO}" not in ident:
            sys.exit(f"this board does not speak PROTO {PROTO}; refusing to drive it:\n  {ident}")
        m.ask("CAL")
        tel = [t for t in m.ask("?") if t.startswith(PROTO + " ")]

        # --- position: declare park if the board came up cold ----------------------
        # `T4 ... NA -1 <flags>`; K in the flag field means the position estimate is anchored.
        known = bool(tel) and "K" in tel[-1].split()[-1]
        if not known:
            print("\n  position unknown (cold boot). Declaring the park pose as the current one.")
            print("  *** both shafts will SNAP to centre the first time you press the button. ***\n")
            m.ask("OFF")
            m.ask(f"ZERO {PARK[0]:.1f} {PARK[1]:.1f}")
        m.ask(f"RATE {args.rate[0]:.1f} {args.rate[1]:.1f}")
        m.ask("TEL 4")
        m.ask(f"GO {PARK[0]:.1f} {PARK[1]:.1f}")
        time.sleep(1.0 if not args.dry_run else 0)

        # --- the oscillation -------------------------------------------------------
        print("\n  *** HOLD THE BLUE USER BUTTON on the Nucleo. Nothing moves until you do. ***")
        print("  Let go and the pulses stop where they are. Ctrl-C to end the run.\n")
        start = time.monotonic()
        az = el = None
        while time.monotonic() - start < args.seconds:
            # Each axis crosses the middle of its sub-window every move, so the shafts
            # visibly reverse rather than creeping in one direction.
            az_new = random.uniform(*args.az)
            el_new = random.uniform(*args.el)
            if az is not None and abs(az_new - az) < 10.0:
                az_new = args.az[0] + args.az[1] - az_new      # mirror it: make the reversal obvious
            if el is not None and abs(el_new - el) < 5.0:
                el_new = args.el[0] + args.el[1] - el_new
            az, el = az_new, el_new
            m.send(f"GO {az:.2f} {el:.2f}")
            hold = random.uniform(*args.dwell)
            until = time.monotonic() + hold
            while time.monotonic() < until:
                for text in m.lines():
                    print(f"  << {text}", flush=True)
                time.sleep(0.05)
            if args.dry_run:
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n  interrupted.")
    finally:
        # Always leave the mount somewhere defined: stopped, then parked.
        try:
            m.ask("STOP", timeout=0.6)
            m.ask(f"GO {PARK[0]:.1f} {PARK[1]:.1f}", timeout=0.6)
            parked = True
            if args.off:
                time.sleep(4.0 if not args.dry_run else 0)
                m.ask("OFF", timeout=0.6)
            m.ask("TEL 0", timeout=0.6)
        finally:
            m.close()
    print("\n  parked." if parked else "\n  done.", "Pulses off, servos limp." if args.off else "Pulses still on, holding park.")


if __name__ == "__main__":
    main()

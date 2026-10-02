# PERIGEE mount calibration

How a sky direction becomes a servo pulse, every constant that sits in that chain, where each one is
measured from, and a bench procedure to establish them. Written 2026-10-01 against the uncommitted
cable-wrap safety layer (`src/limits.rs`, `src/main.rs`, host `src/mount.rs`).

Every number quoted below was read out of the source or computed from it. Anything that depends on
hardware is marked **unverified**: none of it has been measured on the built mount yet.

---

## 1. The chain, end to end

```
satellite ECI  ──► look_angles()  ──►  true bearing, elevation      perigee-viewer/src/lib.rs:209
                                              │
                                              │  az_center_bearing_deg, el_correction_deg   calibration.toml (/here, /bearing)
                                              ▼
                                       mount frame az_m, el_m        perigee-control/src/mount.rs
                                              │
                                              │  wrap choice: nearest_allowed_az / solve_path
                                              ▼
                                       "GO az el" over USB serial     115200 8N1, /dev/ttyACM0
                                              │
                                              │  offset, soft window, gear_margin        limits.rs
                                              ▼
                                       gearbox angle  ──►  pulse µs   pulse_at_zero + gear·us_per_deg
                                              │
                                              │  TIM3 CH2 / TIM4 CH1, 3.2 MHz, 20 ms frame     hw.rs
                                              ▼
                                       Stingray internal pot loop    no feedback wire exists
```

There are exactly **four** calibration numbers per axis plus **one** sky number, and not one of them
has been measured yet.

### 1.1 Sky → mount frame (host)

`MountGeom` (`src/mount.rs:44`) holds the whole sky tie:

```
az_zero  = (az_center_bearing_deg − az_travel_deg/2) mod 360
bearing  = (az_zero + az_m) mod 360
```

With the shipped `az_center_bearing_deg = 217.5` and `az_travel_deg = 450`:

| quantity | value | meaning |
|---|---|---|
| `az_zero` | **352.5°** | the true bearing the boresight faces at mount azimuth 0 |
| mount az 225 | bearing 217.5° | mid *travel*, which is what `az_center_bearing_deg` is defined at |
| mount az 200 | bearing 192.5° | mid *window*, which is where `HOME` and the firmware's `PARK` go |
| mount az 400 | bearing 32.5° | top of the usable window |

`az_center_bearing_deg` is **the number tying mount azimuth to the sky** (elevation's optional tie,
`el_correction_deg`, is below). It is measured from true (geodetic) north, clockwise, at mid hardware
travel, and set with `/here` or `/bearing` (§6.7). Until then it is a guess: the comment
in `control.toml` says "middle of the 120..315 DISH SECTOR; the gap sits due north", which is a
*design intent*, not a measurement of the built pedestal.

Note the trap: `HOME` sends mid *window* (200), not mid *travel* (225), so the dish at `HOME` faces
192.5°, not 217.5°. Do not use `HOME` as the reference pose when calibrating.

Elevation's host-side tie is one optional number, `el_correction_deg`: how much higher the mount's
elevation reads than the true elevation, measured with `/here AZ EL` and 0 until then. The host adds it
to every elevation it sends (`el_m = el + el_correction_deg`) and takes it off every elevation it shows.
With it at 0, mount elevation is *defined* to be elevation above the level horizon, and what makes that
true is mechanical (a level pedestal) plus `EL_CAL.offset` in the firmware (§6.5). The correction lets a
dish whose level is a few degrees out track without a reflash; it costs the same few degrees at one end
of the elevation range, because the firmware's window (−2..91) stays in mount degrees. Fixing
`EL_CAL.offset` and clearing the correction gets them back.

### 1.2 No flip-over

There is no flip-over (removed 2026-10-01 for simplicity): elevation stops at 91°, one degree past vertical. Every sky direction
has exactly one mount solution: azimuth from the bearing, elevation as it is. Two consequences:

- A pass near the zenith swings the azimuth quickly at the top; the dish may lag there for a few
  seconds, which costs little because near the zenith an azimuth error barely moves the beam.
- A pass that runs across the window's seam (the bearing at mount azimuth 0, about north-northwest)
  cannot be held at one wrap, so `solve_path` schedules one unwind, about 16 s off the satellite at the
  20°/s slew limit. In a count of 908 real passes this was 3–6 % of them.

### 1.3 Mount frame → pulse (firmware)

`AxisCal` (`src/limits.rs:53`). Two frames, and the difference is the whole point:

```
gearbox angle  0 .. gear_travel      measured from the mechanical end stop
mount angle    = gearbox − offset    what the protocol and the host talk in

pulse_µs = clamp( pulse_at_zero + (clamp_mount(mount_deg) + offset) · us_per_deg,
                  pulse_min, pulse_max )
```

Shipped constants, and the physical reference each is measured from:

| | AZ (Stingray-4) | EL (Stingray-9) | measured from |
|---|---|---|---|
| `gear_travel` | 450.0° | 200.0° | vendor spec, mechanical stop to mechanical stop |
| `offset` | **8.0°** | **5.0°** | gearbox angle at mount zero |
| `limit_lo` / `limit_hi` | 0 / 400 | −2 / 91 | mount frame soft window |
| `gear_margin` | 2.0° | 2.0° | kept clear of *each* mechanical stop, independently |
| `pulse_at_zero` | 500 µs | 500 µs | pulse that puts the **gearbox** at 0° |
| `us_per_deg` | 2000/450 = 4.4444 | 2000/200 = 10.0 | slope, endpoint form |
| `pulse_min` / `pulse_max` | 500 / 2500 | 500 / 2500 | vendor PWM range |

Derived, computed from those constants:

| | AZ | EL |
|---|---|---|
| `gear_lo` → `gear_hi` | 8 → 408 | 3 → 96 |
| `mount_lo` → `mount_hi` | **0 → 400** | **−2 → 91** |
| pulse at window ends | 535.56 → 2313.33 µs | 530 → 1460 µs |
| slope, degrees per µs | 0.225 | 0.100 |

**What `offset` means physically.** `offset` is simultaneously the frame tie and the safety margin at
the low end. `AZ_CAL.offset = 8.0` asserts that mount azimuth 0 sits 8° off the azimuth hard stop:
8 rather than a tighter 2, so the window stays clear of both stops whichever of goBILDA's two slopes
the gearbox really follows (§1.4). It costs nothing: the window still spans 400°.
`EL_CAL.offset = 5.0` asserts that the boresight is **level** when the elevation gearbox is at 5° off
its own low stop. That second one is a claim about the built geometry of a 3D-printed cradle, a boom
and a 1 m dish, and it is the single most likely number in this system to be wrong.

**Changing `offset` moves the window in gearbox space**, because `gear_lo = limit_lo + offset` and
`gear_hi = min(limit_hi + offset, gear_travel − gear_margin)`. On elevation with `offset = 5` the
gearbox runs 3..96 of its 200, so the top has plenty of room. At the bottom, an `offset` below 4 makes
`mount_lo` shrink above −2 silently; the only sign is the number `CAL` reports back.

### 1.4 Pulse → shaft (hardware, uncalibrated)

The Stingrays are positional servo gearboxes with a 3-pin TJC8 connector and a 5 kΩ pot inside their
own control loop (`~/.openclaw/workspace/reports/stingray-power.md`). **The pulse width is the
position command and there is no feedback wire in existence.** PA0/PA1 are bare on purpose; the ADC
is not just unused, it is unclocked (`hw.rs:106`).

Timer resolution: 16 MHz HSI, PSC 4 → 3.2 MHz, ARR 63999 → 20 ms frames. A pulse is settable in
0.3125 µs steps, which is 0.070° of azimuth and 0.031° of elevation. Preload is on, so a width change
lands on a frame boundary.

**The vendor's own numbers do not agree.** goBILDA publish "450° travel" *and* "0.23°/µs" for the
Stingray-4 over a 500–2500 µs range. 0.23 × 2000 = 460°, not 450. The firmware uses the endpoint
form (2000 µs ÷ 450° = 4.4444 µs/deg). If the 0.23 figure is the true slope, the error depends on
where the servo's scale is pinned:

- pinned at 500 µs: the pulse for mount azimuth 400 (2313.33 µs) puts the gearbox at **417.1°**
  instead of 408°, a **9.1° pointing error** at the top of the window, rising roughly linearly from
  zero at the bottom;
- pinned at the usual servo centre (1500 µs = 225°): mount 0 lands at gearbox 3.2° instead of 8°
  (−4.8°) and mount 400 at 412.1° (+4.1°).

Under either reading the window stays at least 3° clear of both stops, and that is why `offset` is 8:
with the old offset of 2, the centred reading put mount 0 at gearbox −3°, and the simulated servo
stalled against the stop on `GO 0`. Still, that single unresolved 2% discrepancy dwarfs every other
error term below, and measuring the real slope is the highest-value thing on this page.

---

## 2. Mount zero, and how it is established

There is **no home switch, no index pulse, no limit switch, and no absolute encoder on either axis**.
Nothing on this mount can tell the firmware where the axes are. "Zero" is therefore not found, it is
*declared*, and the declaration has to be re-made every time power is lost.

### 2.1 The `.uninit` SRAM record

`Persist` (`limits.rs:139`) lives in a linker section the cortex-m-rt startup code does not clear:

```rust
#[unsafe(link_section = ".uninit.PERIGEE_POS")]
static mut SAVED: MaybeUninit<Persist> = MaybeUninit::uninit();
```

It carries a magic word (`0x50474D04`, changed only when the record's layout changes), the two
**pulse widths** on the wire, and a rotate-and-xor checksum. `valid()` rejects a stale image from an
older layout, a half-written record, uninitialised RAM that happens to hold the magic word, NaN or
infinity, and any pulse outside 500..2500 µs. It is rewritten every 20 ms servo frame, and on `ZERO`,
`STOP` and `OFF`, so a reset mid-slew restores where the dish is.

Because it stores pulses, not angles, a **reflash with new calibration constants does not move the
dish**: the new build drives the very pulse that was on the wire and reads it back as a mount angle
through the new constants (`Persist::to_mount`). You do not need to change the magic word when you
change `AZ_CAL` / `EL_CAL`.

What survives and what does not:

| event | SRAM | result on boot |
|---|---|---|
| watchdog reset (1 s stall) | kept | `POS <az> <el> restored, holding` + `NOTE reset by watchdog`, pulses resume at the same width |
| reset button | kept | `POS ... restored, holding` |
| `probe-rs run` / reflash | kept | `POS ... restored, holding`, same pulse: no jump even if the calibration changed |
| **power cycle** | **gone** | `POS UNKNOWN`, board stays limp, no pulses at all |

The restore path is genuinely good: `main()` reads `SAVED` before anything can touch it, and
`take_wire()` is called immediately, so the servos pick the pulses back up where they were instead of
staying limp and then jumping on the first command.

### 2.2 `POS UNKNOWN`: nothing moves until you say where the dish is

After a power cycle `known = false`, the servos are limp, and the firmware prints:

```
POS UNKNOWN: send ZERO az el before moving
```

Every command that would switch the pulses on (`GO`, `AZ`, `EL`, `PARK`, `RAW`) is refused until a
`ZERO` arrives:

```
GO 120 30         ERR position unknown: send ZERO az el first
```

Why: a positional servo drives to its first pulse at full speed, and the rate limiter can only shape
moves that start from a position it actually knows. (Earlier builds assumed the dish was at park and
drove there on the first move; a dish that was anywhere else got slammed across at full servo speed.)
So after a power cycle, look at the dish, and if it is where you parked it, type `ZERO 200 45`.
perigee-control shows `POSITION UNKNOWN` in MOTOR CONTROL and its LINK check refuses to start a pass
until you have.

### 2.3 `ZERO az el`

```
ZERO 200 0        OK ZERO 200.00 0.00
```

`ZERO` declares where the axes really are. It does not move anything: it sets `cmd`, sets `tgt` to
match, sets `known = true`, and saves. It is checked against the soft window and refused outside it.
perigee-control never sends it on its own; it is a console command Beck types.

It is accepted **only while the servos are limp** (after a power cycle, or after `OFF`):

```
ZERO 200 45       ERR ZERO only while the servos are limp: send OFF first (with pulses on, the pulse is the position)
OFF               OK OFF
ZERO 200 45       OK ZERO 200.00 45.00
```

With the pulses on, the servo already sits wherever the pulse on the wire puts it, so a `ZERO` that
disagreed with that pulse would be either a lie (the numbers change, the dish does not) or a jump
(the pulse is re-driven for the new number and the dish snaps across at full servo speed). Earlier
builds did the second. Limp first, the declaration and the dish agree, and the next move starts the
pulses from the declared pose.

`RAW` does not establish position. Like every move it is refused until `ZERO`, and it is an ordinary
ramped move to the angle its pulse width means (§5.2).

---

## 3. No encoders: both axes are open loop

Neither axis has an encoder, and none will be fitted. Both Stingrays are positional servo gearboxes
with a 3-pin TJC8 connector (signal, V+, GND) and a 5 kΩ pot that closes the loop *inside* the servo;
nothing comes back on the wire. The pulse width is the position, so every position the firmware
reports is a model: `tgt` (the clamped request), `cmd` (the slew-limited command, which is the pulse
on the wire) and `set` (`cmd` lagged by a pessimistic servo speed model). Telemetry fields 7-8 are
fixed at `NA -1` and kept only so the `T4` line shape does not change. PB8/PB9 and I2C1 are unused.

What that costs, stated once so nobody designs around a measurement that does not exist:

- A stalled, unpowered or slipping axis cannot be detected. The model says "there" either way.
- After a power cycle the position is genuinely unknown (§2.2) and only a human can re-declare it.
- Every constant in §1.3 has to be right on the bench, because nothing downstream corrects it.

---

## 4. Host-side calibration: what exists

Very little, and it is worth being blunt about it.

**Boot page** (`src/boot.rs:246`, `queue_checks`) is a *configuration* readout, not a calibration
check. It prints config, viewer station, data-store file ages, engine status, serial ports, and then:

```
MOUNT FRAME     AZ 0..400 OF 450 TRAVEL   EL -2..91
CALIBRATION     UNCALIBRATED. FRAME IS NOMINAL 217.5 DEG; SIMULATOR ONLY, TRACKING A REAL MOUNT REFUSED (/HERE AZ EL TO TIE IT)
CABLE WRAP      AZ HELD INSIDE 0..400 DEG   PARK 200/45   RATE 20/15 DEG/S
```

(Once `/here` or `/bearing` has saved a tie, the CALIBRATION line reads `CENTRE BEARING 211.44 DEG,
MEASURED   FROM calibration.toml`, with `EL CORRECTION +2.00 DEG` before the comma when `/here` measured one.)

It never talks to the mount. It cannot: the serial link opens after the boot page. It does not read
`CAL` back from the firmware and does not compare the host's window against the firmware's. `POS
UNKNOWN` is caught later, from telemetry: once the link is open, the `K` flag in every `T4` line drives
the `POSITION known` / `POSITION UNKNOWN` line in MOTOR CONTROL, and the procedure's LINK check refuses
a real mount whose position is unknown.

**On connect**, `hello()` (`command.rs`) sends exactly three lines:

```
ID
RATE 20.0 15.0
TEL 5.0
```

No `CAL`, no `?`, no `ZERO`, no check that the firmware's reported window matches the host's. The
`ID` response (`ID PERIGEE-MOUNT fw4-stm32 PROTO T4 AZ 0-400 EL -2-91`) carries the firmware's real window
and the host parses it into a display string and nothing else.

**No north alignment, no level alignment, no sun or star pointing, no satellite-based offset fit, no
pointing model of any kind.** Verified by grep across `src/`: there is no sun ephemeris, no star
catalogue, no signal-strength peaking, no residual logging. The only calibration inputs the host has
are `az_center_bearing_deg` and `el_correction_deg`, saved into `calibration.toml` with `/here` or
`/bearing` (§6.7). `/here` solves them from one reference you point the dish at, and from a second
one it reports the azimuth motor's scale error; that is the whole of its offset solving.

### 4.1 The on-point check

The tracker's `SLEW` step (`tracking.rs`, `Phase::Slew`) passes only when the firmware's own telemetry
says the dish is there (`mount.rs`, `Pose::on_point`):

```rust
self.known && !self.moving && !self.clip[0] && !self.clip[1]
    && near(self.tgt, asked)          // the firmware holds the target the GO just sent
    && near(self.settled, self.tgt)   // and its settle estimate has reached that target
```

`near` means within `on_point_deg` (0.5°) on both axes. `MountState::ingest` parses `T4` (fields 1-2
the target, 3-4 the ramped command, 5-6 the settle estimate, `K` / `M` / `A` / `E` from the flags); a
firmware that still sends the old `T` line can never confirm a position. While it waits, the SLEW line
says what for (`position unknown (ZERO az el)`, `target clipped at the azimuth limit`, `moving`,
`settling, 2.3 deg to go`, ...), and if AOS arrives first, ARMED says `mount NOT confirmed on point`
with the reason instead of claiming it.

Remember what this checks: the firmware's **estimate**. With no encoder, `set` is the command lagged by
a speed model, so "on point" means "the servo has had time to get there", not "it was seen there" (§3).
The built-in simulator (SIM button) speaks `T4` with the same settle lag, refuses `ZERO` while powered
and answers `RAW` with an error, so the host tests exercise this same path. One difference stays on
purpose: it starts at park with the position known, so the SIM button runs a pass straight away, where
a real board after a power cycle starts `POS UNKNOWN`.

---

## 5. What is not implemented

Absolute position
- No encoder on either axis, by decision (§3).
- No home switch, index mark, or limit switch on either axis.
- No persistence across a power cycle. Would need the RTC backup domain and a VBAT cell; the README
  calls that a deliberate next step rather than something this build claims.

Pointing model
- Nothing beyond one azimuth bearing scalar and two firmware offsets. Specifically absent: pedestal
  tilt (two terms), azimuth/elevation axis non-perpendicularity, boresight collimation error,
  gravitational sag of a 150 mm boom carrying a 1 m dish, and any direction-dependent
  backlash term.
- The deg↔µs map is a straight line through two assumed endpoints. No measured intermediate points,
  no non-linearity term, and the two endpoints themselves are vendor claims that contradict each
  other by 2%.

Verification
- No sun, star, or known-satellite pointing check anywhere in the codebase.
- No residual logging, so even if a pass is tracked there is no record of how far off it was.
- `lead_seconds = 0.4` is a fixed guess at servo lag. Nothing measures the real lag, and with no
  feedback nothing can.

### 5.1 Config disagreements (resolved)

The three mismatches recorded here (`el_min_deg = -5`, missing `az_limit_lo/hi_deg`, `park_az_deg =
225`) are fixed in `control.toml`, and the host test
`the_shipped_control_toml_matches_the_firmware_and_carries_no_calibration` now fails if any of them
comes back. `MountCfg::clamp_to_firmware` also narrows anything wider than the firmware window on load.

### 5.2 Documented procedures that did not work (resolved)

`RAW` turns the requested pulse into the mount angle it means (`deg_of_pulse`), clamps that into the
window (`clamp_mount`) and moves there exactly like `GO`: ramped at the `RATE` limit, with telemetry
that tells the truth about where the command is. So `RAW AZ 500` ends at 535.56 µs and `RAW AZ 2500`
at 2313.33 µs: **`RAW` cannot reach a mechanical stop, by design.** The old "drive to both stops" step in `docs/wiring.md` and the old `FB` / `FB_CALIBRATED`
procedure in the firmware `README.md` both contradicted that; both now point here, and §6.3 below
calibrates inside the window instead.

---

## 6. Bench calibration procedure

**Do stages 1–5 with the dish off the boom.** Counterweight for the boom alone, or leave the cradle
unloaded. A mis-set offset with 9 kg on a 150 mm arm driving into a stop is how a printed cradle
becomes two printed cradles.

Tools: digital angle gauge or inclinometer reading to 0.1° (a £20 magnetic one is fine), a protractor
or printed 360° disc for the azimuth deck, a bubble level, shims, a tape measure, and a laptop on
`/dev/ttyACM0`.

Talk to the board from the SERIAL CONSOLE tile, or directly:

```sh
stty -F /dev/ttyACM0 115200 raw -echo
cat /dev/ttyACM0 &                       # watch replies
printf 'PING\r\n' > /dev/ttyACM0
```

### 6.0 Before touching the hardware

```sh
cd ~/projects/perigee/perigee-control/firmware/perigee_mount_stm32
./test_host.sh          # 18 tests on the limit arithmetic, no board needed
```

`control.toml` already mirrors the firmware window (§5.1); `cargo test` in `perigee-control` checks it.

Power, from `docs/wiring.md`, each rule with its own way of destroying something: servo red and black
straight to the 7.4 V rail, never through the Nucleo; rail negative tied to a Nucleo GND.

### 6.1 Link and window check

```
PING                → PONG
ID                  → ID PERIGEE-MOUNT fw4-stm32 PROTO T4 AZ 0-400 EL -2-91
CAL                 → OK CAL AZ 0.00..400.00 deg 536..2313 us  EL -2.00..91.00 deg 530..1460 us  known 0
TEL 2
```

`known 0` after a power cycle is correct and expected. Confirm the pulse endpoints `CAL` reports
match the table in §1.3; if they do not, the firmware on the board is not the firmware in the tree.

### 6.2 Level the pedestal

Everything downstream assumes the azimuth axis is vertical. Bubble level across the flange plate in
two perpendicular directions, shim the feet until both read level, then check again at four azimuth
positions 90° apart (`GO 50 45`, `GO 140 45`, `GO 230 45`, `GO 320 45`) — if level changes with
azimuth, the deck is not flat and no amount of software fixes it.

Record the residual. A 0.5° pedestal tilt is a 0.5° pointing error that varies sinusoidally with
azimuth, and with no pointing model there is nowhere to put it.

### 6.3 Measure the azimuth slope — the important one

This replaces the broken `RAW AZ 500 / 2500` step and resolves the 450°-vs-0.23°/µs contradiction
that is worth up to 9° of pointing error.

Tape a printed 360° disc to the fixed deck, with a pointer on the rotating head. Then, staying inside
the legal window the whole time:

```
ZERO 200 45                 # declare the current pose (nothing moves until you do)
RAW AZ 600                  # ramps there at the RATE limit, like GO
```

If `ZERO` answers `ERR ZERO only while the servos are limp`, the board kept its position through a
reset and is still driving it: send `OFF`, check the dish really is at 200/45, then `ZERO`.

Record the pointer angle. Then step and record at each of:

```
RAW AZ 700      RAW AZ 900      RAW AZ 1100     RAW AZ 1300
RAW AZ 1500     RAW AZ 1700     RAW AZ 1900     RAW AZ 2100     RAW AZ 2250
```

Wait for the axis to stop at each step, read the pointer to the nearest 0.5°, and record both numbers.
Then repeat the whole sweep *downwards* from 2250 to 600 — the difference between the two sweeps at
the same pulse is the gearbox backlash plus the 4 µs deadband, and you want that number written down
even though nothing in the firmware can currently use it.

**Unwrap the readings before fitting.** The sweep turns the head about 380°, but the disc only reads
0..360: somewhere in the sweep the pointer passes the disc's 0 mark and the reading drops from about
350 back to about 10. Add 360 to every reading after that point (on the way down, the same in reverse),
so the numbers keep rising with the pulse. Fitting the raw readings is meaningless: in the simulated run
it gave 28.9 µs/deg instead of 4.35.

Least-squares the ten (pulse, angle) pairs. The slope is `us_per_deg`; use the *mean* of the up and
down sweeps so backlash does not bias it. Expect 4.44 µs/deg if goBILDA's travel figure is right,
4.35 if their °/µs figure is right. Put the measured value in `limits.rs`:

```rust
pub const AZ_CAL: AxisCal = AxisCal {
    us_per_deg: 4.4444,        // ← replace with the measured slope
    pulse_at_zero: 500.0,      // ← and the measured intercept, see below
```

Leave `pulse_at_zero` alone for now. It is the pulse at *gearbox* 0, which needs an absolute
reference (the low stop), and §6.4 measures it. Reflash with the new `us_per_deg` before §6.4.

Repeat the whole stage for elevation with an inclinometer on the boom instead of a protractor on the
deck: `RAW EL 600` through `RAW EL 1400` in 100 µs steps, both directions. `RAW` is clamped into the window, which ends at 91° (1460 µs with the
shipped constants), so pulses above that all land on the same angle and would spoil the fit. Expect
10.0 µs/deg; the vendor's two figures agree on this axis.

The sweep stays below vertical, so the inclinometer readings (angle to the horizontal, 0..90) rise
steadily. If a reading ever goes *down* as the pulse goes up, the boom has passed vertical: use
`180 − reading` for it.

### 6.4 Azimuth zero: measure `pulse_at_zero`, keep `offset` = 8

`offset` is not something you measure. It is the margin you *choose* to keep between mount azimuth 0
and the low stop (8°). What the stop tells you is where the gearbox really sits for a given pulse,
which is `pulse_at_zero`. With §6.3's slope already flashed, approach the stop slowly:

```
RATE 2 15                   # slow, so a stop is felt not slammed
GO 0 45                     # the bottom of the window: offset says this is 8° off the stop
```

If the axis **touches the stop** (it stops short of where it was heading, and the bench supply's
current climbs to about 3 A with nothing moving), send `GO 5 45` at once, then step up 2° at a time
(`GO 7 45`, ...) until there is a visible gap. Note the mount azimuth `m` you ended on (0 if it never
touched), and measure the gap `g` in degrees of gearbox rotation from the stop (tape on the drum
against a mark, converted to degrees).

The firmware sent `p = pulse_at_zero + (m + offset) × us_per_deg` (for `m = 0` that is simply the
first number of the AZ pulse range `CAL` prints), and at that pulse the gearbox is really at `g`. So:

```
pulse_at_zero_new = pulse_at_zero + (m + offset − g) × us_per_deg
```

Put that in `AZ_CAL.pulse_at_zero`, leave `offset = 8.0`, reflash, and repeat `GO 0 45`: the gap should
now read 8° (within the tape's 0.5°). Restore `RATE 20 15` when done.

Example from the simulated bench: slope 4.3511 µs/deg, `GO 0 45` did not touch and left a 3.5° gap,
so `pulse_at_zero_new = 500 + (0 + 8 − 3.5) × 4.3511 = 519.58`; the next `GO 0 45` showed 8.0°.

Why not simply "set `offset` to the measured gap": changing `offset` moves the pulse sent for mount 0
by the same amount, so the gap moves with it and the rule never settles. On the simulated bench
(starting from the old offset of 2) it went 2 → 5 → 1 → 4 → 7 → 2.5, touching the stop four times.

Do §6.7 (north) after this and §6.5. Changing `AZ_CAL` afterwards moves the frame, and north would
have to be measured again.

### 6.5 Elevation zero and `offset` — level the boresight

The number most likely to be wrong in the whole system.

```
GO 200 0
```

Put the inclinometer on a surface you trust to be parallel to the boresight: the dish backing plate,
or the boom's flat if the boom was printed square to the dish mount. Read it.

The mount frame is defined by `mount = gearbox − offset`: at command 0 the firmware puts the gearbox at
`offset`. If the inclinometer reads `r` degrees (positive = boresight above level), the gearbox angle
that is really level is `offset − r`, so

```
offset_new = offset − r          # reads +3.0 with offset 5.0  ->  2.0
                                 # reads −3.4 with offset 5.0  ->  8.4
```

A boresight that is **high** needs a **smaller** offset. Reflash, repeat, confirm it reads 0.0 ± 0.2.
If the error grows instead of shrinking, the sign is wrong: undo it before the next reflash (the wrong
sign doubles the error; on the simulated bench it went from −3.5° to −6.5°).

Then check the window did not shrink:

```
CAL      → EL window must still read -2.00..91.00
```

With `offset = 8` the elevation gearbox runs 6..99 of 200, which is fine. Below `offset = 4` the
bottom of the window shrinks (the firmware keeps the gearbox 2° off its stop); the top has about 100°
of spare travel, so no sensible offset reaches it. `CAL` is the only thing that will tell you. If the
window shrinks, the fix is `EL_CAL.pulse_at_zero`, not a bigger or smaller offset.

### 6.6 Elevation repeatability (there is no encoder to calibrate)

With no encoder, the inclinometer is the only check elevation will ever get, so take it once at
several angles and from both directions:

```
GO 200 0     GO 200 30     GO 200 60     GO 200 90
```

then the same list in reverse. At each stop read the inclinometer on the boom flat. Up and down
readings at the same command should agree within the 4 µs deadband (0.4°); a bigger gap is backlash
or the arm sagging under load, and it is a pointing error nothing in the software can absorb. Write
both columns down next to the §6.3 slope fit.

### 6.7 North: `az_center_bearing_deg`

Each method gives the same thing: the true bearing the **boresight** faces (where the dish points, not
where a pointer on the head points) at a known mount azimuth `az_m`, from which

```
az_center_bearing_deg = (measured_bearing − az_m + 225) mod 360
```

**The quick way: `/here`.** With the dish pointing at a reference whose true bearing (and elevation)
you know, type `/here AZ EL` in perigee-control's SERIAL CONSOLE. It takes `az_m` and `el_m` from the
firmware's telemetry (refused while the position is unknown or the dish is still moving), works out the
formula above and `el_correction_deg = el_m − EL`, saves both to `calibration.toml` and uses them at
once. Give `EL` only when the reference pins elevation too (the sun's feed shadow centred on a covered
dish, a satellite's signal peaked, an inclinometer reading); the plumb lines below line up azimuth only,
so after them use `/here AZ`, which leaves the elevation correction as it is. `/here` shows the tie in
use and where the dish points now; `/here clear` forgets the tie (the mount is uncalibrated again).

Take a second `/here` at least 30° round in azimuth. The old tie was exact at the first reference, so
how far off it is at the second, over the turn between them, is the azimuth motor's scale error (the 2%
vendor discrepancy of §1.4). The console prints it, with the factor to multiply `AZ_CAL.us_per_deg`
by; after that reflash, `/here clear` and take the references again, because every mount azimuth has
moved. One reference fixes the offsets; only the scale makes pointing right far from the reference.

The methods below are ways to know a reference's true direction precisely, and to point the dish at
it; each can end in `/here` instead of the arithmetic.

**Sighting the boresight.** Hang a plumb line from the centre of the feed and another from the centre
of the dish (the vertex). Both points lie on the boresight, so both strings hang in its vertical plane
at any elevation. Stand behind the dish and line the two strings up on the target, like the two sights
of a rifle. Keep the dish low (`GO az_m 10`) so the strings hang far apart: about the focal length,
and 1 mm of misjudged overlap at 0.4 m apart is 0.14°.

**Compass (±3°, use only as a sanity check).** Magnetic declination at the station
(29.2452787 N, 81.1031710 W) is **−7.22°** — verified 2026-10-01 against NOAA's WMM-2025 calculator,
uncertainty ±0.34°, drifting −0.07°/yr. So `true = magnetic − 7.22`. A phone compass near a mount
full of brushed DC motors and 10 A of cable is worth less than that uncertainty suggests.

**Distant landmark (±0.2°, good).** Pick something sharp and far: a tower, a mast, a chimney, 1 km or
more. Get its coordinates from OpenStreetMap or satellite imagery, compute the true bearing from the
station, move the mount until the two plumb strings line up on it, and read the mount azimuth. At
1 km, 0.2° is 3.5 m of lateral error in picking the point, so pick a thin vertical thing.

Compute the bearing on the WGS-84 ellipsoid (the World Geodetic System 1984 shape of the Earth), not
on a sphere. The spherical great-circle formula uses one radius in every direction, but at this
latitude the Earth's north–south radius of curvature is 6351 km and its east–west one 6383 km: for the
example landmark below that is 0.14° of bearing, most of the error budget. Turning both points into
Earth-centred coordinates and the difference into local east, north and up is exact at any distance:

```python
import math
a, f = 6378137.0, 1 / 298.257223563                  # WGS-84 radius (m) and flattening
e2 = f * (2 - f)
def ecef(lat, lon, h):                                # Earth-centred x, y, z in metres
    p, l = math.radians(lat), math.radians(lon)
    N = a / math.sqrt(1 - e2 * math.sin(p) ** 2)      # east-west radius of curvature
    return ((N + h) * math.cos(p) * math.cos(l), (N + h) * math.cos(p) * math.sin(l), (N * (1 - e2) + h) * math.sin(p))
lat1, lon1, h1 = 29.2452787, -81.1031710, 10.0        # station, from viewer.toml [station] (alt_m)
lat2, lon2, h2 = 29.2600000, -81.0900000, 60.0        # the landmark and the height of the point you sight: replace
s, t = ecef(lat1, lon1, h1), ecef(lat2, lon2, h2)
d = [t[i] - s[i] for i in range(3)]
p, l = math.radians(lat1), math.radians(lon1)
east = -math.sin(l) * d[0] + math.cos(l) * d[1]
north = -math.sin(p) * math.cos(l) * d[0] - math.sin(p) * math.sin(l) * d[1] + math.cos(p) * d[2]
up = math.cos(p) * math.cos(l) * d[0] + math.cos(p) * math.sin(l) * d[1] + math.sin(p) * d[2]
print(f"true bearing to landmark: {math.degrees(math.atan2(east, north)) % 360:.3f}"
      f"   elevation {math.degrees(math.atan2(up, math.hypot(east, north))):.2f}")
```

For the example coordinates it prints 38.114 (the spherical formula gave 37.972).

**Sun shadow on the deck (±0.1° for the north line, and it costs nothing).** Stand a vertical pin
on the azimuth deck, note the exact UTC time, and mark the shadow. The shadow's bearing is the sun's
azimuth plus 180°. Compute the sun's azimuth for that instant with the NOAA solar calculator
(`https://gml.noaa.gov/grad/solcalc/`) or:

```python
# Low-precision solar position, good to ~0.02 deg, enough for a 12 deg beam.
# NOAA solar calculator algorithm; feed it UTC.
import math, datetime
t = datetime.datetime(2026, 10, 2, 17, 30, 0)        # UTC of the shadow mark
lat, lon = 29.2452787, -81.1031710
jd = t.toordinal() + 1721424.5 + (t.hour + t.minute/60 + t.second/3600)/24
n = (jd - 2451545.0) / 36525.0
L = (280.46646 + n*(36000.76983 + n*0.0003032)) % 360
g = math.radians((357.52911 + n*(35999.05029 - 0.0001537*n)) % 360)
C = (1.914602 - n*(0.004817 + 0.000014*n))*math.sin(g) + (0.019993 - 0.000101*n)*math.sin(2*g) + 0.000289*math.sin(3*g)
lam = math.radians(L + C - 0.00569 - 0.00478*math.sin(math.radians(125.04 - 1934.136*n)))
eps = math.radians(23 + (26 + (21.448 - n*(46.815 + n*(0.00059 - n*0.001813)))/60)/60)
dec = math.asin(math.sin(eps)*math.sin(lam))
ra = math.atan2(math.cos(eps)*math.sin(lam), math.cos(lam))
gmst = math.radians((280.46061837 + 360.98564736629*(jd - 2451545.0)) % 360)
H = gmst + math.radians(lon) - ra
p = math.radians(lat)
el = math.asin(math.sin(p)*math.sin(dec) + math.cos(p)*math.cos(dec)*math.cos(H))
az = math.degrees(math.atan2(math.sin(H), math.cos(H)*math.sin(p) - math.tan(dec)*math.cos(p))) + 180
print(f"sun az {az % 360:.3f}  el {math.degrees(el):.3f}")
```

Read the shadow line on the §6.3 disc (numbered clockwise seen from above, the same way as azimuth).
If it reads `d_s`, any disc reading `d` is the true bearing `(d − d_s + sun_az + 180) mod 360`.

That orients the **disc**, so it gives the true bearing of the **pointer**, and the pointer is not the
boresight: it was taped on by eye. In the simulated session an untested pointer put 0.36° into the
bearing. Tie the two together once, before trusting the number: with the head at `az_m`, line the two
plumb strings up on any object 500 m or more away (no coordinates needed), then lay a straightedge on
the disc from its centre towards the same object and read where it crosses the rim (`d_o`). The
boresight's true bearing at `az_m` is then

```
measured_bearing = (d_o − d_s + sun_az + 180) mod 360
```

Write down the pointer's offset too, `d_o − d_p` with `d_p` the pointer's reading at the same moment:
every later pointer reading needs it added. Sighting along a short straightedge is the weak step
(about ±0.2°), so the result is about as good as the landmark; do both and compare.

**Check it on the boresight (§7.1), with the sun low.** The sun shadow target looks at the boresight
itself, with no pointer and no landmark in the chain. Do it with the sun below about 25° (morning and
afternoon): the dish's collimation error grows as 1/cos(el) in azimuth, so a check with the sun high
folds it into the bearing (in the simulated session a check at 57° elevation left the bearing 0.58°
off; checks at 20° left it 0.18° off). Command the sun's computed az/el, then jog the azimuth until the
pin's shadow is centred on the crosshair. If that took a jog of `j` degrees of mount azimuth (positive
= more azimuth), the bearing is `j` too high. One reading carries up to ±0.45° of servo deadband, so
take three in the morning and three in the afternoon, approaching from both sides, and average them.
If the average is under about 0.3°, the bearing you have is as good as this mount can tell; if it is
larger, set `/bearing` to the old value minus the average `j`.

Save the bearing from perigee-control's SERIAL CONSOLE, either while the dish is still on the
reference or afterwards with the arithmetic done:

```
/here 38.114         # the dish is on the landmark (plumb lines: azimuth only, so no elevation)
/bearing 211.44      # or: the computed az_center_bearing_deg
```

Both write `calibration.toml` next to `control.toml` and use the value at once (no restart); `/here`
and `/bearing` on their own show the tie in use. `/bearing` keeps any elevation correction. Do **not**
put the keys in `control.toml`: a measured number lives only in the calibration file, and the
shipped-file test fails if `control.toml` carries one.

North does not touch the firmware: `pulse_at_zero` came from the stop in §6.4, and `/here` and
`/bearing` only move the host's frame, so nothing needs reflashing (unless `/here` reports a scale error).

---

## 7. Verifying the pointing

### 7.1 Sun shadow boresight test (daytime, best sensitivity)

Build a shadow target: a disc of card over the feed with a pin through its centre standing `h` mm
proud, and a crosshair on the dish vertex `h` mm below. Command the mount at the sun's computed az/el
for a recorded UTC time (§6.7 gives az/el), and measure how far the pin's shadow falls from the
crosshair.

```
pointing error = atan(d / h)
```

With `h = 200 mm`, **1° of error is 3.5 mm of shadow offset** — readable with a ruler, and about
fifteen times finer than the azimuth deadband. Resolve the offset into azimuth and elevation
components: lateral error divided by cos(el) is the azimuth error, vertical error is the elevation
error.

Repeat at four or five sun positions across a day (morning, mid-morning, noon, afternoon). The
pattern is diagnostic:

| residual pattern | cause |
|---|---|
| constant azimuth offset at all elevations | `az_center_bearing_deg` is wrong |
| azimuth error growing with mount azimuth | `us_per_deg` is wrong — the 2% slope problem |
| constant elevation offset | `EL_CAL.offset` is wrong |
| azimuth error ∝ 1/cos(el), sinusoidal in azimuth | pedestal tilt |
| elevation error growing with elevation | axis non-perpendicularity, or boom sag |

**Never look down the boresight at the sun, and do not leave a 1 m dish staring at it for long with
anything flammable at the focus.** 0.785 m² of collecting area at ~1 kW/m² concentrates to a point.

### 7.2 Known GEO satellite (static, repeatable, no ephemeris needed)

A geostationary target does not move, so there is no timing error, no lead-time guess, and no pass
window — the single best repeatable check once a receiver is on the feed. Computed for the station at
29.2452787 N, 81.1031710 W, with `az_center_bearing_deg = 217.5` (so mount az = true bearing − 352.5,
mod 360, taken inside the 0..400 window):

| satellite | GEO longitude | true bearing | elevation | mount az | `GO` command |
|---|---|---|---|---|---|
| GOES-19 (GOES-East) | 75.2° W | 168.04° | 55.30° | 175.54 | `GO 175.54 55.30` |
| Galaxy 19 | 97.0° W | 210.26° | 51.69° | 217.76 | `GO 217.76 51.69` |
| SES-1 | 101.0° W | 216.56° | 49.51° | 224.06 | `GO 224.06 49.51` |
| Eutelsat 113 West A | 113.0° W | 231.90° | 41.29° | 239.40 | `GO 239.40 41.29` |
| GOES-18 (GOES-West) | 137.0° W | 251.72° | 21.19° | 259.22 | `GO 259.22 21.19` |

(Spherical-GEO geometry at r = 42164.17 km, WGS-84 station. Good to ~0.05°, which is far inside
anything this mount can resolve. Note these elevations are above the DISH SECTOR's `el_max = 45`, so
pick them from FULL SKY in the viewer or drive them by hand.)

Procedure: `GO` the computed pose, then jog in 0.5° steps (`motor.step`, cycled with the STEP button)
and record the pose of peak signal. The difference between computed and peak is the pointing error at
that one sky point. Five satellites across 84° of azimuth and 34° of elevation gives five residuals,
which is enough to separate a constant offset from a slope error.

A clean single-point check with no receiver at all: Ku-band GEO sats are bright enough that a cheap
satellite finder meter on an LNB will peak on them audibly.

### 7.3 Beamwidth, or how good this needs to be

The 1 m dish (`params.json: dish_diameter = 1000`) at the usual half-power beamwidth approximation
70λ/D:

| band | frequency | half-power beamwidth | tolerable pointing error |
|---|---|---|---|
| NOAA APT / meteor scatter | 137 MHz | 153° | irrelevant, any pointing works |
| UHF cubesats | 400 MHz | 52° | ±10° is fine |
| **NOAA/Metop HRPT** | **1.7 GHz** | **12.3°** | **±3°** |
| S-band / 2.4 GHz downlinks | 2.4 GHz | 8.7° | ±2° |
| C-band GEO | 4 GHz | 5.2° | ±1.3° |
| Ku-band GEO | 11 GHz | 1.9° | ±0.5° |

Set against that, the current error budget:

| term | magnitude | status |
|---|---|---|
| azimuth slope ambiguity (450° vs 0.23°/µs) | **up to 9°** at the top of the window | unresolved, dominates everything |
| `az_center_bearing_deg` set by design intent, not measured | unknown, plausibly 5–20° | unresolved |
| `EL_CAL.offset` as built geometry, not measured | unknown, plausibly 2–10° | unresolved |
| pedestal level | unknown | never measured |
| azimuth servo deadband | 0.9° | vendor spec, irreducible without feedback |
| elevation servo deadband | 0.4° | vendor spec, irreducible without feedback |
| gearbox backlash, 4:1 and 9:1 | unknown | never measured |
| boom and dish sag under 9 kg | unknown | FEA exists in `perigee-mount/tools_fea.py`, never tied to pointing |
| pulse quantisation | 0.070° az / 0.031° el | negligible |

**Pointing accuracy is currently not merely poor, it is unknowable** — the two largest terms are
unmeasured constants, not noise. Running §6 reduces them to the deadband plus backlash, which puts
1.7 GHz HRPT comfortably in reach and leaves Ku-band GEO out of it.

---

## 8. The strongest fix

In the order worth doing them.

**1. Resolve the azimuth slope (§6.3). Two hours with a printed protractor.** Nothing else matters
until the 9° term is gone, and it needs no new hardware.

**2. (Dropped.) Encoders.** An absolute encoder per axis was the obvious way to make `POS UNKNOWN`
disappear. Beck will not fit one, so position stays a declared number, and item 5 is what keeps it
across a power cycle.

**3. Star camera and a real pointing model.** This is the ambitious one and it is the correct answer
for a mount whose offsets are all unknown. A Raspberry Pi camera module with a 25 mm lens, bolted
coaligned with the boresight, has roughly a 15° field — comfortably wider than the 12.3° radio beam,
so anything the dish can see the camera can frame. Point at 25–30 stars spread over the sky, plate
solve each frame (`astrometry.net` solves a 15° field in under a second and gives arcsecond-level
centres), and fit the standard six-term model:

```
IA   azimuth index offset          ← replaces az_center_bearing_deg, measured not guessed
IE   elevation index offset        ← replaces EL_CAL.offset, measured not guessed
AN   azimuth axis tilt, north      ← the pedestal level term there is nowhere to put today
AW   azimuth axis tilt, west
CA   boresight collimation         ← the boresight's sideways offset from the cradle, measured
NPAE axis non-perpendicularity     ← elevation axis not square to azimuth, measured
```

Six terms from thirty measurements, solvable by least squares in fifty lines, and the residual tells
you honestly what the mount's repeatability is. It subsumes every manual stage in §6 except the slope
measurement, works at night with no sun-safety problem, and can be re-run in twenty minutes whenever
something is bumped. It is also the only approach on this list that produces a *number* for pointing
accuracy rather than an assumption.

A sun sensor is the cheaper cousin of the same idea (one photodiode quad, daytime only, one target),
and it is a reasonable intermediate step. The star camera is strictly better for the same order of
money.

**4. (Done.) The host reads the telemetry (§4.1).** `T4` carries the target, the command and the
settle model separately, and perigee-control parses all three: SLEW waits for the firmware's own
on-point estimate and says what it is waiting for.

**5. RTC backup registers plus a CR2032 on VBAT.** Position survives a power cycle, `POS UNKNOWN`
becomes rare rather than routine. With no encoders this is the only way to keep a position across a
power cycle.

---

## 9. Quick reference

```
PING                PONG
ID                  ID PERIGEE-MOUNT fw4-stm32 PROTO T4 AZ 0-400 EL -2-91
?                   T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags
CAL                 OK CAL AZ lo..hi deg lo..hi us  EL lo..hi deg lo..hi us  known 0|1
ZERO az el          declare where the axes are; does not move anything; only while limp (OFF first)
RAW AZ|EL us        move to the angle that pulse means: ramped like GO, clamped into the window
GO az el            move; ERR + clamp if outside the window
RATE az_dps el_dps  slew limits, 0.1..90
TEL hz              telemetry rate, 0..20, 0 = off
STOP / PARK / OFF   hold here / go to 200 45 / pulses off, servos limp
```

Where each constant lives:

| constant | file | measured from |
|---|---|---|
| `az_center_bearing_deg` | `perigee-control/calibration.toml`, saved with `/here` or `/bearing` | true north, at mount azimuth 225 |
| `el_correction_deg` | same file, saved with `/here AZ EL` | mount elevation minus true elevation |
| `az_limit_lo/hi_deg`, `el_min/max_deg` | `perigee-control/control.toml` `[mount]`, **must match `limits.rs`** | mount frame |
| `AZ_CAL` / `EL_CAL` offsets, slopes, windows | `firmware/.../src/limits.rs`, `AZ_CAL` / `EL_CAL` | mechanical end stop (gearbox frame) |
| `PARK`, `RAW_US`, `TICK_US` | `firmware/.../src/main.rs`, top of file | mount frame / µs / µs |
| station coordinates, elevation mask | `perigee-viewer/viewer.toml` `[station]` | WGS-84 |
| gimbal dimensions | `perigee-mount/params.json` | mm, as built |

One mismatch worth noting while you are in there: `mount.rs:448` sets `EL_AXIS_Z = 599.0` "from
perigee-mount/params.json", but that file says `el_axis_height: 510`. Wireframe drawing only, no
effect on pointing, but it means the MOUNT tile is drawing a mount 89 mm taller than the one being
built.

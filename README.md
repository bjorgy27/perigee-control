# Perigee Control

The ground-station control base for Perigee, in one window: a **boot page** (checks, Space-Track login,
an engine run if you want fresh data), then the **orbit viewer** as one tile beside the four command
tiles, laid out like Hyprland but inside the window, with bright dividers between them. Pick a
satellite on the globe and watch the mount run the procedure and take the pass.

```
perigee-control/            this crate: `cargo run --release` from here
  control.toml              command page, mount, serial and tracking settings
  src/main.rs               builds the viewer's App (perigee_viewer::build_app), adds ControlPlugin and BootPlugin
  src/boot.rs               the boot page: checks, Space-Track login, engine run, load, then the tiles
  src/command.rs            the control page: the two cameras, tiles, keys, buttons, text, overlays, dividers
  src/tiles.rs              dwindle layout tree (focus / swap / resize / zoom / hide) incl. the ORBIT VIEW tile
  src/input.rs              input routing: keys reach the viewer only while ORBIT VIEW has the focus
  src/tracking.rs           next-pass finder, pass sampling, the ARM -> TRACK -> PARK sequence
  src/mount.rs              mount frame, path solver (cable wrap), telemetry, wireframe
  src/serial.rs             termios serial link on a worker thread + the mount simulator
  src/console.rs            scrollback, input line, history
  firmware/perigee_mount_stm32/  Nucleo-F401RE firmware: the mount's safety layer (docs/wiring.svg)
  firmware/perigee_mount/   superseded Arduino Uno sketch, kept for reference only
../perigee-viewer           the viewer, unchanged, used as a library
../Perigee                  the engine's outputs (SORTED_SATS.json, SATELLITE_RANKS.json, ...)
```

## Running

```
cd ~/projects/perigee/perigee-control
cargo run --release
```

One window opens, `PERIGEE // control`, on the boot page. Settings: `control.toml` here, and the viewer's
own `viewer.toml` (path in `[viewer] config`). `[data] dir` says where Perigee's outputs are (the engine
writes into the folder it is run from: `../Perigee` when run by hand there, `../Perigee/src` under the
systemd timer). `[perigee] bin` / `dir` say how to run the engine; `[boot] enabled = false` skips the
boot page entirely.

Debug switches in the environment: `PERIGEE_BOOT=0` skips the boot page, `PERIGEE_SCREENSHOT=<prefix>`
saves `<prefix>-<seconds>.png` of the window every 3 s (`PERIGEE_SCREENSHOT_EVERY` changes that), `PERIGEE_BOOT_AUTO=1` takes the direct-entry path by itself (`=demo` presses Enter at the logon and data prompts instead), `PERIGEE_RECORD=<dir>` writes every frame as a bitmap at 24 frames per second until two seconds after the tiles are up, then quits (`ffmpeg -framerate 24 -start_number 1 -i <dir>/frame-%05d.bmp -c:v libx264 -pix_fmt yuv420p demo.mp4` makes the video), `PERIGEE_EXIT_AFTER=<seconds>` quits by itself, `PERIGEE_AUTOPICK=<NORAD id or rank>` picks that satellite (or the top-ranked one) two seconds after the tiles are up, so the procedure runs unattended.
Together they let the app run unattended on a spare workspace and report what it drew.

## The boot page

A plain WarGames teletype in the phosphor green of the viewer: black page, every line typed out a character
at a time behind a block cursor, `LOGON:` prompt. Speed is `[boot] type_cps`, type size `[boot] font_size`.

1. **SYSTEM CHECK**: `control.toml`, the viewer settings and station, the data store with the age of every
   file, the engine binary (or `cargo run` as a fallback), serial ports (or the simulator), the mount frame,
   then everything the viewer said while loading (data source, propagation, catalog, ranks, categories, the
   history and live windows: the same lines Perigee itself prints).
2. **LOGON**: identification and password for space-track.org, identification prefilled from the engine's
   `.env`; with a password on file there, Enter uses it. The page runs `perigee login`, a new engine
   subcommand that logs in and proves the session with one small query (the newest ISS element set), and
   streams its lines: `SESSION ESTABLISHED` or `IDENTIFICATION NOT RECOGNIZED BY SYSTEM`. **Tab** switches field, **F2**
   bypasses the logon, **Esc** is direct entry.
3. **DATA**: **Enter** proceeds with the data on file; **F** = full catalog refresh (`perigee`: fetch
   Space-Track and SatNOGS, propagate, rank; about a minute), **R** = re-rank (`perigee rank`), **L** = log
   on again. The engine runs as a child process in `[data] dir` with the credentials in its environment; its
   output streams onto the page; **Esc** stops it. When it finishes the viewer **reloads** the new files in
   place (satellites respawned, propagation restarted, ranking panel rebuilt) and the page returns to DATA.
4. **PROPAGATING ORBITS**: waits for the viewer's propagation (Enter skips), then `ENTERING CONTROL`. The
   page lifts and the tiles come online one after another, ORBIT VIEW first, while the dividers draw in.

## The control page

Five tiles in one window. **0 ORBIT VIEW** is the viewer itself: its 3D camera is given that tile as a
viewport, so its globe, panels, buttons, search box and every key work exactly as in `perigee-viewer`,
inside the tile, whenever the tile has the focus (click it, or Ctrl+Arrows to it). The other four tiles
are drawn by this crate. A bright divider (`[tiles] divider`) runs down the middle of every gap and
around the page; the focused tile has a bright frame and a bar under its title.

| Tile | Shows | Does |
|---|---|---|
| **0 ORBIT VIEW** | the viewer: globe, tracks, view cone, ranking panel, info box, HUD | all the viewer's keys and mouse (drag orbits, wheel zooms, click picks, `/` searches, V regions, T types, L live/history, X explore, ...) |
| **1 LIVE DATA** | the picked satellite: bearing / elevation / range / range rate on the clock the dish follows, Doppler on its first listed downlink, its next pass (AOS, LOS, peak) and the solved mount path (while a pass is being flown, that pass and the path the procedure is flying, not the remainder recomputed), the dish's sky direction and pointing error, the procedure checklist (a failed or warned step's reason wraps onto extra lines instead of being cut off); a polar sky plot of the pass with the satellite (cross), the dish (circle) and the azimuth limits (red ticks) | **ARM** (A) start the procedure by hand, **AIM** (I) point at the satellite right now by the shortest move, **WARP** (W) jump the viewer's clock to the pass (simulator), **ABORT** (Escape) stop everything, **PARK**, **AUTO** toggle the procedure-on-pick |
| **2 MOTOR CONTROL** | link status, firmware id, whether the firmware knows the dish position (POSITION known / UNKNOWN: type ZERO), commanded and estimated axes in mount degrees and as sky bearing / elevation (estimated: the firmware's model, there is no position sensor), limits, step size | **AZ- AZ+ EL- EL+** jog by the step (arrow keys when focused), **STEP** cycles 0.5 / 1 / 5 / 10 (`[` `]`), **STOP** (S), **PARK** (P), **HOME** (H, mid travel), **CONNECT** / **CLOSE** the serial port, **SIM** switch to the simulator |
| **3 MOUNT** | wireframe of the Perigee gimbal (pedestal, housing, columns, drums, hub, boom, 1 m dish, counterweight) following the telemetry; rays: bright = estimated boresight, blue = commanded, green = satellite; compass ticks, azimuth limit radials, the unreachable gap if any | drag to orbit, wheel to zoom |
| **4 SERIAL CONSOLE** | every line to (`>`) and from (`<`) the mount board, in the order they happened, local notes (`#`); long lines wrap. While TRACKING, the stream of `GO` commands and their `OK GO` replies is not echoed (four of each a second would scroll everything else away); every other line still is | type a command and Enter to send it straight to the mount; Up / Down history; PageUp / PageDown or wheel to scroll; `/help` `/ports` `/open PORT [BAUD]` `/close` `/sim` `/clear`; `/here AZ [EL]` (the dish points at true bearing AZ, elevation EL, right now: ties the mount to the sky there), `/here` (show the tie), `/here clear` (forget it); `/bearing [DEG]` (show, or save and use, a measured centre bearing) |

Page keys (whatever tile is focused): **Ctrl+Arrows** or **Ctrl+H J K L** move focus, **Ctrl+Shift+Arrows**
swap tiles, **Ctrl+=** / **Ctrl+-** grow / shrink the focused tile, **Ctrl+F** zoom it to the whole page
(the globe alone, for instance), **Ctrl+T** flip the split it sits in, **Ctrl+0..4** show / hide a tile
(Ctrl+0 hides the globe: the old command page), **Tab** / **Shift+Tab** cycle focus, click focuses.
Escape aborts the sequence from any command tile; in ORBIT VIEW it is the viewer's (clears the pick).
Manual moves are refused while a sequence is ARMED or TRACKING: ABORT first.

How the two share the window: this crate's 2D camera renders first (order -1) and clears the whole
window; the viewer's 3D camera renders after it with a viewport set to the ORBIT VIEW tile each frame,
so its bloom, tonemapping and UI stay inside the tile. Bevy blits the second camera into the window with
a scissor on its viewport and never clears the surface twice, so the two never overwrite each other.
Keys: Bevy's `ButtonInput` is global, so `input.rs` erases every key from it unless ORBIT VIEW has the
focus (page keys always), and tells the viewer through `ViewerFocus` to ignore typed text; mouse presses
outside the globe are erased the same way.

## The procedure

Picking a satellite in ORBIT VIEW starts it (`[tracking] auto_arm`; AUTO in LIVE DATA or `/auto` in the
console toggles that, ARM starts it by hand). Every step is ticked off in LIVE DATA and noted in the
console as it happens:

1. **TARGET** the pick. **LINK** the serial port or simulator, telemetry alive. **EPHEMERIS** the viewer's
   propagated track covers the clock. **PASS** the **next pass** from that track (`tracking::find_next_pass`:
   coarse scan one step at a time, crossings refined by bisection; floor `[tracking] mask_deg` or the
   viewer's mask); a pass already in progress counts. **PATH** the pass sampled every `sample_seconds`
   into (time, bearing, elevation) and solved by `mount::MountGeom::solve_path` into axis commands (see
   below), flagged when it is clipped or faster than the slew limits. The checks come one per
   `step_seconds` so the list can be read; a failed check ends the procedure with the reason.
2. **SLEW**: `GO az el` to the AOS point, then wait until the mount reports it is there (`T4` settled estimate
   within `on_point_deg` of the firmware's target, that target within `on_point_deg` of the GO, known,
   not moving, not clipped; a legacy `T` line never counts). While it waits, LIVE DATA says what for:
   `SLEW: waiting: settling, 3.2 deg to go`, `moving`, `target clipped at the azimuth limit`, ... If AOS
   comes first the procedure goes on, and ARMED says `mount NOT confirmed on point` with the reason.
3. **ARMED**: countdown to AOS.
4. **TRACKING**: `GO az el` at `command_hz`, aiming `lead_seconds` ahead (servo lag). LIVE DATA and the
   MOUNT title show the *model* error: the firmware's estimated boresight against the satellite. It shows
   lag, clipping or a wrong plan, but it cannot see calibration errors (there is no position sensor), so it
   is never shown as a lock. On the simulated mount it read 0.04 deg while the real error was 7.9 deg uncalibrated.
5. **PARK** at LOS (if `park_after`), then DONE once the mount stops. A new pick restarts the procedure for
   the new satellite; clearing the pick (Escape on the globe) aborts and parks; ABORT (Escape in any
   command tile) stops the mount. Losing the link aborts.

### Simulating it

With the built-in mount simulator (no serial port) the dish follows the **viewer's clock** rather than the
wall clock (`sim_clock`): HISTORY mode's speed and pause move the simulated mount too, and its axes slew
at their real rate limits in clock seconds. When the procedure is armed for a pass that is still far off
it **warps** the viewer to `warp_lead_s` before AOS at `warp_speed` (`sim_warp`; WARP / W does it by
hand): the globe jumps to the pass, the wireframe slews to the AOS point, locks on as the satellite
rises, tracks it across the sky, parks at LOS, and the viewer returns to LIVE. Everything the real mount
would be sent goes through the same link and console. A real serial link never warps and always follows
real time.

The simulator behaves like the firmware where it matters, so a pass that works on SIM works the same
way on the board: it speaks `T4` with the same settle estimate (the command lagged at 70 % of the
servos' rated speed, so "on point" takes as long as on the board), accepts finite numbers only,
refuses `ZERO` while the servos are powered (`OFF` first, which leaves them limp), and answers `RAW`
with an error because it has no pulse map. One difference is on purpose: it starts at park with the
position known, so the SIM button can run a pass straight away, where a real board after a power cycle
starts `POS UNKNOWN` and waits for `ZERO`.

### Mount frame and the path solver

The mount uses two axis angles: azimuth 0..400 (inside the Stingray-4's 450 of travel; the cable
loop limits it) and elevation -2..91 on the Stingray-9. Two measured numbers tie them to the sky, both
kept in `calibration.toml`: the centre bearing, the true bearing the dish faces at mount azimuth 225,
and the elevation correction, how much higher the mount's elevation reads than the true one (0 until
measured). `/here AZ EL` sets both at once: point the dish at something whose direction you know (a far
tower, the sun with the dish face covered, a geostationary satellite) and type the true bearing and
elevation it points at; the numbers come from where the firmware says the dish is. A second `/here` at
least 30 degrees round in azimuth also measures the azimuth motor's scale, and says by what factor
`AZ_CAL.us_per_deg` should change if it is off. `/here clear` forgets the tie; `/bearing DEG` sets the
centre bearing alone. Everything else follows:

* `el_m = el + el_correction`: the host adds the correction to every elevation it sends and takes it
  off every elevation it shows; the window `-2..91` stays in mount degrees.
* `bearing = az_zero + az_m`, `az_zero = center - travel/2`. With a 400 deg window every bearing is
  reachable and a 40 deg band is reachable twice, 360 apart. The window ends at `az_zero` and
  `az_zero + 400`: the seam.
* **No flip-over** (removed 2026-10-01 for simplicity): elevation never goes more than a degree past
  vertical, so each sky direction has one pose. A pass near the zenith swings the azimuth quickly at
  the top and may lag for a few seconds there, which costs little: near the zenith an azimuth error
  barely moves the beam.

`solve_path` unwraps the pass's bearings into one continuous azimuth run and picks the whole-turn
offset that keeps it inside the window, preferring the shortest slew from where the dish is, then the
most margin. A pass that runs across the seam cannot fit at one wrap, so one mid-pass **unwind** is
scheduled (the dish runs the long way round, about 16 s off the satellite at 20 deg/s) and logged;
in a count of 908 real passes this was 3-6 % of them. A path that cannot be planned at all is marked
**CLIPPED** and held at the limit. Unit tests in `mount.rs` cover a southern pass, a zenith pass, the
seam, and "no plan ever commands an azimuth outside the window".

## Serial link and firmware

`serial.rs` opens the port with termios directly (raw, 8N1, `[serial] baud`), reads it on a worker
thread and delivers whole lines. `port = "auto"` takes the first of `/dev/rfcomm*`, `/dev/ttyACM*`,
`/dev/ttyUSB*` (Bluetooth: pair the HC-05 and `rfcomm bind 0 <addr>`). With no port and `simulate =
true` the built-in **simulator** answers instead (two rate-limited axes, same protocol and `T4` telemetry), so the whole
chain runs with nothing plugged in. On connect the PC sends `ID`, `RATE`, `TEL`.

Protocol (ASCII lines, mount-frame degrees):

```
PC -> mount: PING  ID  ?  GO az el  AZ deg  EL deg  STOP  PARK  RATE az_dps el_dps  TEL hz  RAW AZ|EL us
             ZERO az el  CAL  OFF
mount -> PC: READY ...  POS ...  PONG  ID ...  OK ...  ERR ...  T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags
```

`T4` is what this crate parses (`mount::Pose::parse`), from the STM32 firmware (fw4) and the simulator
alike: the target after the window clamp, the ramped command (the pulse on the wire), the settle
estimate, and the flags K known, M moving, A / E target clipped at the azimuth / elevation limit. The
old sketch's `T az_cmd el_cmd az_est el_est moving enc` line is still read for display, but it can
never confirm the dish is on point. `ZERO` is accepted only while the servos are limp (`OFF` first),
and `RAW` is an ordinary ramped move to the angle that pulse width means.

`firmware/perigee_mount_stm32/` implements it on a **Nucleo-F401RE**: AZ on PC7 (TIM3 CH2, header D9),
EL on PB6 (TIM4 CH1, header D10), USART2 to the ST-LINK. Bare-metal
Rust, registers written directly. It ramps each axis toward its target at the RATE limit so the servos
never see a step, and it enforces the azimuth and elevation limits itself. Pin table and diagram:
`firmware/perigee_mount_stm32/docs/wiring.md`.

Every position in the telemetry is the firmware's **estimate**, not a measurement: the Stingrays
have a 3-pin connector and an internal pot, so there is no feedback wire, and no encoder is fitted on
either axis. Both axes run open loop; the estimate is the rate-limited command.

### Cable wrap

The azimuth service loop is 1.25 turns with no slip ring, so azimuth is limited to **0..400 deg** of
the Stingray-4's 450. Azimuth is absolute and continuous, with no modular arithmetic anywhere, so a
move always traverses the interval between two positions and can never take the short way through the
wrap. Two layers, independently:

- **Here.** `MountGeom::nearest_allowed_az` picks, of the whole-turn equivalents inside the window,
  the one nearest where the axis is. At 399 asked for what 401 would reach, 401 is outside the
  window, so the mount unwinds 358 deg to 41 instead of tangling. `MountGeom::solve_path` does the
  same thing for a whole predicted pass at ARM: one wrap for the entire track, chosen for the
  shortest slew to the AOS point, with a mid-pass unwind scheduled (and logged) only when the pass
  runs across the seam (3-6 % of passes). The simulator enforces the same limits and counts any violation.
- **In the firmware.** `limits.rs` clamps every pulse into the window regardless of what arrives on
  the wire. A bug here costs a pass; it cannot reach the cable.

`az_limit_lo_deg` / `az_limit_hi_deg` in `control.toml` and `AZ_CAL` in the firmware's `limits.rs`
are the two places these numbers live, and they must be edited together.

The old `firmware/perigee_mount/perigee_mount.ino` is the superseded Uno sketch. It is not built or
tested, its A0/A1 "feedback" reads do not correspond to any real wire, and it does not know about the
400 deg limit. Kept for reference only.

## Viewer changes

perigee-viewer gained `pub fn build_app(config_path, orbit_file) -> App` (main() is now one line
calling it), public access to the resources this crate reads (`Selected`, `Catalog`, `Orbits`, `Sim`,
`Mode`, `Ranks`, `UiFont`, `DataSource`, `PropStatus`, the look-angle helpers), and for the single window:

* `load_data()`: the file loading that used to be inline in `build_app`, so it can run again;
  `ReloadData { elsets }` is an event that re-reads the files (respawning the satellite markers,
  restarting the propagator, rebuilding the ranking panel) or just the ranking and categories.
* `BootLog`: every line the viewer printed while loading, for the boot page.
* `ViewerFocus`: when false, the viewer ignores typed keys (search box and its key bindings).
* `viewport_cursor()`: picking, the info-box drag and scroll zoom use the cursor relative to the camera's
  viewport and ignore it outside; the camera's aspect and the info-box clamp use the viewport size.
  With no viewport (the viewer on its own) all of this is exactly the old behaviour.

The desktop viewer binary behaves as before. The engine (Perigee) gained the `perigee login`
subcommand (`spacetrack::login_check`) used by the boot page; nothing else in it changed.

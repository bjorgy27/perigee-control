# Perigee Control

The ground-station control base for Perigee, in one window: a **boot page** (checks, Space-Track login,
an engine run if you want fresh data), then the **orbit viewer** as one tile beside the four command
tiles, laid out like Hyprland but inside the window, with bright dividers between them. Pick the
satellite on the globe, ARM in LIVE DATA, and watch the mount take the pass.

```
perigee-control/            this crate: `cargo run --release` from here
  control.toml              command page, mount, serial and tracking settings
  src/main.rs               builds the viewer's App (perigee_viewer::build_app), adds ControlPlugin and BootPlugin
  src/boot.rs               the boot page: checks, Space-Track login, engine run, load, then the tiles
  src/command.rs            the control page: the two cameras, tiles, keys, buttons, text, overlays, dividers
  src/tiles.rs              dwindle layout tree (focus / swap / resize / zoom / hide) incl. the ORBIT VIEW tile
  src/input.rs              input routing: keys reach the viewer only while ORBIT VIEW has the focus
  src/tracking.rs           next-pass finder, pass sampling, the ARM -> TRACK -> PARK sequence
  src/mount.rs              mount frame, path solver (flip-over + wrap), telemetry, wireframe
  src/serial.rs             termios serial link on a worker thread + the mount simulator
  src/console.rs            scrollback, input line, history
  firmware/perigee_mount/   Arduino Uno sketch speaking the same protocol
../perigee-viewer           the viewer, unchanged, used as a library
../Perigee                  the engine's outputs (SORTED_SATS.json, SATELLITE_RANKS.json, ...)
```

## Running

```
cd ~/projects/perigee-control
cargo run --release
```

One window opens, `PERIGEE // control`, on the boot page. Settings: `control.toml` here, and the viewer's
own `viewer.toml` (path in `[viewer] config`). `[data] dir` says where Perigee's outputs are (the engine
writes into the folder it is run from: `../Perigee` when run by hand there, `../Perigee/src` under the
systemd timer). `[perigee] bin` / `dir` say how to run the engine; `[boot] enabled = false` skips the
boot page entirely.

## The boot page

Glyph rain behind a log that types itself out, every line resolving from noise into text:

1. **CHECK**: `control.toml`, the viewer settings and station, the data folder with the age of every file,
   the engine binary (or `cargo run` as a fallback), serial ports (or the simulator), the mount frame, then
   everything the viewer said while loading (data source, propagation, catalog, ranks, categories, the
   history and live windows: the same lines Perigee itself prints).
2. **LOGIN**: identity and password for space-track.org, identity prefilled from the engine's `.env`; with
   a password saved there, Enter uses it. The page runs `perigee login`, a new engine subcommand that logs
   in and proves the session with one small query (the newest ISS element set), and streams its lines:
   `ACCESS GRANTED` or `ACCESS DENIED`. **Tab** switches field, **F2** skips the login, **Esc** goes
   straight to the tiles.
3. **MENU**: **Enter** = FULL RUN (`perigee`: fetch Space-Track and SatNOGS, propagate, rank; about a
   minute), **R** = RANK ONLY (`perigee rank`), **S** = SKIP (data on disk as it is), **L** = back to login.
   The engine runs as a child process in `[data] dir` with the credentials in its environment; its output
   streams onto the page; **Esc** stops it. When it finishes the viewer **reloads** the new files in place
   (satellites respawned, propagation restarted, ranking panel rebuilt).
4. **LOAD**: the viewer's propagation progress (`PROPAGATING n/N`), then `ENTERING CONTROL` and the tiles.

## The control page

Five tiles in one window. **0 ORBIT VIEW** is the viewer itself: its 3D camera is given that tile as a
viewport, so its globe, panels, buttons, search box and every key work exactly as in `perigee-viewer`,
inside the tile, whenever the tile has the focus (click it, or Ctrl+Arrows to it). The other four tiles
are drawn by this crate. A bright divider (`[tiles] divider`) runs down the middle of every gap and
around the page; the focused tile has a bright frame and a bar under its title.

| Tile | Shows | Does |
|---|---|---|
| **0 ORBIT VIEW** | the viewer: globe, tracks, view cone, ranking panel, info box, HUD | all the viewer's keys and mouse (drag orbits, wheel zooms, click picks, `/` searches, V regions, T types, L live/history, X explore, ...) |
| **1 LIVE DATA** | the picked satellite: bearing / elevation / range / range rate now, Doppler on its first listed downlink, its next pass (AOS, LOS, peak) and the solved mount path, the sequence state, the dish's current sky direction; a polar sky plot of the pass with the satellite (cross), the dish (circle) and the azimuth limits (red ticks) | **ARM** (A) solve the next pass and start the sequence, **AIM** (I) point at the satellite right now by the shortest move, **ABORT** (Escape) stop everything, **PARK** |
| **2 MOTOR CONTROL** | link status, firmware id, commanded and measured axes in mount degrees and as sky bearing / elevation, encoder, limits, step size | **AZ- AZ+ EL- EL+** jog by the step (arrow keys when focused), **STEP** cycles 0.5 / 1 / 5 / 10 (`[` `]`), **STOP** (S), **PARK** (P), **HOME** (H, mid travel), **CONNECT** / **CLOSE** the serial port, **SIM** switch to the simulator |
| **3 MOUNT** | wireframe of the Perigee gimbal (pedestal, housing, columns, drums, hub, boom, 1 m dish, counterweight) following the telemetry; rays: bright = measured boresight, blue = commanded, green = satellite; compass ticks, azimuth limit radials, the unreachable gap if any | drag to orbit, wheel to zoom |
| **4 SERIAL CONSOLE** | every line to (`>`) and from (`<`) the Arduino, local notes (`#`) | type a command and Enter to send it straight to the mount; Up / Down history; PageUp / PageDown or wheel to scroll; `/help` `/ports` `/open PORT [BAUD]` `/close` `/sim` `/clear` |

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

## The tracking sequence

1. Pick a satellite in the viewer. LIVE DATA immediately shows where it is and finds its **next pass**
   from the viewer's own propagated track (`tracking::find_next_pass`: coarse scan one step at a time,
   crossings refined by bisection; the elevation floor is `[tracking] mask_deg`, or the viewer's mask).
2. The pass is sampled every `sample_seconds` into (time, bearing, elevation) and handed to the
   **path solver** (`mount::MountGeom::solve_path`), which turns it into axis commands, see below.
3. **ARM**: `preposition_min` minutes before AOS the dish is sent to the AOS point (`GO az el`).
4. At AOS the tracker sends `GO az el` at `command_hz`, aiming `lead_seconds` ahead of real time
   (servo lag). The dish follows the real clock even if the viewer is in History mode.
5. At LOS: `PARK` (if `park_after`), then back to idle. Changing the pick while armed does not retarget
   the dish; ABORT then ARM the new one. Losing the link aborts.

### Mount frame and the path solver

The mount reports two axis angles: azimuth 0..450 on the Stingray-4, elevation -5..185 on the
Stingray-9. One calibration number ties them to the sky, `[mount] az_center_bearing_deg`: the true
bearing the dish faces at mid travel (225). Everything else follows:

* `bearing = az_zero + az_m`, `az_zero = center - travel/2`. With 450 deg of travel every bearing is
  reachable and a 90 deg band (through north with the default 217.5) is reachable twice, 360 apart.
  The axis limits sit at bearings `az_zero` and `az_zero + 450`; a pass may not cross them.
* Elevation past 90 is the **flip-over**: the same sky direction is `(bearing + 180, 180 - el)`.
  A pass through the zenith would make a normal az/el mount spin 180 deg in seconds; flipped it is
  one slow elevation sweep from -5 towards 185, which is why the Stingray-9 has 200 deg.

`solve_path` builds the axis track greedily from each of the two starting representations: at every
sample it takes whichever of normal / flipped is nearest the previous pose (azimuth unwrapped by whole
turns, elevation free to run past 90), then picks the whole-turn offset that keeps the run inside
0..450 with the most margin. Of the two candidates it keeps the one that is not clipped, then the one
with the gentler peak azimuth rate. A path that cannot fit (only possible on a mount with less travel
or no flip-over) is marked **CLIPPED** and held at the limit for that part. Unit tests in `mount.rs`
cover a normal southern pass, a zenith crossing, and the seam on a plain 360 deg mount.

## Serial link and firmware

`serial.rs` opens the port with termios directly (raw, 8N1, `[serial] baud`), reads it on a worker
thread and delivers whole lines. `port = "auto"` takes the first of `/dev/rfcomm*`, `/dev/ttyACM*`,
`/dev/ttyUSB*` (Bluetooth: pair the HC-05 and `rfcomm bind 0 <addr>`). With no port and `simulate =
true` the built-in **simulator** answers instead (two rate-limited axes, same protocol), so the whole
chain runs with nothing plugged in. On connect the PC sends `ID`, `RATE`, `TEL`.

Protocol (ASCII lines, mount-frame degrees):

```
PC -> mount: PING  ID  ?  GO az el  AZ deg  EL deg  STOP  PARK  RATE az_dps el_dps  TEL hz  RAW AZ|EL us
mount -> PC: READY ...  PONG  ID ...  OK ...  ERR ...  T az_cmd el_cmd az_fb el_fb moving enc
```

`firmware/perigee_mount/perigee_mount.ino` implements it on an Uno: Stingray signals on D9 / D10
(Servo library, 500..2500 us over the gearbox's travel), feedback wires on A0 / A1, AS5600 on I2C.
It ramps each axis toward its target at the RATE limit so the servos never see a step. Calibration
constants (pulse ends, feedback ADC ends, elevation offset) are at the top of the sketch; use `RAW` and
the telemetry to measure them. The sketch is not built or tested here: it needs the hardware.

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

The desktop binary and the TV build behave as before. The engine (Perigee) gained the `perigee login`
subcommand (`spacetrack::login_check`) used by the boot page; nothing else in it changed.

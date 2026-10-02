# perigee_mount_stm32

Mount firmware for a Nucleo-F401RE driving the two Stingray servo gearboxes. Same command set as
`../perigee_mount/perigee_mount.ino`, but telemetry is the `T4` line (`docs/wiring.md` §Protocol), which
perigee-control parses: target, command and settle estimate separately, with the K / M / A / E flags
(the sketch's old `T` line is still read, but it can never confirm the dish is on point). The Nucleo's ST-LINK shows
up as `/dev/ttyACM0` and `port = "auto"` picks it up. Bare-metal Rust, register level, same toolchain
as `~/projects/STM32Test`.

## Flash

```sh
cargo run --release        # builds, flashes through the ST-LINK with probe-rs, and runs
```

Then talk to it by hand if you like: `picocom -b 115200 --omap crlf /dev/ttyACM0`, type `ID`, `?`.

## Wiring

**Full diagram and pin table: [`docs/wiring.md`](docs/wiring.md) and
[`docs/wiring.svg`](docs/wiring.svg).** The short version:

| Signal | Nucleo pin | Header | Notes |
|---|---|---|---|
| AZ servo signal (Stingray-4) | PC7, TIM3 CH2 | D9 | 3.3 V logic |
| EL servo signal (Stingray-9) | PB6, TIM4 CH1 | D10 | 3.3 V logic |
| PC link | PA2 / PA3, USART2 | USB (ST-LINK) | D0/D1 are not connected by default |
| nothing | PA0 / PA1 | A0 / A1 | **leave bare** |
| nothing | PB8 / PB9 | D15 / D14 | free, no encoder is fitted |

- **Servo power comes from its own 7.4 V 10 A supply, never the Nucleo.** Each servo's red and black
  go straight to the rail, not daisy-chained. Tie the grounds together or nothing works.
- **PA0 / PA1 have nothing on them.** These gearboxes have a 3-pin TJC8 connector (signal, V+, GND)
  and the pot is internal: there is no feedback wire. The old `FB` command and the ADC that backed it
  read floating pins and returned noise; both are gone. There is no encoder on either axis either:
  both run open loop, and the pulse width is the position.
- 3.3 V signal is in spec (pulse amplitude 3-5 V). If a Stingray ignores it, a 74AHCT125 buffer on
  5 V lifts it.
- LD2 shows the state: a short blink every second while limp, fast blinking while slewing, steady
  while holding.
- For Bluetooth instead of USB, the HC-05 needs a different UART (D0/D1 are cut from USART2 on the
  Nucleo); not wired up yet.

## Limits

Azimuth **0 to 400 deg** of the Stingray-4's 450 (the cable service loop is 1.25 turns, no slip
ring), with mount 0 sitting 8 deg off the low stop so the window clears both stops under either of
goBILDA's two published slopes; elevation **-2 to 91 deg** of the Stingray-9's 200 (never more than a degree past vertical; there is no flip-over), with margin off both stops. Azimuth is
absolute and continuous, with no modular arithmetic anywhere, so a move always traverses the interval
between two positions and can never take the short way round through the wrap.

`src/limits.rs` holds the whole safety layer and every pulse goes through it. The host
(`perigee-control`) plans which wrap to track a pass at; the firmware enforces the window regardless,
so a planner bug costs a pass and not the cable.

## Host test

`src/limits.rs` has no hardware in it, so it compiles twice: as a module here, and as its own crate
root on a PC. No board needed:

```sh
./test_host.sh        # 18 tests: windows, clamping, round-trips, the ramp, the settle model,
                      # the telemetry line, the saved position
```

## Behaviour

- **Position survives a reset.** The last position lives in a `.uninit` SRAM region the startup code
  does not clear, behind a magic word and a checksum, and is rewritten every 20 ms frame. A watchdog
  reset, the reset button or a `probe-rs run` restores it and holds the same pulses straight away.
  It stores the pulse widths on the wire (the servos' real position), so reflashing with new calibration constants holds the same pulse: the dish does not jump. A **power cycle does not** survive: the board stays limp, says `POS UNKNOWN` and
  refuses every move until `ZERO az el` arrives.
- **Silent start when the position is unknown.** No pulses, and every move refused, until `ZERO az el`
  says where the dish is, so a cold boot never slams the dish anywhere. `OFF` returns to limp.
- **`ZERO` only while limp.** With the pulses on, the pulse is the position, so re-declaring it could
  only lie or jump the dish: `ZERO` answers `ERR ... send OFF first` until the servos are limp.
- **`RAW` is an ordinary move.** `RAW AZ|EL us` goes to the angle that pulse width means, ramped at the
  `RATE` limit and clamped into the window like `GO`, and the telemetry tells the truth about it.
- **Nothing is dropped silently.** Incoming bytes land in a 1024-byte ring by DMA while telemetry is
  being sent. A burst bigger than that (a long pasted script) is reported as `ERR receive overrun:
  input lost, resend`, and the partial line is thrown away rather than run half-read.
- **Watchdog.** One second without the main loop and the board resets, limp, and says `NOTE reset by
  watchdog` after `READY`.
- Pulses are set in 0.3125 microsecond steps (about 0.07 degrees on azimuth).

## Calibration

Full procedure: [`docs/calibration.md`](docs/calibration.md). The constants are `AZ_CAL` and `EL_CAL`
in `src/limits.rs`: `pulse_at_zero` and `us_per_deg` (the pulse that puts each gearbox at its
mechanical zero, and the slope), and `offset` (mount angle = gearbox angle - offset). There is
nothing to read back: both axes are open loop, so calibration is done by commanding `RAW` pulses and
measuring where the axis physically ends up. The sky calibration (`az_center_bearing_deg`) lives on the
PC, saved into perigee-control's `calibration.toml` with `/here` or `/bearing` in the console.

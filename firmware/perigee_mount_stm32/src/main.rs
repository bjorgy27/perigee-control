/*
  PERIGEE mount firmware, STM32 edition  (Nucleo-F401RE on the rotating head)
  ------------------------------------------------------------------------------
  Drives the two goBILDA Stingray servo gearboxes and speaks the line protocol perigee-control
  expects: plug the Nucleo in, it appears as /dev/ttyACM0, and port = "auto" finds it.

    PC -> mount                          mount -> PC
    PING                                 PONG
    ID                                   ID PERIGEE-MOUNT fw4-stm32 PROTO T4 AZ 0-400 EL -2-91
    ?                                    one telemetry line
    GO az el                             OK GO az el        (ERR ... if out of window: clamped)
    AZ deg  /  EL deg                    OK AZ deg / OK EL deg
    STOP                                 OK STOP            (hold where the axes are now)
    PARK                                 OK PARK az el
    RATE az_dps el_dps                   OK RATE ...        (slew speed limits)
    TEL hz                               OK TEL hz          (telemetry rate, 0 = off)
    RAW AZ us  /  RAW EL us              OK RAW ...         (move to the angle that pulse width means: ramped, window-clamped)
    ZERO az el                           OK ZERO az el      (declare where the axes really are; only while limp)
    CAL                                  OK CAL ...         (windows and pulse endpoints)        new
    OFF                                  OK OFF             (stop the pulses, servos go limp)
    telemetry:  T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags
    on boot:    READY PERIGEE-MOUNT fw4-stm32 PROTO T4  then  POS <az> <el> | POS UNKNOWN
    on a burst over the 1 KB receive ring:  ERR receive overrun: input lost, resend

  TELEMETRY, and why it has a version tag (`limits::TEL_TAG`). The line carries three different
  position claims because they are three different things and the host has to be able to tell them
  apart: `tgt` is what was asked for after the window clamp, `cmd` is the slew-rate-ramped command
  that IS the pulse on the wire, and `set` is `cmd` lagged by a conservative servo speed model, so
  it reaches `tgt` only once a gearbox running at 70% of its rated speed could have got there.
  `fw3` sent the ramped command in fields 1-2 *and* fields 3-4 and never transmitted the target at
  all, so a host asking "did the dish reach what I asked for?" was comparing a number against
  itself and could never answer no. `flags` carries K known / M moving / A,E limit clipped. Fields
  7-8 (`NA -1`) are where an encoder reading would sit; no encoder is fitted, so they never change,
  and they stay so the line shape is unchanged. A host that sees any tag other than `T4` must refuse
  the line.

  THIS IS THE AUTHORITATIVE SAFETY LAYER. The host plans which 0-360 wrap to track a pass at; the
  firmware's job is to make a host bug harmless. Azimuth is an absolute, continuous, unwrapped angle
  with no modular arithmetic anywhere, clamped to [0, 400] deg inside the Stingray-4's 450 deg of
  travel, so a command can never be satisfied "the short way round" through the 1.25-turn cable loop.
  All of that lives in src/limits.rs, which also compiles on a PC: run ./test_host.sh.

  Position tracking. These gearboxes are positional servos with an internal pot and a 3-pin
  connector, so the pulse width IS the absolute position and there is no feedback wire to read
  (reports/stingray-power.md). The estimate is therefore the rate-limited command plus a servo lag
  model, which is as honest as an open-loop axis can be. No encoder is fitted on either axis, so
  nothing here can detect a stalled or unpowered servo. The last
  position is kept in a RAM region the startup code does not clear, so a watchdog or button reset
  picks the pulses back up where they were instead of slewing blind. A power cycle loses it: the
  board then stays limp, reports POS UNKNOWN and refuses every move until the host sends ZERO.

  There are no ADC reads. PA0/PA1 used to be read as "feedback wires"; those wires do not exist on
  these gearboxes and the pins have nothing attached, so the reads returned noise. Removed.

  CALIBRATION (constants in src/limits.rs, procedure in docs/calibration.md):
    * pulse_at_zero / us_per_deg: pulse width at the gearbox's mechanical zero, and its slope.
    * offset: mount angle = gearbox angle - offset, and the margin off the low stop.
    * The sky calibration (az_center_bearing_deg) lives on the PC, in calibration.toml (/bearing), not here.
*/
#![no_std]
#![no_main]

mod hw;
mod limits;

use core::fmt::Write;
use core::mem::MaybeUninit;
use cortex_m_rt::entry;
use limits::{clamp, fabs, ramp, servo_model_dps, AxisCal, Persist, Telemetry, AZ, CAL, EL, TEL_TAG};
use panic_halt as _;

const BAUD: u32 = 115_200;
const FW: &str = "PERIGEE-MOUNT fw4-stm32";

/// Park pose, mount frame. Mid azimuth travel inside the [0, 400] window.
const PARK: [f32; 2] = [200.0, 45.0];
/// RAW accepts this pulse range, then the angle it means is clamped into the legal window like any GO
const RAW_US: (f32, f32) = (500.0, 2500.0);
/// Slew update period: one servo frame
const TICK_US: u32 = 20_000;
/// A command this far outside the window is a protocol error worth reporting, not float noise
const WINDOW_TOL: f32 = 0.02;

/// The saved position, in `.uninit` so cortex-m-rt's startup leaves it alone across a reset.
/// Single-threaded: the main loop is the only thing that touches it.
#[unsafe(link_section = ".uninit.PERIGEE_POS")]
static mut SAVED: MaybeUninit<Persist> = MaybeUninit::uninit();

macro_rules! say {
    ($($arg:tt)*) => {{ let _ = write!(hw::Tx, $($arg)*); let _ = hw::Tx.write_str("\r\n"); }};
}

fn cal(axis: usize) -> AxisCal { CAL[axis] }

struct Mount {
    cmd: [f32; 2],     // the slew-rate-limited command: exactly what the pulse on the wire says
    tgt: [f32; 2],     // where it should end up
    settled: [f32; 2], // cmd lagged by the servo speed model: the earliest the axis can be there
    rate: [f32; 2],    // deg/s
    pulse: [f32; 2],   // pulse on the wire now, 0 = off
    driving: bool,     // pulses are going out
    known: bool,       // the position estimate is anchored to something real
    clip: [bool; 2],   // the standing target was clamped at this axis's limit
    tel_hz: f32,
}

impl Mount {
    /// Start from the saved position when there is one, otherwise park but flagged unknown
    fn new(saved: Option<(f32, f32)>) -> Self {
        let (cmd, known) = match saved {
            Some((az, el)) => ([cal(AZ).clamp_mount(az), cal(EL).clamp_mount(el)], true),
            None => (PARK, false),
        };
        Mount { cmd, tgt: cmd, settled: cmd, rate: [20.0, 15.0], pulse: [0.0; 2],
                driving: false, known, clip: [false; 2], tel_hz: 0.0 }
    }

    /// The only way a pulse reaches a servo. `limits::pulse_of` clamps into the mount window and then
    /// into the servo's pulse range, so no path through this firmware can drive past a soft limit.
    fn drive(&mut self, axis: usize, mount_deg: f32) {
        let us = cal(axis).pulse_of(mount_deg);
        self.pulse[axis] = us;
        hw::servo_us(axis, us);
    }

    /// Remember where we are, so a watchdog or button reset does not slew blind. Stored as the pulse
    /// widths (the servo's real position), so a reflash with new calibration constants holds the same
    /// pulse instead of jumping. The pulse for `cmd` is what `drive` sends (and what it would send on the
    /// next move while the servos are limp).
    fn save(&self) {
        let mut p = Persist::blank();
        let us = |a: usize| cal(a).pulse_of(self.cmd[a]);
        if self.known { p.store(us(AZ), us(EL)); }
        unsafe { core::ptr::write_volatile((&raw mut SAVED).cast::<Persist>(), p) };
    }

    /// Called before any motion: start the pulses if they are off, from a position that does not make
    /// the servo jump: the restored, declared (ZERO) or last-held command.
    fn take_wire(&mut self) {
        if !self.driving {
            for a in [AZ, EL] {
                self.cmd[a] = cal(a).clamp_mount(self.cmd[a]);
                let d = self.cmd[a];
                self.drive(a, d);
            }
            // The pulses were off, so the servo has had as long as it likes to be where the last
            // pulse left it: the settle model starts caught up rather than pretending to lag.
            self.settled = self.cmd;
            self.driving = true;
            self.save();
        }
    }

    /// Set both targets. Returns true when the request was outside the window and got clamped.
    ///
    /// The per-axis `clip` flags it leaves behind are a **standing** condition, not an event: while
    /// one is set the axis is holding at its limit and can never reach what the host asked for, so
    /// the host's on-point test must refuse to pass. They clear on the next request that fits.
    fn set_target(&mut self, az: f32, el: f32) -> bool {
        self.take_wire();
        self.clip[AZ] = !cal(AZ).accepts(az, WINDOW_TOL);
        self.clip[EL] = !cal(EL).accepts(el, WINDOW_TOL);
        self.tgt = [cal(AZ).clamp_mount(az), cal(EL).clamp_mount(el)];
        self.clip[AZ] || self.clip[EL]
    }

    /// Still in motion: either the command is still ramping toward the target, or the command has
    /// arrived and the modelled servo has not caught up with it yet.
    fn moving(&self) -> bool {
        if !self.driving { return false; }
        [AZ, EL].iter().any(|&a| fabs(self.cmd[a] - self.tgt[a]) > 0.01 || fabs(self.settled[a] - self.cmd[a]) > 0.01)
    }

    /// Move each command toward its target by at most rate * dt, then update the pulse, then move
    /// the settle model toward the command at the servo's modelled speed.
    ///
    /// Two ramps, because they model two different things. The first is the slew limiter: it keeps
    /// `cmd` a command the servo can follow instead of a step it lags by seconds. The second is the
    /// gearbox itself, which takes time to travel to a pulse it has already been given. Only the
    /// second one can answer "could the dish be there yet", which is what the host needs.
    fn tick(&mut self, dt: f32) {
        if !self.driving { return; }
        for a in [AZ, EL] {
            self.cmd[a] = cal(a).clamp_mount(ramp(self.cmd[a], self.tgt[a], self.rate[a], dt));
            let d = self.cmd[a];
            self.drive(a, d);
            self.settled[a] = ramp(self.settled[a], self.cmd[a], servo_model_dps(a), dt);
        }
        // Save every frame (every 20 ms): it is a 16-byte write to RAM, so it costs nothing, and a
        // reset in the middle of a slew then restores where the dish is, not where the slew began
        self.save();
    }

    /// Everything known about where the axes are, with each claim kept separate
    fn state(&self) -> Telemetry {
        Telemetry {
            tgt: self.tgt, cmd: self.cmd, settled: self.settled,
            known: self.known, moving: self.moving(), clip: self.clip,
        }
    }

    fn telemetry(&self) {
        say!("{}", self.state().line().as_str());
    }

    fn handle(&mut self, line: &str) {
        let mut it = line.split_ascii_whitespace();
        let Some(verb) = it.next() else { return };
        let (a1, a2) = (it.next(), it.next());
        // Only ordinary numbers: Rust's parser also accepts "nan" and "inf", and not-a-number slips
        // through every comparison in limits.rs (it ends up as a 0 pulse: a limp servo)
        let num = |s: Option<&str>| s.and_then(|s| s.parse::<f32>().ok()).filter(|v| v.is_finite());
        let is = |w: &str| verb.eq_ignore_ascii_case(w);
        let (az_c, el_c) = (cal(AZ), cal(EL));

        // A positional servo jumps to its first pulse at full speed, so nothing may switch the
        // pulses on until the position is known: declared with ZERO, or restored after a reset.
        let moves = is("GO") || is("AZ") || is("EL") || is("PARK") || is("RAW");
        if moves && !self.known {
            say!("ERR position unknown: send ZERO az el first");
            return;
        }

        if is("PING") { say!("PONG"); }
        else if is("ID") {
            // PROTO is how a host finds out at connect, rather than at the first telemetry line,
            // that it is talking to a firmware whose line shape it does not understand.
            say!("ID {FW} PROTO {TEL_TAG} AZ {:.0}-{:.0} EL {:.0}-{:.0}", az_c.mount_lo(), az_c.mount_hi(), el_c.mount_lo(), el_c.mount_hi());
        }
        else if is("?") { self.telemetry(); }
        else if is("GO") {
            match (num(a1), num(a2)) {
                (Some(az), Some(el)) => {
                    if self.set_target(az, el) { say!("ERR out of window, clamped to {:.2} {:.2}", self.tgt[AZ], self.tgt[EL]); }
                    else { say!("OK GO {:.2} {:.2}", self.tgt[AZ], self.tgt[EL]); }
                }
                _ => say!("ERR GO needs az el"),
            }
        }
        else if is("AZ") || is("EL") {
            match num(a1) {
                Some(v) => {
                    let (az, el) = if is("AZ") { (v, self.tgt[EL]) } else { (self.tgt[AZ], v) };
                    let out = self.set_target(az, el);
                    let (name, got) = if is("AZ") { ("AZ", self.tgt[AZ]) } else { ("EL", self.tgt[EL]) };
                    if out { say!("ERR {} out of window, clamped to {:.2}", name, got); }
                    else { say!("OK {} {:.2}", name, got); }
                }
                None => say!("ERR {} needs deg", verb),
            }
        }
        else if is("STOP") {
            self.tgt = self.cmd;
            self.save();
            say!("OK STOP");
        }
        else if is("PARK") { self.set_target(PARK[AZ], PARK[EL]); say!("OK PARK {:.1} {:.1}", PARK[AZ], PARK[EL]); }
        else if is("RATE") {
            match (num(a1), num(a2)) {
                (Some(a), Some(e)) => { self.rate = [clamp(a, 0.1, 90.0), clamp(e, 0.1, 90.0)]; say!("OK RATE {:.1} {:.1}", self.rate[AZ], self.rate[EL]); }
                _ => say!("ERR RATE needs az_dps el_dps"),
            }
        }
        else if is("TEL") {
            match num(a1) {
                Some(h) => { self.tel_hz = clamp(h, 0.0, 20.0); say!("OK TEL {:.1}", self.tel_hz); }
                None => say!("ERR TEL needs hz"),
            }
        }
        else if is("ZERO") && self.driving {
            // With the pulses on, the pulse on the wire already IS the position (a positional servo goes
            // where its pulse says). Declaring a different angle would re-drive the pulse and jump the
            // dish unramped, so ZERO is only for limp servos: after a power cycle, or OFF and a move by hand.
            say!("ERR ZERO only while the servos are limp: send OFF first (with pulses on, the pulse is the position)");
        }
        else if is("ZERO") {
            // Declare where the axes really are: after a power cycle, or after OFF and moving them by hand.
            // Nothing moves (the servos are limp); it anchors the estimate so the next command starts from truth.
            match (num(a1), num(a2)) {
                (Some(az), Some(el)) => {
                    if !az_c.accepts(az, WINDOW_TOL) || !el_c.accepts(el, WINDOW_TOL) {
                        say!("ERR ZERO outside the window {:.0}..{:.0} / {:.0}..{:.0}", az_c.mount_lo(), az_c.mount_hi(), el_c.mount_lo(), el_c.mount_hi());
                    } else {
                        self.cmd = [az_c.clamp_mount(az), el_c.clamp_mount(el)];
                        self.tgt = self.cmd;
                        // A declared position is where the axes *are*, so the settle model is there
                        // too and the clipped-target condition no longer applies to anything.
                        self.settled = self.cmd;
                        self.clip = [false; 2];
                        self.known = true;
                        self.save();
                        say!("OK ZERO {:.2} {:.2}", self.cmd[AZ], self.cmd[EL]);
                    }
                }
                _ => say!("ERR ZERO needs az el"),
            }
        }
        else if is("CAL") {
            let (al, ah) = az_c.pulse_window();
            let (el, eh) = el_c.pulse_window();
            say!("OK CAL AZ {:.2}..{:.2} deg {:.0}..{:.0} us  EL {:.2}..{:.2} deg {:.0}..{:.0} us  known {}",
                 az_c.mount_lo(), az_c.mount_hi(), al, ah, el_c.mount_lo(), el_c.mount_hi(), el, eh, self.known as u8);
        }
        else if is("RAW") {
            let axis = match a1 {
                Some(s) if s.eq_ignore_ascii_case("AZ") => Some(AZ),
                Some(s) if s.eq_ignore_ascii_case("EL") => Some(EL),
                _ => None,
            };
            match (axis, num(a2)) {
                (Some(a), Some(us)) => {
                    // A raw pulse width, but it moves like every other command: to the angle that pulse
                    // means through this build's calibration, held inside the window, ramped at the RATE
                    // limit, with honest telemetry. It ends on exactly that pulse; the other axis holds.
                    let c = cal(a);
                    let deg = c.clamp_mount(c.deg_of_pulse(clamp(us, RAW_US.0, RAW_US.1)));
                    let (az, el) = if a == AZ { (deg, self.tgt[EL]) } else { (self.tgt[AZ], deg) };
                    self.set_target(az, el);
                    say!("OK RAW {} {}", if a == AZ { "AZ" } else { "EL" }, (c.pulse_of(deg) + 0.5) as u32);
                }
                (None, _) => say!("ERR RAW needs AZ or EL"),
                _ => say!("ERR RAW needs us"),
            }
        }
        else if is("OFF") {
            // Save first: where the axes are is still known, they are just no longer held there
            self.save();
            hw::servo_us(AZ, 0.0); hw::servo_us(EL, 0.0);
            self.pulse = [0.0; 2];
            self.driving = false;
            say!("OK OFF");
        }
        else { say!("ERR unknown command {}", verb); }
    }
}

#[entry]
fn main() -> ! {
    hw::init(BAUD);
    let watchdog_reset = hw::reset_was_watchdog();

    // Read the saved position before anything can overwrite it. MaybeUninit on a cold boot holds
    // whatever the SRAM powered up with; Persist::valid() is what rejects that.
    let saved = {
        let p = unsafe { core::ptr::read_volatile((&raw const SAVED).cast::<Persist>()) };
        if p.valid() { Some(p.to_mount(&CAL)) } else { None }
    };

    let mut m = Mount::new(saved);
    // Hold the restored position straight away: the servos pick up exactly where they were left,
    // rather than staying limp and then jumping on the first command.
    if saved.is_some() { m.take_wire(); }

    hw::watchdog_start();

    let mut rx = hw::Rx::new();
    let mut line = [0u8; 64];
    let mut len = 0usize;
    let mut overflow = false;
    let mut resync = false;     // after a receive overrun: skip to the next line end

    say!("READY {FW} PROTO {TEL_TAG}");
    match saved {
        Some((az, el)) => say!("POS {az:.2} {el:.2} restored, holding"),
        None => say!("POS UNKNOWN: send ZERO az el before moving"),
    }
    if watchdog_reset { say!("NOTE reset by watchdog"); }

    let mut last_tick = hw::micros();
    let mut last_tel = last_tick;
    loop {
        hw::watchdog_feed();

        // serial: assemble lines from the DMA ring
        loop {
            let c = rx.read();
            if rx.overrun {
                // Input arrived faster than the replies drained and lapped the receive ring. Unread
                // commands are gone and the next bytes may start mid-command: say so, drop the line
                // being assembled, and skip to the next line end rather than guess at a damaged command.
                rx.overrun = false;
                say!("ERR receive overrun: input lost, resend");
                len = 0; overflow = false; resync = true;
            }
            let Some(c) = c else { break };
            if c == b'\n' || c == b'\r' {
                if resync { resync = false; }
                else if overflow { say!("ERR line too long"); }
                else if len > 0 {
                    match core::str::from_utf8(&line[..len]) {
                        Ok(s) => m.handle(s),
                        Err(_) => say!("ERR not text"),
                    }
                }
                len = 0; overflow = false;
            } else if resync {
                // the tail of a command whose start was lost: dropped
            } else if len < line.len() {
                line[len] = c; len += 1;
            } else {
                overflow = true;
            }
        }

        let now = hw::micros();
        // slew, once per servo frame
        let since = now.wrapping_sub(last_tick);
        if since >= TICK_US {
            last_tick = now;
            m.tick(since as f32 * 1e-6);
        }
        // telemetry
        if m.tel_hz > 0.0 && now.wrapping_sub(last_tel) >= (1e6 / m.tel_hz) as u32 {
            last_tel = now;
            m.telemetry();
        }
        // LD2: slow blink while limp, fast while slewing, steady while holding
        let ms = now / 1000;
        hw::led(if !m.driving { ms % 1000 < 100 } else if m.moving() { ms % 200 < 100 } else { true });
    }
}

//! Axis limits, the degree <-> pulse map and the persisted position, as pure arithmetic.
//!
//! This file is the authoritative safety layer. It has no hardware in it on purpose, so the same
//! source compiles two ways:
//!   * as a module of the firmware (`mod limits;`), for the board
//!   * as its own crate root under `rustc --test`, for a host test run (`./test_host.sh`)
//! Nothing here calls `std`, allocates, or uses an `f32` method that only exists in `std`
//! (`fabs`/`fmin` are hand-rolled below), which is what lets both builds work.
//!
//! ## Why azimuth cannot tangle the cable
//!
//! The mount azimuth is an **absolute, continuous, unwrapped** angle: 0 at one mechanical end of the
//! Stingray-4's 450 deg of travel, rising monotonically to 450 at the other. There is no modular
//! arithmetic anywhere on this axis, in this file or in the firmware. A move from A to B therefore
//! always traverses the interval [min(A,B), max(A,B)] and nothing else, so a command can never be
//! satisfied "the short way round" through the cable wrap.
//!
//! The cable service loop is 1.25 turns and there is no slip ring, so the usable window is clamped to
//! `limit_lo .. limit_hi` = 0 .. 400 deg (Beck's limit, 50 deg inside the hardware travel). Choosing
//! *which* 0-360 wrap a bearing is tracked at is a planning problem, and that belongs to the host
//! (`perigee-control`, `MountGeom::nearest_allowed_az` / `solve_path`). The firmware's only job is to
//! make a host bug harmless: every pulse that leaves this file is clamped into the window, so the
//! worst a bad command can do is stop at the limit.
//!
//! ## Position tracking
//!
//! These are positional servo gearboxes with an internal pot and a 3-wire connector: the pulse width
//! *is* the absolute position command and there is no feedback wire to read (see
//! `reports/stingray-power.md`). So the position estimate is the ramped command, `Mount::cmd`, which
//! is exact to the servo's own tracking error once it has settled. Neither axis has an encoder: both
//! run open loop, and nothing here can detect a stalled or unpowered servo.

/// Axis indices, used everywhere as array subscripts
pub const AZ: usize = 0;
pub const EL: usize = 1;

/// Absolute value without `std`
#[inline]
pub fn fabs(v: f32) -> f32 { if v < 0.0 { -v } else { v } }
/// Clamp without `std`; `lo` wins if the bounds are crossed
#[inline]
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32 { if v < lo { lo } else if v > hi { hi } else { v } }

/// One axis: its gearbox, its calibration and its limits.
///
/// Angles come in two frames and the difference matters:
///   * **gearbox angle** 0 .. `gear_travel`, measured from the mechanical end stop
///   * **mount angle**, what the protocol and the host talk in: `mount = gearbox - offset`
///
/// `offset` is therefore both the frame tie and the mechanical safety margin at the low end: with
/// `offset = 2.0`, mount azimuth 0 sits 2 deg off the hard stop.
#[derive(Clone, Copy)]
pub struct AxisCal {
    /// Mechanical travel of the gearbox, deg (Stingray-4: 450, Stingray-9: 200)
    pub gear_travel: f32,
    /// mount angle = gearbox angle - offset
    pub offset: f32,
    /// Soft limits in the mount frame, the window the protocol accepts
    pub limit_lo: f32,
    pub limit_hi: f32,
    /// Deg of gearbox travel kept clear of *each* mechanical end, whatever the soft limits say.
    /// An independent backstop: it still applies if `limit_lo`/`limit_hi` are edited wrongly.
    pub gear_margin: f32,
    /// Pulse width that puts the gearbox at 0 deg, us (bench calibration)
    pub pulse_at_zero: f32,
    /// Microseconds of pulse per degree of gearbox travel (bench calibration)
    pub us_per_deg: f32,
    /// Hard clamp on the pulse itself, us, the last line of defence
    pub pulse_min: f32,
    pub pulse_max: f32,
}

impl AxisCal {
    /// Lowest gearbox angle this axis may ever be driven to
    #[inline]
    pub fn gear_lo(&self) -> f32 {
        let from_limit = self.limit_lo + self.offset;
        if from_limit > self.gear_margin { from_limit } else { self.gear_margin }
    }
    /// Highest gearbox angle this axis may ever be driven to
    #[inline]
    pub fn gear_hi(&self) -> f32 {
        let from_limit = self.limit_hi + self.offset;
        let from_mech = self.gear_travel - self.gear_margin;
        if from_limit < from_mech { from_limit } else { from_mech }
    }
    /// The usable window in the mount frame: the soft limits, tightened by the mechanical margin.
    /// This is what the protocol reports and what a command is checked against.
    #[inline]
    pub fn mount_lo(&self) -> f32 { self.gear_lo() - self.offset }
    #[inline]
    pub fn mount_hi(&self) -> f32 { self.gear_hi() - self.offset }

    /// Is this mount angle inside the window? `tol` absorbs float noise on a command sent as "400.00".
    #[inline]
    pub fn accepts(&self, mount_deg: f32, tol: f32) -> bool {
        mount_deg >= self.mount_lo() - tol && mount_deg <= self.mount_hi() + tol
    }
    #[inline]
    pub fn clamp_mount(&self, mount_deg: f32) -> f32 { clamp(mount_deg, self.mount_lo(), self.mount_hi()) }

    /// Mount angle -> pulse width, us. Clamped twice: into the mount window, then into the pulse
    /// range. Every pulse the firmware emits goes through here, so nothing can drive past the limit.
    #[inline]
    pub fn pulse_of(&self, mount_deg: f32) -> f32 {
        let gear = self.clamp_mount(mount_deg) + self.offset;
        clamp(self.pulse_at_zero + gear * self.us_per_deg, self.pulse_min, self.pulse_max)
    }
    /// Pulse width -> mount angle, the inverse, for picking up where a RAW pulse left the axis
    #[inline]
    pub fn deg_of_pulse(&self, us: f32) -> f32 {
        (us - self.pulse_at_zero) / self.us_per_deg - self.offset
    }
    /// Pulse width at the two ends of the usable window, us, for the calibration readout
    pub fn pulse_window(&self) -> (f32, f32) { (self.pulse_of(self.mount_lo()), self.pulse_of(self.mount_hi())) }
}

/// Move `from` toward `to` by at most `rate * dt` degrees. The slew-rate limit is what keeps the
/// commanded position a believable model of where the gearbox actually is: the servo is given a
/// target it can reach within one 20 ms frame instead of a step it lags behind by seconds.
#[inline]
pub fn ramp(from: f32, to: f32, rate: f32, dt: f32) -> f32 {
    let step = rate * dt;
    from + clamp(to - from, -step, step)
}

//--------------------------------------------------------------------------------- the servo model
/// Vendor no-load speed of each gearbox at 7.4 V, converted to deg/s: the Stingray-4 on azimuth is
/// 15 RPM (90 deg/s) and the Stingray-9 on elevation is 6.6 RPM (39.6 deg/s). Both are *no load*
/// figures at the top of the voltage range; a 9 kg dish on a 150 mm boom at 6 V is slower.
pub const SERVO_RATED_DPS: [f32; 2] = [90.0, 39.6];

/// Fraction of the rated speed the settle model credits the servo with. Deliberately pessimistic:
/// the model's only job is to say "the dish cannot possibly be there yet", so crediting the servo
/// with less speed than it has is the safe direction to be wrong in. Raise it only against a
/// measured step response.
pub const SERVO_SETTLE_FRACTION: f32 = 0.70;

/// Speed the settle model moves an axis at, deg/s
#[inline]
pub fn servo_model_dps(axis: usize) -> f32 { SERVO_RATED_DPS[axis] * SERVO_SETTLE_FRACTION }

//------------------------------------------------------------------------------ the telemetry line
/// Protocol tag on every telemetry line, and the one the host must be built against.
///
/// Bumped whenever the line's shape changes. Before `T4` the firmware put the ramped command in
/// fields 1-2 *and* fields 3-4 and never transmitted the target at all, so a host comparing "what
/// was asked for" against "where we are" was comparing a number against itself and could never
/// report off point. A host that sees any other tag must refuse the line rather than guess at it.
pub const TEL_TAG: &str = "T4";

/// Fixed-size line buffer: builds one protocol line with `write!` and no allocator.
/// An overflowing write truncates and reports an error rather than panicking.
pub struct Line<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> Line<N> {
    pub const fn new() -> Self { Line { buf: [0; N], len: 0 } }
    pub fn as_str(&self) -> &str {
        match core::str::from_utf8(&self.buf[..self.len]) { Ok(s) => s, Err(_) => "" }
    }
    #[allow(dead_code)]
    pub fn len(&self) -> usize { self.len }
}

impl<const N: usize> Default for Line<N> {
    fn default() -> Self { Self::new() }
}

impl<const N: usize> core::fmt::Write for Line<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if self.len >= N { return Err(core::fmt::Error); }
            self.buf[self.len] = b;
            self.len += 1;
        }
        Ok(())
    }
}

/// Everything the firmware honestly knows about where the axes are, in one place.
///
/// The three position fields are three different claims and the host treats them as such:
///   * `tgt`     what the host asked for, after the window clamp. The firmware's intent.
///   * `cmd`     the slew-rate-ramped command, which *is* the pulse on the wire. Where the servo
///               has been told to be, not where it is.
///   * `settled` `cmd` lagged by `servo_model_dps`, so it reaches `tgt` only once a servo running
///               at a conservative fraction of its rated speed could have got there. This is the
///               earliest moment the dish can be on target, and nothing weaker should be allowed
///               to satisfy an on-point test.
/// None of them is a measurement: no encoder is fitted on either axis.
pub struct Telemetry {
    pub tgt: [f32; 2],
    pub cmd: [f32; 2],
    pub settled: [f32; 2],
    /// The position estimate is anchored to something real (a `ZERO`, a `RAW` pulse, or a restored
    /// record). False after a power cycle, and then nothing downstream may be trusted.
    pub known: bool,
    pub moving: bool,
    /// The last target request was outside the axis window and was clamped, so the axis will never
    /// reach what the host asked for. A standing condition, not a one-shot event.
    pub clip: [bool; 2],
}

/// Fields 7 and 8 of the line, where an encoder angle and raw count would go. No encoder is fitted, so
/// they are always `NA -1`; they stay in the line so the `T4` shape, and every parser of it, is unchanged.
pub const TEL_NO_ENCODER: &str = "NA -1";

impl Telemetry {
    /// Status letters: K known, M moving, A azimuth clipped, E elevation clipped. `-` when none of
    /// them apply, so the field is never empty.
    pub fn flags(&self) -> Line<8> {
        use core::fmt::Write;
        let mut f = Line::<8>::new();
        if self.known { let _ = f.write_char('K'); }
        if self.moving { let _ = f.write_char('M'); }
        if self.clip[AZ] { let _ = f.write_char('A'); }
        if self.clip[EL] { let _ = f.write_char('E'); }
        if f.len == 0 { let _ = f.write_char('-'); }
        f
    }

    /// The whole line, without its terminator:
    /// `T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags`
    pub fn line(&self) -> Line<96> {
        use core::fmt::Write;
        let mut l = Line::<96>::new();
        let _ = write!(l, "{TEL_TAG} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2} {TEL_NO_ENCODER} {}",
                       self.tgt[AZ], self.tgt[EL], self.cmd[AZ], self.cmd[EL], self.settled[AZ], self.settled[EL],
                       self.flags().as_str());
        l
    }
}

//-------------------------------------------------------------------------------------- persistence
/// Magic word marking a valid saved position. Changed only when the record's *layout* changes: the
/// record holds pulse widths, so a calibration change needs no bump (see `Persist`).
pub const PERSIST_MAGIC: u32 = 0x5047_4D04; // "PGM" + version 4: pulse widths instead of mount degrees

/// The last known axis position, kept in a RAM region the startup code does not clear.
///
/// A system reset (the 1 s watchdog, the reset button, a reflash with `probe-rs run`) leaves SRAM
/// alone, so the position survives it and the firmware picks the pulses back up where they were
/// instead of slewing blind. A **power cycle does not**: SRAM is gone, `valid()` returns false, and
/// the firmware stays limp, says `POS UNKNOWN` and refuses to move until the host sends `ZERO az el`.
///
/// It stores the **pulse widths**, not mount degrees. The pulse *is* the servo's position, so after a
/// reflash with new calibration constants the restored pulse is exactly the one already on the wire:
/// the dish does not move, and `to_mount` re-reads that pulse as an angle through the new calibration.
/// (Version 3 stored degrees, which the new constants turned into a different pulse: a jump at boot.)
#[derive(Clone, Copy)]
pub struct Persist {
    pub magic: u32,
    pub az_us: f32,
    pub el_us: f32,
    pub check: u32,
}

impl Persist {
    pub const fn blank() -> Self { Persist { magic: 0, az_us: 0.0, el_us: 0.0, check: 0 } }

    fn checksum(az_us: f32, el_us: f32) -> u32 {
        // Sum of the bit patterns, rotated: catches a half-written record or uninitialised RAM that
        // happens to hold the magic word.
        let a = az_us.to_bits();
        let e = el_us.to_bits();
        PERSIST_MAGIC ^ a.rotate_left(7) ^ e.rotate_left(19) ^ 0xA5A5_5A5A
    }

    /// Remember the two pulse widths on the wire, microseconds
    pub fn store(&mut self, az_us: f32, el_us: f32) {
        self.az_us = az_us;
        self.el_us = el_us;
        self.check = Self::checksum(az_us, el_us);
        self.magic = PERSIST_MAGIC;
    }

    /// Is the record intact, and are both pulses finite and inside the servos' 500..2500 range?
    pub fn valid(&self) -> bool {
        let pulse = |us: f32| finite(us) && us >= 500.0 && us <= 2500.0;
        self.magic == PERSIST_MAGIC
            && self.check == Self::checksum(self.az_us, self.el_us)
            && pulse(self.az_us)
            && pulse(self.el_us)
    }

    /// The saved pulses read as mount angles through *this* build's calibration, held inside the window
    pub fn to_mount(&self, cal: &[AxisCal; 2]) -> (f32, f32) {
        (cal[AZ].clamp_mount(cal[AZ].deg_of_pulse(self.az_us)), cal[EL].clamp_mount(cal[EL].deg_of_pulse(self.el_us)))
    }

    /// Throw the saved position away. Used by the host tests; the firmware never needs it, because a
    /// cold boot leaves SRAM holding something that fails `valid()` by itself.
    #[allow(dead_code)]
    pub fn invalidate(&mut self) { self.magic = 0; self.check = 0; }
}

/// True for a normal number: rejects NaN and both infinities without `std`
#[inline]
pub fn finite(v: f32) -> bool {
    let bits = v.to_bits();
    (bits & 0x7F80_0000) != 0x7F80_0000
}

//-------------------------------------------------------------------------------------- the two axes
/// Azimuth: goBILDA Stingray-4, 3215-0001-0004, 450 deg of travel.
///
/// The slope is the endpoint calibration 500 us -> 0 deg, 2500 us -> 450 deg, i.e. 4.4444 us/deg
/// (0.225 deg/us). goBILDA publish both "450 deg travel" and "0.23 deg/us", which disagree by 2%, and
/// until docs/calibration.md 6.3/6.4 measure it nobody knows which one this gearbox follows.
///
/// `offset = 8.0` (mount azimuth 0 sits 8 deg off the low stop by this model) is chosen so that the
/// `[0, 400]` window stays clear of BOTH hard stops under either reading: if the gearbox really runs at
/// 0.23 deg/us centred on 1500 us, mount 0 lands about 3 deg off the low stop (with offset 2 it was 3 deg
/// *past* it: the servo stalled against the stop on the simulated mount) and mount 400 about 412 of 450.
/// Keep 8 after calibrating too: it costs nothing (the window still spans 400 deg; 42 deg of travel
/// stay unused at the top) and it is the margin that protects the stops from the next calibration error.
pub const AZ_CAL: AxisCal = AxisCal {
    gear_travel: 450.0,
    offset: 8.0,
    limit_lo: 0.0,
    limit_hi: 400.0,
    gear_margin: 2.0,
    pulse_at_zero: 500.0,
    us_per_deg: 2000.0 / 450.0,
    pulse_min: 500.0,
    pulse_max: 2500.0,
};

/// Elevation: goBILDA Stingray-9, 3215-0001-0009, 200 deg of travel.
///
/// `offset = 5.0` is the built geometry (mount elevation 0 at gearbox 5 deg). The soft window is
/// -2 .. 91: the dish never tips past vertical (there is no flip-over: a pass near the zenith is
/// followed in azimuth, which may briefly lag at the top), 91 leaves one degree of slack over the
/// zenith, and `el_min` -2 (not the -5 the config used to carry) is what buys margin at the
/// bottom stop -- at -5 with this offset the gearbox sits at exactly 0 deg, hard against it, with a
/// 9 kg dish on the arm. The slope 10 us/deg is the endpoint calibration and agrees exactly with the
/// published 0.10 deg/us.
pub const EL_CAL: AxisCal = AxisCal {
    gear_travel: 200.0,
    offset: 5.0,
    limit_lo: -2.0,
    limit_hi: 91.0,
    gear_margin: 2.0,
    pulse_at_zero: 500.0,
    us_per_deg: 2000.0 / 200.0,
    pulse_min: 500.0,
    pulse_max: 2500.0,
};

pub const CAL: [AxisCal; 2] = [AZ_CAL, EL_CAL];

//-------------------------------------------------------------------------------------- host tests
// Only compiled when this file is built as its own crate root by `./test_host.sh`; invisible to the
// firmware build, where `limits.rs` is a module and `cfg(test)` is never set.
#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool { fabs(a - b) < 1e-3 }

    #[test]
    fn azimuth_window_is_becks_zero_to_four_hundred() {
        assert!(close(AZ_CAL.mount_lo(), 0.0), "{}", AZ_CAL.mount_lo());
        assert!(close(AZ_CAL.mount_hi(), 400.0), "{}", AZ_CAL.mount_hi());
        // and it sits inside the 450 deg of hardware travel with margin at both ends
        assert!(AZ_CAL.gear_lo() >= AZ_CAL.gear_margin);
        assert!(AZ_CAL.gear_hi() <= AZ_CAL.gear_travel - AZ_CAL.gear_margin);
    }

    #[test]
    fn azimuth_window_clears_both_stops_under_either_vendor_slope() {
        // M7: goBILDA publish 450 deg over 500..2500 us (0.225 deg/us, anchored at 500 us) and 0.23 deg/us.
        // Read the second as centred on 1500 us = gearbox 225. Under both, the window's two end pulses
        // must land at least 2 deg inside the hard stops (0 and 450).
        let (lo, hi) = AZ_CAL.pulse_window();
        let endpoint = |us: f32| (us - 500.0) * 0.225;
        let centred = |us: f32| 225.0 + (us - 1500.0) * 0.23;
        for g in [endpoint(lo), centred(lo)] { assert!(g >= 2.0, "mount 0 lands at gearbox {g}"); }
        for g in [endpoint(hi), centred(hi)] { assert!(g <= 448.0, "mount 400 lands at gearbox {g}"); }
    }

    #[test]
    fn elevation_window_keeps_margin_at_both_stops() {
        assert!(close(EL_CAL.mount_lo(), -2.0), "{}", EL_CAL.mount_lo());
        assert!(close(EL_CAL.mount_hi(), 91.0), "{}", EL_CAL.mount_hi());
        // never past vertical by more than the one degree of slack: there is no flip-over
        assert!(!EL_CAL.accepts(92.0, 0.01) && close(EL_CAL.clamp_mount(150.0), 91.0));
        assert!(EL_CAL.gear_lo() >= 2.0 - 1e-3, "gear_lo {}", EL_CAL.gear_lo());
        assert!(EL_CAL.gear_hi() <= 198.0 + 1e-3, "gear_hi {}", EL_CAL.gear_hi());
    }

    #[test]
    fn a_bad_limit_still_cannot_reach_a_mechanical_stop() {
        // Somebody edits limit_hi past the travel: gear_margin is the independent backstop
        let mut bad = AZ_CAL;
        bad.limit_hi = 9_999.0;
        bad.limit_lo = -9_999.0;
        assert!(close(bad.gear_hi(), 448.0), "{}", bad.gear_hi());
        assert!(close(bad.gear_lo(), 2.0), "{}", bad.gear_lo());
        assert!(bad.pulse_of(100_000.0) <= 500.0 + 448.0 * bad.us_per_deg + 1e-3);
    }

    #[test]
    fn pulses_stay_inside_the_servos_range() {
        for c in CAL {
            let (lo, hi) = c.pulse_window();
            assert!(lo >= 500.0 && hi <= 2500.0, "{lo} .. {hi}");
            // and the extremes of the range are never produced by an in-window angle
            assert!(lo > 500.0 && hi < 2500.0, "no pulse headroom: {lo} .. {hi}");
        }
    }

    #[test]
    fn every_pulse_is_clamped_however_absurd_the_command() {
        for c in CAL {
            let (lo, hi) = c.pulse_window();
            for cmd in [-1.0e9, -400.0, -5.0, 0.0, 180.0, 399.0, 400.0, 401.0, 450.0, 1.0e9] {
                let us = c.pulse_of(cmd);
                assert!(us >= lo - 1e-3 && us <= hi + 1e-3, "cmd {cmd} -> {us}, window {lo}..{hi}");
            }
        }
    }

    #[test]
    fn az_401_is_refused_not_wrapped() {
        // The firmware does not reason about wraps; it refuses and clamps. 401 becomes 400, NOT 41.
        assert!(!AZ_CAL.accepts(401.0, 0.01));
        assert!(close(AZ_CAL.clamp_mount(401.0), 400.0));
        assert!(AZ_CAL.accepts(400.0, 0.01), "exactly 400 must be legal");
        assert!(AZ_CAL.accepts(0.0, 0.01), "exactly 0 must be legal");
        assert!(!AZ_CAL.accepts(-0.5, 0.01));
    }

    #[test]
    fn pulse_and_degrees_round_trip() {
        for c in CAL {
            for deg in [0.0f32, 10.0, 100.0, 180.0] {
                let d = c.clamp_mount(deg);
                assert!(close(c.deg_of_pulse(c.pulse_of(d)), d), "{d} on a {} deg axis", c.gear_travel);
            }
        }
    }

    #[test]
    fn calibration_matches_the_published_slopes() {
        // 0.225 deg/us on azimuth (endpoint form; vendor says 0.23, 2% apart, see the doc comment)
        assert!(close(1.0 / AZ_CAL.us_per_deg, 0.225));
        // 0.10 deg/us on elevation, exactly as published
        assert!(close(1.0 / EL_CAL.us_per_deg, 0.10));
    }

    #[test]
    fn ramp_is_rate_limited_and_lands_exactly() {
        assert!(close(ramp(0.0, 100.0, 20.0, 1.0), 20.0));
        assert!(close(ramp(0.0, -100.0, 20.0, 1.0), -20.0));
        assert!(close(ramp(99.5, 100.0, 20.0, 1.0), 100.0), "a short move must not overshoot");
        assert!(close(ramp(100.0, 100.0, 20.0, 1.0), 100.0));
    }

    #[test]
    fn a_full_sweep_at_the_rate_limit_never_leaves_the_window() {
        // 0 -> 400 at 20 deg/s in 20 ms frames: every intermediate pulse must be legal
        let (lo, hi) = AZ_CAL.pulse_window();
        let mut p = AZ_CAL.mount_lo();
        for _ in 0..2_000 {
            p = ramp(p, 400.0, 20.0, 0.02);
            let us = AZ_CAL.pulse_of(p);
            assert!(us >= lo - 1e-3 && us <= hi + 1e-3, "at {p} deg -> {us} us");
        }
        assert!(close(p, 400.0), "did not arrive: {p}");
    }

    #[test]
    fn persist_round_trips_and_rejects_garbage() {
        let mut p = Persist::blank();
        assert!(!p.valid(), "a blank record must not be trusted");
        p.store(2275.5, 950.25);
        assert!(p.valid());
        assert!(close(p.az_us, 2275.5) && close(p.el_us, 950.25));
        // a single flipped bit in either field is caught
        let mut t = p; t.az_us = 0.0;
        assert!(!t.valid());
        let mut t = p; t.el_us += 1.0;
        assert!(!t.valid());
        // and a pulse no servo would ever be sent is not trusted even with a good checksum
        let mut t = Persist::blank(); t.store(3000.0, 950.0);
        assert!(!t.valid());
        // the magic word alone is not enough
        let mut t = Persist::blank(); t.magic = PERSIST_MAGIC;
        assert!(!t.valid());
        p.invalidate();
        assert!(!p.valid());
    }

    //---------------------------------------------------------------------------- the servo model
    #[test]
    fn the_settle_model_is_slower_than_the_servos_are_rated_for() {
        // The model must never claim the dish arrived before it could have. It also must not be so
        // slow that it never arrives: both axes stay well above the 20 / 15 deg/s slew limits the
        // host commands, so `settled` lags `cmd` by a bounded amount and then catches up.
        assert!(close(servo_model_dps(AZ), 63.0), "{}", servo_model_dps(AZ));
        assert!(close(servo_model_dps(EL), 27.72), "{}", servo_model_dps(EL));
        for a in [AZ, EL] {
            assert!(servo_model_dps(a) < SERVO_RATED_DPS[a], "the model must be pessimistic");
            assert!(servo_model_dps(a) > 20.0, "axis {a} model is slower than the commanded slew rate");
        }
        // A 90 deg azimuth step cannot be settled in under 90/63 s however the ramp is driven
        let mut settled = 0.0f32;
        let mut t = 0.0f32;
        while settled < 90.0 - 1e-3 && t < 10.0 { settled = ramp(settled, 90.0, servo_model_dps(AZ), 0.02); t += 0.02; }
        assert!(t >= 90.0 / 63.0 - 0.02, "settled in {t} s, faster than the model allows");
    }

    //---------------------------------------------------------------------------- the telemetry line
    fn sample() -> Telemetry {
        Telemetry {
            tgt: [200.0, 45.0], cmd: [187.25, 45.0], settled: [180.5, 45.0],
            known: true, moving: true, clip: [false; 2],
        }
    }

    #[test]
    fn telemetry_carries_the_target_the_command_and_the_settled_estimate_separately() {
        // The whole point of the format: three different numbers, so the host can compare what was
        // asked for against where the dish can be. The old line repeated the command twice.
        let t = sample();
        let l = t.line();
        let s = l.as_str();
        assert_eq!(s, "T4 200.00 45.00 187.25 45.00 180.50 45.00 NA -1 KM");
        assert_eq!(n_fields(s), 10, "{s}");
        assert_ne!(field(s, 1), field(s, 3), "target and command must not be the same field twice");
        assert_ne!(field(s, 3), field(s, 5), "command and settled estimate must not be the same field twice");
    }

    #[test]
    fn telemetry_reports_every_flag_and_never_an_encoder() {
        let mut t = sample();
        t.clip = [true, true]; t.moving = false;
        let l = t.line();
        assert_eq!(l.as_str(), "T4 200.00 45.00 187.25 45.00 180.50 45.00 NA -1 KAE");
        // nothing anchored, nothing moving: the encoder slots are NA -1 as always, flags `-`
        let mut t = sample();
        t.known = false; t.moving = false;
        assert_eq!(t.line().as_str(), "T4 200.00 45.00 187.25 45.00 180.50 45.00 NA -1 -");
        assert_eq!(t.flags().as_str(), "-", "the flag field is never empty");
    }

    #[test]
    fn a_telemetry_line_fits_its_buffer_at_every_extreme() {
        // Widest possible: four-digit negatives on every axis, every flag set
        let t = Telemetry {
            tgt: [-399.99, -185.55], cmd: [-399.99, -185.55], settled: [-399.99, -185.55],
            known: true, moving: true, clip: [true; 2],
        };
        let l = t.line();
        let s = l.as_str();
        assert!(l.len() < 96, "{} bytes: {s}", l.len());
        assert!(s.starts_with("T4 "));
        assert_eq!(n_fields(s), 10);
        assert!(s.ends_with(" NA -1 KMAE"));
    }

    fn n_fields(s: &str) -> usize { s.split_ascii_whitespace().count() }
    fn field(s: &str, i: usize) -> &str { s.split_ascii_whitespace().nth(i).unwrap_or("") }

    #[test]
    fn a_reflash_with_new_constants_restores_the_same_pulse() {
        // M4: the record holds the pulse on the wire. A build with a different calibration reads it back
        // as a different *angle*, but drives exactly the same *pulse*: the servo does not move.
        let old = CAL;
        let mut new = CAL;
        new[AZ].us_per_deg = 1.0 / 0.23; new[AZ].pulse_at_zero = 519.58;   // the simulated bench result
        new[EL].offset = 8.518;
        let (az, el) = (300.0f32, 40.0f32);
        let mut p = Persist::blank();
        p.store(old[AZ].pulse_of(az), old[EL].pulse_of(el));
        assert!(p.valid());
        let (az2, el2) = p.to_mount(&new);
        assert!(fabs(new[AZ].pulse_of(az2) - old[AZ].pulse_of(az)) < 0.01, "azimuth pulse changed");
        assert!(fabs(new[EL].pulse_of(el2) - old[EL].pulse_of(el)) < 0.01, "elevation pulse changed");
        // under the old constants it is simply the same pose again
        let (az3, el3) = p.to_mount(&old);
        assert!(close(az3, az) && close(el3, el));
    }

    #[test]
    fn persist_rejects_nan_and_infinity() {
        assert!(finite(0.0) && finite(-400.0) && finite(1e30));
        assert!(!finite(f32::NAN) && !finite(f32::INFINITY) && !finite(f32::NEG_INFINITY));
        let mut p = Persist::blank();
        p.store(f32::NAN, 0.0);
        assert!(!p.valid(), "a NaN position must never be restored");
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The mount: its frame, its limits, the path solver that turns a sky track into axis commands, and the
/// wireframe drawing of the gimbal for the MOUNT tile.
///
/// Frames.  The sky uses true bearing (0 north, clockwise) and elevation above the level horizon, the
/// same frame the viewer and Perigee's ranking use.  The mount uses its own two axis readings:
///   az_m  az_lo .. az_hi        (0 .. 400, inside the Stingray-4's 450 of hardware travel)
///                               bearing = az_zero + az_m,  az_zero = center - az_travel/2
///   el_m  el_min .. el_max      (-2 .. 91 on the Stingray-9: never more than a degree past vertical)
///                               el_m = el + el_corr
/// Two measured numbers tie them together: az_center_bearing, the bearing the dish faces at mid travel, and
/// el_corr, how much higher the mount's elevation reads than the true one (0 unless /here measured it).
///
/// No flip-over.  Elevation stops at 91, so every sky direction has one mount solution: azimuth from the
/// bearing, elevation as is.  A pass that climbs near the zenith makes the azimuth axis swing quickly at
/// the top; the dish may lag there for a few seconds, which costs little because near the zenith an
/// azimuth error barely moves the beam.  (The flip-over was removed for simplicity on 2026-10-01.)
///
/// Wrap, and why it matters.  Azimuth is absolute and continuous: there is no modular arithmetic on it
/// anywhere, here or in the firmware, so a move from A to B always traverses [min, max] and nothing else.
/// The cable service loop is 1.25 turns with no slip ring, so the usable window is az_lo..az_hi = 0..400,
/// not the full 450 of hardware travel.  Because the window still spans more than a turn, 40 degrees'
/// worth of bearings are reachable at two axis positions 360 apart, and choosing between them is the
/// whole game:
///
///   * `nearest_allowed_az` is the point rule: of the candidates inside the window, take the one nearest
///     where the axis is now.  At 399 asked for the position 401 would reach, 401 is outside the window,
///     so the only candidate is 41 and the mount unwinds 358 degrees the other way rather than tangling.
///   * `solve_path` is the pass rule: unwrap the whole predicted track into one continuous azimuth run and
///     pick the whole-turn offset that fits all of it inside the window, preferring the start nearest the
///     current position.  Only when no single offset fits is an unwind scheduled mid-pass, at the split
///     point that loses the fewest samples, and it is reported so it gets logged rather than discovered.
///
/// The firmware enforces the same window independently (`firmware/perigee_mount_stm32/src/limits.rs`):
/// a bug on this side can cost a pass, never the cable.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::config::MountCfg;
use bevy::prelude::*;

/// Tolerance for "is this azimuth inside the window": a candidate computed as exactly `az_hi` must
/// count as reachable, and the arithmetic that produced it went through two `rem_euclid`s.
const EPS: f64 = 1e-9;

#[derive(Clone, Copy, Debug)]
pub struct MountGeom {
    /// Hardware travel of the azimuth gearbox, deg. Defines the frame (az_zero sits at travel/2 below
    /// the centre bearing); it is NOT the usable range.
    pub az_travel: f64,
    pub az_center: f64,
    /// The usable azimuth window, deg, inside `az_travel`: the cable-wrap soft limits. The firmware
    /// enforces the same pair independently.
    pub az_lo: f64,
    pub az_hi: f64,
    pub el_min: f64,
    pub el_max: f64,
    /// Mount elevation minus true elevation (the console's /here): added to every sky elevation going to
    /// the mount, taken off every mount elevation shown as sky. `el_min .. el_max` stay in mount degrees.
    pub el_corr: f64,
}

impl MountGeom {
    pub fn from_cfg(c: &MountCfg) -> Self {
        Self {
            az_travel: c.az_travel_deg, az_center: c.az_center_or_nominal(),
            az_lo: c.az_limit_lo_deg, az_hi: c.az_limit_hi_deg,
            el_min: c.el_min_deg, el_max: c.el_max_deg, el_corr: c.el_correction_deg,
        }
    }
    /// True bearing of mount azimuth zero
    pub fn az_zero(&self) -> f64 { (self.az_center - self.az_travel / 2.0).rem_euclid(360.0) }
    /// Mount azimuth -> true bearing
    pub fn bearing(&self, az_m: f64) -> f64 { (self.az_zero() + az_m).rem_euclid(360.0) }
    /// Width of the usable azimuth window, deg
    pub fn az_span(&self) -> f64 { self.az_hi - self.az_lo }
    /// Is this mount azimuth inside the usable window?
    pub fn az_ok(&self, az_m: f64) -> bool { az_m >= self.az_lo - EPS && az_m <= self.az_hi + EPS }

    /// Every mount azimuth **inside the usable window** that faces this bearing. One value where the
    /// window covers a bearing once, two where it covers it twice (the 40 degrees of overlap on a
    /// 0..400 window), and none at all if the window were ever narrower than a full turn.
    ///
    /// `nearest_allowed_az` is what the planners call; this is the full set, kept for the tests and
    /// for anything that needs to show an operator both positions a bearing can be reached at.
    #[allow(dead_code)]
    pub fn az_candidates(&self, bearing: f64) -> Vec<f64> {
        let base = (bearing - self.az_zero()).rem_euclid(360.0);
        //First whole-turn offset of `base` that reaches the bottom of the window
        let mut a = base + 360.0 * ((self.az_lo - base) / 360.0).ceil();
        let mut out = Vec::new();
        while a <= self.az_hi + EPS {
            if a >= self.az_lo - EPS { out.push(a); }
            a += 360.0;
        }
        out
    }

    /// **The wrap rule, in mount-azimuth space.** `want` is any azimuth that points the right way;
    /// its whole-turn equivalents inside the window are the candidates, and the one nearest `from`
    /// wins. When the short way out of the window is excluded, what is left is the long way round,
    /// which is exactly the behaviour that keeps the cable intact.
    ///
    /// Beck's case: window 0..400, `from` 399, `want` 401 -> 401 is outside, 41 is the only candidate,
    /// so the answer is 41 and the move is 358 degrees of unwinding, not 2 degrees of tangling.
    ///
    /// `None` when no equivalent of `want` lands inside the window at all.
    pub fn nearest_allowed_az(&self, want: f64, from: f64) -> Option<f64> {
        //Lowest whole-turn equivalent of `want` that reaches the window, then step up by turns
        let mut a = want + 360.0 * ((self.az_lo - want) / 360.0).ceil();
        let mut best: Option<(f64, f64)> = None;   // (az, |move|)
        while a <= self.az_hi + EPS {
            if a >= self.az_lo - EPS {
                let cost = (a - from).abs();
                if best.map_or(true, |(_, c)| cost < c) { best = Some((a, cost)); }
            }
            a += 360.0;
        }
        best.map(|(a, _)| a)
    }

    /// The wrap rule in bearing space: the reachable mount azimuth facing `bearing` that is nearest to
    /// where the axis is now. What a manual aim or any single pointing command should go through.
    pub fn az_for_bearing(&self, bearing: f64, from: f64) -> Option<f64> {
        self.nearest_allowed_az((bearing - self.az_zero()).rem_euclid(360.0), from)
    }

    /// Sky elevation -> the mount elevation that looks there
    pub fn mount_el(&self, el: f64) -> f64 { el + self.el_corr }

    /// The mount pose that looks at (bearing, el), its azimuth the reachable one nearest `from` (the wrap
    /// rule, via `az_for_bearing`). None when the elevation is outside the window (below the horizon).
    pub fn nearest_pose(&self, bearing: f64, el: f64, from: (f64, f64)) -> Option<(f64, f64)> {
        let el_m = self.mount_el(el);
        if el_m < self.el_min - EPS || el_m > self.el_max + EPS { return None; }
        Some((self.az_for_bearing(bearing, from.0)?, el_m))
    }
    pub fn clamp(&self, az_m: f64, el_m: f64) -> (f64, f64) {
        (az_m.clamp(self.az_lo, self.az_hi), el_m.clamp(self.el_min, self.el_max))
    }
    /// Sky direction the mount is looking at for a mount pose (bearing, elevation). Past 90 (the one
    /// degree of slack at the top) the boresight has tipped over, so it faces the other way.
    pub fn sky_of(&self, az_m: f64, el_m: f64) -> (f64, f64) {
        let el = el_m - self.el_corr;
        if el > 90.0 { ((self.bearing(az_m) + 180.0).rem_euclid(360.0), 180.0 - el) } else { (self.bearing(az_m), el) }
    }

    /// Solve a whole pass. `samples` are (t seconds, bearing deg, elevation deg) in time order.
    ///
    /// The bearings are unwrapped into one continuous azimuth run (each sample moved by whole turns to
    /// sit within half a turn of the one before) and the elevations moved into mount degrees (`el_corr`).
    ///
    /// **Pass-level wrap choice.** The azimuth run is then shifted by whole turns so the entire track
    /// fits inside `az_lo .. az_hi`, with no unwind anywhere in the middle of the pass. Where several
    /// offsets fit, the one whose start is nearest `from_az` wins (shortest slew to the AOS point),
    /// and margin to the limits breaks the remaining ties. Where **none** fits, because the track is
    /// longer than the window, an unwind is scheduled at the split point that loses the fewest
    /// samples, and recorded in `MountPath::unwind` so the tracker logs it.
    ///
    /// `from_az` is where the azimuth axis is now (None: no preference). `az_rate` is the slew limit
    /// in deg/s, needed to price an unwind in lost samples.
    pub fn solve_path(&self, samples: &[(f64, f64, f64)], from_az: Option<f64>, az_rate: f64) -> Option<MountPath> {
        if samples.len() < 2 { return None; }
        let mut seq: Vec<(f64, f64)> = Vec::with_capacity(samples.len());   // (az from mount zero, unwrapped; el)
        for &(_, b, e) in samples {
            let a = (b - self.az_zero()).rem_euclid(360.0);
            let a = match seq.last() { Some(&(pa, _)) => nearest_turn(a, pa), None => a };
            seq.push((a, self.mount_el(e)));
        }
        let el_out = seq.iter().filter(|&&(_, e)| e < self.el_min || e > self.el_max).count();
        let lo = seq.iter().map(|s| s.0).fold(f64::MAX, f64::min);
        let hi = seq.iter().map(|s| s.0).fold(f64::MIN, f64::max);
        //Which whole-turn offsets could ever matter: enough to cover the window from either end
        let turns = || {
            let k0 = ((self.az_lo - hi) / 360.0).floor() as i32 - 1;
            let k1 = ((self.az_hi - lo) / 360.0).ceil() as i32 + 1;
            (k0..=k1).map(|k| k as f64 * 360.0)
        };
        let fits = |off: f64| seq.iter().all(|&(a, _)| self.az_ok(a + off));
        let margin_of = |off: f64| (lo + off - self.az_lo).min(self.az_hi - (hi + off));

        //1. The whole pass at one wrap, if any offset fits. Prefer the start nearest where the axis
        //   is now (shortest pre-position slew), then the most room to the limits.
        let whole = turns().filter(|&off| fits(off)).min_by(|&x, &y| {
            let key = |off: f64| {
                let reach = from_az.map_or(0.0, |f| (seq[0].0 + off - f).abs());
                (reach, -margin_of(off))
            };
            key(x).partial_cmp(&key(y)).unwrap_or(std::cmp::Ordering::Equal)
        });

        let (off, unwind) = match whole {
            Some(off) => (off, None),
            //2. Nothing fits: the azimuth run is longer than the window, so one unwind is
            //   unavoidable. Score every (start offset, split index, end offset) that is legal on
            //   both sides by how many samples survive, and take the best.
            None => {
                let best_split = self.best_unwind(&seq, samples, &turns().collect::<Vec<_>>(), az_rate.max(0.1), from_az);
                match best_split {
                    Some(u) => (u.off_a, Some(u)),
                    //3. Not even a split works (a track longer than two windows, or a window
                    //   narrower than the run on both sides): fall back to the offset that keeps
                    //   the most samples and let the path be CLIPPED at the limit.
                    None => {
                        let off = turns().max_by_key(|&off| seq.iter().filter(|&&(a, _)| self.az_ok(a + off)).count())?;
                        (off, None)
                    }
                }
            }
        };

        //Build the commanded points. With an unwind, the track is two segments joined by a
        //deliberate traverse the other way round; samples that fall inside the traverse are lost.
        let mut points: Vec<(f64, f64, f64)> = Vec::with_capacity(samples.len() + 1);
        match &unwind {
            None => for (&(t, _, _), &(a, e)) in samples.iter().zip(&seq) {
                let (az, el) = self.clamp(a + off, e);
                points.push((t, az, el));
            },
            Some(u) => {
                //Segment A at the first wrap...
                for i in 0..u.split {
                    let (az, el) = self.clamp(seq[i].0 + u.off_a, seq[i].1);
                    points.push((samples[i].0, az, el));
                }
                //...the traverse, arriving at the re-entry sample when the slew can finish...
                let (az, el) = self.clamp(seq[u.resume].0 + u.off_b, seq[u.resume].1);
                points.push((u.t_end, az, el));
                //...then segment B. Samples the traverse ran past are already gone.
                for i in (u.resume + 1)..seq.len() {
                    if samples[i].0 <= u.t_end { continue; }
                    let (az, el) = self.clamp(seq[i].0 + u.off_b, seq[i].1);
                    points.push((samples[i].0, az, el));
                }
            }
        }
        let inside = seq.iter().filter(|&&(a, _)| self.az_ok(a + off)).count();
        let margin = match &unwind {
            None => margin_of(off),
            Some(u) => u.margin,
        };
        //Rates measured on the commanded path, with the unwind traverse excluded: it is a planned
        //repositioning slew at the axis limit, not a tracking rate the pass demands.
        let (mut max_az_rate, mut max_el_rate) = (0.0f64, 0.0f64);
        for w in points.windows(2) {
            let dt = (w[1].0 - w[0].0).max(1e-6);
            let is_traverse = unwind.as_ref().map_or(false, |u| w[1].0 == u.t_end);
            if !is_traverse { max_az_rate = max_az_rate.max(((w[1].1 - w[0].1) / dt).abs()); }
            max_el_rate = max_el_rate.max(((w[1].2 - w[0].2) / dt).abs());
        }
        let clipped = (unwind.is_none() && inside < seq.len()) || el_out > 0;
        Some(MountPath { points, clipped, margin_deg: margin, max_az_rate, max_el_rate, unwind })
    }

    /// Cheapest single mid-pass unwind, or None when no two-segment split is legal.
    ///
    /// Cost is counted in lost samples: the traverse takes |delta| / az_rate seconds at the slew limit,
    /// and every sample whose time falls inside that window is time the dish is not on the satellite.
    /// Ties go to the split that leaves the most margin, then the earliest one (unwinding sooner leaves
    /// the later, usually higher-elevation, part of the pass intact).
    fn best_unwind(
        &self, seq: &[(f64, f64)], samples: &[(f64, f64, f64)], offsets: &[f64], az_rate: f64,
        from_az: Option<f64>,
    ) -> Option<Unwind> {
        let n = seq.len();
        //Tightest approach to either limit over a span of azimuths
        let margin_of = |lo: f64, hi: f64| (lo - self.az_lo).min(self.az_hi - hi);
        let span = |s: &[(f64, f64)], off: f64| {
            let lo = s.iter().map(|q| q.0).fold(f64::MAX, f64::min) + off;
            let hi = s.iter().map(|q| q.0).fold(f64::MIN, f64::max) + off;
            (lo, hi)
        };
        let mut best: Option<(f64, Unwind)> = None;   // (score, unwind); higher score wins
        for &off_a in offsets {
            //How far into the pass this offset stays legal. `whole` already failed, so this is < n.
            let lead = seq.iter().position(|&(a, _)| !self.az_ok(a + off_a)).unwrap_or(n).min(n - 1);
            if lead == 0 { continue; }
            for &off_b in offsets {
                if (off_b - off_a).abs() < 1.0 { continue; }
                //...and this one has to be legal from there to the end
                if !seq[lead..].iter().all(|&(a, _)| self.az_ok(a + off_b)) { continue; }
                //Split anywhere in the legal lead: run segment A to sample `split - 1`, traverse,
                //resume at the first sample the slew can still reach in time
                for split in 1..=lead {
                    let t_split = samples[split - 1].0;
                    let leave = seq[split - 1].0 + off_a;
                    //Resume at the first sample the traverse can actually reach in time. The traverse
                    //gets longer the later it lands, so the condition is evaluated per candidate
                    //rather than against one up-front estimate.
                    let reaches = |i: usize| samples[i].0 >= t_split + ((seq[i].0 + off_b) - leave).abs() / az_rate;
                    let resume = (split..n).find(|&i| reaches(i)).unwrap_or(n - 1);
                    let secs = ((seq[resume].0 + off_b) - leave).abs() / az_rate;
                    //Arrive on the sample if the slew is done by then, otherwise as soon as it can be
                    let t_end = (t_split + secs).max(samples[resume].0);
                    let lost = resume - split;
                    let (a_lo, a_hi) = span(&seq[..split], off_a);
                    let (b_lo, b_hi) = span(&seq[resume..], off_b);
                    let margin = margin_of(a_lo, a_hi).min(margin_of(b_lo, b_hi));
                    let reach = from_az.map_or(0.0, |f| (seq[0].0 + off_a - f).abs());
                    //Samples tracked dominates; then margin; then unwind earlier (so the later, higher
                    //part of the pass stays intact); then the shorter slew to the start
                    let score = (n - lost) as f64 * 1e6 + margin * 1e2 - split as f64 - reach * 1e-3;
                    let u = Unwind {
                        off_a, off_b, split, resume, t_start: t_split, t_end,
                        from_az: seq[split - 1].0 + off_a, to_az: seq[resume].0 + off_b,
                        seconds: secs, lost, margin,
                    };
                    if best.as_ref().map_or(true, |(s, _)| score > *s) { best = Some((score, u)); }
                }
            }
        }
        best.map(|(_, u)| u)
    }
}

/// A deliberate mid-pass unwind: the azimuth axis runs the long way round to the other wrap of the
/// same bearing, because the track is longer than the usable window and no single wrap fits it.
#[derive(Clone, Copy, Debug)]
pub struct Unwind {
    pub off_a: f64,      // whole-turn offset before the unwind
    pub off_b: f64,      // and after it
    pub split: usize,    // first sample index that belongs to the second segment
    pub resume: usize,   // first sample index actually tracked after the traverse
    pub t_start: f64,    // pass-relative seconds the traverse begins
    pub t_end: f64,      // and ends
    pub from_az: f64,    // mount azimuth it leaves
    pub to_az: f64,      // and arrives at
    pub seconds: f64,    // how long the traverse takes at the slew limit
    pub lost: usize,     // samples the pass loses to it
    pub margin: f64,     // tightest approach to a limit across both segments
}

/// `a` moved by whole turns to sit within half a turn of `prev`
pub fn nearest_turn(a: f64, prev: f64) -> f64 {
    let mut v = a;
    while v - prev > 180.0 { v -= 360.0; }
    while v - prev < -180.0 { v += 360.0; }
    v
}


#[derive(Clone, Debug)]
pub struct MountPath {
    pub points: Vec<(f64, f64, f64)>,   // (t seconds, az_m, el_m)
    pub clipped: bool,                  // part of the pass is outside the window (held at the limit there)
    pub margin_deg: f64,                // room to the nearest azimuth limit
    pub max_az_rate: f64,               // deg/s, the unwind traverse excluded
    pub max_el_rate: f64,
    /// Set when the track is longer than the usable azimuth window and one mid-pass unwind the long
    /// way round was unavoidable. `points` already contains the traverse; this is what to log.
    pub unwind: Option<Unwind>,
}

impl MountPath {
    pub fn t_start(&self) -> f64 { self.points.first().map_or(0.0, |p| p.0) }
    pub fn t_end(&self) -> f64 { self.points.last().map_or(0.0, |p| p.0) }
    /// True while the dish is running the long way round rather than following the satellite
    pub fn unwinding_at(&self, t: f64) -> bool {
        self.unwind.map_or(false, |u| t >= u.t_start && t <= u.t_end)
    }
    /// Axis commands at time t (linear between samples, held at the ends)
    pub fn at(&self, t: f64) -> (f64, f64) {
        let p = &self.points;
        if t <= p[0].0 { return (p[0].1, p[0].2); }
        if t >= p[p.len() - 1].0 { let l = p[p.len() - 1]; return (l.1, l.2); }
        let i = p.partition_point(|s| s.0 <= t).max(1);
        let (a, b) = (p[i - 1], p[i]);
        let f = ((t - a.0) / (b.0 - a.0).max(1e-9)).clamp(0.0, 1.0);
        (a.1 + (b.1 - a.1) * f, a.2 + (b.2 - a.2) * f)
    }
}

//------------------------------------------------------------------------------------------ live state
/// Telemetry tag this host is built against; must equal `limits::TEL_TAG` in the firmware (a test checks the source)
pub const TEL_TAG: &str = "T4";

/// One `T4` report, its three position claims kept apart as the firmware sends them. None is measured: no encoder is fitted.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct Pose {
    pub tgt: (f64, f64),        // the target the firmware holds, after its window clamp
    pub cmd: (f64, f64),        // the slew-ramped command: the pulse on the wire, not where the dish is
    pub settled: (f64, f64),    // cmd lagged by the firmware's servo model: the earliest the dish can be there
    pub known: bool,            // K: anchored to something real; false after a power cycle
    pub moving: bool,           // M
    pub clip: [bool; 2],        // A, E: the standing target was clamped at that axis's limit
}

impl Pose {
    /// `T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags`. None for any other shape, so a reshaped line is refused, not guessed at.
    pub fn parse(line: &str) -> Option<Pose> {
        let f: Vec<&str> = line.split_whitespace().collect();
        if f.len() != 10 || f[0] != TEL_TAG { return None; }
        let n = |i: usize| f[i].parse::<f64>().ok().filter(|v| v.is_finite());
        let flags = f[9];
        if flags != "-" && !flags.chars().all(|c| "KMAE".contains(c)) { return None; }
        Some(Pose { tgt: (n(1)?, n(2)?), cmd: (n(3)?, n(4)?), settled: (n(5)?, n(6)?),
                    known: flags.contains('K'), moving: flags.contains('M'), clip: [flags.contains('A'), flags.contains('E')] })
    }
    /// On point for `asked` (the host's last GO): anchored, unclipped, still, holding that very target (not a stale one), settled on it
    pub fn on_point(&self, asked: (f64, f64), tol: f64) -> bool {
        let near = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol;
        self.known && !self.moving && !self.clip[0] && !self.clip[1] && near(self.tgt, asked) && near(self.settled, self.tgt)
    }
    /// Why `on_point` is false, as a short phrase for the screen (the first condition that fails); None when on point
    pub fn waiting_for(&self, asked: (f64, f64), tol: f64) -> Option<String> {
        let near = |a: (f64, f64), b: (f64, f64)| (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol;
        if !self.known { return Some("position unknown (ZERO az el)".into()); }
        if self.clip[0] { return Some("target clipped at the azimuth limit".into()); }
        if self.clip[1] { return Some("target clipped at the elevation limit".into()); }
        if !near(self.tgt, asked) { return Some("firmware holds a different target".into()); }
        if self.moving { return Some("moving".into()); }
        if !near(self.settled, self.tgt) {
            let d = (self.settled.0 - self.tgt.0).abs().max((self.settled.1 - self.tgt.1).abs());
            return Some(format!("settling, {d:.1} deg to go"));
        }
        None
    }
}

/// What the firmware last reported. `T4` is the protocol; the pre-T4 `T` line (fields: going-to az el, at az el,
/// moving 0/1, encoder) is still shown so an old board is visible, but it carries no settled estimate and can never put the mount on point.
#[derive(Resource, Default, Debug, Clone)]
pub struct MountState {
    pub cmd: Option<(f64, f64)>,      // mount-frame target the firmware is moving to: T4 tgt, legacy fields 1-2
    pub fb: Option<(f64, f64)>,       // where the dish is taken to be: T4 settled estimate, legacy fields 3-4. Never measured.
    pub moving: bool,
    pub encoder: Option<f64>,         // legacy field 6 only; T4 sends `NA -1` because no encoder is fitted
    pub pose: Option<Pose>,           // the last T4 report whole; None while only legacy lines arrive
    pub last_seen: Option<std::time::Instant>,
    pub firmware: String,
}

impl MountState {
    /// Parse one line from the mount. Returns true when it was telemetry.
    pub fn ingest(&mut self, line: &str) -> bool {
        let mut it = line.split_whitespace();
        match it.next() {
            Some(TEL_TAG) => {
                if let Some(p) = Pose::parse(line) {
                    self.cmd = Some(p.tgt); self.fb = Some(p.settled); self.moving = p.moving; self.encoder = None;
                    self.pose = Some(p);
                    self.last_seen = Some(std::time::Instant::now());
                }
                true
            }
            Some("T") => {
                let v: Vec<f64> = it.filter_map(|s| s.parse().ok()).collect();
                if v.len() >= 4 {
                    self.cmd = Some((v[0], v[1]));
                    self.fb = Some((v[2], v[3]));
                    self.moving = v.get(4).map_or(false, |m| *m != 0.0);
                    self.encoder = v.get(5).copied();
                    self.pose = None;
                    self.last_seen = Some(std::time::Instant::now());
                }
                true
            }
            Some("READY") | Some("ID") => { self.firmware = it.collect::<Vec<_>>().join(" "); false }
            _ => false,
        }
    }
    pub fn alive(&self) -> bool { self.last_seen.map_or(false, |t| t.elapsed().as_secs_f64() < 3.0) }
}

//------------------------------------------------------------------------------------------ wireframe
/// Camera for the MOUNT tile: orbits the gimbal, hand-rolled perspective (mm in, pixels out)
#[derive(Resource)]
pub struct MountView { pub yaw: f32, pub pitch: f32, pub zoom: f32 }
impl Default for MountView { fn default() -> Self { Self { yaw: -0.6, pitch: 0.42, zoom: 1.0 } } }

/// Perigee gimbal dimensions (mm) from perigee-mount/params.json, enough for a recognisable outline
mod dims {
    pub const PEDESTAL_R: f32 = 126.0; pub const PEDESTAL_H: f32 = 75.0; pub const PLATE_H: f32 = 8.0;
    pub const HOUSING_R: f32 = 116.0; pub const HOUSING_TOP: f32 = 240.0; pub const YOKE_PLATE_R: f32 = 126.0; pub const YOKE_PLATE_H: f32 = 16.0;
    pub const COLUMN_X: f32 = 91.0; pub const COLUMN_R: f32 = 33.0;
    pub const EL_AXIS_Z: f32 = 599.0; pub const DRUM_R: f32 = 75.0; pub const DRUM_X0: f32 = 58.0; pub const DRUM_X1: f32 = 124.0;
    pub const HUB: f32 = 90.0; pub const BOOM_LEN: f32 = 150.0; pub const BOOM_R: f32 = 20.0;
    pub const DISH_OFFSET: f32 = 150.0; pub const DISH_R: f32 = 500.0; pub const DISH_DEPTH: f32 = 150.0; pub const DISH_FOCAL: f32 = 400.0;
    pub const CW_ARM: f32 = 175.0; pub const CW_R: f32 = 50.0; pub const CW_LEN: f32 = 165.0;
}

/// Draw the gimbal into a screen rectangle (UI pixels, y down).  `fb` is the estimated pose (no sensor), `cmd` the pose
/// the firmware is driving to (its boresight is drawn as a dim ray), `target` a sky direction (bearing, el)
/// drawn as a third ray so you can see the dish converge on the satellite.
pub fn draw_mount(
    lines: &mut Vec<(Vec2, Vec2, Color)>, rect: crate::tiles::Rect, view: &MountView, geom: &MountGeom,
    fb: Option<(f64, f64)>, cmd: Option<(f64, f64)>, target: Option<(f64, f64)>, colors: &MountColors,
) {
    //Projection: world mm (x east, y north, z up) -> tile pixels
    let center = Vec3::new(0.0, 0.0, 380.0);
    let dist = 3600.0f32;
    let scale = rect.h.min(rect.w * 0.8) / 1500.0 * view.zoom;
    let (sy, cy) = view.yaw.sin_cos();
    let (sp, cp) = view.pitch.sin_cos();
    let project = |p: Vec3| -> Vec2 {
        let q = p - center;
        let (x, y, z) = (q.x * cy - q.y * sy, q.x * sy + q.y * cy, q.z);   // yaw about z
        let u = x;
        let v = z * cp - y * sp;                                            // pitch about the screen x axis
        let depth = y * cp + z * sp;
        let s = scale * dist / (dist + depth);
        Vec2::new(rect.x + rect.w * 0.5 + u * s, rect.y + rect.h * 0.62 - v * s)
    };
    let seg = |lines: &mut Vec<(Vec2, Vec2, Color)>, a: Vec3, b: Vec3, c: Color| lines.push((project(a), project(b), c));
    let circle_z = |lines: &mut Vec<(Vec2, Vec2, Color)>, cz: Vec3, r: f32, c: Color, tf: &dyn Fn(Vec3) -> Vec3| {
        let n = 28;
        let mut prev = None;
        for i in 0..=n {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            let p = tf(Vec3::new(cz.x + r * a.cos(), cz.y + r * a.sin(), cz.z));
            if let Some(q) = prev { lines.push((project(q), project(p), c)); }
            prev = Some(p);
        }
    };
    let ident = |p: Vec3| p;

    //Fixed base: pedestal, flange plate, north tick
    let base = colors.fixed;
    circle_z(lines, Vec3::ZERO, dims::PEDESTAL_R, base, &ident);
    circle_z(lines, Vec3::new(0.0, 0.0, dims::PEDESTAL_H), dims::PEDESTAL_R, base, &ident);
    circle_z(lines, Vec3::new(0.0, 0.0, dims::PEDESTAL_H + dims::PLATE_H), dims::PEDESTAL_R, base, &ident);
    for i in 0..4 {
        let a = i as f32 * std::f32::consts::FRAC_PI_2 + 0.4;
        let p = Vec3::new(dims::PEDESTAL_R * a.cos(), dims::PEDESTAL_R * a.sin(), 0.0);
        seg(lines, p, p + Vec3::Z * dims::PEDESTAL_H, base);
    }
    //Compass: N tick (double), E/S/W single, at the pedestal foot
    for (k, b) in [0.0f32, 90.0, 180.0, 270.0].iter().enumerate() {
        let d = Vec3::new(b.to_radians().sin(), b.to_radians().cos(), 0.0);
        seg(lines, d * (dims::PEDESTAL_R + 20.0), d * (dims::PEDESTAL_R + if k == 0 { 110.0 } else { 55.0 }), colors.compass);
        if k == 0 { let s = Vec3::new(d.y, -d.x, 0.0) * 12.0; seg(lines, d * (dims::PEDESTAL_R + 20.0) + s, d * (dims::PEDESTAL_R + 110.0) + s, colors.compass); }
    }
    //Azimuth soft limits on the ground: a radial at each end of the usable window (0 and 400, not the
    //450 of hardware travel). When the window is under a full turn the bearings between them cannot be
    //reached at all: a dim arc marks that gap. At 400 there is no gap, there is 40 deg of overlap.
    {
        let r0 = dims::PEDESTAL_R + 25.0; let r1 = dims::PEDESTAL_R + 60.0;
        for b in [geom.az_zero() + geom.az_lo, geom.az_zero() + geom.az_hi] {
            let a = (b as f32).to_radians();
            seg(lines, Vec3::new(r0 * a.sin(), r0 * a.cos(), 0.0), Vec3::new(r1 * a.sin(), r1 * a.cos(), 0.0), colors.gap);
        }
        let gap = (360.0 - geom.az_span()) as f32;
        if gap > 0.5 {
            let start = (geom.az_zero() as f32 + geom.az_hi as f32).to_radians();
            let n = 16; let r = dims::PEDESTAL_R + 35.0;
            for i in 0..n {
                let a0 = start + gap.to_radians() * i as f32 / n as f32;
                let a1 = start + gap.to_radians() * (i + 1) as f32 / n as f32;
                seg(lines, Vec3::new(r * a0.sin(), r * a0.cos(), 0.0), Vec3::new(r * a1.sin(), r * a1.cos(), 0.0), colors.gap);
            }
        }
    }

    let pose = fb.or(cmd).unwrap_or(((geom.az_lo + geom.az_hi) / 2.0, 0.0));
    let bearing = geom.bearing(pose.0) as f32;
    //Drawn at the dish's real elevation, so the model lines up with its own boresight ray (sky_of)
    let el = (pose.1 - geom.el_corr) as f32;
    //Head frame: local +y = boresight at EL 0, local x = elevation axis. Rotate about z by the bearing.
    let (sb, cb) = (bearing.to_radians().sin(), bearing.to_radians().cos());
    let head = move |p: Vec3| Vec3::new(p.x * cb + p.y * sb, -p.x * sb + p.y * cb, p.z);
    //Cradle frame: elevation about the head's x axis through z = EL_AXIS_Z
    let (se, ce) = (el.to_radians().sin(), el.to_radians().cos());
    let cradle = move |p: Vec3| {
        let (y, z) = (p.y, p.z - dims::EL_AXIS_Z);
        head(Vec3::new(p.x, y * ce - z * se, y * se + z * ce + dims::EL_AXIS_Z))
    };
    let hc = colors.head;
    //Housing drum + yoke plate
    circle_z(lines, Vec3::new(0.0, 0.0, dims::PEDESTAL_H + dims::PLATE_H), dims::HOUSING_R, hc, &head);
    circle_z(lines, Vec3::new(0.0, 0.0, dims::HOUSING_TOP), dims::HOUSING_R, hc, &head);
    circle_z(lines, Vec3::new(0.0, 0.0, dims::HOUSING_TOP), dims::YOKE_PLATE_R, hc, &head);
    circle_z(lines, Vec3::new(0.0, 0.0, dims::HOUSING_TOP + dims::YOKE_PLATE_H), dims::YOKE_PLATE_R, hc, &head);
    for i in 0..4 {
        let a = i as f32 * std::f32::consts::FRAC_PI_2 + 0.8;
        let p = Vec3::new(dims::HOUSING_R * a.cos(), dims::HOUSING_R * a.sin(), dims::PEDESTAL_H + dims::PLATE_H);
        seg(lines, head(p), head(p + Vec3::Z * (dims::HOUSING_TOP - dims::PEDESTAL_H - dims::PLATE_H)), hc);
    }
    //Columns and shoulder drums
    let z0 = dims::HOUSING_TOP + dims::YOKE_PLATE_H;
    for sx in [-1.0f32, 1.0] {
        let cx = sx * dims::COLUMN_X;
        circle_z(lines, Vec3::new(cx, 0.0, z0), dims::COLUMN_R, hc, &head);
        for j in 0..3 {
            let a = j as f32 * std::f32::consts::TAU / 3.0;
            let p = Vec3::new(cx + dims::COLUMN_R * a.cos(), dims::COLUMN_R * a.sin(), z0);
            seg(lines, head(p), head(Vec3::new(p.x, p.y, dims::EL_AXIS_Z - dims::DRUM_R * 0.6)), hc);
        }
        //Drum: circles in the y-z plane at |x| = DRUM_X0 and DRUM_X1
        for x in [sx * dims::DRUM_X0, sx * dims::DRUM_X1] {
            let n = 28; let mut prev = None;
            for i in 0..=n {
                let a = i as f32 / n as f32 * std::f32::consts::TAU;
                let p = head(Vec3::new(x, dims::DRUM_R * a.cos(), dims::EL_AXIS_Z + dims::DRUM_R * a.sin()));
                if let Some(q) = prev { lines.push((project(q), project(p), hc)); }
                prev = Some(p);
            }
        }
        for j in 0..4 {
            let a = j as f32 * std::f32::consts::FRAC_PI_2;
            let (y, z) = (dims::DRUM_R * a.cos(), dims::EL_AXIS_Z + dims::DRUM_R * a.sin());
            seg(lines, head(Vec3::new(sx * dims::DRUM_X0, y, z)), head(Vec3::new(sx * dims::DRUM_X1, y, z)), hc);
        }
    }
    //Cradle: hub box, boom, dish, counterweight
    let cc = colors.cradle;
    let h = dims::HUB / 2.0;
    let corners = |x: f32| [Vec3::new(x, -h, dims::EL_AXIS_Z - h), Vec3::new(x, h, dims::EL_AXIS_Z - h), Vec3::new(x, h, dims::EL_AXIS_Z + h), Vec3::new(x, -h, dims::EL_AXIS_Z + h)];
    let (l, r) = (corners(-h), corners(h));
    for i in 0..4 { seg(lines, cradle(l[i]), cradle(l[(i + 1) % 4]), cc); seg(lines, cradle(r[i]), cradle(r[(i + 1) % 4]), cc); seg(lines, cradle(l[i]), cradle(r[i]), cc); }
    let boom_end = h + dims::BOOM_LEN;
    for (dx, dz) in [(dims::BOOM_R, 0.0), (-dims::BOOM_R, 0.0), (0.0, dims::BOOM_R), (0.0, -dims::BOOM_R)] {
        seg(lines, cradle(Vec3::new(dx, h, dims::EL_AXIS_Z + dz)), cradle(Vec3::new(dx, boom_end, dims::EL_AXIS_Z + dz)), cc);
        seg(lines, cradle(Vec3::new(dx, -h, dims::EL_AXIS_Z + dz)), cradle(Vec3::new(dx, -h - dims::CW_ARM, dims::EL_AXIS_Z + dz)), cc);
    }
    //Counterweight canister
    let cw0 = -h - dims::CW_ARM;
    for y in [cw0, cw0 - dims::CW_LEN] {
        let n = 20; let mut prev = None;
        for i in 0..=n {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            let p = cradle(Vec3::new(dims::CW_R * a.cos(), y, dims::EL_AXIS_Z + dims::CW_R * a.sin()));
            if let Some(q) = prev { lines.push((project(q), project(p), cc)); }
            prev = Some(p);
        }
    }
    for j in 0..4 {
        let a = j as f32 * std::f32::consts::FRAC_PI_2;
        seg(lines, cradle(Vec3::new(dims::CW_R * a.cos(), cw0, dims::EL_AXIS_Z + dims::CW_R * a.sin())),
            cradle(Vec3::new(dims::CW_R * a.cos(), cw0 - dims::CW_LEN, dims::EL_AXIS_Z + dims::CW_R * a.sin())), cc);
    }
    //Dish: vertex at boom end + offset, paraboloid z = r^2 / (4 f), rim at r = DISH_R
    let vy = boom_end + dims::DISH_OFFSET;
    let dc = colors.dish;
    {
        let n = 36; let mut prev = None;
        for i in 0..=n {
            let a = i as f32 / n as f32 * std::f32::consts::TAU;
            let p = cradle(Vec3::new(dims::DISH_R * a.cos(), vy + dims::DISH_DEPTH, dims::EL_AXIS_Z + dims::DISH_R * a.sin()));
            if let Some(q) = prev { lines.push((project(q), project(p), dc)); }
            prev = Some(p);
        }
        //Ribs: 8 parabolic sections from the vertex to the rim
        for j in 0..8 {
            let a = j as f32 * std::f32::consts::TAU / 8.0;
            let mut prev = None;
            for k in 0..=6 {
                let rr = dims::DISH_R * k as f32 / 6.0;
                let depth = rr * rr / (4.0 * dims::DISH_FOCAL) * (dims::DISH_DEPTH / (dims::DISH_R * dims::DISH_R / (4.0 * dims::DISH_FOCAL)));
                let p = cradle(Vec3::new(rr * a.cos(), vy + depth, dims::EL_AXIS_Z + rr * a.sin()));
                if let Some(q) = prev { lines.push((project(q), project(p), dc)); }
                prev = Some(p);
            }
        }
        //Feed: three struts from the rim to the focus
        let focus = cradle(Vec3::new(0.0, vy + dims::DISH_FOCAL, dims::EL_AXIS_Z));
        for j in 0..3 {
            let a = j as f32 * std::f32::consts::TAU / 3.0 + 0.5;
            let p = cradle(Vec3::new(dims::DISH_R * 0.9 * a.cos(), vy + dims::DISH_DEPTH * 0.8, dims::EL_AXIS_Z + dims::DISH_R * 0.9 * a.sin()));
            lines.push((project(p), project(focus), dc));
        }
    }
    //Rays from the elevation axis: estimated boresight (bright), commanded (dim), sky target (accent)
    let axis = Vec3::new(0.0, 0.0, dims::EL_AXIS_Z);
    let ray = |lines: &mut Vec<(Vec2, Vec2, Color)>, bearing: f32, el: f32, len: f32, c: Color| {
        let (sb, cb) = (bearing.to_radians().sin(), bearing.to_radians().cos());
        let (se, ce) = (el.to_radians().sin(), el.to_radians().cos());
        let d = Vec3::new(sb * ce, cb * ce, se);
        lines.push((project(axis), project(axis + d * len), c));
    };
    if let Some(p) = fb { let (b, e) = geom.sky_of(p.0, p.1); ray(lines, b as f32, e as f32, 1500.0, colors.ray_fb); }
    if let Some(p) = cmd { let (b, e) = geom.sky_of(p.0, p.1); ray(lines, b as f32, e as f32, 1300.0, colors.ray_cmd); }
    if let Some((b, e)) = target { ray(lines, b as f32, e as f32, 1700.0, colors.ray_target); }
}

pub struct MountColors { pub fixed: Color, pub head: Color, pub cradle: Color, pub dish: Color, pub compass: Color, pub gap: Color, pub ray_fb: Color, pub ray_cmd: Color, pub ray_target: Color }

#[cfg(test)]
mod tests {
    use super::*;
    fn geom() -> MountGeom { MountGeom { az_travel: 450.0, az_center: 217.5, az_lo: 0.0, az_hi: 400.0, el_min: -2.0, el_max: 91.0, el_corr: 0.0 } }

    #[test]
    fn the_elevation_correction_applies_both_ways_and_the_window_stays_in_mount_degrees() {
        //The mount reads 2 degrees high: everything sent goes up by 2, everything shown comes down by 2
        let g = MountGeom { el_corr: 2.0, ..geom() };
        assert_eq!(g.nearest_pose(200.0, 45.0, (200.0, 45.0)).map(|p| p.1), Some(47.0));
        let (_, e) = g.sky_of(200.0, 47.0);
        assert!((e - 45.0).abs() < 1e-12, "{e}");
        //The firmware's window is mount degrees: sky 89.5 is mount 91.5, past the top, so out of reach...
        assert!(g.nearest_pose(200.0, 89.5, (200.0, 45.0)).is_none());
        //...and sky -3.5 is mount -1.5, inside it
        assert!(g.nearest_pose(200.0, -3.5, (200.0, 45.0)).is_some());
        //A whole pass is planned in mount degrees too
        let samples = vec![(0.0, 120.0, 10.0), (300.0, 180.0, 40.0), (600.0, 240.0, 10.0)];
        let p = g.solve_path(&samples, None, 20.0).unwrap();
        assert!(p.points.iter().zip(&samples).all(|(q, s)| (q.2 - (s.2 + 2.0)).abs() < 1e-9), "{:?}", p.points);
        //Over the top is judged on the real elevation: mount 91 with the dish reading 2 high is 89, not tipped over
        let (b, e) = g.sky_of(200.0, 91.0);
        assert!((e - 89.0).abs() < 1e-12 && (b - g.bearing(200.0)).abs() < 1e-12);
    }

    #[test]
    fn bearing_roundtrip() {
        let g = geom();
        assert!((g.az_zero() - 352.5).abs() < 1e-9);
        assert!((g.bearing(225.0) - 217.5).abs() < 1e-9);
        //A bearing inside the doubly-covered 90 degrees has two mount azimuths
        let c = g.az_candidates(30.0);   // 30 - 352.5 = 37.5 -> 37.5 and 397.5
        assert_eq!(c.len(), 2);
        assert!((c[0] - 37.5).abs() < 1e-9 && (c[1] - 397.5).abs() < 1e-9);
        assert_eq!(g.az_candidates(217.5).len(), 1);
    }

    #[test]
    fn nearest_turn_is_continuous() {
        assert_eq!(nearest_turn(5.0, 350.0), 365.0);
        assert_eq!(nearest_turn(355.0, 380.0), 355.0);
        assert_eq!(nearest_turn(20.0, -170.0), -340.0);
    }

    //---------------------------------------------------------------------------- the wrap rule
    /// A plain 0..400 window in mount-azimuth space, which is the frame Beck's cases are stated in
    fn win() -> MountGeom { MountGeom { az_travel: 450.0, az_center: 200.0, az_lo: 0.0, az_hi: 400.0, el_min: -2.0, el_max: 91.0, el_corr: 0.0 } }

    #[test]
    fn at_399_wanting_401_the_mount_unwinds_358_the_other_way() {
        //Beck's case. 401 is outside the 400 window, so the only reachable equivalent is 41 and the
        //axis must run 358 degrees backwards rather than 2 degrees forwards into the cable.
        let g = win();
        let got = g.nearest_allowed_az(401.0, 399.0).unwrap();
        assert!((got - 41.0).abs() < 1e-9, "got {got}, wanted 41");
        assert!((got - 399.0).abs() > 357.0, "the move must be the long way: {}", got - 399.0);
    }

    #[test]
    fn the_short_way_is_taken_whenever_it_is_legal() {
        let g = win();
        //Same bearing, but from 10 rather than 399: now 41 is also the short way, 2 away
        assert!((g.nearest_allowed_az(401.0, 39.0).unwrap() - 41.0).abs() < 1e-9);
        //From 350 asked for 40: both 40 and 400 are legal, 400 is 50 away and 40 is 310, so 400 wins
        assert!((g.nearest_allowed_az(40.0, 350.0).unwrap() - 400.0).abs() < 1e-9);
        //From 100 asked for the same thing: 40 is nearer now
        assert!((g.nearest_allowed_az(40.0, 100.0).unwrap() - 40.0).abs() < 1e-9);
    }

    #[test]
    fn zero_to_359_is_the_only_candidate_so_it_goes_the_long_way() {
        //719 is outside the window, so there is no short way: 359 degrees forward it is.
        let g = win();
        let got = g.nearest_allowed_az(359.0, 0.0).unwrap();
        assert!((got - 359.0).abs() < 1e-9, "got {got}");
    }

    #[test]
    fn the_boundaries_themselves_are_reachable() {
        let g = win();
        //Exactly 400 is inside the window, not one step outside it
        assert!(g.az_ok(400.0) && g.az_ok(0.0));
        assert!(!g.az_ok(400.1) && !g.az_ok(-0.1));
        assert!((g.nearest_allowed_az(400.0, 395.0).unwrap() - 400.0).abs() < 1e-9);
        assert!((g.nearest_allowed_az(0.0, 5.0).unwrap() - 0.0).abs() < 1e-9);
        //and 400.0 arrived at by whole turns from 40 is still exactly 400, not 399.999
        assert!((g.nearest_allowed_az(40.0, 400.0).unwrap() - 400.0).abs() < 1e-9);
    }

    #[test]
    fn every_bearing_is_reachable_and_never_outside_the_window() {
        let g = geom();
        for b in 0..3600 {
            let bearing = b as f64 / 10.0;
            for from in [0.0, 100.0, 200.0, 399.0, 400.0] {
                let a = g.az_for_bearing(bearing, from).unwrap_or_else(|| panic!("bearing {bearing} unreachable"));
                assert!(g.az_ok(a), "bearing {bearing} from {from} -> {a}, outside 0..400");
                //and it really does face that bearing
                let d = (g.bearing(a) - bearing).rem_euclid(360.0);
                assert!(d < 1e-6 || d > 360.0 - 1e-6, "bearing {bearing} -> az {a} faces {}", g.bearing(a));
            }
        }
    }

    #[test]
    fn a_window_narrower_than_a_turn_has_unreachable_bearings() {
        //Not the built mount, but the rule has to hold: a 300 deg window leaves a 60 deg blind arc,
        //and the planner says None rather than clamping into it and pointing at the wrong sky.
        let g = MountGeom { az_travel: 450.0, az_center: 200.0, az_lo: 0.0, az_hi: 300.0, el_min: -2.0, el_max: 91.0, el_corr: 0.0 };
        assert!(g.nearest_allowed_az(330.0, 100.0).is_none());
        assert!(g.nearest_allowed_az(290.0, 100.0).is_some());
    }

    #[test]
    fn clamp_uses_the_soft_window_not_the_hardware_travel() {
        let g = geom();
        assert_eq!(g.clamp(430.0, 0.0).0, 400.0, "the extra 50 deg of travel is not usable");
        assert_eq!(g.clamp(-10.0, 0.0).0, 0.0);
        assert_eq!(g.clamp(200.0, -40.0).1, -2.0);
    }

    #[test]
    fn nearest_pose_prefers_the_short_move() {
        let g = geom();
        //Bearing 30 is reachable at az 37.5 and 397.5: from az 400 the second is nearer
        let (a, e) = g.nearest_pose(30.0, 20.0, (400.0, 20.0)).unwrap();
        assert!((a - 397.5).abs() < 1e-9 && e == 20.0);
        //No flip-over: a high target is reached with the elevation as it is, never past the window
        let (a, e) = g.nearest_pose(37.5, 80.0, (225.0, 45.0)).unwrap();
        assert!((a - 45.0).abs() < 1e-9 && (e - 80.0).abs() < 1e-9, "{a} {e}");
        //and below the horizon there is no pose at all
        assert!(g.nearest_pose(37.5, -10.0, (225.0, 45.0)).is_none());
    }

    #[test]
    fn south_pass_is_normal() {
        //Rising in the south-east, setting in the south-west, peak 40 deg: no wrap needed
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=60).map(|i| { let f = i as f64 / 60.0; (f * 600.0, 120.0 + f * 120.0, 40.0 * (std::f64::consts::PI * f).sin()) }).collect();
        let p = g.solve_path(&s, None, 20.0).unwrap();
        assert!(!p.clipped);
        assert!(p.points.iter().all(|&(_, a, e)| (0.0..=450.0).contains(&a) && (-5.0..=91.0).contains(&e)));
    }

    #[test]
    fn zenith_pass_swings_azimuth_and_says_so() {
        //No flip-over. Straight overhead south -> north: the elevation never passes 90 and the azimuth
        //swings 180 at the peak. The plan stays legal (no clipping, inside both windows) and the swing is
        //reported as a rate the slew limit cannot follow, which is what the PATH check warns about.
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=100).map(|i| {
            let f = i as f64 / 100.0; let el = 10.0 + 160.0 * f;          // 10 .. 170 measured from the south horizon
            if el <= 90.0 { (f * 600.0, 180.0, el) } else { (f * 600.0, 0.0, 180.0 - el) }
        }).collect();
        let p = g.solve_path(&s, None, 20.0).unwrap();
        assert!(!p.clipped);
        assert!(p.points.iter().all(|&(_, a, e)| g.az_ok(a) && e <= 90.0 + 1e-9), "{:?}", p.points);
        assert!(p.max_az_rate > 20.0, "the swing at the top must show up in the rate: {}", p.max_az_rate);
        //and the azimuth run has exactly one 180 degree swing, not a wrap jump
        let azs: Vec<f64> = p.points.iter().map(|p| p.1).collect();
        let span = azs.iter().cloned().fold(f64::MIN, f64::max) - azs.iter().cloned().fold(f64::MAX, f64::min);
        assert!((span - 180.0).abs() < 1e-6, "az span {span}");
    }

    #[test]
    fn a_track_over_the_seam_unwinds_rather_than_clipping() {
        //A track 350 -> 70 runs across the window's seam (bearing 352.5, azimuth 0 on this mount): neither
        //wrap holds all 80 degrees of it, so one unwind is scheduled, never a clip against the limit.
        //(Before 2026-10-01 the flip-over absorbed this case; without it about 3-6 % of real passes do this.)
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=40).map(|i| (i as f64 * 10.0, 350.0 + i as f64 * 2.0, 15.0)).collect();
        let p = g.solve_path(&s, None, 20.0).unwrap();
        assert!(p.unwind.is_some() && !p.clipped, "unwind {:?} clipped {}", p.unwind, p.clipped);
        assert!(p.points.iter().all(|&(_, a, _)| g.az_ok(a)));
        //A plain 360 mount has a real seam and no overlap at all: the same track is split the same way.
        let g360 = MountGeom { az_travel: 360.0, az_center: 217.5, az_lo: 0.0, az_hi: 360.0, el_min: -2.0, el_max: 90.0, el_corr: 0.0 };
        let p = g360.solve_path(&s, None, 20.0).unwrap();
        assert!(p.unwind.is_some(), "expected an unwind, got clipped={}", p.clipped);
        assert!(!p.clipped, "an unwind is a plan, not a clip");
        assert!(p.points.iter().all(|&(_, a, _)| g360.az_ok(a)));
    }

    //------------------------------------------------------------------- pass-level wrap planning
    /// A track that crosses north without reaching the seam, sampled every 10 s: bearing 330 -> 30, elevation 20
    fn north_crossing() -> Vec<(f64, f64, f64)> {
        (0..=24).map(|i| (i as f64 * 10.0, (330.0 + i as f64 * 2.5).rem_euclid(360.0), 20.0)).collect()
    }

    #[test]
    fn a_pass_crossing_north_is_planned_at_one_wrap_with_no_unwind() {
        let g = geom();
        let s = north_crossing();
        let p = g.solve_path(&s, None, 20.0).unwrap();
        assert!(p.unwind.is_none(), "a 60 deg track crossing north fits in a 400 deg window at one wrap");
        assert!(!p.clipped);
        //the azimuth run must be continuous: no 360 jump anywhere, and all of it inside the window
        for w in p.points.windows(2) {
            assert!((w[1].1 - w[0].1).abs() < 90.0, "discontinuity {} -> {}", w[0].1, w[1].1);
        }
        assert!(p.points.iter().all(|&(_, a, _)| g.az_ok(a)), "{:?}", p.points);
    }

    #[test]
    fn the_starting_wrap_is_chosen_for_the_shortest_slew_from_where_we_are() {
        //A track inside the 40 degrees the 0..400 window covers twice fits at two wraps, so the
        //current position decides which one, which is the whole point of pass-level planning. The
        //elevation ceiling is 90 here (the built mount uses 91; it makes no difference to this choice).
        let g = MountGeom { az_travel: 450.0, az_center: 200.0, az_lo: 0.0, az_hi: 400.0, el_min: -2.0, el_max: 90.0, el_corr: 0.0 };
        //az_zero is 335, so bearings 345..355 are mount az 10..20, or 370..380 a turn up
        let s: Vec<(f64, f64, f64)> = (0..=10).map(|i| (i as f64 * 10.0, 345.0 + i as f64, 20.0)).collect();
        let low = g.solve_path(&s, Some(0.0), 20.0).unwrap();
        let high = g.solve_path(&s, Some(400.0), 20.0).unwrap();
        assert!((low.points[0].1 - 10.0).abs() < 1e-9, "from 0 the plan should start at 10: {}", low.points[0].1);
        assert!((high.points[0].1 - 370.0).abs() < 1e-9, "from 400 the plan should start at 370: {}", high.points[0].1);
        //neither needs an unwind, and both stay inside the window
        for p in [&low, &high] {
            assert!(p.unwind.is_none() && !p.clipped);
            assert!(p.points.iter().all(|&(_, a, _)| g.az_ok(a)));
        }
    }

    #[test]
    fn a_pass_that_cannot_fit_gets_one_scheduled_unwind() {
        //A geostationary-style stare would never do this, but a long low pass can: an azimuth run of
        //more than 400 degrees cannot be held at any single wrap. Two full turns of bearing, sampled
        //every 10 s, is 720 degrees of continuous azimuth.
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=144).map(|i| (i as f64 * 10.0, (i as f64 * 5.0).rem_euclid(360.0), 20.0)).collect();
        let p = g.solve_path(&s, Some(0.0), 20.0).unwrap();
        let u = p.unwind.expect("a 720 deg run must schedule an unwind, not clip");
        //It really is the long way round: a whole turn, at the slew limit
        assert!((u.to_az - u.from_az).abs() > 300.0, "{} -> {}", u.from_az, u.to_az);
        //The traverse is priced at the slew limit and the schedule allows at least that long
        assert!((u.seconds - (u.to_az - u.from_az).abs() / 20.0).abs() < 1e-6, "{} s for {} deg", u.seconds, u.to_az - u.from_az);
        assert!(u.t_end - u.t_start >= u.seconds - 1e-9, "scheduled {} s for a {} s slew", u.t_end - u.t_start, u.seconds);
        assert!(u.lost > 0, "a 360 deg traverse at 20 deg/s spans more than one 10 s sample");
        //Both ends of the traverse, and every commanded point, stay inside the window
        assert!(g.az_ok(u.from_az) && g.az_ok(u.to_az));
        assert!(p.points.iter().all(|&(_, a, _)| g.az_ok(a)), "{:?}", p.points);
        //Times stay in order across the splice, or `at()` would interpolate backwards
        for w in p.points.windows(2) { assert!(w[1].0 > w[0].0, "times out of order at {}", w[0].0); }
        //and the planner reports when it is off target
        assert!(p.unwinding_at(u.t_start + u.seconds / 2.0));
        assert!(!p.unwinding_at(u.t_start - 1.0));
    }

    #[test]
    fn no_plan_ever_commands_an_azimuth_outside_the_window() {
        //The guarantee that matters, swept over start bearings and rates: whatever the planner picks,
        //every commanded azimuth is legal, so the firmware's own clamp never has to fire.
        let g = geom();
        for b0 in (0..360).step_by(15) {
            for sweep in [-720.0, -400.0, -90.0, 90.0, 400.0, 720.0] {
                let s: Vec<(f64, f64, f64)> = (0..=40)
                    .map(|i| { let f = i as f64 / 40.0; (i as f64 * 10.0, (b0 as f64 + sweep * f).rem_euclid(360.0), 20.0) })
                    .collect();
                let p = g.solve_path(&s, Some(200.0), 20.0).unwrap();
                for &(t, a, e) in &p.points {
                    assert!(g.az_ok(a), "b0 {b0} sweep {sweep} t {t}: az {a} outside 0..400");
                    assert!(e >= g.el_min - 1e-9 && e <= g.el_max + 1e-9, "el {e}");
                }
                //and interpolating between them cannot escape either
                let (t0, t1) = (p.t_start(), p.t_end());
                for k in 0..=100 {
                    let (a, _) = p.at(t0 + (t1 - t0) * k as f64 / 100.0);
                    assert!(g.az_ok(a), "interpolated az {a} outside the window");
                }
            }
        }
    }

    #[test]
    fn path_interpolates() {
        let p = MountPath { points: vec![(0.0, 10.0, 0.0), (10.0, 20.0, 10.0)], clipped: false, margin_deg: 0.0, max_az_rate: 1.0, max_el_rate: 1.0, unwind: None };
        let (a, e) = p.at(5.0);
        assert!((a - 15.0).abs() < 1e-9 && (e - 5.0).abs() < 1e-9);
        assert_eq!(p.at(-1.0), (10.0, 0.0));
        assert_eq!(p.at(99.0), (20.0, 10.0));
    }

    #[test]
    fn telemetry_parses() {
        let mut m = MountState::default();
        assert!(m.ingest("T 100.0 45.5 99.8 45.1 1 2048"));
        assert_eq!(m.cmd, Some((100.0, 45.5)));
        assert_eq!(m.fb, Some((99.8, 45.1)));
        assert!(m.moving);
        assert_eq!(m.encoder, Some(2048.0));
        assert_eq!(m.pose, None, "a legacy line carries no settled estimate");
        assert!(!m.ingest("OK GO"));
    }

    #[test]
    fn t4_telemetry_parses_into_its_three_claims() {
        //The firmware's own test vector (limits.rs, telemetry_carries_the_target_...)
        let mut m = MountState::default();
        assert!(m.ingest("T4 200.00 45.00 187.25 45.00 180.50 45.00 NA -1 KM"));
        let p = m.pose.unwrap();
        assert_eq!((p.tgt, p.cmd, p.settled), ((200.0, 45.0), (187.25, 45.0), (180.5, 45.0)));
        assert!(p.known && p.moving && p.clip == [false; 2]);
        assert_eq!((m.cmd, m.fb, m.moving, m.encoder), (Some((200.0, 45.0)), Some((180.5, 45.0)), true, None));
        assert_eq!(Pose::parse("T4 1 2 3 4 5 6 NA -1 KAE").map(|p| (p.moving, p.clip)), Some((false, [true, true])));
        assert_eq!(Pose::parse("T4 1 2 3 4 5 6 NA -1 -").map(|p| p.known), Some(false));
        //anything else is refused, not guessed at: wrong tag, wrong shape, junk flags, non-finite numbers
        for bad in ["T5 1 2 3 4 5 6 NA -1 K", "T4 1 2 3 4 5 6 NA K", "T4 1 2 3 4 5 6 NA -1 KX", "T4 1 2 nan 4 5 6 NA -1 K", "T4 1 2 3 4 5 6 NA -1 K extra"] {
            assert_eq!(Pose::parse(bad), None, "{bad}");
        }
        let mut m = MountState::default();
        assert!(!m.ingest("T5 1 2 3 4 5 6 NA -1 K") && m.last_seen.is_none(), "an unknown tag is not telemetry");
    }

    #[test]
    fn t4_on_point_can_fail() {
        let p = |line: &str| Pose::parse(line).unwrap();
        let asked = (200.0, 45.0);
        //settled far from the target: not there yet, even with the ramp finished and nothing moving
        assert!(!p("T4 200.00 45.00 200.00 45.00 180.50 45.00 NA -1 K").on_point(asked, 0.5));
        assert!(p("T4 200.00 45.00 200.00 45.00 199.70 44.80 NA -1 K").on_point(asked, 0.5));
        //each other condition on its own blocks it: moving, unanchored, clipped, a stale target
        assert!(!p("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 KM").on_point(asked, 0.5));
        assert!(!p("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 -").on_point(asked, 0.5));
        assert!(!p("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 KA").on_point(asked, 0.5));
        assert!(!p("T4 120.00 45.00 120.00 45.00 120.00 45.00 NA -1 K").on_point(asked, 0.5), "settled on the previous GO");
    }

    #[test]
    fn waiting_for_names_what_blocks_on_point() {
        let p = |line: &str| Pose::parse(line).unwrap();
        let asked = (200.0, 45.0);
        let why = |line: &str| p(line).waiting_for(asked, 0.5).unwrap_or_default();
        assert!(why("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 -").contains("position unknown"));
        assert!(why("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 KA").contains("azimuth limit"));
        assert!(why("T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 KE").contains("elevation limit"));
        assert!(why("T4 120.00 45.00 120.00 45.00 120.00 45.00 NA -1 K").contains("different target"));
        assert_eq!(why("T4 200.00 45.00 196.00 45.00 190.00 45.00 NA -1 KM"), "moving");
        assert_eq!(why("T4 200.00 45.00 200.00 45.00 196.80 45.00 NA -1 K"), "settling, 3.2 deg to go");
        //and it always agrees with on_point, which stays the one test the procedure uses
        for l in ["T4 200.00 45.00 200.00 45.00 199.70 44.80 NA -1 K", "T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 KM",
                  "T4 200.00 45.00 200.00 45.00 180.00 45.00 NA -1 K", "T4 200.00 45.00 200.00 45.00 200.00 45.00 NA -1 -"] {
            assert_eq!(p(l).waiting_for(asked, 0.5).is_none(), p(l).on_point(asked, 0.5), "{l}");
        }
    }

    #[test]
    fn host_and_firmware_agree_on_the_telemetry_tag() {
        let fw = include_str!("../firmware/perigee_mount_stm32/src/limits.rs");
        assert!(fw.contains(&format!("pub const TEL_TAG: &str = \"{TEL_TAG}\";")), "firmware TEL_TAG is not {TEL_TAG}");
    }

    #[test]
    fn sim_telemetry_round_trips_through_the_parser() {
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, az_lo: 0.0, az_hi: 400.0, el_min: -2.0, el_max: 91.0, el_corr: 0.0 };
        let mut s = crate::serial::MountSim::new(&g, 20.0, 15.0);
        s.handle("GO 100 30"); s.step(1.0);
        let mut m = MountState::default();
        assert!(m.ingest(&s.telemetry()), "{}", s.telemetry());
        let p = m.pose.unwrap();
        assert_eq!((p.tgt, p.cmd, p.settled), ((100.0, 30.0), (s.az, s.el), (s.az, s.el)));
        assert!(p.known && p.moving && !p.on_point((100.0, 30.0), 0.5));
        for _ in 0..20 { s.step(1.0); }
        let p = Pose::parse(&s.telemetry()).unwrap();
        assert!(!p.moving && p.on_point((100.0, 30.0), 0.5), "{}", s.telemetry());
        //a clamped request leaves the standing clip flag, so the sim can never claim a point it was refused
        s.handle("GO 401 30"); for _ in 0..30 { s.step(1.0); }
        let p = Pose::parse(&s.telemetry()).unwrap();
        assert!(p.clip == [true, false] && !p.on_point((401.0, 30.0), 0.5) && !p.on_point((400.0, 30.0), 0.5), "{}", s.telemetry());
        s.handle("GO 300 30");
        assert_eq!(Pose::parse(&s.telemetry()).unwrap().clip, [false; 2], "the next request that fits clears it");
    }
}

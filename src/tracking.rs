/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// From a picked satellite to axis commands: find its next pass over the station, sample the pass into
/// (time, bearing, elevation), hand that to the mount's path solver, and run the tracking sequence
///   IDLE -> ARMED (waiting for AOS, dish pre-positioned) -> TRACKING (GO az el at command_hz) -> DONE/PARK
/// on the real clock. The sky geometry is the viewer's (same look_angles), the orbit is the viewer's
/// propagated track for that column, so what you see on the globe is exactly what the dish follows.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::mount::{MountGeom, MountPath};
use nalgebra::Matrix6xX;
use perigee_viewer::config::Config as ViewerCfg;
use perigee_viewer::{eci_to_ecef, geodetic_to_ecef, gmst_rad, look_angles, sat_eci_km, sat_eci_vel};

/// Station in ECEF once, then bearing/elevation/range and range rate for any inertial state
pub struct Station { pub ecef: [f64; 3], pub lat: f64, pub lon: f64 }
impl Station {
    pub fn from_cfg(c: &ViewerCfg) -> Self {
        let s = &c.station;
        Station { ecef: geodetic_to_ecef(s.lat_deg, s.lon_deg, s.alt_m), lat: s.lat_deg, lon: s.lon_deg }
    }
    /// (bearing deg, elevation deg, range km) of an ECI position at Julian date jd
    pub fn look(&self, r_eci: [f64; 3], jd: f64) -> (f64, f64, f64) {
        look_angles(self.ecef, self.lat, self.lon, eci_to_ecef(r_eci, gmst_rad(jd)))
    }
    /// Range rate km/s (negative = approaching) from ECI position and velocity
    pub fn range_rate(&self, r_eci: [f64; 3], v_eci: [f64; 3], jd: f64) -> f64 {
        let g = gmst_rad(jd);
        let r = eci_to_ecef(r_eci, g);
        let v = eci_to_ecef(v_eci, g);
        let w = 7.2921159e-5;
        let vrel = [v[0] + w * r[1], v[1] - w * r[0], v[2]];
        let los = [r[0] - self.ecef[0], r[1] - self.ecef[1], r[2] - self.ecef[2]];
        let range = (los[0] * los[0] + los[1] * los[1] + los[2] * los[2]).sqrt();
        (vrel[0] * los[0] + vrel[1] * los[1] + vrel[2] * los[2]) / range
    }
}

/// One propagated track: the viewer's 6xN matrix for a column plus its epoch
pub struct Track<'a> { pub m: &'a Matrix6xX<f64>, pub epoch_jd: f64, pub cfg: &'a ViewerCfg }
impl Track<'_> {
    pub fn t_of(&self, jd: f64) -> f64 { (jd - self.epoch_jd) * 86400.0 }
    pub fn covers(&self, jd: f64) -> bool { let t = self.t_of(jd); t >= 0.0 && t <= (self.m.ncols().saturating_sub(1)) as f64 * self.cfg.data.step_seconds }
    pub fn end_jd(&self) -> f64 { self.epoch_jd + (self.m.ncols().saturating_sub(1)) as f64 * self.cfg.data.step_seconds / 86400.0 }
    pub fn r(&self, jd: f64) -> [f64; 3] { sat_eci_km(self.cfg, self.m, self.t_of(jd)) }
    pub fn v(&self, jd: f64) -> [f64; 3] { sat_eci_vel(self.cfg, self.m, self.t_of(jd)) }
    pub fn el(&self, sta: &Station, jd: f64) -> f64 { sta.look(self.r(jd), jd).1 }
}

#[derive(Clone, Debug)]
pub struct Pass { pub aos_jd: f64, pub los_jd: f64, pub max_el: f64, pub max_el_jd: f64 }

/// The first pass above `mask` degrees that ends after `from_jd`, searching up to `until_jd`.
/// Coarse scan one propagation step at a time (60 s), then the crossings are refined by bisection to
/// a tenth of a second. A pass already in progress at from_jd is returned with aos_jd = from_jd.
pub fn find_next_pass(track: &Track, sta: &Station, from_jd: f64, until_jd: f64, mask: f64) -> Option<Pass> {
    let step_d = track.cfg.data.step_seconds / 86400.0;
    let end = until_jd.min(track.end_jd());
    if !(from_jd < end) || !track.covers(from_jd) { return None; }
    let above = |jd: f64| track.el(sta, jd) >= mask;
    let bisect = |mut lo: f64, mut hi: f64, want_above_at_hi: bool| {
        for _ in 0..24 {
            let mid = 0.5 * (lo + hi);
            if above(mid) == want_above_at_hi { hi = mid } else { lo = mid }
        }
        0.5 * (lo + hi)
    };
    let mut jd = from_jd;
    let mut aos = if above(jd) { Some(jd) } else { None };
    let mut prev = jd;
    while jd < end {
        let next = (jd + step_d).min(end);
        let was = above(jd); let now = above(next);
        if aos.is_none() && !was && now { aos = Some(bisect(jd, next, true)); }
        if let Some(a) = aos {
            if was && !now {
                let los = bisect(jd, next, false);
                return Some(finish(track, sta, a, los));
            }
        }
        prev = jd; jd = next;
    }
    let _ = prev;
    aos.map(|a| finish(track, sta, a, end))   // still above the mask when the data ends
}

fn finish(track: &Track, sta: &Station, aos: f64, los: f64) -> Pass {
    //Peak elevation: sample every 5 s
    let n = (((los - aos) * 86400.0 / 5.0).ceil() as usize).max(2);
    let (mut max_el, mut max_el_jd) = (f64::MIN, aos);
    for i in 0..=n {
        let jd = aos + (los - aos) * i as f64 / n as f64;
        let e = track.el(sta, jd);
        if e > max_el { max_el = e; max_el_jd = jd; }
    }
    Pass { aos_jd: aos, los_jd: los, max_el, max_el_jd }
}

/// (t seconds from pass AOS, bearing, elevation) every `every_s` seconds, plus the endpoints
pub fn sample_pass(track: &Track, sta: &Station, pass: &Pass, every_s: f64) -> Vec<(f64, f64, f64)> {
    let dur = (pass.los_jd - pass.aos_jd) * 86400.0;
    let n = ((dur / every_s.max(0.1)).ceil() as usize).max(1);
    (0..=n).map(|i| {
        let t = (dur * i as f64 / n as f64).min(dur);
        let (b, e, _) = sta.look(track.r(pass.aos_jd + t / 86400.0), pass.aos_jd + t / 86400.0);
        (t, b, e)
    }).collect()
}

//------------------------------------------------------------------------------------------ sequence
#[derive(Clone, Debug, PartialEq)]
pub enum Phase { Idle, Armed, Tracking, Done }

#[derive(Clone, Debug)]
pub struct Plan { pub column: usize, pub name: String, pub pass: Pass, pub samples: Vec<(f64, f64, f64)>, pub path: MountPath }

#[derive(bevy::prelude::Resource)]
pub struct Tracker {
    pub phase: Phase,
    pub plan: Option<Plan>,
    pub prepositioned: bool,
    pub next_cmd: f64,          // app seconds of the next GO
    pub last_cmd: Option<(f64, f64)>,
    pub commands_sent: usize,
    pub message: String,        // one line for the LIVE tile
    pub done_at: Option<f64>,   // app seconds when the pass ended (DONE is shown briefly)
}
impl Default for Tracker {
    fn default() -> Self { Self { phase: Phase::Idle, plan: None, prepositioned: false, next_cmd: 0.0, last_cmd: None, commands_sent: 0, message: "pick a satellite in the viewer, then ARM".into(), done_at: None } }
}

pub enum Step { Send(String), Nothing }

impl Tracker {
    pub fn arm(&mut self, plan: Plan) {
        self.plan = Some(plan); self.phase = Phase::Armed; self.prepositioned = false; self.commands_sent = 0; self.last_cmd = None;
        self.message = "ARMED: waiting for AOS".into();
    }
    pub fn abort(&mut self, why: &str) -> Step {
        let was_active = matches!(self.phase, Phase::Armed | Phase::Tracking);
        self.phase = Phase::Idle; self.plan = None; self.prepositioned = false; self.message = format!("ABORTED: {why}");
        if was_active { Step::Send("STOP".into()) } else { Step::Nothing }
    }
    pub fn active(&self) -> bool { matches!(self.phase, Phase::Armed | Phase::Tracking) }

    /// Advance on the real clock. `now_jd` is the current Julian date, `app_s` the app's monotonic seconds.
    pub fn tick(&mut self, now_jd: f64, app_s: f64, geom: &MountGeom, cfg: &crate::config::TrackingCfg) -> Vec<String> {
        let mut out = Vec::new();
        let Some(plan) = &self.plan else { return out };
        let t = (now_jd - plan.pass.aos_jd) * 86400.0;   // seconds since AOS (negative before)
        let mut again = true;
        let mut clear = false;
        while std::mem::take(&mut again) { match self.phase {
            Phase::Armed => {
                if !self.prepositioned && t >= -cfg.preposition_min * 60.0 {
                    let (az, el) = plan.path.at(plan.path.t_start());
                    let (az, el) = geom.clamp(az, el);
                    out.push(format!("GO {az:.2} {el:.2}"));
                    self.prepositioned = true; self.last_cmd = Some((az, el)); self.commands_sent += 1;
                    self.message = "ARMED: dish at the AOS point".into();
                } else if !self.prepositioned {
                    self.message = format!("ARMED: pre-position in {}", fmt_countdown(-t - cfg.preposition_min * 60.0));
                }
                if t >= 0.0 { self.phase = Phase::Tracking; self.next_cmd = 0.0; self.message = "TRACKING".into(); again = true; }   // first GO this same tick
            }
            Phase::Tracking => {
                if t >= plan.path.t_end() {
                    self.phase = Phase::Done; self.done_at = Some(app_s);
                    self.message = if cfg.park_after { "LOS: pass complete, parking".into() } else { "LOS: pass complete".into() };
                    if cfg.park_after { out.push("PARK".into()); }
                } else if app_s >= self.next_cmd {
                    self.next_cmd = app_s + 1.0 / cfg.command_hz.max(0.5);
                    let (az, el) = plan.path.at(t + cfg.lead_seconds);
                    let (az, el) = geom.clamp(az, el);
                    out.push(format!("GO {az:.2} {el:.2}"));
                    self.last_cmd = Some((az, el)); self.commands_sent += 1;
                    self.message = format!("TRACKING: {} left", fmt_countdown(plan.path.t_end() - t));
                }
            }
            Phase::Done => { if self.done_at.map_or(true, |d| app_s - d > 8.0) { self.phase = Phase::Idle; clear = true; self.message = "idle".into(); } }
            Phase::Idle => {}
        } }
        if clear { self.plan = None; }
        out
    }
}

pub fn fmt_countdown(s: f64) -> String {
    let s = s.max(0.0).round() as i64;
    if s >= 3600 { format!("{}h{:02}m{:02}s", s / 3600, (s % 3600) / 60, s % 60) } else { format!("{:02}m{:02}s", s / 60, s % 60) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mount::Flip;

    fn plan() -> Plan {
        let pass = Pass { aos_jd: 2461000.0, los_jd: 2461000.0 + 600.0 / 86400.0, max_el: 40.0, max_el_jd: 2461000.0 + 300.0 / 86400.0 };
        let samples = vec![(0.0, 120.0, 0.0), (300.0, 180.0, 40.0), (600.0, 240.0, 0.0)];
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 };
        let path = g.solve_path(&samples).unwrap();
        assert_eq!(path.flip, Flip::Normal);
        Plan { column: 0, name: "TEST".into(), pass, samples, path }
    }

    #[test]
    fn sequence_runs_aos_to_park() {
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 };
        let cfg = crate::config::TrackingCfg { command_hz: 2.0, preposition_min: 3.0, lead_seconds: 0.0, park_after: true, ..Default::default() };
        let mut tr = Tracker::default();
        let p = plan(); let aos = p.pass.aos_jd;
        tr.arm(p);
        //10 minutes early: armed, nothing sent
        assert!(tr.tick(aos - 600.0 / 86400.0, 0.0, &g, &cfg).is_empty());
        //2 minutes early: pre-position to the AOS point (bearing 120 -> mount az 127.5)
        let cmds = tr.tick(aos - 120.0 / 86400.0, 1.0, &g, &cfg);
        assert_eq!(cmds, vec!["GO 127.50 0.00".to_string()]);
        assert!(tr.tick(aos - 60.0 / 86400.0, 2.0, &g, &cfg).is_empty());
        //AOS: tracking, one GO per tick spacing
        let c1 = tr.tick(aos + 1.0 / 86400.0, 3.0, &g, &cfg);
        assert_eq!(tr.phase, Phase::Tracking);
        assert_eq!(c1.len(), 1);
        assert!(tr.tick(aos + 1.2 / 86400.0, 3.1, &g, &cfg).is_empty());   // too soon for the next command
        let c2 = tr.tick(aos + 300.0 / 86400.0, 4.0, &g, &cfg);
        assert_eq!(c2, vec!["GO 187.50 40.00".to_string()]);              // peak: bearing 180 -> 187.5
        //LOS: park
        let c3 = tr.tick(aos + 601.0 / 86400.0, 5.0, &g, &cfg);
        assert_eq!(c3, vec!["PARK".to_string()]);
        assert_eq!(tr.phase, Phase::Done);
        tr.tick(aos + 700.0 / 86400.0, 20.0, &g, &cfg);
        assert_eq!(tr.phase, Phase::Idle);
    }

    #[test]
    fn abort_stops_the_mount() {
        let mut tr = Tracker::default();
        assert!(matches!(tr.abort("test"), Step::Nothing));
        tr.arm(plan());
        assert!(matches!(tr.abort("test"), Step::Send(s) if s == "STOP"));
        assert_eq!(tr.phase, Phase::Idle);
    }

    #[test]
    fn countdown_format() {
        assert_eq!(fmt_countdown(59.4), "00m59s");
        assert_eq!(fmt_countdown(3661.0), "1h01m01s");
        assert_eq!(fmt_countdown(-5.0), "00m00s");
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// From a picked satellite to axis commands: find its next pass over the station, sample the pass into
/// (time, bearing, elevation), hand that to the mount's path solver, and run the tracking sequence
///   IDLE -> CHECK -> SLEW -> ARMED (waiting for AOS) -> TRACKING (GO az el at command_hz) -> PARKING -> DONE
/// on the real clock, or on the viewer's clock when the mount is the simulator. The sky geometry is the viewer's (same look_angles), the orbit is the viewer's
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
/// The procedure, as it runs on the real mount or on the simulator:
///   IDLE -> CHECK (target, link, ephemeris, pass, path: one step at a time so they can be read)
///        -> SLEW (GO to the AOS point, wait until the mount reports it is on point)
///        -> ARMED (waiting for AOS) -> TRACKING (GO az el at command_hz) -> PARKING -> DONE -> IDLE
#[derive(Clone, Debug, PartialEq)]
pub enum Phase { Idle, Check, Slew, Armed, Tracking, Parking, Done }
impl Phase {
    pub fn label(&self) -> &'static str {
        match self { Phase::Idle => "IDLE", Phase::Check => "CHECK", Phase::Slew => "SLEW", Phase::Armed => "ARMED", Phase::Tracking => "TRACKING", Phase::Parking => "PARKING", Phase::Done => "DONE" }
    }
}

#[derive(Clone, Debug)]
pub struct Plan { pub column: usize, pub name: String, pub pass: Pass, pub samples: Vec<(f64, f64, f64)>, pub path: MountPath }

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepState { Pending, Active, Done, Warn, Failed }

/// One line of the procedure checklist shown in LIVE DATA
#[derive(Clone, Debug)]
pub struct ProcStep { pub label: &'static str, pub detail: String, pub state: StepState }
impl ProcStep {
    pub fn mark(&self) -> &'static str {
        match self.state { StepState::Pending => "[ ]", StepState::Active => "[>]", StepState::Done => "[x]", StepState::Warn => "[!]", StepState::Failed => "[X]" }
    }
}

pub const STEP_LABELS: [&str; 9] = ["TARGET", "LINK", "EPHEMERIS", "PASS", "PATH", "SLEW", "AOS", "TRACK", "PARK"];
const S_TARGET: usize = 0; const S_LINK: usize = 1; const S_EPHEM: usize = 2; const S_PASS: usize = 3; const S_PATH: usize = 4;
const S_SLEW: usize = 5; const S_AOS: usize = 6; const S_TRACK: usize = 7; const S_PARK: usize = 8;

/// What the sequence needs to know each tick, gathered by the control page
pub struct Inputs<'a> {
    pub now_jd: f64,                    // the clock the dish follows (real, or the viewer's when simulating)
    pub app_s: f64,                     // app monotonic seconds
    pub time_scale: f64,                // clock seconds per app second (1 on the real clock)
    pub link_open: bool, pub link_name: String, pub firmware_alive: bool,
    pub have_data: bool, pub data_note: String,
    pub preview: Option<&'a Plan>, pub preview_note: String,
    pub fb: Option<(f64, f64)>, pub moving: bool,
    pub az_rate_limit: f64, pub el_rate_limit: f64,
}

#[derive(bevy::prelude::Resource)]
pub struct Tracker {
    pub phase: Phase,
    pub plan: Option<Plan>,
    pub target: Option<(usize, String)>,   // column, name the procedure was started for
    pub steps: Vec<ProcStep>,
    pub step_at: f64,           // app seconds the current CHECK step was entered
    pub next_cmd: f64,          // app seconds of the next GO
    pub last_cmd: Option<(f64, f64)>,
    pub commands_sent: usize,
    pub message: String,        // one line for the LIVE tile
    pub done_at: Option<f64>,   // app seconds when the pass ended (DONE is shown briefly)
    pub phase_at: f64,          // app seconds the current phase was entered
    pub events: Vec<String>,    // notes for the console, drained by the page
}
impl Default for Tracker {
    fn default() -> Self {
        Self { phase: Phase::Idle, plan: None, target: None, steps: fresh_steps(), step_at: 0.0, next_cmd: 0.0, last_cmd: None, commands_sent: 0,
               message: "pick a satellite in ORBIT VIEW".into(), done_at: None, phase_at: 0.0, events: Vec::new() }
    }
}

fn fresh_steps() -> Vec<ProcStep> { STEP_LABELS.iter().map(|l| ProcStep { label: l, detail: String::new(), state: StepState::Pending }).collect() }

pub enum Step { Send(String), Nothing }

impl Tracker {
    /// Begin the procedure for a picked satellite: the checks run one per `step_seconds` from the next tick
    pub fn start(&mut self, column: usize, name: &str, app_s: f64) {
        self.plan = None; self.target = Some((column, name.to_string())); self.steps = fresh_steps();
        self.phase = Phase::Check; self.phase_at = app_s; self.step_at = app_s; self.commands_sent = 0; self.last_cmd = None; self.done_at = None;
        self.set_step(S_TARGET, StepState::Active, name.to_string());
        self.message = format!("procedure started for {name}");
        self.events.push(format!("PROCEDURE {name}: start"));
    }
    pub fn abort(&mut self, why: &str) -> Step {
        let was_active = self.active();
        if let Some(i) = self.steps.iter().position(|s| s.state == StepState::Active) { self.set_step(i, StepState::Failed, format!("aborted: {why}")); }
        self.phase = Phase::Idle; self.plan = None; self.message = format!("ABORTED: {why}");
        if was_active { self.events.push(format!("PROCEDURE aborted: {why}")); Step::Send("STOP".into()) } else { Step::Nothing }
    }
    pub fn active(&self) -> bool { matches!(self.phase, Phase::Check | Phase::Slew | Phase::Armed | Phase::Tracking | Phase::Parking) }
    fn set_step(&mut self, i: usize, state: StepState, detail: String) {
        if let Some(s) = self.steps.get_mut(i) { s.state = state; s.detail = detail; }
    }
    fn step_done(&mut self, i: usize, state: StepState, detail: String) {
        let d = detail.clone();
        self.set_step(i, state, detail);
        self.events.push(format!("{} {}: {}", self.steps[i].mark(), self.steps[i].label, d));
    }
    fn fail(&mut self, i: usize, detail: String) {
        self.step_done(i, StepState::Failed, detail.clone());
        self.phase = Phase::Idle; self.plan = None; self.message = format!("{} failed: {}", STEP_LABELS[i], detail);
    }
    fn enter(&mut self, phase: Phase, app_s: f64, msg: &str) {
        self.events.push(format!("sequence: {} -> {}", self.phase.label(), phase.label()));
        self.phase = phase; self.phase_at = app_s; self.message = msg.into();
    }

    /// Advance the procedure. Returns the lines to send to the mount.
    pub fn tick(&mut self, inp: &Inputs, geom: &MountGeom, cfg: &crate::config::TrackingCfg) -> Vec<String> {
        let mut out = Vec::new();
        let app_s = inp.app_s;
        //The checks: one per step_seconds so the list can be read as it fills in
        if self.phase == Phase::Check {
            if app_s - self.step_at < cfg.step_seconds.max(0.0) { return out; }
            self.step_at = app_s;
            let i = self.steps.iter().position(|s| s.state == StepState::Active).unwrap_or(S_TARGET);
            match i {
                S_TARGET => { let d = self.steps[i].detail.clone(); self.step_done(i, StepState::Done, d); self.set_step(S_LINK, StepState::Active, String::new()); }
                S_LINK => {
                    if !inp.link_open { self.fail(i, "link closed: CONNECT or SIM".into()); return out; }
                    let d = format!("{}{}", inp.link_name, if inp.firmware_alive { ", telemetry OK" } else { ", no telemetry yet" });
                    self.step_done(i, if inp.firmware_alive { StepState::Done } else { StepState::Warn }, d); self.set_step(S_EPHEM, StepState::Active, String::new());
                }
                S_EPHEM => {
                    if !inp.have_data { self.fail(i, inp.data_note.clone()); return out; }
                    self.step_done(i, StepState::Done, format!("propagated track covers {}", crate::command::jd_utc_full(inp.now_jd))); self.set_step(S_PASS, StepState::Active, String::new());
                }
                S_PASS => {
                    let Some(p) = inp.preview else { self.fail(i, inp.preview_note.clone()); return out; };
                    let until = (p.pass.aos_jd - inp.now_jd) * 86400.0;
                    let d = format!("AOS {} ({})  LOS {}  max el {:.0}  {:.1} min", crate::command::jd_local(p.pass.aos_jd),
                        if until > 1.0 { format!("in {}", fmt_countdown(until)) } else { "in progress".into() }, crate::command::jd_local(p.pass.los_jd), p.pass.max_el, (p.pass.los_jd - p.pass.aos_jd) * 1440.0);
                    self.step_done(i, StepState::Done, d); self.set_step(S_PATH, StepState::Active, String::new());
                }
                S_PATH => {
                    let Some(p) = inp.preview else { self.fail(i, "pass gone".into()); return out; };
                    let (a0, a1) = p.path.points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), q| (lo.min(q.1), hi.max(q.1)));
                    let too_fast = p.path.max_az_rate > inp.az_rate_limit || p.path.max_el_rate > inp.el_rate_limit;
                    let d = format!("{}{}  az {:.0}..{:.0}  margin {:.0}  peak rate az {:.2} el {:.2} deg/s{}{}", p.path.flip.name(), if p.path.flips_mid > 0 { "+FLIP" } else { "" },
                        a0, a1, p.path.margin_deg, p.path.max_az_rate, p.path.max_el_rate, if p.path.clipped { "  CLIPPED at the travel limit" } else { "" }, if too_fast { "  exceeds the slew rate" } else { "" });
                    self.step_done(i, if p.path.clipped || too_fast { StepState::Warn } else { StepState::Done }, d);
                    self.plan = Some(p.clone());
                    //Pre-position now: GO to the AOS point, then wait for the mount to report it is there
                    let (az, el) = geom.clamp(p.path.at(p.path.t_start()).0, p.path.at(p.path.t_start()).1);
                    out.push(format!("GO {az:.2} {el:.2}"));
                    self.last_cmd = Some((az, el)); self.commands_sent += 1;
                    let (b, e) = geom.sky_of(az, el);
                    self.set_step(S_SLEW, StepState::Active, format!("to the AOS point: mount {az:.1} / {el:.1} (bearing {b:.0} el {e:.0})"));
                    self.enter(Phase::Slew, app_s, "SLEW: moving to the AOS point");
                }
                _ => {}
            }
            return out;
        }
        let Some(plan) = self.plan.clone() else { if self.phase != Phase::Done { return out; } else { self.phase = Phase::Idle; return out; } };
        let t = (inp.now_jd - plan.pass.aos_jd) * 86400.0;   // clock seconds since AOS (negative before)
        match self.phase {
            Phase::Slew => {
                let on_point = match (inp.fb, self.last_cmd) {
                    (Some(f), Some(c)) => !inp.moving && (f.0 - c.0).abs() < cfg.on_point_deg && (f.1 - c.1).abs() < cfg.on_point_deg,
                    (None, _) => app_s - self.phase_at > 2.0,      // no feedback: assume it got there
                    _ => false,
                };
                if on_point || t >= 0.0 {
                    let d = if on_point { format!("on point after {:.1} s", app_s - self.phase_at) } else { "AOS reached before the mount was on point".into() };
                    self.step_done(S_SLEW, if on_point { StepState::Done } else { StepState::Warn }, d);
                    self.set_step(S_AOS, StepState::Active, String::new());
                    self.enter(Phase::Armed, app_s, "ARMED: on point, waiting for AOS");
                } else if let Some(f) = inp.fb {
                    self.message = format!("SLEW: mount {:.1} / {:.1}", f.0, f.1);
                }
            }
            Phase::Armed => {
                if t >= 0.0 {
                    self.step_done(S_AOS, StepState::Done, format!("AOS {}", crate::command::jd_local(plan.pass.aos_jd)));
                    self.set_step(S_TRACK, StepState::Active, String::new());
                    self.next_cmd = 0.0;
                    self.enter(Phase::Tracking, app_s, "TRACKING");
                } else {
                    self.message = format!("ARMED: AOS in {}", fmt_countdown(-t));
                    self.set_step(S_AOS, StepState::Active, format!("in {}", fmt_countdown(-t)));
                }
            }
            Phase::Tracking => {
                if t >= plan.path.t_end() {
                    self.step_done(S_TRACK, StepState::Done, format!("LOS {}  {} commands", crate::command::jd_local(plan.pass.los_jd), self.commands_sent));
                    if cfg.park_after {
                        out.push("PARK".into());
                        self.set_step(S_PARK, StepState::Active, "parking".into());
                        self.enter(Phase::Parking, app_s, "LOS: pass complete, parking");
                    } else {
                        self.step_done(S_PARK, StepState::Done, "holding (park_after = false)".into());
                        self.done_at = Some(app_s); self.enter(Phase::Done, app_s, "LOS: pass complete");
                    }
                } else if app_s >= self.next_cmd {
                    //command_hz is in clock seconds: on a sped-up clock commands come faster, at most once a frame
                    self.next_cmd = app_s + (1.0 / cfg.command_hz.max(0.5) / inp.time_scale.max(1e-3)).max(1.0 / 60.0);
                    let (az, el) = plan.path.at(t + cfg.lead_seconds);
                    let (az, el) = geom.clamp(az, el);
                    out.push(format!("GO {az:.2} {el:.2}"));
                    self.last_cmd = Some((az, el)); self.commands_sent += 1;
                    self.message = format!("TRACKING: {} left", fmt_countdown(plan.path.t_end() - t));
                    self.set_step(S_TRACK, StepState::Active, format!("{} left, {} commands", fmt_countdown(plan.path.t_end() - t), self.commands_sent));
                }
            }
            Phase::Parking => {
                let parked = inp.fb.map_or(app_s - self.phase_at > 2.0, |_| !inp.moving && app_s - self.phase_at > 0.5);
                if parked {
                    self.step_done(S_PARK, StepState::Done, "parked".into());
                    self.done_at = Some(app_s); self.enter(Phase::Done, app_s, "SEQUENCE COMPLETE: dish parked");
                }
            }
            Phase::Done => { if self.done_at.map_or(true, |d| app_s - d > 8.0) { self.phase = Phase::Idle; self.plan = None; self.message = "idle".into(); } }
            Phase::Idle | Phase::Check => {}
        }
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
    fn inputs<'a>(now_jd: f64, app_s: f64, p: Option<&'a Plan>, fb: Option<(f64, f64)>, moving: bool) -> Inputs<'a> {
        Inputs { now_jd, app_s, time_scale: 1.0, link_open: true, link_name: "SIMULATOR".into(), firmware_alive: true, have_data: true, data_note: String::new(),
                 preview: p, preview_note: String::new(), fb, moving, az_rate_limit: 20.0, el_rate_limit: 15.0 }
    }

    #[test]
    fn procedure_runs_checks_slew_aos_track_park() {
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 };
        let cfg = crate::config::TrackingCfg { command_hz: 2.0, lead_seconds: 0.0, park_after: true, step_seconds: 0.0, on_point_deg: 0.5, ..Default::default() };
        let mut tr = Tracker::default();
        let p = plan(); let aos = p.pass.aos_jd;
        tr.start(0, "TEST", 0.0);
        assert_eq!(tr.phase, Phase::Check);
        //Five checks (target, link, ephemeris, pass, path), one per tick; the last one sends the pre-position GO
        let mut cmds = Vec::new();
        for k in 0..5 { cmds.extend(tr.tick(&inputs(aos - 600.0 / 86400.0, 1.0 + k as f64, Some(&p), Some((225.0, 45.0)), false), &g, &cfg)); }
        assert_eq!(cmds, vec!["GO 127.50 0.00".to_string()]);    // bearing 120 -> mount az 127.5
        assert_eq!(tr.phase, Phase::Slew);
        assert!(tr.steps[..5].iter().all(|s| s.state == StepState::Done), "{:?}", tr.steps);
        //Still moving: stays in SLEW; on point and stopped: ARMED
        assert!(tr.tick(&inputs(aos - 500.0 / 86400.0, 7.0, Some(&p), Some((100.0, 20.0)), true), &g, &cfg).is_empty());
        assert_eq!(tr.phase, Phase::Slew);
        tr.tick(&inputs(aos - 400.0 / 86400.0, 8.0, Some(&p), Some((127.5, 0.0)), false), &g, &cfg);
        assert_eq!(tr.phase, Phase::Armed);
        assert!(tr.tick(&inputs(aos - 60.0 / 86400.0, 9.0, Some(&p), Some((127.5, 0.0)), false), &g, &cfg).is_empty());
        //AOS: tracking, one GO per command interval
        tr.tick(&inputs(aos + 1.0 / 86400.0, 10.0, Some(&p), Some((127.5, 0.0)), false), &g, &cfg);
        assert_eq!(tr.phase, Phase::Tracking);
        let c1 = tr.tick(&inputs(aos + 1.0 / 86400.0, 10.0, Some(&p), Some((127.5, 0.0)), false), &g, &cfg);
        assert_eq!(c1.len(), 1);
        assert!(tr.tick(&inputs(aos + 1.2 / 86400.0, 10.1, Some(&p), Some((127.5, 0.0)), false), &g, &cfg).is_empty());   // too soon
        let c2 = tr.tick(&inputs(aos + 300.0 / 86400.0, 11.0, Some(&p), Some((150.0, 20.0)), true), &g, &cfg);
        assert_eq!(c2, vec!["GO 187.50 40.00".to_string()]);      // peak: bearing 180 -> 187.5
        //LOS: park, then DONE once the mount stops, then IDLE
        let c3 = tr.tick(&inputs(aos + 601.0 / 86400.0, 12.0, Some(&p), Some((240.0, 0.0)), true), &g, &cfg);
        assert_eq!(c3, vec!["PARK".to_string()]);
        assert_eq!(tr.phase, Phase::Parking);
        tr.tick(&inputs(aos + 605.0 / 86400.0, 13.0, Some(&p), Some((225.0, 45.0)), false), &g, &cfg);
        assert_eq!(tr.phase, Phase::Done);
        assert!(tr.steps.iter().all(|s| s.state == StepState::Done), "{:?}", tr.steps);
        tr.tick(&inputs(aos + 700.0 / 86400.0, 30.0, Some(&p), Some((225.0, 45.0)), false), &g, &cfg);
        assert_eq!(tr.phase, Phase::Idle);
    }

    #[test]
    fn checks_fail_cleanly() {
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 };
        let cfg = crate::config::TrackingCfg { step_seconds: 0.0, ..Default::default() };
        let mut tr = Tracker::default();
        tr.start(3, "X", 0.0);
        let mut i = inputs(2461000.0, 1.0, None, None, false); i.link_open = false;
        tr.tick(&i, &g, &cfg); tr.tick(&i, &g, &cfg);
        assert_eq!(tr.phase, Phase::Idle);
        assert_eq!(tr.steps[1].state, StepState::Failed);
        assert!(tr.message.contains("LINK failed"));
    }

    #[test]
    fn faster_clock_commands_faster() {
        let g = MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 };
        let cfg = crate::config::TrackingCfg { command_hz: 4.0, step_seconds: 0.0, ..Default::default() };
        let mut tr = Tracker::default();
        let p = plan(); let aos = p.pass.aos_jd;
        tr.plan = Some(p.clone()); tr.phase = Phase::Tracking; tr.next_cmd = 0.0;
        let mut i = inputs(aos + 10.0 / 86400.0, 100.0, Some(&p), None, false); i.time_scale = 20.0;
        tr.tick(&i, &g, &cfg);
        assert!((tr.next_cmd - 100.0 - 1.0 / 60.0).abs() < 1e-9, "{}", tr.next_cmd);   // 0.25 s / 20 = 12.5 ms, floored to one frame
    }

    #[test]
    fn abort_stops_the_mount() {
        let mut tr = Tracker::default();
        assert!(matches!(tr.abort("test"), Step::Nothing));
        tr.start(0, "TEST", 0.0);
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

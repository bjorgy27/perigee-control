/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The mount: its frame, its limits, the path solver that turns a sky track into axis commands, and the
/// wireframe drawing of the gimbal for the MOUNT tile.
///
/// Frames.  The sky uses true bearing (0 north, clockwise) and elevation above the level horizon, the
/// same frame the viewer and Perigee's ranking use.  The mount uses its own two axis readings:
///   az_m  0 .. az_travel        (450 on the Stingray-4)   bearing = az_zero + az_m,  az_zero = center - travel/2
///   el_m  el_min .. el_max      (-5 .. 185 on the Stingray-9)
/// One calibration number ties them together: az_center_bearing, the bearing the dish faces at mid travel.
///
/// Flip-over.  Because elevation can go past 90, a sky direction (az, el) has two mount solutions:
///   normal    az_m from az,        el_m = el
///   flipped   az_m from az + 180,  el_m = 180 - el
/// A pass that climbs near the zenith makes the azimuth axis spin through 180 degrees in a few seconds;
/// tracked flipped, that same pass is a slow elevation sweep from -5 towards 185.  The solver tries both.
///
/// Wrap.  450 degrees of travel means two bearings (90 degrees' worth) can be reached at two axis positions
/// 360 apart, and the remaining gap of 270 cannot be crossed.  The solver unwraps the pass into a continuous
/// azimuth run and then looks for a whole-turn offset that keeps the run inside 0..travel.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::config::MountCfg;
use bevy::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct MountGeom { pub az_travel: f64, pub az_center: f64, pub el_min: f64, pub el_max: f64 }

impl MountGeom {
    pub fn from_cfg(c: &MountCfg) -> Self {
        Self { az_travel: c.az_travel_deg, az_center: c.az_center_bearing_deg, el_min: c.el_min_deg, el_max: c.el_max_deg }
    }
    /// True bearing of mount azimuth zero
    pub fn az_zero(&self) -> f64 { (self.az_center - self.az_travel / 2.0).rem_euclid(360.0) }
    /// Mount azimuth -> true bearing
    pub fn bearing(&self, az_m: f64) -> f64 { (self.az_zero() + az_m).rem_euclid(360.0) }
    /// Every mount azimuth that faces this bearing (one or two values inside the travel)
    pub fn az_candidates(&self, bearing: f64) -> Vec<f64> {
        let base = (bearing - self.az_zero()).rem_euclid(360.0);
        let mut out = Vec::new();
        let mut a = base;
        while a <= self.az_travel + 1e-9 { out.push(a); a += 360.0; }
        out
    }
    /// The mount pose nearest to `from` that looks at (bearing, el): tries normal and flipped
    pub fn nearest_pose(&self, bearing: f64, el: f64, from: (f64, f64)) -> Option<(f64, f64, Flip)> {
        let mut best: Option<(f64, f64, Flip, f64)> = None;
        for flip in [Flip::Normal, Flip::Flipped] {
            let (b, e) = flip.apply(bearing, el);
            if e < self.el_min - 1e-9 || e > self.el_max + 1e-9 { continue; }
            for a in self.az_candidates(b) {
                let cost = (a - from.0).abs() + (e - from.1).abs();
                if best.map_or(true, |(_, _, _, c)| cost < c) { best = Some((a, e, flip, cost)); }
            }
        }
        best.map(|(a, e, f, _)| (a, e, f))
    }
    pub fn clamp(&self, az_m: f64, el_m: f64) -> (f64, f64) {
        (az_m.clamp(0.0, self.az_travel), el_m.clamp(self.el_min, self.el_max))
    }
    /// Sky direction the mount is looking at for a mount pose (bearing, elevation)
    pub fn sky_of(&self, az_m: f64, el_m: f64) -> (f64, f64) {
        if el_m > 90.0 { ((self.bearing(az_m) + 180.0).rem_euclid(360.0), 180.0 - el_m) } else { (self.bearing(az_m), el_m) }
    }

    /// Solve a whole pass. `samples` are (t seconds, bearing deg, elevation deg) in time order.
    ///
    /// Every sample has two mount representations (normal / flipped). Starting from each of them, the
    /// path is built greedily: at each step the representation nearest to the previous pose (azimuth
    /// unwrapped by whole turns, elevation free to run past 90) is taken, so a pass through the zenith
    /// becomes one smooth elevation sweep instead of a 180 degree azimuth spin. Then a whole-turn offset
    /// is chosen that keeps the azimuth run inside 0..travel. The better of the two starts wins.
    pub fn solve_path(&self, samples: &[(f64, f64, f64)]) -> Option<MountPath> {
        if samples.len() < 2 { return None; }
        let mut best: Option<MountPath> = None;
        for start in [Flip::Normal, Flip::Flipped] {
            let mut seq: Vec<(f64, f64, Flip)> = Vec::with_capacity(samples.len());   // (az from mount zero, el, representation)
            for (i, &(_, b, e)) in samples.iter().enumerate() {
                let cands = [Flip::Normal, Flip::Flipped].map(|f| { let (bb, ee) = f.apply(b, e); ((bb - self.az_zero()).rem_euclid(360.0), ee, f) });
                let pick = if i == 0 {
                    cands[if start == Flip::Normal { 0 } else { 1 }]
                } else {
                    let (pa, pe, _) = seq[i - 1];
                    let mut best_c = cands[0]; let mut best_cost = f64::MAX;
                    for (a, ee, f) in cands {
                        let a = nearest_turn(a, pa);
                        let cost = (a - pa).abs() + (ee - pe).abs();
                        if cost < best_cost { best_cost = cost; best_c = (a, ee, f); }
                    }
                    best_c
                };
                seq.push(pick);
            }
            let el_out = seq.iter().filter(|&&(_, e, _)| e < self.el_min || e > self.el_max).count();
            let lo = seq.iter().map(|s| s.0).fold(f64::MAX, f64::min);
            let hi = seq.iter().map(|s| s.0).fold(f64::MIN, f64::max);
            //Whole-turn offset: the one that leaves the most room on both sides. If the run is longer than the
            //travel no offset fits; keep the one that clips the least time and mark the path CLIPPED.
            let mut pick: Option<(f64, f64, usize)> = None;   // (k*360, margin, samples inside)
            for k in -3..=3 {
                let off = k as f64 * 360.0;
                let margin = (lo + off).min(self.az_travel - (hi + off));
                let inside = seq.iter().filter(|&&(a, _, _)| a + off >= 0.0 && a + off <= self.az_travel).count();
                if pick.map_or(true, |(_, m, n)| (inside, margin) > (n, m)) { pick = Some((off, margin, inside)); }
            }
            let (off, margin, inside) = pick?;
            let points: Vec<(f64, f64, f64)> = samples.iter().zip(&seq)
                .map(|(&(t, _, _), &(a, e, _))| { let (az, el) = self.clamp(a + off, e); (t, az, el) }).collect();
            let (mut max_az_rate, mut max_el_rate) = (0.0f64, 0.0f64);
            for w in points.windows(2) {
                let dt = (w[1].0 - w[0].0).max(1e-6);
                max_az_rate = max_az_rate.max(((w[1].1 - w[0].1) / dt).abs());
                max_el_rate = max_el_rate.max(((w[1].2 - w[0].2) / dt).abs());
            }
            let flips_mid = seq.windows(2).filter(|w| w[0].2 != w[1].2).count();
            let cand = MountPath { flip: start, flips_mid, points, clipped: inside < seq.len() || el_out > 0, margin_deg: margin, max_az_rate, max_el_rate };
            //Prefer: not clipped, then the gentler azimuth axis (that is what the flip is for), then margin
            let better = match &best {
                None => true,
                Some(b) => (!cand.clipped, -cand.max_az_rate, cand.margin_deg) > (!b.clipped, -b.max_az_rate, b.margin_deg),
            };
            if better { best = Some(cand); }
        }
        best
    }
}

/// `a` moved by whole turns to sit within half a turn of `prev`
pub fn nearest_turn(a: f64, prev: f64) -> f64 {
    let mut v = a;
    while v - prev > 180.0 { v -= 360.0; }
    while v - prev < -180.0 { v += 360.0; }
    v
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Flip { Normal, Flipped }
impl Flip {
    /// Sky (bearing, el) -> the (bearing, el) the mount frame is asked for under this solution
    pub fn apply(self, bearing: f64, el: f64) -> (f64, f64) {
        match self { Flip::Normal => (bearing.rem_euclid(360.0), el), Flip::Flipped => ((bearing + 180.0).rem_euclid(360.0), 180.0 - el) }
    }
    pub fn name(self) -> &'static str { match self { Flip::Normal => "NORMAL", Flip::Flipped => "FLIPPED" } }
}

#[derive(Clone, Debug)]
pub struct MountPath {
    pub flip: Flip,                     // representation at the start of the pass
    pub flips_mid: usize,               // how many times the representation changes along the pass (zenith crossings)
    pub points: Vec<(f64, f64, f64)>,   // (t seconds, az_m, el_m)
    pub clipped: bool,                  // part of the pass is outside the travel (held at the limit there)
    pub margin_deg: f64,                // room to the nearest azimuth limit
    pub max_az_rate: f64,               // deg/s
    pub max_el_rate: f64,
}

impl MountPath {
    pub fn t_start(&self) -> f64 { self.points.first().map_or(0.0, |p| p.0) }
    pub fn t_end(&self) -> f64 { self.points.last().map_or(0.0, |p| p.0) }
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
/// What the firmware last reported (parsed from its "T ..." telemetry lines)
#[derive(Resource, Default, Debug, Clone)]
pub struct MountState {
    pub cmd: Option<(f64, f64)>,      // mount-frame target the firmware is moving to
    pub fb: Option<(f64, f64)>,       // mount-frame position it measures
    pub moving: bool,
    pub encoder: Option<f64>,         // AS5600 raw reading on the elevation axis, if the firmware sends it
    pub last_seen: Option<std::time::Instant>,
    pub firmware: String,
}

impl MountState {
    /// Parse one line from the mount. Returns true when it was telemetry.
    pub fn ingest(&mut self, line: &str) -> bool {
        let mut it = line.split_whitespace();
        match it.next() {
            Some("T") => {
                let v: Vec<f64> = it.filter_map(|s| s.parse().ok()).collect();
                if v.len() >= 4 {
                    self.cmd = Some((v[0], v[1]));
                    self.fb = Some((v[2], v[3]));
                    self.moving = v.get(4).map_or(false, |m| *m != 0.0);
                    self.encoder = v.get(5).copied();
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

/// Draw the gimbal into a screen rectangle (UI pixels, y down).  `fb` is the measured pose, `cmd` the pose
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
    //Azimuth travel limits on the ground: a radial at mount az 0 and one at az_travel. When the travel is
    //under a full turn the bearings between them cannot be reached at all: a dim arc marks that gap.
    {
        let r0 = dims::PEDESTAL_R + 25.0; let r1 = dims::PEDESTAL_R + 60.0;
        for b in [geom.az_zero(), geom.az_zero() + geom.az_travel] {
            let a = (b as f32).to_radians();
            seg(lines, Vec3::new(r0 * a.sin(), r0 * a.cos(), 0.0), Vec3::new(r1 * a.sin(), r1 * a.cos(), 0.0), colors.gap);
        }
        let gap = (360.0 - geom.az_travel) as f32;
        if gap > 0.5 {
            let start = (geom.az_zero() as f32 + geom.az_travel as f32).to_radians();
            let n = 16; let r = dims::PEDESTAL_R + 35.0;
            for i in 0..n {
                let a0 = start + gap.to_radians() * i as f32 / n as f32;
                let a1 = start + gap.to_radians() * (i + 1) as f32 / n as f32;
                seg(lines, Vec3::new(r * a0.sin(), r * a0.cos(), 0.0), Vec3::new(r * a1.sin(), r * a1.cos(), 0.0), colors.gap);
            }
        }
    }

    let pose = fb.or(cmd).unwrap_or((geom.az_travel / 2.0, 0.0));
    let bearing = geom.bearing(pose.0) as f32;
    let el = pose.1 as f32;
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
    //Rays from the elevation axis: measured boresight (bright), commanded (dim), sky target (accent)
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
    fn geom() -> MountGeom { MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 } }

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

    #[test]
    fn nearest_pose_prefers_the_short_move() {
        let g = geom();
        //Bearing 30 is reachable at az 37.5 and 397.5: from az 400 the second is nearer
        let (a, e, f) = g.nearest_pose(30.0, 20.0, (400.0, 20.0)).unwrap();
        assert!((a - 397.5).abs() < 1e-9 && e == 20.0 && f == Flip::Normal);
        //From a flipped pose high up, staying flipped is the short move
        let (_, e, f) = g.nearest_pose(217.5, 80.0, (225.0 + 180.0, 120.0)).unwrap();
        assert!(f == Flip::Flipped && (e - 100.0).abs() < 1e-9);
    }

    #[test]
    fn south_pass_is_normal() {
        //Rising in the south-east, setting in the south-west, peak 40 deg: no wrap, no flip needed
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=60).map(|i| { let f = i as f64 / 60.0; (f * 600.0, 120.0 + f * 120.0, 40.0 * (std::f64::consts::PI * f).sin()) }).collect();
        let p = g.solve_path(&s).unwrap();
        assert_eq!(p.flip, Flip::Normal);
        assert!(!p.clipped);
        assert!(p.points.iter().all(|&(_, a, e)| (0.0..=450.0).contains(&a) && (-5.0..=185.0).contains(&e)));
    }

    #[test]
    fn zenith_pass_flips() {
        //Straight overhead south -> north: azimuth jumps 180 at the peak; the flipped solution is a smooth EL sweep
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=100).map(|i| {
            let f = i as f64 / 100.0; let el = 10.0 + 160.0 * f;          // 10 .. 170 measured from the south horizon
            if el <= 90.0 { (f * 600.0, 180.0, el) } else { (f * 600.0, 0.0, 180.0 - el) }
        }).collect();
        let p = g.solve_path(&s).unwrap();
        assert!(p.max_az_rate < 1.0, "az rate {}", p.max_az_rate);
        assert!(!p.clipped);
        //The elevation axis carries the whole pass: one smooth run of 160 degrees
        let els: Vec<f64> = p.points.iter().map(|p| p.2).collect();
        let span = els.iter().cloned().fold(f64::MIN, f64::max) - els.iter().cloned().fold(f64::MAX, f64::min);
        assert!((span - 160.0).abs() < 1e-6, "el span {span}");
    }

    #[test]
    fn crossing_the_limit_is_clipped_only_without_overlap() {
        //With 450 deg of travel every bearing is reachable and the axis limits sit at bearings 352.5 and 82.5:
        //a track 350 -> 70 runs from mount az 357.5 to 437.5 and fits.
        let g = geom();
        let s: Vec<(f64, f64, f64)> = (0..=40).map(|i| (i as f64 * 10.0, 350.0 + i as f64 * 2.0, 15.0)).collect();
        let p = g.solve_path(&s).unwrap();
        assert!(!p.clipped);
        //A plain 360 mount with no flip-over (el max 90) has a real seam at bearing 37.5: the same track is clipped
        let g360 = MountGeom { az_travel: 360.0, az_center: 217.5, el_min: -5.0, el_max: 90.0 };
        let p = g360.solve_path(&s).unwrap();
        assert!(p.clipped);
    }

    #[test]
    fn path_interpolates() {
        let p = MountPath { flip: Flip::Normal, flips_mid: 0, points: vec![(0.0, 10.0, 0.0), (10.0, 20.0, 10.0)], clipped: false, margin_deg: 0.0, max_az_rate: 1.0, max_el_rate: 1.0 };
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
        assert!(!m.ingest("OK GO"));
    }
}

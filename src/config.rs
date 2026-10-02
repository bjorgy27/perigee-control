/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Perigee Control settings, loaded from control.toml (see that file for what each key does).
/// Every section and key has a default, so a partial or missing file still works. The viewer's own
/// settings stay in its viewer.toml; [viewer] only says where that file is.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use bevy::prelude::*;
use serde::Deserialize;

#[derive(Resource, Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct ControlConfig {
    pub viewer: ViewerRef,
    pub data: DataDir,
    pub window: WindowCfg,
    pub tiles: TilesCfg,
    pub serial: SerialCfg,
    pub mount: MountCfg,
    pub calibration: CalibrationCfg,
    pub tracking: TrackingCfg,
    pub perigee: PerigeeCfg,
    pub boot: BootCfg,
}

impl ControlConfig {
    pub fn load(path: &str) -> ControlConfig {
        let mut c = match std::fs::read_to_string(path) {
            Ok(txt) => match toml::from_str(&txt) {
                Ok(c) => c,
                Err(e) => { eprintln!("{path}: {e}; using defaults"); ControlConfig::default() }
            },
            Err(_) => { println!("{path} not found; using defaults"); ControlConfig::default() }
        };
        //The sky tie, if one was saved (/here or /bearing); then force the window inside the
        //firmware's, so an edit here can only ever narrow the safety envelope.
        c.load_calibration();
        for note in c.mount.clamp_to_firmware() { eprintln!("{path}: {note}"); }
        c
    }

    /// Record a measured centre bearing (the console's `/bearing DEG`): write it to the `[calibration]`
    /// file, which `load_calibration` reads at every start, and use it from now on. Returns the path written.
    /// The elevation correction is kept; the last `/here` point is dropped, because a typed bearing moves
    /// the frame and it is no longer exact there (so it cannot anchor the next motor-scale check).
    pub fn set_center_bearing(&mut self, bearing: f64) -> Result<String, String> {
        if !bearing.is_finite() { return Err("not a number".into()); }
        self.cal_path()?;
        let before = (self.mount.az_center_bearing_deg, self.calibration.here);
        self.mount.az_center_bearing_deg = Some(round3(bearing.rem_euclid(360.0)));
        self.calibration.here = None;
        self.save_calibration().inspect_err(|_| (self.mount.az_center_bearing_deg, self.calibration.here) = before)
    }

    /// `/here AZ [EL]`: the dish, which the firmware's telemetry puts at mount pose `at` (az_m, el_m), is
    /// pointing at true bearing `bearing` and, if given, true elevation `el`. Ties the mount to the sky at
    /// this point: the centre bearing that makes `at` face `bearing`, and the elevation correction
    /// (mount minus true) that makes it look at `el`. Saved to the `[calibration]` file and used at once.
    ///
    /// One reference fixes the offsets, not the azimuth motor's scale (goBILDA's two figures disagree by
    /// 2%). So when a previous `/here` was taken far enough round, the report says how far off the old tie
    /// was here: the old tie was exact at that previous point, so its error over the turn between the two
    /// is the scale error, and `turn / true turn` is the factor `AZ_CAL.us_per_deg` needs in the firmware.
    pub fn set_here(&mut self, at: (f64, f64), bearing: f64, el: Option<f64>) -> Result<(String, HereReport), String> {
        if !bearing.is_finite() || !at.0.is_finite() || !at.1.is_finite() { return Err("not a number".into()); }
        self.cal_path()?;
        let el_corr = match el {
            None => self.mount.el_correction_deg,
            Some(e) if !e.is_finite() || !(-5.0..=90.0).contains(&e) => return Err(format!("elevation {e} is not a sky elevation (-5 .. 90)")),
            Some(e) => round3(at.1 - e),
        };
        if el_corr.abs() > MAX_EL_CORRECTION_DEG {
            return Err(format!("that would be an elevation correction of {el_corr:+.1} deg; more than {MAX_EL_CORRECTION_DEG:.0} means the wrong reference or a wrong number"));
        }
        let b = bearing.rem_euclid(360.0);
        let old = crate::mount::MountGeom::from_cfg(&self.mount);
        let before = old.sky_of(at.0, at.1);
        let scale = self.calibration.here.and_then(|(ref_az, _)| {
            let turn = at.0 - ref_az;
            if turn.abs() < MIN_SCALE_TURN_DEG { return None; }
            let off = wrap180(old.bearing(at.0) - b);   // where the old tie said this was, minus where it is
            let true_turn = turn - off;
            (true_turn.abs() > 1.0).then(|| (turn.abs(), off, turn / true_turn))
        });
        let prev = (self.mount.az_center_bearing_deg, self.mount.el_correction_deg, self.calibration.here);
        let center = round3((b - at.0 + self.mount.az_travel_deg / 2.0).rem_euclid(360.0));
        self.mount.az_center_bearing_deg = Some(center);
        self.mount.el_correction_deg = el_corr;
        self.calibration.here = Some((round3(at.0), round3(b)));
        match self.save_calibration() {
            Ok(path) => Ok((path, HereReport { center, el_corr, before, scale })),
            Err(e) => { (self.mount.az_center_bearing_deg, self.mount.el_correction_deg, self.calibration.here) = prev; Err(e) }
        }
    }

    /// `/here clear`: forget the sky tie (centre bearing, elevation correction, last `/here` point), however
    /// it was set. The mount is uncalibrated again: nominal frame, and a real mount will not track until
    /// it is tied again with `/here` or `/bearing`. Returns the path written.
    pub fn clear_here(&mut self) -> Result<String, String> {
        self.cal_path()?;
        let prev = (self.mount.az_center_bearing_deg, self.mount.el_correction_deg, self.calibration.here);
        self.mount.az_center_bearing_deg = None;
        self.mount.el_correction_deg = 0.0;
        self.calibration.here = None;
        self.save_calibration().inspect_err(|_| (self.mount.az_center_bearing_deg, self.mount.el_correction_deg, self.calibration.here) = prev)
    }

    fn cal_path(&self) -> Result<String, String> {
        let path = self.calibration.file.trim().to_string();
        if path.is_empty() { return Err("[calibration] file is empty in control.toml, so there is nowhere to keep it".into()); }
        Ok(path)
    }

    /// Write the whole sky tie to the `[calibration]` file: what `load_calibration` reads at every start
    fn save_calibration(&self) -> Result<String, String> {
        let path = self.cal_path()?;
        let m = &self.mount;
        let mut txt = String::from("# Written by perigee-control (/here, /bearing): how the mount is tied to the sky.\n\
                                    # /here clear in the SERIAL CONSOLE, or deleting this file, makes the mount uncalibrated again.\n");
        match m.az_center_bearing_deg {
            Some(b) => txt += &format!("az_center_bearing_deg = {b:.3}   # true bearing the dish faces at mount azimuth 225\n"),
            None => txt += "# no sky tie: the mount is uncalibrated (nominal frame)\n",
        }
        if m.el_correction_deg != 0.0 {
            txt += &format!("el_correction_deg = {:.3}   # mount elevation minus true elevation, added to every elevation sent to the mount\n", m.el_correction_deg);
        }
        if let Some((a, b)) = self.calibration.here {
            txt += &format!("here_mount_az_deg = {a:.3}   # where the last /here was taken: mount azimuth,\n\
                             here_bearing_deg = {b:.3}    # and the true bearing given there (the next /here checks the motor scale against it)\n");
        }
        std::fs::write(&path, txt).map_err(|e| format!("{path}: {e}"))?;
        Ok(path)
    }

    /// Overlay the calibration state file onto `[mount]`. A measured number (saved with the
    /// console's /here or /bearing) wins over anything in control.toml, which is exactly why the
    /// numbers are not kept there: a hand-typed guess would silently outrank a measurement.
    fn load_calibration(&mut self) {
        let path = self.calibration.file.trim().to_string();
        if path.is_empty() { return; }
        //No file is the normal state before the first calibration run, not an error.
        let Ok(txt) = std::fs::read_to_string(&path) else { return };
        match toml::from_str::<CalibrationState>(&txt) {
            Ok(s) => {
                if let Some(b) = s.az_center_bearing_deg {
                    if b.is_finite() { self.mount.az_center_bearing_deg = Some(b.rem_euclid(360.0)); }
                    else { eprintln!("{path}: az_center_bearing_deg is not a number; mount azimuth stays uncalibrated"); }
                }
                if let Some(c) = s.el_correction_deg {
                    if c.is_finite() && c.abs() <= MAX_EL_CORRECTION_DEG { self.mount.el_correction_deg = c; }
                    else { eprintln!("{path}: el_correction_deg = {c} is not a believable correction; using 0"); }
                }
                if let (Some(a), Some(b)) = (s.here_mount_az_deg, s.here_bearing_deg) {
                    if a.is_finite() && b.is_finite() { self.calibration.here = Some((a, b.rem_euclid(360.0))); }
                }
            }
            Err(e) => eprintln!("{path}: {e}; mount azimuth stays uncalibrated"),
        }
    }
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct ViewerRef { pub config: String }
impl Default for ViewerRef { fn default() -> Self { Self { config: "../perigee-viewer/viewer.toml".into() } } }

#[derive(Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct DataDir { pub dir: String }

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct WindowCfg { pub title: String, pub width: f32, pub height: f32, pub font_size: f32, pub hdr: bool, pub bloom: f32, pub scanlines: bool }
impl Default for WindowCfg {
    fn default() -> Self { Self { title: "PERIGEE // control".into(), width: 1920.0, height: 1080.0, font_size: 12.0, hdr: true, bloom: 0.25, scanlines: true } }
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct TilesCfg {
    pub gap: f32, pub main_ratio: f32, pub row_ratio: f32,
    pub globe_ratio: f32,       // width share of the ORBIT VIEW tile at launch (the four command tiles share the rest)
    pub divider: String,        // colour of the bright lines drawn down the middle of every gap ("" = none)
    pub divider_width: f32,     // their width in pixels
    pub border: String,         // tile frame colour ("" = the viewer's dim text colour)
}
impl Default for TilesCfg {
    fn default() -> Self { Self { gap: 8.0, main_ratio: 0.5, row_ratio: 0.58, globe_ratio: 0.5, divider: "#9CFFB8".into(), divider_width: 2.5, border: "#3BEB6B".into() } }
}

//The Perigee engine, run from the boot page: `perigee login`, `perigee` (full run) and `perigee rank`
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct PerigeeCfg {
    pub bin: String,    // the built engine; when it is missing, `cargo run --release` in `dir` is used instead
    pub dir: String,    // the engine's crate (its .env holds the saved Space-Track credentials)
    pub orbits: String, // which orbits a full refresh pulls: "leo" (default), "geo" (the belt: GOES, ...), "all"
}
impl Default for PerigeeCfg { fn default() -> Self { Self { bin: "../Perigee/target/release/perigee".into(), dir: "../Perigee".into(), orbits: "leo".into() } } }

//The boot page shown before the tiles: checks, Space-Track login, engine run, load
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct BootCfg {
    pub enabled: bool,      // false: straight to the tiles, no login
    pub type_cps: f64,      // teletype speed, characters per second (a backlog of engine output speeds it up)
    pub line_seconds: f64,  // pause after each line
    pub font_size: f32,     // the boot page's type (larger than the tiles: it is read from across the room)
}
impl Default for BootCfg { fn default() -> Self { Self { enabled: true, type_cps: 240.0, line_seconds: 0.03, font_size: 15.0 } } }

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct SerialCfg { pub port: String, pub baud: u32, pub simulate: bool, pub reconnect_seconds: f64, pub telemetry_hz: f64 }
impl Default for SerialCfg {
    fn default() -> Self { Self { port: "auto".into(), baud: 115200, simulate: true, reconnect_seconds: 3.0, telemetry_hz: 5.0 } }
}

//--------------------------------------------------------------------------- the firmware's window
/// The mount's safety envelope, mirrored from `firmware/perigee_mount_stm32/src/limits.rs`
/// (`AZ_CAL` / `EL_CAL`). That file is the authority: it clamps every pulse it emits against these
/// numbers whatever the host asks for, so a wider value in control.toml buys nothing except a host
/// that plans moves the board answers with `ERR ... clamped`. The host may **narrow** the window,
/// never widen it, which is what `MountCfg::clamp_to_firmware` enforces on load.
/// `docs/calibration.md` §5.1 is the record of what happens when these two places disagree.
pub const FW_AZ_TRAVEL_DEG: f64 = 450.0;
pub const FW_AZ_LO_DEG: f64 = 0.0;
pub const FW_AZ_HI_DEG: f64 = 400.0;
pub const FW_EL_LO_DEG: f64 = -2.0;
pub const FW_EL_HI_DEG: f64 = 91.0;
/// `RATE` accepts this range and the firmware clamps to it (see the protocol in `serial.rs`)
pub const FW_RATE_MIN_DPS: f64 = 0.1;
pub const FW_RATE_MAX_DPS: f64 = 90.0;

/// The centre bearing the mount frame falls back to while the real one is unmeasured: the design
/// intent (middle of the 120..315 dish sector), enough to draw the wireframe and run the simulator,
/// nowhere near good enough to point a 1 m dish. Tracking a *real* mount is refused until a
/// measured value has been saved (/bearing, docs/calibration.md 6.7); see `MountCfg::calibrated`.
pub const NOMINAL_AZ_CENTER_BEARING_DEG: f64 = 217.5;

/// The largest elevation correction (mount minus true elevation) believed: more than this is a wrong
/// reference or a typo, not a mount that is really that far off level
pub const MAX_EL_CORRECTION_DEG: f64 = 20.0;
/// Two /here references closer together than this in azimuth say nothing useful about the motor scale
const MIN_SCALE_TURN_DEG: f64 = 30.0;

fn round3(v: f64) -> f64 { (v * 1000.0).round() / 1000.0 }
/// An angle difference folded into -180 .. 180
fn wrap180(d: f64) -> f64 { (d + 180.0).rem_euclid(360.0) - 180.0 }

/// What `/here` did, for the console to report
#[derive(Clone, Debug)]
pub struct HereReport {
    pub center: f64,                    // the centre bearing now in use
    pub el_corr: f64,                   // the elevation correction now in use
    pub before: (f64, f64),             // where the old tie said the dish was looking (bearing, elevation)
    /// Against the previous /here: (degrees of azimuth turn between the two, how far off the old tie was
    /// here in degrees, the factor AZ_CAL.us_per_deg needs). None without a previous /here far enough round.
    pub scale: Option<(f64, f64, f64)>,
}

/// Narrow one configured value into a firmware bound, noting it if that changed anything
fn narrow(name: &str, v: &mut f64, lo: f64, hi: f64, notes: &mut Vec<String>) {
    let c = if v.is_finite() { v.clamp(lo, hi) } else { lo };
    if (c - *v).abs() > 1e-9 || !v.is_finite() {
        notes.push(format!("[mount] {name} = {v} is outside what the firmware allows ({lo} .. {hi}); using {c}"));
        *v = c;
    }
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct MountCfg {
    /// Hardware travel of the azimuth gearbox. Defines the mount frame; not the usable range.
    /// Must equal the firmware's `AZ_CAL.gear_travel`: the frame tie is derived from it.
    pub az_travel_deg: f64,
    /// True bearing the boresight faces at mid hardware travel, the one number tying the mount to
    /// the sky. **Measured, never typed:** measured on the bench (docs/calibration.md
    /// 6.7), saved with the console's /bearing, and kept in the `[calibration] file`, not in control.toml. `None` = never
    /// calibrated, so the mount frame is nominal only: the simulator still runs the full procedure,
    /// a real serial link refuses to TRACK.
    pub az_center_bearing_deg: Option<f64>,
    /// Mount elevation minus true elevation at the same pointing, deg: the host adds it to every
    /// elevation it sends and takes it off every elevation it shows. Measured with `/here AZ EL` (the
    /// dish pointed at something whose elevation is known) and kept in the `[calibration] file`, like
    /// the bearing. 0 until measured. It lets a dish whose level is a few degrees out track without
    /// reflashing `EL_CAL.offset`; the firmware's window (`el_min_deg .. el_max_deg`) stays in mount degrees.
    pub el_correction_deg: f64,
    /// Cable-wrap soft limits on azimuth, inside `az_travel_deg`. Must match
    /// `firmware/perigee_mount_stm32/src/limits.rs`, which enforces them independently.
    pub az_limit_lo_deg: f64,
    pub az_limit_hi_deg: f64,
    pub el_min_deg: f64, pub el_max_deg: f64,
    pub az_rate_dps: f64, pub el_rate_dps: f64, pub park_az_deg: f64, pub park_el_deg: f64, pub home_el_deg: f64,
}
impl Default for MountCfg {
    fn default() -> Self { Self {
        az_travel_deg: FW_AZ_TRAVEL_DEG, az_center_bearing_deg: None, el_correction_deg: 0.0,
        az_limit_lo_deg: FW_AZ_LO_DEG, az_limit_hi_deg: FW_AZ_HI_DEG,
        el_min_deg: FW_EL_LO_DEG, el_max_deg: FW_EL_HI_DEG,
        az_rate_dps: 20.0, el_rate_dps: 15.0, park_az_deg: 200.0, park_el_deg: 45.0, home_el_deg: 0.0,
    } }
}

impl MountCfg {
    /// Has a measured centre bearing been saved, so we know where the mount is pointing?
    pub fn calibrated(&self) -> bool { self.az_center_bearing_deg.is_some() }

    /// The centre bearing to do geometry with: the measured one when there is one, otherwise the
    /// nominal design value so the wireframe and the simulator still have a frame to work in.
    pub fn az_center_or_nominal(&self) -> f64 {
        self.az_center_bearing_deg.unwrap_or(NOMINAL_AZ_CENTER_BEARING_DEG)
    }

    /// Force every configured angle inside the firmware's own window, returning one note per value
    /// that had to be changed. Called on load, so the toml can only ever tighten the safety
    /// envelope: the firmware would clamp a wider one anyway, and a host planning against limits
    /// the board refuses is how a pass gets silently clipped at the wrong angle.
    pub fn clamp_to_firmware(&mut self) -> Vec<String> {
        let mut n = Vec::new();
        //The frame tie depends on the travel, so this one has to match rather than merely fit
        if (self.az_travel_deg - FW_AZ_TRAVEL_DEG).abs() > 1e-9 {
            n.push(format!("[mount] az_travel_deg = {} does not match the firmware's gearbox travel {FW_AZ_TRAVEL_DEG}; using {FW_AZ_TRAVEL_DEG}", self.az_travel_deg));
            self.az_travel_deg = FW_AZ_TRAVEL_DEG;
        }
        //Limits first: everything below is then held inside the window they ended up describing
        narrow("az_limit_lo_deg", &mut self.az_limit_lo_deg, FW_AZ_LO_DEG, FW_AZ_HI_DEG, &mut n);
        narrow("az_limit_hi_deg", &mut self.az_limit_hi_deg, self.az_limit_lo_deg, FW_AZ_HI_DEG, &mut n);
        narrow("el_min_deg", &mut self.el_min_deg, FW_EL_LO_DEG, FW_EL_HI_DEG, &mut n);
        narrow("el_max_deg", &mut self.el_max_deg, self.el_min_deg, FW_EL_HI_DEG, &mut n);
        //Poses the host sends or uses as a jog origin: inside the window it just settled on
        narrow("park_az_deg", &mut self.park_az_deg, self.az_limit_lo_deg, self.az_limit_hi_deg, &mut n);
        narrow("park_el_deg", &mut self.park_el_deg, self.el_min_deg, self.el_max_deg, &mut n);
        narrow("home_el_deg", &mut self.home_el_deg, self.el_min_deg, self.el_max_deg, &mut n);
        //Rates are sent verbatim as RATE, which the firmware clamps to the same range
        narrow("az_rate_dps", &mut self.az_rate_dps, FW_RATE_MIN_DPS, FW_RATE_MAX_DPS, &mut n);
        narrow("el_rate_dps", &mut self.el_rate_dps, FW_RATE_MIN_DPS, FW_RATE_MAX_DPS, &mut n);
        if !self.el_correction_deg.is_finite() || self.el_correction_deg.abs() > MAX_EL_CORRECTION_DEG {
            n.push(format!("[mount] el_correction_deg = {} is not a believable correction; using 0", self.el_correction_deg));
            self.el_correction_deg = 0.0;
        }
        //A bearing is modular, so it is wrapped rather than clamped
        if let Some(b) = self.az_center_bearing_deg {
            if !b.is_finite() {
                n.push("[mount] az_center_bearing_deg is not a number; treating the mount as uncalibrated".into());
                self.az_center_bearing_deg = None;
            } else if !(0.0..360.0).contains(&b) {
                self.az_center_bearing_deg = Some(b.rem_euclid(360.0));
            }
        }
        n
    }
}

//--------------------------------------------------------------------------------- calibration file
/// Where the sky tie is kept (written by /here and /bearing). Separate from control.toml on purpose:
/// control.toml is Beck's to edit and holds no calibration numbers, this file is written by the console
/// commands and is safe to delete (or `/here clear`) to force a recalibration.
#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct CalibrationCfg {
    pub file: String,
    /// Where the last /here was taken: (mount azimuth, the true bearing given there). Read from the file,
    /// never from control.toml; the next /here measures the azimuth motor scale against it.
    #[serde(skip)]
    pub here: Option<(f64, f64)>,
}
impl Default for CalibrationCfg { fn default() -> Self { Self { file: "calibration.toml".into(), here: None } } }

/// The calibration state file's contents. Every field is optional: absent means that number has not
/// been measured yet, which is not an error, it is the state the mount ships in.
#[derive(Deserialize, Clone, Debug, Default)]
#[serde(default)]
pub struct CalibrationState {
    /// True bearing at mid hardware travel, as measured. See `docs/calibration.md` §6.7.
    pub az_center_bearing_deg: Option<f64>,
    /// Mount elevation minus true elevation (`/here AZ EL`)
    pub el_correction_deg: Option<f64>,
    /// The last /here point: mount azimuth and the true bearing given there
    pub here_mount_az_deg: Option<f64>,
    pub here_bearing_deg: Option<f64>,
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct TrackingCfg {
    pub sample_seconds: f64, pub command_hz: f64, pub lead_seconds: f64,
    pub lookahead_hours: f64, pub mask_deg: f64, pub park_after: bool,
    pub auto_arm: bool,         // picking a satellite starts the procedure by itself (a new pick restarts it)
    pub step_seconds: f64,      // pause between the procedure's checks so they can be read as they fill in
    pub on_point_deg: f64,      // on point: T4 settled estimate this close to the firmware's target, and that target to the last GO
    pub sim_clock: bool,        // with the simulator, follow the viewer's clock (HISTORY speed applies) instead of the wall clock
    pub sim_warp: bool,         // with the simulator, jump the viewer's clock to just before AOS so the pass plays out now
    pub warp_lead_s: f64,       // seconds before AOS the warp lands
    pub warp_speed: f64,        // viewer speed while the warped pass runs
}
impl Default for TrackingCfg {
    fn default() -> Self { Self {
        sample_seconds: 2.0, command_hz: 4.0, lead_seconds: 0.4, lookahead_hours: 24.0, mask_deg: 0.0, park_after: true,
        auto_arm: true, step_seconds: 0.7, on_point_deg: 0.5, sim_clock: true, sim_warp: true, warp_lead_s: 90.0, warp_speed: 20.0,
    } }
}

//-------------------------------------------------------------------------------------------- tests
#[cfg(test)]
mod tests {
    use super::*;

    fn mount_from(toml_body: &str) -> (MountCfg, Vec<String>) {
        let mut m: MountCfg = toml::from_str(toml_body).expect("parses");
        let notes = m.clamp_to_firmware();
        (m, notes)
    }

    #[test]
    fn a_fresh_config_is_uncalibrated_and_falls_back_to_the_nominal_frame() {
        let m = MountCfg::default();
        assert!(!m.calibrated(), "no calibration file has been read, so the mount is not calibrated");
        assert_eq!(m.az_center_bearing_deg, None, "the bearing must never default to a guessed number");
        //The frame still has to exist, or the wireframe and the simulator have nothing to draw in
        assert_eq!(m.az_center_or_nominal(), NOMINAL_AZ_CENTER_BEARING_DEG);
    }

    #[test]
    fn the_defaults_are_exactly_the_firmware_window() {
        let m = MountCfg::default();
        assert_eq!((m.az_limit_lo_deg, m.az_limit_hi_deg), (FW_AZ_LO_DEG, FW_AZ_HI_DEG));
        assert_eq!((m.el_min_deg, m.el_max_deg), (FW_EL_LO_DEG, FW_EL_HI_DEG));
        assert_eq!(m.az_travel_deg, FW_AZ_TRAVEL_DEG);
        //and the defaults are already legal, so loading an empty file changes nothing
        let (_, notes) = mount_from("");
        assert!(notes.is_empty(), "{notes:?}");
    }

    #[test]
    fn the_toml_can_never_widen_the_safety_window() {
        //Every limit pushed outside the firmware's, in both directions at once
        let (m, notes) = mount_from(
            "az_limit_lo_deg = -50.0\naz_limit_hi_deg = 449.0\nel_min_deg = -5.0\nel_max_deg = 200.0\n");
        assert_eq!(m.az_limit_lo_deg, FW_AZ_LO_DEG, "azimuth floor widened");
        assert_eq!(m.az_limit_hi_deg, FW_AZ_HI_DEG, "azimuth ceiling widened past the cable wrap");
        //-5 is the exact mismatch that was live in control.toml: it puts the el gearbox on its stop
        assert_eq!(m.el_min_deg, FW_EL_LO_DEG, "elevation floor widened onto the bottom stop");
        assert_eq!(m.el_max_deg, FW_EL_HI_DEG, "elevation ceiling widened");
        assert_eq!(notes.len(), 4, "each widened key must be reported: {notes:?}");
        assert!(notes.iter().any(|n| n.contains("el_min_deg")), "{notes:?}");
    }

    #[test]
    fn the_toml_may_still_narrow_the_window() {
        //Tightening is the whole point of having the keys here, so it must survive untouched
        let (m, notes) = mount_from(
            "az_limit_lo_deg = 20.0\naz_limit_hi_deg = 380.0\nel_min_deg = 0.0\nel_max_deg = 90.0\n");
        assert_eq!((m.az_limit_lo_deg, m.az_limit_hi_deg), (20.0, 380.0));
        assert_eq!((m.el_min_deg, m.el_max_deg), (0.0, 90.0));
        assert!(notes.is_empty(), "{notes:?}");
    }

    #[test]
    fn an_inverted_window_collapses_instead_of_opening_up() {
        //hi below lo would otherwise describe a negative span that az_ok accepts nothing from
        let (m, _) = mount_from("az_limit_lo_deg = 300.0\naz_limit_hi_deg = 10.0\nel_min_deg = 90.0\nel_max_deg = 0.0\n");
        assert!(m.az_limit_hi_deg >= m.az_limit_lo_deg, "{} .. {}", m.az_limit_lo_deg, m.az_limit_hi_deg);
        assert!(m.el_max_deg >= m.el_min_deg, "{} .. {}", m.el_min_deg, m.el_max_deg);
    }

    #[test]
    fn park_home_and_rates_are_held_inside_the_window_too() {
        //Park outside the cable wrap is the 225-vs-200 bug class: the host would show a pose the
        //firmware refuses. Rates outside 0.1..90 are silently clamped by the board's RATE handler.
        let (m, notes) = mount_from(
            "az_limit_hi_deg = 300.0\npark_az_deg = 400.0\npark_el_deg = 300.0\nhome_el_deg = -40.0\naz_rate_dps = 500.0\nel_rate_dps = 0.0\n");
        assert_eq!(m.park_az_deg, 300.0, "park must sit inside the narrowed window");
        assert_eq!(m.park_el_deg, FW_EL_HI_DEG);
        assert_eq!(m.home_el_deg, FW_EL_LO_DEG);
        assert_eq!(m.az_rate_dps, FW_RATE_MAX_DPS);
        assert_eq!(m.el_rate_dps, FW_RATE_MIN_DPS);
        assert!(notes.len() >= 5, "{notes:?}");
    }

    #[test]
    fn a_travel_that_disagrees_with_the_gearbox_is_corrected() {
        //az_travel defines the frame tie (az_zero = centre - travel/2), so a wrong one silently
        //rotates every bearing the host computes
        let (m, notes) = mount_from("az_travel_deg = 360.0\n");
        assert_eq!(m.az_travel_deg, FW_AZ_TRAVEL_DEG);
        assert!(notes.iter().any(|n| n.contains("az_travel_deg")), "{notes:?}");
    }

    #[test]
    fn garbage_angles_do_not_become_a_frame() {
        let (m, _) = mount_from("az_center_bearing_deg = nan\n");
        assert!(!m.calibrated(), "a NaN bearing must read as uncalibrated, not as a frame");
        let (m, notes) = mount_from("el_min_deg = nan\n");
        assert_eq!(m.el_min_deg, FW_EL_LO_DEG);
        assert!(!notes.is_empty());
    }

    #[test]
    fn a_measured_bearing_is_wrapped_not_clamped() {
        //Bearings are modular: 380 is 20, and clamping it to 360 would be a 20 degree pointing error
        let (m, _) = mount_from("az_center_bearing_deg = 380.0\n");
        assert_eq!(m.az_center_bearing_deg, Some(20.0));
        let (m, _) = mount_from("az_center_bearing_deg = -10.0\n");
        assert_eq!(m.az_center_bearing_deg, Some(350.0));
        //and an ordinary measured value is left exactly alone
        let (m, _) = mount_from("az_center_bearing_deg = 213.47\n");
        assert_eq!(m.az_center_bearing_deg, Some(213.47));
        assert!(m.calibrated());
    }

    #[test]
    fn the_calibration_file_is_what_makes_the_mount_calibrated() {
        let dir = std::env::temp_dir().join(format!("perigee_cal_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("calibration.toml");

        //Absent file: uncalibrated, and that is a normal state, not an error
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        c.load_calibration();
        assert!(!c.mount.calibrated(), "no file must mean uncalibrated");

        //What /bearing writes
        std::fs::write(&path, "az_center_bearing_deg = 213.470\n").unwrap();
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        c.load_calibration();
        assert_eq!(c.mount.az_center_bearing_deg, Some(213.47));
        assert!(c.mount.calibrated());

        //A measurement outranks anything typed into control.toml
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        c.mount.az_center_bearing_deg = Some(217.5);
        c.load_calibration();
        assert_eq!(c.mount.az_center_bearing_deg, Some(213.47), "the measured value must win");

        //A file that exists but says nothing yet leaves the mount uncalibrated
        std::fs::write(&path, "# written before the fit converged\n").unwrap();
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        c.load_calibration();
        assert!(!c.mount.calibrated());

        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn bearing_command_writes_the_file_that_the_next_start_reads() {
        let dir = std::env::temp_dir().join(format!("perigee_bearing_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("calibration.toml");
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        assert!(!c.mount.calibrated());
        //Used at once, and wrapped like any bearing
        c.set_center_bearing(-148.5637).unwrap();
        assert_eq!(c.mount.az_center_bearing_deg, Some(211.436));
        //and what was written is exactly what a fresh start loads
        let mut fresh = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        fresh.load_calibration();
        assert_eq!(fresh.mount.az_center_bearing_deg, Some(211.436));
        //Nonsense is refused and changes nothing
        assert!(c.set_center_bearing(f64::NAN).is_err());
        assert_eq!(c.mount.az_center_bearing_deg, Some(211.436));
        let mut nowhere = ControlConfig { calibration: CalibrationCfg { file: String::new(), here: None }, ..Default::default() };
        assert!(nowhere.set_center_bearing(200.0).is_err() && !nowhere.mount.calibrated());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn here_ties_the_mount_to_the_sky_and_clear_forgets_it() {
        let dir = std::env::temp_dir().join(format!("perigee_here_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("calibration.toml");
        let cal = || CalibrationCfg { file: path.to_string_lossy().into(), here: None };
        let mut c = ControlConfig { calibration: cal(), ..Default::default() };
        //The dish sits at mount 212.3 / 47.0 and really points at bearing 245, elevation 45
        let (_, r) = c.set_here((212.3, 47.0), 245.0, Some(45.0)).unwrap();
        assert_eq!(c.mount.az_center_bearing_deg, Some(257.7), "245 - 212.3 + 225");
        assert_eq!(c.mount.el_correction_deg, 2.0, "the mount reads 2 degrees high");
        assert!(c.mount.calibrated() && r.scale.is_none(), "the first /here has nothing to check the scale against");
        //...so from now on that pose IS bearing 245 elevation 45, and that direction is commanded as that pose
        let g = crate::mount::MountGeom::from_cfg(&c.mount);
        let (b, e) = g.sky_of(212.3, 47.0);
        assert!((b - 245.0).abs() < 1e-6 && (e - 45.0).abs() < 1e-6, "{b} {e}");
        let (az, el) = g.nearest_pose(245.0, 45.0, (200.0, 45.0)).unwrap();
        assert!((az - 212.3).abs() < 1e-6 && (el - 47.0).abs() < 1e-6, "{az} {el}");
        //A fresh start reads back exactly the same tie
        let mut fresh = ControlConfig { calibration: cal(), ..Default::default() };
        fresh.load_calibration();
        assert_eq!((fresh.mount.az_center_bearing_deg, fresh.mount.el_correction_deg, fresh.calibration.here),
                   (Some(257.7), 2.0, Some((212.3, 245.0))));
        //Azimuth only: the elevation correction is left as it was
        c.set_here((100.0, 30.0), 132.6, None).unwrap();
        assert_eq!(c.mount.el_correction_deg, 2.0);
        //A typed /bearing keeps the elevation correction but drops the /here point it no longer passes through
        c.set_center_bearing(250.0).unwrap();
        assert_eq!((c.mount.el_correction_deg, c.calibration.here), (2.0, None));
        //Clear: uncalibrated again, nothing left in the file to bring it back
        c.clear_here().unwrap();
        assert!(!c.mount.calibrated() && c.mount.el_correction_deg == 0.0 && c.calibration.here.is_none());
        let mut fresh = ControlConfig { calibration: cal(), ..Default::default() };
        fresh.load_calibration();
        assert!(!fresh.mount.calibrated() && fresh.mount.el_correction_deg == 0.0, "a cleared file must load as uncalibrated");
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn a_second_here_far_round_measures_the_azimuth_motor_scale() {
        let dir = std::env::temp_dir().join(format!("perigee_here_scale_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("calibration.toml");
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        c.set_here((100.0, 45.0), 150.0, None).unwrap();
        //150 degrees of commanded turn later the dish has really turned 147: the firmware's slope is 2% short
        let (_, r) = c.set_here((250.0, 45.0), 297.0, None).unwrap();
        let (turn, off, k) = r.scale.expect("150 degrees apart is plenty to judge the scale");
        assert!((turn - 150.0).abs() < 1e-9 && (off - 3.0).abs() < 1e-9, "{turn} {off}");
        assert!((k - 150.0 / 147.0).abs() < 1e-9, "AZ_CAL.us_per_deg needs x{k}");
        //and the tie now passes through the new point exactly
        assert!((crate::mount::MountGeom::from_cfg(&c.mount).bearing(250.0) - 297.0).abs() < 1e-6);
        //Two references close together say nothing about the scale
        let (_, r) = c.set_here((260.0, 45.0), 307.0, None).unwrap();
        assert!(r.scale.is_none());
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn here_refuses_nonsense_and_changes_nothing() {
        let dir = std::env::temp_dir().join(format!("perigee_here_bad_{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("calibration.toml");
        let mut c = ControlConfig { calibration: CalibrationCfg { file: path.to_string_lossy().into(), here: None }, ..Default::default() };
        assert!(c.set_here((200.0, 45.0), f64::NAN, None).is_err());
        assert!(c.set_here((200.0, 45.0), 245.0, Some(120.0)).is_err(), "120 is not a sky elevation");
        assert!(c.set_here((200.0, 45.0), 245.0, Some(15.0)).is_err(), "a 30 degree correction is a wrong reference");
        assert!(!c.mount.calibrated() && c.mount.el_correction_deg == 0.0 && !path.exists(), "a refusal must leave no trace");
        let mut nowhere = ControlConfig { calibration: CalibrationCfg { file: String::new(), here: None }, ..Default::default() };
        assert!(nowhere.set_here((200.0, 45.0), 245.0, Some(45.0)).is_err() && !nowhere.mount.calibrated());
        assert!(nowhere.clear_here().is_err());
        //A hand-edited file with an unbelievable correction is not used
        std::fs::write(&path, "az_center_bearing_deg = 200.0\nel_correction_deg = 45.0\n").unwrap();
        c.load_calibration();
        assert_eq!((c.mount.az_center_bearing_deg, c.mount.el_correction_deg), (Some(200.0), 0.0));
        std::fs::remove_dir_all(&dir).ok();
    }

    #[test]
    fn the_shipped_control_toml_matches_the_firmware_and_carries_no_calibration() {
        //The file Beck actually runs, checked against the authority it is mirrored from. This is the
        //test that would have caught el_min = -5, the missing az limit keys and park 225.
        let txt = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/control.toml"))
            .expect("control.toml sits next to Cargo.toml");
        let c: ControlConfig = toml::from_str(&txt).expect("control.toml parses");
        let m = &c.mount;

        assert_eq!(m.az_travel_deg, FW_AZ_TRAVEL_DEG);
        assert_eq!((m.az_limit_lo_deg, m.az_limit_hi_deg), (FW_AZ_LO_DEG, FW_AZ_HI_DEG), "azimuth window must mirror limits.rs exactly");
        assert_eq!((m.el_min_deg, m.el_max_deg), (FW_EL_LO_DEG, FW_EL_HI_DEG), "elevation window must mirror limits.rs exactly");
        //Firmware main.rs:63 PARK = [200.0, 45.0]: the PARK button sends the firmware's own PARK, so
        //a different number here is a host that displays one park pose while the dish drives to another
        assert_eq!((m.park_az_deg, m.park_el_deg), (200.0, 45.0), "park must match the firmware's PARK");
        //No guessed calibration in a file a human edits
        assert_eq!(m.az_center_bearing_deg, None, "control.toml must not carry a calibration bearing");
        assert!(!txt.contains("\naz_center_bearing_deg"), "the key must stay commented out, not set");
        assert_eq!(m.el_correction_deg, 0.0, "nor an elevation correction");
        assert!(!txt.contains("\nel_correction_deg"), "the key must stay commented out, not set");

        //And it is already legal, so loading it clamps nothing
        let mut m2 = m.clone();
        assert!(m2.clamp_to_firmware().is_empty(), "the shipped file should need no correction");
    }
}

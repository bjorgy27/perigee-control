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
    pub tracking: TrackingCfg,
    pub perigee: PerigeeCfg,
    pub boot: BootCfg,
}

impl ControlConfig {
    pub fn load(path: &str) -> ControlConfig {
        match std::fs::read_to_string(path) {
            Ok(txt) => match toml::from_str(&txt) {
                Ok(c) => c,
                Err(e) => { eprintln!("{path}: {e}; using defaults"); ControlConfig::default() }
            },
            Err(_) => { println!("{path} not found; using defaults"); ControlConfig::default() }
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
    pub rain: bool,         // the falling glyphs behind the boot log
    pub line_seconds: f64,  // pause between lines as the log types itself out
    pub decode_seconds: f64,// how long a new line takes to resolve from noise into text
    pub font_size: f32,     // the boot page's type (larger than the tiles: it is read from across the room)
}
impl Default for BootCfg { fn default() -> Self { Self { enabled: true, rain: true, line_seconds: 0.09, decode_seconds: 0.45, font_size: 15.0 } } }

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct SerialCfg { pub port: String, pub baud: u32, pub simulate: bool, pub reconnect_seconds: f64, pub telemetry_hz: f64 }
impl Default for SerialCfg {
    fn default() -> Self { Self { port: "auto".into(), baud: 115200, simulate: true, reconnect_seconds: 3.0, telemetry_hz: 5.0 } }
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct MountCfg {
    pub az_travel_deg: f64, pub az_center_bearing_deg: f64, pub el_min_deg: f64, pub el_max_deg: f64,
    pub az_rate_dps: f64, pub el_rate_dps: f64, pub park_az_deg: f64, pub park_el_deg: f64, pub home_el_deg: f64,
}
impl Default for MountCfg {
    fn default() -> Self { Self {
        az_travel_deg: 450.0, az_center_bearing_deg: 217.5, el_min_deg: -5.0, el_max_deg: 185.0,
        az_rate_dps: 20.0, el_rate_dps: 15.0, park_az_deg: 225.0, park_el_deg: 45.0, home_el_deg: 0.0,
    } }
}

#[derive(Deserialize, Clone, Debug)]
#[serde(default)]
pub struct TrackingCfg {
    pub sample_seconds: f64, pub command_hz: f64, pub lead_seconds: f64,
    pub lookahead_hours: f64, pub mask_deg: f64, pub park_after: bool,
    pub auto_arm: bool,         // picking a satellite starts the procedure by itself (a new pick restarts it)
    pub step_seconds: f64,      // pause between the procedure's checks so they can be read as they fill in
    pub on_point_deg: f64,      // the mount counts as on point when measured and commanded agree this closely
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The boot page: what the window shows before the tiles. It walks through the launch like a terminal
/// waking up, every line resolving out of noise, glyph rain behind it:
///
///   SYSTEM CHECK    settings, data store and file ages, the engine binary, serial ports, the mount
///                   frame, then everything the viewer said while loading (the lines Perigee prints)
///   LOGON           Space-Track identification and password (prefilled from the engine's .env),
///                   verified by running `perigee login`, whose lines stream in: SESSION ESTABLISHED or
///                   AUTHENTICATION FAILED
///   DATA            proceed with the cached data, or run the engine first (full catalog refresh, re-rank)
///   INITIALIZING    every subsystem comes up on a progress bar (the orbit view's bar is the viewer's
///                   real propagation), ALL SYSTEMS NOMINAL, then the page lifts and the tiles come
///                   online one after another (`Reveal`, read by the control page)
///
/// The engine runs as a child process with the credentials in its environment; its output is read on
/// two threads (stdout, stderr) and shown as it arrives. Escape at any prompt skips ahead.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::command::{CmdCamera, CmdUi, Palette};
use crate::config::ControlConfig;
use crate::input::CmdInput;
use crate::serial::SerialLink;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use perigee_viewer::config::Config as ViewerCfg;
use perigee_viewer::{BootLog, PropStatus, ReloadData, UiFont};
use std::collections::VecDeque;
use std::io::BufRead;
use std::process::{Child, Command, Stdio};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Mutex;

/// True while the boot page owns the window (the viewer's camera is off and every key is ours)
#[derive(Resource, Default)]
pub struct Booting(pub bool);

const GLYPHS: &[u8] = b"01ABCDEF23456789#%&*+<=>?@[]{}|~$";
const MAX_ROWS: usize = 64;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Stage { Check, Login, Verify, Menu, Run, Entrance, Exit }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind { Info, Ok, Warn, Err, Engine, Title, Dim }

struct Line { text: String, kind: Kind, born: f64 }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RunKind { Login, Full, Rank }

/// A running engine process and the lines it has printed so far
struct Runner { child: Child, rx: Mutex<Receiver<String>>, started: std::time::Instant, kind: RunKind, exit: Option<i32>, exit_at: Option<f64> }

impl Runner {
    /// Start `perigee <sub>` in the data folder with the credentials in its environment (an empty
    /// value leaves the engine's own .env in charge of that variable)
    fn spawn(cfg: &ControlConfig, kind: RunKind, user: &str, pass: &str) -> Result<(Runner, String), String> {
        let sub = match kind { RunKind::Login => Some("login"), RunKind::Full => None, RunKind::Rank => Some("rank") };
        let (mut cmd, desc) = engine_command(cfg, sub)?;
        let cwd = if cfg.data.dir.is_empty() { cfg.perigee.dir.clone() } else { cfg.data.dir.clone() };
        cmd.current_dir(&cwd).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
        if !user.is_empty() { cmd.env("SPACETRACK_USER", user); }
        if !pass.is_empty() { cmd.env("SPACETRACK_PASS", pass); }
        let mut child = cmd.spawn().map_err(|e| format!("{desc}: {e}"))?;
        let (tx, rx) = channel::<String>();
        let readers: [Option<Box<dyn std::io::Read + Send>>; 2] = [
            child.stdout.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>),
            child.stderr.take().map(|s| Box::new(s) as Box<dyn std::io::Read + Send>),
        ];
        for r in readers.into_iter().flatten() {
            let tx = tx.clone();
            std::thread::spawn(move || {
                for line in std::io::BufReader::new(r).lines() {
                    match line { Ok(l) => { if tx.send(l).is_err() { break; } } Err(_) => break }
                }
            });
        }
        Ok((Runner { child, rx: Mutex::new(rx), started: std::time::Instant::now(), kind, exit: None, exit_at: None }, desc))
    }
    fn poll(&mut self, now: f64) -> Vec<String> {
        let mut out = Vec::new();
        if let Ok(rx) = self.rx.lock() { while let Ok(l) = rx.try_recv() { out.push(l); } }
        if self.exit.is_none() {
            if let Ok(Some(st)) = self.child.try_wait() { self.exit = Some(st.code().unwrap_or(-1)); self.exit_at = Some(now); }
        }
        out
    }
    /// Exit code once the process is gone and its last lines have had a moment to arrive
    fn finished(&self, now: f64) -> Option<i32> { match (self.exit, self.exit_at) { (Some(c), Some(t)) if now - t > 0.25 => Some(c), _ => None } }
    fn kill(&mut self) { let _ = self.child.kill(); let _ = self.child.wait(); }
    fn secs(&self) -> f64 { self.started.elapsed().as_secs_f64() }
}

/// The engine as a command: the built binary from [perigee] bin, or `cargo run --release` in [perigee] dir
fn engine_command(cfg: &ControlConfig, sub: Option<&str>) -> Result<(Command, String), String> {
    let bin = std::path::Path::new(&cfg.perigee.bin);
    if bin.is_file() {
        let full = std::fs::canonicalize(bin).map_err(|e| format!("{}: {e}", cfg.perigee.bin))?;
        let mut c = Command::new(&full);
        if let Some(s) = sub { c.arg(s); }
        return Ok((c, format!("{} {}", cfg.perigee.bin, sub.unwrap_or("")).trim().to_string()));
    }
    let manifest = std::path::Path::new(&cfg.perigee.dir).join("Cargo.toml");
    if !manifest.is_file() { return Err(format!("no engine: {} is not built and {} has no Cargo.toml", cfg.perigee.bin, cfg.perigee.dir)); }
    let manifest = std::fs::canonicalize(&manifest).map_err(|e| format!("{}: {e}", manifest.display()))?;
    let mut c = Command::new("cargo");
    c.args(["run", "--release", "--quiet", "--manifest-path"]).arg(&manifest).arg("--");
    if let Some(s) = sub { c.arg(s); }
    Ok((c, format!("cargo run --release -- {} (in {})", sub.unwrap_or(""), cfg.perigee.dir)))
}

/// How the engine will be run, for the check line: Ok(description) or Err(why not)
fn engine_status(cfg: &ControlConfig) -> Result<String, String> {
    let bin = std::path::Path::new(&cfg.perigee.bin);
    if bin.is_file() {
        let built = std::fs::metadata(bin).and_then(|m| m.modified()).ok()
            .map(|t| chrono::DateTime::<chrono::Local>::from(t).format("%Y-%m-%d %H:%M").to_string()).unwrap_or_default();
        return Ok(format!("{}   built {built}", cfg.perigee.bin));
    }
    if std::path::Path::new(&cfg.perigee.dir).join("Cargo.toml").is_file() {
        return Err(format!("{} not built: cargo run --release in {} will be used (slow first time)", cfg.perigee.bin, cfg.perigee.dir));
    }
    Err(format!("no engine at {} and no crate in {}", cfg.perigee.bin, cfg.perigee.dir))
}

/// "3.4 h" / "12 min" / "2.1 d" since the file was written, or None when it is missing
fn age_of(path: &std::path::Path) -> Option<String> {
    let t = std::fs::metadata(path).and_then(|m| m.modified()).ok()?;
    let s = std::time::SystemTime::now().duration_since(t).map(|d| d.as_secs_f64()).unwrap_or(0.0);
    Some(if s < 90.0 { format!("{s:.0} s") } else if s < 5400.0 { format!("{:.0} min", s / 60.0) } else if s < 172800.0 { format!("{:.1} h", s / 3600.0) } else { format!("{:.1} d", s / 86400.0) })
}

/// SPACETRACK_USER and whether SPACETRACK_PASS is set, from the engine's .env
fn saved_credentials(cfg: &ControlConfig) -> (String, bool) {
    let txt = std::fs::read_to_string(std::path::Path::new(&cfg.perigee.dir).join(".env")).unwrap_or_default();
    let mut user = String::new(); let mut has_pass = false;
    for line in txt.lines() {
        let Some((k, v)) = line.split_once('=') else { continue };
        let v = v.trim().trim_matches('"').trim_matches('\'');
        match k.trim() { "SPACETRACK_USER" => user = v.to_string(), "SPACETRACK_PASS" => has_pass = !v.is_empty(), _ => {} }
    }
    (user, has_pass)
}

#[derive(Resource)]
pub struct Boot {
    stage: Stage, t: f64, since: f64,
    lines: Vec<Line>, pending: VecDeque<(Kind, String)>, next_emit: f64,
    user: String, pass: String, field: usize, saved_pass: bool, logged_in: bool,
    run: Option<Runner>, rng: u64, shown_log: usize, reloaded: bool, checked: bool,
    nominal_at: Option<f64>,     // Entrance: when every subsystem reached 100 %
}

/// Left behind when the boot page lifts: the control page brings the tiles online one by one from
/// `start` (seconds of app time) and draws the dividers in, then removes this resource.
#[derive(Resource)]
pub struct Reveal { pub start: f64 }
impl Reveal {
    pub const STEP: f64 = 0.22;      // seconds between tiles coming online
    pub const LINES: f64 = 1.1;      // seconds for the dividers to draw themselves
    /// How far along the reveal is for the i-th tile (0 = still dark, 1 = fully on)
    pub fn tile(&self, now: f64, i: usize) -> f64 { ((now - self.start - i as f64 * Self::STEP) / 0.15).clamp(0.0, 1.0) }
    pub fn lines(&self, now: f64) -> f64 { ((now - self.start) / Self::LINES).clamp(0.0, 1.0) }
    pub fn done(&self, now: f64) -> bool { now - self.start > Self::LINES + Self::STEP * 6.0 }
}

/// The subsystems shown coming up on the INITIALIZING table, with the delay before each bar starts
const SUBSYSTEMS: [(&str, f64); 5] = [("ORBIT VIEW", 0.0), ("LIVE DATA", 0.35), ("MOTOR CONTROL", 0.7), ("MOUNT MODEL", 1.05), ("SERIAL LINK", 1.4)];
const BAR_SECONDS: f64 = 1.1;

impl Boot {
    fn rand(&mut self) -> u64 {
        if self.rng == 0 { self.rng = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_nanos() as u64).unwrap_or(0x9E37_79B9_7F4A_7C15) | 1; }
        let mut x = self.rng; x ^= x << 13; x ^= x >> 7; x ^= x << 17; self.rng = x; x
    }
    fn glyph(&mut self) -> char { GLYPHS[(self.rand() % GLYPHS.len() as u64) as usize] as char }
    fn say(&mut self, kind: Kind, text: impl Into<String>) { self.pending.push_back((kind, text.into())); }
    fn go(&mut self, stage: Stage) { self.stage = stage; self.since = self.t; }
    /// A line as it looks `decode` seconds after birth: the first part real, the rest still noise
    fn decoded(&mut self, i: usize, decode: f64) -> String {
        let (text, born, kind) = { let l = &self.lines[i]; (l.text.clone(), l.born, l.kind) };
        let span = if kind == Kind::Title { decode * 2.2 } else { decode }.max(0.01);
        let n = text.chars().count();
        let solved = (((self.t - born) / span).clamp(0.0, 1.0) * n as f64).ceil() as usize;
        if solved >= n { return text; }
        text.chars().enumerate().map(|(k, c)| if k < solved || c.is_whitespace() { c } else { self.glyph() }).collect()
    }
    /// Progress of every subsystem bar during the Entrance (the orbit view stalls at the real propagation)
    fn subsystems(&self, prop: &PropStatus) -> Vec<(String, f64, String)> {
        let el = self.t - self.since;
        SUBSYSTEMS.iter().enumerate().map(|(i, (name, delay))| {
            let timed = ((el - delay) / BAR_SECONDS).clamp(0.0, 1.0);
            let (p, note) = if i == 0 {
                let real = if prop.total == 0 { 1.0 } else if prop.finished { 1.0 } else { prop.done as f64 / prop.total as f64 };
                let p = timed.min(real.max(0.02));
                (p, if p >= 1.0 { "READY".to_string() } else if timed >= 1.0 { format!("PROPAGATING {}/{}", prop.done, prop.total) } else { "LOADING".to_string() })
            } else {
                (timed, if timed >= 1.0 { if i == 4 { "ONLINE".to_string() } else { "READY".to_string() } } else if timed > 0.0 { "LOADING".to_string() } else { "STANDBY".to_string() })
            };
            (name.to_string(), p, note)
        }).collect()
    }
}

#[derive(Component)] struct BootRoot;
#[derive(Component)] struct BootLogSpan(usize);
#[derive(Component)] struct BootPrompt;
#[derive(Component)] struct BootHint;
#[derive(Component)] struct RainCol { x: f32, y: f32, speed: f32, len: usize, next: f64 }

pub struct BootPlugin;
impl Plugin for BootPlugin {
    fn build(&self, app: &mut App) {
        //PERIGEE_BOOT=0 in the environment skips the boot page whatever control.toml says (for testing)
        let enabled = app.world().get_resource::<ControlConfig>().map_or(true, |c| c.boot.enabled)
            && std::env::var("PERIGEE_BOOT").map_or(true, |v| v != "0");
        app.insert_resource(Booting(enabled))
            .insert_resource(Boot {
                stage: Stage::Check, t: 0.0, since: 0.0, lines: Vec::new(), pending: VecDeque::new(), next_emit: 0.0,
                user: String::new(), pass: String::new(), field: 0, saved_pass: false, logged_in: false,
                run: None, rng: 0, shown_log: 0, reloaded: false, checked: false, nominal_at: None,
            })
            .add_systems(PostStartup, spawn_boot_ui)
            .add_systems(Update, (boot_tick, boot_rain, boot_render).chain().run_if(|b: Res<Booting>| b.0));
    }
}

fn spawn_boot_ui(
    mut commands: Commands, booting: Res<Booting>, cam: Res<CmdCamera>, font: Res<UiFont>, pal: Res<Palette>, cfg: Res<ControlConfig>,
    windows: Query<&Window, With<PrimaryWindow>>, mut boot: ResMut<Boot>,
) {
    if !booting.0 { return; }
    let Some(cam) = cam.0 else { return };
    let fs = cfg.boot.font_size;
    let tf = |size: f32| TextFont { font: font.0.clone(), font_size: size, ..default() };
    let (ww, wh) = windows.get_single().map(|w| (w.width(), w.height())).unwrap_or((cfg.window.width, cfg.window.height));
    let (user, has_pass) = saved_credentials(&cfg);
    boot.user = user; boot.saved_pass = has_pass;

    commands.spawn((
        Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), width: Val::Percent(100.0), height: Val::Percent(100.0), overflow: Overflow::clip(), ..default() },
        BackgroundColor(pal.space), GlobalZIndex(40), TargetCamera(cam), BootRoot, CmdUi, Name::new("boot page"),
    )).with_children(|root| {
        //Glyph rain: one text column each, moved and re-lettered by boot_rain
        if cfg.boot.rain {
            let step = fs * 2.6;
            let cols = ((ww / step) as usize).clamp(8, 72);
            for i in 0..cols {
                let x = i as f32 * step + (boot.rand() % 9) as f32;
                let len = 5 + (boot.rand() % 14) as usize;
                let speed = 35.0 + (boot.rand() % 120) as f32;
                let y = -((boot.rand() % (wh.max(1.0) as u64 + 1)) as f32);
                let text: String = (0..len).map(|_| boot.glyph()).map(|c| format!("{c}\n")).collect();
                root.spawn((
                    Node { position_type: PositionType::Absolute, left: Val::Px(x), top: Val::Px(y), ..default() },
                    Text::new(text), tf(fs), TextColor(pal.dim.with_alpha(0.16)), TextLayout::new_with_no_wrap(), PickingBehavior::IGNORE,
                    RainCol { x, y, speed, len, next: 0.0 },
                ));
            }
        }
        //The page: designation, a rule, the log (one span per row), the prompt, the key legend
        root.spawn(Node { position_type: PositionType::Absolute, left: Val::Percent(6.0), top: Val::Percent(5.0), width: Val::Percent(88.0), bottom: Val::Percent(4.0),
                          flex_direction: FlexDirection::Column, row_gap: Val::Px(12.0), ..default() })
            .with_children(|col| {
                col.spawn((Text::new("PERIGEE"), tf(fs * 3.4), TextColor(pal.bright), TextLayout::new_with_no_wrap()));
                col.spawn((Text::new(format!("GROUND STATION CONTROL SYSTEM   REV {}   {}", env!("CARGO_PKG_VERSION"), chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC"))),
                           tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap()));
                col.spawn((Node { height: Val::Px(2.0), width: Val::Percent(100.0), margin: UiRect::vertical(Val::Px(4.0)), ..default() }, BackgroundColor(pal.divider)));
                col.spawn((Text::new(""), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap(), Node { flex_shrink: 0.0, ..default() }))
                    .with_children(|log| { for i in 0..MAX_ROWS { log.spawn((TextSpan::new(""), tf(fs), TextColor(pal.text), BootLogSpan(i))); } });
                col.spawn((Text::new(""), tf(fs), TextColor(pal.bright), TextLayout::new_with_no_wrap(), BootPrompt, Node { flex_shrink: 0.0, margin: UiRect::top(Val::Px(6.0)), ..default() }));
                col.spawn((Text::new(""), tf(fs - 1.0), TextColor(pal.dim), TextLayout::new_with_no_wrap(), BootHint, Node { flex_shrink: 0.0, margin: UiRect::top(Val::Px(4.0)), ..default() }));
            });
    });
}

/// The system check: what this program found around itself, one fact per line
fn queue_checks(boot: &mut Boot, cfg: &ControlConfig, vcfg: &ViewerCfg, log: &BootLog) {
    boot.say(Kind::Title, "SYSTEM CHECK");
    let have_toml = std::path::Path::new("control.toml").is_file();
    boot.say(if have_toml { Kind::Ok } else { Kind::Warn }, if have_toml { "[ OK ] CONFIGURATION     control.toml".to_string() } else { "[ -- ] CONFIGURATION     control.toml NOT FOUND. BUILT-IN DEFAULTS IN EFFECT".to_string() });
    let lat = vcfg.station.lat_deg; let lon = vcfg.station.lon_deg;
    boot.say(Kind::Ok, format!("[ OK ] VIEWER            {}   STATION {}   {:.4}{} {:.4}{}   MASK {:.0} DEG",
                               cfg.viewer.config, vcfg.station.name.to_uppercase(), lat.abs(), if lat >= 0.0 { "N" } else { "S" }, lon.abs(), if lon >= 0.0 { "E" } else { "W" }, vcfg.station.elevation_mask_deg));
    let dir = std::path::Path::new(if cfg.data.dir.is_empty() { "." } else { &cfg.data.dir });
    if dir.is_dir() {
        let ages: Vec<String> = ["ELSET.json", "SORTED_SATS.json", "NORADs.json", "SATELLITE_RANKS.json", "CATEGORIES.json"].iter()
            .map(|f| match age_of(&dir.join(f)) { Some(a) => format!("{f} {}", a.to_uppercase()), None => format!("{f} MISSING") }).collect();
        boot.say(Kind::Ok, format!("[ OK ] DATA STORE        {}", dir.display()));
        boot.say(Kind::Dim, format!("                         {}", ages.join("   ")));
    } else {
        boot.say(Kind::Err, format!("[FAIL] DATA STORE        {} IS NOT A DIRECTORY", dir.display()));
    }
    match engine_status(cfg) {
        Ok(s) => boot.say(Kind::Ok, format!("[ OK ] ENGINE            {}", s.replace("built", "BUILT"))),
        Err(s) => boot.say(Kind::Warn, format!("[ -- ] ENGINE            {}", s.to_uppercase())),
    }
    let ports = SerialLink::scan_ports();
    if ports.is_empty() {
        boot.say(if cfg.serial.simulate { Kind::Warn } else { Kind::Err }, format!("[ -- ] SERIAL LINK       NO PORT DETECTED. {}", if cfg.serial.simulate { "MOUNT SIMULATOR ENGAGED" } else { "SIMULATION DISABLED: NO MOUNT" }));
    } else {
        boot.say(Kind::Ok, format!("[ OK ] SERIAL LINK       {}   {} BAUD", ports.join("  "), cfg.serial.baud));
    }
    let m = &cfg.mount;
    boot.say(Kind::Ok, format!("[ OK ] MOUNT FRAME       AZ 0..{:.0}   EL {:.0}..{:.0}   CENTRE BEARING {:.1}   PARK {:.0}/{:.0}   RATE {:.0}/{:.0} DEG/S",
                               m.az_travel_deg, m.el_min_deg, m.el_max_deg, m.az_center_bearing_deg, m.park_az_deg, m.park_el_deg, m.az_rate_dps, m.el_rate_dps));
    boot.say(Kind::Dim, "");
    boot.say(Kind::Title, "VIEWER DATA");
    for l in &log.0 { boot.say(Kind::Info, format!("       {l}")); }
    boot.shown_log = log.0.len();
    boot.say(Kind::Dim, "");
    boot.say(Kind::Title, "AUTHENTICATION REQUIRED   SPACE-TRACK.ORG");
}

fn start_engine(boot: &mut Boot, cfg: &ControlConfig, kind: RunKind, title: &str) {
    let (u, p) = if kind == RunKind::Rank { (String::new(), String::new()) } else { (boot.user.clone(), boot.pass.clone()) };
    boot.say(Kind::Title, title);
    match Runner::spawn(cfg, kind, &u, &p) {
        Ok((r, desc)) => { boot.say(Kind::Dim, format!("       EXEC {desc}")); boot.run = Some(r); boot.go(Stage::Run); }
        Err(e) => boot.say(Kind::Err, format!("[FAIL] {}", e.to_uppercase())),
    }
}

fn boot_tick(
    time: Res<Time>, input: Res<CmdInput>, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, mut boot: ResMut<Boot>, log: Res<BootLog>,
    prop: Res<PropStatus>, mut reload: EventWriter<ReloadData>, mut booting: ResMut<Booting>, mut commands: Commands,
    roots: Query<Entity, With<BootRoot>>,
) {
    boot.t += time.delta_secs_f64();
    let t = boot.t;
    //PERIGEE_BOOT_AUTO=1 takes the direct-entry path by itself once the checks are up; =demo presses Enter at
    //the logon (credential on file) and data prompts after a pause, as an operator would (for tests, recordings)
    let auto = std::env::var("PERIGEE_BOOT_AUTO").unwrap_or_default();
    let enter = input.pressed(KeyCode::Enter) || input.pressed(KeyCode::NumpadEnter)
        || (auto == "demo" && boot.run.is_none() && boot.pending.is_empty() && match boot.stage { Stage::Login => t - boot.since > 2.2, Stage::Menu => t - boot.since > 2.6, _ => false });
    let escape = input.pressed(KeyCode::Escape)
        || (auto == "1" && boot.stage == Stage::Check && boot.checked && boot.pending.is_empty());

    //Lines the viewer printed since we last looked (a reload after the engine ran)
    if boot.checked && log.0.len() > boot.shown_log {
        for l in &log.0[boot.shown_log..] { boot.say(Kind::Info, format!("       VIEWER  {l}")); }
        boot.shown_log = log.0.len();
    }
    //Queued lines appear one at a time
    while boot.pending.front().is_some() && t >= boot.next_emit {
        let (kind, text) = boot.pending.pop_front().unwrap();
        boot.lines.push(Line { text, kind, born: t });
        boot.next_emit = t + cfg.boot.line_seconds.max(0.0);
    }
    //Engine output, whatever the stage
    let mut finished: Option<(RunKind, i32, f64)> = None;
    if let Some(run) = boot.run.as_mut() {
        let lines = run.poll(t);
        let kind = run.kind;
        let secs = run.secs();
        let fin = run.finished(t).map(|c| (kind, c, secs));
        for l in lines { boot.say(Kind::Engine, format!("       ENGINE> {l}")); }
        finished = fin;
    }
    if let Some((kind, code, secs)) = finished {
        boot.run = None;
        match (kind, code) {
            (RunKind::Login, 0) => { boot.logged_in = true; boot.say(Kind::Title, "SESSION ESTABLISHED"); boot.say(Kind::Dim, ""); boot.go(Stage::Menu); }
            (RunKind::Login, c) => { boot.say(Kind::Err, format!("[FAIL] AUTHENTICATION FAILED   CREDENTIALS REJECTED (EXIT {c})")); boot.field = 1; boot.go(Stage::Login); }
            (k, 0) => {
                boot.say(Kind::Ok, format!("[ OK ] ENGINE COMPLETE IN {secs:.1} S   RELOADING VIEWER DATA"));
                reload.send(ReloadData { elsets: k == RunKind::Full });
                boot.reloaded = true;
                boot.go(Stage::Menu);
            }
            (_, c) => { boot.say(Kind::Err, format!("[FAIL] ENGINE TERMINATED   EXIT {c} AFTER {secs:.1} S")); boot.go(Stage::Menu); }
        }
        return;
    }

    match boot.stage {
        Stage::Check => {
            if !boot.checked { boot.checked = true; queue_checks(&mut boot, &cfg, &vcfg, &log); boot.since = t; }
            if escape { boot.say(Kind::Warn, "[ -- ] DIRECT ENTRY"); boot.go(Stage::Entrance); }
            else if boot.pending.is_empty() && (t - boot.next_emit > 0.5 || enter) { boot.field = if boot.user.is_empty() { 0 } else { 1 }; boot.go(Stage::Login); }
        }
        Stage::Login => {
            let typed = input.typed();
            if !typed.is_empty() { if boot.field == 0 { boot.user.push_str(&typed); } else { boot.pass.push_str(&typed); } }
            for p in &input.presses {
                match p.code {
                    KeyCode::Backspace => { if boot.field == 0 { boot.user.pop(); } else { boot.pass.pop(); } }
                    KeyCode::Tab | KeyCode::ArrowDown | KeyCode::ArrowUp => boot.field = 1 - boot.field,
                    KeyCode::F2 => { boot.say(Kind::Warn, "[ -- ] LOGON BYPASSED   CATALOG REFRESH UNAVAILABLE"); boot.say(Kind::Dim, ""); boot.go(Stage::Menu); }
                    KeyCode::Escape => { boot.say(Kind::Warn, "[ -- ] DIRECT ENTRY"); boot.go(Stage::Entrance); }
                    _ => {}
                }
            }
            if enter && boot.stage == Stage::Login {
                if boot.field == 0 && !boot.user.is_empty() { boot.field = 1; }
                else if boot.user.is_empty() { boot.say(Kind::Err, "[FAIL] IDENTIFICATION REQUIRED"); }
                else if boot.pass.is_empty() && !boot.saved_pass { boot.say(Kind::Err, "[FAIL] PASSWORD REQUIRED   NO CREDENTIAL ON FILE"); }
                else {
                    let (u, p) = (boot.user.clone(), boot.pass.clone());
                    boot.say(Kind::Info, format!("       CONNECTING   space-track.org   IDENT {u}{}", if p.is_empty() { "   CREDENTIAL ON FILE" } else { "" }));
                    match Runner::spawn(&cfg, RunKind::Login, &u, &p) {
                        Ok((r, desc)) => { boot.say(Kind::Dim, format!("       EXEC {desc}")); boot.run = Some(r); boot.go(Stage::Verify); }
                        Err(e) => boot.say(Kind::Err, format!("[FAIL] {}", e.to_uppercase())),
                    }
                }
            }
        }
        Stage::Verify => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Warn, "[ -- ] LOGON ABORTED"); boot.go(Stage::Login); }
        }
        Stage::Menu => {
            if enter && !input.presses.iter().any(|p| matches!(p.code, KeyCode::Enter | KeyCode::NumpadEnter)) { boot.say(Kind::Info, "       PROCEEDING WITH DATA ON FILE"); boot.go(Stage::Entrance); }
            for p in &input.presses {
                match p.code {
                    KeyCode::Enter | KeyCode::NumpadEnter | KeyCode::KeyS | KeyCode::Escape => { boot.say(Kind::Info, "       PROCEEDING WITH DATA ON FILE"); boot.go(Stage::Entrance); }
                    KeyCode::KeyF if boot.logged_in => start_engine(&mut boot, &cfg, RunKind::Full, "FULL CATALOG REFRESH"),
                    KeyCode::KeyF => boot.say(Kind::Warn, "[ -- ] CATALOG REFRESH REQUIRES A SPACE-TRACK SESSION   L TO LOG ON"),
                    KeyCode::KeyR => start_engine(&mut boot, &cfg, RunKind::Rank, "RE-RANK"),
                    KeyCode::KeyL => { boot.field = 1; boot.go(Stage::Login); }
                    _ => {}
                }
                if boot.stage != Stage::Menu { break; }
            }
        }
        Stage::Run => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Warn, "[ -- ] ENGINE STOPPED BY OPERATOR"); boot.go(Stage::Menu); }
        }
        Stage::Entrance => {
            if boot.nominal_at.is_none() && t - boot.since > 0.2 && boot.lines.last().map_or(true, |l| l.text != "INITIALIZING SUBSYSTEMS") && boot.pending.is_empty() && t - boot.since < 0.6 {
                boot.say(Kind::Title, "INITIALIZING SUBSYSTEMS");
            }
            let all_up = boot.subsystems(&prop).iter().all(|(_, p, _)| *p >= 1.0);
            if boot.nominal_at.is_none() && (all_up || enter || escape) {
                boot.nominal_at = Some(t);
                boot.say(Kind::Title, "ALL SYSTEMS NOMINAL");
                boot.say(Kind::Title, "ENTERING CONTROL");
            }
            if let Some(n) = boot.nominal_at { if t - n > 1.4 { boot.go(Stage::Exit); } }
        }
        Stage::Exit => {
            booting.0 = false;
            for e in &roots { commands.entity(e).despawn_recursive(); }
            commands.insert_resource(Reveal { start: time.elapsed_secs_f64() });
            println!("boot page done{}", if boot.reloaded { " (viewer reloaded)" } else { "" });
        }
    }
}

fn boot_rain(time: Res<Time>, windows: Query<&Window, With<PrimaryWindow>>, mut boot: ResMut<Boot>, cfg: Res<ControlConfig>, mut cols: Query<(&mut RainCol, &mut Node, &mut Text)>) {
    let Ok(w) = windows.get_single() else { return };
    let (ww, wh) = (w.width(), w.height());
    let line_h = cfg.boot.font_size * 1.2;
    //The rain rushes while the subsystems come up
    let rush: f32 = if boot.stage == Stage::Entrance { 4.0 } else { 1.0 };
    let dt = time.delta_secs() * rush;
    let t = boot.t;
    for (mut c, mut node, mut text) in &mut cols {
        c.y += c.speed * dt;
        if c.y > wh + 10.0 {
            c.y = -(c.len as f32 * line_h) - (boot.rand() % 400) as f32;
            c.x = (boot.rand() % (ww.max(1.0) as u64 + 1)) as f32;
            c.speed = 35.0 + (boot.rand() % 120) as f32;
            node.left = Val::Px(c.x);
        }
        node.top = Val::Px(c.y);
        if t >= c.next {
            c.next = t + (0.12 + (boot.rand() % 20) as f64 * 0.01) / rush as f64;
            let n = c.len;
            text.0 = (0..n).map(|_| boot.glyph()).map(|g| format!("{g}\n")).collect();
        }
    }
}

fn bar(p: f64, width: usize) -> String {
    let filled = (p.clamp(0.0, 1.0) * width as f64).round() as usize;
    format!("[{}{}]", "\u{2588}".repeat(filled), " ".repeat(width - filled))
}

fn boot_render(
    mut boot: ResMut<Boot>, cfg: Res<ControlConfig>, pal: Res<Palette>, windows: Query<&Window, With<PrimaryWindow>>, prop: Res<PropStatus>,
    mut spans: Query<(&BootLogSpan, &mut TextSpan, &mut TextColor)>,
    mut prompt: Query<&mut Text, (With<BootPrompt>, Without<BootHint>)>,
    mut hint: Query<&mut Text, (With<BootHint>, Without<BootPrompt>)>,
) {
    let fs = cfg.boot.font_size;
    let line_h = fs * 1.2;
    let wh = windows.get_single().map(|w| w.height()).unwrap_or(cfg.window.height);
    //Rows the log may use: the page minus the header, the prompt block (up to 7 lines) and the legend
    let rows = (((wh * 0.91 - fs * 3.4 * 1.2 - line_h - 40.0 - line_h * 9.0) / line_h).floor() as usize).clamp(6, MAX_ROWS);
    let n = boot.lines.len();
    let first = n.saturating_sub(rows);
    let amber = Color::srgb(1.0, 0.78, 0.35);
    let decode = cfg.boot.decode_seconds;
    let mut texts: Vec<(String, Color)> = Vec::with_capacity(rows);
    for i in first..n {
        let kind = boot.lines[i].kind;
        let s = boot.decoded(i, decode);
        let c = match kind { Kind::Info => pal.text, Kind::Ok => pal.good, Kind::Warn => amber, Kind::Err => pal.warn, Kind::Engine => pal.bright, Kind::Title => pal.border_focus, Kind::Dim => pal.dim };
        texts.push((s, c));
    }
    for (span, mut ts, mut col) in &mut spans {
        match texts.get(span.0) {
            Some((s, c)) => { let want = format!("{s}\n"); if ts.0 != want { ts.0 = want; } col.0 = *c; }
            None => { if !ts.0.is_empty() { ts.0.clear(); } }
        }
    }

    let blink = (boot.t * 2.5) as i64 % 2 == 0;
    let cur = if blink { "\u{2588}" } else { " " };
    let spinner = ['|', '/', '-', '\\'][((boot.t * 8.0) as usize) % 4];
    let stars: String = "*".repeat(boot.pass.chars().count());
    let (p, h): (String, String) = match boot.stage {
        Stage::Check => (format!("{cur}"), "ENTER  CONTINUE          ESC  DIRECT ENTRY".into()),
        Stage::Login => (
            format!("LOGON:\n  IDENTIFICATION   {}{}\n  PASSWORD         {}{}{}", boot.user, if boot.field == 0 { cur } else { "" }, stars, if boot.field == 1 { cur } else { "" },
                    if boot.pass.is_empty() && boot.saved_pass && boot.field == 1 { "   CREDENTIAL ON FILE: ENTER TO USE" } else { "" }),
            "ENTER  SUBMIT          TAB  FIELD          F2  BYPASS LOGON          ESC  DIRECT ENTRY".into()),
        Stage::Verify => (format!("  {spinner}  AUTHENTICATING   space-track.org   {:.1} S", boot.run.as_ref().map_or(0.0, |r| r.secs())), "ESC  ABORT".into()),
        Stage::Menu => (
            format!("DATA:\n  [ENTER]  PROCEED WITH DATA ON FILE\n  [F]      FULL CATALOG REFRESH     SPACE-TRACK + SATNOGS, PROPAGATE, RANK   (ABOUT ONE MINUTE){}\n  [R]      RE-RANK                  DATA ON FILE, CURRENT TIME{}",
                    if boot.logged_in { "" } else { "   [SESSION REQUIRED]" }, if boot.logged_in { "" } else { "\n  [L]      LOG ON" }),
            "THE ENGINE WRITES INTO THE DATA STORE. THE VIEWER RELOADS WHEN IT COMPLETES.".into()),
        Stage::Run => (format!("  {spinner}  ENGINE RUNNING   {:.0} S", boot.run.as_ref().map_or(0.0, |r| r.secs())), "ESC  STOP ENGINE".into()),
        Stage::Entrance => {
            let subs = boot.subsystems(&prop);
            let table: Vec<String> = subs.iter().map(|(name, p, note)| format!("  {name:<16} {}  {:3.0}%   {note}", bar(*p, 30), p * 100.0)).collect();
            (table.join("\n"), if boot.nominal_at.is_some() { String::new() } else { "ENTER  SKIP".into() })
        }
        Stage::Exit => (String::new(), String::new()),
    };
    for mut t in &mut prompt { if t.0 != p { t.0 = p.clone(); } }
    for mut t in &mut hint { if t.0 != h { t.0 = h.clone(); } }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The boot page: what the window shows before the tiles. A plain WarGames teletype: one phosphor green
/// on black, every line typed out a character at a time behind a block cursor:
///
///   SYSTEM CHECK    settings, data store and file ages, the engine binary, serial ports, the mount
///                   frame, then everything the viewer said while loading (the lines Perigee prints)
///   LOGON           Space-Track identification and password (prefilled from the engine's .env),
///                   verified by running `perigee login`, whose lines stream in: SESSION ESTABLISHED or
///                   IDENTIFICATION NOT RECOGNIZED BY SYSTEM
///   DATA            proceed with the cached data, or run the engine first (full catalog refresh, re-rank)
///   LOADING         waits for the viewer's propagation, ENTERING CONTROL, then the page lifts and the
///                   tiles come online one after another (`Reveal`, read by the control page)
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

const MAX_ROWS: usize = 64;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Stage { Check, Login, Verify, Menu, Run, Entrance, Exit }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind { Text, Loud, Dim }   // the phosphor green, brighter (failures, warnings), dimmer (detail)

/// A log line and how many of its characters the teletype has printed so far
struct Line { text: String, kind: Kind, len: usize, shown: usize }

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RunKind { Login, Full, Rank }

/// A running engine process and the lines it has printed so far
struct Runner { child: Child, rx: Mutex<Receiver<String>>, started: std::time::Instant, kind: RunKind, exit: Option<i32>, exit_at: Option<f64> }

impl Runner {
    /// Start `perigee <sub>` in the data folder with the credentials in its environment (an empty
    /// value leaves the engine's own .env in charge of that variable)
    fn spawn(cfg: &ControlConfig, kind: RunKind, user: &str, pass: &str, orbits: &str) -> Result<(Runner, String), String> {
        let sub = match kind { RunKind::Login => Some("login"), RunKind::Full => None, RunKind::Rank => Some("rank") };
        let (mut cmd, desc) = engine_command(cfg, sub)?;
        let cwd = if cfg.data.dir.is_empty() { cfg.perigee.dir.clone() } else { cfg.data.dir.clone() };
        cmd.current_dir(&cwd).stdin(Stdio::null()).stdout(Stdio::piped()).stderr(Stdio::piped());
        if !user.is_empty() { cmd.env("SPACETRACK_USER", user); }
        if !pass.is_empty() { cmd.env("SPACETRACK_PASS", pass); }
        //Which orbits a full refresh pulls; the engine defaults to LEO on its own if this is empty
        if kind == RunKind::Full && !orbits.is_empty() { cmd.env("PERIGEE_ORBITS", orbits); }
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
    lines: Vec<Line>, pending: VecDeque<(Kind, String)>,
    next_emit: f64,              // when the teletype may start the next line (the last one finished + line_seconds)
    budget: f64,                 // characters the teletype owes this frame
    user: String, pass: String, field: usize, saved_pass: bool, logged_in: bool,
    run: Option<Runner>, shown_log: usize, reloaded: bool, checked: bool,
    leaving_at: Option<f64>,     // Entrance: when ENTERING CONTROL was queued
    orbits: String,              // LEO / GEO / ALL for the next full refresh; seeded from [perigee] orbits
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

impl Boot {
    fn say(&mut self, kind: Kind, text: impl Into<String>) { self.pending.push_back((kind, text.into())); }
    fn go(&mut self, stage: Stage) { self.stage = stage; self.since = self.t; }
    /// Nothing left to type: the cursor belongs to the prompt
    fn idle(&self) -> bool { self.pending.is_empty() && self.lines.last().map_or(true, |l| l.shown >= l.len) }
    /// Print `dt` seconds' worth of characters, one line after another. A backlog (the engine talking
    /// fast) speeds the teletype up so it never falls far behind.
    fn type_out(&mut self, dt: f64, cps: f64, pause: f64) {
        self.budget += dt * cps.max(1.0) * (1.0 + self.pending.len() as f64 * 0.5);
        loop {
            if let Some(l) = self.lines.last_mut().filter(|l| l.shown < l.len) {
                let k = (self.budget.floor() as usize).min(l.len - l.shown);
                if k == 0 { return; }
                l.shown += k; self.budget -= k as f64;
                if l.shown >= l.len { self.next_emit = self.t + pause; }
                continue;
            }
            if self.t < self.next_emit { self.budget = 0.0; return; }
            let Some((kind, text)) = self.pending.pop_front() else { self.budget = 0.0; return };
            let len = text.chars().count();
            if len == 0 { self.next_emit = self.t + pause; }
            self.lines.push(Line { text, kind, len, shown: 0 });
        }
    }
}

#[derive(Component)] struct BootRoot;
#[derive(Component)] struct BootLogSpan(usize);
#[derive(Component)] struct BootPrompt;
#[derive(Component)] struct BootHint;

pub struct BootPlugin;
impl Plugin for BootPlugin {
    fn build(&self, app: &mut App) {
        //PERIGEE_BOOT=0 in the environment skips the boot page whatever control.toml says (for testing)
        let enabled = app.world().get_resource::<ControlConfig>().map_or(true, |c| c.boot.enabled)
            && std::env::var("PERIGEE_BOOT").map_or(true, |v| v != "0");
        app.insert_resource(Booting(enabled))
            .insert_resource(Boot {
                stage: Stage::Check, t: 0.0, since: 0.0, lines: Vec::new(), pending: VecDeque::new(), next_emit: 0.0, budget: 0.0,
                user: String::new(), pass: String::new(), field: 0, saved_pass: false, logged_in: false,
                run: None, shown_log: 0, reloaded: false, checked: false, leaving_at: None,
                orbits: String::new(),
            })
            .add_systems(PostStartup, spawn_boot_ui)
            .add_systems(Update, (boot_tick, boot_render).chain().run_if(|b: Res<Booting>| b.0));
    }
}

fn spawn_boot_ui(
    mut commands: Commands, booting: Res<Booting>, cam: Res<CmdCamera>, font: Res<UiFont>, pal: Res<Palette>, cfg: Res<ControlConfig>,
    mut boot: ResMut<Boot>,
) {
    if !booting.0 { return; }
    let Some(cam) = cam.0 else { return };
    let fs = cfg.boot.font_size;
    let tf = |size: f32| TextFont { font: font.0.clone(), font_size: size, ..default() };
    let (user, has_pass) = saved_credentials(&cfg);
    boot.user = user; boot.saved_pass = has_pass;

    //Black page; the log (one span per row), the prompt under it, the key legend under that
    commands.spawn((
        Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), width: Val::Percent(100.0), height: Val::Percent(100.0), overflow: Overflow::clip(), ..default() },
        BackgroundColor(pal.space), GlobalZIndex(40), TargetCamera(cam), BootRoot, CmdUi, Name::new("boot page"),
    )).with_children(|root| {
        root.spawn(Node { position_type: PositionType::Absolute, left: Val::Percent(6.0), top: Val::Percent(5.0), width: Val::Percent(88.0), bottom: Val::Percent(4.0),
                          flex_direction: FlexDirection::Column, row_gap: Val::Px(12.0), ..default() })
            .with_children(|col| {
                col.spawn((Text::new(""), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap(), Node { flex_shrink: 0.0, ..default() }))
                    .with_children(|log| { for i in 0..MAX_ROWS { log.spawn((TextSpan::new(""), tf(fs), TextColor(pal.text), BootLogSpan(i))); } });
                col.spawn((Text::new(""), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap(), BootPrompt, Node { flex_shrink: 0.0, ..default() }));
                col.spawn((Text::new(""), tf(fs - 1.0), TextColor(pal.dim), TextLayout::new_with_no_wrap(), BootHint, Node { flex_shrink: 0.0, margin: UiRect::top(Val::Px(4.0)), ..default() }));
            });
    });
}

/// The system check: what this program found around itself, one fact per line
fn queue_checks(boot: &mut Boot, cfg: &ControlConfig, vcfg: &ViewerCfg, log: &BootLog) {
    boot.say(Kind::Text, format!("PERIGEE GROUND STATION CONTROL   REV {}", env!("CARGO_PKG_VERSION")));
    boot.say(Kind::Dim, chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC").to_string());
    boot.say(Kind::Text, "");
    boot.say(Kind::Text, "SYSTEM CHECK");
    if std::path::Path::new("control.toml").is_file() { boot.say(Kind::Text, "  CONFIGURATION   control.toml"); }
    else { boot.say(Kind::Loud, "  CONFIGURATION   control.toml NOT FOUND. BUILT-IN DEFAULTS IN EFFECT"); }
    let lat = vcfg.station.lat_deg; let lon = vcfg.station.lon_deg;
    boot.say(Kind::Text, format!("  VIEWER          {}   STATION {}   {:.4}{} {:.4}{}   MASK {:.0} DEG",
                               cfg.viewer.config, vcfg.station.name.to_uppercase(), lat.abs(), if lat >= 0.0 { "N" } else { "S" }, lon.abs(), if lon >= 0.0 { "E" } else { "W" }, vcfg.station.elevation_mask_deg));
    let dir = std::path::Path::new(if cfg.data.dir.is_empty() { "." } else { &cfg.data.dir });
    if dir.is_dir() {
        boot.say(Kind::Text, format!("  DATA STORE      {}", dir.display()));
        for f in ["ELSET.json", "SORTED_SATS.json", "NORADs.json", "SATELLITE_RANKS.json", "CATEGORIES.json"] {
            match age_of(&dir.join(f)) {
                Some(a) => boot.say(Kind::Dim, format!("                  {f:<22}{}", a.to_uppercase())),
                None => boot.say(Kind::Loud, format!("                  {f:<22}MISSING")),
            }
        }
    } else {
        boot.say(Kind::Loud, format!("  DATA STORE      {} IS NOT A DIRECTORY", dir.display()));
    }
    match engine_status(cfg) {
        Ok(s) => boot.say(Kind::Text, format!("  ENGINE          {}", s.replace("built", "BUILT"))),
        Err(s) => boot.say(Kind::Loud, format!("  ENGINE          {}", s.to_uppercase())),
    }
    let ports = SerialLink::scan_ports();
    if ports.is_empty() {
        boot.say(Kind::Loud, format!("  SERIAL LINK     NO PORT DETECTED. {}", if cfg.serial.simulate { "MOUNT SIMULATOR ENGAGED" } else { "SIMULATION DISABLED: NO MOUNT" }));
    } else {
        boot.say(Kind::Text, format!("  SERIAL LINK     {}   {} BAUD", ports.join("  "), cfg.serial.baud));
    }
    let m = &cfg.mount;
    boot.say(Kind::Text, format!("  MOUNT FRAME     AZ {:.0}..{:.0} OF {:.0} TRAVEL   EL {:.0}..{:.0}",
                               m.az_limit_lo_deg, m.az_limit_hi_deg, m.az_travel_deg, m.el_min_deg, m.el_max_deg));
    //The sky tie is the measured part of the frame, so say plainly when it is a guess:
    //an uncalibrated mount still draws and still simulates, it just will not track real hardware.
    match m.az_center_bearing_deg {
        Some(b) => boot.say(Kind::Text, format!("  CALIBRATION     CENTRE BEARING {b:.2} DEG{}, MEASURED   FROM {}",
                                                if m.el_correction_deg != 0.0 { format!("   EL CORRECTION {:+.2} DEG", m.el_correction_deg) } else { String::new() },
                                                cfg.calibration.file)),
        None => boot.say(Kind::Loud, format!("  CALIBRATION     UNCALIBRATED. FRAME IS NOMINAL {:.1} DEG; SIMULATOR ONLY, TRACKING A REAL MOUNT REFUSED (/HERE AZ EL TO TIE IT)", m.az_center_or_nominal())),
    }
    boot.say(Kind::Text, format!("  CABLE WRAP      AZ HELD INSIDE {:.0}..{:.0} DEG   PARK {:.0}/{:.0}   RATE {:.0}/{:.0} DEG/S",
                               m.az_limit_lo_deg, m.az_limit_hi_deg, m.park_az_deg, m.park_el_deg, m.az_rate_dps, m.el_rate_dps));
    boot.say(Kind::Text, "");
    boot.say(Kind::Text, "VIEWER DATA");
    for l in &log.0 { boot.say(Kind::Dim, format!("  {l}")); }
    boot.shown_log = log.0.len();
    boot.say(Kind::Text, "");
}

fn start_engine(boot: &mut Boot, cfg: &ControlConfig, kind: RunKind, title: &str) {
    let (u, p) = if kind == RunKind::Rank { (String::new(), String::new()) } else { (boot.user.clone(), boot.pass.clone()) };
    let orbits = boot.orbits.clone();
    boot.say(Kind::Text, "");
    boot.say(Kind::Text, title);
    if kind == RunKind::Full { boot.say(Kind::Dim, format!("  ORBITS {}", orbits.to_uppercase())); }
    match Runner::spawn(cfg, kind, &u, &p, &orbits) {
        Ok((r, desc)) => { boot.say(Kind::Dim, format!("  EXEC {desc}")); boot.run = Some(r); boot.go(Stage::Run); }
        Err(e) => boot.say(Kind::Loud, format!("  {}", e.to_uppercase())),
    }
}

fn boot_tick(
    time: Res<Time>, input: Res<CmdInput>, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, mut boot: ResMut<Boot>, log: Res<BootLog>,
    prop: Res<PropStatus>, mut reload: EventWriter<ReloadData>, mut booting: ResMut<Booting>, mut commands: Commands,
    roots: Query<Entity, With<BootRoot>>,
) {
    boot.t += time.delta_secs_f64();
    let t = boot.t;
    //Which orbits the next full refresh pulls: from [perigee] orbits until O cycles it
    if boot.orbits.is_empty() {
        let o = cfg.perigee.orbits.trim().to_lowercase();
        boot.orbits = if matches!(o.as_str(), "geo" | "all") { o } else { "leo".into() };
    }
    //PERIGEE_BOOT_AUTO=1 takes the direct-entry path by itself once the checks are up; =demo presses Enter at
    //the logon (credential on file) and data prompts after a pause, as an operator would (for tests, recordings)
    let auto = std::env::var("PERIGEE_BOOT_AUTO").unwrap_or_default();
    let enter = input.pressed(KeyCode::Enter) || input.pressed(KeyCode::NumpadEnter)
        || (auto == "demo" && boot.run.is_none() && boot.idle() && match boot.stage { Stage::Login => t - boot.since > 2.2, Stage::Menu => t - boot.since > 2.6, _ => false });
    let escape = input.pressed(KeyCode::Escape)
        || (auto == "1" && boot.stage == Stage::Check && boot.checked && boot.idle());

    //Lines the viewer printed since we last looked (a reload after the engine ran)
    if boot.checked && log.0.len() > boot.shown_log {
        for l in &log.0[boot.shown_log..] { boot.say(Kind::Dim, format!("  VIEWER  {l}")); }
        boot.shown_log = log.0.len();
    }
    //The teletype prints queued lines a character at a time
    boot.type_out(time.delta_secs_f64(), cfg.boot.type_cps, cfg.boot.line_seconds.max(0.0));
    //Engine output, whatever the stage
    let mut finished: Option<(RunKind, i32, f64)> = None;
    if let Some(run) = boot.run.as_mut() {
        let lines = run.poll(t);
        let kind = run.kind;
        let secs = run.secs();
        let fin = run.finished(t).map(|c| (kind, c, secs));
        for l in lines { boot.say(Kind::Dim, format!("  {l}")); }
        finished = fin;
    }
    if let Some((kind, code, secs)) = finished {
        boot.run = None;
        match (kind, code) {
            (RunKind::Login, 0) => { boot.logged_in = true; boot.say(Kind::Text, "SESSION ESTABLISHED"); boot.say(Kind::Text, ""); boot.go(Stage::Menu); }
            (RunKind::Login, c) => { boot.say(Kind::Loud, format!("IDENTIFICATION NOT RECOGNIZED BY SYSTEM (EXIT {c})")); boot.field = 1; boot.go(Stage::Login); }
            (k, 0) => {
                boot.say(Kind::Text, format!("ENGINE COMPLETE IN {secs:.1} S   RELOADING VIEWER DATA"));
                reload.send(ReloadData { elsets: k == RunKind::Full });
                boot.reloaded = true;
                boot.go(Stage::Menu);
            }
            (_, c) => { boot.say(Kind::Loud, format!("ENGINE TERMINATED   EXIT {c} AFTER {secs:.1} S")); boot.go(Stage::Menu); }
        }
        return;
    }

    match boot.stage {
        Stage::Check => {
            if !boot.checked { boot.checked = true; queue_checks(&mut boot, &cfg, &vcfg, &log); boot.since = t; }
            if escape { boot.say(Kind::Text, "DIRECT ENTRY"); boot.go(Stage::Entrance); }
            else if boot.idle() && (t - boot.next_emit > 0.5 || enter) { boot.field = if boot.user.is_empty() { 0 } else { 1 }; boot.go(Stage::Login); }
        }
        Stage::Login => {
            let typed = input.typed();
            if !typed.is_empty() { if boot.field == 0 { boot.user.push_str(&typed); } else { boot.pass.push_str(&typed); } }
            for p in &input.presses {
                match p.code {
                    KeyCode::Backspace => { if boot.field == 0 { boot.user.pop(); } else { boot.pass.pop(); } }
                    KeyCode::Tab | KeyCode::ArrowDown | KeyCode::ArrowUp => boot.field = 1 - boot.field,
                    KeyCode::F2 => { boot.say(Kind::Text, "LOGON BYPASSED   CATALOG REFRESH UNAVAILABLE"); boot.say(Kind::Text, ""); boot.go(Stage::Menu); }
                    KeyCode::Escape => { boot.say(Kind::Text, "DIRECT ENTRY"); boot.go(Stage::Entrance); }
                    _ => {}
                }
            }
            if enter && boot.stage == Stage::Login {
                if boot.field == 0 && !boot.user.is_empty() { boot.field = 1; }
                else if boot.user.is_empty() { boot.say(Kind::Loud, "IDENTIFICATION REQUIRED"); }
                else if boot.pass.is_empty() && !boot.saved_pass { boot.say(Kind::Loud, "PASSWORD REQUIRED   NO CREDENTIAL ON FILE"); }
                else {
                    let (u, p) = (boot.user.clone(), boot.pass.clone());
                    boot.say(Kind::Text, format!("LOGON: {u}{}", if p.is_empty() { "   CREDENTIAL ON FILE" } else { "" }));
                    boot.say(Kind::Text, "CONNECTING TO SPACE-TRACK.ORG");
                    match Runner::spawn(&cfg, RunKind::Login, &u, &p, "") {
                        Ok((r, desc)) => { boot.say(Kind::Dim, format!("  EXEC {desc}")); boot.run = Some(r); boot.go(Stage::Verify); }
                        Err(e) => boot.say(Kind::Loud, format!("  {}", e.to_uppercase())),
                    }
                }
            }
        }
        Stage::Verify => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Text, "LOGON ABORTED"); boot.go(Stage::Login); }
        }
        Stage::Menu => {
            if enter && !input.presses.iter().any(|p| matches!(p.code, KeyCode::Enter | KeyCode::NumpadEnter)) { boot.say(Kind::Text, "PROCEEDING WITH DATA ON FILE"); boot.go(Stage::Entrance); }
            for p in &input.presses {
                match p.code {
                    KeyCode::Enter | KeyCode::NumpadEnter | KeyCode::KeyS | KeyCode::Escape => { boot.say(Kind::Text, "PROCEEDING WITH DATA ON FILE"); boot.go(Stage::Entrance); }
                    KeyCode::KeyF if boot.logged_in => start_engine(&mut boot, &cfg, RunKind::Full, "FULL CATALOG REFRESH"),
                    KeyCode::KeyF => boot.say(Kind::Loud, "CATALOG REFRESH REQUIRES A SPACE-TRACK SESSION   L TO LOG ON"),
                    KeyCode::KeyR => start_engine(&mut boot, &cfg, RunKind::Rank, "RE-RANK"),
                    KeyCode::KeyO => {
                        boot.orbits = match boot.orbits.as_str() { "leo" => "geo", "geo" => "all", _ => "leo" }.to_string();
                        let what = match boot.orbits.as_str() {
                            "geo" => "GEOSTATIONARY BELT ONLY   GOES AND ITS NEIGHBOURS, PARKED POINTING",
                            "all" => "LOW ORBIT + THE BELT",
                            _ => "LOW ORBIT",
                        };
                        let shown = boot.orbits.to_uppercase();
                        boot.say(Kind::Text, format!("ORBITS {shown}   {what}"));
                    }
                    KeyCode::KeyL => { boot.field = 1; boot.go(Stage::Login); }
                    _ => {}
                }
                if boot.stage != Stage::Menu { break; }
            }
        }
        Stage::Run => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Text, "ENGINE STOPPED BY OPERATOR"); boot.go(Stage::Menu); }
        }
        Stage::Entrance => {
            //Wait for the viewer's propagation (Enter or Escape stops waiting), then say so and go in
            let ready = prop.total == 0 || prop.finished;
            if boot.leaving_at.is_none() && (ready || enter || escape) {
                boot.leaving_at = Some(t);
                boot.say(Kind::Text, "");
                boot.say(Kind::Text, "ENTERING CONTROL");
            }
            if boot.leaving_at.is_some() && boot.idle() && t - boot.next_emit > 0.8 { boot.go(Stage::Exit); }
        }
        Stage::Exit => {
            booting.0 = false;
            for e in &roots { commands.entity(e).despawn_recursive(); }
            commands.insert_resource(Reveal { start: time.elapsed_secs_f64() });
            println!("boot page done{}", if boot.reloaded { " (viewer reloaded)" } else { "" });
        }
    }
}

fn boot_render(
    boot: Res<Boot>, cfg: Res<ControlConfig>, pal: Res<Palette>, windows: Query<&Window, With<PrimaryWindow>>, prop: Res<PropStatus>,
    mut spans: Query<(&BootLogSpan, &mut TextSpan, &mut TextColor)>,
    mut prompt: Query<&mut Text, (With<BootPrompt>, Without<BootHint>)>,
    mut hint: Query<&mut Text, (With<BootHint>, Without<BootPrompt>)>,
) {
    let fs = cfg.boot.font_size;
    let line_h = fs * 1.2;
    let wh = windows.get_single().map(|w| w.height()).unwrap_or(cfg.window.height);
    //Rows the log may use: the page minus the prompt block (up to 7 lines) and the legend
    let rows = (((wh * 0.91 - 40.0 - line_h * 9.0) / line_h).floor() as usize).clamp(6, MAX_ROWS);
    let blink = (boot.t * 2.5) as i64 % 2 == 0;
    let cur = if blink { "\u{2588}" } else { " " };
    let idle = boot.idle();
    let n = boot.lines.len();
    let first = n.saturating_sub(rows);
    for (span, mut ts, mut col) in &mut spans {
        match boot.lines.get(first + span.0) {
            Some(l) => {
                //The line being typed carries the cursor
                let typing = l.shown < l.len;
                let want: String = l.text.chars().take(l.shown).chain(if typing { cur.chars().next() } else { None }).chain(std::iter::once('\n')).collect();
                if ts.0 != want { ts.0 = want; }
                col.0 = match l.kind { Kind::Text => pal.text, Kind::Loud => pal.bright, Kind::Dim => pal.dim };
            }
            None => { if !ts.0.is_empty() { ts.0.clear(); } }
        }
    }

    //The prompt waits for the teletype to finish; running things just count seconds
    let stars: String = "*".repeat(boot.pass.chars().count());
    let secs = boot.run.as_ref().map_or(0.0, |r| r.secs());
    let (p, h): (String, String) = match boot.stage {
        _ if !idle && matches!(boot.stage, Stage::Check | Stage::Login | Stage::Menu) => (String::new(), String::new()),
        Stage::Check => (cur.to_string(), "ENTER  CONTINUE          ESC  DIRECT ENTRY".into()),
        Stage::Login => (
            format!("LOGON: {}{}\nPASSWORD: {}{}{}", boot.user, if boot.field == 0 { cur } else { "" }, stars, if boot.field == 1 { cur } else { "" },
                    if boot.pass.is_empty() && boot.saved_pass && boot.field == 1 { "   (ON FILE: ENTER TO USE)" } else { "" }),
            "ENTER  SUBMIT          TAB  FIELD          F2  BYPASS LOGON          ESC  DIRECT ENTRY".into()),
        Stage::Verify => (format!("AUTHENTICATING  {secs:.0} S {cur}"), "ESC  ABORT".into()),
        Stage::Menu => (
            format!("SELECT:\n  ENTER  PROCEED WITH DATA ON FILE\n  F      FULL CATALOG REFRESH{}\n  R      RE-RANK\n  O      ORBITS: {}{}\n{cur}",
                    if boot.logged_in { "" } else { "   (LOGON REQUIRED)" },
                    match boot.orbits.as_str() { "geo" => "GEOSTATIONARY BELT", "all" => "LOW ORBIT + BELT", _ => "LOW ORBIT" },
                    if boot.logged_in { "" } else { "\n  L      LOGON" }),
            String::new()),
        Stage::Run => (format!("ENGINE RUNNING  {secs:.0} S {cur}"), "ESC  STOP ENGINE".into()),
        Stage::Entrance if boot.leaving_at.is_none() => (format!("PROPAGATING ORBITS  {}/{} {cur}", prop.done, prop.total), "ENTER  SKIP".into()),
        Stage::Entrance | Stage::Exit => (String::new(), String::new()),
    };
    for mut t in &mut prompt { if t.0 != p { t.0 = p.clone(); } }
    for mut t in &mut hint { if t.0 != h { t.0 = h.clone(); } }
}

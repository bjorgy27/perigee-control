/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The boot page: what the window shows before the tiles. It walks through the launch like a terminal
/// waking up, every line resolving out of noise, glyph rain behind it:
///
///   CHECK   settings, data folder and file ages, the engine binary, serial ports, the mount frame, then
///           everything the viewer said while loading its data (the same lines Perigee prints)
///   LOGIN   Space-Track identity and password (prefilled from the engine's .env), verified by running
///           `perigee login`, whose lines stream in; ACCESS GRANTED or DENIED
///   MENU    FULL RUN (the engine fetches, propagates and ranks; the viewer reloads the new files),
///           RANK ONLY, or SKIP
///   LOAD    the viewer's propagation progress, then ENTERING CONTROL and the tiles appear
///
/// The engine runs as a child process with the credentials in its environment; its output is read on
/// two threads (stdout, stderr) and shown as it arrives. Escape at any prompt goes straight in.
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
enum Stage { Check, Login, Verify, Menu, Run, Load, Exit }

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
}

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
}

#[derive(Component)] struct BootRoot;
#[derive(Component)] struct BootLogSpan(usize);
#[derive(Component)] struct BootPrompt;
#[derive(Component)] struct BootHint;
#[derive(Component)] struct RainCol { x: f32, y: f32, speed: f32, len: usize, next: f64 }

pub struct BootPlugin;
impl Plugin for BootPlugin {
    fn build(&self, app: &mut App) {
        let enabled = app.world().get_resource::<ControlConfig>().map_or(true, |c| c.boot.enabled);
        app.insert_resource(Booting(enabled))
            .insert_resource(Boot {
                stage: Stage::Check, t: 0.0, since: 0.0, lines: Vec::new(), pending: VecDeque::new(), next_emit: 0.0,
                user: String::new(), pass: String::new(), field: 0, saved_pass: false, logged_in: false,
                run: None, rng: 0, shown_log: 0, reloaded: false, checked: false,
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
    let fs = cfg.window.font_size + 1.0;
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
            let step = fs * 1.9;
            let cols = ((ww / step) as usize).clamp(8, 96);
            for i in 0..cols {
                let x = i as f32 * step + (boot.rand() % 7) as f32;
                let len = 6 + (boot.rand() % 18) as usize;
                let speed = 40.0 + (boot.rand() % 140) as f32;
                let y = -((boot.rand() % (wh.max(1.0) as u64 + 1)) as f32);
                let text: String = (0..len).map(|_| boot.glyph()).map(|c| format!("{c}\n")).collect();
                root.spawn((
                    Node { position_type: PositionType::Absolute, left: Val::Px(x), top: Val::Px(y), ..default() },
                    Text::new(text), tf(fs), TextColor(pal.dim.with_alpha(0.32)), TextLayout::new_with_no_wrap(), PickingBehavior::IGNORE,
                    RainCol { x, y, speed, len, next: 0.0 },
                ));
            }
        }
        //The page: title, the log (one span per row), the prompt, the key hints
        root.spawn(Node { position_type: PositionType::Absolute, left: Val::Percent(7.0), top: Val::Percent(5.0), width: Val::Percent(86.0), bottom: Val::Percent(4.0),
                          flex_direction: FlexDirection::Column, row_gap: Val::Px(10.0), ..default() })
            .with_children(|col| {
                col.spawn((Text::new("PERIGEE"), tf(fs * 3.6), TextColor(pal.bright), TextLayout::new_with_no_wrap()));
                col.spawn((Text::new("GROUND STATION CONTROL   //   SWEEP THE CATALOG.  RANK THE PASSES.  POINT.  LISTEN."), tf(fs), TextColor(pal.dim), TextLayout::new_with_no_wrap()));
                col.spawn((Node { height: Val::Px(2.0), width: Val::Percent(100.0), margin: UiRect::vertical(Val::Px(4.0)), ..default() }, BackgroundColor(pal.divider)));
                col.spawn((Text::new(""), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap(), Node { flex_grow: 1.0, min_height: Val::Px(0.0), overflow: Overflow::clip(), ..default() }))
                    .with_children(|log| { for i in 0..MAX_ROWS { log.spawn((TextSpan::new(""), tf(fs), TextColor(pal.text), BootLogSpan(i))); } });
                col.spawn((Text::new(""), tf(fs + 1.0), TextColor(pal.bright), TextLayout::new_with_no_wrap(), BootPrompt, Node { flex_shrink: 0.0, ..default() }));
                col.spawn((Text::new(""), tf(fs - 1.0), TextColor(pal.dim), TextLayout::new_with_no_wrap(), BootHint, Node { flex_shrink: 0.0, ..default() }));
            });
    });
}

/// The check lines: what this program found around itself
fn queue_checks(boot: &mut Boot, cfg: &ControlConfig, vcfg: &ViewerCfg, log: &BootLog) {
    let now = chrono::Local::now().format("%Y-%m-%d %H:%M:%S %Z").to_string();
    boot.say(Kind::Title, format!("PERIGEE CONTROL   boot {now}"));
    boot.say(if std::path::Path::new("control.toml").is_file() { Kind::Ok } else { Kind::Warn },
             if std::path::Path::new("control.toml").is_file() { "[OK] settings   control.toml".to_string() } else { "[--] settings   control.toml not found, built-in defaults".to_string() });
    boot.say(Kind::Ok, format!("[OK] viewer     {}   station {}  {:.4} {:.4}   mask {:.0} deg", cfg.viewer.config, vcfg.station.name, vcfg.station.lat_deg, vcfg.station.lon_deg, vcfg.station.elevation_mask_deg));
    let dir = std::path::Path::new(if cfg.data.dir.is_empty() { "." } else { &cfg.data.dir });
    if dir.is_dir() {
        let ages: Vec<String> = ["ELSET.json", "SORTED_SATS.json", "NORADs.json", "SATELLITE_RANKS.json", "CATEGORIES.json"].iter()
            .map(|f| match age_of(&dir.join(f)) { Some(a) => format!("{f} {a}"), None => format!("{f} MISSING") }).collect();
        boot.say(Kind::Ok, format!("[OK] data       {}", dir.display()));
        boot.say(Kind::Dim, format!("                {}", ages.join("  ·  ")));
    } else {
        boot.say(Kind::Err, format!("[!!] data       {} is not a folder", dir.display()));
    }
    match engine_status(cfg) {
        Ok(s) => boot.say(Kind::Ok, format!("[OK] engine     {s}")),
        Err(s) => boot.say(Kind::Warn, format!("[--] engine     {s}")),
    }
    let ports = SerialLink::scan_ports();
    if ports.is_empty() {
        boot.say(if cfg.serial.simulate { Kind::Warn } else { Kind::Err }, format!("[--] serial     no port found{}", if cfg.serial.simulate { ": the mount simulator will answer" } else { " and simulate = false" }));
    } else {
        boot.say(Kind::Ok, format!("[OK] serial     {}   {} baud", ports.join("  "), cfg.serial.baud));
    }
    let m = &cfg.mount;
    boot.say(Kind::Ok, format!("[OK] mount      az 0..{:.0}  el {:.0}..{:.0}   centre bearing {:.1}   park {:.0} / {:.0}   rate {:.0} / {:.0} deg/s",
                               m.az_travel_deg, m.el_min_deg, m.el_max_deg, m.az_center_bearing_deg, m.park_az_deg, m.park_el_deg, m.az_rate_dps, m.el_rate_dps));
    boot.say(Kind::Dim, "");
    boot.say(Kind::Title, "VIEWER DATA");
    for l in &log.0 { boot.say(Kind::Info, format!("  {l}")); }
    boot.shown_log = log.0.len();
    boot.say(Kind::Dim, "");
    boot.say(Kind::Title, "SPACE-TRACK.ORG   AUTHENTICATION");
}

fn boot_tick(
    time: Res<Time>, input: Res<CmdInput>, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, mut boot: ResMut<Boot>, log: Res<BootLog>,
    prop: Res<PropStatus>, mut reload: EventWriter<ReloadData>, mut booting: ResMut<Booting>, mut commands: Commands,
    roots: Query<Entity, With<BootRoot>>,
) {
    boot.t += time.delta_secs_f64();
    let t = boot.t;
    let enter = input.pressed(KeyCode::Enter) || input.pressed(KeyCode::NumpadEnter);
    let escape = input.pressed(KeyCode::Escape);

    //Lines the viewer printed since we last looked (a reload after the engine ran)
    if log.0.len() > boot.shown_log {
        for l in &log.0[boot.shown_log..] { boot.say(Kind::Info, format!("  viewer  {l}")); }
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
        for l in lines { boot.say(Kind::Engine, format!("  > {l}")); }
        finished = fin;
    }
    if let Some((kind, code, secs)) = finished {
        boot.run = None;
        match (kind, code) {
            (RunKind::Login, 0) => { boot.logged_in = true; boot.say(Kind::Title, "ACCESS GRANTED"); boot.say(Kind::Dim, ""); boot.go(Stage::Menu); }
            (RunKind::Login, c) => { boot.say(Kind::Err, format!("ACCESS DENIED   (perigee login exited with {c})")); boot.field = 1; boot.go(Stage::Login); }
            (k, 0) => {
                boot.say(Kind::Ok, format!("[OK] engine finished in {secs:.1} s; reloading the viewer"));
                reload.send(ReloadData { elsets: k == RunKind::Full });
                boot.reloaded = true;
                boot.go(Stage::Load);
            }
            (_, c) => { boot.say(Kind::Err, format!("[!!] engine failed (exit {c}) after {secs:.1} s")); boot.go(Stage::Menu); }
        }
        return;
    }

    match boot.stage {
        Stage::Check => {
            if !boot.checked { boot.checked = true; queue_checks(&mut boot, &cfg, &vcfg, &log); boot.since = t; }
            if escape { boot.go(Stage::Load); }
            else if boot.pending.is_empty() && (t - boot.next_emit > 0.5 || enter) { boot.field = if boot.user.is_empty() { 0 } else { 1 }; boot.go(Stage::Login); }
        }
        Stage::Login => {
            let typed = input.typed();
            if !typed.is_empty() { if boot.field == 0 { boot.user.push_str(&typed); } else { boot.pass.push_str(&typed); } }
            for p in &input.presses {
                match p.code {
                    KeyCode::Backspace => { if boot.field == 0 { boot.user.pop(); } else { boot.pass.pop(); } }
                    KeyCode::Tab | KeyCode::ArrowDown | KeyCode::ArrowUp => boot.field = 1 - boot.field,
                    KeyCode::F2 => { boot.say(Kind::Warn, "[--] login skipped: FULL RUN is unavailable"); boot.say(Kind::Dim, ""); boot.go(Stage::Menu); }
                    KeyCode::Escape => { boot.say(Kind::Warn, "[--] boot skipped"); boot.go(Stage::Load); }
                    _ => {}
                }
            }
            if enter && boot.stage == Stage::Login {
                if boot.field == 0 && !boot.user.is_empty() { boot.field = 1; }
                else if boot.user.is_empty() { boot.say(Kind::Err, "[!!] identity is empty"); }
                else if boot.pass.is_empty() && !boot.saved_pass { boot.say(Kind::Err, "[!!] password is empty and none is saved in .env"); }
                else {
                    let (u, p) = (boot.user.clone(), boot.pass.clone());
                    boot.say(Kind::Info, format!("  connecting to space-track.org as {u}{}", if p.is_empty() { "  (password from .env)" } else { "" }));
                    match Runner::spawn(&cfg, RunKind::Login, &u, &p) {
                        Ok((r, desc)) => { boot.say(Kind::Dim, format!("  $ {desc}")); boot.run = Some(r); boot.go(Stage::Verify); }
                        Err(e) => boot.say(Kind::Err, format!("[!!] {e}")),
                    }
                }
            }
        }
        Stage::Verify => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Warn, "[--] login cancelled"); boot.go(Stage::Login); }
        }
        Stage::Menu => {
            for p in &input.presses {
                match p.code {
                    KeyCode::Enter | KeyCode::NumpadEnter | KeyCode::KeyF if boot.logged_in => {
                        let (u, pw) = (boot.user.clone(), boot.pass.clone());
                        boot.say(Kind::Title, "FULL RUN");
                        match Runner::spawn(&cfg, RunKind::Full, &u, &pw) {
                            Ok((r, desc)) => { boot.say(Kind::Dim, format!("  $ {desc}")); boot.run = Some(r); boot.go(Stage::Run); }
                            Err(e) => boot.say(Kind::Err, format!("[!!] {e}")),
                        }
                    }
                    KeyCode::Enter | KeyCode::NumpadEnter | KeyCode::KeyF => boot.say(Kind::Warn, "[--] FULL RUN needs a Space-Track login: L to log in"),
                    KeyCode::KeyR => {
                        boot.say(Kind::Title, "RANK ONLY");
                        match Runner::spawn(&cfg, RunKind::Rank, "", "") {
                            Ok((r, desc)) => { boot.say(Kind::Dim, format!("  $ {desc}")); boot.run = Some(r); boot.go(Stage::Run); }
                            Err(e) => boot.say(Kind::Err, format!("[!!] {e}")),
                        }
                    }
                    KeyCode::KeyS | KeyCode::Escape => { boot.say(Kind::Info, "  using the data on disk"); boot.go(Stage::Load); }
                    KeyCode::KeyL => { boot.field = 1; boot.go(Stage::Login); }
                    _ => {}
                }
                if boot.stage != Stage::Menu { break; }
            }
        }
        Stage::Run => {
            if escape { if let Some(mut r) = boot.run.take() { r.kill(); } boot.say(Kind::Warn, "[--] engine stopped"); boot.go(Stage::Menu); }
        }
        Stage::Load => {
            let settled = t - boot.since > 1.0;
            let ready = prop.total == 0 || prop.finished;
            if (settled && ready && boot.pending.is_empty()) || enter || escape {
                boot.say(Kind::Dim, "");
                boot.say(Kind::Title, "ENTERING CONTROL");
                boot.go(Stage::Exit);
            }
        }
        Stage::Exit => {
            if t - boot.since > 1.2 {
                booting.0 = false;
                for e in &roots { commands.entity(e).despawn_recursive(); }
                println!("boot page done{}", if boot.reloaded { " (viewer reloaded)" } else { "" });
            }
        }
    }
}

fn boot_rain(time: Res<Time>, windows: Query<&Window, With<PrimaryWindow>>, mut boot: ResMut<Boot>, cfg: Res<ControlConfig>, mut cols: Query<(&mut RainCol, &mut Node, &mut Text)>) {
    let Ok(w) = windows.get_single() else { return };
    let (ww, wh) = (w.width(), w.height());
    let line_h = (cfg.window.font_size + 1.0) * 1.2;
    let dt = time.delta_secs();
    let t = boot.t;
    for (mut c, mut node, mut text) in &mut cols {
        c.y += c.speed * dt;
        if c.y > wh + 10.0 {
            c.y = -(c.len as f32 * line_h) - (boot.rand() % 400) as f32;
            c.x = (boot.rand() % (ww.max(1.0) as u64 + 1)) as f32;
            c.speed = 40.0 + (boot.rand() % 140) as f32;
            node.left = Val::Px(c.x);
        }
        node.top = Val::Px(c.y);
        if t >= c.next {
            c.next = t + 0.12 + (boot.rand() % 20) as f64 * 0.01;
            let n = c.len;
            text.0 = (0..n).map(|_| boot.glyph()).map(|g| format!("{g}\n")).collect();
        }
    }
}

fn boot_render(
    mut boot: ResMut<Boot>, cfg: Res<ControlConfig>, pal: Res<Palette>, windows: Query<&Window, With<PrimaryWindow>>, prop: Res<PropStatus>,
    mut spans: Query<(&BootLogSpan, &mut TextSpan, &mut TextColor)>,
    mut prompt: Query<&mut Text, (With<BootPrompt>, Without<BootHint>)>,
    mut hint: Query<&mut Text, (With<BootHint>, Without<BootPrompt>)>,
) {
    let fs = cfg.window.font_size + 1.0;
    let line_h = fs * 1.2;
    let wh = windows.get_single().map(|w| w.height()).unwrap_or(cfg.window.height);
    let rows = (((wh * 0.91 - fs * 3.6 * 1.2 - fs * 1.2 - 40.0 - line_h * 5.0) / line_h).floor() as usize).clamp(6, MAX_ROWS);
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
        Stage::Check => (format!("{cur}"), "Enter  skip ahead      Esc  straight to the tiles".into()),
        Stage::Login => (
            format!("  identity   {}{}\n  password   {}{}{}", boot.user, if boot.field == 0 { cur } else { "" }, stars, if boot.field == 1 { cur } else { "" },
                    if boot.pass.is_empty() && boot.saved_pass && boot.field == 1 { "   (saved in .env: Enter uses it)" } else { "" }),
            "Enter  log in      Tab  switch field      F2  skip the login      Esc  straight to the tiles".into()),
        Stage::Verify => (format!("  {spinner}  verifying with space-track.org   {:.1} s", boot.run.as_ref().map_or(0.0, |r| r.secs())), "Esc  cancel".into()),
        Stage::Menu => (
            format!("  [ENTER]  FULL RUN    fetch the catalog and SatNOGS, propagate, rank, reload the viewer   (about a minute){}\n  [R]      RANK ONLY   re-rank the data on disk for right now\n  [S]      SKIP        use the data on disk as it is{}",
                    if boot.logged_in { "" } else { "   (needs login)" }, if boot.logged_in { "" } else { "\n  [L]      LOGIN" }),
            "the engine writes into the data folder; the viewer reloads when it is done".into()),
        Stage::Run => (format!("  {spinner}  engine running   {:.0} s", boot.run.as_ref().map_or(0.0, |r| r.secs())), "Esc  stop the engine".into()),
        Stage::Load => (
            if prop.total > 0 && !prop.finished { format!("  {spinner}  PROPAGATING {}/{}", prop.done, prop.total) } else { format!("  {cur}  tracks ready") },
            "Enter  go".into()),
        Stage::Exit => (String::new(), String::new()),
    };
    for mut t in &mut prompt { if t.0 != p { t.0 = p.clone(); } }
    for mut t in &mut hint { if t.0 != h { t.0 = h.clone(); } }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Perigee Control
/// One window: a boot page (checks, Space-Track login, an engine run if wanted), then the orbit viewer
/// (perigee-viewer, as a library, in its own tile) beside the command tiles: live data on the picked
/// satellite, motor control, a serial console straight to the mount's Arduino, and a wireframe of the
/// gimbal following its telemetry. Pick a satellite on the globe, ARM in LIVE DATA: the mount
/// pre-positions for AOS and tracks the pass.
///
/// Settings: control.toml (this program) and the viewer's viewer.toml (pointed to from control.toml).
/////////////////////////////////////////////////////////////////////////////////////////////////////////
mod boot;
mod command;
mod config;
mod console;
mod input;
mod mount;
mod serial;
mod tiles;
mod tracking;

use config::ControlConfig;

fn main() {
    let ccfg = ControlConfig::load("control.toml");
    //The viewer reads its own settings; [data] dir here redirects where it looks for Perigee's files
    let viewer_toml = ccfg.viewer.config.clone();
    let orbit = if ccfg.data.dir.is_empty() { None } else { Some(std::path::Path::new(&ccfg.data.dir).join("ORBIT_DATA.json").to_string_lossy().to_string()) };
    println!("perigee-control: viewer settings {viewer_toml}{}", orbit.as_deref().map_or(String::new(), |o| format!(", data folder of {o}")));
    let mut app = perigee_viewer::build_app(&viewer_toml, orbit.as_deref());
    app.insert_resource(ccfg);
    app.add_plugins(command::ControlPlugin);
    app.add_plugins(boot::BootPlugin);
    app.add_plugins(DebugHooks);
    app.run();
}

/// PERIGEE_AUTOPICK=<NORAD id, or "rank"> picks that satellite (or the top-ranked one) two seconds after
/// the tiles are up, as a click on the globe would, so the procedure can be watched or recorded unattended.
fn auto_pick(
    time: bevy::prelude::Res<bevy::prelude::Time>, booting: bevy::prelude::Res<boot::Booting>, reveal: Option<bevy::prelude::Res<boot::Reveal>>,
    ranks: bevy::prelude::Res<perigee_viewer::Ranks>, cat: bevy::prelude::Res<perigee_viewer::Catalog>, mut sel: bevy::prelude::ResMut<perigee_viewer::Selected>,
    mut state: bevy::prelude::Local<(Option<f64>, bool)>,   // time the tiles came up, done
) {
    if state.1 || booting.0 || reveal.is_some() { return; }
    let t = time.elapsed_secs_f64();
    let up = *state.0.get_or_insert(t);
    if t - up < 2.0 { return; }
    let want = std::env::var("PERIGEE_AUTOPICK").unwrap_or_default();
    let col = match want.parse::<u32>() {
        Ok(id) => cat.ids.iter().position(|c| *c == Some(id)),
        Err(_) => ranks.entries.first().map(|e| e.pass.column),
    };
    match col {
        Some(c) => { println!("autopick: column {c} ({want})"); sel.0 = Some(c); }
        None => println!("autopick: nothing matches {want}"),
    }
    state.1 = true;
}

/// Environment variables for debugging and recording without a screen: PERIGEE_SCREENSHOT=<prefix> saves
/// <prefix>-<seconds>.png of the window every 3 s (PERIGEE_SCREENSHOT_EVERY changes that),
/// PERIGEE_EXIT_AFTER=<seconds> quits after that long, PERIGEE_RECORD=<dir> writes every frame as
/// <dir>/frame-NNNNN.bmp at 24 frames per second from launch until two seconds after the tiles are up,
/// then quits (ffmpeg turns the folder into a video).
struct DebugHooks;
impl bevy::prelude::Plugin for DebugHooks {
    fn build(&self, app: &mut bevy::prelude::App) {
        if ["PERIGEE_SCREENSHOT", "PERIGEE_EXIT_AFTER", "PERIGEE_RECORD"].iter().any(|v| std::env::var(v).is_ok()) {
            app.add_systems(bevy::prelude::Update, debug_hooks);
        }
        if std::env::var("PERIGEE_AUTOPICK").is_ok() { app.add_systems(bevy::prelude::Update, auto_pick); }
    }
}
fn debug_hooks(
    time: bevy::prelude::Res<bevy::prelude::Time>, mut commands: bevy::prelude::Commands, mut next_shot: bevy::prelude::Local<f64>,
    mut exit: bevy::prelude::EventWriter<bevy::prelude::AppExit>,
    mut rec: bevy::prelude::Local<(f64, u32, Option<f64>)>,   // next frame time, frame count, time the tiles came up
    booting: bevy::prelude::Res<boot::Booting>, reveal: Option<bevy::prelude::Res<boot::Reveal>>,
) {
    use bevy::render::view::screenshot::{save_to_disk, Screenshot};
    let t = time.elapsed_secs_f64();
    if let Ok(dir) = std::env::var("PERIGEE_RECORD") {
        if t >= rec.0 {
            rec.0 = t + 1.0 / 24.0;
            let path = format!("{dir}/frame-{:05}.bmp", rec.1);
            rec.1 += 1;
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
        }
        if !booting.0 && reveal.is_none() && rec.2.is_none() { rec.2 = Some(t); }
        if rec.2.map_or(false, |up| t - up > 2.0) { println!("recording done: {} frames", rec.1); exit.send(bevy::prelude::AppExit::Success); }
    }
    if let Ok(prefix) = std::env::var("PERIGEE_SCREENSHOT") {
        let every = std::env::var("PERIGEE_SCREENSHOT_EVERY").ok().and_then(|s| s.parse::<f64>().ok()).unwrap_or(3.0).max(0.2);
        if *next_shot == 0.0 { *next_shot = every; }
        if t >= *next_shot {
            let path = format!("{prefix}-{:04.1}.png", t);
            println!("screenshot -> {path}");
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
            *next_shot = t + every;
        }
    }
    if let Some(limit) = std::env::var("PERIGEE_EXIT_AFTER").ok().and_then(|s| s.parse::<f64>().ok()) {
        if t >= limit { println!("exiting after {limit} s (PERIGEE_EXIT_AFTER)"); exit.send(bevy::prelude::AppExit::Success); }
    }
}

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

/// Two environment variables for debugging without a screen: PERIGEE_SCREENSHOT=<prefix> saves
/// <prefix>-<seconds>.png of the window every 3 s, PERIGEE_EXIT_AFTER=<seconds> quits after that long.
struct DebugHooks;
impl bevy::prelude::Plugin for DebugHooks {
    fn build(&self, app: &mut bevy::prelude::App) {
        if std::env::var("PERIGEE_SCREENSHOT").is_ok() || std::env::var("PERIGEE_EXIT_AFTER").is_ok() {
            app.add_systems(bevy::prelude::Update, debug_hooks);
        }
    }
}
fn debug_hooks(time: bevy::prelude::Res<bevy::prelude::Time>, mut commands: bevy::prelude::Commands, mut next_shot: bevy::prelude::Local<f64>, mut exit: bevy::prelude::EventWriter<bevy::prelude::AppExit>) {
    use bevy::render::view::screenshot::{save_to_disk, Screenshot};
    let t = time.elapsed_secs_f64();
    if let Ok(prefix) = std::env::var("PERIGEE_SCREENSHOT") {
        if *next_shot == 0.0 { *next_shot = 3.0; }
        if t >= *next_shot {
            let path = format!("{prefix}-{:03.0}.png", t);
            println!("screenshot -> {path}");
            commands.spawn(Screenshot::primary_window()).observe(save_to_disk(path));
            *next_shot = t + 3.0;
        }
    }
    if let Some(limit) = std::env::var("PERIGEE_EXIT_AFTER").ok().and_then(|s| s.parse::<f64>().ok()) {
        if t >= limit { println!("exiting after {limit} s (PERIGEE_EXIT_AFTER)"); exit.send(bevy::prelude::AppExit::Success); }
    }
}

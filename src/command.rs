/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The control page: one window, five tiles laid out by `tiles`. ORBIT VIEW is the viewer itself (its
/// camera is given that tile as a viewport, its own panels and buttons come with it); LIVE DATA, MOTOR
/// CONTROL, MOUNT and SERIAL CONSOLE are drawn here with the viewer's font and colours, reading the
/// viewer's resources for the picked satellite and driving the mount through `serial`.
///
/// Two cameras share the window: this page's 2D camera renders first (order -1) over the whole window,
/// then the viewer's 3D camera renders only inside its tile. Bright dividers run down every gap.
///
/// Page keys (any tile focused):
///   Ctrl+Arrows / Ctrl+H J K L   focus the tile in that direction      Tab   next tile
///   Ctrl+Shift+Arrows            swap tiles                             Ctrl+F  zoom the focused tile
///   Ctrl+= / Ctrl+-              grow / shrink the focused tile         Ctrl+T  flip its split
///   Ctrl+0..4                    show / hide a tile                     click   focus
/// Tile keys: ORBIT VIEW  all the viewer's keys and mouse   MOTOR  arrows jog, [ ] step, S stop, P park,
///   H home   CONSOLE  type, Enter, Up/Down history, PageUp/PageDown scroll   MOUNT  drag to orbit,
///   wheel to zoom   LIVE  A arm, I aim, W warp, Escape abort
///
/// The procedure: picking a satellite starts it (auto_arm): TARGET, LINK, EPHEMERIS, PASS and PATH checks,
/// SLEW to the AOS point, ARMED until AOS, TRACKING, PARK, each step ticked off in LIVE DATA and noted in
/// the console. On the simulator the dish follows the viewer's clock and the clock is warped to just
/// before AOS, so the whole pass plays out on screen (the wireframe slews, locks on, tracks, parks).
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::boot::{Booting, Reveal};
use crate::config::ControlConfig;
use crate::console::Console;
use crate::input::{CmdInput, CmdWindow};
use crate::mount::{draw_mount, MountColors, MountGeom, MountState, MountView};
use crate::serial::SerialLink;
use crate::tiles::{Dir, Layout, Rect, Tile, ALL};

use crate::tracking::{find_next_pass, sample_pass, Inputs, Phase, Plan, Station, Step, Track, Tracker};
use bevy::core_pipeline::bloom::Bloom;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::camera::Viewport;
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::render::view::RenderLayers;
use bevy::ui::widget::NodeImageMode;
use bevy::ui::IsDefaultUiCamera;
use bevy::window::PrimaryWindow;
use perigee_viewer::config::{hex, Config as ViewerCfg};
use perigee_viewer::{now_jd, Catalog, Mode, Orbits, Ranks, Selected, Sim, UiFont, NOT_YET_JD};

/// Render layers. Interface nodes are always on layer 0 and are only marked visible by a camera that has
/// layer 0, so both cameras carry it. This page's lines live on LAYER, which only the control camera has;
/// the viewer's gizmo lines are moved to VIEWER_LINES, which only the globe camera has. Without that split
/// each camera would draw the other's lines through its own projection.
pub const LAYER: usize = 1;
pub const VIEWER_LINES: usize = 2;

#[derive(Default, GizmoConfigGroup, Reflect)]
pub struct CmdLines;

/// The tile dividers: their own gizmo group so they can be wider than the drawings
#[derive(Default, GizmoConfigGroup, Reflect)]
pub struct CmdDividers;

#[derive(Resource, Default)]
pub struct CmdCamera(pub Option<Entity>);

/// The viewer's 3D camera, whose viewport follows the ORBIT VIEW tile
#[derive(Resource, Default)]
pub struct GlobeCamera(pub Option<Entity>);

#[derive(Resource)]
pub struct Geom(pub MountGeom);

#[derive(Resource)]
pub struct Motor { pub step: f64 }
impl Default for Motor { fn default() -> Self { Self { step: 1.0 } } }

#[derive(Resource)]
pub struct Palette {
    pub text: Color, pub dim: Color, pub bright: Color, pub warn: Color, pub good: Color, pub accent: Color,
    pub panel: Color, pub title_bg: Color, pub border: Color, pub border_focus: Color,
    pub button: Color, pub button_hover: Color, pub button_border: Color,
    pub line: Color, pub line_dim: Color, pub divider: Color, pub space: Color,
}
impl Palette {
    pub fn from_viewer(c: &ViewerCfg, t: &crate::config::TilesCfg) -> Self {
        let k = &c.colors;
        Palette {
            text: hex(&k.text), dim: hex(&k.text_dim), bright: hex(&k.marker_in_view), warn: hex(&k.los), good: hex(&k.aos), accent: hex(&k.track_in_cone),
            panel: hex(&k.button).with_alpha(0.55), title_bg: hex(&k.button_hover).with_alpha(0.5),
            border: if t.border.is_empty() { hex(&k.text_dim).with_alpha(0.7) } else { hex(&t.border) }, border_focus: hex(&k.reticle_sel),
            button: hex(&k.button), button_hover: hex(&k.button_hover), button_border: hex(&k.button_border),
            line: hex(&k.marker), line_dim: hex(&k.orbit).with_alpha(0.55),
            divider: if t.divider.is_empty() { Color::NONE } else { hex(&t.divider) }, space: hex(&k.space),
        }
    }
}

#[derive(Component, Clone, Copy)] pub struct TileRoot(pub Tile);
#[derive(Component, Clone, Copy)] pub struct TileTitle(pub Tile);
#[derive(Component, Clone, Copy)] pub struct TileBody(pub Tile);
#[derive(Component)] pub struct ConsoleInputText;
#[derive(Component)] pub struct CmdUi;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Action { Arm, Aim, Warp, Auto, Abort, Park, Stop, Home, JogAzNeg, JogAzPos, JogElNeg, JogElPos, StepCycle, Connect, Disconnect, Sim, ClearConsole }
impl Action {
    fn label(self) -> &'static str {
        match self {
            Action::Arm => "ARM", Action::Aim => "AIM", Action::Warp => "WARP", Action::Auto => "AUTO", Action::Abort => "ABORT", Action::Park => "PARK", Action::Stop => "STOP", Action::Home => "HOME",
            Action::JogAzNeg => "AZ -", Action::JogAzPos => "AZ +", Action::JogElNeg => "EL -", Action::JogElPos => "EL +", Action::StepCycle => "STEP",
            Action::Connect => "CONNECT", Action::Disconnect => "CLOSE", Action::Sim => "SIM", Action::ClearConsole => "CLEAR",
        }
    }
}
#[derive(Component, Clone, Copy)] pub struct CmdButton(pub Action);
#[derive(Resource, Default)] pub struct Actions(pub Vec<Action>);

/// Everything the LIVE tile shows about the picked satellite, refreshed every frame from the viewer
#[derive(Resource, Default)]
pub struct TargetInfo {
    pub column: Option<usize>, pub norad: Option<u32>, pub name: String,
    pub have_data: bool, pub sim_jd: f64, pub real_jd: f64, pub history_mode: bool,
    pub bearing: f64, pub el: f64, pub range: f64, pub range_rate: f64,
    pub downlink_hz: Option<f64>, pub tx_desc: String, pub tx_mode: String,
    pub rank: Option<(usize, f64)>,
    pub preview: Option<Plan>, pub preview_for: Option<usize>, pub preview_at: f64, pub preview_note: String,
    pub preview_at_jd: f64,             // clock time the preview was computed at
    pub clock_jd: f64,                  // the clock the dish follows: real time, or the viewer's when simulating
    pub time_scale: f64,                // clock seconds per real second
    pub clock_label: &'static str,
    pub point_err_deg: Option<f64>,     // angle between the measured boresight and the satellite
}

/// auto_arm at run time (AUTO button / "/auto" toggles it; control.toml sets the start value)
#[derive(Resource)]
pub struct AutoArm(pub bool);

/// The viewer's clock has been moved to a pass by the procedure (simulator only); restored at the end
#[derive(Resource, Default)]
pub struct Warp { pub active: bool, pub prev_speed: f64, pub prev_mode_live: bool }

#[derive(Resource, Default)]
pub struct DragState { pub tile: Option<Tile> }

//------------------------------------------------------------------------------------------ plugin
pub struct ControlPlugin;
impl Plugin for ControlPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<CmdWindow>()
            .init_resource::<CmdInput>()
            .init_resource::<CmdCamera>()
            .init_resource::<GlobeCamera>()
            .init_resource::<Console>()
            .init_resource::<Tracker>()
            .init_resource::<SerialLink>()
            .init_resource::<MountState>()
            .init_resource::<MountView>()
            .init_resource::<Motor>()
            .init_resource::<Actions>()
            .init_resource::<TargetInfo>()
            .init_resource::<DragState>()
            .init_resource::<Warp>()
            .init_gizmo_group::<CmdLines>()
            .init_gizmo_group::<CmdDividers>()
            .add_systems(Startup, (setup_palette, setup_camera, setup_gizmos).chain())
            .add_systems(PostStartup, (build_ui, adopt_globe_camera).chain())
            .add_systems(PreUpdate, crate::input::route_input.after(bevy::input::InputSystem).before(bevy::ui::UiSystem::Focus))
            .add_systems(Update, (
                (layout_tiles, page_keys, console_keys, motor_keys, live_keys, buttons).chain(),
                (clock_tick, link_tick, target_tick, apply_actions, tracker_tick, warp_tick, refresh_text, draw_overlays).chain(),
            ).chain());
    }
}

fn setup_palette(mut commands: Commands, vcfg: Res<ViewerCfg>, ccfg: Res<ControlConfig>) {
    commands.insert_resource(Palette::from_viewer(&vcfg, &ccfg.tiles));
    commands.insert_resource(Geom(MountGeom::from_cfg(&ccfg.mount)));
    commands.insert_resource(AutoArm(ccfg.tracking.auto_arm));
    commands.insert_resource(Layout::new(ccfg.tiles.main_ratio, ccfg.tiles.row_ratio, ccfg.tiles.gap, ccfg.tiles.globe_ratio, ccfg.window.font_size + 8.0));
}

fn setup_gizmos(mut store: ResMut<GizmoConfigStore>, ccfg: Res<ControlConfig>) {
    let (config, _) = store.config_mut::<CmdLines>();
    config.render_layers = RenderLayers::layer(LAYER);
    config.line_width = 1.5;
    let (config, _) = store.config_mut::<CmdDividers>();
    config.render_layers = RenderLayers::layer(LAYER);
    config.line_width = ccfg.tiles.divider_width.max(0.5);
}

/// The page's own camera: a 2D camera on the viewer's window, rendered before the viewer's camera
/// (order -1) and clearing the whole window, so the viewer paints its tile on top afterwards.
fn setup_camera(mut commands: Commands, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, mut primary: Query<(Entity, &mut Window), With<PrimaryWindow>>, mut cw: ResMut<CmdWindow>, mut cc: ResMut<CmdCamera>) {
    let w = &cfg.window;
    //Only the title: asking for a size here races the compositor's own resize (Hyprland tiles the window)
    //and for one frame the viewport below would exceed the surface, which wgpu treats as fatal.
    if let Ok((e, mut win)) = primary.get_single_mut() {
        win.title = w.title.clone();
        cw.0 = Some(e);
    }
    let tonemap = match vcfg.perf.tonemapping.to_lowercase().as_str() { "none" => Tonemapping::None, "reinhard" => Tonemapping::Reinhard, "aces" => Tonemapping::AcesFitted, _ => Tonemapping::TonyMcMapface };
    let mut cam = commands.spawn((
        Camera2d,
        Camera { hdr: w.hdr, order: -1, clear_color: ClearColorConfig::Custom(hex(&vcfg.colors.space)), ..default() },
        tonemap,
        RenderLayers::from_layers(&[0, LAYER]),
        Name::new("control camera"),
    ));
    if w.hdr && w.bloom > 0.0 { cam.insert(Bloom { intensity: w.bloom, ..Bloom::NATURAL }); }
    if !vcfg.perf.msaa { cam.insert(Msaa::Off); }
    cc.0 = Some(cam.id());
}

/// The viewer's 3D camera becomes the ORBIT VIEW tile: it is the default UI camera (the viewer's panels
/// and buttons target it, so they lay out inside the tile) and its viewport follows the tile from now on.
/// It stays off while the boot page is up.
fn adopt_globe_camera(mut commands: Commands, mut cams: Query<(Entity, &mut Camera), With<Camera3d>>, mut globe: ResMut<GlobeCamera>, booting: Res<Booting>, mut store: ResMut<GizmoConfigStore>) {
    if let Some((e, mut cam)) = cams.iter_mut().next() {
        cam.is_active = !booting.0;
        commands.entity(e).insert((IsDefaultUiCamera, RenderLayers::from_layers(&[0, VIEWER_LINES])));
        globe.0 = Some(e);
    }
    //Every gizmo group that is not ours belongs to the viewer: only the globe camera draws it
    let ours = [std::any::TypeId::of::<CmdLines>(), std::any::TypeId::of::<CmdDividers>()];
    for (id, config, _) in store.iter_mut() {
        if !ours.contains(id) { config.render_layers = RenderLayers::layer(VIEWER_LINES); }
    }
}

//------------------------------------------------------------------------------------------ ui
fn build_ui(mut commands: Commands, cam: Res<CmdCamera>, font: Res<UiFont>, pal: Res<Palette>, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, mut images: ResMut<Assets<Image>>) {
    if let Some(c) = cam.0 { spawn_ui(&mut commands, c, &font.0, &pal, &cfg, &vcfg, &mut images); }
}

fn spawn_ui(commands: &mut Commands, cam: Entity, font: &Handle<Font>, pal: &Palette, cfg: &ControlConfig, vcfg: &ViewerCfg, images: &mut Assets<Image>) {
    let fs = cfg.window.font_size;
    let tf = |size: f32| TextFont { font: font.clone(), font_size: size, ..default() };
    for t in ALL {
        commands.spawn((
            Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), width: Val::Px(100.0), height: Val::Px(100.0),
                   flex_direction: FlexDirection::Column, border: UiRect::all(Val::Px(1.0)), overflow: Overflow::clip(), ..default() },
            BorderColor(pal.border), BackgroundColor(if t == Tile::Globe { Color::NONE } else { pal.panel }), TargetCamera(cam), TileRoot(t), CmdUi, Name::new(t.title()),
        )).with_children(|p| {
            p.spawn((Node { width: Val::Percent(100.0), height: Val::Px(fs + 8.0), padding: UiRect::axes(Val::Px(6.0), Val::Px(2.0)), flex_shrink: 0.0, ..default() }, BackgroundColor(pal.title_bg)))
                .with_children(|b| { b.spawn((Text::new(t.title()), tf(fs), TextColor(pal.bright), TextLayout::new_with_no_wrap(), TileTitle(t))); });
            p.spawn((Node { flex_grow: 1.0, width: Val::Percent(100.0), padding: UiRect::all(Val::Px(6.0)), overflow: Overflow::clip(), flex_direction: FlexDirection::Column, row_gap: Val::Px(4.0), ..default() },))
                .with_children(|b| {
                    b.spawn((Text::new(""), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap(), TileBody(t), Node { flex_grow: 1.0, flex_shrink: 1.0, min_height: Val::Px(0.0), overflow: Overflow::clip(), ..default() }));
                    if t == Tile::Console {
                        b.spawn((Text::new("> "), tf(fs), TextColor(pal.bright), TextLayout::new_with_no_wrap(), ConsoleInputText, Node { flex_shrink: 0.0, ..default() }));
                    }
                    let actions: &[Action] = match t {
                        Tile::Live => &[Action::Arm, Action::Aim, Action::Warp, Action::Abort, Action::Park, Action::Auto],
                        Tile::Motor => &[Action::JogAzNeg, Action::JogAzPos, Action::JogElNeg, Action::JogElPos, Action::StepCycle, Action::Stop, Action::Park, Action::Home, Action::Connect, Action::Sim, Action::Disconnect],
                        Tile::Console => &[Action::ClearConsole],
                        Tile::Mount | Tile::Globe => &[],
                    };
                    if !actions.is_empty() {
                        b.spawn((Node { flex_direction: FlexDirection::Row, flex_wrap: FlexWrap::Wrap, column_gap: Val::Px(4.0), row_gap: Val::Px(4.0), flex_shrink: 0.0, ..default() },))
                            .with_children(|r| {
                                for a in actions {
                                    r.spawn((Button, Node { padding: UiRect::axes(Val::Px(8.0), Val::Px(3.0)), border: UiRect::all(Val::Px(1.0)), ..default() },
                                             BackgroundColor(pal.button), BorderColor(pal.button_border), CmdButton(*a)))
                                        .with_children(|c| { c.spawn((Text::new(a.label()), tf(fs), TextColor(pal.text), TextLayout::new_with_no_wrap())); });
                                }
                            });
                    }
                });
        });
    }
    //Same CRT scanlines as the viewer, over the whole command window
    if cfg.window.scanlines && vcfg.fx.scanlines > 0.0 {
        let period = vcfg.fx.scanline_period_px.max(2);
        let width = 512u32;
        let mut data = vec![0u8; (width * period * 4) as usize];
        let dark = (vcfg.fx.scanlines.clamp(0.0, 1.0) * 255.0) as u8;
        for x in 0..width { data[(x * 4 + 3) as usize] = dark; }
        let mut img = Image::new(Extent3d { width, height: period, depth_or_array_layers: 1 }, TextureDimension::D2, data, TextureFormat::Rgba8UnormSrgb, RenderAssetUsages::RENDER_WORLD);
        img.sampler = bevy::image::ImageSampler::nearest();
        commands.spawn((
            ImageNode { image: images.add(img), image_mode: NodeImageMode::Tiled { tile_x: true, tile_y: true, stretch_value: 1.0 }, ..default() },
            Node { position_type: PositionType::Absolute, left: Val::Px(0.0), top: Val::Px(0.0), width: Val::Percent(100.0), height: Val::Percent(100.0), ..default() },
            GlobalZIndex(50), PickingBehavior::IGNORE, TargetCamera(cam), CmdUi,
        ));
    }
}

fn layout_tiles(
    cw: Res<CmdWindow>, windows: Query<&Window>, mut layout: ResMut<Layout>, pal: Res<Palette>, booting: Res<Booting>,
    mut roots: Query<(&TileRoot, &mut Node, &mut Visibility, &mut BorderColor)>,
    globe: Res<GlobeCamera>, mut cams: Query<&mut Camera>, mut last_size: Local<Option<UVec2>>,
    reveal: Option<Res<Reveal>>, time: Res<Time>, mut commands: Commands,
) {
    let Some(w) = cw.0.and_then(|e| windows.get(e).ok()) else { return };
    let g = layout.gap;
    layout.compute(Rect { x: g, y: g, w: (w.width() - 2.0 * g).max(50.0), h: (w.height() - 2.0 * g).max(50.0) });
    //Coming out of the boot page the tiles come online one after another (ORBIT VIEW first)
    let now = time.elapsed_secs_f64();
    let on = |t: Tile| reveal.as_ref().map_or(true, |r| r.tile(now, ALL.iter().position(|x| *x == t).unwrap_or(0)) >= 1.0);
    if reveal.as_ref().map_or(false, |r| r.done(now)) { commands.remove_resource::<Reveal>(); }
    for (root, mut node, mut vis, mut border) in &mut roots {
        match layout.rects[root.0.idx()] {
            Some(r) if on(root.0) => {
                node.left = Val::Px(r.x); node.top = Val::Px(r.y); node.width = Val::Px(r.w); node.height = Val::Px(r.h);
                *vis = Visibility::Inherited;
                border.0 = if layout.focus == root.0 { pal.border_focus } else { pal.border };
            }
            _ => *vis = Visibility::Hidden,
        }
    }
    //The viewer's camera draws inside the ORBIT VIEW tile: its viewport is that rectangle in physical
    //pixels, clamped to the window. No tile (hidden, or the boot page): the camera is switched off.
    let Some(mut cam) = globe.0.and_then(|e| cams.get_mut(e).ok()) else { return };
    match layout.globe_view() {
        Some(r) if !booting.0 && on(Tile::Globe) => {
            let sf = w.scale_factor();
            //The surface can lag the window by a frame while it is being resized: stay inside both sizes
            let now = UVec2::new(w.physical_width().max(1), w.physical_height().max(1));
            let bound = last_size.map_or(now, |p| p.min(now));
            *last_size = Some(now);
            let (pw, ph) = (bound.x, bound.y);
            let x = ((r.x * sf).round() as u32).min(pw - 1);
            let y = ((r.y * sf).round() as u32).min(ph - 1);
            let wv = ((r.w * sf).round() as u32).clamp(1, pw - x);
            let hv = ((r.h * sf).round() as u32).clamp(1, ph - y);
            let want = Viewport { physical_position: UVec2::new(x, y), physical_size: UVec2::new(wv, hv), ..default() };
            let same = cam.viewport.as_ref().map_or(false, |v| v.physical_position == want.physical_position && v.physical_size == want.physical_size);
            if !same { cam.viewport = Some(want); }
            if !cam.is_active { cam.is_active = true; }
        }
        _ => { if cam.is_active { cam.is_active = false; } }
    }
}

//------------------------------------------------------------------------------------------ keys
fn page_keys(input: Res<CmdInput>, mut layout: ResMut<Layout>, mut drag: ResMut<DragState>, mut view: ResMut<MountView>, mut console: ResMut<Console>) {
    if !input.focused && input.clicks.is_empty() { return; }
    for (button, pos) in &input.clicks {
        if *button == MouseButton::Left { if let Some(t) = layout.tile_at(*pos) { layout.focus = t; drag.tile = Some(t); } }
    }
    if !input.mouse_down.contains(&MouseButton::Left) { drag.tile = None; }
    if drag.tile == Some(Tile::Mount) && input.motion != Vec2::ZERO {
        view.yaw += input.motion.x * 0.01;
        view.pitch = (view.pitch + input.motion.y * 0.01).clamp(-0.2, 1.4);
    }
    if input.wheel != 0.0 {
        match input.cursor.and_then(|c| layout.tile_at(c)) {
            Some(Tile::Mount) => view.zoom = (view.zoom * (1.0 + 0.1 * input.wheel)).clamp(0.3, 4.0),
            Some(Tile::Console) => console.scroll_by((input.wheel * 3.0) as i32, 10),
            _ => {}
        }
    }
    if input.ctrl {
        let dir = |code: KeyCode| match code {
            KeyCode::ArrowLeft | KeyCode::KeyH => Some(Dir::Left), KeyCode::ArrowRight | KeyCode::KeyL => Some(Dir::Right),
            KeyCode::ArrowUp | KeyCode::KeyK => Some(Dir::Up), KeyCode::ArrowDown | KeyCode::KeyJ => Some(Dir::Down), _ => None,
        };
        for p in &input.presses {
            if let Some(d) = dir(p.code) { if input.shift { layout.swap_dir(d) } else { layout.focus_dir(d) } continue; }
            match p.code {
                KeyCode::KeyF => layout.toggle_zoom(),
                KeyCode::KeyT => layout.toggle_split(),
                KeyCode::Equal | KeyCode::NumpadAdd => { layout.resize(true, 0.05); layout.resize(false, 0.05); }
                KeyCode::Minus | KeyCode::NumpadSubtract => { layout.resize(true, -0.05); layout.resize(false, -0.05); }
                KeyCode::Digit0 => layout.toggle_hidden(Tile::Globe),
                KeyCode::Digit1 => layout.toggle_hidden(Tile::Live),
                KeyCode::Digit2 => layout.toggle_hidden(Tile::Motor),
                KeyCode::Digit3 => layout.toggle_hidden(Tile::Mount),
                KeyCode::Digit4 => layout.toggle_hidden(Tile::Console),
                _ => {}
            }
        }
    } else if input.pressed(KeyCode::Tab) {
        let vis: Vec<Tile> = ALL.into_iter().filter(|t| layout.visible(*t)).collect();
        if let Some(i) = vis.iter().position(|t| *t == layout.focus) {
            let n = vis.len();
            layout.focus = vis[if input.shift { (i + n - 1) % n } else { (i + 1) % n }];
        }
    }
}

fn console_keys(input: Res<CmdInput>, layout: Res<Layout>, mut console: ResMut<Console>, mut link: ResMut<SerialLink>, cfg: Res<ControlConfig>, geom: Res<Geom>, mut tracker: ResMut<Tracker>, mut auto: ResMut<AutoArm>) {
    if !input.focused || layout.focus != Tile::Console || input.ctrl { return; }
    let typed = input.typed();
    if !typed.is_empty() { console.input.push_str(&typed); }
    for p in &input.presses {
        match p.code {
            KeyCode::Backspace => { console.input.pop(); }
            KeyCode::ArrowUp => console.history_up(),
            KeyCode::ArrowDown => console.history_down(),
            KeyCode::PageUp => console.scroll_by(10, 10),
            KeyCode::PageDown => console.scroll_by(-10, 10),
            KeyCode::Enter | KeyCode::NumpadEnter => {
                if let Some(line) = console.submit() {
                    if let Some(cmd) = line.strip_prefix('/') {
                        local_command(cmd, &mut console, &mut link, &cfg, &geom.0, &mut tracker, &mut auto);
                    } else if link.is_open() {
                        link.send(&line);
                    } else {
                        console.note("link closed: /open PORT, /sim, or CONNECT in MOTOR CONTROL");
                    }
                }
            }
            _ => {}
        }
    }
}

fn local_command(cmd: &str, console: &mut Console, link: &mut SerialLink, cfg: &ControlConfig, geom: &MountGeom, tracker: &mut Tracker, auto: &mut AutoArm) {
    let mut it = cmd.split_whitespace();
    match it.next().unwrap_or("") {
        "help" => console.help(),
        "ports" => { let ps = SerialLink::scan_ports(); console.note(&if ps.is_empty() { "no serial ports found".into() } else { ps.join("  ") }); }
        "open" => {
            let port = it.next().map(|s| s.to_string()).or_else(|| SerialLink::scan_ports().into_iter().next());
            let baud = it.next().and_then(|s| s.parse().ok()).unwrap_or(cfg.serial.baud);
            match port {
                Some(p) => match link.open(&p, baud) { Ok(()) => console.note(&format!("opened {p} at {baud}")), Err(e) => console.note(&format!("open failed: {e}")) },
                None => console.note("no port given and none found"),
            }
        }
        "close" => { if tracker.active() { if let Step::Send(s) = tracker.abort("link closed") { link.send(&s); } } link.close(); link.next_reconnect = f64::MAX; console.note("link closed (auto-reconnect off until /open or CONNECT)"); }
        "sim" => { link.open_sim(geom, cfg.mount.az_rate_dps, cfg.mount.el_rate_dps); console.note("simulator on"); }
        "clear" => console.lines.clear(),
        "auto" => { auto.0 = match it.next() { Some("on") => true, Some("off") => false, _ => !auto.0 }; console.note(&format!("auto procedure on pick: {}", if auto.0 { "ON" } else { "OFF" })); }
        other => console.note(&format!("unknown local command /{other}; /help")),
    }
}

fn motor_keys(input: Res<CmdInput>, layout: Res<Layout>, mut actions: ResMut<Actions>) {
    if !input.focused || layout.focus != Tile::Motor || input.ctrl { return; }
    for p in &input.presses {
        match p.code {
            KeyCode::ArrowLeft => actions.0.push(Action::JogAzNeg), KeyCode::ArrowRight => actions.0.push(Action::JogAzPos),
            KeyCode::ArrowUp => actions.0.push(Action::JogElPos), KeyCode::ArrowDown => actions.0.push(Action::JogElNeg),
            KeyCode::BracketLeft | KeyCode::BracketRight => actions.0.push(Action::StepCycle),
            KeyCode::KeyS => actions.0.push(Action::Stop), KeyCode::KeyP => actions.0.push(Action::Park), KeyCode::KeyH => actions.0.push(Action::Home),
            _ => {}
        }
    }
}

fn live_keys(input: Res<CmdInput>, layout: Res<Layout>, mut actions: ResMut<Actions>) {
    if !input.focused || input.ctrl { return; }
    for p in &input.presses {
        //Escape in the orbit view is the viewer's (it clears the pick); in every other tile it aborts
        if p.code == KeyCode::Escape && layout.focus != Tile::Globe { actions.0.push(Action::Abort); }
        if layout.focus == Tile::Live && p.code == KeyCode::KeyA { actions.0.push(Action::Arm); }
        if layout.focus == Tile::Live && p.code == KeyCode::KeyI { actions.0.push(Action::Aim); }
        if layout.focus == Tile::Live && p.code == KeyCode::KeyW { actions.0.push(Action::Warp); }
    }
}

fn buttons(mut q: Query<(&Interaction, &CmdButton, &mut BackgroundColor), (Changed<Interaction>, With<Button>)>, pal: Res<Palette>, mut actions: ResMut<Actions>) {
    for (i, b, mut bg) in &mut q {
        match i {
            Interaction::Pressed => { bg.0 = pal.button_hover; actions.0.push(b.0); }
            Interaction::Hovered => bg.0 = pal.button_hover,
            Interaction::None => bg.0 = pal.button,
        }
    }
}

//------------------------------------------------------------------------------------------ clock
/// The clock the dish follows. A real mount always follows real time. The simulator follows the viewer's
/// clock (sim_clock), so HISTORY mode's speed and pause move the simulated mount too, and a warp lands
/// the whole pass on screen.
fn clock_tick(cfg: Res<ControlConfig>, link: Res<SerialLink>, sim: Res<Sim>, mode: Res<Mode>, mut info: ResMut<TargetInfo>) {
    if link.sim.is_some() && cfg.tracking.sim_clock {
        info.clock_jd = sim.jd();
        info.time_scale = match *mode { Mode::History => if sim.paused { 0.0 } else { sim.speed }, Mode::Live => 1.0 };
        info.clock_label = "SIM CLOCK (the viewer's time)";
    } else {
        info.clock_jd = now_jd(); info.time_scale = 1.0; info.clock_label = "REAL CLOCK";
    }
}

//------------------------------------------------------------------------------------------ link
fn link_tick(time: Res<Time>, cfg: Res<ControlConfig>, geom: Res<Geom>, mut link: ResMut<SerialLink>, mut console: ResMut<Console>, mut mount: ResMut<MountState>, mut tracker: ResMut<Tracker>, info: Res<TargetInfo>) {
    let now = time.elapsed_secs_f64();
    //Open something: the configured port when it exists, else the simulator when allowed
    if !link.is_open() && now >= link.next_reconnect {
        link.next_reconnect = now + cfg.serial.reconnect_seconds.max(1.0);
        let port = if cfg.serial.port.eq_ignore_ascii_case("auto") { SerialLink::scan_ports().into_iter().next() }
                   else if std::path::Path::new(&cfg.serial.port).exists() { Some(cfg.serial.port.clone()) } else { None };
        match port {
            Some(p) => match link.open(&p, cfg.serial.baud) {
                Ok(()) => { console.note(&format!("opened {p} at {}", cfg.serial.baud)); hello(&mut link, &cfg); }
                Err(e) => console.note(&format!("open {p} failed: {e}")),
            },
            None if cfg.serial.simulate => { link.open_sim(&geom.0, cfg.mount.az_rate_dps, cfg.mount.el_rate_dps); console.note("no serial port: mount simulator on"); hello(&mut link, &cfg); }
            None => {}
        }
    }
    let was_open = link.is_open();
    for line in link.poll(time.delta_secs_f64(), info.time_scale) {
        let telemetry = mount.ingest(&line);
        if !telemetry { console.rx(&line); }
        if line.starts_with("READY") { hello(&mut link, &cfg); }
    }
    for s in std::mem::take(&mut link.sent) { console.tx(&s); }
    if was_open && !link.is_open() {
        console.note(&format!("link lost: {}", link.last_error));
        if tracker.active() { tracker.abort("link lost"); }
        for e in std::mem::take(&mut tracker.events) { console.note(&e); }
    }
}

/// After a (re)connect: identify, set the slew limits and ask for telemetry
fn hello(link: &mut SerialLink, cfg: &ControlConfig) {
    link.send("ID");
    link.send(&format!("RATE {:.1} {:.1}", cfg.mount.az_rate_dps, cfg.mount.el_rate_dps));
    link.send(&format!("TEL {:.1}", cfg.serial.telemetry_hz));
}

//------------------------------------------------------------------------------------------ target
fn target_tick(
    time: Res<Time>, cfg: Res<ControlConfig>, vcfg: Res<ViewerCfg>, geom: Res<Geom>,
    sel: Res<Selected>, cat: Res<Catalog>, orbits: Res<Orbits>, sim: Res<Sim>, mode: Res<Mode>, ranks: Res<Ranks>,
    mut info: ResMut<TargetInfo>, mut tracker: ResMut<Tracker>,
    (mount, auto, mut link, mut console): (Res<MountState>, Res<AutoArm>, ResMut<SerialLink>, ResMut<Console>),   // one param: Bevy allows 16
    mut last_pick: Local<Option<Option<usize>>>,
) {
    let app_s = time.elapsed_secs_f64();
    info.sim_jd = sim.jd(); info.real_jd = now_jd();
    info.history_mode = *mode == Mode::History;
    info.column = sel.0;
    //The pick changed. With AUTO on, the procedure starts for the new satellite (restarting whatever ran);
    //clearing the pick aborts and parks. With AUTO off a running sequence keeps its satellite: ABORT, then ARM.
    let changed = *last_pick != Some(sel.0);
    *last_pick = Some(sel.0);
    if changed {
        match sel.0 {
            Some(col) if auto.0 => {
                if tracker.active() { if let Step::Send(c) = tracker.abort("new pick") { link.send(&c); } }
                let name = cat.ids.get(col).copied().flatten().and_then(|id| cat.names.get(&id).cloned()).unwrap_or_else(|| format!("track #{col}"));
                tracker.start(col, &name, app_s);
            }
            None if tracker.active() => { if let Step::Send(c) = tracker.abort("pick cleared") { link.send(&c); } link.send("PARK"); console.note("no target: parking"); }
            _ => {}
        }
    } else if let (Some(plan), Some(col)) = (&tracker.plan, sel.0) {
        if tracker.active() && plan.column != col && !tracker.message.starts_with("pick changed") {
            tracker.message = format!("pick changed; still {} {}. ABORT then ARM to switch", if tracker.phase == Phase::Tracking { "tracking" } else { "armed for" }, plan.name);
        }
    }
    let Some(col) = sel.0 else { info.norad = None; info.name.clear(); info.have_data = false; info.preview = None; info.preview_for = None; info.point_err_deg = None; return };
    if col >= orbits.0.len() { return; }
    info.norad = cat.ids.get(col).copied().flatten();
    info.name = info.norad.and_then(|id| cat.names.get(&id).cloned()).unwrap_or_else(|| format!("track #{col}"));
    info.rank = ranks.entries.iter().find(|e| e.pass.column == col).map(|e| (e.rank, e.score));
    let epoch = cat.epochs.get(col).copied().flatten().unwrap_or(NOT_YET_JD);
    let track = Track { m: &orbits.0[col], epoch_jd: epoch, cfg: &vcfg };
    let sta = Station::from_cfg(&vcfg);
    //Where it is on the clock the dish follows (real time with a real mount, the viewer's time on the simulator)
    let jd = info.clock_jd;
    info.have_data = epoch < NOT_YET_JD / 2.0 && track.covers(jd);
    info.point_err_deg = None;
    if info.have_data {
        let r = track.r(jd); let v = track.v(jd);
        let (b, e, rng) = sta.look(r, jd);
        info.bearing = b; info.el = e; info.range = rng; info.range_rate = sta.range_rate(r, v, jd);
        if let Some((a, m)) = mount.fb {
            let (db, de) = geom.0.sky_of(a, m);
            info.point_err_deg = Some(angle_between(db, de, b, e));
        }
    }
    //Transmitter: first SatNOGS record with a downlink
    info.downlink_hz = None; info.tx_desc.clear(); info.tx_mode.clear();
    if let Some(txs) = info.norad.and_then(|id| cat.transmitters.get(&id)) {
        if let Some(tx) = txs.iter().find(|t| t["downlink_low"].as_f64().is_some()) {
            info.downlink_hz = tx["downlink_low"].as_f64();
            info.tx_desc = tx["description"].as_str().unwrap_or("").to_string();
            info.tx_mode = tx["mode"].as_str().unwrap_or("").to_string();
        }
    }
    //Next pass + mount path: on a new pick, then every 20 s (the pass may end, the track may extend), and
    //at once after a clock jump (warp, mode change) of more than a minute
    let jumped = (info.preview_at_jd - jd).abs() * 86400.0 > 60.0 + 20.0 * info.time_scale.max(1.0);
    let stale = info.preview_for != Some(col) || app_s - info.preview_at > 20.0 || jumped;
    if stale && info.have_data {
        info.preview_at = app_s; info.preview_at_jd = jd; info.preview_for = Some(col);
        let mask = if cfg.tracking.mask_deg > 0.0 { cfg.tracking.mask_deg } else { vcfg.station.elevation_mask_deg };
        match find_next_pass(&track, &sta, jd, jd + cfg.tracking.lookahead_hours / 24.0, mask) {
            Some(pass) => {
                let samples = sample_pass(&track, &sta, &pass, cfg.tracking.sample_seconds);
                match geom.0.solve_path(&samples) {
                    Some(path) => { info.preview_note.clear(); info.preview = Some(Plan { column: col, name: info.name.clone(), pass, samples, path }); }
                    None => { info.preview = None; info.preview_note = "pass too short to solve".into(); }
                }
            }
            None => { info.preview = None; info.preview_note = format!("no pass above {mask:.0} deg in the next {:.0} h of data", cfg.tracking.lookahead_hours); }
        }
    } else if !info.have_data {
        info.preview = None; info.preview_for = None;
        info.preview_note = if epoch >= NOT_YET_JD / 2.0 { "track not propagated yet".into() } else { "no propagated data at this time".into() };
    }
}

//------------------------------------------------------------------------------------------ actions
fn apply_actions(
    mut actions: ResMut<Actions>, cfg: Res<ControlConfig>, geom: Res<Geom>, mut motor: ResMut<Motor>,
    mut link: ResMut<SerialLink>, mut console: ResMut<Console>, mut tracker: ResMut<Tracker>, mount: Res<MountState>, info: Res<TargetInfo>,
    time: Res<Time>, mut auto: ResMut<AutoArm>, mut warp: ResMut<Warp>, mut sim: ResMut<Sim>, mut mode: ResMut<Mode>,
) {
    let app_s = time.elapsed_secs_f64();
    let base = mount.cmd.or(mount.fb).unwrap_or((cfg.mount.park_az_deg, cfg.mount.park_el_deg));
    let steps = [0.5, 1.0, 5.0, 10.0];
    let _ = &warp;
    for a in std::mem::take(&mut actions.0) {
        let manual = matches!(a, Action::JogAzNeg | Action::JogAzPos | Action::JogElNeg | Action::JogElPos | Action::Park | Action::Home | Action::Aim);
        if manual && tracker.active() { console.note("the procedure is running: ABORT (Escape) before moving the mount by hand"); continue; }
        if manual && !link.is_open() { console.note("link closed: CONNECT or SIM first"); continue; }
        match a {
            Action::JogAzNeg => { let (az, el) = geom.0.clamp(base.0 - motor.step, base.1); link.send(&format!("GO {az:.2} {el:.2}")); }
            Action::JogAzPos => { let (az, el) = geom.0.clamp(base.0 + motor.step, base.1); link.send(&format!("GO {az:.2} {el:.2}")); }
            Action::JogElNeg => { let (az, el) = geom.0.clamp(base.0, base.1 - motor.step); link.send(&format!("GO {az:.2} {el:.2}")); }
            Action::JogElPos => { let (az, el) = geom.0.clamp(base.0, base.1 + motor.step); link.send(&format!("GO {az:.2} {el:.2}")); }
            Action::StepCycle => { let i = steps.iter().position(|s| (*s - motor.step).abs() < 1e-9).unwrap_or(0); motor.step = steps[(i + 1) % steps.len()]; }
            Action::Stop => { if tracker.active() { tracker.abort("STOP"); } link.send("STOP"); }
            Action::Park => link.send("PARK"),
            Action::Home => { let (az, el) = geom.0.clamp(geom.0.az_travel / 2.0, cfg.mount.home_el_deg); link.send(&format!("GO {az:.2} {el:.2}")); }
            Action::Connect => { link.close(); link.next_reconnect = 0.0; console.note("reconnecting"); }
            Action::Disconnect => { if tracker.active() { tracker.abort("link closed"); } link.close(); link.next_reconnect = f64::MAX; console.note("link closed (CONNECT to reopen)"); }
            Action::Sim => { link.open_sim(&geom.0, cfg.mount.az_rate_dps, cfg.mount.el_rate_dps); link.next_reconnect = f64::MAX; console.note("simulator on"); hello(&mut link, &cfg); }
            Action::ClearConsole => console.lines.clear(),
            Action::Aim => {
                //Point at the satellite where it is right now, by the shortest move from the current pose
                if !info.have_data { console.note(&format!("nothing to aim at: {}", if info.column.is_none() { "pick a satellite in ORBIT VIEW" } else { info.preview_note.as_str() })); continue; }
                if info.el < geom.0.el_min { console.note(&format!("{} is below the horizon (el {:.1})", info.name, info.el)); continue; }
                match geom.0.nearest_pose(info.bearing, info.el, base) {
                    Some((az, el, flip)) => { console.note(&format!("AIM {}: bearing {:.1} el {:.1} -> mount {:.2} {:.2} ({})", info.name, info.bearing, info.el, az, el, flip.name())); link.send(&format!("GO {az:.2} {el:.2}")); }
                    None => console.note("no mount pose reaches that direction"),
                }
            }
            Action::Abort => { if let Step::Send(s) = tracker.abort("by hand") { link.send(&s); } }
            Action::Auto => { auto.0 = !auto.0; console.note(&format!("auto procedure on pick: {}", if auto.0 { "ON" } else { "OFF" })); }
            Action::Arm => {
                if tracker.active() { console.note("procedure already running; ABORT first"); continue; }
                if !link.is_open() { console.note("link closed: CONNECT or SIM before arming"); continue; }
                match info.column {
                    Some(col) => tracker.start(col, &info.name, app_s),
                    None => console.note("nothing to arm: pick a satellite in ORBIT VIEW"),
                }
            }
            Action::Warp => {
                if link.sim.is_none() { console.note("WARP is for the simulator only: a real mount follows real time"); continue; }
                let Some(p) = tracker.plan.as_ref().or(info.preview.as_ref()) else { console.note(&format!("nothing to warp to: {}", if info.column.is_none() { "pick a satellite" } else { info.preview_note.as_str() })); continue };
                let target = p.pass.aos_jd - cfg.tracking.warp_lead_s / 86400.0;
                warp_to(&mut warp, &mut sim, &mut mode, target, cfg.tracking.warp_speed);
                console.note(&format!("WARP: viewer clock -> {} ({:.0} s before AOS of {}) at x{:.0}", jd_utc_full(target), cfg.tracking.warp_lead_s, p.name, cfg.tracking.warp_speed));
            }
        }
    }
}

fn tracker_tick(time: Res<Time>, cfg: Res<ControlConfig>, geom: Res<Geom>, mut tracker: ResMut<Tracker>, mut link: ResMut<SerialLink>, mut console: ResMut<Console>, mount: Res<MountState>, info: Res<TargetInfo>) {
    if tracker.active() || tracker.phase == Phase::Done {
        if tracker.active() && !link.is_open() { tracker.abort("link closed"); }
        else {
            let inp = Inputs {
                now_jd: info.clock_jd, app_s: time.elapsed_secs_f64(), time_scale: info.time_scale,
                link_open: link.is_open(), link_name: link.port_name(), firmware_alive: mount.alive(),
                have_data: info.have_data, data_note: info.preview_note.clone(),
                preview: info.preview.as_ref(), preview_note: info.preview_note.clone(),
                fb: mount.fb, moving: mount.moving,
                az_rate_limit: cfg.mount.az_rate_dps, el_rate_limit: cfg.mount.el_rate_dps,
            };
            for cmd in tracker.tick(&inp, &geom.0, &cfg.tracking) { link.send(&cmd); }
        }
    }
    for e in std::mem::take(&mut tracker.events) { console.note(&e); }
}

/// Simulator only: once the procedure is armed for a pass that is still far off, move the viewer's clock
/// to warp_lead_s before AOS at warp_speed so the pass plays out now; when the sequence ends (or is
/// aborted) put the viewer back on LIVE time.
fn warp_tick(cfg: Res<ControlConfig>, link: Res<SerialLink>, tracker: Res<Tracker>, info: Res<TargetInfo>, mut warp: ResMut<Warp>, mut sim: ResMut<Sim>, mut mode: ResMut<Mode>, mut console: ResMut<Console>) {
    if link.sim.is_none() {
        if warp.active { unwarp(&mut warp, &mut sim, &mut mode); console.note("real link: viewer clock back to LIVE"); }
        return;
    }
    if !cfg.tracking.sim_warp { return; }
    if !warp.active && matches!(tracker.phase, Phase::Slew | Phase::Armed) {
        if let Some(p) = &tracker.plan {
            let until = (p.pass.aos_jd - info.clock_jd) * 86400.0;
            if until > cfg.tracking.warp_lead_s + 5.0 {
                let target = p.pass.aos_jd - cfg.tracking.warp_lead_s / 86400.0;
                warp_to(&mut warp, &mut sim, &mut mode, target, cfg.tracking.warp_speed);
                console.note(&format!("SIM WARP: AOS of {} is {} away; viewer clock -> {} at x{:.0}", p.name, crate::tracking::fmt_countdown(until), jd_utc_full(target), cfg.tracking.warp_speed));
            }
        }
    }
    if warp.active && matches!(tracker.phase, Phase::Idle | Phase::Done) {
        unwarp(&mut warp, &mut sim, &mut mode);
        console.note("SIM WARP over: viewer clock back to LIVE");
    }
}

fn warp_to(warp: &mut Warp, sim: &mut Sim, mode: &mut Mode, target_jd: f64, speed: f64) {
    if !warp.active { warp.prev_speed = sim.speed; warp.prev_mode_live = *mode == Mode::Live; warp.active = true; }
    *mode = Mode::History;
    sim.jd0 = sim.jd_hist0;
    sim.t_max = ((sim.jd_end - sim.jd0) * 86400.0).max(0.0);
    sim.t = ((target_jd - sim.jd0) * 86400.0).clamp(0.0, sim.t_max);
    sim.speed = speed; sim.paused = false; sim.exhausted = false;
}

fn unwarp(warp: &mut Warp, sim: &mut Sim, mode: &mut Mode) {
    warp.active = false;
    sim.speed = warp.prev_speed;
    *mode = Mode::Live;
    sim.jd0 = now_jd(); sim.t = 0.0; sim.exhausted = false;
    sim.t_max = ((sim.jd_end - sim.jd0) * 86400.0).max(0.0);
}

/// Angle between two sky directions (bearing, elevation), degrees
pub fn angle_between(b1: f64, e1: f64, b2: f64, e2: f64) -> f64 {
    let v = |b: f64, e: f64| { let (sb, cb) = b.to_radians().sin_cos(); let (se, ce) = e.to_radians().sin_cos(); [sb * ce, cb * ce, se] };
    let (a, c) = (v(b1, e1), v(b2, e2));
    (a[0] * c[0] + a[1] * c[1] + a[2] * c[2]).clamp(-1.0, 1.0).acos().to_degrees()
}

//------------------------------------------------------------------------------------------ text
pub fn jd_local(jd: f64) -> String {
    let unix = (jd - 2440587.5) * 86400.0;
    chrono::DateTime::<chrono::Utc>::from_timestamp(unix.floor() as i64, 0)
        .map(|dt| dt.with_timezone(&chrono::Local).format("%H:%M:%S").to_string()).unwrap_or_else(|| "-".into())
}
pub fn jd_utc_full(jd: f64) -> String {
    let unix = (jd - 2440587.5) * 86400.0;
    chrono::DateTime::<chrono::Utc>::from_timestamp(unix.floor() as i64, 0).map(|dt| dt.format("%Y-%m-%d %H:%M:%S UTC").to_string()).unwrap_or_else(|| "-".into())
}

fn live_text(info: &TargetInfo, tracker: &Tracker, mount: &MountState, geom: &MountGeom, auto: bool) -> String {
    let mut s = String::new();
    match info.column {
        None => { s += "TARGET     none: click a satellite in ORBIT VIEW\n"; }
        Some(_) => {
            s += &format!("TARGET     {}{}\n", info.name, info.norad.map_or(String::new(), |n| format!("   NORAD {n}")));
            if let Some((r, sc)) = info.rank { s += &format!("RANK       #{r}   score {sc:.3}\n"); }
        }
    }
    s += &format!("CLOCK      {}   {}{}\n", jd_utc_full(info.clock_jd), info.clock_label,
        if info.time_scale != 1.0 { format!("   x{:.0}", info.time_scale) } else if info.history_mode && info.clock_label.starts_with("REAL") { "   (viewer in HISTORY mode: a real mount follows real time)".into() } else { String::new() });
    if info.column.is_some() {
        if info.have_data {
            let vis = if info.el >= 0.0 { "above the horizon" } else { "below the horizon" };
            s += &format!("NOW        bearing {:6.1}   el {:5.1}   range {:6.0} km   {:+.3} km/s   {}\n", info.bearing, info.el, info.range, info.range_rate, vis);
            match info.downlink_hz {
                Some(f) => s += &format!("DOWNLINK   {:.4} MHz {} {}   doppler {:+.2} kHz\n", f / 1e6, info.tx_mode, info.tx_desc, -info.range_rate / 299792.458 * f / 1e3),
                None => s += "DOWNLINK   no frequency listed\n",
            }
        } else { s += &format!("NOW        {}\n", info.preview_note); }
        match &info.preview {
            Some(p) => {
                let until = (p.pass.aos_jd - info.real_jd) * 86400.0;
                s += &format!("NEXT PASS  AOS {} ({})   LOS {}   max el {:.0} at {}   {:.1} min\n", jd_local(p.pass.aos_jd),
                    if until > 0.0 { format!("in {}", crate::tracking::fmt_countdown(until)) } else { "in progress".into() },
                    jd_local(p.pass.los_jd), p.pass.max_el, jd_local(p.pass.max_el_jd), (p.pass.los_jd - p.pass.aos_jd) * 1440.0);
                let (a0, a1) = p.path.points.iter().fold((f64::MAX, f64::MIN), |(lo, hi), q| (lo.min(q.1), hi.max(q.1)));
                s += &format!("MOUNT PATH {}{}   az {:.0}..{:.0} (bearing {:.0}..{:.0})   peak rate az {:.2} el {:.2} deg/s   margin {:.0}{}\n",
                    p.path.flip.name(), if p.path.flips_mid > 0 { "+FLIP" } else { "" }, a0, a1, geom.bearing(a0), geom.bearing(a1), p.path.max_az_rate, p.path.max_el_rate, p.path.margin_deg,
                    if p.path.clipped { "   CLIPPED" } else { "" });
            }
            None if info.have_data => s += &format!("NEXT PASS  {}\n", info.preview_note),
            None => {}
        }
    }
    if let Some((a, e)) = mount.fb {
        let (b, el) = geom.sky_of(a, e);
        let lock = match info.point_err_deg { Some(d) if d < 1.0 => format!("   LOCKED  error {d:.2} deg"), Some(d) => format!("   error {d:.1} deg"), None => String::new() };
        s += &format!("DISH       bearing {:.1}   el {:.1}   (mount {:.1} / {:.1}){}{}\n", b, el, a, e, if mount.moving { "   moving" } else { "" }, lock);
    }
    s += &format!("\nPROCEDURE  {}   {}{}\n", tracker.phase.label(), tracker.message, if let Some((_, n)) = &tracker.target { if tracker.active() { format!("   [{n}]") } else { String::new() } } else { String::new() });
    for st in &tracker.steps { s += &format!("  {} {:<10} {}\n", st.mark(), st.label, st.detail); }
    if let Some((a, e)) = tracker.last_cmd { if tracker.active() { s += &format!("           last GO {:.2} {:.2}   {} commands\n", a, e, tracker.commands_sent); } }
    s += &format!("\nA arm   I aim   W warp   Escape abort   AUTO {}", if auto { "on: a pick starts the procedure" } else { "off" });
    s
}

fn motor_text(link: &SerialLink, mount: &MountState, motor: &Motor, geom: &MountGeom, cfg: &ControlConfig, tracker: &Tracker) -> String {
    let mut s = String::new();
    s += &format!("LINK       {}   tx {}   rx {}{}\n", link.status, link.tx_count, link.rx_count, if link.last_error.is_empty() { String::new() } else { format!("   last error: {}", link.last_error) });
    s += &format!("FIRMWARE   {}   telemetry {}\n", if mount.firmware.is_empty() { "-" } else { mount.firmware.as_str() }, if mount.alive() { "OK" } else if link.is_open() { "STALE" } else { "-" });
    match mount.cmd { Some((a, e)) => s += &format!("COMMANDED  az {:7.2}   el {:7.2}   -> bearing {:.1}\n", a, e, geom.bearing(a)), None => s += "COMMANDED  -\n" }
    match mount.fb {
        Some((a, e)) => { let (b, el) = geom.sky_of(a, e); s += &format!("MEASURED   az {:7.2}   el {:7.2}   -> bearing {:.1}  el {:.1}   {}\n", a, e, b, el, if mount.moving { "MOVING" } else { "holding" }); }
        None => s += "MEASURED   -\n",
    }
    if let Some(enc) = mount.encoder { s += &format!("ENCODER    {enc:.0}\n"); }
    s += &format!("LIMITS     az 0..{:.0}   el {:.0}..{:.0}   zero bearing {:.1}   rate {:.0} / {:.0} deg/s\n", geom.az_travel, geom.el_min, geom.el_max, geom.az_zero(), cfg.mount.az_rate_dps, cfg.mount.el_rate_dps);
    s += &format!("STEP       {:.1} deg   mode {}\n", motor.step, if tracker.active() { "PROCEDURE (manual locked)" } else { "MANUAL" });
    s += "\narrows jog   [ ] step   S stop   P park   H home";
    s
}

fn refresh_text(
    layout: Res<Layout>, cfg: Res<ControlConfig>, info: Res<TargetInfo>, tracker: Res<Tracker>, mount: Res<MountState>, geom: Res<Geom>,
    link: Res<SerialLink>, motor: Res<Motor>, mut console: ResMut<Console>, time: Res<Time>, view: Res<MountView>, auto: Res<AutoArm>,
    mut bodies: Query<(&TileBody, &mut Text)>, mut titles: Query<(&TileTitle, &mut Text), Without<TileBody>>,
    mut input_line: Query<&mut Text, (With<ConsoleInputText>, Without<TileBody>, Without<TileTitle>)>,
) {
    let line_h = cfg.window.font_size * 1.2;
    for (b, mut text) in &mut bodies {
        let Some(r) = layout.rects[b.0.idx()] else { continue };
        text.0 = match b.0 {
            Tile::Live => live_text(&info, &tracker, &mount, &geom.0, auto.0),
            Tile::Motor => motor_text(&link, &mount, &motor, &geom.0, &cfg, &tracker),
            Tile::Mount | Tile::Globe => { let _ = view; String::new() }
            Tile::Console => {
                let rows = (((r.h - (cfg.window.font_size + 8.0) - 12.0 - line_h * 2.0 - 30.0) / line_h).floor() as usize).max(1);
                let v = console.view(rows);
                let mut s = v.join("\n");
                if console.scroll > 0 { s += &format!("\n-- {} more below --", console.scroll); }
                s
            }
        };
    }
    for (t, mut text) in &mut titles {
        text.0 = match t.0 {
            Tile::Mount => match mount.fb {
                Some((a, e)) => format!("3 MOUNT   az {:.1}  el {:.1}   {}{}", a, e, if mount.moving { "MOVING" } else { "" },
                    match info.point_err_deg { Some(d) if d < 1.0 && tracker.phase == Phase::Tracking => format!("   LOCKED {d:.2} deg"), Some(d) if tracker.phase == Phase::Tracking => format!("   error {d:.1} deg"), _ => String::new() }),
                None => "3 MOUNT   (no telemetry)".into(),
            },
            Tile::Console => format!("4 SERIAL CONSOLE   {}", link.port_name()),
            Tile::Live => format!("1 LIVE DATA   {}", if tracker.phase == Phase::Idle { "" } else { tracker.phase.label() }),
            Tile::Globe => format!("0 ORBIT VIEW   {}{}", if info.history_mode { "HISTORY" } else { "LIVE" },
                                   if info.column.is_some() { format!("   pick {}", info.name) } else { String::new() }),
            other => other.title().to_string(),
        };
    }
    console.blink = time.elapsed_secs_f64();
    let cursor = if layout.focus == Tile::Console && (console.blink * 2.0) as i64 % 2 == 0 { "\u{2588}" } else { " " };
    for mut t in &mut input_line { t.0 = format!("> {}{}", console.input, cursor); }
}

//------------------------------------------------------------------------------------------ overlays
fn draw_overlays(
    cw: Res<CmdWindow>, windows: Query<&Window>, layout: Res<Layout>, pal: Res<Palette>, geom: Res<Geom>, view: Res<MountView>,
    mount: Res<MountState>, info: Res<TargetInfo>, tracker: Res<Tracker>, cfg: Res<ControlConfig>, mut gizmos: Gizmos<CmdLines>,
    mut dividers: Gizmos<CmdDividers>, booting: Res<Booting>, reveal: Option<Res<Reveal>>, time: Res<Time>,
) {
    if booting.0 { return; }
    let Some(w) = cw.0.and_then(|e| windows.get(e).ok()) else { return };
    let (ww, wh) = (w.width(), w.height());
    let to_world = |p: Vec2| Vec2::new(p.x - ww / 2.0, wh / 2.0 - p.y);
    let title_h = cfg.window.font_size + 8.0;
    let now = time.elapsed_secs_f64();
    //Bright dividers down the middle of every gap, and a frame around the page; after the boot page
    //they draw themselves in from one end
    let k = reveal.as_ref().map_or(1.0, |r| r.lines(now)) as f32;
    if pal.divider != Color::NONE {
        for (a, b) in &layout.dividers { dividers.line_2d(to_world(*a), to_world(*a + (*b - *a) * k), pal.divider); }
        let a = layout.area; let g = layout.gap / 2.0;
        let corners = [Vec2::new(a.x - g, a.y - g), Vec2::new(a.x + a.w + g, a.y - g), Vec2::new(a.x + a.w + g, a.y + a.h + g), Vec2::new(a.x - g, a.y + a.h + g)];
        for i in 0..4 { let (p0, p1) = (corners[i], corners[(i + 1) % 4]); dividers.line_2d(to_world(p0), to_world(p0 + (p1 - p0) * k), pal.divider.with_alpha(0.6)); }
    }
    let revealed = |t: Tile| reveal.as_ref().map_or(true, |r| r.tile(now, ALL.iter().position(|x| *x == t).unwrap_or(0)) >= 1.0);
    let target_sky = if info.have_data && info.el > -5.0 { Some((info.bearing, info.el)) } else { None };
    //MOUNT tile: the gimbal wireframe
    if let Some(r) = layout.rects[Tile::Mount.idx()].filter(|_| revealed(Tile::Mount)) {
        let body = Rect { x: r.x + 4.0, y: r.y + title_h + 4.0, w: r.w - 8.0, h: r.h - title_h - 8.0 };
        let colors = MountColors { fixed: pal.line_dim, head: pal.line, cradle: pal.line, dish: pal.bright, compass: pal.dim, gap: pal.warn.with_alpha(0.5), ray_fb: pal.bright, ray_cmd: pal.accent, ray_target: pal.good };
        let mut lines = Vec::with_capacity(600);
        draw_mount(&mut lines, body, &view, &geom.0, mount.fb, mount.cmd, target_sky, &colors);
        for (a, b, c) in lines {
            if let Some((a, b)) = crate::tiles::clip_segment(a, b, body) { gizmos.line_2d(to_world(a), to_world(b), c); }
        }
    }
    //LIVE tile: polar sky plot on the right (only when the tile is wide enough)
    if let Some(r) = layout.rects[Tile::Live.idx()].filter(|_| revealed(Tile::Live)) {
        if r.w > 780.0 && r.h > 220.0 {
            let radius = ((r.h - title_h - 70.0) / 2.0).min(150.0);
            let c = Vec2::new(r.x + r.w - radius - 14.0, r.y + title_h + radius + 12.0);
            let polar = |b: f64, e: f64| -> Vec2 { let rr = radius * ((90.0 - e.clamp(-5.0, 90.0)) / 90.0) as f32; c + Vec2::new(b.to_radians().sin() as f32, -(b.to_radians().cos() as f32)) * rr };
            for e in [0.0, 30.0, 60.0] { gizmos.circle_2d(bevy::math::Isometry2d::from_translation(to_world(c)), radius * ((90.0 - e) / 90.0) as f32, if e == 0.0 { pal.line } else { pal.line_dim }); }
            for b in [0.0, 90.0, 180.0, 270.0] { gizmos.line_2d(to_world(polar(b, 0.0)), to_world(polar(b, if b == 0.0 { -12.0 } else { -6.0 })), pal.dim); }
            //Mount reach: the two azimuth limits as ticks outside the horizon ring, and (travel under a full
            //turn only) the bearings the axis cannot reach as a dim arc
            for b in [geom.0.az_zero(), geom.0.az_zero() + geom.0.az_travel] { gizmos.line_2d(to_world(polar(b, -1.0)), to_world(polar(b, -9.0)), pal.warn.with_alpha(0.8)); }
            let gap = 360.0 - geom.0.az_travel;
            if gap > 0.5 {
                let start = geom.0.az_zero() + geom.0.az_travel;
                let n = 24;
                for i in 0..n {
                    let b0 = start + gap * i as f64 / n as f64; let b1 = start + gap * (i + 1) as f64 / n as f64;
                    gizmos.line_2d(to_world(polar(b0, -4.0)), to_world(polar(b1, -4.0)), pal.warn.with_alpha(0.6));
                }
            }
            if let Some(p) = tracker.plan.as_ref().or(info.preview.as_ref()) {
                let mut prev: Option<Vec2> = None;
                for &(_, b, e) in &p.samples {
                    let q = polar(b, e);
                    if let Some(pp) = prev { gizmos.line_2d(to_world(pp), to_world(q), pal.accent); }
                    prev = Some(q);
                }
                if let Some(&(_, b, e)) = p.samples.first() { let q = to_world(polar(b, e)); gizmos.circle_2d(bevy::math::Isometry2d::from_translation(q), 4.0, pal.good); }
                if let Some(&(_, b, e)) = p.samples.last() { let q = to_world(polar(b, e)); gizmos.line_2d(q + Vec2::new(-4.0, -4.0), q + Vec2::new(4.0, 4.0), pal.warn); gizmos.line_2d(q + Vec2::new(-4.0, 4.0), q + Vec2::new(4.0, -4.0), pal.warn); }
            }
            if let Some((b, e)) = target_sky { let q = to_world(polar(b, e)); gizmos.line_2d(q + Vec2::new(-6.0, 0.0), q + Vec2::new(6.0, 0.0), pal.bright); gizmos.line_2d(q + Vec2::new(0.0, -6.0), q + Vec2::new(0.0, 6.0), pal.bright); }
            if let Some((a, e)) = mount.fb { let (b, el) = geom.0.sky_of(a, e); let q = to_world(polar(b, el)); gizmos.circle_2d(bevy::math::Isometry2d::from_translation(q), 6.0, pal.good); }
        }
    }
    //Focus mark: a short bright bar under the focused tile's title
    if let Some(r) = layout.rects[layout.focus.idx()].filter(|_| revealed(layout.focus)) {
        let y = r.y + title_h;
        gizmos.line_2d(to_world(Vec2::new(r.x + 1.0, y)), to_world(Vec2::new(r.x + r.w - 1.0, y)), pal.border_focus.with_alpha(0.8));
    }
}

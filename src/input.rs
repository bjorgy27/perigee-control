/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Input routing inside the one window. Bevy's ButtonInput resources are global, so a key typed into the
/// serial console would also drive the viewer (Space pauses it, L flips modes, ...). This module reads
/// the raw events, keeps the control page's own copy, and erases from the shared resources whatever the
/// viewer must not see: every key while a command tile has the focus or the boot page is up, page keys
/// (Ctrl+..., Tab) always, and mouse presses that land outside the globe. With the ORBIT VIEW tile
/// focused the viewer gets its keys exactly as when it runs alone.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use crate::boot::Booting;
use crate::tiles::{Layout, Tile};
use bevy::input::keyboard::{Key, KeyboardInput};
use bevy::input::mouse::{MouseButtonInput, MouseWheel};
use bevy::input::ButtonState;
use bevy::prelude::*;
use bevy::window::{CursorMoved, PrimaryWindow};
use perigee_viewer::ViewerFocus;
use std::collections::HashSet;

/// The window everything lives in (the viewer's primary window)
#[derive(Resource, Default)]
pub struct CmdWindow(pub Option<Entity>);

#[derive(Clone, Debug)]
#[allow(dead_code)]
pub struct KeyPress { pub code: KeyCode, pub logical: Key, pub repeat: bool }

#[derive(Resource, Default)]
pub struct CmdInput {
    pub focused: bool,
    pub presses: Vec<KeyPress>,          // key presses this frame (with repeats)
    pub down: HashSet<KeyCode>,          // keys currently held
    pub ctrl: bool, pub shift: bool, pub alt: bool,
    pub cursor: Option<Vec2>,            // cursor position in the window (UI pixels)
    pub clicks: Vec<(MouseButton, Vec2)>,
    pub mouse_down: HashSet<MouseButton>,
    pub wheel: f32,
    pub motion: Vec2,
    pub viewer_keys: bool,               // this frame the viewer had the keyboard (ORBIT VIEW focused, no boot page)
}

impl CmdInput {
    pub fn pressed(&self, code: KeyCode) -> bool { self.presses.iter().any(|p| p.code == code) }
    /// Characters typed this frame (Ctrl/Alt combinations excluded)
    pub fn typed(&self) -> String {
        if self.ctrl || self.alt { return String::new(); }
        self.presses.iter().filter_map(|p| match &p.logical {
            Key::Character(s) => Some(s.to_string()),
            Key::Space => Some(" ".into()),
            _ => None,
        }).collect()
    }
}

fn is_modifier(c: KeyCode) -> bool {
    matches!(c, KeyCode::ControlLeft | KeyCode::ControlRight | KeyCode::ShiftLeft | KeyCode::ShiftRight | KeyCode::AltLeft | KeyCode::AltRight)
}

pub fn route_input(
    mut cmd: ResMut<CmdWindow>,
    primary: Query<(Entity, &Window), With<PrimaryWindow>>,
    layout: Res<Layout>,
    booting: Res<Booting>,
    mut vfocus: ResMut<ViewerFocus>,
    mut input: ResMut<CmdInput>,
    mut keys: ResMut<ButtonInput<KeyCode>>,
    mut mouse: ResMut<ButtonInput<MouseButton>>,
    mut key_ev: EventReader<KeyboardInput>,
    mut btn_ev: EventReader<MouseButtonInput>,
    mut wheel_ev: EventReader<MouseWheel>,
    mut cursor_ev: EventReader<CursorMoved>,
    mut motion_ev: EventReader<bevy::input::mouse::MouseMotion>,
) {
    input.presses.clear(); input.clicks.clear(); input.wheel = 0.0; input.motion = Vec2::ZERO;
    let Ok((win, window)) = primary.get_single() else { key_ev.clear(); btn_ev.clear(); wheel_ev.clear(); cursor_ev.clear(); motion_ev.clear(); return };
    cmd.0 = Some(win);
    input.focused = window.focused;
    let viewer_keys = !booting.0 && layout.focus == Tile::Globe;
    input.viewer_keys = viewer_keys;
    vfocus.0 = viewer_keys;

    for ev in key_ev.read() {
        if ev.window != win { continue; }
        match ev.state {
            ButtonState::Pressed => {
                input.down.insert(ev.key_code);
                if !is_modifier(ev.key_code) { input.presses.push(KeyPress { code: ev.key_code, logical: ev.logical_key.clone(), repeat: ev.repeat }); }
            }
            ButtonState::Released => { input.down.remove(&ev.key_code); }
        }
        let ctrl = input.down.contains(&KeyCode::ControlLeft) || input.down.contains(&KeyCode::ControlRight);
        //Page keys are ours in every state; everything else is ours unless the viewer has the keyboard
        let ours = !viewer_keys || ctrl || ev.key_code == KeyCode::Tab;
        if ours && !is_modifier(ev.key_code) { keys.reset(ev.key_code); }
    }
    input.ctrl = input.down.contains(&KeyCode::ControlLeft) || input.down.contains(&KeyCode::ControlRight);
    input.shift = input.down.contains(&KeyCode::ShiftLeft) || input.down.contains(&KeyCode::ShiftRight);
    input.alt = input.down.contains(&KeyCode::AltLeft) || input.down.contains(&KeyCode::AltRight);
    if !input.focused { input.down.clear(); }

    for ev in cursor_ev.read() { if ev.window == win { input.cursor = Some(ev.position); } }
    //Mouse presses reach the viewer only when they land on the globe; the release of a press it never
    //saw is nothing to it either (ButtonInput ignores a release of a button it does not hold)
    let on_globe = |p: Option<Vec2>| !booting.0 && p.map_or(false, |c| layout.globe_view().map_or(false, |r| r.contains(c)));
    for ev in btn_ev.read() {
        if ev.window != win { continue; }
        match ev.state {
            ButtonState::Pressed => {
                input.mouse_down.insert(ev.button);
                if let Some(c) = input.cursor { input.clicks.push((ev.button, c)); }
                if !on_globe(input.cursor) { mouse.reset(ev.button); }
            }
            ButtonState::Released => { input.mouse_down.remove(&ev.button); }
        }
    }
    for ev in wheel_ev.read() { if ev.window == win { input.wheel += ev.y; } }
    for ev in motion_ev.read() { if input.focused { input.motion += ev.delta; } }
    if !input.focused { input.mouse_down.clear(); }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Tiling for the control page: a binary split tree like Hyprland's dwindle layout, in one window.
/// Every leaf is one of the five tiles (the orbit view is one of them); every split divides its rectangle
/// between two children either side by side (vertical split) or stacked (horizontal split) at a ratio.
/// Hidden tiles are pruned before layout, a zoomed tile takes the whole page. Focus, swap and resize all
/// work on this tree. Every split also yields a divider: the bright line drawn down the middle of its gap.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use bevy::prelude::*;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum Tile { Live, Motor, Mount, Console, Globe }

pub const N: usize = 5;
pub const ALL: [Tile; N] = [Tile::Globe, Tile::Live, Tile::Motor, Tile::Mount, Tile::Console];

impl Tile {
    pub fn idx(self) -> usize { match self { Tile::Live => 0, Tile::Motor => 1, Tile::Mount => 2, Tile::Console => 3, Tile::Globe => 4 } }
    pub fn title(self) -> &'static str {
        match self { Tile::Live => "1 LIVE DATA", Tile::Motor => "2 MOTOR CONTROL", Tile::Mount => "3 MOUNT", Tile::Console => "4 SERIAL CONSOLE", Tile::Globe => "0 ORBIT VIEW" }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Rect { pub x: f32, pub y: f32, pub w: f32, pub h: f32 }
impl Rect {
    pub fn contains(&self, p: Vec2) -> bool { p.x >= self.x && p.x <= self.x + self.w && p.y >= self.y && p.y <= self.y + self.h }
    pub fn center(&self) -> Vec2 { Vec2::new(self.x + self.w / 2.0, self.y + self.h / 2.0) }
}

/// The part of segment a-b inside the rectangle (Liang-Barsky), None when it misses
pub fn clip_segment(a: Vec2, b: Vec2, r: Rect) -> Option<(Vec2, Vec2)> {
    let d = b - a;
    let (mut t0, mut t1) = (0.0f32, 1.0f32);
    for (p, q) in [(-d.x, a.x - r.x), (d.x, r.x + r.w - a.x), (-d.y, a.y - r.y), (d.y, r.y + r.h - a.y)] {
        if p == 0.0 { if q < 0.0 { return None; } continue; }
        let t = q / p;
        if p < 0.0 { if t > t1 { return None; } if t > t0 { t0 = t; } }
        else { if t < t0 { return None; } if t < t1 { t1 = t; } }
    }
    Some((a + d * t0, a + d * t1))
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Dir { Left, Right, Up, Down }

#[derive(Clone, Debug)]
pub enum Node {
    Leaf(Tile),
    Split { side_by_side: bool, ratio: f32, a: Box<Node>, b: Box<Node> },
}

impl Node {
    fn prune(&self, hidden: &[bool; N]) -> Option<Node> {
        match self {
            Node::Leaf(t) => if hidden[t.idx()] { None } else { Some(Node::Leaf(*t)) },
            Node::Split { side_by_side, ratio, a, b } => match (a.prune(hidden), b.prune(hidden)) {
                (Some(a), Some(b)) => Some(Node::Split { side_by_side: *side_by_side, ratio: *ratio, a: Box::new(a), b: Box::new(b) }),
                (Some(a), None) => Some(a),
                (None, Some(b)) => Some(b),
                (None, None) => None,
            },
        }
    }
    fn layout(&self, r: Rect, gap: f32, out: &mut [Option<Rect>; N], dividers: &mut Vec<(Vec2, Vec2)>) {
        match self {
            Node::Leaf(t) => out[t.idx()] = Some(r),
            Node::Split { side_by_side, ratio, a, b } => {
                if *side_by_side {
                    let wa = ((r.w - gap) * ratio).round();
                    a.layout(Rect { x: r.x, y: r.y, w: wa, h: r.h }, gap, out, dividers);
                    b.layout(Rect { x: r.x + wa + gap, y: r.y, w: r.w - wa - gap, h: r.h }, gap, out, dividers);
                    let x = r.x + wa + gap / 2.0;
                    dividers.push((Vec2::new(x, r.y), Vec2::new(x, r.y + r.h)));
                } else {
                    let ha = ((r.h - gap) * ratio).round();
                    a.layout(Rect { x: r.x, y: r.y, w: r.w, h: ha }, gap, out, dividers);
                    b.layout(Rect { x: r.x, y: r.y + ha + gap, w: r.w, h: r.h - ha - gap }, gap, out, dividers);
                    let y = r.y + ha + gap / 2.0;
                    dividers.push((Vec2::new(r.x, y), Vec2::new(r.x + r.w, y)));
                }
            }
        }
    }
    /// Path from the root to the leaf holding `t`: true = went into child a
    fn path_to(&self, t: Tile, path: &mut Vec<bool>) -> bool {
        match self {
            Node::Leaf(x) => *x == t,
            Node::Split { a, b, .. } => {
                path.push(true);
                if a.path_to(t, path) { return true; }
                path.pop(); path.push(false);
                if b.path_to(t, path) { return true; }
                path.pop(); false
            }
        }
    }
    fn get_mut(&mut self, path: &[bool]) -> &mut Node {
        match path.split_first() {
            None => self,
            Some((first, rest)) => match self {
                Node::Split { a, b, .. } => if *first { a.get_mut(rest) } else { b.get_mut(rest) },
                Node::Leaf(_) => self,
            },
        }
    }
    fn swap_tiles(&mut self, x: Tile, y: Tile) {
        match self {
            Node::Leaf(t) => { if *t == x { *t = y } else if *t == y { *t = x } }
            Node::Split { a, b, .. } => { a.swap_tiles(x, y); b.swap_tiles(x, y); }
        }
    }
}

#[derive(Resource)]
pub struct Layout {
    pub root: Node,
    pub focus: Tile,
    pub zoom: Option<Tile>,
    pub hidden: [bool; N],
    pub gap: f32,
    pub title_h: f32,                      // height of every tile's title bar (the globe renders below it)
    pub rects: [Option<Rect>; N],
    pub dividers: Vec<(Vec2, Vec2)>,       // one line per split, down the middle of its gap
    pub area: Rect,
}

impl Layout {
    /// At launch: ORBIT VIEW on the left (globe_ratio of the width), the four command tiles in a
    /// 2 x 2 grid on the right (LIVE over MOTOR, MOUNT over CONSOLE)
    pub fn new(main_ratio: f32, row_ratio: f32, gap: f32, globe_ratio: f32, title_h: f32) -> Self {
        let col = |top: Tile, bottom: Tile| Node::Split { side_by_side: false, ratio: row_ratio, a: Box::new(Node::Leaf(top)), b: Box::new(Node::Leaf(bottom)) };
        let grid = Node::Split { side_by_side: true, ratio: main_ratio, a: Box::new(col(Tile::Live, Tile::Motor)), b: Box::new(col(Tile::Mount, Tile::Console)) };
        Layout {
            root: Node::Split { side_by_side: true, ratio: globe_ratio.clamp(0.15, 0.85), a: Box::new(Node::Leaf(Tile::Globe)), b: Box::new(grid) },
            focus: Tile::Globe, zoom: None, hidden: [false; N], gap, title_h, rects: [None; N], dividers: Vec::new(), area: Rect::default(),
        }
    }

    pub fn compute(&mut self, area: Rect) {
        self.area = area;
        self.rects = [None; N];
        self.dividers.clear();
        if let Some(z) = self.zoom { if !self.hidden[z.idx()] { self.rects[z.idx()] = Some(area); return; } else { self.zoom = None; } }
        if let Some(tree) = self.root.prune(&self.hidden) { let mut d = Vec::new(); tree.layout(area, self.gap, &mut self.rects, &mut d); self.dividers = d; }
    }

    /// Where the globe is drawn: the ORBIT VIEW tile below its title bar, inside its 1 px frame
    pub fn globe_view(&self) -> Option<Rect> {
        let r = self.rects[Tile::Globe.idx()]?;
        let v = Rect { x: r.x + 1.0, y: r.y + self.title_h, w: r.w - 2.0, h: r.h - self.title_h - 1.0 };
        if v.w >= 8.0 && v.h >= 8.0 { Some(v) } else { None }
    }

    pub fn visible(&self, t: Tile) -> bool { self.rects[t.idx()].is_some() }

    /// The visible tile next to the focused one in this direction (nearest centre in that half-plane)
    pub fn neighbour(&self, dir: Dir) -> Option<Tile> {
        let from = self.rects[self.focus.idx()]?;
        let c = from.center();
        let mut best: Option<(Tile, f32)> = None;
        for t in ALL {
            if t == self.focus { continue; }
            let Some(r) = self.rects[t.idx()] else { continue };
            let d = r.center() - c;
            let ok = match dir { Dir::Left => d.x < -1.0 && d.x.abs() >= d.y.abs(), Dir::Right => d.x > 1.0 && d.x.abs() >= d.y.abs(),
                                 Dir::Up => d.y < -1.0 && d.y.abs() > d.x.abs(), Dir::Down => d.y > 1.0 && d.y.abs() > d.x.abs() };
            if !ok { continue; }
            let dist = d.length();
            if best.map_or(true, |(_, bd)| dist < bd) { best = Some((t, dist)); }
        }
        best.map(|(t, _)| t)
    }

    pub fn focus_dir(&mut self, dir: Dir) { if let Some(t) = self.neighbour(dir) { self.focus = t; } }
    pub fn swap_dir(&mut self, dir: Dir) { if let Some(t) = self.neighbour(dir) { let f = self.focus; self.root.swap_tiles(f, t); } }

    /// Grow (delta > 0) or shrink the focused tile along one axis: adjusts the nearest ancestor split of that orientation
    pub fn resize(&mut self, side_by_side: bool, delta: f32) {
        let mut path = Vec::new();
        if !self.root.path_to(self.focus, &mut path) { return; }
        for depth in (0..path.len()).rev() {
            let node = self.root.get_mut(&path[..depth]);
            if let Node::Split { side_by_side: s, ratio, .. } = node {
                if *s == side_by_side {
                    let in_a = path[depth];
                    *ratio = (*ratio + if in_a { delta } else { -delta }).clamp(0.15, 0.85);
                    return;
                }
            }
        }
    }

    /// Flip the orientation of the split right above the focused tile
    pub fn toggle_split(&mut self) {
        let mut path = Vec::new();
        if !self.root.path_to(self.focus, &mut path) || path.is_empty() { return; }
        if let Node::Split { side_by_side, .. } = self.root.get_mut(&path[..path.len() - 1]) { *side_by_side = !*side_by_side; }
    }

    pub fn toggle_hidden(&mut self, t: Tile) {
        self.hidden[t.idx()] = !self.hidden[t.idx()];
        if self.hidden.iter().all(|h| *h) { self.hidden[t.idx()] = false; }   // never hide everything
        if self.hidden[self.focus.idx()] { if let Some(v) = ALL.iter().find(|x| !self.hidden[x.idx()]) { self.focus = *v; } }
    }

    pub fn toggle_zoom(&mut self) { self.zoom = if self.zoom == Some(self.focus) { None } else { Some(self.focus) }; }

    pub fn tile_at(&self, p: Vec2) -> Option<Tile> { ALL.into_iter().find(|t| self.rects[t.idx()].map_or(false, |r| r.contains(p))) }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn lay() -> Layout {
        let mut l = Layout::new(0.5, 0.5, 4.0, 0.5, 20.0);
        l.hidden[Tile::Globe.idx()] = true; l.focus = Tile::Live;    // the four command tiles alone, as on the old command page
        l.compute(Rect { x: 0.0, y: 0.0, w: 1004.0, h: 804.0 }); l
    }

    #[test]
    fn globe_takes_the_left_half_with_a_divider() {
        let mut l = Layout::new(0.5, 0.5, 4.0, 0.5, 20.0);
        l.compute(Rect { x: 0.0, y: 0.0, w: 2012.0, h: 804.0 });
        assert_eq!(l.rects[Tile::Globe.idx()].unwrap(), Rect { x: 0.0, y: 0.0, w: 1004.0, h: 804.0 });
        assert_eq!(l.rects[Tile::Live.idx()].unwrap(), Rect { x: 1008.0, y: 0.0, w: 500.0, h: 400.0 });
        assert_eq!(l.globe_view().unwrap(), Rect { x: 1.0, y: 20.0, w: 1002.0, h: 783.0 });
        assert_eq!(l.dividers.len(), 4);   // globe | grid, LIVE/MOTOR column | MOUNT/CONSOLE column, and one per column
        assert!(l.dividers.contains(&(Vec2::new(1006.0, 0.0), Vec2::new(1006.0, 804.0))));
        l.focus = Tile::Globe; l.focus_dir(Dir::Right); assert_eq!(l.focus, Tile::Live);
        l.toggle_hidden(Tile::Globe); l.compute(l.area);
        assert!(l.globe_view().is_none());
        assert_eq!(l.rects[Tile::Live.idx()].unwrap().x, 0.0);
    }

    #[test]
    fn four_tiles_fill_the_page() {
        let l = lay();
        let r = |t: Tile| l.rects[t.idx()].unwrap();
        let r = [r(Tile::Live), r(Tile::Motor), r(Tile::Mount), r(Tile::Console)];
        assert_eq!(r[0], Rect { x: 0.0, y: 0.0, w: 500.0, h: 400.0 });         // LIVE top-left
        assert_eq!(r[1], Rect { x: 0.0, y: 404.0, w: 500.0, h: 400.0 });       // MOTOR under it
        assert_eq!(r[2], Rect { x: 504.0, y: 0.0, w: 500.0, h: 400.0 });       // MOUNT top-right
        assert_eq!(r[3], Rect { x: 504.0, y: 404.0, w: 500.0, h: 400.0 });     // CONSOLE bottom-right
    }

    #[test]
    fn segments_clip_to_the_tile() {
        let r = Rect { x: 0.0, y: 0.0, w: 100.0, h: 100.0 };
        assert_eq!(clip_segment(Vec2::new(10.0, 10.0), Vec2::new(20.0, 20.0), r), Some((Vec2::new(10.0, 10.0), Vec2::new(20.0, 20.0))));
        assert_eq!(clip_segment(Vec2::new(-50.0, 50.0), Vec2::new(150.0, 50.0), r), Some((Vec2::new(0.0, 50.0), Vec2::new(100.0, 50.0))));
        assert_eq!(clip_segment(Vec2::new(-50.0, -50.0), Vec2::new(-10.0, -10.0), r), None);
    }

    #[test]
    fn focus_moves_by_geometry() {
        let mut l = lay();
        l.focus_dir(Dir::Right); assert_eq!(l.focus, Tile::Mount);
        l.focus_dir(Dir::Down); assert_eq!(l.focus, Tile::Console);
        l.focus_dir(Dir::Left); assert_eq!(l.focus, Tile::Motor);
        l.focus_dir(Dir::Up); assert_eq!(l.focus, Tile::Live);
        l.focus_dir(Dir::Up); assert_eq!(l.focus, Tile::Live);   // nothing above: stays
    }

    #[test]
    fn swap_resize_hide_zoom() {
        let mut l = lay();
        l.swap_dir(Dir::Right); l.compute(l.area);
        assert!(l.rects[Tile::Mount.idx()].unwrap().x == 0.0 && l.rects[Tile::Live.idx()].unwrap().x == 504.0);
        l.focus = Tile::Live; l.resize(true, 0.1); l.compute(l.area);
        assert!(l.rects[Tile::Live.idx()].unwrap().w > 500.0);
        l.toggle_hidden(Tile::Console); l.compute(l.area);
        assert!(l.rects[Tile::Console.idx()].is_none());
        assert_eq!(l.rects[Tile::Live.idx()].unwrap().h, 804.0);   // its column mate (LIVE, after the swap) takes the full height
        l.toggle_zoom(); l.compute(l.area);
        assert_eq!(l.rects[Tile::Live.idx()].unwrap(), l.area);
        assert!(l.rects[Tile::Motor.idx()].is_none());
    }
}

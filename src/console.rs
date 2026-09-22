/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The serial console tile's model: a scrollback of everything that crossed the link (and local notes),
/// a line being typed, command history. Lines typed here go straight to the Arduino; lines starting
/// with '/' are handled locally (see `help`).
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use bevy::prelude::*;
use std::collections::VecDeque;

pub const MAX_LINES: usize = 600;

#[derive(Resource)]
pub struct Console {
    pub lines: VecDeque<String>,
    pub input: String,
    pub history: Vec<String>,
    pub hist_pos: Option<usize>,
    pub scroll: usize,         // lines scrolled up from the bottom
    pub blink: f64,
}
impl Default for Console {
    fn default() -> Self {
        let mut c = Console { lines: VecDeque::new(), input: String::new(), history: Vec::new(), hist_pos: None, scroll: 0, blink: 0.0 };
        c.note("PERIGEE serial console. Type a command for the mount and press Enter; /help for local commands.");
        c
    }
}

impl Console {
    fn push(&mut self, s: String) {
        if self.lines.len() >= MAX_LINES { self.lines.pop_front(); }
        self.lines.push_back(s);
    }
    pub fn rx(&mut self, line: &str) { self.push(format!("< {line}")); }
    pub fn tx(&mut self, line: &str) { self.push(format!("> {line}")); }
    pub fn note(&mut self, line: &str) { self.push(format!("# {line}")); }

    /// Enter pressed: returns the line to act on (moved into history)
    pub fn submit(&mut self) -> Option<String> {
        let line = self.input.trim().to_string();
        self.input.clear(); self.hist_pos = None; self.scroll = 0;
        if line.is_empty() { return None; }
        if self.history.last() != Some(&line) { self.history.push(line.clone()); }
        Some(line)
    }
    pub fn history_up(&mut self) {
        if self.history.is_empty() { return; }
        let i = match self.hist_pos { None => self.history.len() - 1, Some(0) => 0, Some(i) => i - 1 };
        self.hist_pos = Some(i); self.input = self.history[i].clone();
    }
    pub fn history_down(&mut self) {
        match self.hist_pos {
            None => {}
            Some(i) if i + 1 < self.history.len() => { self.hist_pos = Some(i + 1); self.input = self.history[i + 1].clone(); }
            Some(_) => { self.hist_pos = None; self.input.clear(); }
        }
    }
    /// The last `rows` lines, honouring the scroll offset
    pub fn view(&self, rows: usize) -> Vec<&str> {
        let n = self.lines.len();
        let end = n.saturating_sub(self.scroll.min(n.saturating_sub(rows)));
        let start = end.saturating_sub(rows);
        self.lines.range(start..end).map(|s| s.as_str()).collect()
    }
    pub fn scroll_by(&mut self, d: i32, rows: usize) {
        let max = self.lines.len().saturating_sub(rows);
        self.scroll = (self.scroll as i64 + d as i64).clamp(0, max as i64) as usize;
    }
    pub fn help(&mut self) {
        for l in [
            "/help              this list",
            "/ports             list serial ports",
            "/open PORT [BAUD]  open a port (e.g. /open /dev/rfcomm0 115200)",
            "/close             close the link",
            "/sim               switch to the built-in mount simulator",
            "/clear             clear the scrollback",
            "mount commands: PING  ID  ?  GO az el  AZ deg  EL deg  STOP  PARK  RATE az el  TEL hz  RAW AZ|EL us",
        ] { self.note(l); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn history_and_view() {
        let mut c = Console::default();
        c.input = "PING".into(); assert_eq!(c.submit(), Some("PING".into()));
        c.input = "  ".into(); assert_eq!(c.submit(), None);
        c.history_up(); assert_eq!(c.input, "PING");
        c.history_down(); assert_eq!(c.input, "");
        for i in 0..20 { c.rx(&format!("line {i}")); }
        let v = c.view(5);
        assert_eq!(v.len(), 5); assert_eq!(v[4], "< line 19");
        c.scroll_by(3, 5);
        assert_eq!(c.view(5)[4], "< line 16");
        c.scroll_by(1000, 5);
        assert_eq!(c.view(5)[0], "# PERIGEE serial console. Type a command for the mount and press Enter; /help for local commands.");
    }
}

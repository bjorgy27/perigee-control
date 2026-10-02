/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The serial link to the mount's Arduino, and a simulator that answers the same protocol when there is
/// no port. The port is opened and configured directly through termios (libc), read on a worker thread,
/// and everything crosses to the Bevy side as whole text lines.
///
/// Protocol (ASCII lines, both directions, see firmware/perigee_mount_stm32/src/main.rs):
///   PC -> mount   PING | ID | ? | GO az el | AZ deg | EL deg | STOP | PARK | RATE az_dps el_dps | TEL hz
///                 | RAW AZ us | RAW EL us | ZERO az el | CAL | OFF
///   mount -> PC   READY ... | POS ... | PONG | ID ... | OK ... | ERR ... | T4 tgt_az tgt_el cmd_az cmd_el set_az set_el NA -1 flags
/// Angles are mount-frame degrees (azimuth inside the 0..400 cable-wrap window, elevation -2..91);
/// the PC does the sky conversion. Telemetry (`mount::Pose`): `tgt` the clamped target, `cmd` the slew-ramped
/// pulse, `set` that pulse lagged by a servo model, flags K known / M moving / A,E clipped. All three are
/// estimates, not a feedback wire: these gearboxes have no feedback output (reports/stingray-power.md)
/// and no encoder is fitted, hence the fixed `NA -1`.
/////////////////////////////////////////////////////////////////////////////////////////////////////////
use bevy::prelude::*;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;

pub enum Incoming { Line(String), Closed(String) }

struct Worker { tx: Sender<String>, rx: std::sync::Mutex<Receiver<Incoming>>, stop: Arc<AtomicBool>, path: String }   // Mutex: Receiver is not Sync, resources must be

#[derive(Resource)]
pub struct SerialLink {
    worker: Option<Worker>,
    pub sim: Option<MountSim>,
    pub status: String,
    pub last_error: String,
    pub tx_count: usize,
    pub rx_count: usize,
    pub next_reconnect: f64,      // app seconds
    inbox: Vec<String>,
    pub sent: Vec<String>,        // lines sent this frame (for the console echo)
}

impl Default for SerialLink {
    fn default() -> Self {
        Self { worker: None, sim: None, status: "CLOSED".into(), last_error: String::new(), tx_count: 0, rx_count: 0, next_reconnect: 0.0, inbox: Vec::new(), sent: Vec::new() }
    }
}

impl SerialLink {
    pub fn is_open(&self) -> bool { self.worker.is_some() || self.sim.is_some() }
    pub fn port_name(&self) -> String {
        if let Some(w) = &self.worker { w.path.clone() } else if self.sim.is_some() { "SIMULATOR".into() } else { "-".into() }
    }

    /// Serial ports that look like an Arduino or a Bluetooth SPP channel, in preference order
    pub fn scan_ports() -> Vec<String> {
        let mut out = Vec::new();
        for pat in ["rfcomm", "ttyACM", "ttyUSB"] {
            let mut found: Vec<String> = std::fs::read_dir("/dev").map(|d| d.filter_map(|e| e.ok())
                .map(|e| e.file_name().to_string_lossy().to_string())
                .filter(|n| n.starts_with(pat)).map(|n| format!("/dev/{n}")).collect()).unwrap_or_default();
            found.sort();
            out.extend(found);
        }
        out
    }

    pub fn open(&mut self, path: &str, baud: u32) -> Result<(), String> {
        self.close();
        let fd = open_port(path, baud)?;
        let (tx_out, rx_out) = channel::<String>();
        let (tx_in, rx_in) = channel::<Incoming>();
        let stop = Arc::new(AtomicBool::new(false));
        let stop2 = stop.clone();
        std::thread::Builder::new().name("serial".into()).spawn(move || worker_loop(fd, rx_out, tx_in, stop2)).map_err(|e| e.to_string())?;
        self.worker = Some(Worker { tx: tx_out, rx: std::sync::Mutex::new(rx_in), stop, path: path.to_string() });
        self.status = format!("OPEN {path} {baud}");
        self.last_error.clear();
        Ok(())
    }

    pub fn open_sim(&mut self, geom: &crate::mount::MountGeom, az_rate: f64, el_rate: f64) {
        self.close();
        self.sim = Some(MountSim::new(geom, az_rate, el_rate));
        self.status = "SIMULATOR".into();
        self.inbox.push("READY PERIGEE-MOUNT sim".into());
    }

    pub fn close(&mut self) {
        if let Some(w) = self.worker.take() { w.stop.store(true, Ordering::Relaxed); }
        self.sim = None;
        self.status = "CLOSED".into();
    }

    pub fn send(&mut self, line: &str) {
        let line = line.trim();
        if line.is_empty() { return; }
        self.sent.push(line.to_string());
        self.tx_count += 1;
        if let Some(sim) = &mut self.sim {
            for r in sim.handle(line) { self.inbox.push(r); }
        } else if let Some(w) = &self.worker {
            if w.tx.send(format!("{line}\n")).is_err() { self.last_error = "worker gone".into(); }
        }
    }

    /// Lines received since the last call. Steps the simulator: its axes move by `dt * time_scale` clock
    /// seconds (the viewer's sped-up clock moves the simulated mount faster too), telemetry on real time.
    pub fn poll(&mut self, dt: f64, time_scale: f64) -> Vec<String> {
        let mut out = std::mem::take(&mut self.inbox);
        if let Some(sim) = &mut self.sim { out.extend(sim.step_scaled(dt * time_scale, dt)); }
        let mut closed = None;
        if let Some(w) = &self.worker {
            let rx = w.rx.lock().unwrap();
            while let Ok(m) = rx.try_recv() {
                match m { Incoming::Line(l) => out.push(l), Incoming::Closed(e) => { closed = Some(e); break; } }
            }
        }
        if let Some(e) = closed { self.worker = None; self.status = "CLOSED".into(); self.last_error = e; }
        self.rx_count += out.len();
        out
    }
}

//------------------------------------------------------------------------------------------ termios
fn open_port(path: &str, baud: u32) -> Result<i32, String> {
    use std::ffi::CString;
    let speed = match baud {
        9600 => libc::B9600, 19200 => libc::B19200, 38400 => libc::B38400, 57600 => libc::B57600,
        115200 => libc::B115200, 230400 => libc::B230400, 460800 => libc::B460800,
        _ => return Err(format!("unsupported baud {baud}")),
    };
    let c = CString::new(path).map_err(|e| e.to_string())?;
    let fd = unsafe { libc::open(c.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC) };
    if fd < 0 { return Err(format!("{path}: {}", std::io::Error::last_os_error())); }
    let mut tio: libc::termios = unsafe { std::mem::zeroed() };
    if unsafe { libc::tcgetattr(fd, &mut tio) } != 0 { let e = std::io::Error::last_os_error(); unsafe { libc::close(fd) }; return Err(format!("tcgetattr: {e}")); }
    unsafe { libc::cfmakeraw(&mut tio); }               // no line editing, no echo, no CR/LF translation, 8 bit
    tio.c_cflag |= libc::CLOCAL | libc::CREAD;         // ignore modem control lines, enable the receiver
    tio.c_cflag &= !(libc::CRTSCTS | libc::PARENB | libc::CSTOPB);   // no flow control, 8N1
    tio.c_cc[libc::VMIN] = 0; tio.c_cc[libc::VTIME] = 0;            // reads return what is there
    unsafe { libc::cfsetispeed(&mut tio, speed); libc::cfsetospeed(&mut tio, speed); }
    if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &tio) } != 0 { let e = std::io::Error::last_os_error(); unsafe { libc::close(fd) }; return Err(format!("tcsetattr: {e}")); }
    unsafe { libc::tcflush(fd, libc::TCIOFLUSH); }
    Ok(fd)
}

fn worker_loop(fd: i32, rx_out: Receiver<String>, tx_in: Sender<Incoming>, stop: Arc<AtomicBool>) {
    let mut buf = [0u8; 512];
    let mut line = Vec::<u8>::new();
    loop {
        if stop.load(Ordering::Relaxed) { break; }
        //Outgoing: everything queued
        while let Ok(s) = rx_out.try_recv() {
            let b = s.as_bytes();
            let mut off = 0;
            while off < b.len() {
                let n = unsafe { libc::write(fd, b[off..].as_ptr() as *const libc::c_void, b.len() - off) };
                if n < 0 {
                    let e = std::io::Error::last_os_error();
                    if e.kind() == std::io::ErrorKind::WouldBlock { std::thread::sleep(std::time::Duration::from_millis(2)); continue; }
                    let _ = tx_in.send(Incoming::Closed(format!("write: {e}")));
                    unsafe { libc::close(fd) }; return;
                }
                off += n as usize;
            }
        }
        //Incoming: wait up to 20 ms for bytes
        let mut pfd = libc::pollfd { fd, events: libc::POLLIN, revents: 0 };
        let r = unsafe { libc::poll(&mut pfd, 1, 20) };
        if r < 0 { let _ = tx_in.send(Incoming::Closed(format!("poll: {}", std::io::Error::last_os_error()))); unsafe { libc::close(fd) }; return; }
        if r == 0 { continue; }
        if pfd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 { let _ = tx_in.send(Incoming::Closed("port went away".into())); unsafe { libc::close(fd) }; return; }
        let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
        if n < 0 {
            let e = std::io::Error::last_os_error();
            if e.kind() == std::io::ErrorKind::WouldBlock { continue; }
            let _ = tx_in.send(Incoming::Closed(format!("read: {e}"))); unsafe { libc::close(fd) }; return;
        }
        if n == 0 { continue; }
        for &b in &buf[..n as usize] {
            match b {
                b'\n' => { let s = String::from_utf8_lossy(&line).trim_end_matches('\r').to_string(); line.clear(); if !s.is_empty() && tx_in.send(Incoming::Line(s)).is_err() { unsafe { libc::close(fd) }; return; } }
                b'\r' => {}
                _ => { if line.len() < 4096 { line.push(b); } }
            }
        }
    }
    unsafe { libc::close(fd) };
}

//------------------------------------------------------------------------------------------ simulator
/// A stand-in for the firmware: two rate-limited axes and the same command set, so the whole chain
/// (tracker -> link -> telemetry -> tiles) runs with nothing plugged in.
///
/// It enforces the same azimuth window as the firmware, tracks its own position, and records the
/// worst excursion it was ever asked for (`worst_violation`), so a host-side wrap bug shows up in
/// simulation instead of on the cable.
/// The firmware's settle model: the servos' rated speed (Stingray-4 90, Stingray-9 39.6 deg/s) times 0.7,
/// mirrored from `SERVO_RATED_DPS` / `SERVO_SETTLE_FRACTION` in limits.rs so the on-point timing matches
const SIM_SETTLE_DPS: [f64; 2] = [90.0 * 0.70, 39.6 * 0.70];

pub struct MountSim {
    pub az: f64, pub el: f64,           // where the axes are (the firmware's ramped command)
    pub az_t: f64, pub el_t: f64,       // where they are going
    pub set_az: f64, pub set_el: f64,   // the firmware's settle estimate: the command lagged at SIM_SETTLE_DPS
    pub powered: bool,                  // pulses on (holding); OFF makes the simulated servos limp
    pub az_rate: f64, pub el_rate: f64, // deg/s
    az_lo: f64, az_hi: f64, el_min: f64, el_max: f64,
    park: (f64, f64),
    clip: [bool; 2],                    // the standing target was clamped at that axis's limit, as the firmware's A/E flags
    tel_hz: f64, tel_acc: f64,
    /// Furthest any command ever asked an axis to go outside its window, deg. Stays 0 when the host
    /// behaves; anything above 0 is a planner bug the firmware would have had to catch.
    pub worst_violation: f64,
    /// How many commands were refused and clamped
    pub violations: usize,
}

impl MountSim {
    pub fn new(g: &crate::mount::MountGeom, az_rate: f64, el_rate: f64) -> Self {
        let park = ((g.az_lo + g.az_hi) / 2.0, 45.0);
        //Starts as if the operator had already declared the park pose (ZERO 200 45) and the servos hold it,
        //so the SIM button runs a procedure straight away; a real board after power-on starts unknown
        Self { az: park.0, el: park.1, az_t: park.0, el_t: park.1, set_az: park.0, set_el: park.1, powered: true, az_rate, el_rate,
               az_lo: g.az_lo, az_hi: g.az_hi, el_min: g.el_min, el_max: g.el_max, park,
               clip: [false; 2], tel_hz: 0.0, tel_acc: 0.0, worst_violation: 0.0, violations: 0 }
    }
    /// Is either axis still running? The on-point check and the telemetry both need it.
    pub fn moving(&self) -> bool {
        self.powered && ((self.az - self.az_t).abs() > 1e-3 || (self.el - self.el_t).abs() > 1e-3
                         || (self.set_az - self.az).abs() > 1e-3 || (self.set_el - self.el).abs() > 1e-3)
    }

    /// Clamp into the window, recording how far outside the request was. Same job as
    /// `limits::AxisCal::clamp_mount` in the firmware, and deliberately just as unforgiving: the
    /// simulator never takes a short cut the real mount would refuse.
    fn guard(&mut self, v: f64, lo: f64, hi: f64) -> f64 {
        let over = (lo - v).max(v - hi);
        if over > 1e-9 {
            self.violations += 1;
            if over > self.worst_violation { self.worst_violation = over; }
        }
        v.clamp(lo, hi)
    }
    fn guard_az(&mut self, v: f64) -> f64 { let (lo, hi) = (self.az_lo, self.az_hi); self.guard(v, lo, hi) }
    fn guard_el(&mut self, v: f64) -> f64 { let (lo, hi) = (self.el_min, self.el_max); self.guard(v, lo, hi) }
    /// Set both targets, as the firmware's `set_target`: the clip flags are standing and clear on the next request that fits
    fn set_target(&mut self, a: f64, e: f64) -> bool {
        self.clip = [!(self.az_lo..=self.az_hi).contains(&a), !(self.el_min..=self.el_max).contains(&e)];
        self.az_t = self.guard_az(a); self.el_t = self.guard_el(e);
        self.clip[0] || self.clip[1]
    }
    /// The firmware's `T4` line: target, ramped command, and the settle estimate lagging it as the firmware's
    /// does. K always: the simulator starts declared and never loses its position.
    pub fn telemetry(&self) -> String {
        let fl: String = [(true, 'K'), (self.moving(), 'M'), (self.clip[0], 'A'), (self.clip[1], 'E')].iter().filter(|f| f.0).map(|f| f.1).collect();
        format!("{} {:.2} {:.2} {:.2} {:.2} {:.2} {:.2} NA -1 {fl}", crate::mount::TEL_TAG, self.az_t, self.el_t, self.az, self.el, self.set_az, self.set_el)
    }
    pub fn handle(&mut self, line: &str) -> Vec<String> {
        let mut it = line.split_whitespace();
        //Ordinary numbers only, as the firmware: Rust's parser also accepts "nan" and "inf"
        let nums: Vec<f64> = line.split_whitespace().skip(1).filter_map(|s| s.parse::<f64>().ok()).filter(|v| v.is_finite()).collect();
        match it.next().map(|s| s.to_ascii_uppercase()).as_deref() {
            Some("PING") => vec!["PONG".into()],
            Some("ID") => vec![format!("ID PERIGEE-MOUNT sim PROTO {} AZ {:.0}-{:.0} EL {:.0}-{:.0}", crate::mount::TEL_TAG, self.az_lo, self.az_hi, self.el_min, self.el_max)],
            Some("?") => vec![self.telemetry()],
            Some("GO") if nums.len() >= 2 => {
                self.powered = true;
                let (a, e) = (nums[0], nums[1]);
                if self.set_target(a, e) { vec![format!("ERR GO out of window, clamped to {:.2} {:.2}", self.az_t, self.el_t)] }
                else { vec![format!("OK GO {:.2} {:.2}", a, e)] }
            }
            Some("AZ") if !nums.is_empty() => { self.powered = true; self.set_target(nums[0], self.el_t); vec![format!("OK AZ {:.2}", self.az_t)] }
            Some("EL") if !nums.is_empty() => { self.powered = true; self.set_target(self.az_t, nums[0]); vec![format!("OK EL {:.2}", self.el_t)] }
            Some("STOP") => { self.az_t = self.az; self.el_t = self.el; vec!["OK STOP".into()] }
            Some("PARK") => { self.powered = true; self.set_target(self.park.0, self.park.1); vec![format!("OK PARK {:.1} {:.1}", self.park.0, self.park.1)] }
            Some("RATE") if nums.len() >= 2 => { self.az_rate = nums[0].clamp(0.1, 90.0); self.el_rate = nums[1].clamp(0.1, 90.0); vec![format!("OK RATE {:.1} {:.1}", self.az_rate, self.el_rate)] }
            Some("TEL") if !nums.is_empty() => { self.tel_hz = nums[0].clamp(0.0, 20.0); vec![format!("OK TEL {:.1}", self.tel_hz)] }
            //As the firmware: with the pulses on, the pulse already is the position, so ZERO is only for limp servos
            Some("ZERO") if self.powered => vec!["ERR ZERO only while the servos are limp: send OFF first (with pulses on, the pulse is the position)".into()],
            Some("ZERO") if nums.len() >= 2 => {
                self.az = self.guard_az(nums[0]); self.el = self.guard_el(nums[1]);
                self.az_t = self.az; self.el_t = self.el; self.set_az = self.az; self.set_el = self.el; self.clip = [false; 2];
                vec![format!("OK ZERO {:.2} {:.2}", self.az, self.el)]
            }
            Some("CAL") => vec![format!("OK CAL AZ {:.2}..{:.2} deg  EL {:.2}..{:.2} deg  known 1", self.az_lo, self.az_hi, self.el_min, self.el_max)],
            Some("OFF") => { self.powered = false; self.az_t = self.az; self.el_t = self.el; self.set_az = self.az; self.set_el = self.el; vec!["OK OFF".into()] }
            //The simulator has no pulse map (that lives in the firmware's limits.rs), so it says so rather than pretend
            Some("RAW") => vec!["ERR RAW needs the real board: the simulator has no pulse map".into()],
            Some(cmd) => vec![format!("ERR unknown command {cmd}")],
            None => vec![],
        }
    }
    #[cfg(test)]
    pub fn step(&mut self, dt: f64) -> Vec<String> { self.step_scaled(dt, dt) }
    /// Axes move for `dt_motion` seconds at their rate limits; telemetry is timed on `dt_real`
    pub fn step_scaled(&mut self, dt_motion: f64, dt_real: f64) -> Vec<String> {
        let slew = |x: &mut f64, t: f64, rate: f64, lo: f64, hi: f64| {
            let d = t - *x; let m = rate * dt_motion.max(0.0);
            *x = (*x + d.clamp(-m, m)).clamp(lo, hi);
        };
        if self.powered {
            slew(&mut self.az, self.az_t, self.az_rate, self.az_lo, self.az_hi);
            slew(&mut self.el, self.el_t, self.el_rate, self.el_min, self.el_max);
            slew(&mut self.set_az, self.az, SIM_SETTLE_DPS[0], self.az_lo, self.az_hi);
            slew(&mut self.set_el, self.el, SIM_SETTLE_DPS[1], self.el_min, self.el_max);
        }
        let mut out = Vec::new();
        if self.tel_hz > 0.0 {
            self.tel_acc += dt_real;
            if self.tel_acc >= 1.0 / self.tel_hz { self.tel_acc = 0.0; out.push(self.telemetry()); }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sim() -> MountSim { MountSim::new(&crate::mount::MountGeom { az_travel: 450.0, az_center: 217.5, az_lo: 0.0, az_hi: 400.0, el_min: -2.0, el_max: 91.0, el_corr: 0.0 }, 20.0, 15.0) }

    #[test]
    fn sim_speaks_the_protocol() {
        let mut s = sim();
        assert_eq!(s.handle("PING"), vec!["PONG".to_string()]);
        assert_eq!(s.handle("GO 100 30"), vec!["OK GO 100.00 30.00".to_string()]);
        assert!(s.handle("GO 999 30")[0].starts_with("ERR"));
        assert!(s.handle("BOGUS")[0].starts_with("ERR unknown"));
        s.handle("TEL 5");
        let lines: Vec<String> = (0..10).flat_map(|_| s.step(0.1)).collect();
        assert!(lines.len() >= 4 && lines.iter().all(|l| l.starts_with("T4 ")), "{lines:?}");
    }

    #[test]
    fn sim_behaves_like_the_firmware_where_it_matters() {
        //not-a-number is refused and changes nothing (H3)
        let mut s = sim();
        assert!(s.handle("GO nan 30")[0].starts_with("ERR"));
        assert!(s.handle("RATE nan nan")[0].starts_with("ERR"));
        assert!((s.az_t - 200.0).abs() < 1e-9 && (s.az_rate - 20.0).abs() < 1e-9);
        //RAW is not pretended
        assert!(s.handle("RAW AZ 600")[0].starts_with("ERR"));
        //the settle estimate lags the ramped command as the firmware's does, so on point takes real time
        s.handle("GO 200 85");
        s.step(1.0);
        assert!((s.el - 60.0).abs() < 1e-9, "{}", s.el);
        assert!(s.set_el < s.el + 1e-9 && s.moving());
        let p = crate::mount::Pose::parse(&s.telemetry()).unwrap();
        assert!(!p.on_point((200.0, 85.0), 0.5));
        for _ in 0..10 { s.step(1.0); }
        assert!(crate::mount::Pose::parse(&s.telemetry()).unwrap().on_point((200.0, 85.0), 0.5));
    }

    #[test]
    fn sim_slews_at_the_rate_limit() {
        let mut s = sim();
        s.handle("GO 200 85");       // from park (200, 45): only elevation moves, 40 deg at 15 deg/s
        s.step(1.0);
        assert!((s.el - 60.0).abs() < 1e-9 && (s.az - 200.0).abs() < 1e-9);
        for _ in 0..10 { s.step(1.0); }
        assert!((s.el - 85.0).abs() < 1e-9);
        assert!(!s.moving());
        s.handle("STOP");
        assert_eq!(s.el_t, s.el);
    }

    #[test]
    fn sim_enforces_the_cable_wrap_window_and_records_violations() {
        //The simulator is the host's own backstop: it refuses what the firmware would refuse, so a
        //planner bug shows up here instead of on the cable.
        let mut s = sim();
        assert_eq!(s.violations, 0);
        //Beck's case, sent wrong on purpose: 401 is outside the window
        assert!(s.handle("GO 401 30")[0].starts_with("ERR"));
        assert!((s.az_t - 400.0).abs() < 1e-9, "clamped to the limit, not wrapped to 41: {}", s.az_t);
        assert_eq!(s.violations, 1);
        assert!((s.worst_violation - 1.0).abs() < 1e-9, "{}", s.worst_violation);
        //a bigger excursion is recorded as the worst one
        s.handle("GO -50 30");
        assert_eq!(s.violations, 2);
        assert!((s.worst_violation - 50.0).abs() < 1e-9, "{}", s.worst_violation);
        //exactly the boundaries are fine and leave the count alone
        assert!(s.handle("GO 400 30")[0].starts_with("OK"));
        assert!(s.handle("GO 0 30")[0].starts_with("OK"));
        assert!(s.handle("EL -2")[0].starts_with("OK"));
        assert_eq!(s.violations, 2);
    }

    #[test]
    fn sim_axes_never_leave_the_window_while_slewing() {
        //Even driven at the limits for a long time, the simulated position stays legal: nothing
        //downstream (the tiles, the on-point check) ever sees an impossible pose.
        let mut s = sim();
        for (cmd, _) in [("GO 400 91", 0), ("GO 0 -2", 0), ("GO 401 999", 0), ("GO -99 -99", 0)] {
            s.handle(cmd);
            for _ in 0..400 {
                s.step(0.1);
                assert!(s.az >= 0.0 - 1e-9 && s.az <= 400.0 + 1e-9, "az {}", s.az);
                assert!(s.el >= -2.0 - 1e-9 && s.el <= 91.0 + 1e-9, "el {}", s.el);
            }
        }
    }

    #[test]
    fn sim_tracks_its_own_position_and_zero_anchors_it() {
        let mut s = sim();
        //ZERO is refused while the simulated servos hold (as the firmware); OFF makes them limp
        assert!(s.handle("ZERO 399 10")[0].starts_with("ERR"));
        assert_eq!(s.handle("OFF"), vec!["OK OFF".to_string()]);
        s.handle("ZERO 399 10");
        assert!((s.az - 399.0).abs() < 1e-9 && (s.el - 10.0).abs() < 1e-9);
        assert!(!s.moving(), "ZERO declares a position, it does not start a move");
        //and from there the host's wrap rule is what sends it the long way: the sim just obeys
        s.handle("GO 41 10");
        for _ in 0..400 { s.step(0.1); }
        assert!((s.az - 41.0).abs() < 1e-6, "{}", s.az);
    }

    #[test]
    fn sim_motion_follows_the_scaled_clock() {
        let mut s = sim();
        s.handle("GO 200 85"); s.handle("TEL 5");
        //One real second at 20x clock: 20 clock seconds of motion (el reaches its target), telemetry still ~5 lines
        let lines: Vec<String> = (0..10).flat_map(|_| s.step_scaled(2.0, 0.1)).collect();
        assert!((s.el - 85.0).abs() < 1e-9);
        assert!(lines.len() >= 4 && lines.len() <= 6, "{}", lines.len());
        //A paused clock: no motion at all
        s.handle("GO 200 0"); s.step_scaled(0.0, 1.0);
        assert!((s.el - 85.0).abs() < 1e-9);
    }
}

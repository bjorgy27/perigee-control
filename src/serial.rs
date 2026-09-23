/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// The serial link to the mount's Arduino, and a simulator that answers the same protocol when there is
/// no port. The port is opened and configured directly through termios (libc), read on a worker thread,
/// and everything crosses to the Bevy side as whole text lines.
///
/// Protocol (ASCII lines, both directions, see firmware/perigee_mount/perigee_mount.ino):
///   PC -> mount   PING | ID | ? | GO az el | AZ deg | EL deg | STOP | PARK | RATE az_dps el_dps | TEL hz | RAW AZ us | RAW EL us
///   mount -> PC   READY ... | PONG | ID ... | OK ... | ERR ... | T az_cmd el_cmd az_fb el_fb moving enc   (telemetry)
/// Angles are mount-frame degrees (0..450 azimuth, -5..185 elevation); the PC does the sky conversion.
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
pub struct MountSim {
    pub az: f64, pub el: f64,           // where the axes are
    pub az_t: f64, pub el_t: f64,       // where they are going
    pub az_rate: f64, pub el_rate: f64, // deg/s
    az_travel: f64, el_min: f64, el_max: f64,
    park: (f64, f64),
    tel_hz: f64, tel_acc: f64,
}

impl MountSim {
    pub fn new(g: &crate::mount::MountGeom, az_rate: f64, el_rate: f64) -> Self {
        let park = (g.az_travel / 2.0, 45.0);
        Self { az: park.0, el: park.1, az_t: park.0, el_t: park.1, az_rate, el_rate, az_travel: g.az_travel, el_min: g.el_min, el_max: g.el_max, park, tel_hz: 0.0, tel_acc: 0.0 }
    }
    fn moving(&self) -> bool { (self.az - self.az_t).abs() > 1e-3 || (self.el - self.el_t).abs() > 1e-3 }
    pub fn telemetry(&self) -> String {
        format!("T {:.2} {:.2} {:.2} {:.2} {} {}", self.az_t, self.el_t, self.az, self.el, self.moving() as u8, ((self.el - self.el_min) / (self.el_max - self.el_min) * 4095.0).round() as i64)
    }
    pub fn handle(&mut self, line: &str) -> Vec<String> {
        let mut it = line.split_whitespace();
        let nums: Vec<f64> = line.split_whitespace().skip(1).filter_map(|s| s.parse().ok()).collect();
        match it.next().map(|s| s.to_ascii_uppercase()).as_deref() {
            Some("PING") => vec!["PONG".into()],
            Some("ID") => vec![format!("ID PERIGEE-MOUNT sim AZ 0-{:.0} EL {:.0}-{:.0}", self.az_travel, self.el_min, self.el_max)],
            Some("?") => vec![self.telemetry()],
            Some("GO") if nums.len() >= 2 => {
                let (a, e) = (nums[0], nums[1]);
                if !(0.0..=self.az_travel).contains(&a) || !(self.el_min..=self.el_max).contains(&e) {
                    self.az_t = a.clamp(0.0, self.az_travel); self.el_t = e.clamp(self.el_min, self.el_max);
                    vec![format!("ERR GO out of range, clamped to {:.2} {:.2}", self.az_t, self.el_t)]
                } else { self.az_t = a; self.el_t = e; vec![format!("OK GO {:.2} {:.2}", a, e)] }
            }
            Some("AZ") if !nums.is_empty() => { self.az_t = nums[0].clamp(0.0, self.az_travel); vec![format!("OK AZ {:.2}", self.az_t)] }
            Some("EL") if !nums.is_empty() => { self.el_t = nums[0].clamp(self.el_min, self.el_max); vec![format!("OK EL {:.2}", self.el_t)] }
            Some("STOP") => { self.az_t = self.az; self.el_t = self.el; vec!["OK STOP".into()] }
            Some("PARK") => { self.az_t = self.park.0; self.el_t = self.park.1; vec![format!("OK PARK {:.1} {:.1}", self.park.0, self.park.1)] }
            Some("RATE") if nums.len() >= 2 => { self.az_rate = nums[0].clamp(0.1, 90.0); self.el_rate = nums[1].clamp(0.1, 90.0); vec![format!("OK RATE {:.1} {:.1}", self.az_rate, self.el_rate)] }
            Some("TEL") if !nums.is_empty() => { self.tel_hz = nums[0].clamp(0.0, 20.0); vec![format!("OK TEL {:.1}", self.tel_hz)] }
            Some("RAW") => vec!["OK RAW (ignored by the simulator)".into()],
            Some(cmd) => vec![format!("ERR unknown command {cmd}")],
            None => vec![],
        }
    }
    #[cfg(test)]
    pub fn step(&mut self, dt: f64) -> Vec<String> { self.step_scaled(dt, dt) }
    /// Axes move for `dt_motion` seconds at their rate limits; telemetry is timed on `dt_real`
    pub fn step_scaled(&mut self, dt_motion: f64, dt_real: f64) -> Vec<String> {
        let slew = |x: &mut f64, t: f64, rate: f64| { let d = t - *x; let m = rate * dt_motion.max(0.0); *x += d.clamp(-m, m); };
        slew(&mut self.az, self.az_t, self.az_rate);
        slew(&mut self.el, self.el_t, self.el_rate);
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
    fn sim() -> MountSim { MountSim::new(&crate::mount::MountGeom { az_travel: 450.0, az_center: 217.5, el_min: -5.0, el_max: 185.0 }, 20.0, 15.0) }

    #[test]
    fn sim_speaks_the_protocol() {
        let mut s = sim();
        assert_eq!(s.handle("PING"), vec!["PONG".to_string()]);
        assert_eq!(s.handle("GO 100 30"), vec!["OK GO 100.00 30.00".to_string()]);
        assert!(s.handle("GO 999 30")[0].starts_with("ERR"));
        assert!(s.handle("BOGUS")[0].starts_with("ERR unknown"));
        s.handle("TEL 5");
        let lines: Vec<String> = (0..10).flat_map(|_| s.step(0.1)).collect();
        assert!(lines.len() >= 4 && lines.iter().all(|l| l.starts_with("T ")), "{lines:?}");
    }

    #[test]
    fn sim_slews_at_the_rate_limit() {
        let mut s = sim();
        s.handle("GO 225 100");      // from park (225, 45): only elevation moves, 55 deg at 15 deg/s
        s.step(1.0);
        assert!((s.el - 60.0).abs() < 1e-9 && (s.az - 225.0).abs() < 1e-9);
        for _ in 0..10 { s.step(1.0); }
        assert!((s.el - 100.0).abs() < 1e-9);
        assert!(!s.moving());
        s.handle("STOP");
        assert_eq!(s.el_t, s.el);
    }

    #[test]
    fn sim_motion_follows_the_scaled_clock() {
        let mut s = sim();
        s.handle("GO 225 100"); s.handle("TEL 5");
        //One real second at 20x clock: 20 clock seconds of motion (el reaches its target), telemetry still ~5 lines
        let lines: Vec<String> = (0..10).flat_map(|_| s.step_scaled(2.0, 0.1)).collect();
        assert!((s.el - 100.0).abs() < 1e-9);
        assert!(lines.len() >= 4 && lines.len() <= 6, "{}", lines.len());
        //A paused clock: no motion at all
        s.handle("GO 225 0"); s.step_scaled(0.0, 1.0);
        assert!((s.el - 100.0).abs() < 1e-9);
    }
}

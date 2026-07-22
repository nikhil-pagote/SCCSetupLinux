// Linux replacement for the Minisforum/AtomMan SCC front-panel LCD driver
// (reverse-engineered from SCCSetup.exe's SCCS.exe).
//
// The panel is a Synwit-based USB-CDC virtual COM device (VID 0416, PID
// 50a1) that shows up as /dev/ttyACM0 (or a stable /dev/sccs-lcd symlink,
// see the accompanying udev rule). It continuously polls the host for each
// data type and sends touch-control commands; we push status packets on our
// own schedule and act on the commands.
//
// Host->device framing (length is little-endian):
//   0xAA | len_lo | len_hi | cmd | payload bytes... | 0xCC 0x33 0xC3 0x3C
//   len = payload.len() + 5   (cmd byte + payload + 4-byte trailer)
// Device->host framing uses a SINGLE length byte -- see parse_device_frames.
//
// Command bytes (one per data type) and payload text format, matching the
// original app's sprintf format strings exactly:
//   0x32  CPU:    "{CPU:%s;Tempr:%d;Useage:%d;Freq:%d;Tempr1:%d;}"
//   0x33  GPU:    "{GPU:%s;Tempr:%d;Useage:%d}"
//   0x34  Memory: "{Memory:%s;Used:%.1f;Available:%.1f;Total:%.1f;Useage:%d}"
//   0x35  Disk:   "{DiskName:%s;Tempr:%d;UsageSpace:%d;AllSpace:%d;Usage:%d}"
//   0x36  Date:   "{Date:YYYY/MM/DD;Time:HH:MM:SS;Week:%d;Weather:%d;
//                   TemprLo:%d,TemprHi:%d,Zone:%s,Desc:%s}"
//                 (Week is 0=Sunday..6=Saturday to match Win32 SYSTEMTIME; the
//                  weather block is optional -- empty fields when unavailable.
//                  The vendor app sent only Date/Time/Week with a 2-digit year
//                  and no braces; the panel accepts both.)
//   0x37  Speed:  "{SPEED:%d;NETWORK:%s,%s}"
//                 (SPEED is fan RPM; the two strings are network transfer
//                  rates formatted as "%.1fK/s" / "%.1fM/s" / "%.1fG/s")
//   0x39  Volume: "{VOLUME:%d}"
//
// Note Freq is in kHz: the panel divides by 1e6 and appends "GHz".
//
// The panel's on-screen "name" field for each data type latches onto whatever
// value it first receives after power-up and ignores later changes, while
// numeric fields update live every cycle. So the names sent here should be
// correct from the very first packet after boot; changing them needs a power
// cycle, not a service restart.
//
// Requires root for: the vendor embedded controller via /dev/port (fan RPM
// and CPU/GPU temperatures, see the Ec struct) and the i915 perf PMU (GPU
// utilisation, see GpuBusy). Everything degrades to 0 without it.

use std::ffi::CString;
use std::fs;
use std::os::unix::fs::FileExt;
use std::os::unix::io::RawFd;
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

const PORT_CANDIDATES: [&str; 2] = ["/dev/sccs-lcd", "/dev/ttyACM0"];
const UPDATE_INTERVAL: Duration = Duration::from_secs(1);

// hwmon "name" values that expose a CPU package temperature, in preference
// order: Intel, then AMD (k10temp/zenpower), then generic ARM/ACPI sources.
const CPU_TEMP_HWMON: [&str; 5] = ["coretemp", "k10temp", "zenpower", "cpu_thermal", "acpitz"];

const CMD_CPU: u8 = 0x32;
const CMD_GPU: u8 = 0x33;
const CMD_MEMORY: u8 = 0x34;
const CMD_DISK: u8 = 0x35;
const CMD_DATE: u8 = 0x36;
const CMD_SPEED: u8 = 0x37;
const CMD_VOLUME: u8 = 0x39;

// Frames coming back from the panel use a SINGLE-byte length (host->device
// uses two), i.e. AA <len> <cmd> [value...] CC 33 C3 3C where len counts
// everything after itself. A one-byte body is the panel polling the host to
// send that data type; a longer body is a control command from its touch UI.
const FRAME_TRAILER: [u8; 4] = [0xCC, 0x33, 0xC3, 0x3C];
const DEV_CMD_VOLUME: u8 = 0x61; // 'a', payload is volume 0-100
// 'b', the panel's "Mode Adjustment" button. Payload 1/2/3 =
// Energy-saving/Balanced/Performance; the vendor app applies it by writing the
// EC (see Ec::set_mode). Values confirmed by capture on hardware.
const DEV_CMD_MODE: u8 = 0x62;
const MODE_MIN: u8 = 1;
const MODE_MAX: u8 = 3;

const DISK_MOUNT: &str = "/";

// Vendor embedded controller, reached by direct port I/O exactly as the
// Windows app does through its WinRing0 kernel driver. Note this is NOT the
// standard ACPI EC (which sits on 0x62/0x66 and is owned by the kernel) --
// it is a separate vendor interface, which is why the fan registers read as
// zero through Linux's generic ec_sys interface.
const EC_PORT_CMD: u64 = 0x6C; // status/command port
const EC_PORT_DATA: u64 = 0x68;
const EC_CMD_FAN: u8 = 0xD5; // read command used for the fan registers
const EC_CMD_TEMP: u8 = 0xDD; // read command used for the temperature registers
const EC_CMD_MODE: u8 = 0xDE; // WRITE command that applies the performance mode
const EC_REG_FAN_HI: u8 = 0x19;
const EC_REG_FAN_LO: u8 = 0x18;
const EC_REG_CPU_TEMP: u8 = 0x20;
const EC_REG_GPU_TEMP: u8 = 0x22;
// The EC occasionally returns a torn byte mid-transaction, so the original
// app rejects out-of-range readings and falls back to another source. These
// are its exact thresholds.
const FAN_RPM_SANITY_MAX: i64 = 4000;
const EC_CPU_TEMP_SANITY_MAX: i64 = 99;
const EC_GPU_TEMP_SANITY_MAX: i64 = 100;

/// SCC_LCD_PORT overrides the device path, so the daemon can be pointed at a
/// pty for development without the panel attached.
fn find_port() -> Option<String> {
    if let Ok(p) = std::env::var("SCC_LCD_PORT") {
        return Path::new(&p).exists().then_some(p);
    }
    PORT_CANDIDATES
        .iter()
        .find(|p| Path::new(p).exists())
        .map(|p| p.to_string())
}

fn open_port(path: &str) -> std::io::Result<RawFd> {
    let cpath = CString::new(path).unwrap();
    let fd = unsafe { libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_NOCTTY) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }

    unsafe {
        let mut tio: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut tio) != 0 {
            libc::close(fd);
            return Err(std::io::Error::last_os_error());
        }
        tio.c_iflag = 0;
        tio.c_oflag = 0;
        tio.c_lflag = 0;
        tio.c_cflag &= !libc::PARENB;
        tio.c_cflag &= !libc::CSTOPB;
        tio.c_cflag &= !libc::CSIZE;
        tio.c_cflag |= libc::CS8 | libc::CREAD | libc::CLOCAL;
        // return from read() immediately with whatever is buffered
        tio.c_cc[libc::VMIN] = 0;
        tio.c_cc[libc::VTIME] = 0;
        libc::cfsetispeed(&mut tio, libc::B115200);
        libc::cfsetospeed(&mut tio, libc::B115200);
        if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
            libc::close(fd);
            return Err(std::io::Error::last_os_error());
        }

        // raise DTR + RTS
        let mut status: libc::c_int = 0;
        libc::ioctl(fd, libc::TIOCMGET, &mut status);
        status |= libc::TIOCM_DTR | libc::TIOCM_RTS;
        libc::ioctl(fd, libc::TIOCMSET, &status);
    }

    Ok(fd)
}

fn write_all(fd: RawFd, buf: &[u8]) -> std::io::Result<()> {
    let mut off = 0;
    while off < buf.len() {
        let n = unsafe { libc::write(fd, buf[off..].as_ptr() as *const libc::c_void, buf.len() - off) };
        if n < 0 {
            return Err(std::io::Error::last_os_error());
        }
        off += n as usize;
    }
    Ok(())
}

fn read_available(fd: RawFd) -> Vec<u8> {
    let mut buf = [0u8; 512];
    let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, buf.len()) };
    if n > 0 {
        buf[..n as usize].to_vec()
    } else {
        Vec::new()
    }
}

/// Pull complete device->host frames out of `buf`, returning (cmd, values)
/// for the ones carrying a payload. Routine one-byte polls are dropped --
/// the panel is just asking for data we already push on our own schedule.
fn parse_device_frames(buf: &mut Vec<u8>) -> Vec<(u8, Vec<u8>)> {
    let mut out = Vec::new();
    loop {
        match buf.iter().position(|&b| b == 0xAA) {
            None => {
                buf.clear();
                return out;
            }
            Some(0) => {}
            Some(start) => {
                buf.drain(..start);
            }
        }
        if buf.len() < 2 {
            return out;
        }
        let total = buf[1] as usize + 2;
        if !(6..=64).contains(&total) {
            buf.drain(..1); // not a plausible frame, resync
            continue;
        }
        if buf.len() < total {
            return out;
        }
        let f: Vec<u8> = buf.drain(..total).collect();
        if f[total - 4..] != FRAME_TRAILER {
            continue;
        }
        let body = &f[2..total - 4];
        if body.len() >= 2 {
            out.push((body[0], body[1..].to_vec()));
        }
    }
}

/// UID of the logged-in desktop session, found by looking for a PipeWire
/// socket under /run/user. Avoids hardcoding a user so the package stays
/// portable; the daemon itself runs as root for the EC.
fn session_uid() -> Option<u32> {
    for entry in fs::read_dir("/run/user").ok()?.flatten() {
        if entry.path().join("pipewire-0").exists() {
            if let Some(uid) = entry.file_name().to_str().and_then(|s| s.parse().ok()) {
                return Some(uid);
            }
        }
    }
    None
}

fn wpctl(uid: u32, args: &[&str]) -> Option<std::process::Output> {
    use std::os::unix::process::CommandExt;
    std::process::Command::new("wpctl")
        .args(args)
        .env("XDG_RUNTIME_DIR", format!("/run/user/{uid}"))
        .uid(uid)
        .output()
        .ok()
}

fn set_system_volume(percent: u8) {
    if let Some(uid) = session_uid() {
        wpctl(uid, &["set-volume", "@DEFAULT_AUDIO_SINK@", &format!("{}%", percent.min(100))]);
    }
}

fn system_volume_percent() -> i64 {
    let Some(uid) = session_uid() else { return 0 };
    let Some(out) = wpctl(uid, &["get-volume", "@DEFAULT_AUDIO_SINK@"]) else {
        return 0;
    };
    let text = String::from_utf8_lossy(&out.stdout);
    if text.contains("[MUTED]") {
        return 0;
    }
    text.split_whitespace()
        .nth(1)
        .and_then(|s| s.parse::<f64>().ok())
        .map(|v| (v * 100.0).round() as i64)
        .unwrap_or(0)
}

fn frame(cmd: u8, payload: &str) -> Vec<u8> {
    let p = payload.as_bytes();
    let length = (p.len() + 5) as u16;
    let mut out = Vec::with_capacity(p.len() + 8);
    out.push(0xAA);
    out.extend_from_slice(&length.to_le_bytes());
    out.push(cmd);
    out.extend_from_slice(p);
    out.extend_from_slice(&[0xCC, 0x33, 0xC3, 0x3C]);
    out
}

fn find_hwmon_by_name(name: &str) -> Option<PathBuf> {
    let entries = fs::read_dir("/sys/class/hwmon").ok()?;
    for entry in entries.flatten() {
        let path = entry.path();
        if let Ok(contents) = fs::read_to_string(path.join("name")) {
            if contents.trim() == name {
                return Some(path);
            }
        }
    }
    None
}

fn read_temp_c(hwmon_path: &Path, input_file: &str) -> i64 {
    fs::read_to_string(hwmon_path.join(input_file))
        .ok()
        .and_then(|s| s.trim().parse::<i64>().ok())
        .map(|milli| milli / 1000)
        .unwrap_or(0)
}

/// The panel divides Freq by 1e6 and appends "GHz", so it wants kHz --
/// which is exactly the unit scaling_cur_freq already reports. Do not
/// convert to MHz here; that renders as 0.0GHz.
fn cpu_freq_khz() -> i64 {
    fs::read_to_string("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        .ok()
        .and_then(|s| s.trim().parse::<i64>().ok())
        .unwrap_or(0)
}

/// Block device backing a mount point, e.g. "/" -> "nvme1n1".
///
/// Resolved through /proc/mounts rather than stat(): btrfs (and LVM, ZFS,
/// overlayfs) report an anonymous st_dev with major 0 that maps to no real
/// disk. Walks from the partition up to its whole disk, since the temperature
/// sensor lives on the disk rather than the partition.
fn root_blockdev(mount: &str) -> Option<String> {
    let mounts = fs::read_to_string("/proc/mounts").ok()?;
    let source = mounts
        .lines()
        .filter_map(|l| {
            let mut it = l.split_whitespace();
            let src = it.next()?;
            let dst = it.next()?;
            (dst == mount && src.starts_with("/dev/")).then_some(src)
        })
        .next_back()?; // later entries shadow earlier ones

    let mut name = Path::new(source).file_name()?.to_str()?.to_string();

    // device-mapper (LUKS/LVM): step down to the first underlying device
    if name.starts_with("dm-") || source.starts_with("/dev/mapper/") {
        let real = fs::canonicalize(source).ok()?;
        let dm = real.file_name()?.to_str()?.to_string();
        if let Ok(slaves) = fs::read_dir(format!("/sys/class/block/{dm}/slaves")) {
            if let Some(first) = slaves.flatten().next() {
                name = first.file_name().to_str()?.to_string();
            }
        }
    }

    // a partition has a "partition" attribute; its parent dir is the disk
    if Path::new(&format!("/sys/class/block/{name}/partition")).exists() {
        let link = fs::canonicalize(format!("/sys/class/block/{name}")).ok()?;
        return Some(link.parent()?.file_name()?.to_str()?.to_string());
    }
    Some(name)
}

/// Model string reported by the drive, e.g. "Corsair MP600 MINI".
fn disk_model(blockdev: &str) -> String {
    fs::read_to_string(format!("/sys/block/{blockdev}/device/model"))
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| blockdev.to_string())
}

/// Human name from one `lspci -mm` line. Fields are quoted:
/// slot "class" "vendor" "device" ...  Prefers the bracketed marketing name,
/// e.g. "Meteor Lake-P [Intel Arc Graphics]" -> "Intel Arc Graphics".
fn parse_lspci_device(line: &str) -> Option<String> {
    let parts: Vec<&str> = line.split('"').collect();
    if parts.len() < 6 {
        return None;
    }
    let vendor = parts[3];
    let device = parts[5];
    let short_vendor = vendor.split_whitespace().next().unwrap_or(vendor);
    if let (Some(o), Some(c)) = (device.find('['), device.find(']')) {
        if o < c {
            return Some(device[o + 1..c].to_string());
        }
    }
    Some(format!("{short_vendor} {device}"))
}

/// Name of the PCI device at `slot` (domain:bus:slot.func), via lspci.
fn lspci_name(slot: &str) -> Option<String> {
    let out = std::process::Command::new("lspci").args(["-mm", "-s", slot]).output().ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    parse_lspci_device(text.lines().next()?)
}

/// GPU name from the DRM device's PCI ID, via the shared pci.ids database
/// that lspci uses. Falls back to a generic label.
fn gpu_name() -> String {
    let out = std::process::Command::new("lspci")
        .args(["-mm", "-d", "::0300"]) // display controllers
        .output()
        .ok();
    out.as_ref()
        .and_then(|o| String::from_utf8_lossy(&o.stdout).lines().next().and_then(parse_lspci_device))
        .unwrap_or_else(|| "GPU".to_string())
}

/// Is a working NVIDIA GPU present? `nvidia-smi -L` lists "GPU 0: ..." lines
/// when a card and driver are up; anything else (no driver, no card, error)
/// means no NVIDIA path to use.
fn nvidia_present() -> bool {
    std::process::Command::new("nvidia-smi")
        .arg("-L")
        .output()
        .map(|o| o.status.success() && String::from_utf8_lossy(&o.stdout).contains("GPU "))
        .unwrap_or(false)
}

/// First line of `nvidia-smi --query-gpu=<fields>` in CSV (no header/units).
fn nvidia_query(fields: &str) -> Option<String> {
    let out = std::process::Command::new("nvidia-smi")
        .arg(format!("--query-gpu={fields}"))
        .arg("--format=csv,noheader,nounits")
        .output()
        .ok()?;
    out.status
        .success()
        .then(|| String::from_utf8_lossy(&out.stdout).lines().next().unwrap_or("").trim().to_string())
}

fn nvidia_name() -> Option<String> {
    nvidia_query("name").filter(|s| !s.is_empty())
}

fn parse_nvidia_temp_usage(line: &str) -> (i64, i64) {
    let mut it = line.split(',').map(|s| s.trim().parse::<i64>().unwrap_or(0));
    (it.next().unwrap_or(0), it.next().unwrap_or(0))
}

fn nvidia_temp_usage() -> (i64, i64) {
    nvidia_query("temperature.gpu,utilization.gpu")
        .map(|l| parse_nvidia_temp_usage(&l))
        .unwrap_or((0, 0))
}

/// A discrete AMD GPU exposed through the `amdgpu` driver. Temperature comes
/// from its hwmon edge sensor and utilisation from the card's
/// `gpu_busy_percent`, both standard sysfs -- no EC or PMU involved.
struct AmdGpu {
    hwmon: PathBuf,
    device: PathBuf, // canonical PCI device dir (holds gpu_busy_percent)
}

impl AmdGpu {
    fn detect() -> Option<AmdGpu> {
        let hwmon = find_hwmon_by_name("amdgpu")?;
        let device = fs::canonicalize(hwmon.join("device")).ok()?;
        Some(AmdGpu { hwmon, device })
    }

    fn temp(&self) -> i64 {
        read_temp_c(&self.hwmon, "temp1_input")
    }

    fn usage(&self) -> i64 {
        fs::read_to_string(self.device.join("gpu_busy_percent"))
            .ok()
            .and_then(|s| s.trim().parse().ok())
            .unwrap_or(0)
    }

    fn name(&self) -> String {
        self.device
            .file_name()
            .and_then(|s| s.to_str())
            .and_then(lspci_name)
            .unwrap_or_else(|| "GPU".to_string())
    }
}

/// Which GPU the daemon reports. Discrete cards (an Oculink eGPU) win over the
/// Intel iGPU when present, so the panel shows the card actually doing work.
/// Detected once at startup; a hotplugged eGPU needs a daemon restart (and the
/// panel latches the name at power-up regardless).
enum GpuBackend {
    Intel(Option<GpuBusy>), // i915 PMU for usage; temperature from the vendor EC
    Nvidia,
    Amd(AmdGpu),
}

impl GpuBackend {
    fn detect() -> GpuBackend {
        if nvidia_present() {
            return GpuBackend::Nvidia;
        }
        if let Some(a) = AmdGpu::detect() {
            return GpuBackend::Amd(a);
        }
        GpuBackend::Intel(GpuBusy::open())
    }

    fn name(&self) -> String {
        match self {
            GpuBackend::Nvidia => nvidia_name().unwrap_or_else(|| "GPU".to_string()),
            GpuBackend::Amd(a) => a.name(),
            GpuBackend::Intel(_) => gpu_name(),
        }
    }

    fn label(&self) -> &'static str {
        match self {
            GpuBackend::Nvidia => "nvidia (nvidia-smi)",
            GpuBackend::Amd(_) => "amd (amdgpu sysfs)",
            GpuBackend::Intel(Some(_)) => "intel (i915 PMU + EC)",
            GpuBackend::Intel(None) => "intel (EC temp only; PMU unavailable)",
        }
    }
}

fn cpu_name() -> String {
    fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|s| {
            s.lines()
                .find(|l| l.starts_with("model name"))
                .and_then(|l| l.split(':').nth(1))
                .map(|s| s.trim().to_string())
        })
        .unwrap_or_else(|| "CPU".to_string())
}

struct CpuTimes {
    idle: u64,
    total: u64,
}

fn read_cpu_times() -> CpuTimes {
    parse_cpu_times(&fs::read_to_string("/proc/stat").unwrap_or_default())
}

fn parse_cpu_times(stat: &str) -> CpuTimes {
    let first_line = stat.lines().next().unwrap_or("");
    let fields: Vec<u64> = first_line
        .split_whitespace()
        .skip(1)
        .filter_map(|f| f.parse::<u64>().ok())
        .collect();
    if fields.len() < 4 {
        return CpuTimes { idle: 0, total: 0 };
    }
    let idle = fields[3] + fields.get(4).copied().unwrap_or(0); // idle + iowait
    // Only user..steal. The trailing guest/guest_nice fields are already
    // counted inside user/nice, so summing everything would double-count them.
    let total: u64 = fields.iter().take(8).sum();
    CpuTimes { idle, total }
}

fn cpu_percent(prev: &CpuTimes, cur: &CpuTimes) -> i64 {
    let total_delta = cur.total.saturating_sub(prev.total);
    let idle_delta = cur.idle.saturating_sub(prev.idle);
    if total_delta == 0 {
        return 0;
    }
    let busy = total_delta.saturating_sub(idle_delta);
    ((busy as f64 / total_delta as f64) * 100.0).round() as i64
}

struct MemInfo {
    total_kb: u64,
    available_kb: u64,
}

fn read_meminfo() -> MemInfo {
    parse_meminfo(&fs::read_to_string("/proc/meminfo").unwrap_or_default())
}

fn parse_meminfo(contents: &str) -> MemInfo {
    let mut total = 0u64;
    let mut available = 0u64;
    for line in contents.lines() {
        if let Some(rest) = line.strip_prefix("MemTotal:") {
            total = rest.trim().trim_end_matches(" kB").trim().parse().unwrap_or(0);
        } else if let Some(rest) = line.strip_prefix("MemAvailable:") {
            available = rest.trim().trim_end_matches(" kB").trim().parse().unwrap_or(0);
        }
    }
    MemInfo { total_kb: total, available_kb: available }
}

fn disk_usage(mount: &str) -> (u64, u64) {
    let cpath = CString::new(mount).unwrap();
    unsafe {
        let mut stat: libc::statvfs = std::mem::zeroed();
        if libc::statvfs(cpath.as_ptr(), &mut stat) != 0 {
            return (0, 0);
        }
        let block_size = stat.f_frsize as u64;
        let total = stat.f_blocks as u64 * block_size;
        let free = stat.f_bfree as u64 * block_size;
        (total.saturating_sub(free), total)
    }
}

/// Vendor EC accessed over /dev/port. The read handshake mirrors the one in
/// SCCS.exe: poll the status port for the input buffer to drain, write the
/// read command, write the register address, wait for the output buffer to
/// fill, then read the byte back.
struct Ec {
    port: fs::File,
}

impl Ec {
    fn open() -> Option<Ec> {
        fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/port")
            .ok()
            .map(|port| Ec { port })
    }

    fn inb(&self, port: u64) -> u8 {
        let mut b = [0u8; 1];
        match self.port.read_at(&mut b, port) {
            Ok(_) => b[0],
            Err(_) => 0,
        }
    }

    fn outb(&self, port: u64, val: u8) {
        let _ = self.port.write_at(&[val], port);
    }

    /// Poll the status port until `mask` matches `want_set`, giving up after
    /// 10 tries like the original rather than blocking the update loop.
    fn wait_status(&self, mask: u8, want_set: bool) {
        for _ in 0..10 {
            if ((self.inb(EC_PORT_CMD) & mask) != 0) == want_set {
                return;
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    fn read_reg(&self, cmd: u8, addr: u8) -> u8 {
        self.wait_status(0x02, false); // input buffer empty
        self.outb(EC_PORT_CMD, cmd);
        self.wait_status(0x02, false);
        self.outb(EC_PORT_DATA, addr);
        self.wait_status(0x01, true); // output buffer full
        self.inb(EC_PORT_DATA)
    }

    /// Apply a performance mode (1=Energy saving, 2=Balanced, 3=Performance) by
    /// writing the EC exactly as the vendor app does: the mode command followed
    /// by the value, with no read-back. Verified on hardware to move the CPU
    /// RAPL power limits (e.g. 45/54/65 W long-term for modes 1/2/3).
    fn set_mode(&self, value: u8) {
        self.wait_status(0x02, false); // input buffer empty
        self.outb(EC_PORT_CMD, EC_CMD_MODE);
        self.wait_status(0x02, false);
        self.outb(EC_PORT_DATA, value);
    }

    fn fan_rpm(&self) -> i64 {
        let hi = self.read_reg(EC_CMD_FAN, EC_REG_FAN_HI) as i64;
        let lo = self.read_reg(EC_CMD_FAN, EC_REG_FAN_LO) as i64;
        (hi << 8) | lo
    }

    fn temp_c(&self, reg: u8) -> i64 {
        self.read_reg(EC_CMD_TEMP, reg) as i64
    }
}

/// Intel iGPU utilisation via the i915 perf PMU -- the same source
/// intel_gpu_top uses. Each "<engine>-busy" event counts nanoseconds that
/// engine was active, so utilisation is busy_delta / wall_delta. We track
/// every engine and report the busiest, so video decode (vcs) shows up as
/// readily as 3D (rcs).
struct GpuBusy {
    fds: Vec<RawFd>,
    prev: Vec<u64>,
    prev_at: Instant,
}

impl GpuBusy {
    fn open() -> Option<GpuBusy> {
        let pmu_type: u32 = fs::read_to_string("/sys/devices/i915/type")
            .ok()?
            .trim()
            .parse()
            .ok()?;
        let mut fds = Vec::new();
        let entries = fs::read_dir("/sys/devices/i915/events").ok()?;
        let mut names: Vec<String> = entries
            .flatten()
            .filter_map(|e| e.file_name().to_str().map(str::to_string))
            .filter(|n| n.ends_with("-busy"))
            .collect();
        names.sort();
        for name in names {
            let Ok(text) = fs::read_to_string(format!("/sys/devices/i915/events/{name}")) else {
                continue;
            };
            let Some(hex) = text.trim().strip_prefix("config=0x") else {
                continue;
            };
            let Ok(config) = u64::from_str_radix(hex, 16) else {
                continue;
            };
            if let Some(fd) = perf_open(pmu_type, config) {
                fds.push(fd);
            }
        }
        if fds.is_empty() {
            return None;
        }
        let prev = fds.iter().map(|&f| read_counter(f)).collect();
        Some(GpuBusy { fds, prev, prev_at: Instant::now() })
    }

    /// Busiest engine as a whole percentage since the previous call.
    fn percent(&mut self) -> i64 {
        let now = Instant::now();
        let elapsed_ns = now.duration_since(self.prev_at).as_nanos() as u64;
        let cur: Vec<u64> = self.fds.iter().map(|&f| read_counter(f)).collect();
        let pct = if elapsed_ns == 0 {
            0
        } else {
            cur.iter()
                .zip(&self.prev)
                .map(|(c, p)| c.saturating_sub(*p) * 100 / elapsed_ns)
                .max()
                .unwrap_or(0)
        };
        self.prev = cur;
        self.prev_at = now;
        (pct as i64).min(100)
    }
}

fn perf_open(pmu_type: u32, config: u64) -> Option<RawFd> {
    // perf_event_attr: only type/size/config are non-zero for a plain
    // counting event. Zeroed flags mean enabled, not inherited.
    const ATTR_SIZE: usize = 128;
    let mut attr = [0u8; ATTR_SIZE];
    attr[0..4].copy_from_slice(&pmu_type.to_le_bytes());
    attr[4..8].copy_from_slice(&(ATTR_SIZE as u32).to_le_bytes());
    attr[8..16].copy_from_slice(&config.to_le_bytes());
    // pid = -1 (system wide), cpu = 0 (the PMU is not per-task)
    let fd = unsafe {
        libc::syscall(libc::SYS_perf_event_open, attr.as_ptr(), -1i32, 0i32, -1i32, 0u64)
    };
    if fd < 0 { None } else { Some(fd as RawFd) }
}

fn read_counter(fd: RawFd) -> u64 {
    let mut buf = [0u8; 8];
    let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, 8) };
    if n == 8 { u64::from_le_bytes(buf) } else { 0 }
}

/// Cumulative (rx, tx) byte counters summed over physical interfaces. Virtual
/// interfaces (loopback, docker bridges, veth pairs, ...) have no backing
/// device in sysfs, which is what filters them out here.
fn read_net_bytes() -> (u64, u64) {
    let contents = fs::read_to_string("/proc/net/dev").unwrap_or_default();
    parse_net_dev(&contents, |name| {
        Path::new(&format!("/sys/class/net/{name}/device")).exists()
    })
}

fn parse_net_dev(contents: &str, is_physical: impl Fn(&str) -> bool) -> (u64, u64) {
    let mut rx_total = 0u64;
    let mut tx_total = 0u64;
    for line in contents.lines().skip(2) {
        let Some((name, rest)) = line.split_once(':') else {
            continue;
        };
        let name = name.trim();
        if !is_physical(name) {
            continue;
        }
        let fields: Vec<u64> = rest.split_whitespace().filter_map(|f| f.parse().ok()).collect();
        if fields.len() >= 9 {
            rx_total += fields[0];
            tx_total += fields[8];
        }
    }
    (rx_total, tx_total)
}

/// Matches the original's scaling: bytes/sec into K/s, M/s or G/s.
fn format_rate(bytes_per_sec: f64) -> String {
    const KB: f64 = 1024.0;
    const MB: f64 = 1024.0 * 1024.0;
    const GB: f64 = 1024.0 * 1024.0 * 1024.0;
    if bytes_per_sec < 1_024_000.0 {
        format!("{:.1}K/s", bytes_per_sec / KB)
    } else if bytes_per_sec < 1_048_576_000.0 {
        format!("{:.1}M/s", bytes_per_sec / MB)
    } else {
        format!("{:.1}G/s", bytes_per_sec / GB)
    }
}

/// Temperature of the hwmon named `hwmon_name` whose backing device is the
/// given block device, matched by canonical sysfs path. 0 if none.
fn block_hwmon_temp(blockdev: &str, hwmon_name: &str) -> i64 {
    let dev_real = match fs::canonicalize(format!("/sys/block/{blockdev}/device")) {
        Ok(p) => p,
        Err(_) => return 0,
    };
    let entries = match fs::read_dir("/sys/class/hwmon") {
        Ok(e) => e,
        Err(_) => return 0,
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if fs::read_to_string(path.join("name")).map(|s| s.trim() == hwmon_name).unwrap_or(false) {
            if let Ok(hwmon_dev) = fs::canonicalize(path.join("device")) {
                if hwmon_dev == dev_real {
                    return read_temp_c(&path, "temp1_input");
                }
            }
        }
    }
    0
}

/// Disk temperature: NVMe drives expose it through the `nvme` hwmon; SATA/USB
/// drives through `drivetemp` (needs the drivetemp module loaded). 0 if neither.
fn disk_temp_c(blockdev: &str) -> i64 {
    match block_hwmon_temp(blockdev, "nvme") {
        0 => block_hwmon_temp(blockdev, "drivetemp"),
        t => t,
    }
}

struct DateParts {
    year: i32,
    month: i32,
    day: i32,
    hour: i32,
    minute: i32,
    second: i32,
    win_dow: i32, // 0=Sunday..6=Saturday
}

fn now_local() -> DateParts {
    unsafe {
        let t = libc::time(std::ptr::null_mut());
        let mut tm: libc::tm = std::mem::zeroed();
        libc::localtime_r(&t, &mut tm);
        DateParts {
            year: tm.tm_year + 1900,
            month: tm.tm_mon + 1,
            day: tm.tm_mday,
            hour: tm.tm_hour,
            minute: tm.tm_min,
            second: tm.tm_sec,
            win_dow: tm.tm_wday, // libc tm_wday is already Sun=0..Sat=6
        }
    }
}

// ---- weather -------------------------------------------------------------
//
// Date-tile weather, sent in the panel's braced Date payload (verified on the
// panel). Forecast data comes from Open-Meteo (free, no API
// key). Location follows OW_LOCATION:
//   unset / empty  -> auto: geolocate by public IP (so the shared .deb needs no
//                     per-machine config, matching this project's conventions)
//   off/none       -> weather disabled
//   "lat,lon"      -> used directly
//   a city name    -> geocoded via Open-Meteo
// ATOMMAN_WEATHER_REFRESH changes the 600s poll interval. Everything shells out
// to curl and degrades to no weather (empty fields) if anything fails.
//
// Note auto mode sends the machine's public IP to a third-party geolocation
// service (ip-api.com); OW_LOCATION=off avoids any such call.

const WEATHER_REFRESH_DEFAULT: u64 = 600;
// Until the first successful fetch, retry this often instead of waiting the full
// refresh -- so weather appears soon after the network comes up (e.g. Wi-Fi that
// connects well after boot) rather than staying blank for up to `refresh`.
const WEATHER_RETRY: Duration = Duration::from_secs(30);

struct Weather {
    code: i64, // panel weather-icon code (1..40)
    lo: i64,
    hi: i64,
    zone: String,
    desc: String,
}

/// {Date:YYYY/MM/DD;Time:HH:MM:SS;Week:N;Weather:X;TemprLo:L,TemprHi:H,Zone:Z,Desc:D}
/// The weather block is sent with empty values when unavailable, exactly as the
/// reference driver does. Week is Sun=0..Sat=6 (libc tm_wday, sent unmodified).
fn format_date_packet(d: &DateParts, w: Option<&Weather>) -> String {
    let head = format!(
        "{{Date:{:04}/{:02}/{:02};Time:{:02}:{:02}:{:02};Week:{}",
        d.year, d.month, d.day, d.hour, d.minute, d.second, d.win_dow
    );
    match w {
        Some(w) => format!(
            "{head};Weather:{};TemprLo:{},TemprHi:{},Zone:{},Desc:{}}}",
            w.code, w.lo, w.hi, w.zone, w.desc
        ),
        None => format!("{head};Weather:;TemprLo:,TemprHi:,Zone:,Desc:}}"),
    }
}

struct Located {
    lat: f64,
    lon: f64,
    zone: String,
}

enum LocMode {
    Auto,           // geolocate by public IP
    Off,            // weather disabled
    Manual(String), // explicit "lat,lon" or a city name
}

struct WeatherState {
    mode: LocMode,
    refresh: Duration,
    located: Option<Located>,
    last: Option<Weather>,
    fetched_at: Option<Instant>,
}

impl WeatherState {
    fn from_env() -> WeatherState {
        let mode = match std::env::var("OW_LOCATION") {
            Err(_) => LocMode::Auto,
            Ok(s) => {
                let s = s.trim();
                if s.is_empty() {
                    LocMode::Auto
                } else if ["off", "none", "disabled"].iter().any(|k| s.eq_ignore_ascii_case(k)) {
                    LocMode::Off
                } else {
                    LocMode::Manual(s.to_string())
                }
            }
        };
        let refresh = std::env::var("ATOMMAN_WEATHER_REFRESH")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(WEATHER_REFRESH_DEFAULT);
        WeatherState {
            mode,
            refresh: Duration::from_secs(refresh),
            located: None,
            last: None,
            fetched_at: None,
        }
    }

    fn mode_label(&self) -> &'static str {
        match self.mode {
            LocMode::Auto => "auto (IP geolocation)",
            LocMode::Off => "off (OW_LOCATION=off)",
            LocMode::Manual(_) => "manual (OW_LOCATION)",
        }
    }

    /// Latest weather, refetched at most once per `refresh` once we have a
    /// value -- but retried every `WEATHER_RETRY` until that first success, so a
    /// network that arrives late (Wi-Fi after boot) is picked up quickly rather
    /// than leaving the tile blank for up to `refresh`. Returns the previous
    /// value between refreshes and None when disabled or never fetched.
    fn current(&mut self) -> Option<&Weather> {
        if matches!(self.mode, LocMode::Off) {
            return None;
        }
        let interval = if self.last.is_some() { self.refresh } else { WEATHER_RETRY };
        let stale = self.fetched_at.map(|t| t.elapsed() >= interval).unwrap_or(true);
        if stale {
            if self.located.is_none() {
                self.located = match &self.mode {
                    LocMode::Off => None,
                    LocMode::Auto => geolocate_ip(),
                    LocMode::Manual(s) => geocode(s),
                };
            }
            if let Some(loc) = &self.located {
                if let Some(w) = fetch_open_meteo(loc) {
                    self.last = Some(w);
                }
            }
            // Throttle even on failure so a broken network doesn't spawn curl
            // every cycle; the last good value keeps showing.
            self.fetched_at = Some(Instant::now());
        }
        self.last.as_ref()
    }
}

/// Approximate location from the machine's public IP via ip-api.com (keyless,
/// HTTP). Used only in auto mode; sends the public IP to that service.
fn geolocate_ip() -> Option<Located> {
    parse_ipapi(&curl_get("http://ip-api.com/json/?fields=status,city,countryCode,lat,lon")?)
}

fn parse_ipapi(json: &str) -> Option<Located> {
    if json_string(json, "\"status\":").as_deref() != Some("success") {
        return None;
    }
    let lat = json_number(json, "\"lat\":")?;
    let lon = json_number(json, "\"lon\":")?;
    let city = json_string(json, "\"city\":").unwrap_or_default();
    let cc = json_string(json, "\"countryCode\":").unwrap_or_default();
    let zone = match (city.is_empty(), cc.is_empty()) {
        (false, false) => format!("{city},{cc}"),
        (false, true) => city,
        _ => format!("{lat:.2},{lon:.2}"),
    };
    Some(Located { lat, lon, zone })
}

fn curl_get(url: &str) -> Option<String> {
    let out = std::process::Command::new("curl")
        .args(["-s", "--connect-timeout", "3", "--max-time", "5", url])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let body = String::from_utf8_lossy(&out.stdout).into_owned();
    (!body.is_empty()).then_some(body)
}

fn urlencode(s: &str) -> String {
    let mut out = String::new();
    for b in s.bytes() {
        match b {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'_' | b'.' | b'~' => out.push(b as char),
            _ => out.push_str(&format!("%{b:02X}")),
        }
    }
    out
}

/// Resolve OW_LOCATION to coordinates: a literal "lat,lon" is used directly,
/// otherwise the leading name is geocoded via Open-Meteo's keyless API.
fn geocode(spec: &str) -> Option<Located> {
    if let Some((a, b)) = spec.split_once(',') {
        if let (Ok(lat), Ok(lon)) = (a.trim().parse::<f64>(), b.trim().parse::<f64>()) {
            return Some(Located { lat, lon, zone: format!("{lat:.2},{lon:.2}") });
        }
    }
    let name = spec.split(',').next().unwrap_or(spec).trim();
    let url = format!(
        "https://geocoding-api.open-meteo.com/v1/search?name={}&count=1&language=en&format=json",
        urlencode(name)
    );
    parse_geocode(&curl_get(&url)?)
}

fn parse_geocode(json: &str) -> Option<Located> {
    let lat = json_number(json, "\"latitude\":")?;
    let lon = json_number(json, "\"longitude\":")?;
    let name = json_string(json, "\"name\":").unwrap_or_default();
    let cc = json_string(json, "\"country_code\":").unwrap_or_default();
    let zone = if cc.is_empty() { name } else { format!("{name},{cc}") };
    Some(Located { lat, lon, zone })
}

fn fetch_open_meteo(loc: &Located) -> Option<Weather> {
    let url = format!(
        "https://api.open-meteo.com/v1/forecast?latitude={:.6}&longitude={:.6}\
         &current=temperature_2m,weather_code,is_day\
         &daily=weather_code,temperature_2m_max,temperature_2m_min\
         &timezone=auto&forecast_days=1",
        loc.lat, loc.lon
    );
    parse_open_meteo(&curl_get(&url)?, &loc.zone)
}

/// Read the current conditions and today's min/max. The keys are looked up
/// inside the `current`/`daily` objects specifically, because Open-Meteo emits
/// `current_units`/`daily_units` blocks first whose string values (e.g.
/// `"weather_code":"wmo code"`) would otherwise shadow the real numbers.
fn parse_open_meteo(json: &str, zone: &str) -> Option<Weather> {
    let current = json_object(json, "\"current\":")?;
    let daily = json_object(json, "\"daily\":")?;
    let code = json_number(current, "\"weather_code\":")? as i64;
    let is_day = json_number(current, "\"is_day\":").unwrap_or(1.0) as i64 != 0;
    let lo = json_array_first(daily, "\"temperature_2m_min\":")?;
    let hi = json_array_first(daily, "\"temperature_2m_max\":")?;
    Some(Weather {
        code: wmo_to_panel(code, is_day),
        lo: lo.round() as i64,
        hi: hi.round() as i64,
        zone: sanitize(zone),
        desc: sanitize(wmo_desc(code)),
    })
}

/// Keep payload-breaking bytes out of free-text fields: semicolons become
/// commas and non-ASCII becomes '?', matching the reference driver.
fn sanitize(s: &str) -> String {
    s.chars()
        .map(|c| if c == ';' { ',' } else if c.is_ascii() { c } else { '?' })
        .collect()
}

/// Open-Meteo WMO weather code -> panel icon code (1..40), with day/night
/// variants for clear and cloudy conditions. Unknown codes fall back to
/// overcast (9).
fn wmo_to_panel(code: i64, is_day: bool) -> i64 {
    match code {
        0 => if is_day { 1 } else { 3 },
        1 => if is_day { 5 } else { 6 },
        2 => if is_day { 7 } else { 8 },
        3 => 9,
        45 | 48 => 30,
        51 | 53 | 55 => 13,
        56 | 57 => 19,
        61 => 13,
        63 => 14,
        65 => 15,
        66 | 67 => 19,
        71 => 22,
        73 => 23,
        75 => 24,
        77 => 21,
        80 | 81 => 10,
        82 => 15,
        85 | 86 => 24,
        95 => 11,
        96 | 99 => 16,
        _ => 9,
    }
}

/// Short description for a WMO code (Open-Meteo returns no text field).
fn wmo_desc(code: i64) -> &'static str {
    match code {
        0 => "clear sky",
        1 => "mainly clear",
        2 => "partly cloudy",
        3 => "overcast",
        45 | 48 => "fog",
        51 => "light drizzle",
        53 => "drizzle",
        55 => "dense drizzle",
        56 | 57 => "freezing drizzle",
        61 => "light rain",
        63 => "rain",
        65 => "heavy rain",
        66 | 67 => "freezing rain",
        71 => "light snow",
        73 => "snow",
        75 => "heavy snow",
        77 => "snow grains",
        80 => "light showers",
        81 => "showers",
        82 => "violent showers",
        85 | 86 => "snow showers",
        95 => "thunderstorm",
        96 | 99 => "thunderstorm, hail",
        _ => "",
    }
}

fn json_number(s: &str, key: &str) -> Option<f64> {
    let rest = s[s.find(key)? + key.len()..].trim_start();
    let end = rest
        .find(|c: char| !matches!(c, '0'..='9' | '-' | '+' | '.' | 'e' | 'E'))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

fn json_array_first(s: &str, key: &str) -> Option<f64> {
    let rest = s[s.find(key)? + key.len()..].trim_start().strip_prefix('[')?.trim_start();
    let end = rest
        .find(|c: char| !matches!(c, '0'..='9' | '-' | '+' | '.' | 'e' | 'E'))
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

fn json_string(s: &str, key: &str) -> Option<String> {
    let rest = s[s.find(key)? + key.len()..].trim_start().strip_prefix('"')?;
    let end = rest.find('"')?;
    Some(rest[..end].to_string())
}

/// Inner text of the object value at `key` (`key` must include the closing
/// quote and colon, e.g. `"current":`, so `"current_units":` is not matched),
/// delimited by its matching brace.
fn json_object<'a>(s: &'a str, key: &str) -> Option<&'a str> {
    let rest = s[s.find(key)? + key.len()..].trim_start().strip_prefix('{')?;
    let mut depth = 1;
    for (idx, c) in rest.char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&rest[..idx]);
                }
            }
            _ => {}
        }
    }
    None
}

/// Values detected once at startup rather than hardcoded, so the same binary
/// works on other machines.
struct HostInfo {
    cpu_name: String,
    gpu_name: String,
    disk_label: String,
    disk_blockdev: String,
}

impl HostInfo {
    /// The GPU name is derived from the already-detected backend rather than
    /// re-probed, so an eGPU's name matches the card whose stats we report.
    fn detect(gpu_name: String) -> HostInfo {
        let disk_blockdev = root_blockdev(DISK_MOUNT).unwrap_or_default();
        HostInfo {
            cpu_name: cpu_name(),
            gpu_name,
            disk_label: disk_model(&disk_blockdev),
            disk_blockdev,
        }
    }
}

/// Carried across update cycles so rates can be derived from counter deltas.
struct State {
    prev_cpu: CpuTimes,
    prev_net: (u64, u64),
    prev_net_at: Instant,
    last_fan_rpm: i64,
    gpu: GpuBackend,
    weather: WeatherState,
}

fn build_packets(host: &HostInfo, ec: Option<&Ec>, state: &mut State) -> Vec<(u8, String)> {
    let mut packets = Vec::new();

    let cpu_temp_hwmon = CPU_TEMP_HWMON.iter().find_map(|n| find_hwmon_by_name(n));
    let cpu_temp = cpu_temp_hwmon.as_ref().map(|p| read_temp_c(p, "temp1_input")).unwrap_or(0);
    let cur_cpu_times = read_cpu_times();
    let cpu_pct = cpu_percent(&state.prev_cpu, &cur_cpu_times);
    state.prev_cpu = cur_cpu_times;
    let freq = cpu_freq_khz();
    // Tempr comes from the kernel's coretemp sensor (the original uses its
    // MSR-based equivalent here); Tempr1 prefers vendor EC register 0x20,
    // which tracks coretemp closely, falling back when the read looks torn.
    let cpu_temp_ec = ec
        .map(|e| e.temp_c(EC_REG_CPU_TEMP))
        .filter(|t| *t <= EC_CPU_TEMP_SANITY_MAX)
        .unwrap_or(cpu_temp);
    packets.push((
        CMD_CPU,
        format!(
            "{{CPU:{};Tempr:{cpu_temp};Useage:{cpu_pct};Freq:{freq};Tempr1:{cpu_temp_ec};}}",
            host.cpu_name
        ),
    ));

    // For the Intel iGPU, temperature comes from vendor EC register 0x22 (the
    // same source the Windows app reads -- a separate sensor ~10-13C below the
    // CPU package) and utilisation from the i915 PMU. A discrete eGPU reports
    // its own temperature and utilisation through its driver instead.
    let (gpu_temp, gpu_pct) = match &mut state.gpu {
        GpuBackend::Nvidia => nvidia_temp_usage(),
        GpuBackend::Amd(a) => (a.temp(), a.usage()),
        GpuBackend::Intel(busy) => {
            let temp = ec
                .map(|e| e.temp_c(EC_REG_GPU_TEMP))
                .filter(|t| *t <= EC_GPU_TEMP_SANITY_MAX)
                .unwrap_or(cpu_temp);
            (temp, busy.as_mut().map(|g| g.percent()).unwrap_or(0))
        }
    };
    packets.push((
        CMD_GPU,
        format!("{{GPU:{};Tempr:{gpu_temp};Useage:{gpu_pct}}}", host.gpu_name),
    ));

    let mem = read_meminfo();
    let total_gb = mem.total_kb as f64 / (1024.0 * 1024.0);
    let avail_gb = mem.available_kb as f64 / (1024.0 * 1024.0);
    let used_gb = total_gb - avail_gb;
    let mem_pct = if mem.total_kb > 0 {
        ((mem.total_kb - mem.available_kb) as f64 / mem.total_kb as f64 * 100.0).round() as i64
    } else {
        0
    };
    packets.push((
        CMD_MEMORY,
        format!("{{Memory:Generic Memory;Used:{used_gb:.1};Available:{avail_gb:.1};Total:{total_gb:.1};Useage:{mem_pct}}}"),
    ));

    let (used_bytes, total_bytes) = disk_usage(DISK_MOUNT);
    let used_disk_gb = (used_bytes / (1024 * 1024 * 1024)) as i64;
    let total_disk_gb = (total_bytes / (1024 * 1024 * 1024)) as i64;
    let disk_pct = if total_bytes > 0 { (used_bytes * 100 / total_bytes) as i64 } else { 0 };
    let disk_temp = disk_temp_c(&host.disk_blockdev);
    packets.push((
        CMD_DISK,
        format!(
            "{{DiskName:{};Tempr:{disk_temp};UsageSpace:{used_disk_gb};AllSpace:{total_disk_gb};Usage:{disk_pct}}}",
            host.disk_label
        ),
    ));

    let d = now_local();
    packets.push((CMD_DATE, format_date_packet(&d, state.weather.current())));

    // Fan RPM and network rates share one packet. The original app discards
    // implausible fan readings (the two register bytes are read in separate
    // EC transactions, so the value can tear while the fan is ramping) and
    // reuses the previous one, so do the same.
    let fan_rpm = match ec.map(|e| e.fan_rpm()) {
        Some(rpm) if rpm < FAN_RPM_SANITY_MAX => {
            state.last_fan_rpm = rpm;
            rpm
        }
        _ => state.last_fan_rpm,
    };

    let cur_net = read_net_bytes();
    let elapsed = state.prev_net_at.elapsed().as_secs_f64();
    let (rx_rate, tx_rate) = if elapsed > 0.0 {
        (
            cur_net.0.saturating_sub(state.prev_net.0) as f64 / elapsed,
            cur_net.1.saturating_sub(state.prev_net.1) as f64 / elapsed,
        )
    } else {
        (0.0, 0.0)
    };
    state.prev_net = cur_net;
    state.prev_net_at = Instant::now();
    packets.push((
        CMD_SPEED,
        format!(
            "{{SPEED:{fan_rpm};NETWORK:{},{}}}",
            format_rate(tx_rate),
            format_rate(rx_rate)
        ),
    ));

    // The panel polls for this so its volume slider shows the real level;
    // dragging that slider sends DEV_CMD_VOLUME back to us.
    packets.push((CMD_VOLUME, format!("{{VOLUME:{}}}", system_volume_percent())));

    packets
}

/// `--dump` prints detected host info and one round of packets, then exits.
/// Lets the output be inspected without hardware; run it as root to include
/// the EC and PMU derived fields.
fn dump_once() {
    let gpu = GpuBackend::detect();
    let host = HostInfo::detect(gpu.name());
    println!("cpu_name      = {}", host.cpu_name);
    println!("gpu_name      = {}", host.gpu_name);
    println!("gpu_backend   = {}", gpu.label());
    println!("disk_label    = {}", host.disk_label);
    println!("disk_blockdev = {}", host.disk_blockdev);
    let ec = Ec::open();
    println!("ec            = {}", if ec.is_some() { "open" } else { "unavailable (needs root)" });
    let weather = WeatherState::from_env();
    println!("weather       = {}", weather.mode_label());
    let mut state = State {
        prev_cpu: read_cpu_times(),
        prev_net: read_net_bytes(),
        prev_net_at: Instant::now(),
        last_fan_rpm: 0,
        gpu,
        weather,
    };
    std::thread::sleep(Duration::from_millis(500));
    println!("\npackets:");
    for (cmd, payload) in build_packets(&host, ec.as_ref(), &mut state) {
        println!("  0x{cmd:02x}  {payload}");
    }
}

fn main() {
    if std::env::args().any(|a| a == "--dump") {
        dump_once();
        return;
    }

    let gpu = GpuBackend::detect();
    eprintln!("gpu backend: {}", gpu.label());
    let host = HostInfo::detect(gpu.name());
    let ec = Ec::open();
    if ec.is_none() {
        eprintln!("cannot open /dev/port (needs root) -- fan RPM and EC temps will read 0");
    }
    if matches!(gpu, GpuBackend::Intel(None)) {
        eprintln!("i915 perf PMU unavailable -- GPU usage will read 0");
    }
    let weather = WeatherState::from_env();
    eprintln!("weather: {}", weather.mode_label());
    let mut state = State {
        prev_cpu: read_cpu_times(),
        prev_net: read_net_bytes(),
        prev_net_at: Instant::now(),
        last_fan_rpm: 0,
        gpu,
        weather,
    };
    let mut fd: Option<RawFd> = None;
    let mut input: Vec<u8> = Vec::new();
    let mut last_volume: Option<u8> = None;
    let mut last_mode: Option<u8> = None;
    let ec_ref = ec.as_ref();

    // Drain whatever the panel has sent and act on its touch commands. Called
    // between writes so a volume drag or a mode tap is applied without waiting a
    // full cycle. A drag emits many frames, so only act when the value actually
    // changes -- otherwise every frame would fork a wpctl process (volume) or
    // re-poke the EC (mode).
    let pump = |fd: RawFd, input: &mut Vec<u8>, last_vol: &mut Option<u8>, last_mode: &mut Option<u8>| {
        input.extend_from_slice(&read_available(fd));
        for (cmd, values) in parse_device_frames(input) {
            match cmd {
                DEV_CMD_VOLUME => {
                    if let Some(&v) = values.first() {
                        if *last_vol != Some(v) {
                            *last_vol = Some(v);
                            set_system_volume(v);
                        }
                    }
                }
                // The "Mode Adjustment" button; apply the performance mode via
                // the EC (needs root). Ignored without the EC, like the temps.
                DEV_CMD_MODE => {
                    if let Some(&m) = values.first() {
                        if (MODE_MIN..=MODE_MAX).contains(&m) && *last_mode != Some(m) {
                            *last_mode = Some(m);
                            if let Some(e) = ec_ref {
                                e.set_mode(m);
                                eprintln!("performance mode set to {m}");
                            }
                        }
                    }
                }
                _ => {}
            }
        }
    };

    loop {
        if fd.is_none() {
            match find_port() {
                Some(path) => match open_port(&path) {
                    Ok(f) => {
                        fd = Some(f);
                        input.clear();
                    }
                    Err(e) => {
                        eprintln!("failed to open {path}: {e}");
                        std::thread::sleep(Duration::from_secs(2));
                        continue;
                    }
                },
                None => {
                    std::thread::sleep(Duration::from_secs(2));
                    continue;
                }
            }
        }

        let cycle_start = Instant::now();
        let packets = build_packets(&host, ec.as_ref(), &mut state);

        let mut broke = false;
        for (cmd, payload) in &packets {
            if let Err(e) = write_all(fd.unwrap(), &frame(*cmd, payload)) {
                eprintln!("write failed: {e}");
                unsafe { libc::close(fd.unwrap()) };
                fd = None;
                broke = true;
                break;
            }
            pump(fd.unwrap(), &mut input, &mut last_volume, &mut last_mode);
            std::thread::sleep(Duration::from_millis(20));
        }
        if broke {
            std::thread::sleep(Duration::from_secs(2));
            continue;
        }

        // Service input for the remainder of the cycle, so the whole loop lands
        // on UPDATE_INTERVAL rather than interval + time already spent. The
        // panel legitimately goes quiet on non-stats screens (mode menu, clock
        // face), so silence is not treated as a dead link -- a failed write is
        // the reliable disconnect signal, handled above.
        let deadline = cycle_start + UPDATE_INTERVAL;
        while Instant::now() < deadline {
            pump(fd.unwrap(), &mut input, &mut last_volume, &mut last_mode);
            std::thread::sleep(Duration::from_millis(25));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- framing -------------------------------------------------------

    #[test]
    fn host_frame_layout_matches_protocol() {
        // AA | len_lo len_hi | cmd | payload | CC 33 C3 3C, len = payload + 5
        let f = frame(CMD_CPU, "{A}");
        assert_eq!(f, vec![0xAA, 0x08, 0x00, 0x32, b'{', b'A', b'}', 0xCC, 0x33, 0xC3, 0x3C]);
        assert_eq!(f[1] as usize | ((f[2] as usize) << 8), "{A}".len() + 5);
        assert_eq!(f.len(), "{A}".len() + 8);
    }

    #[test]
    fn host_frame_length_is_little_endian_for_long_payloads() {
        let payload = "x".repeat(300);
        let f = frame(CMD_CPU, &payload);
        let len = f[1] as usize | ((f[2] as usize) << 8);
        assert_eq!(len, 305);
        assert_eq!(f[1], 0x31); // 305 = 0x0131
        assert_eq!(f[2], 0x01);
    }

    // ---- device -> host parsing ---------------------------------------

    fn poll(cmd: u8) -> Vec<u8> {
        vec![0xAA, 0x05, cmd, 0xCC, 0x33, 0xC3, 0x3C]
    }

    fn volume_frame(level: u8) -> Vec<u8> {
        vec![0xAA, 0x06, DEV_CMD_VOLUME, level, 0xCC, 0x33, 0xC3, 0x3C]
    }

    #[test]
    fn routine_polls_are_ignored() {
        let mut buf: Vec<u8> = [0x32, 0x33, 0x34].iter().flat_map(|&c| poll(c)).collect();
        assert!(parse_device_frames(&mut buf).is_empty());
        assert!(buf.is_empty(), "fully consumed");
    }

    #[test]
    fn volume_command_is_decoded() {
        let mut buf = volume_frame(50);
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![50])]);
    }

    #[test]
    fn volume_extremes_round_trip() {
        for level in [0u8, 50, 98, 100] {
            let mut buf = volume_frame(level);
            let got = parse_device_frames(&mut buf);
            assert_eq!(got, vec![(DEV_CMD_VOLUME, vec![level])], "level {level}");
        }
    }

    #[test]
    fn mode_command_is_decoded() {
        // "Mode Adjustment": AA 06 62 <1|2|3> CC 33 C3 3C, captured on hardware.
        for m in [MODE_MIN, 2, MODE_MAX] {
            let mut buf = vec![0xAA, 0x06, DEV_CMD_MODE, m, 0xCC, 0x33, 0xC3, 0x3C];
            assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_MODE, vec![m])], "mode {m}");
        }
    }

    #[test]
    fn command_is_found_among_polls() {
        let mut buf = poll(0x32);
        buf.extend(volume_frame(77));
        buf.extend(poll(0x39));
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![77])]);
    }

    #[test]
    fn partial_frame_is_retained_until_complete() {
        let full = volume_frame(42);
        let (head, tail) = full.split_at(4);
        let mut buf = head.to_vec();
        assert!(parse_device_frames(&mut buf).is_empty(), "incomplete yields nothing");
        assert_eq!(buf.len(), 4, "partial frame kept for next read");
        buf.extend_from_slice(tail);
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![42])]);
    }

    #[test]
    fn resyncs_past_leading_garbage() {
        let mut buf = vec![0x00, 0xFF, 0x12];
        buf.extend(volume_frame(9));
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![9])]);
    }

    #[test]
    fn bad_trailer_is_dropped_without_hanging() {
        let mut buf = vec![0xAA, 0x06, DEV_CMD_VOLUME, 1, 0xDE, 0xAD, 0xBE, 0xEF];
        buf.extend(volume_frame(60));
        // the corrupt frame is discarded; the following good one still parses
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![60])]);
    }

    #[test]
    fn implausible_length_does_not_stall() {
        // 0xAA followed by a nonsense length must not loop forever
        let mut buf = vec![0xAA, 0xFF];
        buf.extend(volume_frame(5));
        assert_eq!(parse_device_frames(&mut buf), vec![(DEV_CMD_VOLUME, vec![5])]);
    }

    // ---- formatting ----------------------------------------------------

    #[test]
    fn rate_units_match_the_original_thresholds() {
        assert_eq!(format_rate(0.0), "0.0K/s");
        assert_eq!(format_rate(1024.0), "1.0K/s");
        // switches to M/s at 1_024_000 B/s, not at 1 MiB
        assert_eq!(format_rate(1_023_999.0), "1000.0K/s");
        assert_eq!(format_rate(1_024_000.0), "1.0M/s");
        assert_eq!(format_rate(1_048_575_999.0), "1000.0M/s");
        assert_eq!(format_rate(1_048_576_000.0), "1.0G/s");
    }

    #[test]
    fn nvidia_temp_usage_is_parsed() {
        // nvidia-smi csv,noheader,nounits: "temperature.gpu, utilization.gpu"
        assert_eq!(parse_nvidia_temp_usage("45, 12"), (45, 12));
        assert_eq!(parse_nvidia_temp_usage("60,100"), (60, 100));
        // a torn/short line must not panic
        assert_eq!(parse_nvidia_temp_usage(""), (0, 0));
        assert_eq!(parse_nvidia_temp_usage("55"), (55, 0));
    }

    #[test]
    fn lspci_device_name_prefers_bracketed_marketing_name() {
        let line = r#"03:00.0 "VGA compatible controller" "Advanced Micro Devices, Inc. [AMD/ATI]" "Navi 33 [Radeon RX 7600]" -r00 "#;
        assert_eq!(parse_lspci_device(line).as_deref(), Some("Radeon RX 7600"));
        // no brackets -> "<short vendor> <device>"
        let plain = r#"00:02.0 "Display controller" "Intel Corporation" "Some iGPU""#;
        assert_eq!(parse_lspci_device(plain).as_deref(), Some("Intel Some iGPU"));
    }

    // ---- weather / date tile ------------------------------------------

    fn sample_date() -> DateParts {
        DateParts { year: 2025, month: 9, day: 15, hour: 14, minute: 22, second: 10, win_dow: 1 }
    }

    #[test]
    fn date_packet_without_weather_sends_empty_block() {
        assert_eq!(
            format_date_packet(&sample_date(), None),
            "{Date:2025/09/15;Time:14:22:10;Week:1;Weather:;TemprLo:,TemprHi:,Zone:,Desc:}"
        );
    }

    #[test]
    fn date_packet_with_weather_matches_reference_format() {
        let w = Weather { code: 4, lo: 12, hi: 27, zone: "Denver,US".into(), desc: "broken clouds".into() };
        assert_eq!(
            format_date_packet(&sample_date(), Some(&w)),
            "{Date:2025/09/15;Time:14:22:10;Week:1;Weather:4;TemprLo:12,TemprHi:27,Zone:Denver,US,Desc:broken clouds}"
        );
    }

    #[test]
    fn wmo_maps_day_night_and_unknown() {
        assert_eq!(wmo_to_panel(0, true), 1); // clear, day
        assert_eq!(wmo_to_panel(0, false), 3); // clear, night
        assert_eq!(wmo_to_panel(95, true), 11); // thunderstorm
        assert_eq!(wmo_to_panel(96, true), 16); // thunderstorm w/ hail
        assert_eq!(wmo_to_panel(12345, true), 9); // unknown -> overcast
    }

    #[test]
    fn open_meteo_reads_current_then_daily() {
        // *_units blocks come first and their STRING values must not be read as
        // the data (this shadowing was a real bug).
        let json = r#"{"latitude":39.7,"longitude":-104.9,
          "current_units":{"weather_code":"wmo code","is_day":""},
          "current":{"time":"t","temperature_2m":18.4,"weather_code":3,"is_day":1},
          "daily_units":{"temperature_2m_max":"°C","temperature_2m_min":"°C"},
          "daily":{"time":["d"],"weather_code":[61],"temperature_2m_max":[26.6],"temperature_2m_min":[11.2]}}"#;
        let w = parse_open_meteo(json, "Denver,US").expect("parsed");
        assert_eq!(w.code, 9, "current weather_code 3 (overcast), not daily's 61");
        assert_eq!((w.lo, w.hi), (11, 27), "rounded from daily min/max");
        assert_eq!(w.zone, "Denver,US");
        assert_eq!(w.desc, "overcast");
    }

    #[test]
    fn open_meteo_night_clear_uses_night_icon() {
        let json = r#"{"current":{"weather_code":0,"is_day":0},
          "daily":{"temperature_2m_max":[5.0],"temperature_2m_min":[-2.4]}}"#;
        let w = parse_open_meteo(json, "z").expect("parsed");
        assert_eq!(w.code, 3, "clear at night");
        assert_eq!((w.lo, w.hi), (-2, 5));
    }

    #[test]
    fn geocode_parses_literal_lat_lon() {
        let loc = geocode("39.7392,-104.9903").expect("literal coords");
        assert!((loc.lat - 39.7392).abs() < 1e-6);
        assert!((loc.lon - (-104.9903)).abs() < 1e-6);
    }

    #[test]
    fn ipapi_result_is_parsed() {
        let json = r#"{"status":"success","countryCode":"GB","city":"Northampton","lat":52.25,"lon":-0.8833}"#;
        let loc = parse_ipapi(json).expect("geolocated");
        assert_eq!((loc.lat, loc.lon), (52.25, -0.8833));
        assert_eq!(loc.zone, "Northampton,GB");
    }

    #[test]
    fn ipapi_failure_status_is_rejected() {
        assert!(parse_ipapi(r#"{"status":"fail","message":"private range"}"#).is_none());
    }

    #[test]
    fn geocode_result_is_parsed() {
        let json = r#"{"results":[{"name":"Denver","latitude":39.74,"longitude":-104.98,"country_code":"US"}]}"#;
        let loc = parse_geocode(json).expect("geocoded");
        assert_eq!((loc.lat, loc.lon), (39.74, -104.98));
        assert_eq!(loc.zone, "Denver,US");
    }

    #[test]
    fn zone_and_desc_are_sanitized() {
        // semicolons would break the ; -delimited payload; non-ascii -> '?'
        assert_eq!(sanitize("a;b"), "a,b");
        assert_eq!(sanitize("Zürich"), "Z?rich");
    }

    // ---- /proc parsing -------------------------------------------------

    #[test]
    fn cpu_total_excludes_guest_fields() {
        // user nice system idle iowait irq softirq steal guest guest_nice
        let stat = "cpu  100 0 100 700 0 0 0 0 500 500\ncpu0 1 2 3 4\n";
        let t = parse_cpu_times(stat);
        assert_eq!(t.total, 900, "guest/guest_nice must not be summed again");
        assert_eq!(t.idle, 700);
    }

    #[test]
    fn cpu_percent_is_busy_over_total() {
        let prev = CpuTimes { idle: 100, total: 200 };
        let cur = CpuTimes { idle: 150, total: 300 };
        assert_eq!(cpu_percent(&prev, &cur), 50);
    }

    #[test]
    fn cpu_percent_handles_no_elapsed_time() {
        let t = CpuTimes { idle: 10, total: 20 };
        let same = CpuTimes { idle: 10, total: 20 };
        assert_eq!(cpu_percent(&t, &same), 0);
    }

    #[test]
    fn meminfo_is_parsed() {
        let m = parse_meminfo("MemTotal:       131401364 kB\nMemFree: 1 kB\nMemAvailable:    27874492 kB\n");
        assert_eq!(m.total_kb, 131_401_364);
        assert_eq!(m.available_kb, 27_874_492);
    }

    #[test]
    fn net_dev_sums_only_physical_interfaces() {
        let contents = "\
Inter-|   Receive                                                |  Transmit
 face |bytes    packets errs drop fifo frame compressed multicast|bytes    packets
    lo:  999999    10    0    0    0     0          0         0    999999      10
  eth0:    1000    10    0    0    0     0          0         0      2000      20
docker0:  555555     1    0    0    0     0          0         0     555555      1
";
        let (rx, tx) = parse_net_dev(contents, |n| n == "eth0");
        assert_eq!((rx, tx), (1000, 2000), "lo and docker0 excluded");
    }

    // ---- EC sanity thresholds -----------------------------------------

    #[test]
    fn ec_sanity_limits_match_the_original() {
        // torn EC reads (e.g. 243C) must be rejected, real values kept
        assert!(!(243i64 <= EC_CPU_TEMP_SANITY_MAX));
        assert!(72i64 <= EC_CPU_TEMP_SANITY_MAX);
        assert!(65i64 <= EC_GPU_TEMP_SANITY_MAX);
        assert!(2400i64 < FAN_RPM_SANITY_MAX);
        assert!(!(65535i64 < FAN_RPM_SANITY_MAX));
    }

    #[test]
    fn fan_rpm_is_big_endian_across_two_registers() {
        // hi=0x08 lo=0xF5 -> 2293 rpm, as observed on hardware
        let (hi, lo) = (0x08i64, 0xF5i64);
        assert_eq!((hi << 8) | lo, 2293);
    }
}

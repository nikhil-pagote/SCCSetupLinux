# How scc-lcd-daemon works

This is a walkthrough of the codebase for anyone who hasn't read it before —
what each part does and why, in the order you'd want to understand it rather
than the order it appears in the file. For protocol/register-level reference
(exact command bytes, EC addresses, unit conventions), the header comment at
the top of `src/main.rs` is the in-repo spec — this document explains the
*code*, that comment records the wire format it implements.

The whole daemon is one file: `src/main.rs` (~1600 lines including tests).
There was no reason to split it up — it's one loop with a handful of data
sources feeding it.

## The problem it solves

The AtomMan X7 Ti's front-panel LCD (CPU/GPU/RAM/SSD/fan/network stats, weather,
touch volume slider, and an Energy-saving/Balanced/Performance mode button) only
worked with a Windows app (`SCCSetup.exe`, actually `app/SCCS.exe` once
installed). This daemon replaces that app: same wire protocol, same registers,
running natively on Linux. The vendor installer itself is not redistributed
here — it was used only as a reference during reverse engineering, never
executed.

## Big picture

```
┌─────────────────────────────────────────────────────────────┐
│                      main() loop, every 1s                  │
│                                                               │
│  build_packets()  ──►  write CPU/GPU/Mem/Disk/Date/Speed/Vol │
│       │                  frames to the panel, one at a time  │
│       │                  (pump() drains panel input between  │
│       ▼                   each write so a volume drag lands  │
│  /proc, /sys,              promptly)                          │
│  vendor EC, i915 PMU,                                         │
│  nvidia-smi / amdgpu,                                         │
│  wpctl, curl (weather)                                        │
└─────────────────────────────────────────────────────────────┘
              ▲                                    │
              │                                    ▼
      panel polls host for              panel sends touch commands
      each field (ignored —              (volume + performance mode)
      we push on our own schedule)       back over the same serial link
```

The panel is a USB-CDC virtual serial device (`/dev/ttyACM0`, stabilized to
`/dev/sccs-lcd` by the udev rule). It's chatty: it continuously polls for
every data type it displays. The daemon ignores those polls and just pushes
fresh values on its own 1-second cadence — the panel updates the instant it
receives a packet, poll or no poll.

## Reading the source top to bottom

### Serial transport (`find_port`, `open_port`)

`find_port()` honors `SCC_LCD_PORT` if set (pointing at a pty instead of the
panel, for development), otherwise picks `/dev/sccs-lcd` if the udev rule
created it and falls back to `/dev/ttyACM0`.

`open_port()` opens it and puts the tty into raw mode (no line discipline —
we want bytes exactly as sent/received) at 115200 8N1, then raises DTR/RTS.
The baud rate is nominal: this is a USB-CDC device, so the line settings
aren't driving a real UART.

### Framing — the two directions are NOT symmetric

This is the single easiest thing to get backwards, so it's called out loudly
in the code and in `panel-protocol`.

**Host → device** (`frame()` in main.rs): 2-byte little-endian length.

```
0xAA | len_lo len_hi | cmd | payload bytes... | 0xCC 0x33 0xC3 0x3C
len = payload.len() + 5   (cmd byte + payload + 4-byte trailer)
```

**Device → host** (`parse_device_frames()`): a single length byte.

```
0xAA | len | cmd | [value bytes...] | 0xCC 0x33 0xC3 0x3C
total bytes = len + 2
```

`parse_device_frames()` is written defensively because serial input can be
sheared at any boundary (partial reads, corrupted bytes):
- it resyncs to the next `0xAA` if the buffer doesn't start with one,
- it rejects implausible total lengths (`total` outside `6..=64`) rather than
  waiting forever for bytes that will never come,
- it holds onto an incomplete frame across calls until the rest arrives,
- it verifies the 4-byte trailer and silently drops anything that doesn't
  match rather than desyncing the whole stream.

One-byte bodies are routine polls ("send me CPU data") and are dropped —
the daemon already pushes that data on its own schedule. Two-or-more-byte
bodies are actual commands from the panel's touch UI: `0x61` (volume, 0-100)
and `0x62` (performance mode, 1-3).

### The main loop (`main()`)

Roughly:

```
loop:
  if no open fd: try to open the port, else sleep 2s and retry
  build_packets()                      # gather all current stats
  for each (cmd, payload) packet:
      write it to the panel
      pump()                           # drain+act on panel input
      sleep 20ms
  # spend the rest of the 1s cycle still servicing input
  while now < cycle_start + 1s:
      pump()
      sleep 25ms
```

Two things worth noticing:
- Input is drained *between* writes, not just once per cycle, so a volume
  knob drag on the panel is applied within tens of milliseconds instead of
  waiting for the next full 1-second cycle.
- The final wait is computed as a deadline from `cycle_start`, not as a flat
  extra sleep — so the loop period stays ~1s regardless of how long writing
  and pumping took.
- `pump()` acts only when a decoded value actually *changes* from the last one
  applied: a volume slider drag emits many identical-ish frames per second
  (each would otherwise fork a `wpctl` process), and a mode tap likewise
  re-writes the EC only on a real change.

If a write fails (panel unplugged, etc.), the fd is closed, `fd` is reset to
`None`, and the top of the loop reopens it on the next iteration — no crash,
just a 2-second backoff and retry.

### Gathering the stats (`build_packets`)

Called once per cycle; returns the 7 `(cmd, payload)` pairs to send, built
from a mix of sources:

| stat | source | notes |
|---|---|---|
| CPU name | `/proc/cpuinfo` (once, at startup) | latched in `HostInfo` |
| CPU temp | first of `coretemp`/`k10temp`/`zenpower`/`cpu_thermal`/`acpitz` | fallback chain |
| CPU temp (EC) | vendor EC reg `0x20` | preferred if within sanity range |
| CPU usage | `/proc/stat`, two samples a cycle apart | `cpu_percent()` |
| CPU freq | `scaling_cur_freq` | **kHz**, sent unmodified — panel divides by 1e6 |
| GPU name | `GpuBackend` (nvidia-smi / amdgpu / lspci, once) | discrete preferred over iGPU |
| GPU temp | Intel: vendor EC `0x22`; discrete: its own driver | see `GpuBackend` below |
| GPU usage | Intel: i915 PMU busiest engine; NVIDIA/AMD: driver | see `GpuBackend` below |
| Memory | `/proc/meminfo` | `Used = Total - Available`, not `Total - Free` |
| Disk name/model | `/sys/block/<dev>/device/model` (once) | via `root_blockdev()` |
| Disk usage | `statvfs("/")` | |
| Disk temp | `nvme` hwmon, then `drivetemp`, by canonical path | `disk_temp_c()` |
| Date/time | `libc::localtime_r` | local time, Win32-style Week field |
| Weather | Open-Meteo via `curl` (see below) | optional; on the date tile |
| Fan RPM | vendor EC reg `0x18`/`0x19` | sanity-checked, else reuses last good |
| Network rates | `/proc/net/dev`, physical interfaces only | delta over elapsed time |
| Volume | `wpctl get-volume` | reported back so the panel's slider stays in sync |

`HostInfo` (name/model strings) is detected once at startup and reused every
cycle — these never change while the daemon runs, and the panel latches its
"name" field on first receipt anyway (see Gotchas).

`State` carries the small amount of data that must survive between cycles to
compute deltas/rates: previous `/proc/stat` snapshot, previous network byte
counters + timestamp, the last accepted fan RPM, the selected `GpuBackend`, and
the weather cache (last fetch + resolved location).

### Vendor embedded controller (`Ec`)

Fan RPM and CPU/GPU temperature (the EC-sourced one) come from a vendor EC
that is **not** the standard ACPI EC the kernel manages. It's read via raw
port I/O on `/dev/port` at ports `0x68`/`0x6c` — the same ports and handshake
the Windows app used through its WinRing0 kernel driver. `Ec::read_reg()`
implements that handshake: wait for the input buffer to drain, write the
command byte, wait again, write the register address, wait for the output
buffer to fill, read the result. Each wait gives up after 10 tries (10ms)
rather than blocking the whole update loop if the EC is unresponsive.

Both fan RPM and EC temperature reads apply a sanity ceiling
(`FAN_RPM_SANITY_MAX`, `EC_*_TEMP_SANITY_MAX`) because register reads can
tear mid-transaction — a torn CPU temp read of 243°C has been observed on
real hardware. On a rejected reading, temperature falls back to the coretemp
hwmon value and fan RPM falls back to the last accepted value, matching what
the original Windows app does.

`Ec::set_mode()` is the one EC *write*: it applies the performance mode from the
panel's "Mode Adjustment" button (command `0xDE`, data `1`/`2`/`3`), the same
handshake in reverse. It moves the CPU RAPL power limits (~45/54/65 W long-term
for Energy-saving/Balanced/Performance). The panel reports its current mode on
connect, so a restart re-applies it.

Without root, `Ec::open()` fails to open `/dev/port` and everything EC-backed
falls back to its non-EC source or 0 — no crash, just degraded data (mode taps
are silently ignored too). This is why the service runs as root rather than
under a dedicated unprivileged user.

### GPU utilisation (`GpuBackend`)

`GpuBackend::detect()` picks one GPU at startup: a working NVIDIA card
(`nvidia-smi`) or an AMD `amdgpu` card is preferred over the Intel iGPU, so an
Oculink eGPU is reported instead of the integrated graphics. NVIDIA temp/usage
come from `nvidia-smi`, AMD from `amdgpu` sysfs (`gpu_busy_percent` + hwmon).

The Intel path (`GpuBusy`) uses the i915 driver's per-engine "busy" counters
(nanoseconds the engine was active), exposed as perf PMU events under
`/sys/devices/i915/events/*-busy` — the same interface `intel_gpu_top` uses.
`GpuBusy::open()` opens a raw `perf_event_open` counting-event fd for every
`*-busy` event it finds (3D, video decode, etc.). `percent()` diffs each counter
against the previous sample, divides by wall-clock elapsed time, and reports the
**busiest single engine** — so a video call on the decode engine reads as GPU
activity just as readily as a 3D workload. This is a known simplifying choice,
not a blended/weighted figure across engines. (For the Intel iGPU, temperature
still comes from the vendor EC reg `0x22`, not the PMU.)

### Root disk resolution (`root_blockdev`)

Finding what physical disk backs `/` sounds like it should be `stat()`, but
that breaks under btrfs, LVM, ZFS, or overlay filesystems, which report an
anonymous `st_dev` with major number 0 — no real device behind it. Instead
`root_blockdev()` reads `/proc/mounts` to find the source device for `/`,
steps through device-mapper (LUKS/LVM) to the underlying physical device via
`/sys/class/block/<dm>/slaves`, then climbs from a partition to its parent
whole disk (since the temperature sensor lives on the disk, not the
partition) using the `partition` sysfs attribute as the signal.

### Volume control (`session_uid`, `wpctl`, `set_system_volume`)

The daemon runs as root (needed for the EC and PMU), but PipeWire/WirePlumber
run inside the logged-in user's desktop session — so volume changes have to
be made *as that user*. `session_uid()` scans `/run/user/*` for whichever UID
has a live `pipewire-0` socket and uses that, rather than hardcoding a
username, so the same `.deb` works on any machine/user. `wpctl()` then runs
with that UID and its `XDG_RUNTIME_DIR` set, calling `wpctl set-volume` /
`get-volume` on `@DEFAULT_AUDIO_SINK@`.

### Weather (`WeatherState`)

The date tile can carry weather. `WeatherState` reads `OW_LOCATION` (unset =
auto-locate by public IP via ip-api.com; `off` = disabled; otherwise a `lat,lon`
pair or a city name geocoded through Open-Meteo). The forecast comes from
Open-Meteo (free, keyless) by shelling out to `curl`, cached for
`ATOMMAN_WEATHER_REFRESH` seconds (default 600) and throttled even on failure so
a broken network doesn't spawn `curl` every cycle. WMO weather codes are mapped
to the panel's own icon codes (1-40, with day/night variants) by `wmo_to_panel`.
Everything degrades to no weather (empty fields) if `curl`/network/parse fails.

### `--dump`

`cargo run -- --dump` (or the installed binary with that flag) prints
detected host info, whether the EC and GPU PMU opened successfully, and one
round of built packets, then exits — a quick way to sanity-check what the
daemon sees on a given machine without needing the panel attached or waiting
for the service to run.

## Tests

All tests live in `#[cfg(test)] mod tests` at the bottom of `main.rs` — no
integration test harness, since almost everything worth testing is pure
parsing/formatting logic (framing, `/proc` parsing, rate formatting, sanity
thresholds) that doesn't touch hardware. Run with:

```bash
cargo test
```

Things that genuinely need the physical panel or EC (does the fan reading
match reality, does the panel's slider move) are out of reach for automated
tests — verification for those is manual, by looking at the panel. Most
fields cannot be read back from the device, so "it compiled and sent" is not
evidence a change worked.

## Packaging, install, and the systemd/udev pieces

- **`99-sccs-lcd.rules`**: udev rule matching the panel's USB VID/PID
  (`0416:50a1`), makes the device world-writable (`0666`) and symlinks it to
  `/dev/sccs-lcd` so the daemon doesn't depend on enumeration order.
- **`sccs-lcd.service`**: a *system* (not per-user) systemd unit — it must run
  as root for `/dev/port` and the PMU. `Restart=always` with `RestartSec=2`,
  so it comes back 2s after any exit, clean or not.
- **`packaging/control.in`**: the `.deb` control file template;
  `tools/build-deb.sh` substitutes `@VERSION@` from `Cargo.toml` so the
  version has exactly one source of truth.
- **`postinst`**: reloads udev rules, reloads systemd, enables+starts the
  service. It also tears down the *old* per-user service from 1.2.x
  installs (`systemctl --user disable --now`) before proceeding — a
  one-time migration shim for anyone upgrading from that layout.
- **`prerm`** / **`postrm`**: stop+disable the service before removal,
  reload udev/systemd after.
- **`tools/build-deb.sh`**: builds the release binary, runs the test suite
  (a release build that fails tests never gets packaged), stages a `pkg/`
  tree, and calls `fakeroot dpkg-deb --build` to produce
  `scc-lcd-daemon_<version>_amd64.deb` in the **parent directory of this
  repo** — deliberately outside it, so build output is never committed.

Install is always interactive since `apt install` needs sudo:
`sudo apt install --reinstall scc-lcd-daemon_<ver>_amd64.deb`.

## Gotchas that have already cost real time

These are the traps that are easy to fall into when changing this code:

- **Two EC interfaces exist.** The kernel's ACPI EC is `0x62`/`0x66`; the
  vendor EC is `0x68`/`0x6c`. Reading via `ec_sys` returns zeros — that means
  you're on the wrong EC, not that the data is missing.
- **Inbound and outbound framing differ** — one-byte length from the device,
  two-byte to it.
- **`Freq` is kHz**, not MHz. Sending MHz renders as `0.0GHz`.
- **`stat()` cannot identify the root disk** — btrfs/LVM/ZFS report an
  anonymous `st_dev` with major 0. Resolve through `/proc/mounts`.
- **The panel latches names at power-up**; changing them needs a power cycle,
  not just a service restart.
- **Verification is mostly manual.** Most fields cannot be read back from the
  panel, so confirm a change by looking at the display.
- **The sanity thresholds and odd unit conventions are deliberate** — they're
  copied from the vendor app. Check the `src/main.rs` header comment before
  "fixing" something that looks wrong.

## Known gaps

- The `{Ver;Name}` (`0x31`) identify packet changes nothing observable, so
  the daemon doesn't send it.
- GPU usage is the busiest single engine, not a blended figure.
- Face/theme switching (swipe up for the clock view, left/right to cycle faces)
  is entirely panel-internal — there is no host command for it, so the daemon
  can neither drive nor read it. It works on Linux exactly as on Windows.

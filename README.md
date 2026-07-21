# scc-lcd-daemon

Linux replacement for the Windows-only front-panel LCD driver on the
Minisforum AtomMan X7 Ti. Streams live CPU/GPU/RAM/SSD/fan/network stats — plus
weather — to the case's front status display, and handles the panel's touch
controls: volume, and the Energy-saving / Balanced / Performance mode button.

For how the code actually works internally, see [ARCHITECTURE.md](ARCHITECTURE.md).

## Requirements

- Rust/Cargo (edition 2024, so a reasonably recent toolchain — 1.85+).
  Install via [rustup](https://rustup.rs/) if you don't have it.
- `fakeroot` and `dpkg-deb` — only needed to build the `.deb` package.
- `pciutils` (`lspci`) at runtime — used to detect the GPU name.
- `wireplumber` (`wpctl`) at runtime — used to read/set system volume.
- `curl` at runtime — used to fetch weather; optional, weather is simply off
  without it.
- `nvidia-smi` (NVIDIA) or the `amdgpu` driver (AMD) — only if you drive a
  discrete/eGPU; the built-in Intel Arc iGPU needs neither.

Building and `--dump` work on any machine; actually driving a display needs
the panel attached (USB-CDC device, VID `0416` PID `50a1`).

## Compiling

From the repository root:

```bash
cargo build --release          # binary at target/release/scc-lcd-daemon
cargo test                     # run the unit tests
```

There's nothing else to configure — no build script, no native dependencies
beyond the `libc` crate.

To try the binary without installing it as a service:

```bash
sudo target/release/scc-lcd-daemon --dump
```

`--dump` detects host info (CPU/GPU/disk names), reports whether the vendor
EC and GPU perf PMU opened successfully, prints one round of built packets,
and exits — useful for sanity-checking a machine before wiring up the panel.
Run it with `sudo` to see real EC/PMU-derived values; without root those
fields report 0.

## Building the .deb package

```bash
bash tools/build-deb.sh
```

This builds the release binary, runs the test suite (it refuses to package
a build with failing tests), and writes
`scc-lcd-daemon_<version>_amd64.deb` to the **parent directory of this repo**
— deliberately outside the working tree, so build output never gets
committed. The version comes from `Cargo.toml` — bump it there before
building a new release.

Prebuilt packages are also attached to the
[GitHub releases](https://github.com/nikhil-pagote/SCCSetupLinux/releases)
if you'd rather not build from source.

## Installing

```bash
sudo apt install --reinstall scc-lcd-daemon_<version>_amd64.deb
```

`--reinstall` is used because plain `apt install` on a local file with an
unchanged version number is otherwise a no-op. This also works for a fresh
install.

Installing:
- adds the udev rule that makes the panel world-writable and symlinks it to
  `/dev/sccs-lcd`,
- installs and enables `sccs-lcd.service` (a system-level systemd unit,
  runs as root — required for `/dev/port` EC access and the i915 perf PMU),
- starts the service immediately.

No further setup is needed; the panel should start showing live stats within
a second or two.

## Installing from source (Arch, Fedora, other non-Debian distros)

There is no `.deb` for non-Debian distros, but the daemon is a single static-ish
binary with no runtime dependencies beyond `lspci` and `wpctl`, so a manual
install is a handful of copies. This does by hand exactly what the package's
`postinst` does.

```bash
cargo build --release

# 1. binary — the service unit expects it at /usr/bin/scc-lcd-daemon
sudo install -m755 target/release/scc-lcd-daemon /usr/bin/scc-lcd-daemon

# 2. systemd unit and udev rule
sudo install -m644 sccs-lcd.service      /etc/systemd/system/sccs-lcd.service
sudo install -m644 99-sccs-lcd.rules     /etc/udev/rules.d/99-sccs-lcd.rules

# 3. optional weather/config env file (safe to skip)
sudo install -m644 packaging/scc-lcd.default /etc/default/scc-lcd

# 4. load the udev rule and start the service
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo systemctl daemon-reload
sudo systemctl enable --now sccs-lcd.service
```

Runtime tools differ by distro — install the equivalents of `pciutils`
(`lspci`), `wireplumber` (`wpctl`), and, for weather, `curl` with your package
manager. To uninstall, reverse it:

```bash
sudo systemctl disable --now sccs-lcd.service
sudo rm /usr/bin/scc-lcd-daemon /etc/systemd/system/sccs-lcd.service \
        /etc/udev/rules.d/99-sccs-lcd.rules /etc/default/scc-lcd
sudo systemctl daemon-reload && sudo udevadm control --reload-rules
```

## Using it

The daemon runs continuously as a systemd service once installed — there's
nothing to launch by hand.

Check status / logs:

```bash
systemctl status sccs-lcd.service
journalctl -u sccs-lcd.service -f
```

Restart after a config or hardware change:

```bash
sudo systemctl restart sccs-lcd.service
```

Point it at a different serial device (e.g. a pty for development instead of
the real panel) by setting `SCC_LCD_PORT` — this only matters if you're
running the binary manually, since the installed service doesn't set it:

```bash
SCC_LCD_PORT=/dev/pts/4 sudo -E target/release/scc-lcd-daemon
```

Touch volume control on the panel works automatically once the service is
running — no configuration needed; it detects the logged-in desktop
session's PipeWire socket on its own. The panel's **Mode Adjustment** button
(Energy saving / Balanced / Performance) also works out of the box: the daemon
applies the chosen mode to the hardware via the vendor EC.

**Weather** appears on the date tile automatically, auto-located from the
machine's public IP. To pin a location or turn it off, edit
`/etc/default/scc-lcd`:

```bash
#OW_LOCATION=off          # disable weather (no external calls)
#OW_LOCATION=denver,us    # a specific city
#OW_LOCATION=19.07,72.87  # exact lat,lon
```

Note: the panel's on-screen name fields (CPU/GPU/disk name) latch to
whatever they first receive after power-up. If a name looks wrong right
after changing hardware, a service restart alone won't fix the display —
power-cycle the panel.

## Uninstalling

```bash
sudo apt remove scc-lcd-daemon
```

This stops and disables the service and removes the udev rule.

## License

MIT — see [LICENSE](LICENSE). The reverse-engineered vendor protocol is noted
in [NOTICE](NOTICE).

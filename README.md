# scc-lcd-daemon

Linux replacement for the Windows-only front-panel LCD driver on the
Minisforum AtomMan X7 Ti. Streams live CPU/GPU/RAM/SSD/fan/network stats to
the case's front status display and applies volume changes made from the
panel's own touch control.

For how the code actually works internally, see [ARCHITECTURE.md](ARCHITECTURE.md).

## Requirements

- Rust/Cargo (edition 2024, so a reasonably recent toolchain — 1.85+).
  Install via [rustup](https://rustup.rs/) if you don't have it.
- `fakeroot` and `dpkg-deb` — only needed to build the `.deb` package.
- `pciutils` (`lspci`) at runtime — used to detect the GPU name.
- `wireplumber` (`wpctl`) at runtime — used to read/set system volume.

The panel itself must be attached (USB-CDC device, VID `0416` PID `50a1`).

## Compiling

From the `scc-lcd-daemon` directory:

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
a build with failing tests), and produces
`scc-lcd-daemon_<version>_amd64.deb` one directory up from `scc-lcd-daemon/`
(i.e. in the repo root). The version comes from `Cargo.toml` — bump it there
before building a new release.

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
session's PipeWire socket on its own.

Note: the panel's on-screen name fields (CPU/GPU/disk name) latch to
whatever they first receive after power-up. If a name looks wrong right
after changing hardware, a service restart alone won't fix the display —
power-cycle the panel.

## Uninstalling

```bash
sudo apt remove scc-lcd-daemon
```

This stops and disables the service and removes the udev rule.

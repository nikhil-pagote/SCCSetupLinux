# Changelog

All notable changes to this project are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/), and this project adheres to
semantic versioning. The version is the single source of truth in `Cargo.toml`.

## [1.9.0] - 2026-07-21

### Added
- **Performance-mode button.** The panel's "Mode Adjustment" button
  (Energy saving / Balanced / Performance) now works: the daemon applies the
  selected mode via the vendor EC (`0xDE`→`0x68`, values 1/2/3), verified on
  hardware to move the CPU RAPL power limits (~45/54/65 W long-term). The panel
  reports its current mode on connect, so a daemon restart re-applies it.

### Removed
- **Link-silence reconnect** (added in 1.8.0). It misfired whenever the panel
  sat on a non-stats screen (mode menu, clock face) — normal use — repeatedly
  reopening the port. Genuine disconnects are already covered by the
  write-error reconnect, so silence is no longer treated as a dead link.

## [1.8.0] - 2026-07-21

### Added
- **Discrete/eGPU support.** A GPU backend is selected at startup: a working
  NVIDIA card (`nvidia-smi`) or an AMD `amdgpu` card is preferred over the Intel
  iGPU, so an Oculink eGPU is reported instead of the integrated graphics.
  Temperature and utilisation come from the card's own driver.
- **Sensor fallbacks.** CPU temperature now tries
  `coretemp`/`k10temp`/`zenpower`/`cpu_thermal`/`acpitz`; disk temperature falls
  back from `nvme` to `drivetemp` (SATA/USB).
- **Date-tile weather.** Optional weather block on the Date tile. Forecast data
  comes from Open-Meteo (free, keyless). Location is auto-detected from the
  public IP by default, or
  set via `OW_LOCATION` (`lat,lon`, a city name, or `off`); refresh interval via
  `ATOMMAN_WEATHER_REFRESH` (default 600s).
- **Link-silence reconnect.** The port is reopened after 15s without any panel
  traffic (suspend/resume, USB re-enumeration), not only on a write error.
- **`/etc/default/scc-lcd`** config file (read by the service via
  `EnvironmentFile`), shipped as a preserved conffile.

### Changed
- The Date packet now uses the braced, 4-digit-year format
  (`{Date:YYYY/MM/DD;...}`) to carry the weather block; the panel accepts both
  this and the previous vendor format.
- `curl` added to `Recommends` (used for weather).

## [1.7.0] - 2026-07-20

### Added
- Initial Linux daemon: streams CPU, GPU, memory, SSD, fan RPM and network
  rates to the front panel, and applies volume changes from its touch control.
  Packaged as a `.deb` with a systemd unit and udev rule.

[1.9.0]: https://github.com/nikhil-pagote/SCCSetupLinux/releases/tag/v1.9.0
[1.8.0]: https://github.com/nikhil-pagote/SCCSetupLinux/releases/tag/v1.8.0
[1.7.0]: https://github.com/nikhil-pagote/SCCSetupLinux/releases/tag/v1.7.0

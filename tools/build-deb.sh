#!/bin/bash
# Build the release binary and wrap it in a .deb.
# Version comes from Cargo.toml so there is one source of truth.
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT="$(dirname "$SRC")"
PKG="$ROOT/pkg"

VERSION=$(grep -m1 '^version' "$SRC/Cargo.toml" | cut -d'"' -f2)
DEB="$ROOT/scc-lcd-daemon_${VERSION}_amd64.deb"

echo "building scc-lcd-daemon $VERSION"
cargo build --release --manifest-path "$SRC/Cargo.toml"
cargo test --release --manifest-path "$SRC/Cargo.toml"

rm -rf "$PKG"
mkdir -p "$PKG/DEBIAN" "$PKG/usr/bin" "$PKG/usr/lib/systemd/system" "$PKG/usr/lib/udev/rules.d" "$PKG/etc/default"

install -m 755 "$SRC/target/release/scc-lcd-daemon" "$PKG/usr/bin/scc-lcd-daemon"
install -m 644 "$SRC/sccs-lcd.service"               "$PKG/usr/lib/systemd/system/sccs-lcd.service"
install -m 644 "$SRC/99-sccs-lcd.rules"              "$PKG/usr/lib/udev/rules.d/99-sccs-lcd.rules"
install -m 644 "$SRC/packaging/scc-lcd.default"      "$PKG/etc/default/scc-lcd"

# /etc/default/scc-lcd is user-editable config; mark it so dpkg preserves edits.
echo "/etc/default/scc-lcd" > "$PKG/DEBIAN/conffiles"

sed "s/@VERSION@/$VERSION/" "$SRC/packaging/control.in" > "$PKG/DEBIAN/control"
for s in postinst prerm postrm; do
    install -m 755 "$SRC/packaging/$s" "$PKG/DEBIAN/$s"
done

rm -f "$ROOT"/scc-lcd-daemon_*.deb
fakeroot dpkg-deb --build --root-owner-group "$PKG" "$DEB" >/dev/null
rm -rf "$PKG"

echo "built $DEB"
echo "install with: sudo apt install --reinstall $DEB"

#!/usr/bin/env bash
# install_daemon.sh — build the ART gripper standalone daemon and install
# it as a systemd-managed service.
#
# Usage:
#   ./scripts/install_daemon.sh                # build + install for current user
#   sudo ./scripts/install_daemon.sh --system  # also install systemd unit
#
# Requirements (already covered by install_etherlab.sh):
#   - cmake, build-essential
#   - /opt/etherlab/lib/libethercat.so

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"

# 1. Build
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake "$ROOT_DIR"
make -j"$(nproc)"

BIN="$BUILD_DIR/art_gripper_daemon"
[[ -x "$BIN" ]] || { echo "build failed — $BIN not found" >&2; exit 1; }
echo
echo "[daemon] build OK: $BIN"

# 2. system-wide install (binary + systemd) only if --system is passed
INSTALL_SYSTEM=0
for arg in "$@"; do
    [[ "$arg" == "--system" ]] && INSTALL_SYSTEM=1
done

if [[ $INSTALL_SYSTEM -eq 1 ]]; then
    if [[ $EUID -ne 0 ]]; then
        echo "--system requested but not root. Re-run with sudo." >&2
        exit 1
    fi
    install -m 755 "$BIN" /usr/local/bin/art_gripper_daemon

    # Pick the user under which the daemon runs.
    DAEMON_USER="${SUDO_USER:-$USER}"
    if [[ "$DAEMON_USER" == "root" ]]; then
        echo "warning: running daemon as root. Set SUDO_USER or run sudo from a normal account." >&2
    fi
    sed "s|^User=.*|User=$DAEMON_USER|" \
        "$ROOT_DIR/systemd/art-gripper-daemon.service" \
        > /etc/systemd/system/art-gripper-daemon.service
    systemctl daemon-reload
    systemctl enable art-gripper-daemon
    systemctl restart art-gripper-daemon
    sleep 2
    echo
    echo "[daemon] === systemd status ==="
    systemctl is-active art-gripper-daemon
    ss -tlnp 2>/dev/null | grep 50053 || echo "WARN: port 50053 not listening"
fi

echo
echo "[daemon] === ping test ==="
python3 - <<'PY'
import socket, struct
s = socket.create_connection(("127.0.0.1", 50053), timeout=3)
s.sendall(bytes([0x01]) + struct.pack("<I", 0))
status = s.recv(1)[0]
plen = struct.unpack("<I", s.recv(4))[0]
print("ping:", s.recv(plen) if plen else b"(empty)", "status=", status)
s.close()
PY

echo
echo "Done. Use python/art_gripper_client.py or import ArtGripperInterface in your code."

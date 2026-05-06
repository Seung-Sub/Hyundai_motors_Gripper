#!/usr/bin/env bash
# restart_gripper.sh — Recover the gripper without rebooting the host.
#
# Use this whenever the EtherCAT bus has been disturbed while the host stayed
# powered on:
#   * gripper Ethernet cable unplugged & plugged back in
#   * a different ART gripper swapped onto the same bus
#   * gripper power cycled but host did not reboot
#   * EtherCAT master got stuck (slaves count went to 0, link down)
#
# Order matters: the daemon holds /dev/EtherCAT0, so we must stop the daemon
# first, then restart the master kernel module, then bring the daemon back.
#
# Usage:
#   sudo ./scripts/restart_gripper.sh
#
# Exit code 0 = TCP ping returned pong; non-zero = something is still wrong.

set -e

if [[ $EUID -ne 0 ]]; then
    echo "must run as root (sudo). re-run with: sudo $0" >&2
    exit 1
fi

DAEMON_UNIT=art-gripper-daemon
ECAT_UNIT=ethercat

echo "[restart] 1/4 stop ${DAEMON_UNIT} (release /dev/EtherCAT0 handle) ..."
systemctl stop "$DAEMON_UNIT" 2>/dev/null || true
sleep 1

echo "[restart] 2/4 restart ${ECAT_UNIT} (reload kernel module + reattach NIC) ..."
systemctl restart "$ECAT_UNIT"

# Wait for /dev/EtherCAT0 to reappear (kernel module reload can take a few sec)
for _ in $(seq 1 20); do
    [[ -e /dev/EtherCAT0 ]] && break
    sleep 0.5
done
if [[ ! -e /dev/EtherCAT0 ]]; then
    echo "[restart] /dev/EtherCAT0 did not reappear within 10s — check NIC link" >&2
    echo "[restart] hints:" >&2
    echo "          ip link show              # is the gripper NIC up?" >&2
    echo "          journalctl -u ethercat -n 30" >&2
    exit 1
fi
echo "[restart]      /dev/EtherCAT0 ready."

echo "[restart] 3/4 start ${DAEMON_UNIT} ..."
systemctl start "$DAEMON_UNIT"
sleep 2

echo "[restart] 4/4 verify ..."
echo -n "       ${ECAT_UNIT} : "; systemctl is-active "$ECAT_UNIT"
echo -n "       ${DAEMON_UNIT} : "; systemctl is-active "$DAEMON_UNIT"
echo "       slaves:"
/opt/etherlab/bin/ethercat slaves 2>&1 | sed 's/^/         /'
echo "       port 50053:"
ss -tlnp 2>/dev/null | grep 50053 | sed 's/^/         /' || echo "         (not listening)"

# Functional ping
python3 - <<'PY' || { echo "[restart] FAIL — daemon not responding" >&2; exit 1; }
import socket, struct, sys
s = socket.create_connection(("127.0.0.1", 50053), timeout=3)
s.sendall(bytes([0x01]) + struct.pack("<I", 0))
st = s.recv(1)[0]
plen = struct.unpack("<I", s.recv(4))[0]
reply = s.recv(plen) if plen else b""
s.close()
if reply == b"pong" and st == 0:
    print("       ping ok — daemon healthy.")
    sys.exit(0)
print(f"       ping returned {reply!r} status={st} — unexpected.", file=sys.stderr)
sys.exit(1)
PY

echo "[restart] OK — gripper recovered without reboot."

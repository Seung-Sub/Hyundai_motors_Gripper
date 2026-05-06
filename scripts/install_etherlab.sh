#!/usr/bin/env bash
# install_etherlab.sh — Install IgH EtherLab master (kernel module + userspace tool)
# and configure it for the ART gripper.
#
# Usage:
#   sudo NIC_MAC=b0:38:6c:f1:30:36 ./scripts/install_etherlab.sh
#   sudo NIC_NAME=enxb0386cf13036 ./scripts/install_etherlab.sh   # alternative
#
# Tested on Ubuntu 22.04 with kernel 6.8. Requires sudo (kernel module install).

set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "must run as root (sudo)" >&2
    exit 1
fi

NIC_MAC="${NIC_MAC:-}"
NIC_NAME="${NIC_NAME:-}"
if [[ -z "$NIC_MAC" && -z "$NIC_NAME" ]]; then
    echo "Set NIC_MAC=<mac> or NIC_NAME=<iface>." >&2
    echo "  Find the MAC of the NIC connected to the gripper:  ip link show" >&2
    exit 1
fi
if [[ -z "$NIC_MAC" && -n "$NIC_NAME" ]]; then
    NIC_MAC=$(ip link show "$NIC_NAME" | awk '/link\/ether/ {print $2}')
    [[ -n "$NIC_MAC" ]] || { echo "could not resolve MAC for $NIC_NAME" >&2; exit 1; }
fi

echo "[etherlab] target NIC MAC: $NIC_MAC"
echo

# 1. Build deps
apt-get update -qq
apt-get install -y -qq build-essential cmake autoconf automake libtool libtool-bin \
    pkg-config "linux-headers-$(uname -r)" git curl

# 2. Clone + build IgH master
WORKDIR="/tmp/etherlab-build-$$"
mkdir -p "$WORKDIR"
trap 'rm -rf "$WORKDIR"' EXIT

git clone --depth 1 https://gitlab.com/etherlab.org/ethercat.git "$WORKDIR/ethercat"
cd "$WORKDIR/ethercat"
./bootstrap
./configure \
    --prefix=/opt/etherlab \
    --disable-8139too --disable-eoe \
    --enable-generic \
    --with-linux-dir=/lib/modules/$(uname -r)/build
make all -j"$(nproc)"
make modules -j"$(nproc)"
make install
make modules_install
depmod

# 3. Library path
echo /opt/etherlab/lib > /etc/ld.so.conf.d/etherlab.conf
ldconfig

# 4. systemd unit + ethercatctl
install -m 644 script/ethercat.service /etc/systemd/system/
install -m 755 script/ethercatctl /usr/local/sbin/

# 5. /etc/sysconfig/ethercat AND /opt/etherlab/etc/ethercat.conf (ethercatctl
# reads from the latter; we keep both in sync for distros that use either).
mkdir -p /etc/sysconfig /opt/etherlab/etc
sed \
    -e "s|^MASTER0_DEVICE=.*|MASTER0_DEVICE=\"$NIC_MAC\"|" \
    -e 's|^DEVICE_MODULES=.*|DEVICE_MODULES="generic"|' \
    script/ethercat.conf > /etc/sysconfig/ethercat
cp /etc/sysconfig/ethercat /opt/etherlab/etc/ethercat.conf

# 6. udev rule — make /dev/EtherCAT0 readable by users.
echo 'KERNEL=="EtherCAT[0-9]*", MODE="0666"' > /etc/udev/rules.d/99-EtherCAT.rules
udevadm control --reload-rules

# 7. Enable + start
systemctl daemon-reload
systemctl enable --now ethercat

sleep 2
echo
echo "[etherlab] === verification ==="
systemctl is-active ethercat
ls -la /dev/EtherCAT* 2>/dev/null
/opt/etherlab/bin/ethercat master 2>&1 | head -5
echo
echo "[etherlab] === detected slaves ==="
/opt/etherlab/bin/ethercat slaves
echo
echo "If you see 'NETX 90-RE/ECS' (or similar) above, the gripper EtherCAT chip is detected."
echo "Next: bash scripts/install_daemon.sh"

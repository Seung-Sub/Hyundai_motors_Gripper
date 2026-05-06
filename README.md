# Hyundai Motors ART Gripper — Standalone EtherCAT Daemon

ROS-free reorganisation of the Hyundai Motor Company **ART Gripper** EtherCAT
driver. Original driver (MIT, HMC) shipped as a ROS 2 package; this repo strips
out the ROS layer, keeps the proven EtherCAT + control thread sources, and
exposes the gripper through a small TCP binary protocol that any host language
can drive in <1 ms.

The intended use-case is hooking the gripper into **non-ROS robotics stacks**
(Polymetis, libfranka, your own teleop/data-collection pipelines) without
pulling in `colcon`, `rclcpp`, or DDS.

---

## What this gives you

- **`art_gripper_daemon`** — a single binary that talks EtherCAT to the
  gripper and exposes a TCP server on port `50053`.
- **`art_gripper_client.py`** — Python client whose public surface mirrors
  `polymetis.GripperInterface` (`goto`, `get_state`, `metadata.max_width`,
  plus a libfranka-style `grasp()`). Drop-in replacement in any code that
  already speaks Polymetis.
- **systemd** unit so the daemon starts on boot after EtherCAT is up.
- **Install scripts** that take a fresh Ubuntu 22.04 host and bring the
  gripper online end-to-end.

---

## Hardware

| Item | Value |
|---|---|
| Gripper | Hyundai Motor Company **ART Gripper** (NETX 90-RE/ECS slave) |
| Variant supported here | 2-finger (single open/close DOF). 3-finger / Grasp pose modes are also exposed by the firmware but the Python client pins the rotation DOF at the gripper's current value so you don't accidentally rotate it. |
| Stroke (`finger_width`) | `0..100` mm (0 = mechanical hard close ≈ tip-to-tip; 100 = fully open) |
| Width speed | `1..200` mm/s, default **150 mm/s** |
| Grip force | `1..100` N, default **30 N** — width command targets a position; once the fingers contact something the firmware caps motor current at this force and holds the part. |
| Bus | EtherCAT, IgH master 1.6.x |
| Host NIC | Any 1 GbE port; the install script uses the IgH **generic** Ethernet driver so you don't need a vendor-specific NIC driver (works fine with the USB-Ethernet adapters that pair well with the gripper, e.g. Realtek `r8152`) |

The reference KIST install pairs the daemon with an Intel Core Ultra 5 host
running Ubuntu 22.04 / kernel 6.8 — both the official IgH master and this
daemon are validated on that combo.

---

## Quick start (clean Ubuntu 22.04 host)

```bash
git clone https://github.com/Seung-Sub/Hyundai_motors_Gripper.git
cd Hyundai_motors_Gripper

# 1. Plug the gripper into a dedicated Ethernet port and find its MAC:
ip link show               # note the relevant interface, e.g. enxb0386cf13036

# 2. Install IgH EtherLab master + udev + systemd unit (kernel module compile)
sudo NIC_NAME=enxb0386cf13036 ./scripts/install_etherlab.sh
#   (or pass the MAC directly: sudo NIC_MAC=b0:38:6c:f1:30:36 ...)

# 3. Build + install the daemon
sudo ./scripts/install_daemon.sh --system
#   - compiles ./build/art_gripper_daemon
#   - copies it to /usr/local/bin
#   - registers systemd unit, enables on boot, starts now
#   - pings localhost:50053 to verify

# 4. (optional) install the Python client as a pip package:
pip install ./python
#   — exposes `art_gripper_client` everywhere without sys.path tricks.

# 5. Sanity check: ping + info + status (no motion)
python3 tests/sanity_check.py
# add --motion to do one open/close cycle in air (no object):
python3 tests/sanity_check.py --motion

# 6. Use the client from your code:
python3 - <<'PY'
from art_gripper_client import ArtGripperInterface  # if pip-installed
# or:  import sys; sys.path.insert(0, "python")  # if not
g = ArtGripperInterface()                # auto motor_on
print("max_width:", g.metadata.max_width)
print("state:", g.get_state())
g.goto(width=0.000, speed=0.15, force=30.0, blocking=True)   # close
print("after close:", g.get_state())
g.goto(width=0.100, blocking=True)                            # open
g.motor_off(); g.close()
PY
```

If `install_etherlab.sh`'s slave detection prints `NETX 90-RE/ECS`, the
hardware path is good. If `install_daemon.sh`'s ping prints `b'pong'`, the
TCP layer is good. If `tests/sanity_check.py --motion` finishes with
`OK — air cycle completed cleanly`, the whole stack is healthy.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  application                                                        │
│    Python:  ArtGripperInterface (./python/art_gripper_client.py)    │
│    or any other language speaking the TCP binary protocol           │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ TCP localhost:50053
                           │ length-prefixed binary frames (see protocol.h)
                           ▼
┌─────────────────────────────────────────────────────────────────────┐
│  art_gripper_daemon (systemd-managed)                               │
│  ┌──────────────────┐  ┌───────────────────────────────────────┐    │
│  │  TCP server      │  │  Control thread (1 kHz target)        │    │
│  │  (src/server.cpp)│  │  - reads/writes RobotData via mutex   │    │
│  └────────┬─────────┘  │  - calls EtherCAT M2S/S2M every cycle │    │
│           │ shared     │  (src/thread/, src/device/)           │    │
│           ▼ RobotData  └───────────────────────────────────────┘    │
│  shared mutex-protected struct (include/robotData.h)                │
└──────────────────────────┬──────────────────────────────────────────┘
                           │ /dev/EtherCAT0  (IgH master kernel module)
                           ▼
                ┌──────────────────────┐
                │  ART Gripper (NETX 90)│
                └──────────────────────┘
```

- The **TCP server** mutates the shared `RobotData` exactly the way the
  original ROS service handlers did (see `src/server.cpp`'s op handlers and
  compare with the original `gripper_ecat.cpp`).
- The **control thread** runs the EtherCAT cycle and wakes up every period
  (`ecMaster.cpp`) to push `RobotData.control` onto the slave and read
  `RobotData.status` back.
- Everything below the TCP boundary is the original HMC code, untouched.

---

## TCP binary protocol

```
Request   : [u8 op_code | u32 payload_len_le | bytes payload]
Response  : [u8 status   | u32 payload_len_le | bytes payload]
```

`status == 0` means OK. All multi-byte integers are little-endian.

| op | name | payload (request) | response payload |
|---|---|---|---|
| 0x01 | PING | (empty) | `"pong"` |
| 0x02 | GET_INFO | (empty) | `[u8 nlen][bytes name][u8 vlen][bytes ver][u16 dlen][bytes desc]` |
| 0x03 | GET_STATUS | (empty) | `RespStatus` (see `include/protocol.h`) |
| 0x04 | MOTOR_ON | `u8 on` (0=off, 1=on) | `i32 result` |
| 0x05 | SET_TARGET | `u8 width, u8 pose, u8 w_speed, u8 p_speed, u8 force, u8 sens` | `i32 result` |
| 0x06 | SET_TARGET_WIDTH | `u8 width` | `i32 result` |
| 0x07 | SET_TARGET_POSE | `u8 pose` | `i32 result` |
| 0x08 | SET_GRIPPING_FORCE | `u8 force` | `i32 result` |
| 0x09 | SET_CONTACT_SENSITIVITY | `u8 sens` | `i32 result` |
| 0x0A | RESET_ABS_ENCODER | (empty) | `i32 result` |
| 0x0B | RESET_FRICTION_MODEL | (empty) | `i32 result` |
| 0x0C | SET_TARGET_WIDTH_SPEED | `u8 mm_s` | `i32 result` |
| 0x0D | SET_TARGET_POSE_SPEED | `u8 deg_s` | `i32 result` |
| 0xFF | SHUTDOWN | (empty) | `i32 result` |

Header file — `include/protocol.h` — is the source of truth and includes
the exact struct layouts.

---

## Python API

`art_gripper_client.py` is intentionally a drop-in for `polymetis.GripperInterface`:

```python
from art_gripper_client import ArtGripperInterface

g = ArtGripperInterface(ip_address="127.0.0.1", port=50053)
                                                  # auto motor_on=True

# State
print(g.metadata.max_width)        # 0.1 m
state = g.get_state()
state.width                        # current finger_width in METERS
state.is_grasped                   # CONTACT bit — True if firmware is holding something
state.is_in_motion, state.is_ready, state.is_fault, state.raw_status

# Position move (Polymetis-compatible)
g.goto(width=0.04, speed=0.15, force=30.0, blocking=True)

# Force-based grasp (libfranka grasp() mirror, returns success bool)
ok = g.grasp(width=0.0, speed=0.15, force=30.0)

# Low-level
g.motor_on(settle_s=1.0); g.motor_off()
g.set_pose(deg)            # only if you actually need rotation; usually leave alone
g.reset_abs_encoder()      # required once after boot if you ever change pose
g.reset_friction_model()
g.get_info()               # {name, version, description}
```

### Standard force-based grasp pattern

This is how Franka Hand `grasp()` and Robotiq 2F-85 work, and what this
gripper's firmware also implements:

1. `goto(width=max, …)` — fully open, then approach the object.
2. `goto(width=0, force=N)` — command "close all the way" with force cap N.
3. The firmware closes until either (a) the fingers reach the commanded
   width, or (b) motor current rises to the cap. In case (b) it stops, holds
   the part with current at N, and sets the `CONTACT` status bit.
4. `goto(width=max)` to release.

`width` is the *target*, not a guarantee — `force` is what determines how
hard the gripper holds an object that prevents it from reaching that target.

### Grip force guideline

| Object class | Suggested force (N) |
|---|---|
| Small / thin / fragile (pen, thin cup) | 5–15 |
| Standard rigid object (cube, mug, bottle, ≤ 300 g) | **30 (default)** |
| Large or heavy (≥ 300 g) | 50–80 |
| Soft / deformable (sponge, plush) | 5–10 |

Force is a per-call argument so you can change it from one `goto` to the
next without restarting anything.

---

## Layout

```
.
├── README.md
├── LICENSE.md                 # MIT (HMC original) — applies to whole repo
├── CMakeLists.txt             # one binary, no ROS deps
├── include/
│   ├── protocol.h             # TCP wire format
│   ├── robotData.h            # original — shared control/status struct
│   └── sysData.h              # original — daemon-wide state
├── src/
│   ├── main.cpp               # daemon entry: initDevice + initThread + server
│   ├── server.{h,cpp}         # TCP binary protocol server
│   ├── device/                # original — EtherCAT master + slave PDO map
│   ├── thread/                # original — RT-style control thread
│   └── util/                  # original — terminal helpers (unused but kept
│                              #   so the original sources compile unchanged)
├── python/
│   └── art_gripper_client.py  # Polymetis-compatible Python client
├── systemd/
│   └── art-gripper-daemon.service
└── scripts/
    ├── install_etherlab.sh    # IgH EtherLab from source + ethercat.conf
    └── install_daemon.sh      # cmake build + (optional --system) install
```

The directories under `src/device`, `src/thread`, and `src/util` are
**unmodified** copies of the original HMC sources. The TCP server in
`src/server.cpp` simply mutates the same `RobotData` struct that the ROS
service handlers used to mutate, so the EtherCAT-side behaviour is
bit-identical to the official driver.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `install_etherlab.sh` fails compiling kernel module | Wrong `linux-headers-$(uname -r)`, or you upgraded the kernel after install | `sudo apt install linux-headers-$(uname -r)` and re-run |
| `ethercat slaves` empty | NIC MAC wrong or cable not plugged | `ip link show <iface>`, verify gripper power, then `sudo systemctl restart ethercat` |
| `art_gripper_daemon` start failed in systemd | EtherCAT master not active | `systemctl status ethercat` — must show `active`. If not, fix that first. |
| `art_gripper daemon did not respond to ping` | daemon crashed | `journalctl -u art-gripper-daemon -n 50` |
| First `goto()` after `motor_on()` does nothing | servo init takes ~1 s before commands stick | `motor_on(settle_s=1.0)` (already the default in `art_gripper_client.py`); never send a target inside the first 100 ms after enabling the servo |
| `set_pose()` calls have no effect | Firmware silently ignores pose targets until you've called `reset_abs_encoder()` once after boot | `g.reset_abs_encoder()` — only needed if you really want to rotate. The 2-finger workflow doesn't need this. |
| `goto(width=0)` stops at ~30 mm tip gap | Expected — `width=0` is the firmware's mechanical hard-close, not finger-tip-touch | Spec; not a bug |
| `is_grasped` toggles 0x09 ↔ 0x19 during HOLD | Just the servo-enabled bit (0x10) cycling — readiness/contact bits are unchanged | Cosmetic; ignore |

---

## Credits

- **Original ART Gripper EtherCAT driver**: Hyundai Motor Company RoboticsLab
  (Sangyup "Sean" Yi, MIT). All sources under `src/device`, `src/thread`,
  `src/util`, and `include/{robotData,sysData}.h` are taken from the
  upstream ROS 2 package, unchanged.
- **TCP protocol, Python client, systemd integration, install scripts**:
  added in this fork to deploy the gripper without ROS.

---

## License

MIT — see `LICENSE.md`. Applies to both the upstream HMC sources and the
new code in this fork.

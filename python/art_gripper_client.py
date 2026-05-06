"""ART Gripper Python client.

Talks to the standalone TCP daemon (`art_gripper_daemon`) on localhost:50053
using the binary protocol defined in include/protocol.h.

The public surface is intentionally a drop-in for ``polymetis.GripperInterface``
so franka_env_kist.RobotEnv can swap implementations behind a single env var.

Polymetis-style API exposed:
    g = ArtGripperInterface(ip_address="127.0.0.1", port=50053)
    g.metadata.max_width        # float, meters
    g.get_state().width         # float, meters
    g.goto(width=0.05, speed=0.05, force=10.0, blocking=True)

Internally we convert SI units (meters, m/s, N) to the gripper's native units
(mm, mm/s, N) and mediate motor on/off.

Contact-state (bit 3 of gripper_status) is exposed as ``state.is_grasped``,
matching Polymetis' GripperState.
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

# ---- Protocol (mirror include/protocol.h) ---------------------------------

DEFAULT_PORT = 50053

OP_PING = 0x01
OP_GET_INFO = 0x02
OP_GET_STATUS = 0x03
OP_MOTOR_ON = 0x04
OP_SET_TARGET = 0x05
OP_SET_TARGET_WIDTH = 0x06
OP_SET_TARGET_POSE = 0x07
OP_SET_GRIPPING_FORCE = 0x08
OP_SET_CONTACT_SENSITIVITY = 0x09
OP_RESET_ABS_ENCODER = 0x0A
OP_RESET_FRICTION_MODEL = 0x0B
OP_SET_TARGET_WIDTH_SPEED = 0x0C
OP_SET_TARGET_POSE_SPEED = 0x0D

STATUS_OK = 0

GS_READY = 0x01
GS_FAULT = 0x02
GS_IN_MOTION = 0x04
GS_CONTACT = 0x08

# ART gripper hardware limits.
# KIST runs the 2-finger variant: only the open/close (finger_width) DOF is
# used for grasping. The firmware still exposes a "finger_pose" rotation knob
# but we lock it at the 2-finger preset (180°) so width commands behave like a
# parallel jaw gripper end-to-end.
MAX_WIDTH_MM = 100   # full open
MIN_WIDTH_MM = 0     # firmware 0 == mechanical hard close (~30mm tip gap)
POSE_2FINGER_DEG = 180  # 0=Grasp, 90=3-finger, 180=2-finger ← KIST 운용 고정값
DEFAULT_POSE_DEG = POSE_2FINGER_DEG
# Speeds tuned for binary GR00T gripper actions (open/close every step).
# 150 mm/s closes from 100→0 mm in ~0.66 s, fast enough not to lag the 15 Hz
# control loop while still being safe for human supervision.
DEFAULT_WIDTH_SPEED_MM_S = 150
DEFAULT_POSE_SPEED_DEG_S = 180  # only used if pose is changed manually
# Grip force: width command targets a position; once the fingers contact an
# object the firmware caps motor current at this force, holding the part.
DEFAULT_FORCE_N = 30
DEFAULT_SENSITIVITY = 80

# ---- Public state objects -------------------------------------------------

@dataclass
class _Metadata:
    max_width: float       # meters; 0.100 for ART gripper
    name: str = "ART Gripper"
    version: str = "1.0.0"


@dataclass
class GripperState:
    width: float           # meters (current finger_width converted)
    pose_deg: int          # 0..180 (ART-specific; meaningless for polymetis but useful)
    is_grasped: bool       # GS_CONTACT bit
    is_ready: bool
    is_in_motion: bool
    is_fault: bool
    raw_status: int        # raw bitfield

    # Polymetis compatibility shim — some callers read 'state.is_grasped' only.


# ---- Client ---------------------------------------------------------------

class _Conn:
    """Thread-safe single TCP connection wrapper with simple binary RPC."""

    def __init__(self, host: str, port: int, timeout: float = 2.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._reconnect()

    def _reconnect(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
        s = socket.create_connection((self._host, self._port), timeout=self._timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = s

    def _recv_n(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("daemon closed connection")
            buf += chunk
        return buf

    def call(self, op: int, payload: bytes = b"") -> bytes:
        with self._lock:
            try:
                self._sock.sendall(bytes([op]) + struct.pack("<I", len(payload)) + payload)
                status = self._recv_n(1)[0]
                plen = struct.unpack("<I", self._recv_n(4))[0]
                pl = self._recv_n(plen) if plen else b""
            except (BrokenPipeError, ConnectionError, OSError):
                # one-shot reconnect & retry
                self._reconnect()
                self._sock.sendall(bytes([op]) + struct.pack("<I", len(payload)) + payload)
                status = self._recv_n(1)[0]
                plen = struct.unpack("<I", self._recv_n(4))[0]
                pl = self._recv_n(plen) if plen else b""
            if status != STATUS_OK:
                raise RuntimeError(f"art_gripper daemon error: status={status}")
            return pl

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except Exception:
                    pass
                self._sock = None


class ArtGripperInterface:
    """Drop-in for ``polymetis.GripperInterface`` for the ART Gripper.

    Args:
        ip_address: daemon host. Defaults to localhost since the daemon owns
            /dev/EtherCAT0 on the same machine.
        port: TCP port the daemon listens on (default 50053).
        auto_motor_on: send MOTOR_ON during construction so callers don't have
            to. Set False if you want to manage servo lifecycle externally.
        default_speed_mm_s: width speed used when ``goto`` is called without
            an explicit speed (or with the legacy normalised speed value 0.05).
        default_force: gripping force in N used by ``goto`` defaults.
    """

    def __init__(
        self,
        ip_address: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        auto_motor_on: bool = True,
        default_speed_mm_s: int = DEFAULT_WIDTH_SPEED_MM_S,
        default_force: int = DEFAULT_FORCE_N,
        default_pose_deg: Optional[int] = None,
        default_pose_speed_deg_s: int = DEFAULT_POSE_SPEED_DEG_S,
        default_sensitivity: int = DEFAULT_SENSITIVITY,
    ):
        self._conn = _Conn(ip_address, port)
        self._default_speed = int(default_speed_mm_s)
        self._default_force = int(default_force)
        self._default_pose_speed = int(default_pose_speed_deg_s)
        self._default_sensitivity = int(default_sensitivity)

        # Polymetis-style metadata — required by franka_env_kist.RobotEnv.__init__.
        self.metadata = _Metadata(max_width=MAX_WIDTH_MM / 1000.0)

        # Sanity ping — surfaces wire issues immediately.
        if self._conn.call(OP_PING) != b"pong":
            raise RuntimeError("art_gripper daemon did not respond to ping")

        # Pin the rotational DOF at the gripper's current physical pose so we
        # never accidentally command a rotation. For the KIST 2-finger build
        # the value is whatever the hardware reports at boot; this lets every
        # goto() call replay the same pose value and the firmware stays put.
        if default_pose_deg is None:
            try:
                cur = self.get_state().pose_deg
                self._default_pose = int(cur)
            except Exception:
                self._default_pose = 0
        else:
            self._default_pose = int(default_pose_deg)

        if auto_motor_on:
            self.motor_on()

    # ---------- Polymetis-style API ----------

    def get_state(self) -> GripperState:
        pl = self._conn.call(OP_GET_STATUS)
        gs, fw, fp, _pad = struct.unpack_from("<BBBB", pl, 0)
        return GripperState(
            width=fw / 1000.0,
            pose_deg=fp,
            is_grasped=bool(gs & GS_CONTACT),
            is_ready=bool(gs & GS_READY),
            is_in_motion=bool(gs & GS_IN_MOTION),
            is_fault=bool(gs & GS_FAULT),
            raw_status=gs,
        )

    def goto(
        self,
        width: float,
        speed: float = 0.15,
        force: float = 30.0,
        blocking: bool = True,
        pose_deg: Optional[int] = None,
    ) -> None:
        """Move to ``width`` meters at ``speed`` m/s with grip ``force`` N.

        Defaults are tuned for KIST 2-finger usage: 150 mm/s travel speed and
        30 N grip force. ``force`` becomes a current cap once the fingers
        touch an object — the firmware holds the part at this force without
        further width command.

        ``pose_deg`` is intentionally ignored unless the caller overrides it;
        we lock the rotational DOF to the 2-finger preset (180°) so the
        gripper behaves as a parallel jaw end-to-end.
        """
        width_mm = max(MIN_WIDTH_MM, min(MAX_WIDTH_MM, int(round(float(width) * 1000.0))))
        # Convert m/s to mm/s. If a caller still passes the legacy Polymetis
        # default 0.05 m/s = 50 mm/s, it stays inside the 1..200 mm/s range.
        speed_mm_s = int(round(float(speed) * 1000.0))
        if speed_mm_s <= 0:
            speed_mm_s = self._default_speed
        speed_mm_s = max(1, min(200, speed_mm_s))
        force_n = max(1, min(100, int(round(float(force)))))
        pose = self._default_pose if pose_deg is None else int(pose_deg)
        pose = max(0, min(180, pose))

        payload = struct.pack(
            "<BBBBBB",
            width_mm,
            pose,
            speed_mm_s,
            self._default_pose_speed,
            force_n,
            self._default_sensitivity,
        )
        self._conn.call(OP_SET_TARGET, payload)
        if blocking:
            self._wait_until_idle()

    def _wait_until_idle(self, timeout_s: float = 5.0, poll_s: float = 0.05,
                         min_settle_s: float = 0.2) -> None:
        """Wait until the firmware reports the motion completed.

        The firmware needs ~100-200 ms to register a new SET_TARGET and flip
        IN_MOTION. Polling too aggressively right after the command can race
        and observe the *previous* idle state. We therefore (1) sleep
        ``min_settle_s`` before the first poll, and (2) require IN_MOTION to
        have actually risen at least once before declaring the motion done.
        """
        time.sleep(min_settle_s)
        start = time.monotonic()
        saw_motion = False
        while time.monotonic() - start < timeout_s:
            st = self.get_state()
            if st.is_fault:
                raise RuntimeError("art_gripper fault during motion")
            if st.is_in_motion:
                saw_motion = True
            elif saw_motion and st.is_ready:
                return  # motion observed and now finished
            time.sleep(poll_s)
        # Timeout: silent exit (matches Polymetis behaviour); caller can poll
        # state if it wants stricter guarantees.

    # ---------- libfranka-style grasp ----------

    def grasp(
        self,
        width: float = 0.0,
        speed: float = 0.15,
        force: float = 30.0,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
        timeout_s: float = 5.0,
    ) -> bool:
        """Force-based grasp matching ``libfranka``'s ``Gripper::grasp`` API.

        Sends a CLOSE command (target ``width`` in meters, default fully closed)
        with the supplied force cap, then watches the status stream:

        - **Success (returns True)**: CONTACT bit rises → firmware is holding
          the object at ``force`` N. The current finger_width may not equal
          the commanded value; if it lies within ``[width - epsilon_outer,
          width + epsilon_inner]`` we still call it a success (Franka-style
          tolerance).
        - **No contact (returns False)**: motion ended without CONTACT — the
          fingers reached the commanded width through air. Caller may treat as
          "no object grabbed".

        ``goto(blocking=True)`` already covers the common case where the
        higher layer doesn't care about the contact distinction; ``grasp`` is
        the explicit form for code that wants to branch on grip success.
        """
        target_mm = max(MIN_WIDTH_MM, min(MAX_WIDTH_MM, int(round(width * 1000.0))))
        speed_mm_s = max(1, min(200, int(round(speed * 1000.0))))
        force_n = max(1, min(100, int(round(force))))
        payload = struct.pack(
            "<BBBBBB",
            target_mm,
            self._default_pose,
            speed_mm_s,
            self._default_pose_speed,
            force_n,
            self._default_sensitivity,
        )
        self._conn.call(OP_SET_TARGET, payload)

        # Mirror _wait_until_idle but also remember whether CONTACT ever fired.
        time.sleep(0.2)  # firmware register delay
        start = time.monotonic()
        saw_motion = False
        while time.monotonic() - start < timeout_s:
            st = self.get_state()
            if st.is_fault:
                raise RuntimeError("art_gripper fault during grasp")
            if st.is_in_motion:
                saw_motion = True
            elif saw_motion and st.is_ready:
                # Motion ended. Decide success.
                if st.is_grasped:
                    return True
                # Reached target width through air → check tolerance.
                cur_mm = int(round(st.width * 1000.0))
                inner_mm = int(round(epsilon_inner * 1000.0))
                outer_mm = int(round(epsilon_outer * 1000.0))
                if (target_mm - outer_mm) <= cur_mm <= (target_mm + inner_mm):
                    return False  # closed to target — nothing grabbed
                return False
            time.sleep(0.05)
        return False  # timeout — no grip confirmed

    # ---------- ART-specific helpers ----------

    def motor_on(self, settle_s: float = 1.0) -> None:
        """Enable the servo and wait briefly for the firmware to register READY.

        The 1 s settle time matches the ROS daemon's observed latency between
        SERVO_SWITCH_BIT being set and the gripper actually responding to
        target commands. Skipping it makes the very first goto() silently
        miss because the firmware ignores commands while still initialising.
        """
        self._conn.call(OP_MOTOR_ON, bytes([1]))
        if settle_s > 0:
            time.sleep(settle_s)

    def motor_off(self) -> None:
        self._conn.call(OP_MOTOR_ON, bytes([0]))

    def set_pose(self, pose_deg: int, speed_deg_s: Optional[int] = None,
                 blocking: bool = True) -> None:
        """Rotate the finger arrangement (0=Grasp, 90=3-finger, 180=2-finger).

        KIST runs the 2-finger variant — leave this alone for normal operation.
        Pose rotation also requires a one-time ``reset_abs_encoder()`` after
        boot before it will execute; otherwise the firmware silently ignores
        pose targets.
        """
        pose = max(0, min(180, int(pose_deg)))
        if speed_deg_s is not None:
            self._conn.call(OP_SET_TARGET_POSE_SPEED, bytes([max(1, min(255, int(speed_deg_s)))]))
        self._conn.call(OP_SET_TARGET_POSE, bytes([pose]))
        if blocking:
            self._wait_until_idle()

    def reset_abs_encoder(self) -> None:
        self._conn.call(OP_RESET_ABS_ENCODER)

    def reset_friction_model(self) -> None:
        self._conn.call(OP_RESET_FRICTION_MODEL)

    def get_info(self) -> dict:
        pl = self._conn.call(OP_GET_INFO)
        nlen = pl[0]
        name = pl[1:1 + nlen].decode()
        i = 1 + nlen
        vlen = pl[i]
        ver = pl[i + 1:i + 1 + vlen].decode()
        i += 1 + vlen
        dlen = pl[i] | (pl[i + 1] << 8)
        desc = pl[i + 2:i + 2 + dlen].decode()
        return {"name": name, "version": ver, "description": desc}

    # ---------- Lifecycle ----------

    def close(self) -> None:
        try:
            self.motor_off()
        except Exception:
            pass
        self._conn.close()


__all__ = ["ArtGripperInterface", "GripperState"]

#!/usr/bin/env python3
"""
morph_bridge.py — WebSocket <-> MAVLink bridge for laptop-commanded morphing.

  *** LAPTOP-DRIVEN SWEEP VERSION (fmuv3 / no-Lua boards) ***

WHY THIS VERSION:
    The original design ran the morph sweep in an onboard Lua script and the
    laptop just wrote one parameter. That requires an H7/F7 board with spare
    RAM for scripting. This airframe's flight controller is an fmuv3 (F4,
    Pixhawk-1 class) whose ArduCopter 4.4.4 build cannot allocate a Lua heap
    ("Scripting: Unable to allocate memory"), so scripting is unavailable.

    Instead, the SWEEP RUNS HERE on the laptop: the bridge interpolates the
    four servo angles over ~3 s and streams MAV_CMD_DO_SET_SERVO to the FC.
    The FC just applies each commanded PWM. No onboard script needed.

ROLE SPLIT (unchanged):
    * The RC transmitter flies the drone with full authority at all times.
    * This bridge ONLY moves the morph servos (outputs 9-12). It never sends
      flight control (no MANUAL_CONTROL, no RC override, no setpoints).

TRADEOFF vs the Lua version (be honest about this):
    * The interpolation packets now travel over the link. On a tethered USB
      connection that's a non-issue. Over the SiK radio, a mid-sweep dropout
      can stall a morph partway, so this bridge WATCHES the link and HALTS a
      sweep if telemetry stalls, holding the last commanded position.
    * The RC gate is now enforced in SOFTWARE here (reading the gate channel
      from RC_CHANNELS), not by the FC. It is therefore only as trustworthy
      as this bridge + link. For props-on flight, add a hardware interlock on
      servo power; do not rely on this software gate alone.

FC-SIDE PARAMETER SETUP (Mission Planner -> Servo Output):
    Outputs 9-12 (AUX 1-4) -> Function = Disabled (0), so DO_SET_SERVO can
    drive them directly. Min 1000 / Max 2000 / Trim 1500.
    (Do NOT set them to Script1-4; that path is gone with Lua.)

Shape index: 0 = X, 1 = O, 2 = T, 3 = H
"""

import argparse
import asyncio
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, asdict

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed.  pip install pymavlink websockets pyserial")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("ERROR: websockets not installed.  pip install pymavlink websockets pyserial")
    sys.exit(1)


# ---------------------------------------------------------------------------
#  Shape definitions - LOCKED canonical set.
#  Arm order [FR, FL, RL, RR]; base headings 45/135/225/315 deg.
#  Angles are outer-segment deflection from radial-outward, +CCW, limits +/-90.
# ---------------------------------------------------------------------------
SHAPES = {
    0: {"name": "X", "ang": [0.0, 0.0, 0.0, 0.0],
        "desc": "Baseline quad, zero CG offset"},
    1: {"name": "O", "ang": [90.0, 90.0, 90.0, 90.0],
        "desc": "Compact pinwheel, zero CG offset"},
    2: {"name": "T", "ang": [-45.0, 45.0, 45.0, -45.0],
        "desc": "Asymmetric, 79.5mm CG offset"},
    3: {"name": "H", "ang": [45.0, 45.0, -45.0, -45.0],
        "desc": "Parallel bars, roll-biased"},
}
NAME_TO_INDEX = {v["name"]: k for k, v in SHAPES.items()}

# Which FC servo OUTPUT each arm is wired to (AUX 1-4 = outputs 9-12).
# DO_SET_SERVO addresses outputs by this 1-based number.
SERVO_OUTPUT = {0: 9, 1: 10, 2: 11, 3: 12}   # FR, FL, RL, RR

# Sweep timing.
SWEEP_TIME_S = 3.0
SWEEP_HZ = 20                 # servo update rate during a sweep
SWEEP_STEP_S = 1.0 / SWEEP_HZ

# PWM mapping - matches the confirmed 1500=X(0deg) / 2000=O(+90deg) calibration.
def angle_to_pwm(deg):
    deg = max(-90.0, min(90.0, deg))
    return int(round(1500 + (deg / 90.0) * 500))

# Allowed flight modes for an IN-FLIGHT morph (bench mode bypasses this).
ALLOWED_MODES = {"ALT_HOLD", "STABILIZE"}

# RC gate (software-enforced here).
RC_GATE_CHANNEL = 7          # RC channel index (1-based) for the enable switch
RC_GATE_HIGH_US = 1700       # PWM at/above this = "morph enabled"

# Safety envelope thresholds (in-flight only).
MIN_ALT_M = 1.5
MAX_TILT_DEG = 25.0

# Link-health: if no telemetry heartbeat within this window, treat link as bad.
LINK_STALL_S = 1.5


@dataclass
class Telemetry:
    armed: bool = False
    mode: str = "UNKNOWN"
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    alt_m: float = 0.0
    batt_v: float = 0.0
    batt_pct: float = -1.0
    txbuf_pct: int = 100
    gate_pwm: int = 0            # last seen PWM on the gate channel
    current_shape: str = "X"
    morph_status: str = "idle"  # idle/morphing/complete/rejected_*/halted
    morph_progress: int = 0
    gate_active: bool = True     # software gate enforcement on/off
    link_ok: bool = False
    last_heartbeat_age: float = 999.0


class MorphBridge:
    def __init__(self, connection_str, baud, gate_active=True, bench=False):
        self.connection_str = connection_str
        self.baud = baud
        self.master = None
        self.tlm = Telemetry()
        self.tlm.gate_active = gate_active
        self.bench = bench          # bench mode: skip in-flight envelope checks
        self._lock = threading.Lock()
        self._running = False
        self._last_hb_time = 0.0
        self._sweep_lock = threading.Lock()   # only one sweep at a time
        self._current_idx = 0
        self.source_system = 255

    # -- connection ---------------------------------------------------------
    def connect(self):
        print(f"[mavlink] Connecting to {self.connection_str} ...")
        if self.connection_str.startswith(("udp:", "tcp:", "udpin:", "udpout:")):
            self.master = mavutil.mavlink_connection(
                self.connection_str, source_system=self.source_system)
        else:
            self.master = mavutil.mavlink_connection(
                self.connection_str, baud=self.baud,
                source_system=self.source_system)
        print("[mavlink] Waiting for heartbeat ...")
        self.master.wait_heartbeat(timeout=30)
        if self.master.target_system == 0:
            raise RuntimeError("timeout waiting for heartbeat")
        print(f"[mavlink] Connected (sys:{self.master.target_system} "
              f"comp:{self.master.target_component})")
        self._request_streams()

    def _request_streams(self):
        # Trim rates for the SiK link; we need attitude, status, and RC (for
        # the gate channel). We are NOT flying from the laptop.
        streams = [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 5),          # attitude
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 3),          # VFR_HUD
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2), # sys/gps
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2),
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 3),     # for gate
            (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, 0),
        ]
        for sid, rate in streams:
            self.master.mav.request_data_stream_send(
                self.master.target_system, self.master.target_component,
                sid, rate, 1 if rate > 0 else 0)
        print("[mavlink] Stream rates trimmed for SiK link")

    # -- background reader --------------------------------------------------
    def start_reader(self):
        self._running = True
        threading.Thread(target=self._reader_loop, daemon=True).start()

    def _reader_loop(self):
        while self._running:
            msg = self.master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            self._ingest(msg)

    def _ingest(self, msg):
        mtype = msg.get_type()
        with self._lock:
            t = self.tlm
            if mtype == "HEARTBEAT":
                self._last_hb_time = time.time()
                t.armed = bool(msg.base_mode &
                               mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    t.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    pass
            elif mtype == "ATTITUDE":
                t.roll_deg = math.degrees(msg.roll)
                t.pitch_deg = math.degrees(msg.pitch)
                t.yaw_deg = math.degrees(msg.yaw)
            elif mtype == "VFR_HUD":
                t.alt_m = msg.alt
            elif mtype == "GLOBAL_POSITION_INT":
                t.alt_m = msg.relative_alt / 1000.0
            elif mtype == "SYS_STATUS":
                t.batt_v = msg.voltage_battery / 1000.0
                t.batt_pct = msg.battery_remaining
            elif mtype == "RADIO_STATUS":
                t.txbuf_pct = msg.txbuf
            elif mtype in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                ch = getattr(msg, f"chan{RC_GATE_CHANNEL}_raw", 0)
                t.gate_pwm = ch

    def heartbeat_age(self):
        return time.time() - self._last_hb_time if self._last_hb_time else 999.0

    def link_ok(self):
        return self.heartbeat_age() < LINK_STALL_S

    def send_gcs_heartbeat(self):
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    # -- low-level servo command -------------------------------------------
    def _set_servo(self, output_num, pwm):
        """Send MAV_CMD_DO_SET_SERVO for one output."""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            float(output_num), float(pwm), 0, 0, 0, 0, 0)

    def _drive_all(self, ang):
        """Drive all four morph servos to an angle table [FR,FL,RL,RR]."""
        for i in range(4):
            self._set_servo(SERVO_OUTPUT[i], angle_to_pwm(ang[i]))

    # -- safety envelope (checked before a sweep starts) -------------------
    def check_preconditions(self, target_idx):
        """Return (ok, reason). Bench mode skips flight checks."""
        with self._lock:
            t = self.tlm
            gate_active = t.gate_active
            armed = t.armed
            mode = t.mode
            alt = t.alt_m
            roll = t.roll_deg
            pitch = t.pitch_deg
            gate_pwm = t.gate_pwm

        # Software RC gate: if active, require the switch high.
        if gate_active:
            if gate_pwm < RC_GATE_HIGH_US:
                return False, "rejected_rc_gate"

        # Link must be healthy to even start.
        if not self.link_ok():
            return False, "rejected_link"

        # Bench mode = skip flight envelope so arms can be exercised on a table.
        bench_now = self.bench and (not armed)
        if bench_now:
            return True, None

        # In-flight envelope.
        if not armed:
            return False, "rejected_disarmed"
        if mode not in ALLOWED_MODES:
            return False, "rejected_mode"
        if alt < MIN_ALT_M:
            return False, "rejected_altitude"
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0,
                math.cos(math.radians(roll)) * math.cos(math.radians(pitch))))))
        if tilt > MAX_TILT_DEG:
            return False, "rejected_tilt"
        return True, None

    # -- the sweep engine (runs in a worker thread) ------------------------
    def run_sweep(self, target_idx, on_progress=None):
        """
        Interpolate from current shape to target over SWEEP_TIME_S, streaming
        DO_SET_SERVO at SWEEP_HZ. Halts (holding last position) if the link
        stalls mid-sweep. Returns (status_str).
        """
        if not self._sweep_lock.acquire(blocking=False):
            return "busy"
        try:
            ok, reason = self.check_preconditions(target_idx)
            if not ok:
                with self._lock:
                    self.tlm.morph_status = reason
                return reason

            from_ang = list(SHAPES[self._current_idx]["ang"])
            to_ang = list(SHAPES[target_idx]["ang"])
            with self._lock:
                self.tlm.morph_status = "morphing"
                self.tlm.morph_progress = 0

            steps = int(SWEEP_TIME_S * SWEEP_HZ)
            for s in range(steps + 1):
                # Abort if the link stalls partway - hold last commanded pos.
                if not self.link_ok():
                    with self._lock:
                        self.tlm.morph_status = "halted_link"
                    return "halted_link"
                # Abort if the gate is dropped mid-sweep.
                if self.tlm.gate_active and self.tlm.gate_pwm < RC_GATE_HIGH_US:
                    with self._lock:
                        self.tlm.morph_status = "halted_gate"
                    return "halted_gate"

                frac = s / steps
                ang = [from_ang[i] + (to_ang[i] - from_ang[i]) * frac
                       for i in range(4)]
                self._drive_all(ang)
                pct = int(frac * 100)
                with self._lock:
                    self.tlm.morph_progress = pct
                if on_progress:
                    on_progress(pct)
                time.sleep(SWEEP_STEP_S)

            self._current_idx = target_idx
            with self._lock:
                self.tlm.current_shape = SHAPES[target_idx]["name"]
                self.tlm.morph_status = "complete"
                self.tlm.morph_progress = 100
            return "complete"
        finally:
            self._sweep_lock.release()

    def snapshot(self):
        with self._lock:
            self.tlm.last_heartbeat_age = self.heartbeat_age()
            self.tlm.link_ok = self.tlm.last_heartbeat_age < LINK_STALL_S
            self.tlm.current_shape = SHAPES[self._current_idx]["name"]
            return asdict(self.tlm)


# ---------------------------------------------------------------------------
#  WebSocket layer
# ---------------------------------------------------------------------------
async def telemetry_pusher(ws, bridge, hz=4):
    interval = 1.0 / hz
    try:
        while True:
            await ws.send(json.dumps({"type": "telemetry",
                                      "data": bridge.snapshot()}))
            await asyncio.sleep(interval)
    except websockets.exceptions.ConnectionClosed:
        pass


async def heartbeat_task(bridge, hz=1):
    interval = 1.0 / hz
    while True:
        try:
            bridge.send_gcs_heartbeat()
        except Exception as e:
            print(f"[hb] {e}")
        await asyncio.sleep(interval)


async def handle_client(ws, bridge):
    print("[ws] Client connected")
    pusher = asyncio.create_task(telemetry_pusher(ws, bridge))
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send(json.dumps({"type": "error", "error": "bad JSON"}))
                continue
            cmd = msg.get("cmd")

            if cmd == "morph":
                shape = msg.get("shape")
                idx = NAME_TO_INDEX.get(shape.upper()) if isinstance(shape, str) else shape
                if idx not in SHAPES:
                    await ws.send(json.dumps({"type": "error",
                        "error": f"unknown shape {shape}"}))
                    continue

                async def progress_cb(pct):
                    pass  # progress is already in the telemetry snapshot

                # Run the blocking sweep in a worker thread so the WS loop
                # stays responsive and telemetry keeps flowing during the morph.
                result = await asyncio.to_thread(bridge.run_sweep, idx)
                await ws.send(json.dumps({
                    "type": "morph_result",
                    "shape": SHAPES[idx]["name"],
                    "shape_index": idx,
                    "result": result,
                }))
                print(f"[morph] -> {SHAPES[idx]['name']}: {result}")

            elif cmd == "set_gate":
                active = bool(msg.get("active", True))
                with bridge._lock:
                    bridge.tlm.gate_active = active
                await ws.send(json.dumps({"type": "gate_ack", "active": active}))
                print(f"[gate] software gate {'ACTIVE' if active else 'DISABLED'}")

            elif cmd == "stop":
                # Best-effort: flip gate check by forcing status; the running
                # sweep checks link/gate each step. For a hard stop, drop gate.
                with bridge._lock:
                    bridge.tlm.gate_active = True
                await ws.send(json.dumps({"type": "stop_ack"}))

            elif cmd == "shapes":
                await ws.send(json.dumps({"type": "shapes", "data": SHAPES}))

            elif cmd == "ping":
                await ws.send(json.dumps({"type": "pong"}))

            else:
                await ws.send(json.dumps({"type": "error",
                    "error": f"unknown command: {cmd}"}))
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        pusher.cancel()
        print("[ws] Client disconnected")


async def main():
    p = argparse.ArgumentParser(description="Morph bridge (laptop-driven sweep)")
    p.add_argument("--mavlink", default="udp:127.0.0.1:14550",
                   help="MAVLink conn: udp:127.0.0.1:14550 (SITL) or COM3 / /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--ws-port", type=int, default=8765)
    p.add_argument("--no-gate", action="store_true",
                   help="disable the software RC gate (bench convenience)")
    p.add_argument("--bench", action="store_true",
                   help="bench mode: skip in-flight envelope checks when disarmed")
    args = p.parse_args()

    bridge = MorphBridge(args.mavlink, args.baud,
                         gate_active=not args.no_gate, bench=args.bench)
    try:
        bridge.connect()
    except Exception as e:
        print(f"[mavlink] Connection failed: {e}")
        sys.exit(1)
    bridge.start_reader()

    asyncio.create_task(heartbeat_task(bridge))
    mode = []
    if args.bench:
        mode.append("BENCH")
    if args.no_gate:
        mode.append("GATE-OFF")
    print(f"[cfg] {' '.join(mode) if mode else 'FLIGHT gate-on'}")
    print(f"[ws] Listening on ws://localhost:{args.ws_port}")
    async with websockets.serve(
            lambda ws: handle_client(ws, bridge), "localhost", args.ws_port):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[bridge] Shutting down")

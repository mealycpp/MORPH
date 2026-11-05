#!/usr/bin/env python3

import argparse
import asyncio
import json
import sys
import time

import websockets
from pymavlink import mavutil


class MavlinkBridge:
    def __init__(self, mavlink_conn, baud):
        self.mavlink_conn = mavlink_conn
        self.baud = baud
        self.master = None
        self.current_values = {}

    def connect(self):
        print(f"[mavlink] Connecting to {self.mavlink_conn}...")
        self.master = mavutil.mavlink_connection(self.mavlink_conn, baud=self.baud)

        print("[mavlink] Waiting for heartbeat...")
        hb = self.master.wait_heartbeat(timeout=15)
        if hb is None:
            raise RuntimeError("No heartbeat received")

        print(
            f"[mavlink] Connected "
            f"(sys:{self.master.target_system} comp:{self.master.target_component})"
        )
        return True

    def _clean_param_id(self, param_id):
        if isinstance(param_id, bytes):
            param_id = param_id.decode("utf-8", errors="ignore")
        return param_id.rstrip("\x00")

    def get_param(self, name):
        print(f"[param] Request read: {name}")
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("utf-8"),
            -1,
        )

        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if not msg:
                continue

            param_id = self._clean_param_id(msg.param_id)
            print(f"[param] RX PARAM_VALUE: {param_id}={msg.param_value}")

            if param_id == name:
                val = msg.param_value
                self.current_values[name] = val
                return val

        print(f"[param] Timeout reading {name}")
        return None

    def set_param(self, name, value):
        old_value = self.get_param(name)
        if old_value is None:
            old_value = self.current_values.get(name, "?")

        print(f"[param] Setting {name} -> {value}")
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("utf-8"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )

        deadline = time.time() + 3.0
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if not msg:
                continue

            param_id = self._clean_param_id(msg.param_id)
            print(f"[param] RX PARAM_VALUE after set: {param_id}={msg.param_value}")

            if param_id == name:
                new_val = msg.param_value
                self.current_values[name] = new_val
                return True, old_value, new_val

        print(f"[param] Timeout setting {name}")
        return False, old_value, None


async def handle_client(ws, bridge):
    print("[ws] Client connected")
    await ws.send(json.dumps({"type": "status", "message": "Bridge ready"}))

    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send(json.dumps({"type": "error", "error": "Invalid JSON"}))
                continue

            cmd = msg.get("cmd")

            if cmd == "ping":
                await ws.send(json.dumps({
                    "type": "status",
                    "message": f"Bridge OK — ArduPilot sys:{bridge.master.target_system}"
                }))

            elif cmd == "test_connection":
                try:
                    val = await asyncio.to_thread(bridge.get_param, "FRAME_CLASS")
                    if val is not None:
                        await ws.send(json.dumps({
                            "type": "status",
                            "message": f"Link OK (FRAME_CLASS={val})"
                        }))
                    else:
                        await ws.send(json.dumps({
                            "type": "error",
                            "error": "No response from ArduPilot"
                        }))
                except Exception as e:
                    await ws.send(json.dumps({"type": "error", "error": f"Test failed: {e}"}))

            elif cmd == "set_param":
                param = msg.get("param")
                value = msg.get("value")
                try:
                    success, old_val, new_val = await asyncio.to_thread(
                        bridge.set_param, param, value
                    )
                    await ws.send(json.dumps({
                        "type": "param_ack",
                        "param": param,
                        "oldValue": old_val,
                        "newValue": new_val if new_val is not None else value,
                        "status": "ok" if success else "timeout",
                    }))
                except Exception as e:
                    await ws.send(json.dumps({"type": "error", "error": f"set_param failed: {e}"}))

            elif cmd == "backup_params":
                params = msg.get("params", [])

                await ws.send(json.dumps({
                    "type": "status",
                    "message": f"Reading {len(params)} current values..."
                }))

                backup = {}

                for p in params:
                    val = await asyncio.to_thread(bridge.get_param, p)
                    if val is not None:
                        backup[p] = val

                # ✅ Save to .param file
                filename = f"param_backup_{int(time.time())}.param"
                with open(filename, "w") as f:
                    for param, value in backup.items():
                        f.write(f"{param}\t{value}\n")

                print(f"[backup] Saved {len(backup)} params to {filename}")

                await ws.send(json.dumps({
                    "type": "status",
                    "message": f"Backup complete: {len(backup)} values read"
                }))

                await ws.send(json.dumps({
                    "type": "backup",
                    "data": backup
                }))
            
            else:
                await ws.send(json.dumps({"type": "error", "error": f"Unknown command: {cmd}"}))

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print("[ws] Client disconnected")


async def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mavlink", default="udp:0.0.0.0:14550")
    p.add_argument("--baud", type=int, default=57600)
    p.add_argument("--ws-port", type=int, default=8765)
    args = p.parse_args()

    bridge = MavlinkBridge(args.mavlink, args.baud)
    bridge.connect()

    print(f"[ws] Listening on ws://localhost:{args.ws_port}")
    async with websockets.serve(lambda ws: handle_client(ws, bridge), "localhost", args.ws_port):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[bridge] Shutting down")
#!/usr/bin/env python3
"""
HAB Telemetry Generator

Sends three APID streams to the Modulator Service:
  APID 0 — TimeSync (raw 8 bytes, every 10s, realtime)
  APID 1 — CoreTelemetry (ASN.1 UPER encoded, every 2s, high priority)
  APID 2 — CBOR {"name":"test-payload"} (every 5s, normal priority)

Usage:
  python telemetry_generator.py [--host HOST] [--port PORT]
"""

import argparse
import asn1tools
import cbor2
import socket
import struct
import sys
import time
import random
import math
from datetime import datetime, timezone
import json
import uuid

HAS_WEBSOCKET = False
try:
    import websocket
    HAS_WEBSOCKET = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# ASN.1 schema (embedded)
# ---------------------------------------------------------------------------
ASN1_SCHEMA = """
HAB-Telemetry DEFINITIONS AUTOMATIC TAGS ::=
BEGIN

-- APID 0: Time Sync Telemetry
APID0-TimeSync ::= SEQUENCE {
    baseTime  INTEGER (0..4294967295), -- 32-bit UNIX timestamp (seconds since epoch)
    baseTicks INTEGER (0..4294967295)  -- 32-bit system uptime in 10ms ticks
}

-- APID 1: Core Telemetry (UPER encoded)
APID1-CoreTelemetry ::= SEQUENCE {
    callsign     IA5String (FROM ("A".."Z" | "0".."9") ^ SIZE (4..6)),
    timeOffset   INTEGER (0..65535),      -- 16-bit relative counter (10ms resolution)
    latitude     INTEGER (-900000000..900000000), -- Latitude * 10^7 (-90 to +90 deg)
    longitude    INTEGER (-1800000000..1800000000), -- Longitude * 10^7 (-180 to +180 deg)
    altitude     INTEGER (0..65535),      -- Altitude in meters (0 to 65535)
    gpsSats      INTEGER (0..31),         -- Number of active satellites (0 to 31)
    gpsLock      GPSLockState,            -- GPS lock state (none, 2D, 3D, differential)
    battVoltage  INTEGER (0..3000),       -- Battery voltage in 10mV units (0 to 30.00V, 12 bits)
    tempInternal INTEGER (-500..1000),    -- Internal temp in 0.1 degC (-50.0 C to +100.0 C)
    tempExternal INTEGER (-1000..500)     -- External temp in 0.1 degC (-100.0 C to +50.0 C)
}

GPSLockState ::= ENUMERATED {
    none(0),
    fix2d(1),
    fix3d(2),
    diff(3)
}

END
"""

# Compile ASN.1 once at module level
_asn1_spec = asn1tools.compile_string(ASN1_SCHEMA, codec='uper')

# ---------------------------------------------------------------------------
# Telemetry state — evolves each cycle to simulate a real balloon
# ---------------------------------------------------------------------------
class TelemetryState:
    """Holds simulated telemetry values that drift over time."""

    def __init__(self, start_time: float):
        self.start_time = start_time
        # Fixed callsign base (will be SP0KS most of the time)
        self.callsign_base = "SP0KS"

        # Starting position: central Poland (approximate)
        self.latitude = 521500000       # 52.15 N * 10^7
        self.longitude = 210000000      # 21.00 E * 10^7
        self.altitude = 500             # 500 meters

        # GPS state
        self.gps_sats = 8
        self.gps_lock = 2              # 3D fix
        self.lock_counter = 0

        # Power / temperature
        self.batt_voltage = 2500        # 25.00 V
        self.temp_internal = 250        # 25.0 °C
        self.temp_external = 100        # 10.0 °C

        # Timing
        self.time_offset = 0            # 10 ms tick counter

        # Drift seed
        self._phase = 0.0

    def step(self, now: float):
        """Advance state by one cycle (called every ~2 seconds)."""
        self._phase += 0.05


        self.callsign_base = "SP0KS"

        # Drift latitude / longitude with sine waves + noise
        self.latitude += int(
            math.sin(self._phase * 1.3) * 5000 + random.randint(-2000, 2000)
        )
        self.latitude = max(-900000000, min(900000000, self.latitude))

        self.longitude += int(
            math.sin(self._phase * 0.7) * 8000 + random.randint(-3000, 3000)
        )
        self.longitude = max(-1800000000, min(1800000000, self.longitude))

        # Altitude changes: slow ascent + turbulence
        self.altitude += random.randint(-5, 15)
        self.altitude = max(0, min(65535, self.altitude))

        # GPS satellites: vary 4..16
        self.gps_sats += random.choice([-1, 0, 1])
        self.gps_sats = max(4, min(16, self.gps_sats))

        # GPS lock state: cycle occasionally
        self.lock_counter += 1
        if self.lock_counter > 30 and random.random() < 0.1:
            self.gps_lock = random.choice([0, 1, 2, 3])
            self.lock_counter = 0

        # Battery voltage: gradually drop then reset
        self.batt_voltage -= random.randint(0, 3)
        if self.batt_voltage < 2200:
            self.batt_voltage = 2500
        self.batt_voltage = max(0, min(3000, self.batt_voltage))

        # Internal temperature: slowly varies
        self.temp_internal += random.choice([-1, 0, 1])
        self.temp_internal = max(-500, min(1000, self.temp_internal))

        # External temperature: more variance
        self.temp_external += random.choice([-2, -1, 0, 1, 2])
        self.temp_external = max(-1000, min(500, self.temp_external))

        # Time offset (10ms ticks) since script started, wrapping 16-bit
        uptime_ticks = int((now - self.start_time) * 100)
        self.time_offset = uptime_ticks % 65536


# ---------------------------------------------------------------------------
# Packet construction
# ---------------------------------------------------------------------------
def make_apid0(time_now: float, start_time: float) -> bytes:
    """APID 0: TimeSync — raw 8 bytes (uint32 baseTime, uint32 baseTicks)."""
    base_time = int(time_now)                           # UNIX seconds
    base_ticks = int((time_now - start_time) * 100)     # 32-bit system uptime in 10ms ticks
    return struct.pack("!II", base_time, base_ticks)


def make_apid1(state: TelemetryState) -> bytes:
    """APID 1: CoreTelemetry — ASN.1 UPER encoded."""
    _LOCK_NAMES = {0: "none", 1: "fix2d", 2: "fix3d", 3: "diff"}
    telemetry = {
        "callsign": state.callsign_base,
        "timeOffset": state.time_offset,
        "latitude": state.latitude,
        "longitude": state.longitude,
        "altitude": state.altitude,
        "gpsSats": state.gps_sats,
        "gpsLock": _LOCK_NAMES[state.gps_lock],
        "battVoltage": state.batt_voltage,
        "tempInternal": state.temp_internal,
        "tempExternal": state.temp_external,
    }
    return _asn1_spec.encode("APID1-CoreTelemetry", telemetry)


def make_apid2() -> bytes:
    """APID 2: CBOR-encoded test payload."""
    return cbor2.dumps({"name": "AweerSat-1"})


# ---------------------------------------------------------------------------
# Wire protocol sender
# ---------------------------------------------------------------------------
def send_ws_telemetry(ws, receiver_id, apid, packet):
    if not ws:
        return
    try:
        msg = {
            "type": "telemetry",
            "receiver_id": receiver_id,
            "apid": apid,
            "packet": packet
        }
        ws.send(json.dumps(msg))
    except Exception as e:
        print(f"[WS] Warning: Failed to send telemetry over WS: {e}")


def send_packet(sock: socket.socket, apid: int, priority: int, payload: bytes):
    """Send one packet using the service wire protocol."""
    header = struct.pack("!B H I", priority, apid, len(payload))
    sock.sendall(header + payload)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="HAB Telemetry Generator — sends APID 0/1/2 to Modulator Service"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Service host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Service port (default: 8000)"
    )
    parser.add_argument(
        "--ws-server", default="ws://localhost:3000/ws", help="Elysia central server WS URL (default: ws://localhost:3000/ws)"
    )
    args = parser.parse_args()

    print(f"Connecting to Modulator Service at {args.host}:{args.port}...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    try:
        sock.connect((args.host, args.port))
    except ConnectionRefusedError:
        print("ERROR: Connection refused. Is the Modulator Service running?")
        sys.exit(1)

    ws = None
    receiver_uuid = str(uuid.uuid4())
    if HAS_WEBSOCKET:
        try:
            print(f"Connecting to Central Telemetry Server at {args.ws_server}...")
            ws = websocket.create_connection(args.ws_server, timeout=3)
            # Register receiver
            reg_msg = {
                "type": "register",
                "receiver_id": receiver_uuid,
                "nickname": "Python Simulator",
                "lat": 52.2297,
                "lon": 21.0122,
                "alt": 110.0,
                "antenna": "Simulated Omni",
                "radio": "Simulated SDR"
            }
            ws.send(json.dumps(reg_msg))
            print(f"[WS] Connected & Registered with UUID: {receiver_uuid}")
        except Exception as e:
            print(f"[WS] Warning: Failed to connect to telemetry server: {e}")
    else:
        print("[WS] Warning: 'websocket-client' module not found. Run 'pip install websocket-client' for live server upload.")

    print("Connected. Sending telemetry streams:")
    print("  APID 0 (TimeSync)     — every 10s  — priority 2 (REALTIME)")
    print("  APID 1 (CoreTelemetry) — every 2s   — priority 1 (HIGH)")
    print("  APID 2 (CBOR Payload)  — every 5s   — priority 0 (NORMAL)")
    print("Press Ctrl+C to stop.\n")

    start_time = time.time()
    state = TelemetryState(start_time)

    # Track last send times (stagger initial sends)
    last_apid0 = 0.0
    last_apid1 = 0.0
    last_apid2 = 0.0

    packet_count = 0

    try:
        while True:
            now = time.time()

            # APID 0 — every 10 seconds
            if now - last_apid0 >= 10.0:
                payload = make_apid0(now, start_time)
                send_packet(sock, apid=0, priority=2, payload=payload)
                packet_count += 1
                utc = datetime.now(timezone.utc).strftime("%H:%M:%S")
                print(
                    f"[{utc}] [APID 0]  TimeSync   | "
                    f"baseTime={int(now)} baseTicks={int((now - start_time) * 100)} | "
                    f"packets={packet_count}"
                )
                
                # Send to Elysia server
                packet_data = {
                    "callsign": "SP0KS",
                    "computed_time": datetime.fromtimestamp(now, tz=timezone.utc).isoformat(),
                    "time_offset_10ms": int((now - start_time) * 100)
                }
                send_ws_telemetry(ws, receiver_uuid, apid=0, packet=packet_data)
                
                last_apid0 = now

            # APID 1 — every 2 seconds
            if now - last_apid1 >= 2.0:
                state.step(now)
                payload = make_apid1(state)
                send_packet(sock, apid=1, priority=1, payload=payload)
                packet_count += 1
                utc = datetime.now(timezone.utc).strftime("%H:%M:%S")
                lock_names = ["NONE", "2D", "3D", "DIFF"]
                print(
                    f"[{utc}] [APID 1]  CoreTel    | "
                    f"call={state.callsign_base} "
                    f"lat={state.latitude/1e7:.5f} lon={state.longitude/1e7:.5f} "
                    f"alt={state.altitude}m sats={state.gps_sats} "
                    f"lock={lock_names[state.gps_lock]} "
                    f"bat={state.batt_voltage/100:.2f}V "
                    f"tInt={state.temp_internal/10:.1f}C "
                    f"tExt={state.temp_external/10:.1f}C | "
                    f"packets={packet_count}"
                )
                
                # Send to Elysia server
                ws_lock_names = ["none", "fix2d", "fix3d", "diff"]
                packet_data = {
                    "callsign": state.callsign_base,
                    "computed_time": datetime.fromtimestamp(now, tz=timezone.utc).isoformat(),
                    "time_offset_10ms": state.time_offset,
                    "latitude": state.latitude / 1e7,
                    "longitude": state.longitude / 1e7,
                    "altitude_m": state.altitude,
                    "gps_sats": state.gps_sats,
                    "gps_lock": ws_lock_names[state.gps_lock],
                    "batt_voltage": state.batt_voltage / 100.0,
                    "temp_internal": state.temp_internal / 10.0,
                    "temp_external": state.temp_external / 10.0
                }
                send_ws_telemetry(ws, receiver_uuid, apid=1, packet=packet_data)
                
                last_apid1 = now

            # APID 2 — every 5 seconds
            if now - last_apid2 >= 5.0:
                payload = make_apid2()
                send_packet(sock, apid=2, priority=0, payload=payload)
                packet_count += 1
                utc = datetime.now(timezone.utc).strftime("%H:%M:%S")
                print(
                    f"[{utc}] [APID 2]  CBOR       | "
                    f"payload_len={len(payload)}B content={cbor2.loads(payload)} | "
                    f"packets={packet_count}"
                )
                
                # Send to Elysia server
                packet_data = {
                    "callsign": "SP0KS",
                    "computed_time": datetime.fromtimestamp(now, tz=timezone.utc).isoformat(),
                    "time_offset_10ms": 0,
                    "name": "AweerSat-1"
                }
                send_ws_telemetry(ws, receiver_uuid, apid=2, packet=packet_data)
                
                last_apid2 = now

            # Sleep 100 ms so we don't busy-spin
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping telemetry generator.")
    finally:
        sock.close()
        print(f"Socket closed. Total packets sent: {packet_count}")


if __name__ == "__main__":
    main()

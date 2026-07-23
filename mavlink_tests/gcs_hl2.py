#!/usr/bin/env python3
"""TEST B - HIGH_LATENCY2 GCS decoder on the ground node's endpoint.
Decodes the single compact HIGH_LATENCY2 frame into a full flight readout as it
crosses the mesh."""

# ═══════════════════════ CONFIG (edit me) ═══════════════════════
GROUND_IP   = "192.168.0.244"   # the ground node's IP (WiFi). No LAN out there?
GROUND_PORT = 14550             #   point this at wherever the ground GCS endpoint is.
DURATION_S  = 62

GCS_HEARTBEAT_HZ = 0.2          # keepalive that holds the ground's wait-for-client
                                # registration. 0.2Hz (every 5s) is the clean bench value;
                                # too fast wastes airtime on the uplink.
# ════════════════════════════════════════════════════════════════

import time, collections
from pymavlink import mavutil

TARGET = f"udpout:{GROUND_IP}:{GROUND_PORT}"
hb_dt = (1.0 / GCS_HEARTBEAT_HZ) if GCS_HEARTBEAT_HZ > 0 else None
m = mavutil.mavlink_connection(TARGET, source_system=255, source_component=190)

def beat():
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                         0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)

beat()
print(f"HL2 GCS on {TARGET}  GCS_HB={GCS_HEARTBEAT_HZ}Hz  for {DURATION_S}s ...", flush=True)
counts = collections.Counter(); last_beat = 0.0; start = time.time(); hl2 = []

while time.time() - start < DURATION_S:
    now = time.time()
    if hb_dt and now - last_beat > hb_dt:
        beat(); last_beat = now
    msg = m.recv_match(blocking=True, timeout=1.0)
    if not msg:
        continue
    t = msg.get_type(); counts[t] += 1
    if t == "HIGH_LATENCY2" and msg.get_srcSystem() == 1:
        lat, lon = msg.latitude / 1e7, msg.longitude / 1e7
        hdg = msg.heading * 2
        gs = msg.groundspeed / 5.0
        climb = msg.climb_rate / 10.0
        hl2.append((round(now - start, 1), lat, msg.altitude, msg.battery))
        print(f"  [{now-start:5.1f}s] HL2  lat={lat:.5f} lon={lon:.4f} alt={msg.altitude:3d}m "
              f"hdg={hdg:3d} gspd={gs:.0f}m/s climb={climb:+.1f} batt={msg.battery}%", flush=True)

print("\n=== received message types ===")
for t, c in counts.most_common():
    print(f"  {t:22s} {c}")
print(f"\nHIGH_LATENCY2 frames received cross-mesh: {len(hl2)}")
if len(hl2) >= 2:
    dt = hl2[-1][0] - hl2[0][0]
    if dt > 0:
        print(f"  {len(hl2)} full-state frames over {dt:.0f}s = {len(hl2)/dt:.2f} Hz sustained")
    print(f"  alt {hl2[0][2]}m -> {hl2[-1][2]}m,  batt {hl2[0][3]}% -> {hl2[-1][3]}%")

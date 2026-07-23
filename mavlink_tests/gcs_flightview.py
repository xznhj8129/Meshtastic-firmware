#!/usr/bin/env python3
"""GCS on the ground node's endpoint: decode the 5-stream fake flight as it crosses the mesh."""

# ═══════════════════════ CONFIG (edit me) ═══════════════════════
GROUND_IP   = "192.168.0.244"   # the ground node's IP (WiFi). Repoint if you moved networks.
GROUND_PORT = 14550
DURATION_S  = 1e10
GCS_HEARTBEAT_HZ = 0.2          # keepalive holding the ground's wait-for-client registration
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
print(f"GCS listening on {TARGET} for {DURATION_S}s ...", flush=True)
counts = collections.Counter(); last_beat = 0.0; start = time.time(); fixes = []

while time.time() - start < DURATION_S:
    now = time.time()
    if hb_dt and now - last_beat > hb_dt:
        beat(); last_beat = now
    msg = m.recv_match(blocking=True, timeout=1.0)
    if not msg:
        continue
    t = msg.get_type(); counts[t] += 1
    if t == "GLOBAL_POSITION_INT" and msg.get_srcSystem() == 1:
        lat, lon, alt = msg.lat/1e7, msg.lon/1e7, msg.alt/1e3
        fixes.append((round(now-start, 1), lat, lon, alt))
        print(f"  [{now-start:5.1f}s] FLIGHT GPS  lat={lat:.5f} lon={lon:.5f} alt={alt:5.1f}m", flush=True)
    elif t == "SYS_STATUS" and msg.get_srcSystem() == 1:
        print(f"  [{now-start:5.1f}s] FLIGHT BATT {msg.voltage_battery/1000:.2f}V "
              f"{msg.battery_remaining}%", flush=True)
    elif t == "HEARTBEAT" and msg.get_srcSystem() == 1 and counts["HEARTBEAT"] == 1:
        print(f"  [{now-start:5.1f}s] FLIGHT HEARTBEAT (autopilot alive, sys=1)", flush=True)

print("\n=== received message types (all sources) ===")
for t, c in counts.most_common():
    print(f"  {t:22s} {c}")
print(f"\nflight GPS fixes received cross-mesh: {len(fixes)}")
if len(fixes) >= 2:
    d = (fixes[-1][1] - fixes[0][1]) * 111320.0
    print(f"  first: {fixes[0]}\n  last:  {fixes[-1]}")
    print(f"  track: moved {d:.0f} m north, climbed {fixes[-1][3]-fixes[0][3]:.0f} m")

#!/usr/bin/env python3
"""Minimal fake MAVLink flight over the air node's UART (CP2102 on the GPIO47/48 pins).
A quad taking off and flying north while climbing, sending the core messages the
Meshtastic MAVLink bridge snoops + a couple of GCS staples. Heavier than HL2 (5 separate
streams) - LoRa is slow, so keep the rates modest."""

# ═══════════════════════ CONFIG (edit me) ═══════════════════════
UART_PORT = "/dev/ttyUSB1"   # CP2102 wired to the air node's GPIO47(RX)/GPIO48(TX)
UART_BAUD = 57600            # must match serial.baud on the air node
DURATION_S = 1e10

# --- per-message stream rates, Hz (set any to 0 to disable that stream) ---
RATES_HZ = {
    "HEARTBEAT":           1.0,
    "GPS_RAW_INT":         0.5,
    "GLOBAL_POSITION_INT": 0.5,
    "SYS_STATUS":          0.2,
    "VFR_HUD":             0.2,
}
# reminder (bench notes): 5 streams @1Hz already saturates MediumFast; on ShortFast it's fine.
# Drop the ones you don't care about to 0 to lighten the link.

# --- flight model ---
START_LAT, START_LON = 45.4215, -75.6972
CLIMB_RATE_MS   = 6.0
CRUISE_ALT_M    = 100.0
GROUND_SPEED_MS = 15.0
TAKEOFF_DELAY_S = 3.0
BATT_START_V    = 12.6
BATT_DRAIN_VPS  = 0.02
# ════════════════════════════════════════════════════════════════

import time
from pymavlink import mavutil

m = mavutil.mavlink_connection(UART_PORT, baud=UART_BAUD, source_system=1,
                               source_component=1, dialect="common", force_mavlink2=True)
mav = m.mav
t0 = time.time()

def state(t):
    alt = min(CRUISE_ALT_M, t * CLIMB_RATE_MS)
    north_m = max(0.0, (t - TAKEOFF_DELAY_S)) * GROUND_SPEED_MS
    lat = START_LAT + north_m / 111320.0
    vbat = max(10.5, BATT_START_V - t * BATT_DRAIN_VPS)
    rem = max(20, int(100 - t * 0.8))
    return lat, START_LON, alt, vbat, rem, north_m

gs5 = int(GROUND_SPEED_MS * 100)  # cm/s for GLOBAL_POSITION_INT.vx
last = {k: 0.0 for k in RATES_HZ}
print(f"fake flight -> {UART_PORT}@{UART_BAUD} for {DURATION_S}s  rates={RATES_HZ}", flush=True)

while True:
    now = time.time(); t = now - t0
    if t >= DURATION_S:
        break
    lat, lon, alt, vbat, rem, north = state(t)
    hdg = 0  # flying north
    for name, hz in RATES_HZ.items():
        if hz <= 0 or now - last[name] < 1.0 / hz:
            continue
        last[name] = now
        if name == "HEARTBEAT":
            mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                               mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                               mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED |
                               mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                               0, mavutil.mavlink.MAV_STATE_ACTIVE)
        elif name == "GPS_RAW_INT":
            mav.gps_raw_int_send(int(now*1e6), 3, int(lat*1e7), int(lon*1e7),
                                 int(alt*1e3), 120, 150, int(GROUND_SPEED_MS*100), hdg*100, 12)
        elif name == "GLOBAL_POSITION_INT":
            mav.global_position_int_send(int(t*1000), int(lat*1e7), int(lon*1e7),
                                         int(alt*1e3), int(alt*1e3), gs5, 0, 0, hdg*100)
        elif name == "SYS_STATUS":
            mav.sys_status_send(0, 0, 0, 400, int(vbat*1000), 800, rem, 0, 0, 0, 0, 0, 0)
        elif name == "VFR_HUD":
            mav.vfr_hud_send(GROUND_SPEED_MS, GROUND_SPEED_MS, hdg, 55, alt,
                             CLIMB_RATE_MS if alt < CRUISE_ALT_M else 0.0)
    if int(t*2) % 20 == 0:
        print(f"  t={t:4.1f}s lat={lat:.5f} lon={lon:.5f} alt={alt:4.0f}m north={north:4.0f}m "
              f"vbat={vbat:.2f}V {rem}%", flush=True)
    m.recv_match(blocking=False)
    time.sleep(0.02)
print("flight done", flush=True)

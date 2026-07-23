#!/usr/bin/env python3
"""TEST A - HIGH_LATENCY2 flight sender over the air node's UART.
HIGH_LATENCY2 (msgid 235) packs the whole vehicle state into ONE ~40-byte frame -
the protocol MAVLink designed for low-bandwidth links. One frame per second is a
complete telemetry picture, tiny enough to stream over LoRa."""

# ═══════════════════════ CONFIG (edit me) ═══════════════════════
UART_PORT = "/dev/ttyUSB1"   # CP2102 wired to the air node's GPIO47(RX)/GPIO48(TX)
UART_BAUD = 57600            # must match serial.baud on the air node
DURATION_S = 60              # how long to fly

# --- stream rates, Hz (set 0 to disable a stream) ---
HEARTBEAT_HZ = 1.0           # vehicle heartbeat (also what makes the bridge learn role=AIR)
HL2_HZ       = 1.0           # HIGH_LATENCY2 full-state frame
# NOTE (from bench notes): on ShortFast, HL2 @0.5Hz stays perfectly ordered;
# @1Hz starts to reorder under load. Drop HL2_HZ to 0.5 for the rock-solid profile.

# --- flight model ---
START_LAT, START_LON = 45.4215, -75.6972
CLIMB_RATE_MS   = 6.0        # m/s climb until cruise altitude
CRUISE_ALT_M    = 120.0      # level off here
GROUND_SPEED_MS = 15.0       # forward speed (flies due north)
TAKEOFF_DELAY_S = 3.0        # hover before moving
BATT_START_V    = 12.6
BATT_DRAIN_VPS  = 0.02       # volts/sec
# ════════════════════════════════════════════════════════════════

import time
from pymavlink import mavutil

m = mavutil.mavlink_connection(UART_PORT, baud=UART_BAUD, source_system=1,
                               source_component=1, dialect="common", force_mavlink2=True)
mav = m.mav
t0 = time.time()
hb_dt  = (1.0 / HEARTBEAT_HZ) if HEARTBEAT_HZ > 0 else None
hl2_dt = (1.0 / HL2_HZ) if HL2_HZ > 0 else None
last_hb = last_hl2 = 0.0

print(f"HL2 flight -> {UART_PORT}@{UART_BAUD}  HB={HEARTBEAT_HZ}Hz HL2={HL2_HZ}Hz  for {DURATION_S}s",
      flush=True)

while True:
    now = time.time(); t = now - t0
    if t >= DURATION_S:
        break
    alt = min(CRUISE_ALT_M, t * CLIMB_RATE_MS)
    north_m = max(0.0, (t - TAKEOFF_DELAY_S)) * GROUND_SPEED_MS
    lat = START_LAT + north_m / 111320.0
    vbat = max(10.5, BATT_START_V - t * BATT_DRAIN_VPS)
    rem = max(15, int(100 - t * 0.9))
    climb = CLIMB_RATE_MS if alt < CRUISE_ALT_M else 0.0

    if hb_dt and now - last_hb >= hb_dt:
        last_hb = now
        mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                           mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                           mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED, 0,
                           mavutil.mavlink.MAV_STATE_ACTIVE)
    if hl2_dt and now - last_hl2 >= hl2_dt:
        last_hl2 = now
        mav.high_latency2_send(
            timestamp=int(t * 1000),
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            custom_mode=0,
            latitude=int(lat * 1e7), longitude=int(START_LON * 1e7),
            altitude=int(alt), target_altitude=int(CRUISE_ALT_M),
            heading=0, target_heading=0, target_distance=0,
            throttle=55,
            airspeed=int(GROUND_SPEED_MS * 5), airspeed_sp=int(GROUND_SPEED_MS * 5),
            groundspeed=int(GROUND_SPEED_MS * 5),
            windspeed=0, wind_heading=0, eph=1, epv=1, temperature_air=20,
            climb_rate=int(climb * 10), battery=rem,
            wp_num=0, failure_flags=0, custom0=0, custom1=0, custom2=0)
        print(f"  t={t:4.1f}s HL2 lat={lat:.5f} alt={int(alt)}m batt={rem}% climb={climb}m/s",
              flush=True)
    m.recv_match(blocking=False)   # drain any GCS uplink
    time.sleep(0.02)
print("HL2 flight done", flush=True)

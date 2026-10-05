#!/usr/bin/env python3
"""
ArduCopter Simulator
====================
A minimal simulated flight controller, so the mission logic can be exercised
end to end without an aircraft leaving the ground.

It deliberately reproduces the conditions that broke the real flight, so a
regression cannot creep back in:
  * GLOBAL_POSITION_INT.relative_alt and LOCAL_POSITION_NED.z carry DIFFERENT
    large offsets (-3.62 m and -5.04 m) while the aircraft sits on the ground.
  * RC_CHANNELS reports chancount = 0 unless CH8_FILE says otherwise.
  * The LOCAL_POSITION_NED origin is NOT the GPS home: it sits at a constant
    offset (SIM_LOCAL_ORIGIN_N/E), the way a real EKF origin set at power-up
    does. Any code that assumes a Google Maps coordinate maps straight onto
    LOCAL_POSITION_NED will put the mission area tens of metres off here.

Horizontal motion (added for raster testing):
  A plain kinematic integrator, position += velocity * dt. Velocity setpoints
  from SET_POSITION_TARGET_LOCAL_NED are honoured, expire like ArduPilot's do,
  and the resulting position is reported in BOTH LOCAL_POSITION_NED (x, y) and
  GLOBAL_POSITION_INT (lat, lon) so the two stay consistent. No drag, no
  attitude, no motor lag: enough to exercise a position controller, and
  deterministic so a test result is reproducible.

Environment:
  SIM_CONN            where to send  (default udpout:127.0.0.1:14560)
  SIM_MOTORS          1 = motors produce thrust, 0 = armed but no climb
  CH8_FILE            file holding a CH8 PWM value; absent/0 means no receiver
  FENCE_FILE          file holding 1 to report an ArduPilot fence breach
  SIM_HOME_LAT/LON    GPS position of the simulated start point
  SIM_LOCAL_ORIGIN_N/E  where the EKF origin sits relative to that start point
  SIM_DRIFT_N/E_MPS   constant wind-like drift added to commanded velocity
  SIM_DRIFT_FILE      file holding 1 to switch that drift on mid-flight
  SIM_VZ_DRIFT_MPS    vertical drift, + = climbing, for altitude-band tests
  SIM_VZ_DRIFT_FILE   file holding 1 to switch the vertical drift on mid-flight
  SIM_BATT_PERCENT    battery percentage reported in SYS_STATUS
  SIM_BATT_FILE       file holding a percentage to report from that moment
  SIM_EKF_FILE        file holding EKF flags to report from that moment
  SIM_STALE_LOCAL_FILE  file holding 1 to stop sending LOCAL_POSITION_NED
  SIM_SETPOINT_TIMEOUT_S  how long a velocity setpoint stays live

Normally driven by run_simulation.sh rather than run directly.
"""
import math, os, sys, time
from pymavlink import mavutil

CONN = os.getenv("SIM_CONN", "udpout:127.0.0.1:14560")
GLOBAL_ALT_OFFSET = -3.62      # metres of bogus home offset
LOCAL_ALT_OFFSET = -5.04       # different bogus EKF-origin offset
MOTORS_POWERED = os.getenv("SIM_MOTORS", "1") == "1"

# --- horizontal simulation -------------------------------------------------
R_EARTH_M = 6378137.0          # same WGS-84 radius the planner/mission use
# Where the simulated aircraft starts. The historical default keeps every
# pre-existing scenario reporting exactly the position it used to.
SIM_HOME_LAT = float(os.getenv("SIM_HOME_LAT", "13.3456125"))
SIM_HOME_LON = float(os.getenv("SIM_HOME_LON", "74.7940916"))
# The EKF origin, in metres north/east of the start point. Non-zero on purpose:
# LOCAL_POSITION_NED is relative to the EKF origin, which a real flight
# controller sets wherever it happened to acquire position, NOT at the mission
# area. Deliberately wrong-by-construction, exactly like the two altitude
# offsets above, so a GPS-to-local conversion that skips the runtime anchor
# fails loudly here instead of in the air.
SIM_LOCAL_ORIGIN_N = float(os.getenv("SIM_LOCAL_ORIGIN_N", "-12.0"))
SIM_LOCAL_ORIGIN_E = float(os.getenv("SIM_LOCAL_ORIGIN_E", "7.0"))
# Constant wind-like drift, added to whatever velocity was commanded. Used to
# push the aircraft across a safety boundary without faking the boundary check.
SIM_DRIFT_N_MPS = float(os.getenv("SIM_DRIFT_N_MPS", "0.0"))
SIM_DRIFT_E_MPS = float(os.getenv("SIM_DRIFT_E_MPS", "0.0"))
# When SIM_DRIFT_FILE names a file holding 1, the drift above is applied only
# from that moment; otherwise it is applied from the start.
#
# A constant drift cannot be used to push the aircraft off a waypoint it is
# actively tracking. Below the controller's speed cap the proportional law simply
# rejects it, leaving a steady-state offset of about drift/gain; above the cap the
# controller is overwhelmed everywhere, including on the way TO the area, so the
# aircraft never arrives. Gating the wind lets a test put the aircraft where it
# belongs first and only then apply a wind it cannot fight — the same idiom as
# SIM_STALE_LOCAL_FILE.

# ArduPilot expires a GUIDED velocity setpoint and holds position rather than
# coasting on the last command forever.
SIM_SETPOINT_TIMEOUT_S = float(os.getenv("SIM_SETPOINT_TIMEOUT_S", "3.0"))
# Below this AGL the aircraft is on the ground and does not slide.
SIM_AIRBORNE_ALT_M = 0.10

# --- fault injection for the telemetry-only dry-run tests --------------------
# The dry run is supposed to REFUSE on a bad aircraft state. Testing that needs a
# simulator that can present a bad state without the test faking the check.
# Defaults reproduce the previous behaviour exactly.
SIM_START_ARMED = os.getenv("SIM_START_ARMED", "0") == "1"
SIM_GPS_FIX = int(os.getenv("SIM_GPS_FIX", "3"))
SIM_SATS = int(os.getenv("SIM_SATS", "16"))
# EKF_STATUS_REPORT flags. 831 is a healthy estimate; bit 4 (16, POS_HORIZ_ABS)
# is the one the mission gates on. 167 reproduces the real CONST_POS_MODE state
# recorded in the project notes, which has that bit clear.
SIM_EKF_FLAGS = int(os.getenv("SIM_EKF_FLAGS", "831"))
# Battery percentage reported in SYS_STATUS. SIM_BATT_FILE, when it names a file
# holding an integer > 0, overrides it from that moment, so a test can drop the
# battery mid-pattern rather than starting flat.
SIM_BATT_PERCENT = int(os.getenv("SIM_BATT_PERCENT", "98"))
# Vertical drift in m/s, + = climbing, gated by SIM_VZ_DRIFT_FILE the same way the
# horizontal drift is gated. Used to walk the aircraft out of the altitude band
# without faking the altitude report: the simulated vehicle really moves.
SIM_VZ_DRIFT_MPS = float(os.getenv("SIM_VZ_DRIFT_MPS", "0.0"))

MODES = {0: "STABILIZE", 4: "GUIDED", 5: "LOITER", 9: "LAND", 6: "RTL"}

m = mavutil.mavlink_connection(CONN, source_system=1, source_component=1,
                               dialect="ardupilotmega")

armed = SIM_START_ARMED
mode = 0
alt = 0.0            # true AGL
setpoint_logged = False   # so a 10 Hz setpoint stream logs once, not 10x/second
vz_cmd = 0.0         # commanded down-velocity (m/s, + = down)
pos_n = 0.0          # true position, metres north of the simulated start point
pos_e = 0.0          # true position, metres east of the simulated start point
vn_cmd = 0.0         # commanded north velocity (m/s)
ve_cmd = 0.0         # commanded east velocity (m/s)
vn_act = 0.0         # velocity actually being flown, drift included
ve_act = 0.0
last_setpoint_t = 0.0
takeoff_target = None
land_speed = 0.20
t_last = time.time()
next_hb = 0.0
next_fast = 0.0
next_slow = 0.0
log = lambda s: print(f"[SIM {time.strftime('%H:%M:%S')}] {s}", flush=True)

def ack(cmd, result=0):
    m.mav.command_ack_send(cmd, result)


def sim_latlon(north_m, east_m):
    """Simulated position as GPS, using the same flat-earth model as the planner."""
    lat = SIM_HOME_LAT + math.degrees(north_m / R_EARTH_M)
    lon = SIM_HOME_LON + math.degrees(
        east_m / (R_EARTH_M * math.cos(math.radians(SIM_HOME_LAT))))
    return lat, lon


def file_flag(env_name):
    """Reads an integer from the file named by an env var; 0 if absent/unreadable."""
    try:
        with open(os.getenv(env_name, "/nonexistent")) as fh:
            return int(fh.read().strip())
    except Exception:
        return 0

log(f"Simulator up on {CONN}. motors_powered={MOTORS_POWERED}")
log(f"home={SIM_HOME_LAT:.7f},{SIM_HOME_LON:.7f}  "
    f"ekf_origin_offset=N{SIM_LOCAL_ORIGIN_N:+.1f},E{SIM_LOCAL_ORIGIN_E:+.1f}m  "
    f"drift=N{SIM_DRIFT_N_MPS:+.2f},E{SIM_DRIFT_E_MPS:+.2f}m/s")
log(f"start_armed={SIM_START_ARMED}  gps_fix={SIM_GPS_FIX}  sats={SIM_SATS}  "
    f"ekf_flags={SIM_EKF_FLAGS}  batt={SIM_BATT_PERCENT}%  "
    f"vz_drift={SIM_VZ_DRIFT_MPS:+.2f}m/s")

while True:
    now = time.time()
    dt = now - t_last
    t_last = now

    # ---------------- physics ----------------
    if armed and MOTORS_POWERED:
        if mode == 9:  # LAND
            alt = max(0.0, alt - land_speed * dt)
        elif mode == 9:
            alt = max(0.0, alt - land_speed * dt)
        elif takeoff_target is not None:
            if alt < takeoff_target - 0.02:
                alt = min(takeoff_target, alt + 0.8 * dt)   # ~0.8 m/s climb
            else:
                takeoff_target = None
                log(f"reached takeoff target, holding at {alt:.2f}m")
        else:
            alt = max(0.0, alt - vz_cmd * dt)
        if mode == 9 and alt <= 0.001 and armed:
            armed = False
            log("touchdown -> auto-disarm")
    elif armed and not MOTORS_POWERED:
        pass  # armed but no thrust: stays on the ground

    # ---------------- horizontal physics ----------------
    # Deliberately trivial: position += velocity * dt. The vertical model above
    # is untouched.
    if now - last_setpoint_t > SIM_SETPOINT_TIMEOUT_S:
        vn_cmd = ve_cmd = 0.0      # setpoint expired, as on a real vehicle
    if not armed:
        vn_cmd = ve_cmd = 0.0
    # Vertical drift, applied to the true altitude while airborne.
    vz_drift_on = ("SIM_VZ_DRIFT_FILE" not in os.environ) or file_flag("SIM_VZ_DRIFT_FILE")
    if (SIM_VZ_DRIFT_MPS and vz_drift_on and armed and MOTORS_POWERED
            and alt > SIM_AIRBORNE_ALT_M):
        alt = max(0.0, alt + SIM_VZ_DRIFT_MPS * dt)

    drift_on = ("SIM_DRIFT_FILE" not in os.environ) or file_flag("SIM_DRIFT_FILE")
    if armed and MOTORS_POWERED and alt > SIM_AIRBORNE_ALT_M:
        vn_act = vn_cmd + (SIM_DRIFT_N_MPS if drift_on else 0.0)
        ve_act = ve_cmd + (SIM_DRIFT_E_MPS if drift_on else 0.0)
    else:
        vn_act = ve_act = 0.0      # on the ground it does not slide
    pos_n += vn_act * dt
    pos_e += ve_act * dt

    # ---------------- telemetry ----------------
    if now >= next_hb:
        next_hb = now + 0.5
        base = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if armed:
            base |= mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_QUADROTOR,
                             mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
                             base, mode, mavutil.mavlink.MAV_STATE_ACTIVE)

    if now >= next_fast:
        next_fast = now + 0.1
        sim_lat, sim_lon = sim_latlon(pos_n, pos_e)
        m.mav.global_position_int_send(
            int(now * 1000) & 0xFFFFFFFF,
            int(round(sim_lat * 1e7)), int(round(sim_lon * 1e7)),
            int((74.58 + alt) * 1000), int((alt + GLOBAL_ALT_OFFSET) * 1000),
            int(vn_act * 100), int(ve_act * 100), int(-vz_cmd * 100), 0)
        # LOCAL_POSITION_NED is relative to the EKF origin, not to the GPS home.
        # SIM_STALE_LOCAL_FILE stops this message while everything else keeps
        # streaming, which is what a position-estimate dropout looks like.
        if not file_flag("SIM_STALE_LOCAL_FILE"):
            m.mav.local_position_ned_send(
                int(now * 1000) & 0xFFFFFFFF,
                pos_n - SIM_LOCAL_ORIGIN_N, pos_e - SIM_LOCAL_ORIGIN_E,
                -(alt + LOCAL_ALT_OFFSET), vn_act, ve_act, vz_cmd)
        # RC: CH8_FILE lets the test move the switch mid-run.
        # Absent file => no receiver at all (chancount 0), matching the real rig.
        ch8 = file_flag("CH8_FILE")
        if ch8 > 0:
            chans = [1500] * 18
            chans[7] = ch8
            m.mav.rc_channels_send(int(now * 1000) & 0xFFFFFFFF, 8, *chans, 200)
        else:
            m.mav.rc_channels_send(int(now * 1000) & 0xFFFFFFFF, 0, *([0] * 18), 255)

    if now >= next_slow:
        next_slow = now + 0.5
        batt = file_flag("SIM_BATT_FILE") or SIM_BATT_PERCENT
        m.mav.sys_status_send(0, 0, 0, 250, 16800, 1000, batt, 0, 0, 0, 0, 0, 0)
        raw_lat, raw_lon = sim_latlon(pos_n, pos_e)
        m.mav.gps_raw_int_send(int(now * 1e6), SIM_GPS_FIX,
                               int(round(raw_lat * 1e7)), int(round(raw_lon * 1e7)),
                               74580, 100, 100, 0, 0, SIM_SATS)
        ekf_flags = file_flag("SIM_EKF_FILE") or SIM_EKF_FLAGS
        m.mav.ekf_status_report_send(ekf_flags, 0.09, 0.001, 0.005, 0.0004, 0.0, 0.0)
        breach = file_flag("FENCE_FILE")
        m.mav.fence_status_send(breach, 1 if breach else 0, 0, 0, 0)

    # ---------------- command handling ----------------
    msg = m.recv_match(blocking=True, timeout=0.02)
    if msg is None:
        continue
    t = msg.get_type()

    if t == "SET_MODE":
        mode = msg.custom_mode
        log(f"SET_MODE -> {MODES.get(mode, mode)}")
        if mode != 4:
            takeoff_target = None

    elif t == "COMMAND_LONG":
        cmd = msg.command
        if cmd == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            mode = int(msg.param2)
            log(f"DO_SET_MODE -> {MODES.get(mode, mode)}")
            ack(cmd)
        elif cmd == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            want = msg.param1 > 0.5
            if want and mode not in (4, 5, 9, 0):
                ack(cmd, 4)
                log("arm REJECTED (bad mode)")
            else:
                armed = want
                ack(cmd)
                log(f"{'ARMED' if armed else 'DISARMED'} (mode={MODES.get(mode, mode)})")
        elif cmd == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            if not armed:
                ack(cmd, 4)
                log("takeoff REJECTED (disarmed)")
            else:
                takeoff_target = float(msg.param7)
                ack(cmd)
                log(f"TAKEOFF accepted, target {takeoff_target:.2f}m")
        else:
            ack(cmd)

    elif t == "SET_POSITION_TARGET_LOCAL_NED":
        # Frames MAV_FRAME_LOCAL_NED (1) and MAV_FRAME_BODY_OFFSET_NED (9) are
        # equivalent here: the simulated vehicle holds yaw = 0, so body-forward
        # is north and body-right is east. Only the velocity fields are read;
        # the mission masks the position fields out.
        vn_cmd = float(msg.vx)
        ve_cmd = float(msg.vy)
        vz_cmd = float(msg.vz)
        last_setpoint_t = now
        takeoff_target = None
        if not setpoint_logged:
            # Logged once so a test can assert that a telemetry-only mode sent no
            # setpoint at all, without 10 Hz of noise in every other scenario.
            setpoint_logged = True
            log("SET_POSITION_TARGET_LOCAL_NED received (first)")

    elif t == "PARAM_SET":
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        pid = pid.strip("\x00")
        if pid == "LAND_SPEED":
            land_speed = msg.param_value / 100.0
        m.mav.param_value_send(pid.encode(), msg.param_value, msg.param_type, 1, 0)
        log(f"PARAM_SET {pid} = {msg.param_value}")

    elif t == "PARAM_REQUEST_READ":
        pid = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
        m.mav.param_value_send(pid.strip("\x00").encode(), 0.0, 9, 1, 0)

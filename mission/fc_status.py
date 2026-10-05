#!/usr/bin/env python3
"""
Snapshot of the flight controller, the mission daemon and the live config.

Reads only — never sends MAVLink, never touches the serial port. Everything
comes from the daemon's own log, its /proc environment, and systemd, so this is
safe to run while a mission is in the air.

    python fc_status.py            human-readable
    python fc_status.py --json     machine-readable (feeds the status page)
"""
import json, os, re, subprocess, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(HERE, "ardupilot_raster_mission.log")
UNIT = "ardupilot-raster-mission.service"
# ArduPilot repeats pre-arm text every ~30 s; anything older than this is stale.
PREARM_WINDOW_S = 90


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True,
                              text=True, timeout=10).stdout.strip()
    except Exception:
        return ""


def tail(path, n):
    try:
        return sh(f"tail -{n} {path!r}").splitlines()
    except Exception:
        return []


def log_time(line):
    m = re.match(r"(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)", line)
    if not m:
        return None
    try:
        return time.mktime(time.strptime(m.group(1), "%Y-%m-%d %H:%M:%S"))
    except Exception:
        return None


def collect():
    pid = sh(f"systemctl show {UNIT} -p MainPID --value")
    env = {}
    try:
        for kv in open(f"/proc/{pid}/environ").read().split("\0"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                env[k] = v
    except Exception:
        pass

    lines = tail(LOG, 400)
    now = time.time()
    serial = sh("ls /dev/serial/by-id/ 2>/dev/null").splitlines()

    # The daemon prints this whenever it cannot find the port. If the most
    # recent line says so, it is not talking to the aircraft right now.
    recent = lines[-4:] if lines else []
    searching = any("no serial device found" in l for l in recent)

    # Richest single line the daemon emits about the aircraft.
    health = {}
    for line in reversed(lines):
        if "ArduPilot Health:" in line:
            def g(pat, cast=str, d=None):
                m = re.search(pat, line)
                return cast(m.group(1)) if m else d
            health = {
                "mode": g(r"Mode=(\w+)"),
                "armed": g(r"Armed=(\w+)") == "True",
                "aglM": g(r"AGL=([-\d.]+)m", float),
                "gpsFix": g(r"Fix=(\d+)", int),
                "sats": g(r"Sats: (\d+)", int),
                "ekf": g(r"EKF=(\w+)"),
                "batteryPct": g(r"Batt=(\d+)%", int),
                "batteryV": g(r"\(([\d.]+)V\)", float),
                "ch8": g(r"CH8=(\d+)", int),
                "ch8State": g(r"CH8=\d+ \((\w+)\)"),
                "at": (re.match(r"([\d-]+ [\d:]+)", line) or [None, None])[1],
            }
            break

    # Standby lines carry live mode/armed even when no Health line is recent.
    for line in reversed(lines):
        if "[STANDBY]" in line:
            m = re.search(r"Mode=(\w+), Armed=(\w+)", line)
            if m:
                health.setdefault("mode", m.group(1))
                health["mode"] = m.group(1)
                health["armed"] = m.group(2) == "True"
            m = re.search(r"CH8=(\w+)", line)
            if m:
                health["ch8State"] = m.group(1)
            break

    prearm = []
    for line in lines:
        m = re.search(r"\[AP STATUSTEXT\] (PreArm: .+)$", line)
        if m:
            t = log_time(line)
            if t and now - t <= PREARM_WINDOW_S:
                msg = m.group(1).strip()
                if msg not in prearm:
                    prearm.append(msg)

    connected = bool(serial) and not searching
    if not connected:
        state = "no flight controller on USB" if not serial else "reconnecting"
    elif health.get("armed"):
        state = "ARMED"
    else:
        state = "standby — waiting for GUIDED"
    for line in reversed(lines[-40:]):
        if "RASTER PROGRESS" in line or "RASTER PHASE" in line:
            state = "mission in progress"
            break

    # Plan block from the most recent daemon start.
    try:
        txt = open(LOG, errors="ignore").read()
    except Exception:
        txt = ""
    plan_txt = txt.split("=== POLYGON RASTER MISSION PLAN ===")[-1]

    def grab(pat, cast=str, d=None):
        m = re.search(pat, plan_txt)
        return cast(m.group(1)) if m else d

    def fnum(key, d=0.0):
        try:
            return float(env.get(key, d) or d)
        except ValueError:
            return d

    return {
        "updatedAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "generatedBy": "fc_status.py on the companion Pi",
        "fc": {
            "model": "MicoAir743v2",
            "connected": connected,
            "serialId": serial[0] if serial else None,
            "state": state,
            **health,
            "prearmBlockers": prearm,
        },
        "daemon": {
            "unit": UNIT,
            "active": sh(f"systemctl is-active {UNIT}") == "active",
            "pid": int(pid) if pid.isdigit() and pid != "0" else None,
            "startedAt": sh(f"systemctl show {UNIT} -p ActiveEnterTimestamp --value"),
            "sourceDir": os.path.realpath(HERE),
        },
        "config": {
            "axis": env.get("RASTER_AXIS"),
            "speedMps": fnum("RASTER_SPEED_MPS"),
            "spacingM": fnum("RASTER_PASS_SPACING_M"),
            "takeoffAltM": fnum("TAKEOFF_ALTITUDE_M"),
            "altMinM": fnum("RASTER_ALT_MIN_M"),
            "altMaxM": fnum("RASTER_ALT_MAX_M"),
            "progressIntervalS": fnum("RASTER_PROGRESS_INTERVAL_S"),
            "dryRun": env.get("RASTER_DRY_RUN") == "1",
            "benchMode": env.get("BENCH_MODE") == "1",
        },
        "plan": {
            "passes": grab(r"Passes: (\d+)", int, 0),
            "pathM": grab(r"Path length ([\d.]+)m", float, 0.0),
            "estS": grab(r"estimated (\d+)s", int, 0),
            "areaM2": grab(r"area=([\d.]+)m\^2", float, 0.0),
            "passMinM": grab(r"Pass lengths: ([\d.]+)m", float, 0.0),
            "passMaxM": grab(r"Pass lengths: [\d.]+m to ([\d.]+)m", float, 0.0),
            "containment": grab(r"Containment: (\d+/\d+) waypoints", str, "?"),
            "innerOuterM": grab(r"minimum separation ([\d.]+)m", float, 0.0),
        },
    }


if __name__ == "__main__":
    s = collect()
    if "--json" in sys.argv:
        print(json.dumps(s, indent=2))
    else:
        fc, d, c, p = s["fc"], s["daemon"], s["config"], s["plan"]
        print(f"FC {fc['model']}: {'CONNECTED' if fc['connected'] else 'NOT CONNECTED'} — {fc['state']}")
        if fc.get("gpsFix") is not None:
            print(f"  GPS fix {fc['gpsFix']} / {fc.get('sats')} sats, EKF {fc.get('ekf')}, "
                  f"battery {fc.get('batteryPct')}% ({fc.get('batteryV')}V)")
        print(f"  mode {fc.get('mode')}, armed {fc.get('armed')}, CH8 {fc.get('ch8State')}")
        for b in fc["prearmBlockers"]:
            print(f"  BLOCKER  {b}")
        print(f"daemon: {'active' if d['active'] else 'INACTIVE'} pid {d['pid']} since {d['startedAt']}")
        print(f"config: axis={c['axis']} {c['speedMps']}m/s spacing {c['spacingM']}m "
              f"alt {c['takeoffAltM']}m band {c['altMinM']}-{c['altMaxM']}m")
        print(f"plan: {p['passes']} passes, {p['pathM']}m, ~{p['estS']}s, containment {p['containment']}")

# Known-Good Autonomous Flight Configuration

**Recorded:** 2026-09-10  
**Status:** The current configuration completed the planned flight. Treat this
document and the files named below as the baseline to return to before changing
anything further.

This is a recovery/runbook, not a replacement for the normal field safety
checks. Do not enable `BENCH_MODE`, disable arming checks, or change failsafes
to work around a failed pre-arm check on an aircraft with propellers fitted.

## The tested baseline

Hardware/software path:

- Flight controller: MicoAir743v2 running ArduCopter, connected to the Pi by
  USB MAVLink at 115200 baud.
- Companion: Raspberry Pi 5. The Python environment is
  `/home/aahswarm/px4_mavsdk_env` (the name is historical; this project uses
  pymavlink and ArduCopter **GUIDED**, not PX4/MAVSDK Offboard).
- Persistent launcher: `ardupilot-mission.service`, which runs
  `start_ardupilot_mission.sh` and then `ardupilot_mission.py`.
- Connection selection: automatic, preferring `/dev/serial/by-id/`; do **not**
  hard-code `/dev/ttyACM0`, because its number can change after a power cycle.

The service launcher currently sets `AUTOSTART_ON_READY=1`. With a live RC
receiver and CH8 already in the GUIDED band, it starts **one** mission as soon
as the flight controller is ready. Therefore, only start/restart the service
when the aircraft is in the intended launch condition and the pilot is ready.

### Mission profile

| Item | Current value |
| --- | --- |
| Takeoff target | 1.2 m AGL |
| Forward motion | 0.3 m/s for 5.0 s |
| Normal descent | 0.15 m/s |
| Final descent below 0.40 m | 0.08 m/s |
| No-climb timeout | 8 s |
| Retry cooldown | 20 s |
| Fence | Disabled (`FENCE_ENABLE=0`) |

### CH8 / Switch SC control

| CH8 PWM band | Meaning |
| --- | --- |
| Below 1250 (about 1000) | MANUAL — companion stops autonomous velocity commands; pilot takes control. |
| 1250–1750 (about 1500) | GUIDED — allows the autonomous mission. |
| Above 1750 (about 2000) | LAND — commands a safety landing. |

The known-good automatic start path is: receiver live, CH8 in GUIDED, then the
mission service starts and sees the flight controller ready. A normal CH8
trigger also works only after the daemon has first seen CH8 outside GUIDED; move
it to MANUAL and then back to GUIDED. This prevents a surprise mission merely
because the switch was already in GUIDED when the USB link reconnects.

### Safety parameters actively restored by the daemon

`make_flight_ready.py` and the daemon use this intended set:

| Parameter | Value | Intended action |
| --- | ---: | --- |
| `ARMING_CHECK` | 1 | All ArduPilot pre-arm checks enabled. |
| `LAND_SPEED` | 20 cm/s | Gentle landing. |
| `LAND_ALT_LOW` | 150 cm | Slow down below 1.5 m. |
| `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT` | 1 | Land on low/critical battery. |
| `FENCE_ENABLE` | 0 | No geofence; pilot keeps the aircraft within the test area. |
| `FS_THR_ENABLE` | 3 | Land on RC loss. |
| `FS_GCS_ENABLE` | 5 | Land if the Pi heartbeat stops during guided flight. |

The daemon sends the required GCS heartbeat at about 1 Hz. Do not change this
arrangement without re-testing the GCS failsafe. The pilot must be prepared to
take manual control because the present baseline has no configured fence.

## Normal field procedure

1. Use an open area; inspect the airframe, props, battery, GPS, and RC link.
   Keep the transmitter in hand.
2. Connect the flight battery and switch on the transmitter. Confirm CH8 moves
   and that MANUAL and LAND are available. Leave CH8 in **MANUAL** through every
   diagnostic and service-restart step; GUIDED is selected only at step 4 when
   the aircraft is deliberately ready to launch.
3. With propellers removed if troubleshooting or making configuration changes,
   run the read-only readiness check. The mission daemon must be stopped first
   because only one process may own the MAVLink serial connection:

   ```bash
   systemctl stop ardupilot-mission.service
   /home/aahswarm/px4_mavsdk_env/bin/python preflight_check.py
   systemctl start ardupilot-mission.service
   ```

   Proceed only when it reports `READY TO FLY` and ArduPilot reports no pre-arm
   blocker. The check sends no arm, mode, or motion command. The final service
   start is safe only while CH8 remains MANUAL.
4. For the tested auto-start setup, place CH8 in GUIDED while the aircraft is on
   the ground and ready for its mission, then start/restart the service:

   ```bash
   systemctl restart ardupilot-mission.service
   journalctl -fu ardupilot-mission.service
   ```

   It will make one flight after the FC is ready. Do not use this restart merely
   to inspect logs while the aircraft is flight-ready in GUIDED.
5. During flight, CH8 to MANUAL is the immediate pilot takeover; CH8 to LAND
   requests the safety landing. After touchdown, verify disarm before handling
   the aircraft.

## If it stops flying: safe recovery order

1. **Aircraft safety first.** If it is airborne, use CH8 MANUAL to take control
   or CH8 LAND to land. Do not restart the Pi service or unplug USB mid-air.
2. After disarm, remove props for all diagnosis or configuration work.
3. Set CH8 to **MANUAL** before restarting or starting the service. Check the
   daemon state and its recent reason for refusing/aborting a mission:

   ```bash
   systemctl show ardupilot-mission.service -p ActiveState -p SubState -p MainPID --no-pager
   journalctl -u ardupilot-mission.service --since '20 minutes ago' --no-pager -o short-iso
   ```

4. Run the read-only preflight sequence from step 3. It identifies battery,
   GPS, EKF, home point, RC, compass, and the FC's own `PreArm:` messages.
5. If a stale/latching fault is reported after correcting its physical cause,
   use the controlled reset script (still with props removed):

   ```bash
   systemctl stop ardupilot-mission.service
   /home/aahswarm/px4_mavsdk_env/bin/python make_flight_ready.py
   systemctl start ardupilot-mission.service
   ```

   It reapplies the baseline parameters, can reboot the flight controller to
   clear a latched failsafe, waits for GPS/EKF, and never arms or takes off.
   Keep CH8 in MANUAL until the final command has completed; the restarted
   service otherwise sees GUIDED and may initiate its configured auto-start.
6. If the blocker is compass-related, calibrate outdoors and well away from
   steel, vehicles, reinforced concrete, and current-carrying equipment:

   ```bash
   systemctl stop ardupilot-mission.service
   /home/aahswarm/px4_mavsdk_env/bin/python compass_calibrate.py
   systemctl start ardupilot-mission.service
   ```

   Rotate the disarmed aircraft slowly through all six faces. Re-run
   `preflight_check.py` after it completes.
7. If no flight controller is found, inspect the stable serial link rather than
   guessing a `ttyACM` number:

   ```bash
   ls -l /dev/serial/by-id /dev/ttyACM*
   ```

## Important constraints

- Never run `ardupilot_mission.py`, `preflight_check.py`, compass calibration,
  or the flight recorder against the flight controller at the same time. They
  intentionally share a lock because multiple MAVLink readers corrupt the data
  stream.
- Do not run the passive flight recorder and the autonomous mission daemon
  together. See `FLIGHT_RECORDER.md` for its separate manual-flight procedure.
- `AUTOSTART_WITHOUT_RC` must stay `0` for real flights. It permits unattended
  arming and is not part of this tested baseline.
- `BENCH_MODE=1` is propellers-off only; it disables ArduPilot pre-arm checks.
- Preserve the baseline source files before experimenting:
  `ardupilot_mission.py`, `start_ardupilot_mission.sh`,
  `ardupilot-mission.service`, `make_flight_ready.py`, and
  `preflight_check.py`.

## Evidence to collect before changing the baseline

For any future unexpected abort, save the service journal above plus the
mission log (`ardupilot_mission.log`). For a manual/PosHold diagnostic flight,
use the passive recorder described in `FLIGHT_RECORDER.md`. Capture the exact
`PreArm:` text, CH8 position, battery voltage, GPS fix/satellites, and whether
the flight controller had just been power-cycled. That is enough to distinguish
most configuration, RC, compass/EKF, battery, and USB-link failures without
weakening the safety setup.

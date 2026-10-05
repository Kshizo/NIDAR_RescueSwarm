# Jerk/snap-limited trajectory + LQR tracking: data requirements

What is needed to add a minimum-jerk / minimum-snap trajectory generator and an
LQR tracking controller to the raster mission, and to tune both to this airframe.
The first three sections matter most.

## 1. Flight controller onboard logs (most important)

The companion-side logs (`fc_telemetry.log`, the mission log) can't support this.
They record position at ~10 Hz and leave out attitude, motor outputs, vibration
and the controller's own targets. Needed instead: the **onboard `.bin` log from
the FC's SD card** for the 2026-10-05 flight and for every new test flight.

Before flying, set:

- `LOG_BITMASK` to include IMU, ATT, RATE, PSCN/PSCE/PSCD (position-controller
  targets vs. actual), CTUN, NTUN, RCOU (motor outputs), VIBE, BAT, GPS and XKF (EKF)
- `INS_LOG_BAT_MASK=1` for vibration spectra

**Why:** LQR needs a model of how the aircraft responds to commands: lag,
acceleration limits and damping. That can only be measured from
commanded-vs-actual data at 50 Hz or more.

## 2. Full parameter file

Export all parameters from Mission Planner (Config → Full Parameter List → Save)
as a `.param` file. Specifically needed:

- `PSC_*`: the position/velocity controller gains LQR will sit on top of
- `WPNAV_*` and `GUID_*`: speed, acceleration and jerk limits
- `ATC_*`: attitude rate limits, `ANGLE_MAX`, and `MOT_THST_HOVER`

**Why:** ArduPilot already runs its own position and velocity loops, so the LQR
has to be designed around them rather than fight them.

## 3. System-identification test flights (about 5 minutes, in GUIDED)

Fly these short maneuvers with full logging on, each along both the North and
East axes:

- **Velocity steps:** hover → 0.5 m/s → stop, then hover → 1.0 m/s → stop
- **Short legs:** a 1 m step and a 4 m leg, matching the raster's step and pass lengths
- **Hover hold:** 30 s of hover, to measure disturbance and noise
- **Optional:** a chirp or frequency sweep in ALTHOLD (ArduPilot's `SID_*`
  system-ID mode), for a proper frequency-domain model

A "sysid" mode can be added to the mission script to fly these automatically
inside the polygon.

## 4. Airframe physical data

- All-up weight with battery, Pi and camera
- Frame size, motor and propeller model, and battery (4S, capacity)
- Approximate arm length and where the heavy items are mounted (affects inertia)
- ArduPilot firmware version (Mission Planner's Messages tab)

## 5. Optimization goal (ranked)

Each goal produces different LQR weights and a different trajectory order:

1. **Shortest mission time.** The 2026-10-05 flight wastes ~3 s on each 1 m
   step because the aircraft never reaches speed.
2. **Smoothest motion for the camera** (minimum jerk or snap means less blur).
3. **Most accurate pass-end positions and spacing** for full coverage.
4. **Lowest battery use.**

Plus hard limits: maximum speed, maximum tilt angle, and minimum allowed edge
clearance.

## 6. Environment and computer

- Typical wind at the site, and which direction
- Pi model, and whether it can run other loads alongside a 20–50 Hz control loop
- The Pi undervoltage problem (2026-10-05, 20:45:40) fixed first. A controller
  loop running on the companion makes a Pi brownout in flight much more dangerous.

## Recommended design

Generate the jerk/snap-limited trajectory on the Pi and send **position +
velocity + acceleration setpoints** to ArduPilot in GUIDED
(`SET_POSITION_TARGET_LOCAL_NED` with feed-forward), with a small LQR
correction for cross-track and along-track error. ArduPilot's inner loops stay
in charge of the motors. Replacing them would be riskier and would need far
more data.

Current performance from the 2026-10-05 flight is already good: 0.22 m maximum
pass-end overshoot and 0.13 m maximum cross-track. The realistic gain is time
and smoothness (roughly 180 s → ~120 s for the pattern), not accuracy.

**Minimum to start:** the 2026-10-05 `.bin` log, the `.param` file, and the
ranked goal. With those, the trajectory generator and a first LQR design can be
drafted and tested in SITL before the system-identification flight is used to
tune it.

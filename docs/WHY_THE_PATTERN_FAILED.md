# Why the 2026-09-15 raster flight looked like a straight line out and back

Analysis of `ardupilotPatterenTest/ardupilot_raster_mission.log`, the
`2026-09-15 20:26:55` daemon run. This folder is the fixed copy; the original is
untouched.

## What actually happened

The daemon flew twice. Both flights were ended by the pilot on CH8, not by a
safety check and not by a crash.

| | flight 1 | flight 2 |
|---|---|---|
| GUIDED / takeoff | 20:27:31 | 20:33:46 |
| pattern start | 20:27:42 | 20:33:56 |
| pilot took over (CH8 -> LAND) | 20:27:51 | 20:34:55 |
| how far it got | 9 s into the transit | 35 s into pass 1 of 6 |

Flight 2 is the one worth reading. Its origin, pattern and abort:

```
20:33:56  [RASTER] Geofence origin (post-takeoff hover): N=+1.76m, E=+1.96m.
20:33:56  [RASTER] 12 waypoints converted into local NED; the first is 11.60m from the origin.
20:33:56  [RASTER PHASE] TRANSIT_TO_RASTER: flying 11.60m from the origin to WP00 (pass 1 pass_start).
20:34:20  [RASTER PHASE] TRANSIT_TO_RASTER complete. Aircraft is +0.39m from the polygon edge.
20:34:20  [RASTER] --- pass 1/6, direction A->B, step offset 0.66m ---
20:34:20  [RASTER] -> WP01 (pass 1 pass_end): target N=-6.50m E=+4.73m (19.82m away)
20:34:55  [CH8] Switch state changed: GUIDED -> LAND (raw=2000)
```

Nothing is logged between 20:34:20 and the abort at 20:34:55.

## The pattern was correct. It was unrecognisable.

The planned waypoints, converted through the flight's own anchor
(`13.3457864, 74.7940113 <-> N=+1.76 E=+1.96`):

```
                        N        E     from origin
WP00  pass 1 start  +13.35    +2.27       11.60      <- transit target
WP01  pass 1 end     -6.50    +4.72        8.71
WP02  pass 2 start   -6.76    +3.82        8.72      <- 0.94m from WP01: a real raster step
WP03  pass 2 end    +13.25    +1.34       11.51
...
WP09  pass 5 end     +0.51    +0.11        2.23      <- 2.2m from the origin
```

The geometry was fine. Every one of the 12 waypoints passed containment
(`12/12 waypoints and 11 segments verified inside the inset polygon`), the passes
alternated correctly, and pass 2 began 0.94 m from where pass 1 ended. Had the
flight continued it would have drawn a correct lawnmower.

The problem is the first 63 seconds of it:

- the takeoff point sat **inside** the polygon, ~11 m from corner A;
- the transit flew **11.6 m north** to corner A — the far end of pass 1;
- pass 1 then flew **20.0 m south**, back down the same line, directly over the
  takeoff point.

Out and back, with no turn in between. The first turn was due at
**t+63 s**. The pilot took over at **t+58 s**, five seconds short of it.

That is the whole failure. Three things caused it, and all three are fixed here.

---

## Cause 1 — the entry point was never chosen, it was always WP00

`fly_raster_pattern` flew `targets[0]`, the corner-A end of pass 1, whatever the
origin was. From this origin that is the single worst of the four available
entries, and it is the one that puts the transit and pass 1 on the same line.

A boustrophedon can be entered from any of four ends and covers identical ground
each time. Measured from the real origin:

```
as planned (first pass, start end)     transit  11.60m  return  11.88m   <- what flew
as planned (first pass, far end)       transit   8.71m  return   7.90m
reversed   (last pass, far end)        transit  11.88m  return  11.60m
reversed   (last pass, start end)      transit   7.90m  return   8.71m
```

**Fix:** `orient_targets_for_origin()` in `ardupilot_raster_mission.py` evaluates
all four and flies the cheapest. It only reorders planned waypoints — it never
moves one — so the ground containment proof still holds, and the whole-route
outer-geofence check and time budget now run on the order actually chosen.

Reversing the pass order and swapping each pass's endpoints are applied as a
coupled pair. Reversing the order alone would make the aircraft jump a full pass
length between every pair instead of one spacing.

## Cause 2 — the passes ran along the 20 m side

`RASTER_AXIS` did not exist; the sweep was hardwired to A->B. On this plot AB is
the 20.30 m side, so the plan was 6 passes of ~20 m — 40 s of straight flight per
leg at 0.50 m/s. Pass length is what an observer reads as "is this a raster, or
is it just flying away?", and 40 s is far past the point where the answer stops
being obvious.

**Fix:** `RASTER_AXIS` = `long` | `short` | `ab` | `ad`, resolved by
`resolve_sweep_axis()`. `long` reproduces the old behaviour exactly. The launcher
now sets **`short`**, which sweeps the 5.99 m side instead:

| | long (old) | short (new) |
|---|---|---|
| passes | 6 | 21 |
| pass length | 4.2 – 20.5 m | 3.3 – 5.5 m |
| first turn at | t+57 s | **t+24 s** |
| turn interval | ~40 s | ~10 s |
| worst transition between passes | **8.00 m** | 0.98 m |
| whole route from the 09-15 origin | 126.0 m / 252 s | 132.4 m / 265 s |

`short` costs **13 seconds**. It does not cost the turn-count advantage you would
expect, because the plot is a trapezoid (BC=3.71 m, DA=5.99 m): with the long
sweep the last passes clip to stubs near corner A and one transition runs 8.0 m
across ground already covered. The short sweep's transitions are all one spacing.

Implementation note: the frame rotation is `u' = v, v' = -u`, determinant +1.
`validate_polygon` requires `signed_area(polygon_uv) > 0` and `edge_halfplanes`
assumes counter-clockwise winding, so only a handedness-preserving transform
keeps both true. Swapping `u` and `v` (determinant -1) would mirror the polygon
and reject every valid area.

## Cause 3 — 35 seconds of silence

The log recorded `-> WP01` and then nothing until the pilot intervened. A 20 m
pass at 0.50 m/s is 40 s, so an entire leg fitted between two log lines. There
was no way to distinguish an aircraft tracking correctly from one stuck, drifting
or flying the wrong way, which is exactly the judgement the pilot had to make.

**Fix:** `RASTER_PROGRESS_INTERVAL_S` (default 3.0 s). Every leg now reports:

```
[RASTER PROGRESS] raster WP01 (pass 1 pass_end):  42.3% (8.45m of 20.01m),
  11.56m to run, 18s of 120s, N=+5.12m E=+3.41m, cross-track 0.07m,
  +1.83m from the polygon edge
```

Cross-track is measured against the straight line from where the leg began to its
target, so a correctly tracking leg reads near zero and one being pushed off by
wind does not.

The full route in flight order is also printed before anything moves, so the
pilot can match each leg against the plan as it happens.

---

## What was NOT wrong

Worth stating, because all of it was suspected and none of it holds up:

- **Not the geometry.** 12/12 waypoints and 11/11 segments contained, worst
  clearance +0.200 m, exactly as designed.
- **Not the anchor.** `samples 0ms apart, fix=3, sats=21` — GPS and local NED were
  captured simultaneously with a good fix.
- **Not a safety abort.** No breach, no timeout, no EKF or battery event. The only
  trigger was `CH8 -> LAND (raw=2000)`.
- **Not the alternation.** Pass 2 started 0.94 m from where pass 1 ended, against
  a 0.928 m planned spacing.
- **Not the speed or the budget.** 366 s expected against a 650 s ceiling.

---

## A fourth problem, found while verifying: the test rig was measuring a retired area

Not a cause of the flight failure, but it is why the failure had no chance of
being caught on the ground, so it is fixed here too.

`run_simulation.sh` places the simulated aircraft at four hand-computed GPS
positions, each chosen to sit in a specific relationship to the two polygons.
Those positions are constants; the polygons are not. **Both polygons were
replaced on 2026-09-15** — inner 56.8 m² -> 99.90 m², outer 204.26 m² ->
217.93 m² — and the fixtures were not moved with them.

Measured against the polygons actually configured today:

| fixture | intended | documented | actually measured |
|---|---|---|---|
| `INSIDE_BOTH` | inside both | outer +3.90 m | outer +2.54 m — ok |
| `OUTSIDE_INNER` | outside inner only | outer +3.08 m | **outer +0.44 m — refused** |
| `OUTSIDE_OUTER` | outside outer | outer -3.04 m | outer -4.63 m — still outside |
| `IN_MARGIN` | in the 0.50 m band | outer +0.39 m | **outer -1.15 m — fully outside** |

Consequences:

- **TEST 2** asserts the mission is *allowed* from a position outside the inner
  polygon but safely inside the outer. Its fixture had drifted into the 0.50 m
  outer safety margin, so the mission was refused at the origin check and the
  test failed. Confirmed against the **original, unmodified** tree — this is not
  a regression from these changes.
- **TEST 4** tests the margin band. Its fixture had drifted outside the outer
  polygon entirely, so it was passing for the wrong reason.
- `FLIGHT_FIXTURE` used 5.0 m spacing, which gave 3 passes on the old area but
  only **2** on this one — so every `All 3 passes complete` assertion in the suite
  was checking a string the daemon could not print.
- TEST 0 asserted `Passes: 33` and a `140.x m` pattern; on this area the
  configured 0.40 m spacing gives **14 passes / 240.8 m**.

**Fix:** all four positions re-derived from the current polygons, `FLIGHT_FIXTURE`
moved to 2.0 m spacing (3 passes, 52.6 m), and TEST 0's expectations corrected.

**Fix so it cannot drift silently again:** `test_simulation_fixtures.py` asserts
each fixture's relationship to the live polygons directly, checks the pass counts
the suite asserts against what the planner actually produces, and prints
re-derived replacements for anything that has drifted. Run it after any change to
`DEFAULT_CORNERS` or `DEFAULT_OUTER_CORNERS`.

A fixture that silently stops testing what its name says is worse than no
fixture. The four positions had been correct when they were written; nothing
told anyone when they stopped being correct.

## Verification

- All four `RASTER_AXIS` settings produce fully contained plans (0 containment
  problems, worst clearance +0.2000 m).
- `long` and `ab` reproduce the pre-fix plan exactly: 6 passes, 109.8 m — the same
  numbers the 09-15 log printed.
- Reordering preserves the boustrophedon: every pass has exactly one start and one
  end, passes renumber 1..N in flight order, and transitions stay at one spacing.
- `python test_raster_entry_and_axis.py` — 43 checks covering containment under
  every axis setting, winding preservation, entry selection, boustrophedon
  integrity, and the specific 2026-09-15 numbers.
- `python test_simulation_fixtures.py` — the fixture-drift guard above.
- `./run_simulation.sh raster` — the seventeen scenario tests, including the
  boundary-breach and telemetry-fault aborts.

## Before the next flight

1. `RASTER_AXIS=short` is set in `start_ardupilot_raster_mission.sh`. Expect
   **21 passes**, the first turn about **24 s** after the pattern starts, and a
   turn roughly every 10 s after that.
2. Read the `[RASTER ENTRY]` and `[RASTER ROUTE]` block in the log before arming.
   It lists every leg in flight order with its length.
3. `[RASTER PROGRESS]` every 3 s is the signal that the leg is tracking. If
   cross-track grows past a few tens of centimetres, that is a real problem and
   worth taking over for.
4. To install this folder as the running daemon:
   ```
   sudo cp /home/aahswarm/ardupilotRasterFix/ardupilot-raster-mission.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl restart ardupilot-raster-mission.service
   ```
   The unit still `Conflicts=ardupilot-mission.service`, and both daemons still
   take `/tmp/ardupilot_mission.lock`.

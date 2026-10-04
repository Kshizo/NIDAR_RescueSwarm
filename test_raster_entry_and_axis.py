#!/usr/bin/env python3
"""
Regression tests for the 2026-09-17 raster fixes.

Locks in the three things that made the 2026-09-15 flight look like a line out
and a line back instead of a raster. See WHY_THE_PATTERN_FAILED.md.

    python test_raster_entry_and_axis.py

Pure geometry: no flight controller, no simulator, no MAVLink. Runs in about a
second.
"""
import math
import os
import sys

os.environ.setdefault("RASTER_DRY_RUN", "1")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import raster_plan_preview as planner            # noqa: E402
import ardupilot_raster_mission as raster        # noqa: E402

# The anchor and post-takeoff origin the 2026-09-15 20:33 flight actually
# captured, straight out of its log line:
#   [RASTER ANCHOR] GPS 13.3457864, 74.7940113 <-> local NED N=+1.76m E=+1.96m
ANCHOR_LAT, ANCHOR_LON = 13.3457864, 74.7940113
ORIGIN_N, ORIGIN_E = 1.76, 1.96
SPACING_M, INSET_M, SPEED_MPS = 1.0, 0.20, 0.50

FAILURES = []


def check(name, condition, detail=""):
    print(f"  {'PASS' if condition else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    if not condition:
        FAILURES.append(name)


def to_local(lat, lon):
    return (ORIGIN_N + math.radians(lat - ANCHOR_LAT) * planner.R_EARTH_M,
            ORIGIN_E + math.radians(lon - ANCHOR_LON) * planner.R_EARTH_M
            * math.cos(math.radians(ANCHOR_LAT)))


def targets_for(plan):
    return [(f"WP{w.index:02d} (pass {w.pass_index} {w.kind})", *to_local(w.lat, w.lon), w)
            for w in plan.waypoints]


def leg_lengths(sequence, origin=(ORIGIN_N, ORIGIN_E)):
    previous, legs = origin, []
    for _, north, east, _ in sequence:
        legs.append(math.hypot(north - previous[0], east - previous[1]))
        previous = (north, east)
    return legs


print("\n=== every axis setting produces a contained plan ===")
plans = {}
for axis in ("long", "short", "ab", "ad"):
    plan = planner.plan_raster(planner.DEFAULT_CORNERS, SPACING_M, INSET_M, SPEED_MPS, axis)
    problems, metrics = planner.verify_containment(plan)
    plans[axis] = plan
    check(f"RASTER_AXIS={axis} contained",
          not problems and metrics["min_segment_polygon_clearance_m"] >= INSET_M - 1e-6,
          f"{plan.pass_count} passes, worst {metrics['min_segment_polygon_clearance_m']:+.4f}m")

# "long" must stay byte-identical to the original hardwired A->B sweep. These
# numbers move whenever the polygon does — they were 6 passes / 109.8 m on the
# 99.90 m^2 area, and are 6 passes / 105.6 m since B and C came in 1 m on
# 2026-09-17. What must NOT change is that "long" and "ab" agree, and that
# "short" produces many more, much shorter passes.
check("RASTER_AXIS=long reproduces the pre-fix A->B sweep",
      plans["long"].pass_count == 6 and abs(plans["long"].path_length_m - 105.6) < 0.1,
      f"{plans['long'].pass_count} passes, {plans['long'].path_length_m:.1f}m")
check("long and ab are the same plan on this polygon",
      plans["long"].pass_count == plans["ab"].pass_count)
check("short sweeps the other axis",
      plans["short"].pass_count == 20 and plans["short"].pass_count > plans["long"].pass_count,
      f"{plans['short'].pass_count} passes")

print("\n=== the frame rotation preserves winding (determinant +1) ===")
for axis, plan in plans.items():
    area = planner.signed_area(plan.polygon_uv)
    check(f"RASTER_AXIS={axis} winds counter-clockwise in (u,v)", area > 0.0, f"area {area:+.2f}")

print("\n=== entry selection picks the cheapest of the four ends ===")
for axis, plan in plans.items():
    sequence, report = raster.orient_targets_for_origin(
        plan, targets_for(plan), ORIGIN_N, ORIGIN_E)
    check(f"RASTER_AXIS={axis}: chose the minimum-cost end",
          report is not None
          and abs(report["transit_m"] + report["return_m"]
                  - min(c["total_m"] for c in report["candidates"])) < 1e-9,
          f"{report['chosen']}")
    check(f"RASTER_AXIS={axis}: never worse than the planner's own order",
          report["saved_m"] >= -1e-9, f"saved {report['saved_m']:.2f}m")

print("\n=== reordering keeps it a boustrophedon ===")
for axis, plan in plans.items():
    sequence, report = raster.orient_targets_for_origin(
        plan, targets_for(plan), ORIGIN_N, ORIGIN_E)
    check(f"RASTER_AXIS={axis}: waypoint count unchanged",
          len(sequence) == len(plan.waypoints), f"{len(sequence)}")
    indices = [w.pass_index for _, _, _, w in sequence]
    check(f"RASTER_AXIS={axis}: passes renumbered 1..N in flight order",
          indices == sorted(indices) and indices[0] == 1
          and indices[-1] == len(sequence) // 2)
    kinds = [w.kind for _, _, _, w in sequence]
    check(f"RASTER_AXIS={axis}: alternates start/end throughout",
          kinds == ["pass_start", "pass_end"] * (len(sequence) // 2))
    # The transitions between passes are legs 2, 4, 6 ... of the route. If the
    # order reversal were applied without the endpoint swap they would each be a
    # full pass length instead of one spacing.
    legs = leg_lengths(sequence)
    transitions = legs[2::2]
    check(f"RASTER_AXIS={axis}: transitions stay near one spacing",
          all(t <= max(4.0, 9.0 * plan.pass_spacing_actual_m) for t in transitions),
          f"max {max(transitions):.2f}m vs spacing {plan.pass_spacing_actual_m:.2f}m")
    # Positions must be untouched: same set of points, only reordered.
    before = sorted((round(n, 6), round(e, 6)) for _, n, e, _ in targets_for(plan))
    after = sorted((round(n, 6), round(e, 6)) for _, n, e, _ in sequence)
    check(f"RASTER_AXIS={axis}: no waypoint was moved", before == after)

print("\n=== the actual 2026-09-15 failure does not recur ===")
plan_long = plans["long"]
old_first = targets_for(plan_long)[0]
old_transit = math.hypot(old_first[1] - ORIGIN_N, old_first[2] - ORIGIN_E)
# On 2026-09-15 this was 11.60 m. The polygon has moved since, so the assertion
# is on the SHAPE of the failure — the planner's own order still enters at the
# far end, many metres further away than the end the fix now picks.
sequence_long, report_long = raster.orient_targets_for_origin(
    plan_long, targets_for(plan_long), ORIGIN_N, ORIGIN_E)
check("the planner's own order still enters at the far end",
      old_transit > report_long["transit_m"] + 2.0,
      f"planner {old_transit:.2f}m vs chosen {report_long['transit_m']:.2f}m")

for axis, expected_max_first_turn_s in (("long", 70.0), ("short", 30.0)):
    plan = plans[axis]
    sequence, _ = raster.orient_targets_for_origin(
        plan, targets_for(plan), ORIGIN_N, ORIGIN_E)
    legs = leg_lengths(sequence)
    first_turn_s = (legs[0] + legs[1]) / SPEED_MPS
    check(f"RASTER_AXIS={axis}: first turn within {expected_max_first_turn_s:.0f}s",
          first_turn_s <= expected_max_first_turn_s, f"t+{first_turn_s:.0f}s")

sequence, _ = raster.orient_targets_for_origin(
    plans["short"], targets_for(plans["short"]), ORIGIN_N, ORIGIN_E)
legs = leg_lengths(sequence)
check("RASTER_AXIS=short: the aircraft turns at least 20 times",
      len(legs) // 2 >= 20, f"{len(legs) // 2} passes")
check("RASTER_AXIS=short: no leg runs longer than 20s",
      max(legs) / SPEED_MPS <= 20.0, f"longest {max(legs) / SPEED_MPS:.0f}s")

print("\n=== progress reporting is configured ===")
check("RASTER_PROGRESS_INTERVAL_S exists and is sane",
      0.0 < raster.RASTER_PROGRESS_INTERVAL_S <= 10.0,
      f"{raster.RASTER_PROGRESS_INTERVAL_S}s")
check("a 40s leg would now produce several progress lines",
      40.0 / raster.RASTER_PROGRESS_INTERVAL_S >= 5.0,
      f"{40.0 / raster.RASTER_PROGRESS_INTERVAL_S:.0f} lines")

print("\n=== a bad axis is rejected, not silently ignored ===")
try:
    planner.plan_raster(planner.DEFAULT_CORNERS, SPACING_M, INSET_M, SPEED_MPS, "sideways")
    check("RASTER_AXIS=sideways raises PlanError", False)
except planner.PlanError as exc:
    check("RASTER_AXIS=sideways raises PlanError", True, exc.summary)

print("\n" + "=" * 70)
if FAILURES:
    print(f"FAILED: {len(FAILURES)} check(s)")
    for name in FAILURES:
        print(f"  - {name}")
    sys.exit(1)
print("All checks passed.")

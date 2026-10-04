#!/usr/bin/env python3
"""
Checks the simulation fixtures still measure what they claim to measure.

WHY THIS EXISTS (2026-09-17)
    run_simulation.sh places the simulated aircraft at four hand-computed GPS
    positions, each chosen to sit in a specific relationship to the two
    configured polygons — inside both, outside the inner only, outside the outer,
    and inside the outer's 0.50 m safety margin.

    Those positions are constants. The polygons are not: DEFAULT_CORNERS and
    DEFAULT_OUTER_CORNERS were both replaced on 2026-09-15 and the fixtures were
    left where they were. OUTSIDE_INNER drifted into the outer safety margin, so
    TEST 2 — "origin outside INNER, inside OUTER: must be allowed" — was refused
    at the origin check. IN_MARGIN drifted outside the outer polygon entirely, so
    TEST 4 passed for the wrong reason. Neither failure said anything about the
    aircraft; both were the test rig measuring a retired mission area.

    A fixture that silently stops testing what it names is worse than no fixture,
    so this asserts the relationships directly and fails loudly when a polygon
    change invalidates one.

    python test_simulation_fixtures.py

Run it after ANY change to either polygon. It prints re-derived replacements for
whatever has drifted.
"""
import math
import os
import re
import sys

os.environ.setdefault("RASTER_DRY_RUN", "1")
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import raster_plan_preview as planner   # noqa: E402

SUITE = os.path.join(HERE, "run_simulation.sh")
MARGIN = planner.OUTER_GEOFENCE_MARGIN_M
FAILURES = []

reference = planner.GpsPoint(*planner.DEFAULT_CORNERS[0], "A")


def ring(corners):
    r = planner.ring_ne(corners, reference)
    return r if planner.ccw(r) else r[::-1]


OUTER_PLANES = planner.edge_halfplanes(ring(planner.DEFAULT_OUTER_CORNERS))
INNER_PLANES = planner.edge_halfplanes(ring(planner.DEFAULT_CORNERS))


def clearances(lat, lon):
    ne = planner.gps_to_ne(planner.GpsPoint(lat, lon), reference)
    return (planner.clearance((ne.north, ne.east), OUTER_PLANES),
            planner.clearance((ne.north, ne.east), INNER_PLANES))


def read_fixture(name):
    text = open(SUITE).read()
    m = re.search(rf"^{name}=\(SIM_HOME_LAT=([0-9.]+) SIM_HOME_LON=([0-9.]+)\)",
                  text, re.M)
    if not m:
        FAILURES.append(f"{name} not found in run_simulation.sh")
        return None
    return float(m.group(1)), float(m.group(2))


def check(name, ok, detail):
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<16} {detail}")
    if not ok:
        FAILURES.append(name)


# (fixture, what it must satisfy, human description)
RULES = (
    ("INSIDE_BOTH",
     lambda outer, inner: outer >= MARGIN + 1.0 and inner > 0.5,
     f"inside both: outer >= {MARGIN + 1.0:.2f}m, inner > 0.50m"),
    ("OUTSIDE_INNER",
     lambda outer, inner: outer >= MARGIN + 0.4 and inner < -0.4,
     f"outside inner, safely inside outer: outer >= {MARGIN + 0.4:.2f}m, inner < -0.40m"),
    ("OUTSIDE_OUTER",
     lambda outer, inner: outer < 0.0,
     "outside the outer polygon: outer < 0.00m"),
    ("IN_MARGIN",
     lambda outer, inner: 0.0 < outer < MARGIN,
     f"inside the safety band: 0.00m < outer < {MARGIN:.2f}m"),
)

print(f"\nPolygons in use: inner {abs(planner.signed_area(ring(planner.DEFAULT_CORNERS))):.2f} m^2, "
      f"outer {abs(planner.signed_area(ring(planner.DEFAULT_OUTER_CORNERS))):.2f} m^2, "
      f"safety margin {MARGIN:.2f} m\n")
print("=== simulated takeoff positions still sit where the tests assume ===")
drifted = []
for name, rule, description in RULES:
    position = read_fixture(name)
    if position is None:
        continue
    outer, inner = clearances(*position)
    ok = rule(outer, inner)
    check(name, ok, f"outer {outer:+6.2f}m  inner {inner:+6.2f}m   ({description})")
    if not ok:
        drifted.append((name, rule))

# The flight fixture's pass count has to be a count the daemon can actually print.
text = open(SUITE).read()
spacing = float(re.search(r"FLIGHT_FIXTURE=\(RASTER_PASS_SPACING_M=([0-9.]+)", text).group(1))
expected = re.search(r'expect "All (\d+) passes complete"', text)
expected_passes = int(expected.group(1)) if expected else None
plan = planner.plan_raster(planner.DEFAULT_CORNERS, spacing, planner.EDGE_INSET_M, 0.50, "long")

print("\n=== the flight fixture produces the pass count the tests assert ===")
check("pass count", plan.pass_count == expected_passes,
      f"FLIGHT_FIXTURE spacing {spacing:.2f}m gives {plan.pass_count} passes; "
      f"the suite asserts 'All {expected_passes} passes complete'")

# TEST 0 asserts the configured spacing is refused on the time budget.
configured = planner.plan_raster(planner.DEFAULT_CORNERS, planner.RASTER_PASS_SPACING_M,
                                 planner.EDGE_INSET_M, 0.50, "long")
print("\n=== TEST 0's budget refusal still describes this area ===")
for label, pattern, actual in (
        ("passes", r'expect "Passes: (\d+)"', configured.pass_count),
        ("pattern length", r'expect "What drives it: (\d+)\\\.\[0-9\]m of pattern"',
         int(configured.path_length_m))):
    m = re.search(pattern, text)
    asserted = int(m.group(1)) if m else None
    check(label, asserted == actual,
          f"suite asserts {asserted}, {planner.RASTER_PASS_SPACING_M:.2f}m spacing gives {actual}")

# Numeric facts about the polygons that the suite asserts as literal strings.
# These are the ones that bit on 2026-09-17: the outer area and the inner-to-outer
# separation were still the retired polygon's 204.26 m^2 and 1.51 m.
print("\n=== polygon numbers the suite asserts as literal strings ===")
nested, outer_metrics = planner.verify_inner_inside_outer(
    plan, planner.DEFAULT_OUTER_CORNERS, MARGIN)

m = re.search(r'expect "Outer safety geofence: area (\d+)\\\."', text)
asserted_area = int(m.group(1)) if m else None
check("outer area", asserted_area == int(outer_metrics["outer_area_m2"]),
      f"suite asserts {asserted_area}, polygon is {outer_metrics['outer_area_m2']:.2f} m^2")

m = re.search(r'expect "Inner inside outer: minimum separation (\d+)\\\.(\d)"', text)
asserted_sep = float(f"{m.group(1)}.{m.group(2)}") if m else None
actual_sep = outer_metrics["min_edge_clearance_m"]
check("inner-outer separation", asserted_sep is not None
      and abs(asserted_sep - math.floor(actual_sep * 10) / 10) < 1e-9,
      f"suite asserts {asserted_sep}, actual {actual_sep:.2f} m")

# The flight fixture must fit the budget it also sets, or every flight test is
# refused before it moves.
# Scoped to the FLIGHT_FIXTURE line: the scenario runner also sets a default
# RASTER_SPEED_MPS earlier in the file, and RASTER_ENV is applied after it, so the
# fixture's value is the one that actually takes effect.
fixture_line = re.search(r"^FLIGHT_FIXTURE=\((.*)\)$", text, re.M).group(1)
speed = float(re.search(r"RASTER_SPEED_MPS=([0-9.]+)", fixture_line).group(1))
ceiling = float(re.search(r"RASTER_TOTAL_TIMEOUT_S=(\d+)", fixture_line).group(1))
# transit + pattern + return, with a generous allowance for the two end legs
route_m = plan.path_length_m + 2 * 12.0
expected_s = route_m / speed * 1.35
print("\n=== the flight fixture fits the budget it sets ===")
check("time budget", expected_s < ceiling,
      f"~{route_m:.0f} m at {speed:.2f} m/s is ~{expected_s:.0f} s against a {ceiling:.0f} s ceiling")

if drifted:
    print("\n=== re-derived replacements ===")
    lat0, lon0 = 13.345800, 74.794000
    for name, rule in drifted:
        best = None
        for i in range(-300, 301):
            for j in range(-300, 301):
                lat, lon = lat0 + i * 2e-6, lon0 + j * 2e-6
                outer, inner = clearances(lat, lon)
                if not rule(outer, inner):
                    continue
                score = min(outer - MARGIN, abs(inner)) if outer > 0 else -outer
                if best is None or score > best[0]:
                    best = (score, lat, lon, outer, inner)
        if best:
            print(f"  {name}=(SIM_HOME_LAT={best[1]:.7f} SIM_HOME_LON={best[2]:.7f})"
                  f"   # outer {best[3]:+.2f}m, inner {best[4]:+.2f}m")
        else:
            print(f"  {name}: no position in the search area satisfies this rule")

print("\n" + "=" * 70)
if FAILURES:
    print(f"FAILED: {len(FAILURES)} check(s)")
    for name in FAILURES:
        print(f"  - {name}")
    sys.exit(1)
print("All simulation fixtures still measure what they claim to.")

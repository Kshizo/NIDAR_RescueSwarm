#!/usr/bin/env python3
"""
Offline polygon raster / lawnmower planner — GEOMETRY ONLY
=========================================================

Takes four GPS corners (read by hand from Google Maps, supplied in perimeter
order A -> B -> C -> D -> A) and produces the lawnmower coverage path that lies
inside that ACTUAL four-corner polygon.

The polygon is the mission area. It is NOT replaced by a bounding rectangle and
NOT replaced by a fitted or inscribed rectangle: each raster line is clipped
against the real inset polygon, so a pass is exactly as long as the polygon
allows at that offset and no longer.

THIS MODULE IS OFFLINE AND INERT.
  * It does not import pymavlink, pyserial, or any MAVLink library.
  * It does not open a serial port, socket, or connection of any kind.
  * It does not arm, take off, set a mode, or emit a single flight command.
  * Standard library only: math, json, argparse, os, sys, dataclasses.

It is also importable. ``ardupilot_raster_mission.py`` imports the geometry from
here rather than reimplementing it, so the path reviewed on a map offline and the
path flown in the air are produced by the same code.

GEOMETRY PIPELINE
-----------------
 1. Project the four GPS corners onto a local flat-earth tangent plane at A.
 2. Build an orthonormal frame: u along A->B (the sweep axis, the direction each
    pass flies), v perpendicular to u and oriented toward D (the step axis).
 3. Validate that A->B->C->D is a simple, convex, non-degenerate polygon.
 4. Inset the polygon inward by EDGE_INSET_M, by clipping it against each edge's
    half-plane shifted inward. This is exact for a convex polygon and collapses
    to an empty polygon (a loud failure) if the inset is too large.
 5. Lay raster lines across the inset polygon's v extent, and clip each line
    against the inset polygon to get that pass's exact u interval.
 6. Alternate direction on successive passes; join consecutive passes with a
    straight transition.
 7. Verify containment of every waypoint AND every point along every segment,
    against both the inset polygon and the original polygon.

WHY STRAIGHT TRANSITIONS ARE SAFE
---------------------------------
The inset polygon is convex, so the straight segment between any two points
inside it is entirely inside it. That is a proof, not an assumption — but the
containment check below still samples every segment numerically rather than
relying on the argument.

WHY PASSES SIT HALF A SPACING IN FROM THE EDGES
-----------------------------------------------
A raster line is the centreline of a covered strip one spacing wide. Placing the
first line ON the boundary would cover only half a strip there, and at the
extreme v the clipped line degenerates to a single polygon vertex — a zero-length
pass. Lines are therefore placed at v_min + (k + 0.5) * spacing, which covers the
full v extent of the inset polygon and never produces a degenerate chord.

The interval count is rounded UP, so the spacing actually flown is always <= the
spacing requested. Rounding down would silently open coverage gaps wider than the
number the operator asked for.

NOT THE GEOFENCE
----------------
Distances printed here are measured from corner A and from the polygon centroid.
Neither is the companion horizontal geofence origin: that origin is the vehicle's
post-takeoff hover position, captured in flight. The mission polygon is where the
aircraft is meant to fly; the geofence is where it is allowed to fly. They are
independent, and sizing the geofence is a flight-side step, not a planning step.

Usage:
    python3 raster_plan_preview.py
    python3 raster_plan_preview.py --spacing 1.5 --inset 0.75
    python3 raster_plan_preview.py --corners "13.34575,74.7940133;13.3458087,74.7939985;13.3458224,74.7940575;13.3457754,74.7940709"

Exit status:
    0  plan produced, fully contained
    3  polygon or parameters invalid  (no path produced)
    4  a waypoint or segment escaped  (no path produced)
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
# Configuration. Planning values only — nothing here reaches an aircraft.
# ---------------------------------------------------------------------------
R_EARTH_M = 6378137.0                  # WGS-84 semi-major axis

# Small first-test values. The area is deliberately about 3 m x 3 m, so the
# pattern is tight and slow: 0.40 m between passes, 0.20 m inset from every edge,
# 0.20 m/s along the passes.
RASTER_PASS_SPACING_M = float(os.getenv("RASTER_PASS_SPACING_M", "0.40"))

# Which side of the polygon the passes run along. See resolve_sweep_axis().
# "long" reproduces the original A->B behaviour whenever AB is the longer side,
# which it is for every area flown so far.
RASTER_AXIS = os.getenv("RASTER_AXIS", "long")
# EDGE_INSET_M is the documented name; RASTER_EDGE_INSET_M is kept as an alias.
EDGE_INSET_M = float(os.getenv("EDGE_INSET_M", os.getenv("RASTER_EDGE_INSET_M", "0.20")))
RASTER_SPEED_MPS = float(os.getenv("RASTER_SPEED_MPS", "0.20"))

# Polygon sanity limits.
#
# These were originally sized for the ~40 m^2 area and rejected anything smaller
# than a 2 m side or 4 m^2. The current first-test polygon is deliberately tiny —
# sides 1.14 to 1.90 m, area 2.56 m^2 — so the floors are lowered to suit it.
# They are still real floors: below roughly 0.8 m a side is shorter than the
# aircraft's own position tolerance plus its turn radius, at which point a raster
# is not a meaningful pattern. They are NOT a safety boundary; containment is the
# geofence and the polygon check in the mission, not these numbers.
RASTER_MIN_SIDE_M = float(os.getenv("RASTER_MIN_SIDE_M", "0.80"))
RASTER_MAX_SIDE_M = float(os.getenv("RASTER_MAX_SIDE_M", "60.0"))
RASTER_MIN_CORNER_SEPARATION_M = float(os.getenv("RASTER_MIN_CORNER_SEPARATION_M", "0.40"))
RASTER_MIN_CONVEX_CROSS = 1e-6         # below this a corner counts as collinear
RASTER_MIN_AREA_M2 = float(os.getenv("RASTER_MIN_AREA_M2", "1.0"))

# A pass shorter than this is dropped rather than flown as a near-zero hop.
RASTER_MIN_PASS_LENGTH_M = float(os.getenv("RASTER_MIN_PASS_LENGTH_M", "0.30"))

# Containment: how finely segments are sampled, and how much clearance a sample
# must keep from the inset polygon boundary to count as contained.
RASTER_SAMPLE_STEP_M = float(os.getenv("RASTER_SAMPLE_STEP_M", "0.05"))
RASTER_CONTAINMENT_EPS_M = float(os.getenv("RASTER_CONTAINMENT_EPS_M", "0.001"))

GEOJSON_OUTPUT = os.getenv("RASTER_GEOJSON", "raster_preview.geojson")

# ---------------------------------------------------------------------------
# The two polygons
# ---------------------------------------------------------------------------
# INNER — the raster / search area, perimeter order A -> B -> C -> D.
# About 99.9 m^2. DERIVED, not surveyed separately: it is the OUTER plot below
# inset 2.0 m on every edge, so the two polygons are guaranteed consistent.
#
# Corner order starts at the plot's NE corner so the SWEEP edge A->B is the
# long (20.3 m) side and the STEP edge is the short one. That gives 6 long
# passes instead of 21 short ones for the same coverage: same path length, but
# 5 turns instead of 20, and turn settling is what actually costs time in
# flight. Rotating this order by one corner flips the pattern 90 degrees.
#
# This replaces an earlier hand-surveyed inner (sides 2.27 / 8.07 / 7.77 /
# 11.72 m, 43.6 m^2) that was 0.55 m WIDER than the outer plot's 10.24 m
# corridor and so could not be contained at any position. Its corner B was also
# near-collinear (146 deg) with a 2.27 m side, below the 4-8 m stated GPS
# accuracy of the readings it came from.
#
# Earlier areas are retired: the ~40 m^2 first area, the ~2.56 m^2 3 m x 3 m
# area, the ~56.8 m^2 4.7 x 13.4 m area, and the 43.6 m^2 area above.
# 2026-09-17: corners B and C pulled 1.0 m further into the plot at the pilot's
# request, which retracts the narrow BC end of the area. B moved 1.01 m, C moved
# 1.03 m, both perpendicular to the BC edge, so the AB and CD sides stay straight
# and the corner angles barely move (B 98.6 -> 98.1 deg, C 87.7 -> 88.2 deg).
# Area 99.90 -> 96.03 m^2; AB 20.30 -> 19.29 m; CD 21.05 -> 20.03 m.
# Previous values, if this needs reverting:
#     B (13.345712, 74.794043)
#     C (13.345703, 74.794010)
DEFAULT_CORNERS = (
    (13.345893, 74.794020),
    (13.345721, 74.794042),
    (13.345712, 74.794008),
    (13.345887, 74.793965),
)

# OUTER — the companion-side horizontal safety geofence, perimeter order
# G1 -> G2 -> G3 -> G4. Roughly 21.2 m x 9.6 m, about 204 m^2, containing the
# inner polygon with about 1.5 m to spare at the tightest point.
#
# This REPLACES the circular geofence used up to now. The circle was centred on
# the post-takeoff hover origin and therefore moved with the aircraft; this is
# fixed to the ground, which is what a containment boundary should be. It is
# still companion-side logic and is NOT an ArduPilot FENCE_* fence.
DEFAULT_OUTER_CORNERS = (
    (13.345903, 74.793942),
    (13.345913, 74.794036),
    (13.345699, 74.794063),
    (13.345680, 74.793997),
)

# How far INSIDE the supplied outer polygon the effective runtime boundary sits.
# The GPS coordinates above are never altered; this margin is applied to them.
OUTER_GEOFENCE_MARGIN_M = float(os.getenv("OUTER_GEOFENCE_MARGIN_M", "0.50"))

BAR = "=" * 78
RULE = "-" * 78


class PlanError(Exception):
    """Raised when the polygon or the parameters cannot produce a safe path."""

    def __init__(self, summary: str, problems=None):
        super().__init__(summary)
        self.summary = summary
        self.problems = list(problems or [])


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class GpsPoint:
    """One WGS-84 point in decimal degrees."""
    lat: float
    lon: float
    name: str = ""


@dataclass(frozen=True)
class NePoint:
    """Metres north / east of a reference point."""
    north: float
    east: float


@dataclass(frozen=True)
class Waypoint:
    """One purely geometric waypoint. Carries no command information."""
    index: int
    pass_index: int
    kind: str            # "pass_start" | "pass_end"
    direction: str       # "A->B" | "B->A"
    u_m: float           # along the sweep axis, from corner A
    v_m: float           # along the step axis, from corner A
    north_m: float       # local NE relative to corner A
    east_m: float
    lat: float
    lon: float
    dist_from_a_m: float
    inset_clearance_m: float     # distance to the inset polygon boundary
    polygon_clearance_m: float   # distance to the original polygon boundary


@dataclass
class Segment:
    """One straight leg of the path, for segment-level containment checking."""
    index: int
    kind: str            # "pass" | "transition"
    pass_index: int
    start_uv: tuple
    end_uv: tuple
    length_m: float
    samples: int = 0
    min_inset_clearance_m: float = 0.0
    min_polygon_clearance_m: float = 0.0


@dataclass
class RasterPlan:
    """Everything the geometry stage produces. Anchor-independent."""
    corners: tuple
    reference: GpsPoint
    corner_ne: tuple
    u_hat: tuple
    v_hat: tuple
    polygon_uv: tuple
    inset_uv: tuple
    waypoints: list
    segments: list
    pass_spacing_requested_m: float
    pass_spacing_actual_m: float
    inset_m: float
    speed_mps: float
    measurements: dict = field(default_factory=dict)
    dropped_passes: list = field(default_factory=list)
    axis: str = "long"
    axis_description: str = ""

    @property
    def pass_count(self) -> int:
        return len(self.waypoints) // 2

    @property
    def path_length_m(self) -> float:
        return sum(s.length_m for s in self.segments)

    @property
    def estimated_time_s(self) -> float:
        return self.path_length_m / self.speed_mps if self.speed_mps > 0 else float("nan")


# ---------------------------------------------------------------------------
# Flat-earth (equirectangular) projection
# ---------------------------------------------------------------------------
# Over tens of metres this is accurate to far below a centimetre — two orders of
# magnitude tighter than the ~1 m GPS eph this aircraft reports. No geodesy
# library, no UTM zone handling, no new dependency.
def gps_to_ne(point: GpsPoint, ref: GpsPoint) -> NePoint:
    """Returns metres north / east of ``ref``."""
    north = math.radians(point.lat - ref.lat) * R_EARTH_M
    east = math.radians(point.lon - ref.lon) * R_EARTH_M * math.cos(math.radians(ref.lat))
    return NePoint(north, east)


def ne_to_gps(north_m: float, east_m: float, ref: GpsPoint) -> tuple:
    """Exact inverse of :func:`gps_to_ne`. Returns (lat, lon) in degrees."""
    lat = ref.lat + math.degrees(north_m / R_EARTH_M)
    lon = ref.lon + math.degrees(east_m / (R_EARTH_M * math.cos(math.radians(ref.lat))))
    return lat, lon


# ---------------------------------------------------------------------------
# Sweep/step frame
# ---------------------------------------------------------------------------
def build_frame(a: NePoint, b: NePoint, d: NePoint, sweep_across: bool = False):
    """
    Orthonormal (u, v) frame: u is the sweep axis, v the step axis.

    By default u runs along A->B and v perpendicular to it, toward D.

    v is a rotation of u, NOT a normalisation of A->D. If A->D is not exactly
    perpendicular to A->B — and hand-picked corners never are — using A->D
    directly would give a sheared frame in which "parallel passes" and "spacing
    in metres" stop meaning what they say.

    ``sweep_across=True`` rotates the whole frame a quarter turn so the passes run
    along A->D instead of A->B (see RASTER_AXIS). The rotation is
    ``u' = v, v' = -u``, which is a proper rotation with determinant +1. That
    matters: ``validate_polygon`` requires ``signed_area(polygon_uv) > 0`` and
    ``edge_halfplanes`` assumes counter-clockwise winding, and only a
    handedness-preserving transform keeps both true. Swapping u and v instead
    (determinant -1) would mirror the polygon and reject every valid area.

    After the rotation v' points from B back toward A, so the polygon occupies
    NEGATIVE v'. Nothing downstream assumes a sign: the pass lines are laid out
    between the measured v_min and v_max of the inset polygon.
    """
    un, ue = b.north - a.north, b.east - a.east
    length = math.hypot(un, ue)
    if length <= 1e-9:
        raise PlanError("corners A and B are the same point; there is no sweep axis")
    un, ue = un / length, ue / length

    vn, ve = -ue, un                      # rotate +90 degrees in the (north, east) plane
    if (d.north - a.north) * vn + (d.east - a.east) * ve < 0.0:
        vn, ve = -vn, -ve                 # orient toward D

    if sweep_across:
        (un, ue), (vn, ve) = (vn, ve), (-un, -ue)
    return (un, ue), (vn, ve)


def resolve_sweep_axis(a: NePoint, b: NePoint, d: NePoint, axis: str):
    """
    Turns a RASTER_AXIS setting into ``(sweep_across, description)``.

        "ab"    passes run A->B                       (the original behaviour)
        "ad"    passes run A->D
        "long"  passes run along whichever of the two is LONGER  (fewest turns,
                the most efficient coverage, and the default)
        "short" passes run along the SHORTER side, so there are many short passes
                instead of few long ones

    "short" exists because pass length sets how long the aircraft spends flying in
    one direction before it turns, and that is what an observer on the ground
    reads as "is this a raster or is it just flying away?". On a 20 m x 6 m plot,
    "long" gives 6 passes of 20 m — 40 s per leg at 0.5 m/s — while "short" gives
    21 passes of 6 m and turns every 12 s.
    """
    key = (axis or "long").strip().lower()
    ab = math.hypot(b.north - a.north, b.east - a.east)
    ad = math.hypot(d.north - a.north, d.east - a.east)

    if key == "ab":
        across = False
    elif key == "ad":
        across = True
    elif key == "long":
        across = ad > ab
    elif key == "short":
        across = ad <= ab
    else:
        raise PlanError(
            f"RASTER_AXIS={axis!r} is not a recognised sweep axis",
            ["expected one of: long, short, ab, ad"],
        )

    along = "A->D" if across else "A->B"
    length = ad if across else ab
    other = ab if across else ad
    return across, (f"{along} ({length:.2f}m side, stepping across the {other:.2f}m side)")


def to_uv(point: NePoint, origin: NePoint, u_hat, v_hat) -> tuple:
    dn, de = point.north - origin.north, point.east - origin.east
    return (dn * u_hat[0] + de * u_hat[1], dn * v_hat[0] + de * v_hat[1])


def from_uv(u_m: float, v_m: float, origin: NePoint, u_hat, v_hat) -> NePoint:
    return NePoint(
        origin.north + u_m * u_hat[0] + v_m * v_hat[0],
        origin.east + u_m * u_hat[1] + v_m * v_hat[1],
    )


# ---------------------------------------------------------------------------
# Convex-polygon primitives, all in the flat (u, v) plane
# ---------------------------------------------------------------------------
def signed_area(poly) -> float:
    total = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        total += x1 * y2 - x2 * y1
    return 0.5 * total


def centroid(poly) -> tuple:
    """Area centroid of a simple polygon."""
    area = signed_area(poly)
    if abs(area) < 1e-12:
        n = len(poly)
        return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)
    cx = cy = 0.0
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cross = x1 * y2 - x2 * y1
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross
    return (cx / (6.0 * area), cy / (6.0 * area))


def edge_halfplanes(poly):
    """
    Inward unit normals for a counter-clockwise convex polygon.

    Returns [(point, normal), ...]. A point X is inside when
    (X - point) . normal >= 0 for every edge, and
    min over edges of (X - point) . normal is exactly its distance to the boundary.
    """
    planes = []
    n = len(poly)
    for i in range(n):
        px, py = poly[i]
        qx, qy = poly[(i + 1) % n]
        dx, dy = qx - px, qy - py
        length = math.hypot(dx, dy)
        if length <= 1e-12:
            continue
        planes.append(((px, py), (-dy / length, dx / length)))
    return planes


def clearance(point, planes) -> float:
    """
    Signed distance from ``point`` to the convex polygon boundary (negative = outside).

    Values within a nanometre of zero are snapped to exactly zero. A pass endpoint
    is clipped ONTO the inset boundary, so its true clearance is 0; without the
    snap, floating-point noise reports it as -4e-16 and prints as "-0.0000",
    which reads like a containment violation when it is not.
    """
    px, py = point
    value = min((px - qx) * nx + (py - qy) * ny for (qx, qy), (nx, ny) in planes)
    return 0.0 if abs(value) < 1e-9 else value


def clip_polygon_halfplane(poly, plane_point, plane_normal, offset: float):
    """
    Sutherland-Hodgman clip: keeps the part of ``poly`` where
    (X - plane_point) . plane_normal >= offset.

    Used to inset the polygon. Clipping is preferred over intersecting adjacent
    shifted edge lines because it stays correct when an inset removes a vertex
    entirely, and it returns an empty polygon — a detectable failure — when the
    inset is larger than the polygon can absorb.
    """
    qx, qy = plane_point
    nx, ny = plane_normal
    out = []
    n = len(poly)
    for i in range(n):
        cx, cy = poly[i]
        dx, dy = poly[(i + 1) % n]
        dist_c = (cx - qx) * nx + (cy - qy) * ny - offset
        dist_d = (dx - qx) * nx + (dy - qy) * ny - offset
        if dist_c >= 0.0:
            out.append((cx, cy))
        if (dist_c >= 0.0) != (dist_d >= 0.0):
            t = dist_c / (dist_c - dist_d)
            out.append((cx + t * (dx - cx), cy + t * (dy - cy)))
    return out


def dedupe_polygon(poly, tol: float = 1e-7):
    out = []
    for point in poly:
        if not out or math.hypot(point[0] - out[-1][0], point[1] - out[-1][1]) > tol:
            out.append(point)
    while len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= tol:
        out.pop()
    return out


def inset_polygon(poly, inset_m: float):
    """Insets a convex CCW polygon inward by ``inset_m``, by clipping half-planes."""
    result = list(poly)
    for plane_point, plane_normal in edge_halfplanes(poly):
        result = clip_polygon_halfplane(result, plane_point, plane_normal, inset_m)
        result = dedupe_polygon(result)
        if len(result) < 3:
            return []
    return result


def clip_line_to_convex(v_m: float, planes):
    """
    Clips the line v = v_m against a convex polygon, returning (u_lo, u_hi) or None.

    Each inward half-plane becomes one linear inequality in u. Handling the
    n_u == 0 case separately is what makes a line exactly collinear with an edge
    behave correctly instead of dividing by zero.
    """
    u_lo, u_hi = -math.inf, math.inf
    for (qx, qy), (nx, ny) in planes:
        constant = qx * nx + qy * ny - v_m * ny     # need u * nx >= constant
        if abs(nx) < 1e-12:
            if -constant < -1e-9:                   # 0 >= constant is false
                return None
            continue
        bound = constant / nx
        if nx > 0.0:
            u_lo = max(u_lo, bound)
        else:
            u_hi = min(u_hi, bound)
    if not math.isfinite(u_lo) or not math.isfinite(u_hi) or u_hi < u_lo:
        return None
    return (u_lo, u_hi)


# ---------------------------------------------------------------------------
# Measurement
# ---------------------------------------------------------------------------
def corner_angle_deg(prev_pt, vertex, next_pt) -> float:
    v1 = (prev_pt[0] - vertex[0], prev_pt[1] - vertex[1])
    v2 = (next_pt[0] - vertex[0], next_pt[1] - vertex[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 <= 0.0 or n2 <= 0.0:
        return float("nan")
    cosine = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosine))


def measure_polygon(poly_uv) -> dict:
    a, b, c, d = poly_uv
    dist = lambda p, q: math.hypot(q[0] - p[0], q[1] - p[1])
    return {
        "ab": dist(a, b), "bc": dist(b, c), "cd": dist(c, d), "da": dist(d, a),
        "ac": dist(a, c), "bd": dist(b, d),
        "angle_a": corner_angle_deg(d, a, b),
        "angle_b": corner_angle_deg(a, b, c),
        "angle_c": corner_angle_deg(b, c, d),
        "angle_d": corner_angle_deg(c, d, a),
        "perimeter": dist(a, b) + dist(b, c) + dist(c, d) + dist(d, a),
        "area": abs(signed_area(poly_uv)),
    }


# ---------------------------------------------------------------------------
# Validation
# ---------------------------------------------------------------------------
def validate_polygon(poly_uv, m: dict, spacing_m: float, inset_m: float):
    """
    Checks the polygon is usable as a mission area. Returns a list of problems;
    empty means valid.

    Deliberately does NOT require a rectangle. The mission area is whatever
    convex quadrilateral the four corners describe.
    """
    problems = []
    labels = "ABCD"

    for i in range(4):
        for j in range(i + 1, 4):
            gap = math.hypot(poly_uv[j][0] - poly_uv[i][0], poly_uv[j][1] - poly_uv[i][1])
            if gap < RASTER_MIN_CORNER_SEPARATION_M:
                problems.append(
                    f"corners {labels[i]} and {labels[j]} are only {gap:.3f} m apart "
                    f"(minimum {RASTER_MIN_CORNER_SEPARATION_M:.2f} m) — duplicated or mistyped corner"
                )

    for name, value in (("AB", m["ab"]), ("BC", m["bc"]), ("CD", m["cd"]), ("DA", m["da"])):
        if value < RASTER_MIN_SIDE_M:
            problems.append(f"side {name} is {value:.3f} m, below the {RASTER_MIN_SIDE_M:.1f} m minimum")
        if value > RASTER_MAX_SIDE_M:
            problems.append(f"side {name} is {value:.3f} m, above the {RASTER_MAX_SIDE_M:.1f} m maximum")

    # Convexity and simplicity. The (u, v) frame is built so that a correctly
    # ordered polygon comes out counter-clockwise, so every cross product must be
    # positive. A negative one means a reflex corner; mixed signs mean the corner
    # order crosses itself.
    crosses = []
    for i in range(4):
        ax, ay = poly_uv[i]
        bx, by = poly_uv[(i + 1) % 4]
        cx, cy = poly_uv[(i + 2) % 4]
        crosses.append((bx - ax) * (cy - by) - (by - ay) * (cx - bx))

    if signed_area(poly_uv) <= 0.0:
        problems.append(
            "A->B->C->D does not wind consistently around the area — the corner "
            "order is not a perimeter walk (check for two corners swapped)"
        )
    if any(c <= RASTER_MIN_CONVEX_CROSS for c in crosses):
        offenders = [labels[(i + 1) % 4] for i, c in enumerate(crosses) if c <= RASTER_MIN_CONVEX_CROSS]
        problems.append(
            f"polygon is not convex and simple: corner(s) {', '.join(offenders)} are "
            f"reflex or collinear (cross products {', '.join(f'{c:+.4f}' for c in crosses)}). "
            "A->B->C->D must walk the perimeter of a convex area."
        )

    if m["area"] < RASTER_MIN_AREA_M2:
        problems.append(f"area is {m['area']:.3f} m^2, below the {RASTER_MIN_AREA_M2:.1f} m^2 minimum")

    if spacing_m <= 0.0:
        problems.append(f"pass spacing must be positive (got {spacing_m})")
    if inset_m < 0.0:
        problems.append(f"edge inset cannot be negative (got {inset_m})")
    return problems


# ---------------------------------------------------------------------------
# Planning
# ---------------------------------------------------------------------------
def plan_raster(corner_latlon, spacing_m: float = None, inset_m: float = None,
                speed_mps: float = None, axis: str = None) -> RasterPlan:
    """
    Builds the raster plan for four GPS corners in perimeter order.

    Raises PlanError with a populated ``problems`` list if the area or the
    parameters cannot produce a safe path. Never returns a partial plan.

    ``axis`` selects which side the passes run along (RASTER_AXIS); see
    resolve_sweep_axis(). The plan stays anchor-independent either way — it is
    pure geometry relative to corner A, so it is built and verified on the ground
    before the flight controller is contacted.
    """
    spacing_m = RASTER_PASS_SPACING_M if spacing_m is None else spacing_m
    inset_m = EDGE_INSET_M if inset_m is None else inset_m
    speed_mps = RASTER_SPEED_MPS if speed_mps is None else speed_mps
    axis = RASTER_AXIS if axis is None else axis

    if len(corner_latlon) != 4:
        raise PlanError(f"exactly 4 corners are required (got {len(corner_latlon)})")

    corners = tuple(GpsPoint(lat, lon, name) for (lat, lon), name in zip(corner_latlon, "ABCD"))
    reference = corners[0]
    corner_ne = tuple(gps_to_ne(corner, reference) for corner in corners)

    sweep_across, axis_description = resolve_sweep_axis(
        corner_ne[0], corner_ne[1], corner_ne[3], axis)
    u_hat, v_hat = build_frame(corner_ne[0], corner_ne[1], corner_ne[3], sweep_across)
    polygon_uv = tuple(to_uv(point, corner_ne[0], u_hat, v_hat) for point in corner_ne)
    m = measure_polygon(polygon_uv)

    problems = validate_polygon(polygon_uv, m, spacing_m, inset_m)
    if problems:
        raise PlanError("the four corners do not form a usable mission polygon", problems)

    inset_uv = dedupe_polygon(inset_polygon(polygon_uv, inset_m))
    if len(inset_uv) < 3 or abs(signed_area(inset_uv)) < 1e-6:
        raise PlanError(
            "the inward safety inset consumes the whole mission area",
            [f"an inset of {inset_m:.3f} m leaves no usable area inside a "
             f"{m['area']:.3f} m^2 polygon — reduce the inset or use a larger area"],
        )

    inset_planes = edge_halfplanes(inset_uv)
    polygon_planes = edge_halfplanes(polygon_uv)

    # Raster lines across the inset polygon's v extent, half a spacing in from
    # each end so no line degenerates to a single vertex (see module docstring).
    v_values = [p[1] for p in inset_uv]
    v_min, v_max = min(v_values), max(v_values)
    v_span = v_max - v_min
    if v_span <= 1e-9:
        raise PlanError("the inset area has no extent along the step axis",
                        [f"step-axis extent is {v_span:.6f} m"])
    interval_count = max(1, math.ceil(v_span / spacing_m))
    actual_spacing = v_span / interval_count
    pass_v = [v_min + (k + 0.5) * actual_spacing for k in range(interval_count)]

    waypoints = []
    segments = []
    dropped = []
    index = 0
    pass_number = 0
    previous_end = None

    for v_m in pass_v:
        span = clip_line_to_convex(v_m, inset_planes)
        if span is None:
            dropped.append((v_m, 0.0, "line does not intersect the inset area"))
            continue
        u_lo, u_hi = span
        length = u_hi - u_lo
        if length < RASTER_MIN_PASS_LENGTH_M:
            dropped.append((v_m, length, f"clipped length below {RASTER_MIN_PASS_LENGTH_M:.2f} m"))
            continue

        forward = (pass_number % 2 == 0)
        start_u, end_u = (u_lo, u_hi) if forward else (u_hi, u_lo)
        # Names the direction along the SWEEP axis, which is A->B only when the
        # passes run along AB. With RASTER_AXIS=short on a wide plot they run
        # along AD instead, and calling that "A->B" in the log would be a lie.
        low_name, high_name = ("A", "D") if sweep_across else ("A", "B")
        direction = (f"{low_name}->{high_name}" if forward
                     else f"{high_name}->{low_name}")
        pass_number += 1

        if previous_end is not None:
            segments.append(Segment(
                index=len(segments), kind="transition", pass_index=pass_number,
                start_uv=previous_end, end_uv=(start_u, v_m),
                length_m=math.hypot(start_u - previous_end[0], v_m - previous_end[1]),
            ))

        for kind, u_m in (("pass_start", start_u), ("pass_end", end_u)):
            ne = from_uv(u_m, v_m, corner_ne[0], u_hat, v_hat)
            lat, lon = ne_to_gps(ne.north, ne.east, reference)
            waypoints.append(Waypoint(
                index=index, pass_index=pass_number, kind=kind, direction=direction,
                u_m=u_m, v_m=v_m, north_m=ne.north, east_m=ne.east, lat=lat, lon=lon,
                dist_from_a_m=math.hypot(ne.north, ne.east),
                inset_clearance_m=clearance((u_m, v_m), inset_planes),
                polygon_clearance_m=clearance((u_m, v_m), polygon_planes),
            ))
            index += 1

        segments.append(Segment(
            index=len(segments), kind="pass", pass_index=pass_number,
            start_uv=(start_u, v_m), end_uv=(end_u, v_m), length_m=length,
        ))
        previous_end = (end_u, v_m)

    if pass_number == 0:
        raise PlanError("no usable raster pass could be generated",
                        [f"every candidate line was shorter than "
                         f"{RASTER_MIN_PASS_LENGTH_M:.2f} m after clipping"])

    return RasterPlan(
        corners=corners, reference=reference, corner_ne=corner_ne,
        u_hat=u_hat, v_hat=v_hat, polygon_uv=polygon_uv, inset_uv=tuple(inset_uv),
        waypoints=waypoints, segments=segments,
        pass_spacing_requested_m=spacing_m, pass_spacing_actual_m=actual_spacing,
        inset_m=inset_m, speed_mps=speed_mps, measurements=m, dropped_passes=dropped,
        axis=axis, axis_description=axis_description,
    )


# ---------------------------------------------------------------------------
# Containment verification — waypoints AND segments
# ---------------------------------------------------------------------------
def verify_containment(plan: RasterPlan):
    """
    Verifies waypoint containment and segment containment.

    Checking waypoints alone is not sufficient: two endpoints can both sit inside
    a shape while the straight line between them leaves it. Convexity rules that
    out here, but the check samples every segment numerically anyway rather than
    trusting the argument, and reports the worst clearance it finds.

    Returns (problems, metrics).
    """
    inset_planes = edge_halfplanes(plan.inset_uv)
    polygon_planes = edge_halfplanes(plan.polygon_uv)
    problems = []

    inside_inset = 0
    inside_polygon = 0
    for w in plan.waypoints:
        if w.inset_clearance_m >= -RASTER_CONTAINMENT_EPS_M:
            inside_inset += 1
        else:
            problems.append(f"WP{w.index:02d} (pass {w.pass_index} {w.kind}) is outside the "
                            f"inset polygon by {-w.inset_clearance_m:.4f} m")
        if w.polygon_clearance_m >= -RASTER_CONTAINMENT_EPS_M:
            inside_polygon += 1
        else:
            problems.append(f"WP{w.index:02d} (pass {w.pass_index} {w.kind}) is outside the "
                            f"mission polygon by {-w.polygon_clearance_m:.4f} m")

    total_samples = 0
    worst_inset = math.inf
    worst_polygon = math.inf
    worst_inset_segment = None
    worst_polygon_segment = None

    for seg in plan.segments:
        steps = max(2, int(math.ceil(seg.length_m / RASTER_SAMPLE_STEP_M)) + 1)
        seg.samples = steps
        seg.min_inset_clearance_m = math.inf
        seg.min_polygon_clearance_m = math.inf
        for i in range(steps):
            t = i / (steps - 1)
            point = (seg.start_uv[0] + t * (seg.end_uv[0] - seg.start_uv[0]),
                     seg.start_uv[1] + t * (seg.end_uv[1] - seg.start_uv[1]))
            ci = clearance(point, inset_planes)
            cp = clearance(point, polygon_planes)
            seg.min_inset_clearance_m = min(seg.min_inset_clearance_m, ci)
            seg.min_polygon_clearance_m = min(seg.min_polygon_clearance_m, cp)
            if ci < -RASTER_CONTAINMENT_EPS_M:
                problems.append(
                    f"segment {seg.index} ({seg.kind}, pass {seg.pass_index}) leaves the inset "
                    f"polygon by {-ci:.4f} m at {100.0 * t:.1f}% along it "
                    f"(u={point[0]:.3f}, v={point[1]:.3f})"
                )
            if cp < -RASTER_CONTAINMENT_EPS_M:
                problems.append(
                    f"segment {seg.index} ({seg.kind}, pass {seg.pass_index}) leaves the mission "
                    f"polygon by {-cp:.4f} m at {100.0 * t:.1f}% along it "
                    f"(u={point[0]:.3f}, v={point[1]:.3f})"
                )
        total_samples += steps
        if seg.min_inset_clearance_m < worst_inset:
            worst_inset, worst_inset_segment = seg.min_inset_clearance_m, seg
        if seg.min_polygon_clearance_m < worst_polygon:
            worst_polygon, worst_polygon_segment = seg.min_polygon_clearance_m, seg

    # Deduplicate repeated complaints about the same segment, keeping the worst.
    seen = set()
    unique = []
    for problem in problems:
        key = problem.split(" at ")[0]
        if key not in seen:
            seen.add(key)
            unique.append(problem)

    centre = centroid(plan.polygon_uv)
    metrics = {
        "waypoints": len(plan.waypoints),
        "waypoints_inside_inset": inside_inset,
        "waypoints_inside_polygon": inside_polygon,
        "segments": len(plan.segments),
        "segment_samples": total_samples,
        "sample_step_m": RASTER_SAMPLE_STEP_M,
        "min_waypoint_inset_clearance_m": min(w.inset_clearance_m for w in plan.waypoints),
        "min_waypoint_polygon_clearance_m": min(w.polygon_clearance_m for w in plan.waypoints),
        "min_segment_inset_clearance_m": worst_inset,
        "min_segment_polygon_clearance_m": worst_polygon,
        "worst_inset_segment": worst_inset_segment,
        "worst_polygon_segment": worst_polygon_segment,
        "max_dist_from_a_m": max(w.dist_from_a_m for w in plan.waypoints),
        "max_dist_from_centroid_m": max(
            math.hypot(w.u_m - centre[0], w.v_m - centre[1]) for w in plan.waypoints),
        "centroid_uv": centre,
    }
    return unique, metrics


# ---------------------------------------------------------------------------
# The outer safety geofence
# ---------------------------------------------------------------------------
# Everything below works in metres north/east of a caller-supplied GPS reference,
# so the flight code can pass the runtime anchor and the offline planner can pass
# inner corner A, and both get identical numbers.
def ring_ne(corner_latlon, ref: GpsPoint):
    """Projects GPS corners into (north, east) metres relative to ``ref``."""
    return [(q.north, q.east) for q in
            (gps_to_ne(GpsPoint(la, lo), ref) for la, lo in corner_latlon)]


def ccw(ring):
    """Returns the ring wound counter-clockwise, as edge_halfplanes requires."""
    return list(ring) if signed_area(ring) > 0.0 else list(ring)[::-1]


def measure_ring(ring):
    """Side lengths, perimeter, area and winding of an arbitrary ring."""
    n = len(ring)
    sides = [math.hypot(ring[(i + 1) % n][0] - ring[i][0],
                        ring[(i + 1) % n][1] - ring[i][1]) for i in range(n)]
    area = signed_area(ring)
    return {
        "sides": sides,
        "perimeter": sum(sides),
        "area": abs(area),
        "signed_area": area,
        "winding": "CCW" if area > 0.0 else "CW",
    }


def validate_outer_polygon(corner_latlon, ref: GpsPoint = None):
    """
    Checks the outer safety geofence is usable. Returns a list of problems.

    The requirements are the same as for the mission polygon and for the same
    reason: the half-plane containment test used at 10 Hz in flight
    (edge_halfplanes / clearance) is only exact for a convex, consistently wound,
    simple polygon. A reflex corner would silently produce an inward normal that
    points the wrong way, and the geofence would then report a breach as safe.

    It deliberately does NOT reorder the corners to make them wind correctly. A
    caller who supplied them in the wrong order has a different polygon in mind
    than the one that would result, and silently flying a different safety
    boundary is exactly the failure this is meant to prevent.
    """
    problems = []
    if len(corner_latlon) != 4:
        return [f"outer geofence needs exactly 4 corners, got {len(corner_latlon)}"]

    ref = ref or GpsPoint(*corner_latlon[0])
    ring = ring_ne(corner_latlon, ref)
    m = measure_ring(ring)
    names = ("G1", "G2", "G3", "G4")

    for i in range(4):
        for j in range(i + 1, 4):
            d = math.hypot(ring[i][0] - ring[j][0], ring[i][1] - ring[j][1])
            if d < RASTER_MIN_CORNER_SEPARATION_M:
                problems.append(
                    f"outer corners {names[i]} and {names[j]} are only {d:.3f} m apart")

    for i, side in enumerate(m["sides"]):
        label = f"{names[i]}{names[(i + 1) % 4]}"
        if side < RASTER_MIN_SIDE_M:
            problems.append(f"outer side {label} is {side:.3f} m, below the "
                            f"{RASTER_MIN_SIDE_M:.2f} m minimum")
        if side > RASTER_MAX_SIDE_M:
            problems.append(f"outer side {label} is {side:.3f} m, above the "
                            f"{RASTER_MAX_SIDE_M:.1f} m maximum")

    crosses = []
    for i in range(4):
        ax, ay = ring[i]
        bx, by = ring[(i + 1) % 4]
        cx, cy = ring[(i + 2) % 4]
        crosses.append((bx - ax) * (cy - by) - (by - ay) * (cx - bx))
    positive = sum(1 for c in crosses if c > RASTER_MIN_CONVEX_CROSS)
    negative = sum(1 for c in crosses if c < -RASTER_MIN_CONVEX_CROSS)
    if positive != 4 and negative != 4:
        offenders = [names[(i + 1) % 4] for i, c in enumerate(crosses)
                     if abs(c) <= RASTER_MIN_CONVEX_CROSS
                     or (positive > negative) != (c > 0)]
        problems.append(
            f"outer geofence is not convex and simple: corner(s) "
            f"{', '.join(offenders) or '?'} are reflex or collinear "
            f"(cross products {', '.join(f'{c:+.3f}' for c in crosses)}). "
            "G1->G2->G3->G4 must walk the perimeter of a convex area.")

    if m["area"] < RASTER_MIN_AREA_M2:
        problems.append(f"outer geofence area is {m['area']:.3f} m^2, below the "
                        f"{RASTER_MIN_AREA_M2:.1f} m^2 minimum")
    return problems


def effective_outer_ring(outer_ring_ccw, margin_m: float):
    """
    The outer ring shrunk inward by ``margin_m`` — the boundary actually enforced.

    Same half-plane clipping the inner inset uses, so it is exact for a convex
    polygon and collapses to an empty ring (a loud failure) if the margin eats
    the whole area.
    """
    return inset_polygon(outer_ring_ccw, margin_m)


def _ring_min_clearance(ring, planes, step_m: float = None):
    """Worst clearance along a ring's EDGES, not just at its vertices."""
    step_m = step_m or RASTER_SAMPLE_STEP_M
    worst, where, samples = float("inf"), None, 0
    n = len(ring)
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(length / step_m) + 1)
        for k in range(steps + 1):
            t = k / steps
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            c = clearance(q, planes)
            samples += 1
            if c < worst:
                worst, where = c, (i, t, q)
    return worst, where, samples


def verify_inner_inside_outer(plan: RasterPlan, outer_corner_latlon,
                              margin_m: float = None):
    """
    Proves the inner polygon and the whole raster path sit inside the outer one.

    Returns (problems, metrics). Everything is measured in the inner plan's own
    NE frame (metres from inner corner A), so the two polygons are directly
    comparable without a second projection.

    Convexity alone would make the corner test sufficient — the segment between
    two points of a convex set stays in the set — but every edge and every path
    segment is sampled numerically anyway. The argument is only as good as the
    convexity check that precedes it, and this costs microseconds.
    """
    margin_m = OUTER_GEOFENCE_MARGIN_M if margin_m is None else margin_m
    problems = []

    outer_ring = ccw(ring_ne(outer_corner_latlon, plan.reference))
    planes = edge_halfplanes(outer_ring)
    outer_m = measure_ring(outer_ring)

    effective = effective_outer_ring(outer_ring, margin_m)
    eff_planes = edge_halfplanes(effective) if len(effective) >= 3 else []
    if len(effective) < 3:
        problems.append(
            f"a {margin_m:.2f} m inward safety margin collapses the outer geofence "
            "to nothing; it is too large for this polygon")

    inner_ring = [(from_uv(u, v, plan.corner_ne[0], plan.u_hat, plan.v_hat))
                  for u, v in plan.polygon_uv]
    inner_ring = [(q.north, q.east) for q in inner_ring]

    names = ("A", "B", "C", "D")
    corner_clearance = {}
    for name, q in zip(names, inner_ring):
        c = clearance(q, planes)
        corner_clearance[name] = c
        if c < 0.0:
            problems.append(f"inner corner {name} is {-c:.3f} m OUTSIDE the outer geofence")

    edge_worst, edge_where, edge_samples = _ring_min_clearance(inner_ring, planes)
    if edge_worst < 0.0:
        i = edge_where[0]
        problems.append(
            f"inner edge {names[i]}{names[(i + 1) % 4]} leaves the outer geofence by "
            f"{-edge_worst:.3f} m")

    # The planned raster path, sampled against both the raw and effective rings.
    path_worst, path_worst_eff, path_samples = float("inf"), float("inf"), 0
    for seg in plan.segments:
        a = from_uv(seg.start_uv[0], seg.start_uv[1], plan.corner_ne[0],
                    plan.u_hat, plan.v_hat)
        b = from_uv(seg.end_uv[0], seg.end_uv[1], plan.corner_ne[0],
                    plan.u_hat, plan.v_hat)
        a, b = (a.north, a.east), (b.north, b.east)
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(length / RASTER_SAMPLE_STEP_M) + 1)
        for k in range(steps + 1):
            t = k / steps
            q = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            path_samples += 1
            path_worst = min(path_worst, clearance(q, planes))
            if eff_planes:
                path_worst_eff = min(path_worst_eff, clearance(q, eff_planes))
    if path_worst < 0.0:
        problems.append(f"the raster path leaves the outer geofence by {-path_worst:.3f} m")
    if eff_planes and path_worst_eff < 0.0:
        problems.append(
            f"the raster path leaves the EFFECTIVE outer geofence (raw minus "
            f"{margin_m:.2f} m) by {-path_worst_eff:.3f} m")

    metrics = {
        "outer_ring_ne": outer_ring,
        "outer_planes": planes,
        "effective_ring_ne": effective,
        "effective_planes": eff_planes,
        "outer_area_m2": outer_m["area"],
        "outer_sides_m": outer_m["sides"],
        "outer_perimeter_m": outer_m["perimeter"],
        "outer_winding": outer_m["winding"],
        "effective_area_m2": abs(signed_area(effective)) if len(effective) >= 3 else 0.0,
        "inner_ring_ne": inner_ring,
        "corner_clearance_m": corner_clearance,
        "min_corner_clearance_m": min(corner_clearance.values()),
        "min_edge_clearance_m": edge_worst,
        "min_edge_clearance_at": edge_where,
        "edge_samples": edge_samples,
        "min_path_clearance_m": path_worst,
        "min_path_clearance_effective_m": path_worst_eff,
        "path_samples": path_samples,
        "margin_m": margin_m,
    }
    return problems, metrics


def centroid_gps(plan: RasterPlan) -> tuple:
    """Area centroid of the mission polygon as (lat, lon). Used to place the simulator."""
    cu, cv = centroid(plan.polygon_uv)
    ne = from_uv(cu, cv, plan.corner_ne[0], plan.u_hat, plan.v_hat)
    return ne_to_gps(ne.north, ne.east, plan.reference)


# ---------------------------------------------------------------------------
# GeoJSON
# ---------------------------------------------------------------------------
def uv_ring_to_gps(poly_uv, plan: RasterPlan):
    ring = []
    for u_m, v_m in list(poly_uv) + [poly_uv[0]]:
        ne = from_uv(u_m, v_m, plan.corner_ne[0], plan.u_hat, plan.v_hat)
        lat, lon = ne_to_gps(ne.north, ne.east, plan.reference)
        ring.append([round(lon, 9), round(lat, 9)])
    return ring


def write_geojson(path: str, plan: RasterPlan, metrics: dict, status: str = "OK",
                  outer_corner_latlon=None, outer_margin_m: float = None):
    """
    Writes a FeatureCollection in WGS-84 [longitude, latitude] order.

    Layers, in the order a viewer stacks them:
      outer_geofence            the supplied G1-G4 safety polygon
      outer_geofence_effective  that polygon shrunk by the inward safety margin
      mission_polygon           the supplied inner A-D raster polygon
      inset_polygon             the inner polygon shrunk by the raster edge inset
      raster_path               the flown line
      waypoint                  one point per raster waypoint
      outer_corner              G1-G4 markers
      corner                    A-D markers
    """
    outer_margin_m = OUTER_GEOFENCE_MARGIN_M if outer_margin_m is None else outer_margin_m
    features = [{
        "type": "Feature",
        "properties": {
            "name": "inner raster polygon (as supplied)", "role": "mission_polygon",
            "area_m2": round(plan.measurements["area"], 3),
            "perimeter_m": round(plan.measurements["perimeter"], 3),
            "stroke": "#d62728", "stroke-width": 2, "fill": "#d62728", "fill-opacity": 0.05,
        },
        "geometry": {"type": "Polygon", "coordinates": [uv_ring_to_gps(plan.polygon_uv, plan)]},
    }]

    # --- outer safety geofence, and the boundary actually enforced -------------
    if outer_corner_latlon:
        outer_ring = ccw(ring_ne(outer_corner_latlon, plan.reference))
        om = measure_ring(outer_ring)
        ring_gps = [[round(lon, 9), round(lat, 9)] for lat, lon in
                    (ne_to_gps(n, e, plan.reference) for n, e in outer_ring)]
        ring_gps.append(ring_gps[0])
        features.insert(0, {
            "type": "Feature",
            "properties": {
                "name": "outer safety geofence (as supplied)",
                "role": "outer_geofence",
                "area_m2": round(om["area"], 3),
                "side_lengths_m": [round(v, 3) for v in om["sides"]],
                "perimeter_m": round(om["perimeter"], 3),
                "note": "companion-side only; NOT an ArduPilot FENCE_* fence",
                "stroke": "#9467bd", "stroke-width": 3,
                "fill": "#9467bd", "fill-opacity": 0.04,
            },
            "geometry": {"type": "Polygon", "coordinates": [ring_gps]},
        })

        eff = effective_outer_ring(outer_ring, outer_margin_m)
        if len(eff) >= 3:
            eff_gps = [[round(lon, 9), round(lat, 9)] for lat, lon in
                       (ne_to_gps(n, e, plan.reference) for n, e in eff)]
            eff_gps.append(eff_gps[0])
            features.insert(1, {
                "type": "Feature",
                "properties": {
                    "name": f"effective outer geofence ({outer_margin_m:.2f} m inward margin)",
                    "role": "outer_geofence_effective",
                    "margin_m": outer_margin_m,
                    "area_m2": round(abs(signed_area(eff)), 3),
                    "note": "this is the boundary the mission actually enforces",
                    "stroke": "#8c564b", "stroke-width": 2, "stroke-dasharray": "6,4",
                    "fill": "#8c564b", "fill-opacity": 0.03,
                },
                "geometry": {"type": "Polygon", "coordinates": [eff_gps]},
            })

        for name, (lat, lon) in zip(("G1", "G2", "G3", "G4"), outer_corner_latlon):
            features.append({
                "type": "Feature",
                "properties": {"name": f"outer corner {name}", "role": "outer_corner",
                               "corner": name,
                               "marker-color": "#9467bd", "marker-symbol": "triangle"},
                "geometry": {"type": "Point", "coordinates": [round(lon, 9), round(lat, 9)]},
            })

    if plan.inset_uv:
        features.append({
            "type": "Feature",
            "properties": {
                "name": f"inset polygon (inward safety inset {plan.inset_m:.2f} m)",
                "role": "inset_polygon",
                "area_m2": round(abs(signed_area(plan.inset_uv)), 3),
                "stroke": "#1f77b4", "stroke-width": 2, "fill": "#1f77b4", "fill-opacity": 0.08,
            },
            "geometry": {"type": "Polygon", "coordinates": [uv_ring_to_gps(plan.inset_uv, plan)]},
        })

    if plan.waypoints:
        features.append({
            "type": "Feature",
            "properties": {
                "name": "raster path", "role": "raster_path",
                "passes": plan.pass_count,
                "pass_spacing_m": round(plan.pass_spacing_actual_m, 4),
                "path_length_m": round(plan.path_length_m, 3),
                "estimated_time_s": round(plan.estimated_time_s, 1),
                "planning_speed_mps": plan.speed_mps,
                "stroke": "#2ca02c", "stroke-width": 3,
            },
            "geometry": {
                "type": "LineString",
                "coordinates": [[round(w.lon, 9), round(w.lat, 9)] for w in plan.waypoints],
            },
        })
        for w in plan.waypoints:
            features.append({
                "type": "Feature",
                "properties": {
                    "name": f"WP{w.index:02d} pass {w.pass_index} {w.kind}",
                    "role": "waypoint", "index": w.index, "pass": w.pass_index,
                    "kind": w.kind, "direction": w.direction,
                    "north_m": round(w.north_m, 3), "east_m": round(w.east_m, 3),
                    "dist_from_a_m": round(w.dist_from_a_m, 3),
                    "inset_clearance_m": round(w.inset_clearance_m, 3),
                    "polygon_clearance_m": round(w.polygon_clearance_m, 3),
                    "marker-color": "#2ca02c", "marker-size": "small",
                },
                "geometry": {"type": "Point", "coordinates": [round(w.lon, 9), round(w.lat, 9)]},
            })

    for corner in plan.corners:
        features.append({
            "type": "Feature",
            "properties": {"name": f"corner {corner.name}", "role": "corner",
                           "marker-color": "#d62728", "marker-size": "medium",
                           "marker-symbol": corner.name.lower()},
            "geometry": {"type": "Point",
                         "coordinates": [round(corner.lon, 9), round(corner.lat, 9)]},
        })

    document = {
        "type": "FeatureCollection",
        "properties": {
            "generator": "raster_plan_preview.py (offline planner, no flight commands)",
            "coordinate_order": "WGS-84 [longitude, latitude]",
            "status": status,
            "pass_count": plan.pass_count,
            "waypoint_count": len(plan.waypoints),
            "requested_spacing_m": plan.pass_spacing_requested_m,
            "actual_spacing_m": plan.pass_spacing_actual_m,
            "inset_m": plan.inset_m,
            "path_length_m": plan.path_length_m,
            "estimated_time_s": plan.estimated_time_s,
            "speed_mps": plan.speed_mps,
            "max_dist_from_a_m": metrics.get("max_dist_from_a_m"),
            "min_segment_inset_clearance_m": metrics.get("min_segment_inset_clearance_m"),
        },
        "features": features,
    }
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")
    return path


def write_boundary_only_geojson(path: str, corner_latlon, summary: str, problems):
    """Writes just the supplied corners, so a rejected area is still inspectable."""
    corners = [GpsPoint(lat, lon, name) for (lat, lon), name in zip(corner_latlon, "ABCD")]
    features = [{
        "type": "Feature",
        "properties": {"name": "supplied corners (REJECTED)", "role": "rejected_polygon",
                       "reason": summary, "problems": problems,
                       "stroke": "#d62728", "stroke-width": 2,
                       "fill": "#d62728", "fill-opacity": 0.05},
        "geometry": {"type": "Polygon",
                     "coordinates": [[[round(c.lon, 9), round(c.lat, 9)] for c in corners]
                                     + [[round(corners[0].lon, 9), round(corners[0].lat, 9)]]]},
    }]
    for corner in corners:
        features.append({
            "type": "Feature",
            "properties": {"name": f"corner {corner.name}", "role": "corner",
                           "marker-color": "#d62728", "marker-size": "medium"},
            "geometry": {"type": "Point",
                         "coordinates": [round(corner.lon, 9), round(corner.lat, 9)]},
        })
    document = {"type": "FeatureCollection",
                "properties": {"status": "REJECTED", "reason": summary, "problems": problems,
                               "coordinate_order": "WGS-84 [longitude, latitude]"},
                "features": features}
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(document, handle, indent=2)
        handle.write("\n")
    return path


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def heading(text):
    print()
    print(RULE)
    print(text)
    print(RULE)


def bearing_deg(vector) -> float:
    return math.degrees(math.atan2(vector[1], vector[0])) % 360.0


def ascii_plot(plan: RasterPlan, cols=68, rows=26):
    us = [p[0] for p in plan.polygon_uv] + [w.u_m for w in plan.waypoints]
    vs = [p[1] for p in plan.polygon_uv] + [w.v_m for w in plan.waypoints]
    u_min, u_max, v_min, v_max = min(us), max(us), min(vs), max(vs)
    span_u = max(u_max - u_min, 1e-6)
    span_v = max(v_max - v_min, 1e-6)
    grid = [[" "] * cols for _ in range(rows)]

    def put(u_m, v_m, ch, overwrite=True):
        col = int(round((u_m - u_min) / span_u * (cols - 1)))
        row = int(round((v_max - v_m) / span_v * (rows - 1)))
        if 0 <= row < rows and 0 <= col < cols and (overwrite or grid[row][col] == " "):
            grid[row][col] = ch

    def trace(poly, ch):
        n = len(poly)
        for i in range(n):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % n]
            for s in range(241):
                t = s / 240.0
                put(ax + (bx - ax) * t, ay + (by - ay) * t, ch, overwrite=False)

    trace(plan.polygon_uv, ":")
    trace(plan.inset_uv, ".")

    for seg in plan.segments:
        mark = str(seg.pass_index % 10) if seg.kind == "pass" else "|"
        for s in range(241):
            t = s / 240.0
            put(seg.start_uv[0] + (seg.end_uv[0] - seg.start_uv[0]) * t,
                seg.start_uv[1] + (seg.end_uv[1] - seg.start_uv[1]) * t, mark)

    for label, point in zip("ABCD", plan.polygon_uv):
        put(point[0], point[1], label)

    print("  ':' mission polygon   '.' inset polygon   digits = pass number   '|' transition")
    # The sweep axis is A->B only when RASTER_AXIS leaves it there; with the
    # short sweep it runs A->D and the step axis runs A->B, so take both names
    # from the plan rather than hardcoding them.
    sweep_name = (plan.axis_description.split(" ")[0]
                  if plan.axis_description else "A->B")
    step_name = "A->B" if sweep_name != "A->B" else "A->D"
    print(f"  (u = sweep axis {sweep_name} rightward, v = step axis {step_name} upward; "
          "not to scale)")
    for row in grid:
        print("  |" + "".join(row).rstrip())


def report_rejection(summary: str, problems, hint: str = None):
    heading(f"RESULT: FAIL — {summary}")
    for problem in problems:
        print(f"  [FAIL] {problem}")
    print()
    print("  NO FLIGHT PATH HAS BEEN PRODUCED.")
    if hint:
        print()
        for line in hint.splitlines():
            print(f"  {line}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_corners(text):
    parts = [chunk.strip() for chunk in text.split(";") if chunk.strip()]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected exactly 4 corners separated by ';'")
    corners = []
    for part in parts:
        try:
            lat_text, lon_text = part.split(",")
            corners.append((float(lat_text), float(lon_text)))
        except ValueError:
            raise argparse.ArgumentTypeError(f"could not parse corner '{part}' as 'lat,lon'")
    return tuple(corners)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline polygon raster/lawnmower planner. Geometry only — "
                    "no MAVLink, no flight controller, no flight commands.")
    parser.add_argument("--spacing", type=float, default=RASTER_PASS_SPACING_M,
                        help=f"pass spacing in metres (default {RASTER_PASS_SPACING_M})")
    parser.add_argument("--inset", type=float, default=EDGE_INSET_M,
                        help=f"inward safety inset in metres (default {EDGE_INSET_M})")
    parser.add_argument("--speed", type=float, default=RASTER_SPEED_MPS,
                        help=f"planning speed in m/s for the time estimate (default {RASTER_SPEED_MPS})")
    parser.add_argument("--corners", type=parse_corners, default=DEFAULT_CORNERS,
                        help="inner raster polygon: 'latA,lonA;...;latD,lonD' in perimeter order")
    parser.add_argument("--outer", type=parse_corners, default=DEFAULT_OUTER_CORNERS,
                        help="outer safety geofence: 'latG1,lonG1;...;latG4,lonG4'")
    parser.add_argument("--outer-margin", type=float, default=OUTER_GEOFENCE_MARGIN_M,
                        help=f"inward outer safety margin in metres "
                             f"(default {OUTER_GEOFENCE_MARGIN_M})")
    parser.add_argument("--geojson", default=GEOJSON_OUTPUT,
                        help=f"GeoJSON output path (default {GEOJSON_OUTPUT})")
    parser.add_argument("--no-geojson", action="store_true", help="skip writing the GeoJSON file")
    parser.add_argument("--quiet", action="store_true",
                        help="print only the summary line and the containment verdict")
    args = parser.parse_args()

    print(BAR)
    print(" OFFLINE POLYGON RASTER / LAWNMOWER PLANNER — GEOMETRY ONLY")
    print(BAR)
    print(" No MAVLink. No serial port. No flight controller. No flight commands.")
    print(" The mission area is the ACTUAL four-corner polygon, not a fitted rectangle.")

    outer_corners = args.outer

    heading("1. INPUT — ORIGINAL COORDINATES")
    print("  INNER raster polygon, perimeter order A -> B -> C -> D -> A")
    print("  Raster sweep    : A -> B      Raster step : A -> D")
    for (lat, lon), name in zip(args.corners, "ABCD"):
        print(f"  Corner {name}        : {lat:.7f}, {lon:.7f}")
    print()
    print("  OUTER safety geofence, perimeter order G1 -> G2 -> G3 -> G4 -> G1")
    for (lat, lon), name in zip(outer_corners, ("G1", "G2", "G3", "G4")):
        print(f"  Corner {name}       : {lat:.7f}, {lon:.7f}")
    print()
    print(f"  Pass spacing    : {args.spacing:.3f} m requested")
    print(f"  Edge inset      : {args.inset:.3f} m inward")
    print(f"  Outer margin    : {args.outer_margin:.3f} m inward from the outer polygon")
    print(f"  Planning speed  : {args.speed:.3f} m/s  (time estimate only — commands nothing)")

    outer_problems = validate_outer_polygon(outer_corners)
    if outer_problems:
        heading("OUTER SAFETY GEOFENCE VALIDITY")
        print("  Validity : FAIL")
        report_rejection("the outer safety geofence is not usable", outer_problems,
                         "The outer polygon must be a simple, convex, consistently wound\n"
                         "quadrilateral. Corners are NOT reordered automatically: supplying\n"
                         "them in the wrong order describes a different area than intended,\n"
                         "and silently flying a different safety boundary is exactly the\n"
                         "failure this check exists to prevent.")
        return 3

    try:
        plan = plan_raster(args.corners, args.spacing, args.inset, args.speed)
    except PlanError as exc:
        heading("4. POLYGON VALIDITY")
        print("  Validity : FAIL")
        report_rejection(exc.summary, exc.problems,
                         "Re-read the four corners from Google Maps and confirm they are listed\n"
                         "in perimeter order A -> B -> C -> D, walking the boundary rather than\n"
                         "crossing a diagonal. The area must be a convex four-corner polygon;\n"
                         "it does not need to be a rectangle.")
        if not args.no_geojson:
            out = write_boundary_only_geojson(args.geojson, args.corners, exc.summary, exc.problems)
            print(f"\n  Boundary-only GeoJSON written for inspection: {out}")
        return 3

    m = plan.measurements

    heading("2. PROJECTION AND LOCAL COORDINATES")
    ref = plan.reference
    print("  Method          : equirectangular (flat-earth) tangent plane")
    print(f"  Reference point : corner A at {ref.lat:.7f}, {ref.lon:.7f}")
    print(f"  Earth radius    : {R_EARTH_M:.1f} m,  cos(lat0) = {math.cos(math.radians(ref.lat)):.9f}")
    print(f"  Scale           : 1e-7 deg lat = {math.radians(1e-7) * R_EARTH_M * 100:.4f} cm, "
          f"1e-7 deg lon = "
          f"{math.radians(1e-7) * R_EARTH_M * math.cos(math.radians(ref.lat)) * 100:.4f} cm")
    print()
    print("  Corner   north (m)   east (m)   |   u sweep (m)   v step (m)   dist from A (m)")
    for corner, ne, uv in zip(plan.corners, plan.corner_ne, plan.polygon_uv):
        print(f"    {corner.name}     {ne.north:+9.3f}  {ne.east:+9.3f}   |"
              f"   {uv[0]:+10.3f}  {uv[1]:+10.3f}   {math.hypot(ne.north, ne.east):11.3f}")

    heading("3. POLYGON GEOMETRY")
    print("  Side lengths:")
    print(f"    AB (sweep edge)  : {m['ab']:8.3f} m")
    print(f"    BC (step edge)   : {m['bc']:8.3f} m")
    print(f"    CD (sweep edge)  : {m['cd']:8.3f} m")
    print(f"    DA (step edge)   : {m['da']:8.3f} m")
    print(f"    perimeter        : {m['perimeter']:8.3f} m")
    sides = (("AB", m["ab"]), ("BC", m["bc"]), ("CD", m["cd"]), ("DA", m["da"]))
    longest_name, longest_len = max(sides, key=lambda s: s[1])
    shortest_name, shortest_len = min(sides, key=lambda s: s[1])
    print(f"    longest side     : {longest_len:8.3f} m  ({longest_name})")
    print(f"    shortest side    : {shortest_len:8.3f} m  ({shortest_name})")
    print("  Diagonals:")
    print(f"    AC               : {m['ac']:8.3f} m")
    print(f"    BD               : {m['bd']:8.3f} m")

    # Centroid and the radius that actually contains the corners. Neither is the
    # geofence radius: the geofence is centred on the post-takeoff hover origin,
    # which is not this point and is not known until the aircraft is in the air.
    cu, cv = centroid(plan.polygon_uv)
    c_ne = from_uv(cu, cv, plan.corner_ne[0], plan.u_hat, plan.v_hat)
    c_lat, c_lon = ne_to_gps(c_ne.north, c_ne.east, plan.reference)
    corner_radii = [math.hypot(u - cu, v - cv) for u, v in plan.polygon_uv]
    worst_corner = "ABCD"[max(range(4), key=lambda i: corner_radii[i])]
    print("  Centroid:")
    print(f"    u, v             : {cu:+8.3f}, {cv:+8.3f} m from corner A")
    print(f"    north, east      : {c_ne.north:+8.3f}, {c_ne.east:+8.3f} m from corner A")
    print(f"    latitude, long.  : {c_lat:.7f}, {c_lon:.7f}")
    print(f"    max centroid->corner : {max(corner_radii):.3f} m  (corner {worst_corner})")
    print("  Corner angles (informational — a rectangle is NOT required):")
    for label in "ABCD":
        angle = m[f"angle_{label.lower()}"]
        print(f"    {label}                : {angle:8.3f} deg   ({angle - 90.0:+.3f} deg from square)")
    print(f"    sum              : {sum(m[f'angle_{l.lower()}'] for l in 'ABCD'):8.3f} deg (must be 360)")
    print("  Area:")
    print(f"    polygon area     : {m['area']:8.3f} m^2  (shoelace, true polygon)")
    inset_area = abs(signed_area(plan.inset_uv))
    print(f"    inset area       : {inset_area:8.3f} m^2  "
          f"({100.0 * inset_area / m['area']:.1f}% of the polygon)")
    print(f"  Raster direction   : A->B, bearing {bearing_deg(plan.u_hat):.2f} deg true")
    print(f"    sweep axis u     : north {plan.u_hat[0]:+.6f}, east {plan.u_hat[1]:+.6f}")
    print(f"    step  axis v     : north {plan.v_hat[0]:+.6f}, east {plan.v_hat[1]:+.6f} "
          f"(bearing {bearing_deg(plan.v_hat):.2f} deg)")

    heading("4. POLYGON VALIDITY")
    print("  Validity : PASS")
    print("    - four distinct corners, all separations above the minimum")
    print("    - all side lengths within the sanity limits")
    print("    - A->B->C->D winds consistently and is a simple CONVEX polygon")
    print(f"    - area {m['area']:.3f} m^2 is above the {RASTER_MIN_AREA_M2:.1f} m^2 minimum")
    print("    - spacing and inset values are usable")
    print("  Note: rectangularity is NOT required. The corner angles above are reported")
    print("        for information only; the area is treated as the real polygon.")

    heading("5. INWARD SAFETY INSET")
    print(f"  Inset distance      : {plan.inset_m:.3f} m inward from every edge")
    print(f"  Method              : half-plane clipping (exact for a convex polygon)")
    print(f"  Inset polygon has {len(plan.inset_uv)} vertices:")
    for i, (u_m, v_m) in enumerate(plan.inset_uv):
        ne = from_uv(u_m, v_m, plan.corner_ne[0], plan.u_hat, plan.v_hat)
        lat, lon = ne_to_gps(ne.north, ne.east, plan.reference)
        print(f"    V{i}  u={u_m:+7.3f}  v={v_m:+7.3f}   north={ne.north:+7.3f}  east={ne.east:+7.3f}"
              f"   {lat:.7f}, {lon:.7f}")
    inset_u = [q[0] for q in plan.inset_uv]
    inset_v = [q[1] for q in plan.inset_uv]
    inset_sides = [math.hypot(plan.inset_uv[i][0] - plan.inset_uv[(i + 1) % len(plan.inset_uv)][0],
                              plan.inset_uv[i][1] - plan.inset_uv[(i + 1) % len(plan.inset_uv)][1])
                   for i in range(len(plan.inset_uv))]
    print("  Inset polygon dimensions:")
    print(f"    extent along u (sweep) : {max(inset_u) - min(inset_u):7.3f} m "
          f"({min(inset_u):+.3f} .. {max(inset_u):+.3f})")
    print(f"    extent along v (step)  : {max(inset_v) - min(inset_v):7.3f} m "
          f"({min(inset_v):+.3f} .. {max(inset_v):+.3f})")
    print(f"    side lengths           : "
          + ", ".join(f"{s:.3f}" for s in inset_sides) + " m")
    print(f"    area                   : {inset_area:7.3f} m^2")

    heading("6. RASTER PLAN")
    v_values = [p[1] for p in plan.inset_uv]
    print(f"  Raster direction           : A->B (bearing {bearing_deg(plan.u_hat):.2f} deg), "
          f"alternating each pass")
    print(f"  Step-axis extent of inset  : {min(v_values):.3f} .. {max(v_values):.3f} m "
          f"(span {max(v_values) - min(v_values):.3f} m)")
    print(f"  Requested pass spacing     : {plan.pass_spacing_requested_m:.4f} m")
    print(f"  Actual pass spacing        : {plan.pass_spacing_actual_m:.4f} m"
          f"   ({'<=' if plan.pass_spacing_actual_m <= plan.pass_spacing_requested_m + 1e-9 else '>'}"
          f" requested)")
    print(f"  Number of passes           : {plan.pass_count}")
    print(f"  Number of waypoints        : {len(plan.waypoints)}  (2 per pass: start + end)")
    print(f"  Number of segments         : {len(plan.segments)}  "
          f"({sum(1 for s in plan.segments if s.kind == 'pass')} passes + "
          f"{sum(1 for s in plan.segments if s.kind == 'transition')} transitions)")
    print(f"  Sweep distance             : "
          f"{sum(s.length_m for s in plan.segments if s.kind == 'pass'):.3f} m")
    print(f"  Transition distance        : "
          f"{sum(s.length_m for s in plan.segments if s.kind == 'transition'):.3f} m")
    print(f"  Total path length          : {plan.path_length_m:.3f} m")
    print(f"  Estimated flight time      : {plan.estimated_time_s:.1f} s "
          f"({plan.estimated_time_s / 60.0:.2f} min) at {plan.speed_mps:.2f} m/s")
    if plan.dropped_passes:
        print(f"  Dropped candidate lines    : {len(plan.dropped_passes)}")
        for v_m, length, reason in plan.dropped_passes:
            print(f"    v={v_m:+7.3f}  clipped length {length:.3f} m  — {reason}")
    else:
        print("  Dropped candidate lines    : 0")
    print()
    print("  Each pass is the raster line CLIPPED to the inset polygon, so pass lengths")
    print("  follow the real shape of the area. Transitions are straight lines between")
    print("  consecutive pass ends; the inset polygon is convex, so they cannot leave it.")
    print("  Turn arcs are not generated — the flight-side proportional controller")
    print("  rounds each corner on its own.")
    print()
    print("  The time estimate covers horizontal travel only: no takeoff, hover, turn")
    print("  settling, or landing. Treat it as a lower bound.")

    heading("7. RASTER PASSES (start / end)")
    print("  Pass  Dir   v offset    u span            Start (north, east)      "
          "End (north, east)        Length")
    for i in range(plan.pass_count):
        start, end = plan.waypoints[2 * i], plan.waypoints[2 * i + 1]
        arrow = "->" if start.direction == "A->B" else "<-"
        length = math.hypot(end.north_m - start.north_m, end.east_m - start.east_m)
        print(f"   {start.pass_index:3d}   {arrow}  {start.v_m:8.3f}  "
              f"{min(start.u_m, end.u_m):6.3f}..{max(start.u_m, end.u_m):6.3f}  "
              f"({start.north_m:+7.3f}, {start.east_m:+7.3f})  "
              f"({end.north_m:+7.3f}, {end.east_m:+7.3f})  {length:6.3f} m")

    heading("8. WAYPOINTS")
    print("   WP  Pass  Kind        Dir     north(m)  east(m)   u(m)    v(m)   dist A(m)  "
          "insetClr  polyClr   latitude     longitude")
    for w in plan.waypoints:
        print(f"  {w.index:3d}  {w.pass_index:4d}  {w.kind:10s}  {w.direction:5s} "
              f"{w.north_m:+8.3f} {w.east_m:+8.3f}  {w.u_m:6.3f}  {w.v_m:6.3f}  "
              f"{w.dist_from_a_m:8.3f}  {w.inset_clearance_m:8.3f}  {w.polygon_clearance_m:7.3f}  "
              f"{w.lat:.7f}  {w.lon:.7f}")

    heading("9. PATH SHAPE")
    ascii_plot(plan)

    heading("10. CONTAINMENT VERIFICATION")
    problems, metrics = verify_containment(plan)
    print("  Waypoint containment:")
    print(f"    waypoints checked                    : {metrics['waypoints']}")
    print(f"    inside the inset polygon             : "
          f"{metrics['waypoints_inside_inset']}/{metrics['waypoints']}")
    print(f"    inside the mission polygon           : "
          f"{metrics['waypoints_inside_polygon']}/{metrics['waypoints']}")
    print(f"    worst waypoint clearance (inset)     : "
          f"{metrics['min_waypoint_inset_clearance_m']:+.4f} m")
    print(f"    worst waypoint clearance (polygon)   : "
          f"{metrics['min_waypoint_polygon_clearance_m']:+.4f} m")
    print("  Segment containment (NOT just the endpoints):")
    print(f"    segments checked                     : {metrics['segments']} "
          f"(every pass and every transition)")
    print(f"    points sampled along segments        : {metrics['segment_samples']} "
          f"at {metrics['sample_step_m']:.3f} m spacing")
    worst_inset_seg = metrics["worst_inset_segment"]
    worst_poly_seg = metrics["worst_polygon_segment"]
    print(f"    worst segment clearance (inset)      : "
          f"{metrics['min_segment_inset_clearance_m']:+.4f} m "
          f"(segment {worst_inset_seg.index}, {worst_inset_seg.kind}, pass {worst_inset_seg.pass_index})")
    print(f"    worst segment clearance (polygon)    : "
          f"{metrics['min_segment_polygon_clearance_m']:+.4f} m "
          f"(segment {worst_poly_seg.index}, {worst_poly_seg.kind}, pass {worst_poly_seg.pass_index})")
    print("  Per-segment detail:")
    print("    Seg  Kind        Pass  Length   Samples  minInsetClr  minPolyClr")
    for seg in plan.segments:
        print(f"    {seg.index:3d}  {seg.kind:10s}  {seg.pass_index:4d}  {seg.length_m:6.3f}  "
              f"{seg.samples:7d}  {seg.min_inset_clearance_m:+11.4f}  "
              f"{seg.min_polygon_clearance_m:+10.4f}")
    print()
    print(f"  Maximum distance from corner A         : {metrics['max_dist_from_a_m']:.3f} m")
    print(f"  Maximum distance from polygon centroid : {metrics['max_dist_from_centroid_m']:.3f} m")
    print()
    print("  NOTE: neither distance is the companion horizontal geofence radius. The")
    print("  geofence is centred on the vehicle's post-takeoff hover position, captured")
    print("  in flight, and is an independent safety layer — not this polygon.")

    if problems:
        report_rejection("a waypoint or segment escaped the mission area", problems)
        if not args.no_geojson:
            out = write_geojson(args.geojson, plan, metrics, status="FAILED_CONTAINMENT",
                                outer_corner_latlon=outer_corners)
            print(f"\n  GeoJSON written for inspection: {out}")
        return 4

    print()
    print("  CONTAINMENT VERIFICATION: PASS")
    print("    - every waypoint lies inside the inset polygon and the mission polygon")
    print("    - every sampled point on every pass and every transition lies inside both")

    # -----------------------------------------------------------------------
    heading("11. OUTER SAFETY GEOFENCE")
    nested, om = verify_inner_inside_outer(plan, outer_corners, args.outer_margin)
    names = ("G1", "G2", "G3", "G4")
    print("  Corner   north (m)   east (m)   |  dist from inner A (m)")
    for name, (n, e) in zip(names, om["outer_ring_ne"]):
        print(f"    {name:<4}  {n:+9.3f}  {e:+9.3f}   |      {math.hypot(n, e):9.3f}")
    print("  Side lengths (in the order the ring is stored, normalised CCW):")
    for i, side in enumerate(om["outer_sides_m"]):
        print(f"    side {i}          : {side:8.3f} m")
    print(f"    perimeter        : {om['outer_perimeter_m']:8.3f} m")
    print(f"  Area               : {om['outer_area_m2']:8.3f} m^2  "
          f"(inner is {100.0 * m['area'] / om['outer_area_m2']:.1f}% of it)")
    print(f"  Winding            : {om['outer_winding']} as stored, normalised CCW for the "
          "half-plane test")
    print(f"  Inward safety margin : {om['margin_m']:.3f} m")
    print(f"  Effective boundary   : {om['effective_area_m2']:.3f} m^2 "
          f"({om['outer_area_m2'] - om['effective_area_m2']:.3f} m^2 given up to the margin)")

    heading("12. INNER INSIDE OUTER")
    print("  Clearance from each inner corner to the OUTER boundary:")
    for name in "ABCD":
        c = om["corner_clearance_m"][name]
        print(f"    corner {name}         : {c:+8.3f} m   "
              f"(after the {om['margin_m']:.2f} m margin: {c - om['margin_m']:+.3f} m)")
    edge_i = om["min_edge_clearance_at"][0] if om["min_edge_clearance_at"] else 0
    edge_label = f"{'ABCD'[edge_i]}{'ABCD'[(edge_i + 1) % 4]}"
    print(f"  Minimum over the inner EDGES (not just corners), "
          f"{om['edge_samples']} samples:")
    edge_t = om["min_edge_clearance_at"][1] if om["min_edge_clearance_at"] else 0.0
    print(f"    minimum clearance  : {om['min_edge_clearance_m']:+8.3f} m  on edge "
          f"{edge_label} at {edge_t * 100:.0f}% along it")
    print(f"    after outer margin : {om['min_edge_clearance_m'] - om['margin_m']:+8.3f} m")
    print(f"  Minimum over the RASTER PATH, {om['path_samples']} samples:")
    print(f"    to the raw outer boundary       : {om['min_path_clearance_m']:+8.3f} m")
    print(f"    to the effective outer boundary : "
          f"{om['min_path_clearance_effective_m']:+8.3f} m")

    if nested:
        report_rejection("the inner raster polygon is not safely contained by the outer "
                         "safety geofence", nested,
                         "The inner polygon must sit wholly inside the outer one, with room\n"
                         "left over for the outer safety margin. Neither polygon is adjusted\n"
                         "automatically and the margin is not reduced to make it fit.")
        if not args.no_geojson:
            out = write_geojson(args.geojson, plan, metrics, status="FAILED_NESTING",
                                outer_corner_latlon=outer_corners,
                                outer_margin_m=args.outer_margin)
            print(f"\n  GeoJSON written for inspection: {out}")
        return 4

    print()
    print("  OUTER CONTAINMENT: PASS")
    print("    - all four inner corners are inside the outer safety geofence")
    print("    - every sampled point on every inner edge is inside it")
    print("    - every sampled point on the raster path is inside the EFFECTIVE boundary")

    # -----------------------------------------------------------------------
    heading("13. PASS-SPACING COMPARISON (informational only)")
    print(f"  All at {args.speed:.2f} m/s and a {args.inset:.2f} m inset. Nothing here")
    print("  changes the configured spacing; it is for choosing one by hand.")
    print(f"  {'spacing':>9} {'passes':>7} {'waypts':>7} {'sweep':>9} {'trans':>8} "
          f"{'path':>9} {'raster time':>14}")
    for candidate in (0.40, 0.60, 0.80, 1.00):
        try:
            alt = plan_raster(args.corners, candidate, args.inset, args.speed)
        except PlanError as exc:
            print(f"  {candidate:9.2f}   rejected: {exc.summary}")
            continue
        sweep = sum(x.length_m for x in alt.segments if x.kind == "pass")
        trans = sum(x.length_m for x in alt.segments if x.kind == "transition")
        marker = "  <- configured" if abs(candidate - args.spacing) < 1e-9 else ""
        print(f"  {candidate:9.2f} {alt.pass_count:7d} {len(alt.waypoints):7d} "
              f"{sweep:9.2f} {trans:8.2f} {alt.path_length_m:9.2f} "
              f"{alt.estimated_time_s:8.0f}s {alt.estimated_time_s / 60:5.2f}min{marker}")

    heading("14. GEOJSON")
    if args.no_geojson:
        print("  Skipped (--no-geojson).")
    else:
        out = write_geojson(args.geojson, plan, metrics,
                            outer_corner_latlon=outer_corners,
                            outer_margin_m=args.outer_margin)
        print(f"  Written: {os.path.abspath(out)}")
        print("  Coordinate order: WGS-84 [longitude, latitude].")
        print("  Features: outer safety geofence, effective outer geofence, inner raster")
        print(f"  polygon, inner inset polygon, raster path (LineString), "
              f"{len(plan.waypoints)} waypoints, G1-G4 markers, A-D markers.")
        print("  Review by dragging onto https://geojson.io over the satellite layer.")

    heading("RESULT: PASS")
    print(f"  {plan.pass_count} passes, {len(plan.waypoints)} waypoints, "
          f"{plan.pass_spacing_actual_m:.3f} m spacing, {plan.path_length_m:.1f} m of path, "
          f"~{plan.estimated_time_s:.0f} s at {plan.speed_mps:.2f} m/s.")
    print("  Geometry only. Nothing here has been sent to a flight controller.")
    print(BAR)
    return 0


if __name__ == "__main__":
    sys.exit(main())

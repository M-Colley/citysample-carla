#!/usr/bin/env python3
"""
roadnetwork_to_xodr.py - convert an extracted road network to ASAM OpenDRIVE 1.4.

Stage 2 of the City Sample -> CARLA map pipeline. Stage 1
(``zonegraph_to_roadnetwork.py``, fed by the ExportZoneGraph commandlet) writes
the intermediate JSON this consumes. This half is plain Python with no Unreal
dependency, so it can be developed and tested without the editor.

    python tools/roadnetwork_to_xodr.py network.json -o CitySample.xodr
    python tools/roadnetwork_to_xodr.py --selftest

Intermediate JSON schema (all coordinates in UNREAL space: centimetres,
left-handed, X forward / Y right / Z up - the conversion to OpenDRIVE's
right-handed metres happens here, in one place):

    {
      "source": "...",
      "roads": [
        {
          "id": "road_0",
          "points": [{"pos": [x, y, z], "tangent": [tx, ty, tz]}, ...],
          "lane_width_cm": 350.0,
          "unidirectional": false,
          "has_center_divider": false,
          "center_divider_width_cm": 0.0,
          "is_freeway": false,
          "predecessor": {"type": "junction", "id": "int_3"},   # optional
          "successor":   {"type": "junction", "id": "int_4"}    # optional
        }
      ],
      "junctions": [ {"id": "int_3", "center": [x, y, z], "connections": [
          {"incoming": "road_0", "connecting": "road_7", "contact": "start",
           "lane_links": [[-1, -1]]} ]} ]
    }

Why paramPoly3 and not a polyline: CARLA derives waypoint headings from the
planView. A chain of ``<line>`` elements gives a heading discontinuity at every
vertex, which shows up as steering jitter in lane-following. Cubic Hermite
segments carry the authored tangents through, so headings stay continuous.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field


def load_json_any(path: str):
    """Load JSON written by either Python or UE.

    UE's TJsonWriter<TCHAR> emits UTF-16LE with no BOM, so a plain utf-8 open
    fails on the very first byte. Sniff instead of assuming.
    """
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-16", "utf-16-le", "utf-16-be", "utf-8"):
        try:
            return json.loads(raw.decode(enc))
        except (UnicodeDecodeError, UnicodeError, json.JSONDecodeError):
            continue
    raise ValueError(f"could not decode {path} as JSON in any known encoding")

CM_TO_M = 0.01


def junction_id_base(n_roads: int) -> int:
    """First junction id: the smallest power of ten above every road id.

    Road and junction ids MUST NOT overlap. LibCarla's RoadParser keeps a
    road link's elementId and throws its elementType away, and
    MapBuilder::GetLaneNext then decides whether a successor is a junction by
    asking whether a ROAD with that id exists (`next_is_junction =
    !ContainsRoad(next_road)`). Junctions numbered 0..J-1 beside roads 0..R-1
    therefore resolve as roads, every road entering a junction gets no
    successor lane at all, the traffic manager destroys each vehicle as its
    path buffer reaches a junction (silently - osm_mode defaults to true), and
    waypoint.next() returns [] at every junction approach.

    A power of ten keeps junctions recognisable in logs: road 42, junction 10042.
    """
    return 10 ** len(str(max(n_roads - 1, 0)))


# --------------------------------------------------------------------------
# coordinate conversion - the single place UE space becomes OpenDRIVE space
# --------------------------------------------------------------------------
def ue_to_odr(v: list[float]) -> tuple[float, float, float]:
    """Unreal (cm, left-handed, +Y right) -> OpenDRIVE (m, right-handed, +Y left).

    This is the same handedness flip CARLA applies between its own world space
    and Unreal's: negate Y, scale cm to m.
    """
    return (v[0] * CM_TO_M, -v[1] * CM_TO_M, v[2] * CM_TO_M)


def heading(tangent: list[float]) -> float:
    """Planar heading in OpenDRIVE space, radians, from an Unreal tangent."""
    return math.atan2(-tangent[1], tangent[0])


# --------------------------------------------------------------------------
@dataclass
class Geometry:
    s: float
    x: float
    y: float
    hdg: float
    length: float
    # paramPoly3 coefficients in the segment's local (u, v) frame
    bu: float
    cu: float
    du: float
    bv: float
    cv: float
    dv: float


@dataclass
class Road:
    rid: str
    numeric_id: int
    geometries: list[Geometry] = field(default_factory=list)
    elevations: list[tuple[float, float, float, float, float]] = field(default_factory=list)
    length: float = 0.0
    lane_width_m: float = 3.5
    lane_widths_m: list[float] = field(default_factory=list)
    unidirectional: bool = False
    has_center_divider: bool = False
    is_freeway: bool = False
    road_type: str = "town"
    speed_kmh: float = 30.0
    lane_count: int = 1
    predecessor: dict | None = None
    successor: dict | None = None
    lane_succ_ids: list[int | None] = field(default_factory=list)
    lane_pred_ids: list[int | None] = field(default_factory=list)
    junction: str | None = None
    lane_offset_m: float = 0.0
    lane_type: str = "driving"


def _hermite_to_parampoly3(p0, p1, m0, m1, hdg0):
    """Cubic Hermite segment -> OpenDRIVE paramPoly3 in the local frame at p0.

    Hermite:  H(t) = h00*p0 + h10*m0 + h01*p1 + h11*m1, t in [0,1]
      h00 = 2t^3-3t^2+1,  h10 = t^3-2t^2+t,  h01 = -2t^3+3t^2,  h11 = t^3-t^2

    Expanded in the monomial basis and rotated into the local frame (u along
    hdg0, v to its left), with aU = aV = bV = 0 by construction because the
    segment starts at the origin travelling along +u.
    """
    cos_h, sin_h = math.cos(-hdg0), math.sin(-hdg0)

    def to_local(px, py):
        dx, dy = px - p0[0], py - p0[1]
        return (dx * cos_h - dy * sin_h, dx * sin_h + dy * cos_h)

    # rotate the endpoint and both tangents into the local frame
    p1l = to_local(p1[0], p1[1])
    m0l = (m0[0] * cos_h - m0[1] * sin_h, m0[0] * sin_h + m0[1] * cos_h)
    m1l = (m1[0] * cos_h - m1[1] * sin_h, m1[0] * sin_h + m1[1] * cos_h)

    # monomial coefficients: f(t) = a + b t + c t^2 + d t^3, with a = 0
    bu, bv = m0l[0], m0l[1]
    cu = 3.0 * p1l[0] - 2.0 * m0l[0] - m1l[0]
    cv = 3.0 * p1l[1] - 2.0 * m0l[1] - m1l[1]
    du = -2.0 * p1l[0] + m0l[0] + m1l[0]
    dv = -2.0 * p1l[1] + m0l[1] + m1l[1]
    return bu, cu, du, bv, cv, dv


def _arc_length(p0, p1, m0, m1, samples: int = 24) -> float:
    """Numeric arc length of the Hermite segment - OpenDRIVE needs a length."""
    total, prev = 0.0, p0
    for i in range(1, samples + 1):
        t = i / samples
        h00 = 2 * t**3 - 3 * t**2 + 1
        h10 = t**3 - 2 * t**2 + t
        h01 = -2 * t**3 + 3 * t**2
        h11 = t**3 - t**2
        pt = (
            h00 * p0[0] + h10 * m0[0] + h01 * p1[0] + h11 * m1[0],
            h00 * p0[1] + h10 * m0[1] + h01 * p1[1] + h11 * m1[1],
        )
        total += math.hypot(pt[0] - prev[0], pt[1] - prev[1])
        prev = pt
    return total


def build_road(spec: dict, numeric_id: int) -> Road | None:
    pts = spec.get("points") or []
    if len(pts) < 2:
        return None

    n_lanes = max(1, int(spec.get("lane_count", 1)))
    base_w_m = float(spec.get("lane_width_cm", 350.0)) * CM_TO_M
    raw_widths = spec.get("lane_widths_cm")
    if isinstance(raw_widths, list) and len(raw_widths) == n_lanes:
        lane_widths_m = [float(w) * CM_TO_M for w in raw_widths]
    else:
        lane_widths_m = [base_w_m] * n_lanes

    raw_succ_ids = spec.get("lane_succ_ids")
    lane_succ_ids = (
        [int(x) if x is not None else None for x in raw_succ_ids]
        if isinstance(raw_succ_ids, list) and len(raw_succ_ids) == n_lanes
        else []
    )
    raw_pred_ids = spec.get("lane_pred_ids")
    lane_pred_ids = (
        [int(x) if x is not None else None for x in raw_pred_ids]
        if isinstance(raw_pred_ids, list) and len(raw_pred_ids) == n_lanes
        else []
    )

    road = Road(
        rid=str(spec.get("id", f"road_{numeric_id}")),
        numeric_id=numeric_id,
        lane_width_m=base_w_m,
        lane_widths_m=lane_widths_m,
        unidirectional=bool(spec.get("unidirectional", False)),
        has_center_divider=bool(spec.get("has_center_divider", False)),
        is_freeway=bool(spec.get("is_freeway", False)),
        road_type=str(spec.get("road_type") or ("motorway" if spec.get("is_freeway") else "town")),
        speed_kmh=float(spec.get("speed_kmh", 80.0 if spec.get("is_freeway") else 30.0)),
        lane_count=n_lanes,
        predecessor=spec.get("predecessor"),
        successor=spec.get("successor"),
        lane_succ_ids=lane_succ_ids,
        lane_pred_ids=lane_pred_ids,
        junction=spec.get("junction"),
        lane_offset_m=float(spec.get("lane_offset_cm", 0.0)) * CM_TO_M,
        # Pedestrian lanes (zonegraph_to_roadnetwork --keep-sidewalks) must be
        # written as sidewalks: as "driving", CARLA routes traffic down them and
        # spawns vehicles on them. Freeway "entry"/"exit" stay "driving" on
        # purpose - CARLA's waypoints and traffic manager only use driving lanes,
        # so typing the ramps would take them out of every route.
        lane_type="sidewalk" if spec.get("lane_type") == "sidewalk" else "driving",
    )

    # Convert once, up front.
    conv = [ue_to_odr(p["pos"]) for p in pts]
    tans = []
    for i, p in enumerate(pts):
        t = p.get("tangent")
        if not t or (abs(t[0]) < 1e-9 and abs(t[1]) < 1e-9):
            # Fall back to a Catmull-Rom style finite difference.
            a = conv[max(i - 1, 0)]
            b = conv[min(i + 1, len(conv) - 1)]
            tans.append((b[0] - a[0], b[1] - a[1]))
        else:
            tx, ty, _ = ue_to_odr([t[0], t[1], t[2] if len(t) > 2 else 0.0])
            tans.append((tx, ty))

    s = 0.0
    for i in range(len(conv) - 1):
        p0, p1 = conv[i][:2], conv[i + 1][:2]
        chord = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        if chord < 1e-6:
            continue
        # Scale unit tangents to the chord so the Hermite stays well-conditioned.
        def scaled(t):
            n = math.hypot(t[0], t[1])
            k = chord / n if n > 1e-9 else 0.0
            return (t[0] * k, t[1] * k)

        m0, m1 = scaled(tans[i]), scaled(tans[i + 1])
        hdg0 = math.atan2(m0[1], m0[0]) if math.hypot(*m0) > 1e-9 else math.atan2(
            p1[1] - p0[1], p1[0] - p0[0])
        bu, cu, du, bv, cv, dv = _hermite_to_parampoly3(p0, p1, m0, m1, hdg0)
        length = _arc_length(p0, p1, m0, m1)
        road.geometries.append(
            Geometry(s=s, x=p0[0], y=p0[1], hdg=hdg0, length=length,
                     bu=bu, cu=cu, du=du, bv=bv, cv=cv, dv=dv))
        # Elevation as a piecewise-linear profile in s.
        z0, z1 = conv[i][2], conv[i + 1][2]
        slope = (z1 - z0) / length if length > 1e-9 else 0.0
        road.elevations.append((s, z0, slope, 0.0, 0.0))
        s += length

    road.length = s
    return road if road.geometries else None


def _lane(parent: ET.Element, lane_id: int, width_m: float, ltype: str = "driving",
          link_succ: int | None = None, link_pred: int | None = None,
          mark_type: str = "broken", lane_change: str = "both"):
    """Emit one lane.

    link_succ / link_pred are LANE ids in the neighbouring road. Road-level
    <link> alone is not enough: without lane-level links CARLA does not know
    which lane continues into which, the topology stays broken at every road
    boundary, and the traffic manager despawns vehicles that cannot find a next
    waypoint. Junction connectivity is expressed by <laneLink> inside
    <junction> instead, so those are left unlinked here.
    """
    lane = ET.SubElement(parent, "lane", id=str(lane_id), type=ltype, level="false")
    link = ET.SubElement(lane, "link")
    if link_pred is not None:
        ET.SubElement(link, "predecessor", id=str(link_pred))
    if link_succ is not None:
        ET.SubElement(link, "successor", id=str(link_succ))
    ET.SubElement(lane, "width", sOffset="0.0", a=f"{width_m:.6f}",
                  b="0.0", c="0.0", d="0.0")
    ET.SubElement(lane, "roadMark", sOffset="0.0", type=mark_type, weight="standard",
                  color="standard", width="0.15", laneChange=lane_change)
    return lane


# Painted width of a crossing, along the direction of vehicle travel. The
# ZoneGraph lane is the pedestrian's 1 m path; this is the stripe it becomes.
CROSSWALK_PAINT_M = 4.0

# A crossing further than this from any reference line is not on that road.
CROSSWALK_MAX_OFFSET_M = 25.0


def carla_polyline(g: Geometry) -> list[tuple[float, float, float, float, float]]:
    """(s, x, y, tx, ty) along one geometry, sampled exactly as LibCarla does.

    GeometryParamPoly3::PreComputeSpline steps the NORMALISED parameter in
    max(int(length / 0.5), 5) equal intervals and measures s as the running
    chord length; PosFromDist then interpolates position and tangent linearly
    between the two samples around s. An object placed at an s worked out any
    other way - a straight-line projection, or s = p * length - lands
    somewhere else in CARLA. (tx, ty) is the world-frame tangent, unnormalised,
    because that is what CARLA interpolates.
    """
    n = max(int(float(f"{g.length:.6f}") / 0.5), 5)
    cos_h, sin_h = math.cos(g.hdg), math.sin(g.hdg)
    out, s, prev = [], g.s, None
    for i in range(n + 1):
        p = i / n
        u = ((g.du * p + g.cu) * p + g.bu) * p
        v = ((g.dv * p + g.cv) * p + g.bv) * p
        du = (3.0 * g.du * p + 2.0 * g.cu) * p + g.bu
        dv = (3.0 * g.dv * p + 2.0 * g.cv) * p + g.bv
        x, y = g.x + u * cos_h - v * sin_h, g.y + u * sin_h + v * cos_h
        if prev is not None:
            s += math.hypot(x - prev[0], y - prev[1])
        out.append((s, x, y, du * cos_h - dv * sin_h, du * sin_h + dv * cos_h))
        prev = (x, y)
    return out


def _bbox(g: Geometry) -> tuple[float, float, float, float]:
    """Bounding box of one geometry. A cubic lies inside the convex hull of
    its Bezier control points, so their box contains the whole curve."""
    cos_h, sin_h = math.cos(g.hdg), math.sin(g.hdg)
    us = (0.0, g.bu / 3.0, (2.0 * g.bu + g.cu) / 3.0, g.bu + g.cu + g.du)
    vs = (0.0, g.bv / 3.0, (2.0 * g.bv + g.cv) / 3.0, g.bv + g.cv + g.dv)
    xs = [g.x + u * cos_h - v * sin_h for u, v in zip(us, vs)]
    ys = [g.y + u * sin_h + v * cos_h for u, v in zip(us, vs)]
    return min(xs), min(ys), max(xs), max(ys)


def emit_crosswalks(roads, crosswalks, road_elems):
    """Emit <object type="crosswalk"> polygons.

    Three things here are load-bearing:

    * The outline must be CLOSED - the first corner repeated as the last.
      carla::road::Map::GetAllCrosswalkMesh ends a triangle fan only when it
      sees crosswalk_vertex[start] == crosswalk_vertex[i]; four unrepeated
      corners produce a crosswalk-material mesh with ZERO triangles. Every
      stock town writes five for this reason.
    * The object sits ON a reference line (t = 0), at the point where the
      crossing cuts it. CARLA does not apply t the way the standard says:
      0.10's Map::GetAllCrosswalkZones turns the lateral offset by the
      object's own hdg before applying it, so a crossing written as "t metres
      across the road, hdg 90 degrees" lands t metres ALONG the road, off the
      painted crossing. (0.9.16 applies t correctly; at t = 0 the two agree.)
    * Candidates are found by each geometry's bounding box, and s comes from
      the curve CARLA actually evaluates (carla_polyline). Indexing segments by
      their ORIGIN missed long segments whose origin was far from the
      crossing, and projecting onto the straight line through the origin put
      crossings on curved roads metres out.

    What remains is CARLA's own deviation, and it cannot be written around
    without breaking the standard: both 0.9.16 and 0.10 measure an object
    from lane 0 *after* laneOffset (Road::GetDirectedPointIn), not from the
    reference line, so in CARLA every crossing sits laneOffset (half a lane)
    further along its own axis. The navmesh this repository builds follows the
    standard and is exact.
    """
    if not crosswalks:
        return 0, 0

    cell = 25.0
    grid: dict[tuple[int, int], list] = {}
    for r in roads:
        for g in r.geometries:
            box = _bbox(g)
            for cx in range(int(box[0] // cell), int(box[2] // cell) + 1):
                for cy in range(int(box[1] // cell), int(box[3] // cell) + 1):
                    grid.setdefault((cx, cy), []).append((r, g, box))
    polylines: dict[int, list] = {}

    def near(x0, y0, x1, y1):
        """Geometries whose box overlaps this one, sampled only when they do."""
        seen = set()
        for cx in range(int(x0 // cell), int(x1 // cell) + 1):
            for cy in range(int(y0 // cell), int(y1 // cell) + 1):
                for r, g, box in grid.get((cx, cy), ()):
                    if (id(g) in seen or box[2] < x0 or box[0] > x1
                            or box[3] < y0 or box[1] > y1):
                        continue
                    seen.add(id(g))
                    if id(g) not in polylines:
                        polylines[id(g)] = carla_polyline(g)
                    yield r, polylines[id(g)]

    by_road, projected = {}, 0
    for i, cw in enumerate(crosswalks):
        ax, ay, _ = ue_to_odr(cw["a_cm"])
        bx, by, _ = ue_to_odr(cw["b_cm"])
        dx, dy = bx - ax, by - ay
        span = math.hypot(dx, dy)
        if span <= 0.5:
            continue
        cross_hdg = math.atan2(dy, dx)

        # Where the crossing cuts a reference line, nearest its middle.
        best = None
        for r, pl in near(min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)):
            for (s0, x0, y0, tx0, ty0), (s1, x1, y1, tx1, ty1) in zip(pl, pl[1:]):
                ex, ey = x1 - x0, y1 - y0
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                lam = ((x0 - ax) * ey - (y0 - ay) * ex) / den
                mu = ((x0 - ax) * dy - (y0 - ay) * dx) / den
                if not (0.0 <= lam <= 1.0 and 0.0 <= mu <= 1.0):
                    continue
                if best is None or abs(lam - 0.5) < best[0]:
                    road_hdg = math.atan2(ty0 + mu * (ty1 - ty0), tx0 + mu * (tx1 - tx0))
                    best = (abs(lam - 0.5), r, s0 + mu * (s1 - s0), 0.0,
                            road_hdg, -lam * span, (1.0 - lam) * span)
        if best is None:
            # It crosses no reference line - nearest point instead, with a
            # lateral offset. Exact in a standard reader and in CARLA 0.9.16;
            # CARLA 0.10 rotates that offset (see above), so these are counted.
            mx, my = ax + dx / 2.0, ay + dy / 2.0
            reach = CROSSWALK_MAX_OFFSET_M
            for r, pl in near(mx - reach, my - reach, mx + reach, my + reach):
                for (s0, x0, y0, tx0, ty0), (s1, x1, y1, tx1, ty1) in zip(pl, pl[1:]):
                    ex, ey = x1 - x0, y1 - y0
                    ee = ex * ex + ey * ey
                    mu = 0.0 if ee < 1e-12 else max(0.0, min(1.0, ((mx - x0) * ex + (my - y0) * ey) / ee))
                    px, py = x0 + mu * ex, y0 + mu * ey
                    d = math.hypot(mx - px, my - py)
                    if d <= reach and (best is None or d < best[0]):
                        tx, ty = tx0 + mu * (tx1 - tx0), ty0 + mu * (ty1 - ty0)
                        road_hdg = math.atan2(ty, tx)
                        # t is signed: left of the reference line is positive.
                        t = (-(mx - px) * math.sin(road_hdg) + (my - py) * math.cos(road_hdg))
                        best = (d, r, s0 + mu * (s1 - s0), t, road_hdg,
                                -span / 2.0, span / 2.0)
            if best is None:
                continue
            projected += 1
        _, road, s_pos, t_pos, road_hdg, u0, u1 = best
        by_road.setdefault(road.numeric_id, []).append(
            (i, s_pos, t_pos, u0, u1, cross_hdg - road_hdg))

    n = 0
    for road in roads:
        items = by_road.get(road.numeric_id)
        if not items:
            continue
        elem = road_elems.get(road.numeric_id)
        if elem is None:
            continue
        objects = elem.find("objects")
        if objects is None:
            objects = ET.SubElement(elem, "objects")
        for (i, s_pos, t_pos, u0, u1, rel_hdg) in items:
            # length runs along the object's u axis (here, along the crossing)
            # and width along v, as ASAM defines them.
            obj = ET.SubElement(
                objects, "object",
                id=f"cw{road.numeric_id}_{i}", name=f"Crosswalk_{i}",
                type="crosswalk", s=f"{max(0.0, s_pos):.6f}", t=f"{t_pos:.6f}",
                zOffset="0.0", hdg=f"{rel_hdg:.6f}", pitch="0.0", roll="0.0",
                orientation="none",
                length=f"{u1 - u0:.6f}", width=f"{CROSSWALK_PAINT_M:.6f}",
                height="0.0")
            outline = ET.SubElement(obj, "outline")
            hv = CROSSWALK_PAINT_M / 2.0
            # CLOSED ring: first corner repeated. Four corners = zero triangles.
            # u0 < 0 < u1 for an anchored crossing, which CARLA 0.10 relies on
            # when it widens the ends by 1 m each (it extends by sign of u).
            ring = ((u1, hv), (u1, -hv), (u0, -hv), (u0, hv), (u1, hv))
            for u, v in ring:
                ET.SubElement(outline, "cornerLocal",
                              u=f"{u:.6f}", v=f"{v:.6f}", z="0.000000",
                              height="0.000000")
            n += 1
    return n, projected


def to_xodr(net: dict, name: str = "CitySample") -> ET.ElementTree:
    roads: list[Road] = []
    id_map: dict[str, int] = {}
    road_elems: dict[int, ET.Element] = {}
    for i, spec in enumerate(net.get("roads", [])):
        r = build_road(spec, i)
        if r:
            id_map[r.rid] = r.numeric_id
            roads.append(r)

    # Clear of every road id - see junction_id_base for what overlap costs.
    jbase = junction_id_base(len(net.get("roads", [])))
    jn_map = {str(j.get("id")): jbase + i
              for i, j in enumerate(net.get("junctions", []))}

    root = ET.Element("OpenDRIVE")
    xs = [g.x for r in roads for g in r.geometries] or [0.0]
    ys = [g.y for r in roads for g in r.geometries] or [0.0]
    hdr = ET.SubElement(
        root, "header", revMajor="1", revMinor="4", name=name, version="1.00",
        date="", north=f"{max(ys):.6f}", south=f"{min(ys):.6f}",
        east=f"{max(xs):.6f}", west=f"{min(xs):.6f}", vendor="carla-citysample")
    geo = ET.SubElement(hdr, "geoReference")
    geo.text = "+proj=tmerc +lat_0=0 +lon_0=0 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"

    for r in roads:
        # A road that belongs to a junction must declare it; CARLA uses this to
        # decide what is an intersection. Connecting roads inside a junction
        # carry that junction's id here instead of "-1".
        jref = "-1"
        if r.junction and str(r.junction) in jn_map:
            jref = str(jn_map[str(r.junction)])
        attrs = {"name": r.rid, "length": f"{r.length:.6f}",
                 "id": str(r.numeric_id), "junction": jref}
        rd = ET.SubElement(root, "road", **attrs)
        road_elems[r.numeric_id] = rd

        link = ET.SubElement(rd, "link")
        for tag, spec in (("predecessor", r.predecessor), ("successor", r.successor)):
            if not spec:
                continue
            etype = spec.get("type", "road")
            eid = spec.get("id")
            if etype == "junction" and str(eid) in jn_map:
                ET.SubElement(link, tag, elementType="junction",
                              elementId=str(jn_map[str(eid)]))
            elif etype == "road" and str(eid) in id_map:
                ET.SubElement(link, tag, elementType="road",
                              elementId=str(id_map[str(eid)]),
                              contactPoint=spec.get("contact", "start"))

        if r.lane_type == "driving" and r.speed_kmh > 0.0:
            rtype = ET.SubElement(rd, "type", s="0.0", type=r.road_type)
            ET.SubElement(rtype, "speed", max=f"{r.speed_kmh:.0f}", unit="km/h")

        pv = ET.SubElement(rd, "planView")
        for g in r.geometries:
            ge = ET.SubElement(pv, "geometry", s=f"{g.s:.6f}", x=f"{g.x:.6f}",
                               y=f"{g.y:.6f}", hdg=f"{g.hdg:.6f}",
                               length=f"{g.length:.6f}")
            ET.SubElement(ge, "paramPoly3",
                          aU="0.0", bU=f"{g.bu:.6f}", cU=f"{g.cu:.6f}", dU=f"{g.du:.6f}",
                          aV="0.0", bV=f"{g.bv:.6f}", cV=f"{g.cv:.6f}", dV=f"{g.dv:.6f}",
                          pRange="normalized")

        ep = ET.SubElement(rd, "elevationProfile")
        for (s, a, b, c, d) in r.elevations:
            ET.SubElement(ep, "elevation", s=f"{s:.6f}", a=f"{a:.6f}",
                          b=f"{b:.9f}", c=f"{c:.6f}", d=f"{d:.6f}")

        lanes = ET.SubElement(rd, "lanes")
        # Shifts lane 0 laterally off the reference line. With a single
        # right lane of width W and an offset of W/2, that lane's centre
        # lands exactly on the reference line - i.e. on the ZoneGraph
        # centreline - instead of half a lane to the side of it.
        if abs(r.lane_offset_m) > 1e-9:
            ET.SubElement(lanes, "laneOffset", s="0.0",
                          a=f"{r.lane_offset_m:.6f}", b="0.0", c="0.0", d="0.0")
        sec = ET.SubElement(lanes, "laneSection", s="0.0")
        # Honour the producer. zonegraph_to_roadnetwork.py emits one road per
        # ZoneGraph lane, always with lane_count 1 and a laneOffset of half a
        # lane width so that single lane's centre sits on the ZoneGraph
        # centreline. Widening freeways to two lanes here without touching the
        # offset made those roads span t in [-1.5W, +0.5W] instead of
        # [-0.5W, +0.5W]: twice as wide AND off-centre, so adjacent freeway
        # lanes produced overlapping meshes - exactly the failure the
        # one-road-per-lane model exists to avoid (see convert()'s docstring).
        n_side = r.lane_count

        if not r.unidirectional:
            left = ET.SubElement(sec, "left")
            for i in range(n_side, 0, -1):
                is_outer = (i == n_side) or (r.lane_type != "driving")
                w_m = r.lane_widths_m[i - 1] if i - 1 < len(r.lane_widths_m) else r.lane_width_m
                _lane(left, i, w_m, r.lane_type,
                      mark_type="solid" if is_outer else "broken",
                      lane_change="none" if is_outer else "both")

        centre = ET.SubElement(sec, "center")
        c_lane = ET.SubElement(centre, "lane", id="0", type="none", level="false")
        solid_centre = r.has_center_divider or r.unidirectional or r.lane_type != "driving"
        ET.SubElement(c_lane, "roadMark", sOffset="0.0",
                      type="solid" if solid_centre else "broken",
                      weight="standard", color="standard", width="0.15",
                      laneChange="none" if solid_centre else "both")

        right = ET.SubElement(sec, "right")
        # Only a road-to-road link produces a lane-level link; a link into a
        # junction is resolved by that junction's <laneLink> entries.
        succ_is_road = (r.successor or {}).get("type") == "road"
        pred_is_road = (r.predecessor or {}).get("type") == "road"
        for i in range(1, n_side + 1):
            w_m = r.lane_widths_m[i - 1] if i - 1 < len(r.lane_widths_m) else r.lane_width_m
            if r.lane_succ_ids:
                l_succ = r.lane_succ_ids[i - 1] if succ_is_road else None
            else:
                l_succ = -i if succ_is_road else None
            if r.lane_pred_ids:
                l_pred = r.lane_pred_ids[i - 1] if pred_is_road else None
            else:
                l_pred = -i if pred_is_road else None
            is_outer = (i == n_side) or (r.lane_type != "driving")
            _lane(right, -i, w_m, r.lane_type,
                  link_succ=l_succ, link_pred=l_pred,
                  mark_type="solid" if is_outer else "broken",
                  lane_change="none" if is_outer else "both")

    n_cw, n_projected = emit_crosswalks(roads, net.get("crosswalks", []), road_elems)
    if n_cw:
        print(f"  {n_cw:,} crosswalk objects"
              + (f" ({n_projected:,} cross no reference line - placed by projection)"
                 if n_projected else ""))

    for j in net.get("junctions", []):
        jid = jn_map[str(j.get("id"))]
        je = ET.SubElement(root, "junction", id=str(jid), name=str(j.get("id")))
        for ci, conn in enumerate(j.get("connections", [])):
            inc, con = str(conn.get("incoming")), str(conn.get("connecting"))
            if inc not in id_map or con not in id_map:
                continue
            ce = ET.SubElement(je, "connection", id=str(ci),
                               incomingRoad=str(id_map[inc]),
                               connectingRoad=str(id_map[con]),
                               contactPoint=conn.get("contact", "start"))
            for (a, b) in conn.get("lane_links", [[-1, -1]]):
                ET.SubElement(ce, "laneLink", **{"from": str(a), "to": str(b)})

    ET.indent(root, space="    ")
    return ET.ElementTree(root)


def carla_lane_successors(root: ET.Element) -> dict[tuple[int, int], list[tuple[int, int]]]:
    """Successor (road_id, lane_id) pairs of every right-hand lane, resolved
    exactly as LibCarla's MapBuilder::GetLaneNext and GetJunctionLanes do."""
    roads = {int(r.get("id")): r for r in root.findall("road")}
    conns = {int(j.get("id")): [int(c.get("connectingRoad"))
                                for c in j.findall("connection")]
             for j in root.findall("junction")}

    def link(road, tag):
        e = road.find(f"link/{tag}")
        return int(e.get("elementId")) if e is not None else 0

    right_lanes: dict[int, dict[int, ET.Element]] = {}
    for rid, road in roads.items():
        right_lanes[rid] = {
            int(l.get("id")): l
            for l in road.findall("lanes/laneSection/right/lane")
        }

    def lane_link_id(lane_el: ET.Element, tag: str) -> int:
        e = lane_el.find(f"link/{tag}")
        return int(e.get("id")) if e is not None else 0

    out: dict[tuple[int, int], list[tuple[int, int]]] = {}
    for rid, road in roads.items():
        nxt = link(road, "successor")
        for lid, lane_el in right_lanes[rid].items():
            if nxt in roads:
                target_lid = lane_link_id(lane_el, "successor")
                if target_lid != 0 and target_lid in right_lanes[nxt]:
                    out[(rid, lid)] = [(nxt, target_lid)]
                else:
                    out[(rid, lid)] = []
                continue
            j_opts: list[tuple[int, int]] = []
            for c in conns.get(nxt, ()):
                if c not in roads:
                    continue
                c_pred = link(roads[c], "predecessor")
                c_succ = link(roads[c], "successor")
                if c_pred == rid:
                    for clid, cl_el in right_lanes[c].items():
                        if lane_link_id(cl_el, "predecessor") == lid:
                            j_opts.append((c, clid))
                if c_succ == rid:
                    for clid, cl_el in right_lanes[c].items():
                        if lane_link_id(cl_el, "successor") == lid:
                            j_opts.append((c, clid))
            out[(rid, lid)] = j_opts
    return out


def carla_successors(root: ET.Element) -> dict[int, list[int]]:
    """Successor roads of every road's lane -1, resolved the way LibCarla does."""
    lane_succ = carla_lane_successors(root)
    roads = {int(r.get("id")): r for r in root.findall("road")}
    return {
        rid: [nr for (nr, _) in lane_succ.get((rid, -1), [])]
        for rid in roads
    }


# --------------------------------------------------------------------------
def selftest() -> int:
    """Round-trip a synthetic network and assert the geometry is continuous."""
    import tempfile, os

    # A 90-degree corner: straight east, then curving north.
    pts = []
    for i in range(5):
        pts.append({"pos": [i * 1000.0, 0.0, 0.0], "tangent": [1.0, 0.0, 0.0]})
    for i in range(1, 5):
        ang = math.radians(i * 22.5)
        pts.append({"pos": [4000.0 + math.sin(ang) * 1000.0,
                            1000.0 - math.cos(ang) * 1000.0,
                            i * 50.0],
                    "tangent": [math.cos(ang), math.sin(ang), 0.0]})

    # A crossing laid radially across the curve, mid-way between two of its
    # points - the case where a straight-line projection went metres wrong.
    phi = math.radians(33.75)
    radial = (math.sin(phi), -math.cos(phi))
    a_cm = [4000.0 + 600.0 * radial[0], 1000.0 + 600.0 * radial[1], 0.0]
    b_cm = [4000.0 + 1400.0 * radial[0], 1000.0 + 1400.0 * radial[1], 0.0]

    net = {
        "source": "selftest",
        "roads": [{"id": "r0", "points": pts, "lane_width_cm": 350.0,
                   "unidirectional": False,
                   "successor": {"type": "junction", "id": "j0"}}],
        "junctions": [{"id": "j0", "center": [5000.0, 1000.0, 0.0],
                       "connections": []}],
        "crosswalks": [{"a_cm": a_cm, "b_cm": b_cm, "path_width_cm": 100.0}],
    }

    tree = to_xodr(net, name="selftest")
    root = tree.getroot()

    failures = []

    roads = root.findall("road")
    if len(roads) != 1:
        failures.append(f"expected 1 road, got {len(roads)}")

    geos = roads[0].findall("planView/geometry")
    if len(geos) != len(pts) - 1:
        failures.append(f"expected {len(pts)-1} geometries, got {len(geos)}")

    # s must be monotonic and match the sum of lengths. Attributes are written
    # at 6 decimal places, so compare at that resolution rather than exactly -
    # the accumulator carries full precision and only the output is rounded.
    TOL = 1e-5
    s_prev, acc = -1.0, 0.0
    for g in geos:
        s, ln = float(g.get("s")), float(g.get("length"))
        if s <= s_prev:
            failures.append(f"s not monotonic at s={s}")
        if abs(s - acc) > TOL:
            failures.append(f"s discontinuity: got {s}, expected {acc}")
        s_prev, acc = s, acc + ln
    if abs(float(roads[0].get("length")) - acc) > TOL:
        failures.append("road length != sum of geometry lengths")

    # Endpoint continuity: evaluating each paramPoly3 at p=1 must land on the
    # next geometry's origin. This is the check that catches a wrong frame.
    for i, g in enumerate(geos[:-1]):
        pp = g.find("paramPoly3")
        bu, cu, du = (float(pp.get(k)) for k in ("bU", "cU", "dU"))
        bv, cv, dv = (float(pp.get(k)) for k in ("bV", "cV", "dV"))
        u, v = bu + cu + du, bv + cv + dv
        h = float(g.get("hdg"))
        x = float(g.get("x")) + u * math.cos(h) - v * math.sin(h)
        y = float(g.get("y")) + u * math.sin(h) + v * math.cos(h)
        nx, ny = float(geos[i + 1].get("x")), float(geos[i + 1].get("y"))
        if math.hypot(x - nx, y - ny) > 1e-4:
            failures.append(
                f"geometry {i} endpoint ({x:.4f},{y:.4f}) != next origin ({nx:.4f},{ny:.4f})")

    # Handedness: UE +Y (right) must become OpenDRIVE -Y.
    if ue_to_odr([100.0, 100.0, 0.0]) != (1.0, -1.0, 0.0):
        failures.append("ue_to_odr handedness/scale wrong")

    # The crossing must come back where it was, read the way CARLA reads it:
    # anchored ON the reference line (t = 0, which CARLA 0.10 needs), at an s
    # from CARLA's own table, with its two ends on the crossing's endpoints.
    cw = roads[0].find("objects/object")
    if cw is None:
        failures.append("the crossing over the curve was not emitted")
    else:
        s_cw, t_cw = float(cw.get("s")), float(cw.get("t"))
        us = [float(c.get("u")) for c in cw.findall("outline/cornerLocal")]
        road = build_road(net["roads"][0], 0)
        g = [g for g in road.geometries if g.s <= s_cw][-1]
        pl = carla_polyline(g)
        k = next(i for i in range(1, len(pl)) if pl[i][0] >= s_cw)
        f = (s_cw - pl[k - 1][0]) / (pl[k][0] - pl[k - 1][0])
        bx, by, tx, ty = (pl[k - 1][j] + f * (pl[k][j] - pl[k - 1][j]) for j in (1, 2, 3, 4))
        th = math.atan2(ty, tx) + float(cw.get("hdg"))
        ends = [(bx + u * math.cos(th), by + u * math.sin(th)) for u in (min(us), max(us))]
        want = [ue_to_odr(a_cm)[:2], ue_to_odr(b_cm)[:2]]
        err = max(math.dist(e, w) for e, w in zip(ends, want))
        if abs(t_cw) > 1e-9:
            failures.append(f"crossing written with t={t_cw}; CARLA 0.10 rotates t "
                            "by hdg, so it must sit on the reference line")
        if err > 1e-3:
            failures.append(f"crossing decodes {err:.3f} m from where it was laid")

    # Junctions must be ENTERABLE as CARLA resolves them. With junction ids
    # that were also road ids, every approach came out with no successor at all
    # - and every check above still passed.
    def one_lane(rid, x0, y0, x1, y1, **links):
        tan = [x1 - x0, y1 - y0, 0.0]
        return {"id": rid, "lane_width_cm": 350.0, "unidirectional": True,
                "lane_count": 1, "lane_offset_cm": 175.0,
                "points": [{"pos": [x0, y0, 0.0], "tangent": tan},
                           {"pos": [x1, y1, 0.0], "tangent": tan}], **links}
    to_j = {"type": "junction", "id": "J"}
    cross = to_xodr({
        "roads": [
            one_lane("A", 0, 0, 5000, 0, successor=to_j),
            one_lane("S", 5000, 0, 6000, 0, junction="J",
                     predecessor={"type": "road", "id": "A", "contact": "end"},
                     successor={"type": "road", "id": "B", "contact": "start"}),
            one_lane("T", 5000, 0, 5500, 500, junction="J",
                     predecessor={"type": "road", "id": "A", "contact": "end"},
                     successor={"type": "road", "id": "C", "contact": "start"}),
            one_lane("B", 6000, 0, 11000, 0, predecessor=to_j),
            one_lane("C", 5500, 500, 5500, 5500, predecessor=to_j),
        ],
        "junctions": [{"id": "J", "center": [5250, 0, 0], "connections": [
            {"incoming": "A", "connecting": c, "contact": "start",
             "lane_links": [[-1, -1]]} for c in ("S", "T")]}],
    }, name="crossroads").getroot()
    rid = {r.get("name"): int(r.get("id")) for r in cross.findall("road")}
    jids = {int(j.get("id")) for j in cross.findall("junction")}
    if jids & set(rid.values()):
        failures.append(f"junction ids {sorted(jids & set(rid.values()))} are also "
                        "road ids - CARLA resolves those links as roads")
    nxt = carla_successors(cross)
    if sorted(nxt.get(rid["A"], [])) != sorted([rid["S"], rid["T"]]):
        failures.append(f"approach A should continue into S and T, CARLA would "
                        f"give it {nxt.get(rid['A'])}")
    if nxt.get(rid["S"]) != [rid["B"]] or nxt.get(rid["T"]) != [rid["C"]]:
        failures.append("connecting roads must continue into their exits")

    # A pedestrian lane kept with --keep-sidewalks is a SIDEWALK. Written as
    # "driving", CARLA would route and spawn traffic on it.
    walk = to_xodr({"roads": [dict(one_lane("P", 0, 0, 5000, 0), lane_type="sidewalk"),
                              dict(one_lane("R", 0, 900, 5000, 900), lane_type="exit",
                                   is_freeway=True, road_type="motorway", speed_kmh=60.0)]},
                   name="lane-types").getroot()
    types = {r.get("name"): r.find("lanes/laneSection/right/lane").get("type")
             for r in walk.findall("road")}
    if types != {"P": "sidewalk", "R": "driving"}:
        failures.append(f"lane types written {types}; a pedestrian lane must be "
                        "'sidewalk' and a ramp must stay 'driving'")
    ramp_rd = next(r for r in walk.findall("road") if r.get("name") == "R")
    ramp_sp = ramp_rd.find("type/speed")
    if ramp_rd.find("type") is None or ramp_sp is None or ramp_sp.get("max") != "60":
        failures.append("expected <type type='motorway'><speed max='60' unit='km/h'/></type> on ramp")

    # Lanes: bidirectional road gets one left, one right, one centre.
    sec = roads[0].find("lanes/laneSection")
    for side, want in (("left", 1), ("center", 1), ("right", 1)):
        got = len(sec.findall(f"{side}/lane"))
        if got != want:
            failures.append(f"{side}: expected {want} lane(s), got {got}")
    w = sec.find("right/lane/width")
    if abs(float(w.get("a")) - 3.5) > 1e-9:
        failures.append(f"lane width should be 3.5 m, got {w.get('a')}")

    # Multi-lane unidirectional road: interior boundary is broken/both, outer curb
    # and center line are solid/none, and lane -2 resolves through a junction into
    # lane -2 of the outgoing 2-lane road.
    ml_cross = to_xodr({
        "roads": [
            {"id": "MA", "lane_width_cm": 350.0, "lane_widths_cm": [350.0, 400.0],
             "unidirectional": True, "lane_count": 2, "lane_offset_cm": 175.0,
             "points": [{"pos": [0, 0, 0], "tangent": [1, 0, 0]},
                        {"pos": [5000, 0, 0], "tangent": [1, 0, 0]}],
             "successor": to_j},
            {"id": "MC1", "lane_width_cm": 350.0, "unidirectional": True,
             "lane_count": 1, "lane_offset_cm": 175.0, "junction": "J",
             "points": [{"pos": [5000, 0, 0], "tangent": [1, 0, 0]},
                        {"pos": [6000, 0, 0], "tangent": [1, 0, 0]}],
             "predecessor": {"type": "road", "id": "MA", "contact": "end"},
             "successor": {"type": "road", "id": "MB", "contact": "start"},
             "lane_pred_ids": [-1], "lane_succ_ids": [-1]},
            {"id": "MC2", "lane_width_cm": 400.0, "unidirectional": True,
             "lane_count": 1, "lane_offset_cm": 200.0, "junction": "J",
             "points": [{"pos": [5000, 375, 0], "tangent": [1, 0, 0]},
                        {"pos": [6000, 375, 0], "tangent": [1, 0, 0]}],
             "predecessor": {"type": "road", "id": "MA", "contact": "end"},
             "successor": {"type": "road", "id": "MB", "contact": "start"},
             "lane_pred_ids": [-2], "lane_succ_ids": [-2]},
            {"id": "MB", "lane_width_cm": 350.0, "lane_widths_cm": [350.0, 400.0],
             "unidirectional": True, "lane_count": 2, "lane_offset_cm": 175.0,
             "points": [{"pos": [6000, 0, 0], "tangent": [1, 0, 0]},
                        {"pos": [11000, 0, 0], "tangent": [1, 0, 0]}],
             "predecessor": to_j},
        ],
        "junctions": [{"id": "J", "center": [5500, 0, 0], "connections": [
            {"incoming": "MA", "connecting": "MC1", "contact": "start", "lane_links": [[-1, -1]]},
            {"incoming": "MA", "connecting": "MC2", "contact": "start", "lane_links": [[-2, -1]]},
        ]}],
    }, name="ml-cross").getroot()
    ml_rid = {r.get("name"): int(r.get("id")) for r in ml_cross.findall("road")}
    ml_ls = carla_lane_successors(ml_cross)
    if ml_ls.get((ml_rid["MA"], -1)) != [(ml_rid["MC1"], -1)] or ml_ls.get((ml_rid["MA"], -2)) != [(ml_rid["MC2"], -1)]:
        failures.append(f"multi-lane junction entry failed: {ml_ls}")
    if ml_ls.get((ml_rid["MC1"], -1)) != [(ml_rid["MB"], -1)] or ml_ls.get((ml_rid["MC2"], -1)) != [(ml_rid["MB"], -2)]:
        failures.append(f"multi-lane junction exit failed: {ml_ls}")
    ma_rd = next(r for r in ml_cross.findall("road") if r.get("name") == "MA")
    c_mark = ma_rd.find("lanes/laneSection/center/lane/roadMark").get("laneChange")
    l1_mark = ma_rd.find("lanes/laneSection/right/lane[@id='-1']/roadMark").get("laneChange")
    l2_mark = ma_rd.find("lanes/laneSection/right/lane[@id='-2']/roadMark").get("laneChange")
    if (c_mark, l1_mark, l2_mark) != ("none", "both", "none"):
        failures.append(f"expected laneChange ('none', 'both', 'none'), got {(c_mark, l1_mark, l2_mark)}")

    # It must serialise and re-parse.
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "t.xodr")
        tree.write(p, encoding="utf-8", xml_declaration=True)
        try:
            ET.parse(p)
        except ET.ParseError as e:
            failures.append(f"output is not well-formed XML: {e}")

    if failures:
        print("SELFTEST FAILED")
        for f in failures:
            print("  -", f)
        return 1
    print(f"selftest passed - {len(geos)} geometries, "
          f"road length {acc:.2f} m, endpoint continuity < 1e-4 m, multi-lane links verified")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", nargs="?", help="road network JSON from stage 1")
    ap.add_argument("-o", "--output", default="CitySample.xodr")
    ap.add_argument("--name", default="CitySample")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.input:
        ap.error("input JSON required (or pass --selftest)")

    net = load_json_any(args.input)

    tree = to_xodr(net, name=args.name)
    tree.write(args.output, encoding="utf-8", xml_declaration=True)

    root = tree.getroot()
    n_r, n_j = len(root.findall("road")), len(root.findall("junction"))
    n_ml = sum(1 for r in root.findall("road")
               if len(r.findall("lanes/laneSection/right/lane[@type='driving']")) > 1)
    total = sum(float(r.get("length")) for r in root.findall("road"))
    print(f"wrote {args.output}: {n_r} roads ({n_ml} multi-lane), {n_j} junctions, {total/1000:.2f} km")
    if n_j == 0:
        print("NOTE: no junctions. CARLA will treat every road as isolated - "
              "vehicles will not turn between them and the traffic manager will "
              "strand them at road ends.", file=sys.stderr)

    # Where CARLA will find no next lane. Checked per driving lane (-1..-k), not
    # only lane -1, so any unlinked outer lane on a multi-lane road is caught too.
    by_id = {int(r.get("id")): r for r in root.findall("road")}
    driven_lanes = {
        (rid, int(l.get("id")))
        for rid, r in by_id.items()
        for l in r.findall("lanes/laneSection/right/lane[@type='driving']")
    }
    dead = [(rid, lid) for (rid, lid), nxt in carla_lane_successors(root).items()
            if not nxt and (rid, lid) in driven_lanes]
    into_junction = sum(
        1 for (rid, _) in dead
        if (by_id[rid].find("link/successor") is not None
            and by_id[rid].find("link/successor").get("elementType") == "junction"))
    print(f"  dead-end driving lanes as CARLA resolves them: {len(dead)} "
          f"({into_junction} into a junction)")
    if into_junction:
        print(f"WARNING: {into_junction} driving lanes enter a junction that offers them "
              "no connecting road - vehicles reaching them are destroyed.",
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

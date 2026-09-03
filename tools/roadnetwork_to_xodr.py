#!/usr/bin/env python3
"""
roadnetwork_to_xodr.py - convert an extracted road network to ASAM OpenDRIVE 1.4.

Stage 2 of the City Sample -> CARLA map pipeline. Stage 1
(``ue_export_roadnetwork.py``) runs inside the Unreal editor and writes the
intermediate JSON this consumes. This half is plain Python with no Unreal
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
    unidirectional: bool = False
    has_center_divider: bool = False
    is_freeway: bool = False
    lane_count: int = 1
    predecessor: dict | None = None
    successor: dict | None = None
    junction: str | None = None
    lane_offset_m: float = 0.0


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

    road = Road(
        rid=str(spec.get("id", f"road_{numeric_id}")),
        numeric_id=numeric_id,
        lane_width_m=float(spec.get("lane_width_cm", 350.0)) * CM_TO_M,
        unidirectional=bool(spec.get("unidirectional", False)),
        has_center_divider=bool(spec.get("has_center_divider", False)),
        is_freeway=bool(spec.get("is_freeway", False)),
        lane_count=max(1, int(spec.get("lane_count", 1))),
        predecessor=spec.get("predecessor"),
        successor=spec.get("successor"),
        junction=spec.get("junction"),
        lane_offset_m=float(spec.get("lane_offset_cm", 0.0)) * CM_TO_M,
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
          link_succ: int | None = None, link_pred: int | None = None):
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
    ET.SubElement(lane, "roadMark", sOffset="0.0", type="broken", weight="standard",
                  color="standard", width="0.15", laneChange="both")
    return lane


# Painted width of a crossing, along the direction of vehicle travel. The
# ZoneGraph lane is the pedestrian's 1 m path; this is the stripe it becomes.
CROSSWALK_PAINT_M = 4.0

# A crossing further than this from any reference line is not on that road.
CROSSWALK_MAX_OFFSET_M = 25.0


def emit_crosswalks(roads, crosswalks, road_elems):
    """Emit <object type="crosswalk"> polygons.

    Two things here are load-bearing:

    * The outline must be CLOSED - the first corner repeated as the last.
      carla::road::Map::GetAllCrosswalkMesh ends a triangle fan only when it
      sees crosswalk_vertex[start] == crosswalk_vertex[i]; four unrepeated
      corners produce a crosswalk-material mesh with ZERO triangles. Every
      stock town writes five for this reason.
    * The crossing is bound to a road by projecting onto its geometry, not by
      distance to the geometry's ORIGIN. Geometry segments here run to ~200 m,
      so a crossing mid-segment can be ~100 m from the origin and would be
      bound to the wrong road, or dropped entirely.
    """
    if not crosswalks:
        return 0

    # Grid-index the geometry segments so this is not len(cw) x len(geoms).
    cell = 50.0
    grid = {}
    for r in roads:
        for g in r.geometries:
            key = (int(g.x // cell), int(g.y // cell))
            grid.setdefault(key, []).append((r, g))

    def candidates(x, y):
        cx, cy = int(x // cell), int(y // cell)
        span = int(CROSSWALK_MAX_OFFSET_M // cell) + 1
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                for item in grid.get((cx + dx, cy + dy), ()):
                    yield item

    by_road = {}
    for i, cw in enumerate(crosswalks):
        ax, ay, _ = ue_to_odr(cw["a_cm"])
        bx, by, _ = ue_to_odr(cw["b_cm"])
        mx, my = (ax + bx) / 2.0, (ay + by) / 2.0
        span = math.hypot(bx - ax, by - ay)
        if span <= 0.5:
            continue
        cross_hdg = math.atan2(by - ay, bx - ax)

        best = None
        for r, g in candidates(mx, my):
            # Project the centre onto this segment's straight approximation.
            dx, dy = mx - g.x, my - g.y
            s_local = dx * math.cos(g.hdg) + dy * math.sin(g.hdg)
            s_local = max(0.0, min(g.length, s_local))
            rx = dx - s_local * math.cos(g.hdg)
            ry = dy - s_local * math.sin(g.hdg)
            resid = math.hypot(rx, ry)
            if resid > CROSSWALK_MAX_OFFSET_M:
                continue
            if best is None or resid < best[0]:
                # t is signed: left of the reference line is positive.
                t = -rx * math.sin(g.hdg) + ry * math.cos(g.hdg)
                best = (resid, r, g.s + s_local, t, g.hdg)
        if best is None:
            continue
        _, road, s_pos, t_pos, road_hdg = best
        by_road.setdefault(road.numeric_id, []).append(
            (i, s_pos, t_pos, span, cross_hdg - road_hdg))

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
        for (i, s_pos, t_pos, span, rel_hdg) in items:
            obj = ET.SubElement(
                objects, "object",
                id=f"cw{road.numeric_id}_{i}", name=f"Crosswalk_{i}",
                type="crosswalk", s=f"{max(0.0, s_pos):.6f}", t=f"{t_pos:.6f}",
                zOffset="0.0", hdg=f"{rel_hdg:.6f}", pitch="0.0", roll="0.0",
                orientation="none",
                length=f"{CROSSWALK_PAINT_M:.6f}", width=f"{span:.6f}",
                height="0.0")
            outline = ET.SubElement(obj, "outline")
            hu, hv = span / 2.0, CROSSWALK_PAINT_M / 2.0
            # CLOSED ring: first corner repeated. Four corners = zero triangles.
            ring = ((hu, hv), (hu, -hv), (-hu, -hv), (-hu, hv), (hu, hv))
            for u, v in ring:
                ET.SubElement(outline, "cornerLocal",
                              u=f"{u:.6f}", v=f"{v:.6f}", z="0.000000",
                              height="0.000000")
            n += 1
    return n


def to_xodr(net: dict, name: str = "CitySample") -> ET.ElementTree:
    roads: list[Road] = []
    id_map: dict[str, int] = {}
    road_elems: dict[int, ET.Element] = {}
    for i, spec in enumerate(net.get("roads", [])):
        r = build_road(spec, i)
        if r:
            id_map[r.rid] = r.numeric_id
            roads.append(r)

    jn_map = {str(j.get("id")): i for i, j in enumerate(net.get("junctions", []))}

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
                _lane(left, i, r.lane_width_m)

        centre = ET.SubElement(sec, "center")
        c_lane = ET.SubElement(centre, "lane", id="0", type="none", level="false")
        ET.SubElement(c_lane, "roadMark", sOffset="0.0",
                      type="solid" if r.has_center_divider else "broken",
                      weight="standard", color="standard", width="0.15",
                      laneChange="none" if r.has_center_divider else "both")

        right = ET.SubElement(sec, "right")
        # Only a road-to-road link produces a lane-level link; a link into a
        # junction is resolved by that junction's <laneLink> entries.
        succ_lane = -1 if (r.successor or {}).get("type") == "road" else None
        pred_lane = -1 if (r.predecessor or {}).get("type") == "road" else None
        for i in range(1, n_side + 1):
            _lane(right, -i, r.lane_width_m,
                  link_succ=succ_lane if i == 1 else None,
                  link_pred=pred_lane if i == 1 else None)

    n_cw = emit_crosswalks(roads, net.get("crosswalks", []), road_elems)
    if n_cw:
        print(f"  {n_cw:,} crosswalk objects")

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

    net = {
        "source": "selftest",
        "roads": [{"id": "r0", "points": pts, "lane_width_cm": 350.0,
                   "unidirectional": False,
                   "successor": {"type": "junction", "id": "j0"}}],
        "junctions": [{"id": "j0", "center": [5000.0, 1000.0, 0.0],
                       "connections": []}],
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

    # Lanes: bidirectional road gets one left, one right, one centre.
    sec = roads[0].find("lanes/laneSection")
    for side, want in (("left", 1), ("center", 1), ("right", 1)):
        got = len(sec.findall(f"{side}/lane"))
        if got != want:
            failures.append(f"{side}: expected {want} lane(s), got {got}")
    w = sec.find("right/lane/width")
    if abs(float(w.get("a")) - 3.5) > 1e-9:
        failures.append(f"lane width should be 3.5 m, got {w.get('a')}")

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
          f"road length {acc:.2f} m, endpoint continuity < 1e-4 m")
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
    total = sum(float(r.get("length")) for r in root.findall("road"))
    print(f"wrote {args.output}: {n_r} roads, {n_j} junctions, {total/1000:.2f} km")
    if n_j == 0:
        print("NOTE: no junctions. CARLA will treat every road as isolated - "
              "vehicles will not turn between them and the traffic manager will "
              "strand them at road ends.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

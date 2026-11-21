#!/usr/bin/env python3
"""
add_signals_to_xodr.py - give an extracted City Sample network real traffic lights.

    python tools/masstraffic_lights_to_json.py "<CitySample>/Content/AI/Traffic/TrafficLights/CitySampleSmallCityTrafficLights.uasset" -o lights.json
    python tools/add_signals_to_xodr.py SmallCity.xodr lights.json -o SmallCity-signals.xodr
    python tools/add_signals_to_xodr.py --selftest

The extracted network has correct geometry and topology but no signals at all,
so CARLA registers zero traffic lights and its traffic manager drives all 172
junctions uncontrolled. Epic's signal layout is not in the ZoneGraph; it lives
in a separate data asset that masstraffic_lights_to_json.py reads.

This works on a finished .xodr rather than inside the converter, so it needs no
Unreal export and no rebuild, and it can be re-run against a network you already
deployed.

WHAT CARLA ACTUALLY REQUIRES (all verified against the 0.10 source, because
several plausible-looking encodings silently produce nothing):

  * type="1000001"          SignalType::IsTrafficLight matches a fixed list.
  * explicit <validity>     MapBuilder::RemoveZeroLaneValiditySignalReferences
                            DELETES any signal reference whose validities are
                            all fromLane="0" toLane="0" - which is what most
                            stock towns write. They get away with it because
                            their lights are hand-placed actors that
                            SpawnTrafficLights adopts; a generated map has no
                            such actors, so 0/0 means no lights at all.
  * negative lane ids       UTrafficLightComponent::InitializeSign builds the
                            trigger box from the validities, and for lane < 0
                            places it BEFORE the signal - correct for the +s
                            travel direction of right-hand lanes.
  * orientation="-"         MapBuilder::GenerateDefaultValiditiesForSignalReferences
                            maps SignalOrientation::Negative onto negative
                            lanes, so "-" is what CARLA means by "governs the
                            right-hand carriageway". "negative" is not accepted:
                            RoadInfoSignal::GetOrientation compares the literal
                            "+" and "-".
  * <controller> + a
    <controller> reference  ATrafficLightGroup is keyed by JUNCTION and cycles
    inside <junction>       its controllers one at a time. So a phase == a
                            controller, and opposing approaches must share one
                            or they would go green together. The junction-side
                            reference is what JunctionParser turns into
                            AddJunctionController; without it SpawnTrafficLights
                            logs "No junctions in controller".
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

CM_TO_M = 0.01

# Stop line a little back from the junction edge, and the pole clear of the kerb.
STOPLINE_SETBACK_M = 2.0
POLE_CLEAR_M = 1.8

# Epic binds a light to an intersection side by nearest midpoint. Our road ends
# sit on those same side midpoints, so the same test works here. Generous
# enough to absorb the cm->m rounding, tight enough not to cross a junction.
MATCH_RADIUS_M = 6.0

# Parallel lanes are separate roads in this network; ends this close together
# with the same heading are the same physical approach.
SIDE_RADIUS_M = 8.0

# How far an unclaimed lane's end may be from a side's host lane and still be
# taken as part of that approach when completing a junction: a whole carriageway
# of one direction, four lanes and a median, fits inside it.
SIDE_JOIN_M = 2 * SIDE_RADIUS_M

SIGNAL_TYPE = "1000001"          # CARLA: SignalType::IsTrafficLight

# A junction with no traffic light is a STOP-SIGN junction, and that is Epic's
# own rule rather than a guess of ours. MassTraffic derives
# FMassTrafficIntersectionDetail::bHasTrafficLights from whether a light was
# found near the intersection
# (MassTrafficIntersections.cpp:397 `bHasTrafficLights |=
# (NearestTrafficLightDetailIndex != INDEX_NONE)`), and everything downstream
# branches on it - MassTrafficIntersectionSpawnDataGenerator.cpp even labels the
# other branch "4WAYSTOPSIGN". So the junctions that took no light here are the
# ones Epic drives as stop-sign intersections, and emitting a stop sign on each
# approach reproduces the behaviour rather than inventing it.
#
# CARLA already knows this type: ATrafficLightManager maps SignalType::StopSign
# to a stop-sign actor with a UStopSignComponent trigger volume, so these become
# real traffic.stop actors the client can query.
STOP_SIGN_TYPE = "206"           # CARLA: SignalType::StopSign
MOUNT_HEIGHT_M = 5.5


def odr_to_ue(x: float, y: float) -> tuple[float, float]:
    """OpenDRIVE metres -> Unreal centimetres. Inverse of the converter's
    ue_to_odr: (x*0.01, -y*0.01). The light asset is authored in UE space."""
    return (x / CM_TO_M, -y / CM_TO_M)


def geom_end(g: ET.Element) -> tuple[float, float, float]:
    """End point and heading of one <geometry>, in OpenDRIVE metres/radians."""
    x0, y0 = float(g.get("x")), float(g.get("y"))
    hdg = float(g.get("hdg"))
    pp = g.find("paramPoly3")
    if pp is None:
        L = float(g.get("length", 0.0))
        return (x0 + L * math.cos(hdg), y0 + L * math.sin(hdg), hdg)
    bu, cu, du = (float(pp.get(k)) for k in ("bU", "cU", "dU"))
    bv, cv, dv = (float(pp.get(k)) for k in ("bV", "cV", "dV"))
    u, v = bu + cu + du, bv + cv + dv                 # t = 1
    dudt, dvdt = bu + 2 * cu + 3 * du, bv + 2 * cv + 3 * dv
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    return (x0 + u * cos_h - v * sin_h,
            y0 + u * sin_h + v * cos_h,
            hdg + math.atan2(dvdt, dudt))


def geom_start(g: ET.Element) -> tuple[float, float, float]:
    return (float(g.get("x")), float(g.get("y")), float(g.get("hdg")))


def geom_at_local(g: ET.Element, ds: float) -> tuple[float, float, float]:
    """Point and heading `ds` metres along one <geometry>, matching LibCarla's
    chord-length paramPoly3 table."""
    x0, y0 = float(g.get("x")), float(g.get("y"))
    hdg = float(g.get("hdg"))
    length = max(0.0, float(g.get("length", 0.0)))
    ds = max(0.0, min(length, ds))
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    pp = g.find("paramPoly3")
    if pp is None or length <= 1e-9:
        return (x0 + ds * cos_h, y0 + ds * sin_h, hdg)
    bu, cu, du = (float(pp.get(k)) for k in ("bU", "cU", "dU"))
    bv, cv, dv = (float(pp.get(k)) for k in ("bV", "cV", "dV"))
    n = max(int(float(f"{length:.6f}") / 0.5), 5)
    s, prev = 0.0, None
    for i in range(n + 1):
        p = i / n
        cur = (((du * p + cu) * p + bu) * p,
               ((dv * p + cv) * p + bv) * p,
               (3.0 * du * p + 2.0 * cu) * p + bu,
               (3.0 * dv * p + 2.0 * cv) * p + bv)
        if prev is not None:
            seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            if s + seg >= ds or i == n:
                k = min(1.0, max(0.0, (ds - s) / seg)) if seg > 1e-12 else 0.0
                u, v, tu, tv = (a + k * (b - a) for a, b in zip(prev, cur))
                return (x0 + u * cos_h - v * sin_h,
                        y0 + u * sin_h + v * cos_h,
                        hdg + math.atan2(tv, tu))
            s += seg
        prev = cur
    return (x0, y0, hdg)


def road_point_at_s(road: ET.Element, s_target: float) -> tuple[float, float, float]:
    """Reference-line (x, y, hdg) at `s_target` along `road`."""
    geoms = road.findall("planView/geometry")
    if not geoms:
        return (0.0, 0.0, 0.0)
    chosen = geoms[0]
    for g in geoms:
        if float(g.get("s", 0.0)) <= s_target + 1e-9:
            chosen = g
        else:
            break
    return geom_at_local(chosen, s_target - float(chosen.get("s", 0.0)))


def odr_lateral_point(rx: float, ry: float, rhdg: float, t: float) -> tuple[float, float]:
    """World (x, y) in OpenDRIVE metres at lateral offset `t` (+left, -right),
    matching MapBuilder::ComputeSignalTransform + DirectedPoint::ApplyLateralOffset."""
    return (rx - t * math.sin(rhdg), ry + t * math.cos(rhdg))


class RoadSurfaceGrid:
    """Spatial index of all driving-lane carriageways in the OpenDRIVE file so a
    spawned signal or stop-sign pole is never placed on any road surface."""

    CELL_M = 15.0

    def __init__(self, root: ET.Element):
        self.grid: dict[tuple[int, int], list[tuple[float, float, float, float, float]]] = defaultdict(list)
        cell = self.CELL_M
        for road in root.findall("road"):
            ls = road.find("lanes/laneSection")
            if ls is None:
                continue
            lo_el = road.find("lanes/laneOffset")
            lo = float(lo_el.get("a")) if lo_el is not None else 0.0
            right_w = sum(
                float(l.find("width").get("a"))
                for l in ls.findall("right/lane")
                if l.get("type") == "driving" and l.find("width") is not None
            )
            left_w = sum(
                float(l.find("width").get("a"))
                for l in ls.findall("left/lane")
                if l.get("type") == "driving" and l.find("width") is not None
            )
            if right_w + left_w <= 0.0:
                continue
            t_mid = lo + 0.5 * (left_w - right_w)
            hw = 0.5 * (left_w + right_w)
            pts: list[tuple[float, float]] = []
            for g in road.findall("planView/geometry"):
                glen = max(0.0, float(g.get("length", 0.0)))
                steps = max(1, int(math.ceil(glen / 2.5)))
                start_k = 0 if not pts else 1
                for k in range(start_k, steps + 1):
                    rx, ry, rhdg = geom_at_local(g, glen * k / steps)
                    pts.append(odr_lateral_point(rx, ry, rhdg, t_mid))
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                pad = hw + 3.0
                c0x = int(math.floor((min(x0, x1) - pad) / cell))
                c1x = int(math.floor((max(x0, x1) + pad) / cell))
                c0y = int(math.floor((min(y0, y1) - pad) / cell))
                c1y = int(math.floor((max(y0, y1) + pad) / cell))
                seg = (x0, y0, x1, y1, hw)
                for cx in range(c0x, c1x + 1):
                    for cy in range(c0y, c1y + 1):
                        self.grid[(cx, cy)].append(seg)

    def min_clearance(self, px: float, py: float) -> float:
        """Signed distance outside the nearest driving lane (> 0 means clear of all lanes)."""
        cell = self.CELL_M
        best = float("inf")
        for seg in self.grid.get((int(math.floor(px / cell)), int(math.floor(py / cell))), ()):
            x0, y0, x1, y1, hw = seg
            dx, dy = x1 - x0, y1 - y0
            dd = dx * dx + dy * dy
            mu = 0.0 if dd <= 1e-12 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / dd))
            dist = math.hypot(px - (x0 + mu * dx), py - (y0 + mu * dy)) - hw
            if dist < best:
                best = dist
        return best


def pick_curbside_host(members: list[Approach], ref_hdg: float | None = None) -> Approach:
    """Choose the rightmost lane of an approach side (closest to the right kerb).

    Under the one-road-per-lane export, build_sides matches Epic's
    controlled_side_midpoint, which sits near the MIDDLE of a multi-lane
    approach. Using that middle lane as `host` placed the signal pole 3.8 m to
    its right - straight into the adjacent driving lane. Picking the rightmost
    member puts the pole outboard of the outer kerb while inner lanes keep
    trigger boxes via <signalReference>."""
    if len(members) <= 1:
        return members[0]
    hdg = members[0].hdg if ref_hdg is None else ref_hdg
    sin_h, cos_h = math.sin(hdg), math.cos(hdg)

    def signed_left_t(a: Approach) -> float:
        s_pos = max(0.0, a.length - STOPLINE_SETBACK_M)
        rx, ry, _ = road_point_at_s(a.road, s_pos)
        # Signed projection onto left normal (-sin hdg, +cos hdg): most negative is rightmost.
        return -rx * sin_h + ry * cos_h + (a.lane_offset - a.n_lanes * a.lane_width)

    return min(members, key=signed_left_t)


def clear_curb_pose(host: Approach, surface_grid: RoadSurfaceGrid | None,
                    min_margin: float = 0.8, max_extra_m: float = 24.0) -> tuple[float, float]:
    """Return `(s_pos, t)` on `host` that places the pole off every driving lane.

    Tries the standard 2 m stop-line setback first (marching rightward past any
    parallel/slip lane), then steps slightly further back from skewed corners
    where an acute cross-street flares over the 2 m mark."""
    s_default = max(0.0, host.length - STOPLINE_SETBACK_M)
    t_base = host.lane_offset - host.n_lanes * host.lane_width - POLE_CLEAR_M
    if surface_grid is None:
        return (s_default, t_base)

    steps = int(round(max_extra_m / 0.5))
    best_pose = (s_default, t_base)
    rx0, ry0, rhdg0 = road_point_at_s(host.road, s_default)
    best_clear = surface_grid.min_clearance(*odr_lateral_point(rx0, ry0, rhdg0, t_base))
    if best_clear >= min_margin:
        return best_pose

    setbacks = [STOPLINE_SETBACK_M, 4.0, 6.0, 8.0, 10.0, 14.0]
    for back in setbacks:
        if host.length > 1.0 and back > host.length - 0.5 and back > STOPLINE_SETBACK_M:
            s_cand = 0.5
        else:
            s_cand = max(0.0, host.length - back)
        rx, ry, rhdg = road_point_at_s(host.road, s_cand)
        for k in range(0, steps + 1):
            cand_t = t_base - 0.5 * k
            clr = surface_grid.min_clearance(*odr_lateral_point(rx, ry, rhdg, cand_t))
            if clr >= min_margin:
                return (s_cand, cand_t)
            if clr > best_clear:
                best_clear = clr
                best_pose = (s_cand, cand_t)

    # Last-resort fallback: median / left kerb if the entire right side is paved.
    t_left_base = host.lane_offset + POLE_CLEAR_M
    for back in setbacks:
        s_cand = max(0.0, host.length - back)
        rx, ry, rhdg = road_point_at_s(host.road, s_cand)
        for k in range(0, steps + 1):
            cand_t = t_left_base + 0.5 * k
            clr = surface_grid.min_clearance(*odr_lateral_point(rx, ry, rhdg, cand_t))
            if clr >= min_margin:
                return (s_cand, cand_t)
            if clr > best_clear:
                best_clear = clr
                best_pose = (s_cand, cand_t)
    return best_pose


class Approach:
    """One road arriving at a junction, at whichever end links to it."""

    __slots__ = ("road", "road_id", "junction", "at_end", "x", "y", "hdg",
                 "length", "n_lanes", "lane_offset", "lane_width")

    def __init__(self, road, road_id, junction, at_end, x, y, hdg,
                 length, n_lanes, lane_offset, lane_width):
        self.road = road
        self.road_id = road_id
        self.junction = junction
        self.at_end = at_end
        self.x, self.y, self.hdg = x, y, hdg
        self.length = length
        self.n_lanes = n_lanes
        self.lane_offset = lane_offset
        self.lane_width = lane_width


def collect_approaches(root: ET.Element) -> list[Approach]:
    """Every ordinary road end that meets a junction.

    Right-hand lanes travel in +s, so a vehicle LEAVES a road at its successor.
    Only that end is an approach; the predecessor end is where traffic arrives
    from the junction and must not carry a stop line.
    """
    out = []
    for road in root.findall("road"):
        if road.get("junction", "-1") != "-1":
            continue                                   # connecting road
        geoms = road.findall("planView/geometry")
        if not geoms:
            continue
        length = float(road.get("length", 0.0))

        ls = road.find("lanes/laneSection")
        if ls is None:
            continue
        right = ls.findall("right/lane")
        driving = [l for l in right if l.get("type") == "driving"]
        if not driving:
            continue
        widths = [float(l.find("width").get("a")) for l in driving
                  if l.find("width") is not None]
        lane_width = widths[0] if widths else 4.0
        lo = road.find("lanes/laneOffset")
        lane_offset = float(lo.get("a")) if lo is not None else 0.0

        succ = road.find("link/successor")
        if succ is not None and succ.get("elementType") == "junction":
            x, y, hdg = geom_end(geoms[-1])
            out.append(Approach(road, road.get("id"), succ.get("elementId"),
                                True, x, y, hdg, length, len(driving),
                                lane_offset, lane_width))
    return out


def build_sides(approaches, lights):
    """Bind Epic's authored lights onto our approach roads.

    Returns {junction_id: [side, ...]} where a side is a dict with the host
    road, its co-located parallel neighbours, and the mean heading.
    """
    # Grid index so this is not 358 x 2000 distance evaluations.
    cell = 25.0
    grid = defaultdict(list)
    for a in approaches:
        ux, uy = odr_to_ue(a.x, a.y)
        grid[(int(ux // (cell / CM_TO_M)), int(uy // (cell / CM_TO_M)))].append((ux, uy, a))

    def near(ux, uy, radius_m):
        r_cm = radius_m / CM_TO_M
        cx, cy = int(ux // (cell / CM_TO_M)), int(uy // (cell / CM_TO_M))
        found = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for (px, py, a) in grid.get((cx + dx, cy + dy), ()):
                    d = math.hypot(px - ux, py - uy)
                    if d <= r_cm:
                        found.append((d, a))
        found.sort(key=lambda t: t[0])
        return found

    by_junction = defaultdict(list)
    used = set()
    matched = 0
    for li in lights:
        mx, my, _ = li["controlled_side_midpoint"]
        hits = near(mx, my, MATCH_RADIUS_M)
        if not hits:
            continue
        matched += 1
        host = hits[0][1]
        if host.road_id in used:
            continue                                   # side already emitted
        # Never a road another side already governs. Two lights whose
        # midpoints lie within SIDE_RADIUS_M of each other would otherwise both
        # claim the lanes between them, and a lane under two signals - in two
        # different phases - stops for one while the other says go.
        members = [a for _, a in near(mx, my, SIDE_RADIUS_M)
                   if a.junction == host.junction
                   and a.road_id not in used
                   and abs(_ang_diff(a.hdg, host.hdg)) < math.radians(25)]
        members = members or [host]
        for m in members:
            used.add(m.road_id)
        curb_host = pick_curbside_host(members, host.hdg)
        by_junction[host.junction].append({
            "host": curb_host,
            "members": members,
            "hdg": host.hdg,
            "type_index": li["type_index"],
        })
    return by_junction, matched


def complete_sides(approaches, by_junction):
    """Give every approach to a signalised junction a light.

    Lights bind to approaches near one of Epic's authored lights, so a lane can
    be left over at a junction that does have lights: the outer lane of a wide
    approach, beyond SIDE_RADIUS_M of the light's midpoint, or a whole side
    Epic gave no pole of its own (4 of Small City's signalised junctions came
    out with a single light). Such a lane got neither a light nor a stop sign,
    and TrafficLightStage routes vehicles on it through as if the junction were
    unsignalised - into crossing traffic that has green. MassTraffic itself
    runs the whole intersection on its light cycle once any side has a light
    (bHasTrafficLights is per intersection).

    A leftover lane joins an existing side that runs the same way, as a
    reference to that side's light; otherwise it starts a side of its own,
    which phase_of then puts in a phase by heading like any other. Returns
    (lanes joined to an existing side, sides added).
    """
    claimed = {m.road_id for sides in by_junction.values()
               for side in sides for m in side["members"]}
    leftover = defaultdict(list)
    for a in approaches:
        if a.junction in by_junction and a.road_id not in claimed:
            leftover[a.junction].append(a)

    joined = added = 0
    same_way = math.radians(25)
    for jid, lanes in leftover.items():
        sides = by_junction[jid]
        for a in sorted(lanes, key=lambda a: int(a.road_id)):
            if a.road_id in claimed:
                continue
            near = [(math.hypot(a.x - s["host"].x, a.y - s["host"].y), i)
                    for i, s in enumerate(sides)
                    if abs(_ang_diff(a.hdg, s["host"].hdg)) < same_way]
            near = [(d, i) for d, i in near if d <= SIDE_JOIN_M]
            if near:
                side = sides[min(near)[1]]
                side["members"].append(a)
                side["host"] = pick_curbside_host(side["members"], side["hdg"])
                claimed.add(a.road_id)
                joined += 1
                continue
            members = [b for b in lanes if b.road_id not in claimed
                       and abs(_ang_diff(b.hdg, a.hdg)) < same_way
                       and math.hypot(b.x - a.x, b.y - a.y) <= SIDE_RADIUS_M]
            claimed.update(m.road_id for m in members)
            sides.append({"host": pick_curbside_host(members, a.hdg),
                          "members": members, "hdg": a.hdg,
                          "type_index": None})
            added += 1
    return joined, added


def _ang_diff(a: float, b: float) -> float:
    d = (a - b) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


def phase_of(sides) -> dict:
    """Split a junction's approaches into two phases so opposing arms run
    together. Bearings are taken modulo 180 degrees: an approach and the one
    facing it across the junction differ by ~180 and belong to one phase."""
    if not sides:
        return {}
    base = sides[0]["hdg"]
    out = {}
    for i, s in enumerate(sides):
        d = abs(_ang_diff(s["hdg"], base)) % math.pi
        # Within 45 deg of the reference axis (or of its opposite) -> phase 0.
        out[i] = 0 if (d < math.radians(45) or d > math.radians(135)) else 1
    return out


def group_approach_sides(approaches_for_junction: list[Approach]) -> list[dict]:
    """Cluster co-located parallel approach lanes of one junction into sides,
    with each side's `host` set to the rightmost (kerbside) lane."""
    same_way = math.radians(25)
    sides: list[dict] = []
    claimed: set[str] = set()
    for a in sorted(approaches_for_junction, key=lambda x: int(x.road_id)):
        if a.road_id in claimed:
            continue
        near = [(math.hypot(a.x - s["host"].x, a.y - s["host"].y), i)
                for i, s in enumerate(sides)
                if abs(_ang_diff(a.hdg, s["hdg"])) < same_way]
        near = [(d, i) for d, i in near if d <= SIDE_JOIN_M]
        if near:
            side = sides[min(near)[1]]
            side["members"].append(a)
            side["host"] = pick_curbside_host(side["members"], side["hdg"])
            claimed.add(a.road_id)
            continue
        members = [b for b in approaches_for_junction if b.road_id not in claimed
                   and abs(_ang_diff(b.hdg, a.hdg)) < same_way
                   and math.hypot(b.x - a.x, b.y - a.y) <= SIDE_RADIUS_M]
        claimed.update(m.road_id for m in members)
        sides.append({
            "host": pick_curbside_host(members, a.hdg),
            "members": members,
            "hdg": a.hdg,
        })
    return sides


def emit_stop_signs(root: ET.Element, approaches, signalised_junctions,
                    surface_grid: RoadSurfaceGrid | None = None):
    """One kerbside stop sign per approach SIDE at unsignalised junctions, with
    <signalReference> on sibling parallel lanes so inner lanes get trigger boxes
    without spawning stop-sign poles in the middle of the street."""
    roads_by_id = {r.get("id"): r for r in root.findall("road")}
    by_junc: dict[str, list[Approach]] = defaultdict(list)
    for a in approaches:
        if a.junction not in signalised_junctions:
            by_junc[a.junction].append(a)

    n = n_ref = 0
    for jid, j_approaches in sorted(by_junc.items(), key=lambda kv: int(kv[0])):
        for side in group_approach_sides(j_approaches):
            host = side["host"]
            sid = f"stop{jid}_{host.road_id}"
            s_pos, t = clear_curb_pose(host, surface_grid)

            signals = host.road.find("signals")
            if signals is None:
                signals = ET.SubElement(host.road, "signals")
            sig = ET.SubElement(
                signals, "signal",
                s=f"{s_pos:.6f}", t=f"{t:.6f}", id=sid,
                name=f"StopSign_{jid}_{host.road_id}",
                dynamic="no", orientation="-",
                zOffset=f"{MOUNT_HEIGHT_M:.6f}", country="OpenDRIVE",
                type=STOP_SIGN_TYPE, subtype="-1", value="-1.0",
                unit="", height="0.8", width="0.8", text="",
                hOffset="0.0", pitch="0.0", roll="0.0")
            ET.SubElement(sig, "validity",
                          fromLane=f"-{host.n_lanes}", toLane="-1")
            n += 1

            for m in side["members"]:
                if m.road_id == host.road_id:
                    continue
                mroad = roads_by_id.get(m.road_id)
                if mroad is None:
                    continue
                msig = mroad.find("signals")
                if msig is None:
                    msig = ET.SubElement(mroad, "signals")
                mt = m.lane_offset - m.n_lanes * m.lane_width - POLE_CLEAR_M
                ref = ET.SubElement(
                    msig, "signalReference",
                    s=f"{max(0.0, m.length - STOPLINE_SETBACK_M):.6f}",
                    t=f"{mt:.6f}", id=sid, orientation="-")
                ET.SubElement(ref, "validity",
                              fromLane=f"-{m.n_lanes}", toLane="-1")
                n_ref += 1
    return n, n_ref, len(by_junc)


def emit(root: ET.Element, by_junction, verbose=False,
         surface_grid: RoadSurfaceGrid | None = None):
    roads_by_id = {r.get("id"): r for r in root.findall("road")}
    junctions_by_id = {j.get("id"): j for j in root.findall("junction")}

    n_sig = n_ref = n_ctl = 0
    for jid, sides in sorted(by_junction.items(), key=lambda kv: int(kv[0])):
        phases = phase_of(sides)
        per_phase = defaultdict(list)

        for si, side in enumerate(sides):
            host = pick_curbside_host(side["members"], side["hdg"])
            side["host"] = host
            sid = f"s{jid}_{si}"
            n_side = host.n_lanes
            # Pole outboard of the rightmost lane, pushed clear of any slip lane
            # or opposing carriageway in the map.
            s_pos, t = clear_curb_pose(host, surface_grid)

            signals = host.road.find("signals")
            if signals is None:
                signals = ET.SubElement(host.road, "signals")
            sig = ET.SubElement(
                signals, "signal",
                s=f"{s_pos:.6f}", t=f"{t:.6f}", id=sid,
                name=f"TrafficLight_{jid}_{si}",
                dynamic="yes", orientation="-",
                zOffset=f"{MOUNT_HEIGHT_M:.6f}", country="OpenDRIVE",
                type=SIGNAL_TYPE, subtype="-1", value="-1.0",
                unit="", height="1.2", width="0.4", text="",
                hOffset="0.0", pitch="0.0", roll="0.0")
            # NEVER 0/0 - that is silently deleted. NEVER omitted either: the
            # default generator only runs for an empty set and the trigger box
            # would then be built from nothing.
            ET.SubElement(sig, "validity",
                          fromLane=f"-{n_side}", toLane="-1")
            n_sig += 1
            per_phase[phases[si]].append(sid)

            # Every other lane of the same approach needs its own reference, or
            # InitializeSign builds no trigger box for it and TrafficLightStage
            # routes those vehicles through as if the junction were unsignalised.
            for m in side["members"]:
                if m.road_id == host.road_id:
                    continue
                mroad = roads_by_id.get(m.road_id)
                if mroad is None:
                    continue
                msig = mroad.find("signals")
                if msig is None:
                    msig = ET.SubElement(mroad, "signals")
                mt = m.lane_offset - m.n_lanes * m.lane_width - POLE_CLEAR_M
                ref = ET.SubElement(
                    msig, "signalReference",
                    s=f"{max(0.0, m.length - STOPLINE_SETBACK_M):.6f}",
                    t=f"{mt:.6f}", id=sid, orientation="-")
                ET.SubElement(ref, "validity",
                              fromLane=f"-{m.n_lanes}", toLane="-1")
                n_ref += 1

        junction = junctions_by_id.get(jid)
        for ph, sids in sorted(per_phase.items()):
            cid = f"ct{jid}_{ph}"
            ctl = ET.SubElement(root, "controller",
                                id=cid, name=f"Controller_{jid}_{ph}",
                                sequence=str(ph))
            for sid in sids:
                ET.SubElement(ctl, "control", signalId=sid, type="0")
            n_ctl += 1
            # JunctionParser -> AddJunctionController. Without this the
            # controller has no junction and SpawnTrafficLights logs an error.
            if junction is not None:
                ET.SubElement(junction, "controller", id=cid, type="0")

    return n_sig, n_ref, n_ctl


def indent(elem, level=0):
    pad = "\n" + "    " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "    "
        for child in elem:
            indent(child, level + 1)
        if not (elem.tail or "").strip():
            elem.tail = pad
        if not (child.tail or "").strip():
            child.tail = pad
    elif level and not (elem.tail or "").strip():
        elem.tail = pad


def read_root(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            return ET.fromstring(raw.decode(enc))
        except (UnicodeDecodeError, ET.ParseError):
            continue
    raise ValueError(f"cannot parse {path}")


def selftest() -> int:
    """A two-arm crossroads, built by hand, run end to end."""
    fails = []
    xodr = """<OpenDRIVE>
      <header revMajor="1" revMinor="4" name="t"/>
      <road name="north" length="50.0" id="1" junction="-1">
        <link><successor elementType="junction" elementId="9"/></link>
        <planView><geometry s="0.0" x="0.0" y="-50.0" hdg="1.5707963" length="50.0">
          <paramPoly3 aU="0.0" bU="50.0" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="normalized"/>
        </geometry></planView>
        <lanes><laneOffset s="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
          <laneSection s="0.0"><right><lane id="-1" type="driving" level="false">
          <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/></lane></right></laneSection>
        </lanes>
      </road>
      <road name="east" length="50.0" id="2" junction="-1">
        <link><successor elementType="junction" elementId="9"/></link>
        <planView><geometry s="0.0" x="-50.0" y="0.0" hdg="0.0" length="50.0">
          <paramPoly3 aU="0.0" bU="50.0" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="normalized"/>
        </geometry></planView>
        <lanes><laneOffset s="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
          <laneSection s="0.0"><right><lane id="-1" type="driving" level="false">
          <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/></lane></right></laneSection>
        </lanes>
      </road>
      <junction id="9" name="j9"/>
    </OpenDRIVE>"""
    root = ET.fromstring(xodr)
    aps = collect_approaches(root)
    if len(aps) != 2:
        fails.append(f"expected 2 approaches, got {len(aps)}")

    # Both roads end at the origin; lights sit on those side midpoints in UE cm.
    lights = [
        {"controlled_side_midpoint": list(odr_to_ue(0.0, 0.0)) + [0.0],
         "type_index": 1, "position": [0, 0, 0], "z_rotation": 0.0},
    ]
    by_j, matched = build_sides(aps, lights)
    if matched != 1:
        fails.append(f"expected 1 matched light, got {matched}")

    n_sig, n_ref, n_ctl = emit(root, by_j)
    if n_sig != 1:
        fails.append(f"expected 1 signal, got {n_sig}")
    if n_ctl != 1:
        fails.append(f"expected 1 controller, got {n_ctl}")

    sig = root.find(".//signal")
    if sig is None:
        fails.append("no <signal> emitted")
    else:
        if sig.get("type") != SIGNAL_TYPE:
            fails.append(f"type is {sig.get('type')}, not {SIGNAL_TYPE}")
        if sig.get("orientation") != "-":
            fails.append(f"orientation is {sig.get('orientation')!r}, not '-'")
        v = sig.find("validity")
        if v is None:
            fails.append("signal has no <validity> - CARLA would build no trigger box")
        elif (v.get("fromLane"), v.get("toLane")) == ("0", "0"):
            fails.append("validity is 0/0 - RemoveZeroLaneValiditySignalReferences deletes it")
        elif v.get("toLane") != "-1":
            fails.append(f"validity toLane is {v.get('toLane')}, expected -1")
        # t must clear the carriageway: 2.0 - 1*4.0 - 1.8 = -3.8
        if abs(float(sig.get("t")) + 3.8) > 1e-6:
            fails.append(f"t is {sig.get('t')}, expected -3.8")
        if abs(float(sig.get("s")) - 48.0) > 1e-6:
            fails.append(f"s is {sig.get('s')}, expected 48.0")

    if root.find("junction/controller") is None:
        fails.append("junction has no <controller> reference - controller would have no junction")

    def approach(rid, x_end, y_end, hdg, length=50.0):
        """A one-lane road ENDING at (x_end, y_end) on junction 9."""
        x0, y0 = x_end - length * math.cos(hdg), y_end - length * math.sin(hdg)
        return f"""<road name="r{rid}" length="{length}" id="{rid}" junction="-1">
          <link><successor elementType="junction" elementId="9"/></link>
          <planView><geometry s="0.0" x="{x0}" y="{y0}" hdg="{hdg}" length="{length}">
            <paramPoly3 aU="0.0" bU="{length}" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="normalized"/>
          </geometry></planView>
          <lanes><laneOffset s="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
            <laneSection s="0.0"><right><lane id="-1" type="driving" level="false">
            <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/></lane></right></laneSection>
          </lanes>
        </road>"""

    def light_at(x, y):
        return {"controlled_side_midpoint": list(odr_to_ue(x, y)) + [0.0],
                "type_index": 0, "position": [0, 0, 0], "z_rotation": 0.0}

    def run(roads_xml, lights_, complete=True, stops=False):
        r = ET.fromstring(f"<OpenDRIVE>{roads_xml}<junction id=\"9\" name=\"j9\"/></OpenDRIVE>")
        grid = RoadSurfaceGrid(r)
        aps_ = collect_approaches(r)
        bj, _ = build_sides(aps_, lights_)
        if complete:
            complete_sides(aps_, bj)
        emit(r, bj, surface_grid=grid)
        if stops:
            emit_stop_signs(r, aps_, set(bj), surface_grid=grid)
        return r, bj, grid

    def governed(r):
        return {road.get("id") for road in r.findall("road")
                if road.find("signals/signal") is not None
                or road.find("signals/signalReference") is not None}

    # Perpendicular arms, one light each: two sides, and they must not share a
    # phase - opposing traffic would get green together.
    r2, bj2, _ = run(approach(1, 2.0, -8.0, math.pi / 2) + approach(2, -8.0, -2.0, 0.0),
                     [light_at(2.0, -8.0), light_at(-8.0, -2.0)])
    ph = phase_of(bj2.get("9", []))
    if len(bj2.get("9", [])) != 2 or len(set(ph.values())) != 2:
        fails.append(f"perpendicular arms should be two sides in two phases, "
                     f"got {len(bj2.get('9', []))} side(s), phases {ph}")
    if len(r2.findall("controller")) != 2:
        fails.append(f"expected 2 controllers, got {len(r2.findall('controller'))}")

    # Three parallel lanes of one approach, with lights near either end of it.
    # The middle lane is within reach of both; it must answer to ONE of them.
    r3, _, _ = run("".join(approach(i, 3.5 * i, 0.0, math.pi / 2) for i in (1, 2, 3)),
                   [light_at(1.5, 0.0), light_at(13.0, 0.0)])
    for road in r3.findall("road"):
        n = len(road.findall("signals/signal")) + len(road.findall("signals/signalReference"))
        if n > 1:
            fails.append(f"road {road.get('id')} is governed by {n} signals")

    # A signalised junction where Epic authored a light for only ONE of its two
    # arms. The other arm used to get nothing - no light and no stop sign - and
    # its traffic crossed whatever the light showed.
    arms = approach(1, 2.0, -8.0, math.pi / 2) + approach(2, -8.0, -2.0, 0.0)
    r4, bj4, _ = run(arms, [light_at(2.0, -8.0)])
    if governed(r4) != {"1", "2"}:
        fails.append(f"only roads {sorted(governed(r4))} are governed; every "
                     "approach to a signalised junction needs a light")
    elif len(set(phase_of(bj4["9"]).values())) != 2:
        fails.append("the arm given a light must run in its own phase")
    r4_old, _, _ = run(arms, [light_at(2.0, -8.0)], complete=False)
    if governed(r4_old) != {"1"}:
        fails.append("fixture no longer reproduces the uncontrolled arm")

    # A three-lane approach whose outer lane ends beyond SIDE_RADIUS_M of the
    # light: it belongs to the same light, as a reference - not a second light.
    # AND the <signal> pole must sit on the RIGHTMOST lane (road 3 at x=10.5),
    # clear of all driving lanes - never in the middle of road 2!
    r5, _, g5 = run("".join(approach(i, 3.5 * i, 0.0, math.pi / 2) for i in (1, 2, 3)),
                    [light_at(1.0, 0.0)])
    n_sig = len(r5.findall("road/signals/signal"))
    n_ref = len(r5.findall("road/signals/signalReference"))
    if (n_sig, n_ref, len(governed(r5))) != (1, 2, 3):
        fails.append(f"wide approach: {n_sig} signals, {n_ref} references, "
                     f"{len(governed(r5))} lanes governed - expected 1, 2, 3")
    sig_roads = [rd.get("id") for rd in r5.findall("road") if rd.find("signals/signal") is not None]
    if sig_roads != ["3"]:
        fails.append(f"multi-lane traffic light pole should sit on rightmost road 3, got {sig_roads}")
    for rd in r5.findall("road"):
        for sg in rd.findall("signals/signal"):
            rx, ry, rhdg = road_point_at_s(rd, float(sg.get("s")))
            px, py = odr_lateral_point(rx, ry, rhdg, float(sg.get("t")))
            if g5.min_clearance(px, py) < 0.5:
                fails.append(f"signal {sg.get('id')} overlaps a driving lane (clearance {g5.min_clearance(px, py):.2f} m)")

    # Unsignalised 3-lane approach: ONE kerbside stop sign on rightmost road 3 +
    # 2 <signalReference>s on inner roads 1 & 2, clear of every driving lane.
    r6, _, g6 = run("".join(approach(i, 3.5 * i, 0.0, math.pi / 2) for i in (1, 2, 3)),
                    [], stops=True)
    stop_sigs = [rd.get("id") for rd in r6.findall("road") if rd.find("signals/signal") is not None]
    stop_refs = len(r6.findall("road/signals/signalReference"))
    if stop_sigs != ["3"] or stop_refs != 2 or len(governed(r6)) != 3:
        fails.append(f"multi-lane stop sign: expected pole on ['3'] + 2 refs, got {stop_sigs} + {stop_refs} refs")
    for rd in r6.findall("road"):
        for sg in rd.findall("signals/signal"):
            rx, ry, rhdg = road_point_at_s(rd, float(sg.get("s")))
            px, py = odr_lateral_point(rx, ry, rhdg, float(sg.get("t")))
            if g6.min_clearance(px, py) < 0.5:
                fails.append(f"stop sign {sg.get('id')} overlaps a driving lane")

    if fails:
        print("SELFTEST FAILED")
        for f in fails:
            print("  -", f)
        return 1
    print("selftest passed - crossroads signal, validity -1..-1, controller "
          "bound to its junction")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("lights", nargs="?")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--no-stop-signs", action="store_true",
                    help="do not emit stop signs at the junctions that have no "
                         "traffic light. They are Epic's stop-sign "
                         "intersections (MassTraffic's bHasTrafficLights is "
                         "false for exactly these), so they are on by default")
    ap.add_argument("--no-complete-junctions", action="store_true",
                    help="leave approaches to a signalised junction that no "
                         "authored light binds to WITHOUT any control, as "
                         "before. By default they join a side or get a light "
                         "of their own - see complete_sides")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.xodr or not args.lights:
        ap.error("need an .xodr and a lights .json (or --selftest)")

    root = read_root(args.xodr)
    with open(args.lights, encoding="utf-8") as fh:
        lights = json.load(fh)["lights"]

    existing = len(root.findall(".//signal"))
    if existing:
        print(f"note: input already has {existing} signals; adding to them",
              file=sys.stderr)

    surface_grid = RoadSurfaceGrid(root)
    approaches = collect_approaches(root)
    by_junction, matched = build_sides(approaches, lights)
    joined, added = (0, 0) if args.no_complete_junctions else complete_sides(
        approaches, by_junction)
    n_sig, n_ref, n_ctl = emit(root, by_junction, surface_grid=surface_grid)
    n_stop, n_stop_ref, n_stop_junc = (
        (0, 0, 0)
        if args.no_stop_signs
        else emit_stop_signs(root, approaches, set(by_junction), surface_grid=surface_grid)
    )

    # Check how many emitted <signal> poles still sit inside any driving lane.
    in_lane = 0
    for rd in root.findall("road"):
        for sg in rd.findall("signals/signal"):
            rx, ry, rhdg = road_point_at_s(rd, float(sg.get("s")))
            px, py = odr_lateral_point(rx, ry, rhdg, float(sg.get("t")))
            if surface_grid.min_clearance(px, py) < 0.0:
                in_lane += 1

    out = args.output or args.xodr.rsplit(".", 1)[0] + "-signals.xodr"
    indent(root)
    ET.ElementTree(root).write(out, encoding="utf-8", xml_declaration=True)

    n_junc = len(root.findall("junction"))
    print(f"wrote {out}")
    print(f"  approaches found      : {len(approaches):,}")
    print(f"  lights matched        : {matched:,} / {len(lights):,}")
    print(f"  signals emitted       : {n_sig:,}")
    print(f"  lane references       : {n_ref:,}")
    print(f"  controllers (phases)  : {n_ctl:,}")
    print(f"  junctions signalised  : {len(by_junction):,} / {n_junc:,}")
    print(f"  completed junctions   : {joined:,} lanes joined an authored side, "
          f"{added:,} sides given a light Epic did not author")
    governed = {r.get("id") for r in root.findall("road")
                if r.find("signals/signal") is not None
                or r.find("signals/signalReference") is not None}
    bare = sum(1 for a in approaches
               if a.junction in by_junction and a.road_id not in governed)
    print(f"  uncontrolled approaches at signalised junctions: {bare:,}")
    print(f"  stop signs emitted    : {n_stop:,} (+ {n_stop_ref:,} lane refs) "
          f"across {n_stop_junc:,} unsignalised junctions")
    print(f"  poles inside lanes    : {in_lane:,}")
    if not n_sig:
        print("  NO SIGNALS EMITTED - check the lights file matches this map",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

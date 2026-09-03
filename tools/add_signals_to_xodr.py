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

SIGNAL_TYPE = "1000001"          # CARLA: SignalType::IsTrafficLight
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
        members = [a for _, a in near(mx, my, SIDE_RADIUS_M)
                   if a.junction == host.junction
                   and abs(_ang_diff(a.hdg, host.hdg)) < math.radians(25)]
        for m in members:
            used.add(m.road_id)
        by_junction[host.junction].append({
            "host": host,
            "members": members or [host],
            "hdg": host.hdg,
            "type_index": li["type_index"],
        })
    return by_junction, matched


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


def emit(root: ET.Element, by_junction, verbose=False):
    roads_by_id = {r.get("id"): r for r in root.findall("road")}
    junctions_by_id = {j.get("id"): j for j in root.findall("junction")}

    n_sig = n_ref = n_ctl = 0
    for jid, sides in sorted(by_junction.items(), key=lambda kv: int(kv[0])):
        phases = phase_of(sides)
        per_phase = defaultdict(list)

        for si, side in enumerate(sides):
            host = side["host"]
            sid = f"s{jid}_{si}"
            n_side = host.n_lanes
            # Pole outboard of the rightmost lane. lane_offset shifts the
            # reference line, so subtract the full carriageway then clear the kerb.
            t = host.lane_offset - n_side * host.lane_width - POLE_CLEAR_M
            s_pos = max(0.0, host.length - STOPLINE_SETBACK_M)

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

    # Perpendicular arms must land in different phases.
    ph = phase_of(by_j["9"]) if "9" in by_j else {}
    if len(by_j.get("9", [])) == 2 and len(set(ph.values())) != 2:
        fails.append(f"perpendicular approaches share a phase: {ph}")

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

    approaches = collect_approaches(root)
    by_junction, matched = build_sides(approaches, lights)
    n_sig, n_ref, n_ctl = emit(root, by_junction)

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
    if not n_sig:
        print("  NO SIGNALS EMITTED - check the lights file matches this map",
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

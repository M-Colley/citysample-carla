#!/usr/bin/env python3
"""
zonegraph_to_roadnetwork.py - turn the raw ZoneGraph dump into the road network
that roadnetwork_to_xodr.py consumes.

Pipeline position:

    ExportZoneGraph commandlet  ->  zonegraph.json      (raw lanes + links + tags)
    THIS SCRIPT                 ->  roadnetwork.json    (roads + junctions)
    roadnetwork_to_xodr.py      ->  CitySample.xodr     (ASAM OpenDRIVE 1.4)

    python tools/zonegraph_to_roadnetwork.py zonegraph.json -o roadnetwork.json
    python tools/zonegraph_to_roadnetwork.py --selftest

Why the split: ZoneGraph is a LANE graph, OpenDRIVE wants ROADS carrying lane
sets. Reassembling roads is a judgement call that will need tuning against real
city data, and doing it in Python means iterating without a C++ rebuild.

The City Sample tags every lane, and those tags carry the semantics (read from
Config/DefaultPlugins.ini, [/Script/ZoneGraph.ZoneGraphSettings]):

    Vehicle          -> OpenDRIVE lane type "driving"
    Pedestrian       -> "sidewalk"
    Crosswalk        -> pedestrian crossing
    Intersection     -> the lane belongs to a junction, not a road
    Freeway          -> motorway; with Trunk Road for the trunk lanes
    Freeway Onramp   -> "entry"
    Freeway Offramp  -> "exit"

Grouping rule: two lanes belong to the same road iff they are joined by an
Adjacent link. Outgoing/Incoming links are longitudinal (they connect one road
to the next) and must NOT merge roads - that distinction is the whole point.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections import defaultdict

# Tag -> OpenDRIVE lane type. Order matters: first match wins.

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

LANE_TYPE_BY_TAG = [
    ("Crosswalk", "sidewalk"),
    ("Pedestrian", "sidewalk"),
    ("Freeway Onramp", "entry"),
    ("Freeway Offramp", "exit"),
    ("Vehicle", "driving"),
]


def lane_type(tags: list[str]) -> str:
    for tag, odr in LANE_TYPE_BY_TAG:
        if tag in tags:
            return odr
    return "driving"


class UnionFind:
    def __init__(self, n: int):
        self.p = list(range(n))

    def find(self, a: int) -> int:
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a: int, b: int) -> None:
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[rb] = ra


def group_lanes(lanes: list[dict]) -> list[list[int]]:
    """Adjacent links define same-road membership. Longitudinal links do not."""
    idx_of = {ln["index"]: i for i, ln in enumerate(lanes)}
    uf = UnionFind(len(lanes))
    for i, ln in enumerate(lanes):
        for lk in ln.get("links", []):
            if lk.get("type") != "Adjacent":
                continue
            j = idx_of.get(lk.get("dest"))
            if j is not None:
                uf.union(i, j)
    groups: dict[int, list[int]] = defaultdict(list)
    for i in range(len(lanes)):
        groups[uf.find(i)].append(i)
    return list(groups.values())


def polyline_length(points: list[dict]) -> float:
    total = 0.0
    for a, b in zip(points, points[1:]):
        pa, pb = a["pos"], b["pos"]
        total += math.dist(pa, pb)
    return total


def extract_crosswalks(lanes: list) -> list:
    """Crosswalk-tagged lanes, as centre segments in UNREAL centimetres.

    A Crosswalk lane in the City Sample ZoneGraph is the pedestrian's path
    ACROSS the carriageway: exactly two points, 1 m wide, 14-34 m long. That is
    the crossing centreline, so the OpenDRIVE object is built by giving it a
    painted width perpendicular to it - see roadnetwork_to_xodr.emit_crosswalks.

    Kept in UE cm like everything else here; the single cm->m and y-negation
    conversion happens in one place downstream.
    """
    out = []
    for lane in lanes:
        tags = lane.get("tags", [])
        if "Crosswalk" not in tags:
            continue
        pts = lane.get("points", [])
        if len(pts) < 2:
            continue
        a, b = pts[0]["pos"], pts[-1]["pos"]
        out.append({
            "a_cm": [a[0], a[1], a[2]],
            "b_cm": [b[0], b[1], b[2]],
            "path_width_cm": lane.get("width_cm", 100.0),
        })
    return out


def convert(doc: dict, *, keep_sidewalks: bool = False) -> dict:
    """One OpenDRIVE road per ZoneGraph lane.

    Earlier this emitted one road per lane GROUP, using a representative lane's
    centreline as the reference line and hanging left/right lanes off it. That
    road then spanned across its neighbours' territory, so adjacent groups'
    generated meshes overlapped - and a vehicle spawned into two overlapping
    meshes is physically stuck. Roughly half of all spawn points were unusable.

    ZoneGraph lanes do not overlap each other, so emitting one road per lane
    tiles the surface exactly. It also makes longitudinal links trivial and
    exact: the lane -> road mapping is 1:1, so an Outgoing link resolves
    directly to a successor road.

    The cost is that parallel lanes are no longer one multi-lane road, so CARLA
    sees no lane-change relationship between them. Correct geometry first.
    """
    roads: list[dict] = []
    junctions: list[dict] = []
    crosswalks: list[dict] = []
    stats = defaultdict(int)

    for graph in doc.get("zone_graphs", []):
        lanes = graph.get("lanes", [])
        if not lanes:
            continue
        prefix = graph.get("actor", "zg")
        crosswalks.extend(extract_crosswalks(lanes))
        by_index = {ln["index"]: ln for ln in lanes}

        def drivable(ln: dict) -> bool:
            if keep_sidewalks:
                return True
            return lane_type(ln.get("tags", [])) != "sidewalk"

        # Groups are still needed, but only to identify junctions: an
        # Adjacent-linked cluster containing an Intersection lane is one junction.
        lane_to_junction: dict[int, str] = {}
        for gi, group in enumerate(group_lanes(lanes)):
            members = [lanes[i] for i in group]
            if any("Intersection" in m.get("tags", []) for m in members):
                jid = f"{prefix}_j{gi}"
                for m in members:
                    lane_to_junction[m["index"]] = jid

        # A lane becomes a road iff it is drivable and has real geometry.
        emitted: set[int] = set()
        for ln in lanes:
            if not drivable(ln):
                stats["skipped_sidewalk_lanes"] += 1
                continue
            if len(ln.get("points", [])) < 2:
                stats["skipped_short"] += 1
                continue
            emitted.add(ln["index"])

        def road_id(idx: int) -> str | None:
            return f"{prefix}_l{idx}" if idx in emitted else None

        used_junctions: set[str] = set()
        for ln in lanes:
            idx = ln["index"]
            if idx not in emitted:
                continue
            tags = ln.get("tags", [])
            jid = lane_to_junction.get(idx)

            # Longitudinal links. OpenDRIVE allows one predecessor and one
            # successor per road; where a lane has several (a fork), prefer the
            # junction, which is what the fork actually is.
            succ = pred = None
            for lk in ln.get("links", []):
                t = lk.get("type")
                if t not in ("Outgoing", "Incoming"):
                    continue
                dest = lk.get("dest")
                dj = lane_to_junction.get(dest)
                dr = road_id(dest)
                if dj and dj != jid:
                    cand = {"type": "junction", "id": dj}
                elif dr:
                    # Arriving at the successor's start / leaving from the
                    # predecessor's end.
                    cand = {"type": "road", "id": dr,
                            "contact": "start" if t == "Outgoing" else "end"}
                else:
                    stats["unresolved_links"] += 1
                    continue
                if t == "Outgoing" and succ is None:
                    succ = cand
                elif t == "Incoming" and pred is None:
                    pred = cand
            if succ is None and pred is None:
                stats["roads_without_links"] += 1

            width = float(ln.get("width_cm", 350.0))
            roads.append({
                "id": f"{prefix}_l{idx}",
                "points": [{"pos": p["pos"], "tangent": p.get("tangent", [0, 0, 0])}
                           for p in ln["points"]],
                "lane_width_cm": width,
                # One directed lane, emitted on the right. laneOffset then puts
                # that lane's CENTRE on the ZoneGraph centreline instead of half
                # a lane to the side of it.
                "unidirectional": True,
                "lane_count": 1,
                "lane_offset_cm": width / 2.0,
                "is_freeway": "Freeway" in tags,
                "lane_type": lane_type(tags),
                "tags": tags,
                **({"junction": jid} if jid else {}),
                **({"successor": succ} if succ else {}),
                **({"predecessor": pred} if pred else {}),
            })
            stats["connecting_roads" if jid else "roads"] += 1
            if jid:
                used_junctions.add(jid)

        # Junction records, with connections resolved lane-by-lane.
        for jid in sorted(used_junctions):
            conns = []
            centre = None
            for ln in lanes:
                idx = ln["index"]
                if lane_to_junction.get(idx) != jid or idx not in emitted:
                    continue
                if centre is None and ln.get("points"):
                    centre = ln["points"][0]["pos"]
                for lk in ln.get("links", []):
                    if lk.get("type") != "Incoming":
                        continue
                    inc = road_id(lk.get("dest"))
                    if inc and lane_to_junction.get(lk.get("dest")) != jid:
                        conns.append({
                            "incoming": inc,
                            "connecting": f"{prefix}_l{idx}",
                            "contact": "start",
                            "lane_links": [[-1, -1]],
                        })
            junctions.append({"id": jid, "center": centre or [0, 0, 0],
                              "connections": conns})
            if not conns:
                stats["junctions_without_connections"] += 1

    stats["crosswalks"] = len(crosswalks)

    return {
        "source": doc.get("source", "CitySample/ZoneGraph"),
        "map": doc.get("map", ""),
        "roads": roads,
        "junctions": junctions,
        "crosswalks": crosswalks,
        "_stats": dict(stats),
    }


def selftest() -> int:
    """Two vehicle lanes -> two roads; an Intersection lane -> a connecting road
    inside a junction; a pedestrian lane dropped. Links must resolve."""
    doc = {
        "source": "selftest",
        "zone_graphs": [{
            "actor": "zg",
            "lanes": [
                # 0 and 1 are adjacent opposite-direction vehicle lanes. Under
                # the one-road-per-lane model they stay SEPARATE roads - that is
                # what stops their generated meshes overlapping.
                {"index": 0, "width_cm": 400.0, "tags": ["Vehicle"],
                 "points": [{"pos": [0, 0, 0], "tangent": [1, 0, 0]},
                            {"pos": [1000, 0, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 1, "type": "Adjacent",
                            "flags": ["Right", "OppositeDirection"]},
                           {"dest": 2, "type": "Outgoing", "flags": []}]},
                {"index": 1, "width_cm": 400.0, "tags": ["Vehicle"],
                 "points": [{"pos": [0, 400, 0], "tangent": [1, 0, 0]},
                            {"pos": [1000, 400, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 0, "type": "Adjacent", "flags": ["Left"]}]},
                {"index": 2, "width_cm": 400.0, "tags": ["Vehicle", "Intersection"],
                 "points": [{"pos": [1000, 0, 0], "tangent": [1, 0, 0]},
                            {"pos": [1400, 200, 0], "tangent": [0, 1, 0]}],
                 "links": [{"dest": 0, "type": "Incoming", "flags": []}]},
                {"index": 3, "width_cm": 120.0, "tags": ["Pedestrian"],
                 "points": [{"pos": [0, -300, 0], "tangent": [1, 0, 0]},
                            {"pos": [1000, -300, 0], "tangent": [1, 0, 0]}],
                 "links": []},
            ],
        }],
    }

    fails = []
    out = convert(doc)
    by_id = {r["id"]: r for r in out["roads"]}

    if sorted(by_id) != ["zg_l0", "zg_l1", "zg_l2"]:
        fails.append(f"expected roads zg_l0/l1/l2, got {sorted(by_id)}")
    if len(out["junctions"]) != 1:
        fails.append(f"expected 1 junction, got {len(out['junctions'])}")

    # geometry: each road carries exactly one lane, offset by half its width so
    # the lane centre sits on the ZoneGraph centreline
    for rid in ("zg_l0", "zg_l1", "zg_l2"):
        r = by_id.get(rid)
        if not r:
            continue
        if r["lane_count"] != 1 or not r["unidirectional"]:
            fails.append(f"{rid}: expected a single one-way lane")
        if abs(r["lane_offset_cm"] - r["lane_width_cm"] / 2) > 1e-9:
            fails.append(f"{rid}: lane_offset_cm should be half the width")

    # the pedestrian lane must not become a road
    if "zg_l3" in by_id:
        fails.append("pedestrian lane leaked into roads")
    if out["_stats"].get("skipped_sidewalk_lanes", 0) != 1:
        fails.append("pedestrian lane should have been counted as skipped")

    # junction membership
    jid = out["junctions"][0]["id"] if out["junctions"] else None
    if by_id.get("zg_l2", {}).get("junction") != jid:
        fails.append("zg_l2 should belong to the junction")
    if by_id.get("zg_l0", {}).get("junction"):
        fails.append("zg_l0 is an ordinary road and must not claim a junction")

    # THE FIX: longitudinal links must resolve, or vehicles run out of road
    r0 = by_id.get("zg_l0", {})
    if r0.get("successor", {}).get("type") != "junction":
        fails.append(f"zg_l0 successor should be the junction, got {r0.get('successor')}")
    r2 = by_id.get("zg_l2", {})
    if r2.get("predecessor", {}).get("id") != "zg_l0":
        fails.append(f"zg_l2 predecessor should be zg_l0, got {r2.get('predecessor')}")

    # junction must reference its connecting road
    if out["junctions"]:
        conns = out["junctions"][0]["connections"]
        if not any(c["connecting"] == "zg_l2" and c["incoming"] == "zg_l0" for c in conns):
            fails.append(f"junction connection zg_l0 -> zg_l2 missing: {conns}")

    kept = convert(doc, keep_sidewalks=True)
    if len(kept["roads"]) != 4:
        fails.append(f"--keep-sidewalks should yield 4 roads, got {len(kept['roads'])}")

    if fails:
        print("SELFTEST FAILED")
        for f in fails:
            print("  -", f)
        return 1
    print(f"selftest passed - {len(out['roads'])} roads (one per lane), "
          f"{len(out['junctions'])} junction, links resolve, lane offset correct, "
          f"sidewalk dropped")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", nargs="?", help="zonegraph.json from the commandlet")
    ap.add_argument("-o", "--output", default="roadnetwork.json")
    ap.add_argument("--keep-sidewalks", action="store_true",
                    help="emit pedestrian-only lane groups as roads too")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.input:
        ap.error("input JSON required (or pass --selftest)")

    doc = load_json_any(args.input)

    out = convert(doc, keep_sidewalks=args.keep_sidewalks)
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(out, fh, indent=1)

    s = out["_stats"]
    print(f"wrote {args.output}")
    print(f"  roads     : {len(out['roads'])}")
    print(f"  junctions : {len(out['junctions'])}")
    for k, v in sorted(s.items()):
        print(f"  {k:<24}: {v}")
    if not out["junctions"]:
        print("NOTE: no junctions found. If the level really has intersections, check that "
              "lanes carry the 'Intersection' tag in the raw dump.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

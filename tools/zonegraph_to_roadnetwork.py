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


def explicit_lane_type(tags: list[str]) -> str | None:
    for tag, odr in LANE_TYPE_BY_TAG:
        if tag in tags:
            return odr
    return None


def lane_type(tags: list[str]) -> str:
    return explicit_lane_type(tags) or "driving"


def road_type_name(tags: list[str]) -> str:
    return "motorway" if "Freeway" in tags else "town"


def road_speed_kmh(tags: list[str]) -> float:
    """Road-class speed limit (km/h) derived from Epic's ZoneGraph tags.

    City Sample carries no 3D speed-limit sign props, but every ZoneGraph lane
    declares its functional class (`Freeway`, `Trunk Road`, `Freeway Onramp` /
    `Offramp`, or `City` with `Vehicle Density 0..3`). Values are multiples of
    10 km/h matching CARLA's `SpeedLimitModels` (`30..120`) and US MUTCD plates.
    """
    if "Freeway" in tags:
        if "Freeway Onramp" in tags or "Freeway Offramp" in tags:
            return 60.0
        if "Trunk Road" in tags:
            return 90.0
        return 80.0
    if "Vehicle Density 3" in tags:
        return 50.0
    if "Vehicle Density 2" in tags:
        return 40.0
    return 30.0


def resolve_lane_types(lanes: list[dict]) -> dict[int, str]:
    """Resolve OpenDRIVE lane type for every lane, propagating across links when
    Epic left a ZoneGraph shape's tags blank (17 1 m sidewalk lanes in
    Big_City_LVL carry tags=[] while linking only to Pedestrian/Crosswalk lanes)."""
    by_idx = {ln["index"]: ln for ln in lanes}
    resolved: dict[int, str] = {}
    queue: list[int] = []
    for ln in lanes:
        lt = explicit_lane_type(ln.get("tags", []))
        if lt is not None:
            resolved[ln["index"]] = lt
            queue.append(ln["index"])

    adj: dict[int, list[int]] = defaultdict(list)
    for ln in lanes:
        u = ln["index"]
        for lk in ln.get("links", []):
            v = lk.get("dest")
            if v in by_idx:
                adj[u].append(v)
                adj[v].append(u)

    head = 0
    while head < len(queue):
        u = queue[head]
        head += 1
        for v in adj.get(u, ()):
            if v not in resolved:
                resolved[v] = resolved[u]
                queue.append(v)

    for ln in lanes:
        if ln["index"] not in resolved:
            resolved[ln["index"]] = "driving"
    return resolved


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


def _unit_planar_tangent(pts: list[dict], i: int) -> tuple[float, float]:
    """Unit XY tangent in Unreal space at vertex `i`."""
    t = pts[i].get("tangent")
    if t and (abs(t[0]) > 1e-9 or abs(t[1]) > 1e-9):
        tx, ty = float(t[0]), float(t[1])
    else:
        pa = pts[max(i - 1, 0)]["pos"]
        pb = pts[min(i + 1, len(pts) - 1)]["pos"]
        tx, ty = float(pb[0] - pa[0]), float(pb[1] - pa[1])
    n = math.hypot(tx, ty)
    return (tx / n, ty / n) if n > 1e-9 else (1.0, 0.0)


def _parallel_chain_geometry_ok(lane_by_idx: dict[int, dict], chain: list[int],
                                max_err_cm: float = 15.0) -> bool:
    """Verify that hanging `chain[1:]` as right sibling lanes off `chain[0]`'s
    reference line (with `laneOffset = +0.5 * w_0`) places every lane centre
    within `max_err_cm` of its authored ZoneGraph polyline at every vertex."""
    if len(chain) <= 1:
        return True
    ref = lane_by_idx[chain[0]]
    ref_pts = ref["points"]
    n_pts = len(ref_pts)
    widths = [float(lane_by_idx[idx].get("width_cm", 350.0)) for idx in chain]
    ref_tans = [_unit_planar_tangent(ref_pts, k) for k in range(n_pts)]
    cos_min = math.cos(math.radians(4.0))

    cum_right_cm = 0.5 * widths[0]
    for m in range(1, len(chain)):
        ln = lane_by_idx[chain[m]]
        pts = ln["points"]
        if len(pts) != n_pts:
            return False
        offset_cm = cum_right_cm + 0.5 * widths[m]
        for k in range(n_pts):
            ux, uy = ref_tans[k]
            mux, muy = _unit_planar_tangent(pts, k)
            if ux * mux + uy * muy < cos_min:
                return False
            # Unreal is left-handed (+X forward, +Y right), so right normal is (-uy, +ux).
            p0 = ref_pts[k]["pos"]
            pm = pts[k]["pos"]
            ex = p0[0] + offset_cm * (-uy)
            ey = p0[1] + offset_cm * ux
            if math.hypot(pm[0] - ex, pm[1] - ey, pm[2] - p0[2]) > max_err_cm:
                return False
        cum_right_cm += widths[m]
    return True


def _build_parallel_chains(lanes: list[dict], emitted: set[int],
                           lane_to_junction: dict[int, str],
                           ltypes: dict[int, str]) -> list[list[int]]:
    """Build ordered Left -> Right chains of co-parallel same-direction lanes
    outside junctions, refined to fixed-point endpoint compatibility so every
    multi-lane `<road>` has unambiguous OpenDRIVE road/lane links."""
    lane_by_idx = {ln["index"]: ln for ln in lanes if ln["index"] in emitted}

    def can_pair(left_idx: int, right_idx: int) -> bool:
        if left_idx == right_idx:
            return False
        if left_idx in lane_to_junction or right_idx in lane_to_junction:
            return False
        if ltypes.get(left_idx) != "driving" or ltypes.get(right_idx) != "driving":
            return False
        a, b = lane_by_idx[left_idx], lane_by_idx[right_idx]
        ta, tb = a.get("tags", []), b.get("tags", [])
        if (road_type_name(ta) != road_type_name(tb)
                or road_speed_kmh(ta) != road_speed_kmh(tb)
                or ("Freeway" in ta) != ("Freeway" in tb)):
            return False
        if len(a["points"]) != len(b["points"]):
            return False
        return _parallel_chain_geometry_ok(lane_by_idx, [left_idx, right_idx])

    right_nbr: dict[int, int] = {}
    left_nbr: dict[int, int] = {}
    bad_flags = {"OppositeDirection", "Splitting", "Merging"}

    for idx in sorted(lane_by_idx):
        ln = lane_by_idx[idx]
        r_cands = [
            lk["dest"] for lk in ln.get("links", [])
            if lk.get("type") == "Adjacent"
            and "Right" in lk.get("flags", ())
            and not (bad_flags & set(lk.get("flags", ())))
            and lk.get("dest") in lane_by_idx
        ]
        if len(r_cands) != 1:
            continue
        r_idx = r_cands[0]
        # Require reciprocal Left link on r_idx with no splitting/merging/opposite flags.
        l_cands = [
            lk["dest"] for lk in lane_by_idx[r_idx].get("links", [])
            if lk.get("type") == "Adjacent"
            and "Left" in lk.get("flags", ())
            and not (bad_flags & set(lk.get("flags", ())))
            and lk.get("dest") in lane_by_idx
        ]
        if l_cands == [idx] and can_pair(idx, r_idx):
            right_nbr[idx] = r_idx
            left_nbr[r_idx] = idx

    chains: list[list[int]] = []
    visited: set[int] = set()
    for ln in lanes:
        idx = ln["index"]
        if idx not in lane_by_idx or idx in visited:
            continue
        if idx in left_nbr:
            continue  # Wait until we hit the leftmost head of this chain (unless cyclic).
        cur = [idx]
        visited.add(idx)
        nxt = right_nbr.get(idx)
        while nxt is not None and nxt not in visited:
            if _parallel_chain_geometry_ok(lane_by_idx, cur + [nxt]):
                cur.append(nxt)
                visited.add(nxt)
                nxt = right_nbr.get(nxt)
            else:
                break
        chains.append(cur)

    for ln in lanes:
        idx = ln["index"]
        if idx in lane_by_idx and idx not in visited:
            chains.append([idx])
            visited.add(idx)

    # Iteratively split any multi-lane chain whose lanes do not share the same
    # longitudinal road/junction predecessor and successor.
    while True:
        chain_of: dict[int, int] = {}
        pos_in_chain: dict[int, int] = {}
        for ci, ch in enumerate(chains):
            for pos, lidx in enumerate(ch):
                chain_of[lidx] = ci
                pos_in_chain[lidx] = pos

        def end_signature(ci: int, lidx: int, link_type: str):
            targets = []
            for lk in lane_by_idx[lidx].get("links", []):
                if lk.get("type") != link_type:
                    continue
                dest = lk.get("dest")
                if dest not in lane_by_idx:
                    continue
                dj = lane_to_junction.get(dest)
                if dj:
                    targets.append(("junction", dj, 0))
                else:
                    dci = chain_of[dest]
                    if dci == ci:
                        # Self-loop onto same chain cannot be represented on one road.
                        return ("SELF", lidx)
                    targets.append(("road", dci, pos_in_chain[dest]))
            if not targets:
                return ("NONE", None)
            juncs = {t[1] for t in targets if t[0] == "junction"}
            roads_t = [(t[1], t[2]) for t in targets if t[0] == "road"]
            if juncs and not roads_t and len(juncs) == 1:
                return ("JUNCTION", next(iter(juncs)))
            if len(roads_t) == 1 and not juncs:
                return ("ROAD", roads_t[0][0], roads_t[0][1])
            # Ambiguous / fork outside junction: isolate this lane so it never
            # corrupts a neighbour's multi-lane road link.
            return ("ISOLATE", lidx)

        new_chains: list[list[int]] = []
        changed = False
        for ci, ch in enumerate(chains):
            if len(ch) <= 1:
                new_chains.append(ch)
                continue

            # Partition contiguous runs of `ch` that share compatible (out, in) road/junction targets
            # AND target distinct lanes when linking road-to-road.
            runs: list[list[int]] = []
            cur_run: list[int] = []
            cur_key = None
            used_out_lanes: set[int] = set()
            used_in_lanes: set[int] = set()

            for lidx in ch:
                osig = end_signature(ci, lidx, "Outgoing")
                isig = end_signature(ci, lidx, "Incoming")
                if osig[0] in ("SELF", "ISOLATE") or isig[0] in ("SELF", "ISOLATE"):
                    if cur_run:
                        runs.append(cur_run)
                        cur_run = []
                        cur_key = None
                    runs.append([lidx])
                    continue

                okey = (osig[0], osig[1])
                ikey = (isig[0], isig[1])
                out_lane = osig[2] if osig[0] == "ROAD" else None
                in_lane = isig[2] if isig[0] == "ROAD" else None

                conflict = (
                    cur_key is not None and (
                        (okey, ikey) != cur_key
                        or (out_lane is not None and out_lane in used_out_lanes)
                        or (in_lane is not None and in_lane in used_in_lanes)
                    )
                )
                if conflict:
                    runs.append(cur_run)
                    cur_run = []
                    used_out_lanes = set()
                    used_in_lanes = set()

                if not cur_run:
                    cur_key = (okey, ikey)
                    used_out_lanes = set()
                    used_in_lanes = set()
                cur_run.append(lidx)
                if out_lane is not None:
                    used_out_lanes.add(out_lane)
                if in_lane is not None:
                    used_in_lanes.add(in_lane)

            if cur_run:
                runs.append(cur_run)

            if len(runs) > 1:
                changed = True
                for r in runs:
                    # Re-verify geometry whenever chain head changes (r[0] != ch[0]).
                    if len(r) <= 1 or _parallel_chain_geometry_ok(lane_by_idx, r):
                        new_chains.append(r)
                    else:
                        for single in r:
                            new_chains.append([single])
            else:
                new_chains.append(ch)

        chains = new_chains
        if not changed:
            break

    # Preserve deterministic ordering by first lane's appearance in `lanes`.
    order_of = {ln["index"]: pos for pos, ln in enumerate(lanes)}
    chains.sort(key=lambda ch: order_of[ch[0]])
    return chains


def convert(doc: dict, *, keep_sidewalks: bool = False,
            single_lane_roads: bool = False) -> dict:
    """Convert ZoneGraph lanes into OpenDRIVE-ready roads, junctions, and crosswalks.

    Co-parallel same-direction non-intersection vehicle lanes (`Adjacent` Left/Right
    with no `OppositeDirection`, `Splitting`, or `Merging`, matching vertex counts
    within 15 cm of lateral offset, and compatible endpoints) are grouped into
    multi-lane unidirectional `<road>` records (`lane_count = k`, `lane_widths_cm`,
    `lane_offset_cm = 0.5 * w_0`) so CARLA's `Waypoint.get_left_lane()` /
    `get_right_lane()` and Traffic Manager lane changes work natively without
    introducing overlapping opposite-direction or gore-area road meshes.
    Pass `single_lane_roads=True` (`--single-lane-roads`) for 1-lane-per-road mode.
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

        ltypes = resolve_lane_types(lanes)

        def drivable(ln: dict) -> bool:
            if keep_sidewalks:
                # Crossings are already emitted as crosswalk objects. As
                # sidewalk-typed roads as well they would lay sidewalk area
                # straight across the carriageway in the navmesh, over the
                # crosswalk area that tells a walker it is crossing.
                return "Crosswalk" not in ln.get("tags", [])
            return ltypes[ln["index"]] != "sidewalk"

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

        # Groups are still needed, but only to identify junctions: an
        # Adjacent-linked cluster of emitted lanes containing an Intersection
        # lane is one junction. Restricting to emitted lanes prevents skipped
        # Pedestrian Intersection lanes from creating empty junctions or pulling
        # adjacent lanes into them.
        emitted_lanes = [ln for ln in lanes if ln["index"] in emitted]
        lane_by_idx = {ln["index"]: ln for ln in emitted_lanes}
        lane_to_junction: dict[int, str] = {}
        for gi, group in enumerate(group_lanes(emitted_lanes)):
            members = [emitted_lanes[i] for i in group]
            if any("Intersection" in m.get("tags", []) for m in members):
                jid = f"{prefix}_j{gi}"
                for m in members:
                    lane_to_junction[m["index"]] = jid

        if single_lane_roads:
            chains = [[ln["index"]] for ln in emitted_lanes]
        else:
            chains = _build_parallel_chains(lanes, emitted, lane_to_junction, ltypes)

        # Every emitted lane maps to (road_id, odr_lane_id), where odr_lane_id is
        # -1 for single-lane roads and -1..-k (left to right) for grouped roads.
        lane_map: dict[int, tuple[str, int]] = {}
        for ch in chains:
            rid = f"{prefix}_l{ch[0]}"
            for pos, lidx in enumerate(ch):
                lane_map[lidx] = (rid, -(pos + 1))

        used_junctions: set[str] = set()
        for ch in chains:
            head_idx = ch[0]
            head_ln = lane_by_idx[head_idx]
            rid = f"{prefix}_l{head_idx}"
            tags = head_ln.get("tags", [])
            jid = lane_to_junction.get(head_idx)
            n_lanes = len(ch)
            if n_lanes > 1:
                stats["multilane_roads"] += 1
                stats["grouped_parallel_lanes"] += n_lanes

            succ = pred = None
            lane_succ_ids: list[int | None] = [None] * n_lanes
            lane_pred_ids: list[int | None] = [None] * n_lanes

            for pos, lidx in enumerate(ch):
                ln = lane_by_idx[lidx]
                succ_targets = set()
                for lk in ln.get("links", []):
                    t = lk.get("type")
                    if t not in ("Outgoing", "Incoming"):
                        continue
                    dest = lk.get("dest")
                    if dest not in lane_map:
                        stats["unresolved_links"] += 1
                        continue
                    dest_rid, dest_lane_id = lane_map[dest]
                    dj = lane_to_junction.get(dest)
                    if dj and dj != jid:
                        cand = {"type": "junction", "id": dj}
                        cand_lane = None
                    else:
                        cand = {"type": "road", "id": dest_rid,
                                "contact": "start" if t == "Outgoing" else "end"}
                        cand_lane = dest_lane_id

                    if t == "Outgoing":
                        succ_targets.add((cand["type"], cand["id"]))
                        if succ is None:
                            succ = cand
                        if lane_succ_ids[pos] is None and cand["type"] == "road" and cand["id"] == succ["id"]:
                            lane_succ_ids[pos] = cand_lane
                    elif t == "Incoming":
                        if pred is None:
                            pred = cand
                        if lane_pred_ids[pos] is None and cand["type"] == "road" and cand["id"] == pred["id"]:
                            lane_pred_ids[pos] = cand_lane

                if len(succ_targets) > 1:
                    stats["forks_outside_junctions"] += 1

            if succ is None and pred is None:
                stats["roads_without_links"] += 1
            if succ is None:
                stats["roads_without_successor"] += 1

            widths = [float(lane_by_idx[lidx].get("width_cm", 350.0)) for lidx in ch]
            width0 = widths[0]
            road_rec = {
                "id": rid,
                "points": [{"pos": p["pos"], "tangent": p.get("tangent", [0, 0, 0])}
                           for p in head_ln["points"]],
                "lane_width_cm": width0,
                "lane_widths_cm": widths,
                # Directed right-hand lane(s) -1..-k. laneOffset = +w_0/2 places
                # lane -1's centre on the leftmost ZoneGraph lane's centreline, and
                # lanes -2..-k tile immediately to its right.
                "unidirectional": True,
                "lane_count": n_lanes,
                "lane_offset_cm": width0 / 2.0,
                "lane_succ_ids": lane_succ_ids,
                "lane_pred_ids": lane_pred_ids,
                "is_freeway": "Freeway" in tags,
                "road_type": road_type_name(tags),
                "speed_kmh": road_speed_kmh(tags),
                "lane_type": ltypes[head_idx],
                "tags": tags,
                **({"junction": jid} if jid else {}),
                **({"successor": succ} if succ else {}),
                **({"predecessor": pred} if pred else {}),
            }
            roads.append(road_rec)
            stats["connecting_roads" if jid else "roads"] += 1
            if jid:
                used_junctions.add(jid)

        # Junction records, with connections resolved from each incoming road's
        # specific lane id (-1..-k) into the connecting road's lane -1.
        junction_lanes: dict[str, list[dict]] = defaultdict(list)
        for ln in lanes:
            jid = lane_to_junction.get(ln["index"])
            if jid is not None and ln["index"] in emitted:
                junction_lanes[jid].append(ln)
        for jid in sorted(used_junctions):
            conns = []
            seen_pairs: set[tuple[str, str]] = set()
            centre = None
            for ln in junction_lanes[jid]:
                idx = ln["index"]
                conn_rid = lane_map[idx][0]
                if centre is None and ln.get("points"):
                    centre = ln["points"][0]["pos"]
                for lk in ln.get("links", []):
                    if lk.get("type") != "Incoming":
                        continue
                    src = lk.get("dest")
                    if src in lane_map and lane_to_junction.get(src) != jid:
                        inc_rid, inc_lane_id = lane_map[src]
                        if (inc_rid, conn_rid) not in seen_pairs:
                            seen_pairs.add((inc_rid, conn_rid))
                            conns.append({
                                "incoming": inc_rid,
                                "connecting": conn_rid,
                                "contact": "start",
                                "lane_links": [[inc_lane_id, -1]],
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

    # Untagged sidewalk lanes (Big_City_LVL has 17 1 m lanes with tags=[] linked
    # only to Pedestrian/Crosswalk lanes) must inherit "sidewalk" from their
    # neighbours instead of defaulting to "driving" and creating dead-end
    # junction approaches.
    untagged_sw = {
        "source": "selftest-untagged-sidewalk",
        "zone_graphs": [{
            "actor": "zg",
            "lanes": [
                {"index": 0, "width_cm": 100.0, "tags": [],
                 "points": [{"pos": [0, 0, 0]}, {"pos": [500, 0, 0]}],
                 "links": [{"dest": 1, "type": "Outgoing", "flags": []}]},
                {"index": 1, "width_cm": 100.0, "tags": ["Pedestrian", "Intersection"],
                 "points": [{"pos": [500, 0, 0]}, {"pos": [900, 0, 0]}],
                 "links": [{"dest": 0, "type": "Incoming", "flags": []}]},
            ],
        }],
    }
    sw_default = convert(untagged_sw)
    if sw_default["roads"] or sw_default["junctions"]:
        fails.append(f"untagged sidewalk leaked into driving network: {sw_default['roads']}")
    sw_kept = convert(untagged_sw, keep_sidewalks=True)
    if {r["id"]: r["lane_type"] for r in sw_kept["roads"]} != {"zg_l0": "sidewalk", "zg_l1": "sidewalk"}:
        fails.append(f"untagged sidewalk did not inherit sidewalk type: {sw_kept['roads']}")

    # Road-class speeds from ZoneGraph tags: Trunk freeway 90, ramp 60, arterial 50, city 30.
    if (road_speed_kmh(["Freeway", "Trunk Road", "Vehicle"]),
        road_speed_kmh(["Freeway", "Freeway Onramp", "Vehicle"]),
        road_speed_kmh(["City", "Vehicle", "Vehicle Density 3"]),
        road_speed_kmh(["City", "Vehicle", "Vehicle Density 0"])) != (90.0, 60.0, 50.0, 30.0):
        fails.append("road_speed_kmh mapping mismatch")

    # Co-parallel same-direction lanes (Left -> Right, no OppositeDirection /
    # Splitting / Merging) must group into one multi-lane road with exact
    # per-lane widths, junction lane_links [-1,-1]/[-2,-1], and connecting
    # road successor lane ids [-1]/[-2].
    ml_doc = {
        "source": "selftest-multilane",
        "zone_graphs": [{
            "actor": "zg",
            "lanes": [
                # 2-lane approach (lane 0 left 350 cm at y=0, lane 1 right 400 cm at y=+375 cm)
                {"index": 0, "width_cm": 350.0, "tags": ["Vehicle"],
                 "points": [{"pos": [0, 0, 0], "tangent": [1, 0, 0]},
                            {"pos": [2000, 0, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 1, "type": "Adjacent", "flags": ["Right"]},
                           {"dest": 2, "type": "Outgoing", "flags": []}]},
                {"index": 1, "width_cm": 400.0, "tags": ["Vehicle"],
                 "points": [{"pos": [0, 375, 0], "tangent": [1, 0, 0]},
                            {"pos": [2000, 375, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 0, "type": "Adjacent", "flags": ["Left"]},
                           {"dest": 3, "type": "Outgoing", "flags": []}]},
                # 2 connecting lanes inside the junction
                {"index": 2, "width_cm": 350.0, "tags": ["Vehicle", "Intersection"],
                 "points": [{"pos": [2000, 0, 0], "tangent": [1, 0, 0]},
                            {"pos": [3000, 0, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 0, "type": "Incoming", "flags": []},
                           {"dest": 4, "type": "Outgoing", "flags": []},
                           {"dest": 3, "type": "Adjacent", "flags": ["Right"]}]},
                {"index": 3, "width_cm": 400.0, "tags": ["Vehicle", "Intersection"],
                 "points": [{"pos": [2000, 375, 0], "tangent": [1, 0, 0]},
                            {"pos": [3000, 375, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 1, "type": "Incoming", "flags": []},
                           {"dest": 5, "type": "Outgoing", "flags": []},
                           {"dest": 2, "type": "Adjacent", "flags": ["Left"]}]},
                # 2-lane exit road
                {"index": 4, "width_cm": 350.0, "tags": ["Vehicle"],
                 "points": [{"pos": [3000, 0, 0], "tangent": [1, 0, 0]},
                            {"pos": [5000, 0, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 5, "type": "Adjacent", "flags": ["Right"]},
                           {"dest": 2, "type": "Incoming", "flags": []}]},
                {"index": 5, "width_cm": 400.0, "tags": ["Vehicle"],
                 "points": [{"pos": [3000, 375, 0], "tangent": [1, 0, 0]},
                            {"pos": [5000, 375, 0], "tangent": [1, 0, 0]}],
                 "links": [{"dest": 4, "type": "Adjacent", "flags": ["Left"]},
                           {"dest": 3, "type": "Incoming", "flags": []}]},
            ],
        }],
    }
    ml = convert(ml_doc)
    ml_by_id = {r["id"]: r for r in ml["roads"]}
    if sorted(ml_by_id) != ["zg_l0", "zg_l2", "zg_l3", "zg_l4"]:
        fails.append(f"multilane expected ['zg_l0', 'zg_l2', 'zg_l3', 'zg_l4'], got {sorted(ml_by_id)}")
    else:
        if ml_by_id["zg_l0"]["lane_count"] != 2 or ml_by_id["zg_l0"]["lane_widths_cm"] != [350.0, 400.0]:
            fails.append(f"zg_l0 expected 2 lanes [350, 400], got {ml_by_id['zg_l0']}")
        if ml_by_id["zg_l2"]["lane_pred_ids"] != [-1] or ml_by_id["zg_l2"]["lane_succ_ids"] != [-1]:
            fails.append(f"zg_l2 expected pred [-1] succ [-1], got {ml_by_id['zg_l2']}")
        if ml_by_id["zg_l3"]["lane_pred_ids"] != [-2] or ml_by_id["zg_l3"]["lane_succ_ids"] != [-2]:
            fails.append(f"zg_l3 expected pred [-2] succ [-2], got {ml_by_id['zg_l3']}")

    if fails:
        print("SELFTEST FAILED")
        for f in fails:
            print("  -", f)
        return 1
    print(f"selftest passed - opposite-direction lanes separated, parallel lanes grouped "
          f"({ml['_stats'].get('multilane_roads', 0)} multi-lane roads), "
          f"links resolve, sidewalk dropped")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", nargs="?", help="zonegraph.json from the commandlet")
    ap.add_argument("-o", "--output", default="roadnetwork.json")
    ap.add_argument("--keep-sidewalks", action="store_true",
                    help="emit pedestrian-only lane groups as roads too")
    ap.add_argument("--single-lane-roads", action="store_true",
                    help="emit one OpenDRIVE road per ZoneGraph lane without parallel grouping")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.input:
        ap.error("input JSON required (or pass --selftest)")

    doc = load_json_any(args.input)

    out = convert(doc, keep_sidewalks=args.keep_sidewalks,
                  single_lane_roads=args.single_lane_roads)
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

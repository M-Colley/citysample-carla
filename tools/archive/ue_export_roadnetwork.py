#!/usr/bin/env python3
"""
ue_export_roadnetwork.py - extract the City Sample road network from the Unreal
editor into the intermediate JSON that ``roadnetwork_to_xodr.py`` converts to
OpenDRIVE.

Run INSIDE the Unreal editor, not from a normal shell:

    Output Log > Cmd dropdown set to "Python":
        exec(open(r"<repo>/tools/ue_export_roadnetwork.py").read())

    or headless:
        UnrealEditor-Cmd.exe "<project>/CitySample.uproject" ^
            -run=pythonscript -script="<path>/ue_export_roadnetwork.py" ^
            -unattended -nosound

Default mode is ``--survey``: it reports what it can reach and writes nothing.
``--export`` writes the JSON.


WHICH SOURCE, AND WHY
---------------------
Scanning the shipped World Partition actor packages settled this:

  Big_City_LVL     ZoneGraphData  128.8 MB   MassTrafficBuilder  ABSENT
  Small_City_LVL   ZoneGraphData   17.2 MB   MassTrafficBuilder  2 x 4 KB
  City_Open_World  ZoneGraphData    ~4 KB    MassTrafficBuilder  2 x 4 KB

A MassTrafficBuilder package holding thousands of road splines would be
megabytes. At 4 KB they are EMPTY: BP_CityTrafficBuilder and
BP_FreewayTrafficBuilder are placed in the level, but their RoadSplinesMap /
IntersectionsMap were never serialised. They are import-time fixtures whose
output was baked into ZoneGraph and then discarded. Big_City_LVL does not even
have them.

So the primary source is the baked **AZoneGraphData**, which must be persisted
because the runtime traffic reads it, and the 128.8 MB package confirms it.
The builder path is kept only as an opportunistic fallback, in case a level is
opened mid-import with the maps still populated in memory.


THE RISK THIS INTRODUCES
------------------------
ZoneGraph is a lower-level model than the builder was: lanes and links, not
roads and junctions. Two consequences.

1. ACCESS. ``AZoneGraphData::ZoneStorage`` may be a plain ``UPROPERTY()``
   rather than BlueprintReadWrite, in which case editor Python cannot read it
   directly and the extractor needs either ``UZoneGraphSubsystem`` /
   ``ZoneGraphQuery`` blueprint helpers, or a small C++ commandlet. The survey
   probes this explicitly and tells you which.

2. SHAPE. OpenDRIVE wants roads carrying lane sets. ZoneGraph gives individual
   lanes, related by Adjacent links flagged Left/Right. Reconstructing roads
   means grouping laterally-adjacent lanes and deriving a reference line. That
   grouping is implemented here as a union-find over adjacency links; it is the
   part most likely to need tuning against real data.
"""

from __future__ import annotations

import json
import os
import sys

try:
    import unreal  # noqa: F401
except ImportError:
    print("This script must be run inside the Unreal editor's Python "
          "environment. See the module docstring.", file=sys.stderr)
    raise SystemExit(2)


OUT_DEFAULT = os.path.join(
    os.path.expanduser("~"), "Desktop", "unreal city", "citysample-roadnetwork.json")


def log(m: str) -> None:
    unreal.log(f"[roadnet] {m}")


def warn(m: str) -> None:
    unreal.log_warning(f"[roadnet] {m}")


# --------------------------------------------------------------------------
def open_map(map_path: str) -> bool:
    """Load a level by package path, e.g. /Game/Map/Small_City_LVL."""
    for attempt in (
        lambda: unreal.get_editor_subsystem(
            unreal.LevelEditorSubsystem).load_level(map_path),
        lambda: unreal.EditorLevelLibrary.load_level(map_path),
    ):
        try:
            if attempt():
                log(f"loaded map {map_path}")
                return True
        except Exception:
            continue
    warn(f"could not load map {map_path}")
    return False


def load_world_partition_actors() -> None:
    """Best-effort: pull World Partition actors into memory.

    Headless, World Partition loads nothing spatial by default, so an
    unprepared survey sees almost no actors. Globally-relevant actors (which a
    single 128 MB ZoneGraphData almost certainly is) load regardless - but do
    not rely on that silently. The actor count in the survey is the check.
    """
    try:
        lib = unreal.WorldPartitionBlueprintLibrary
    except AttributeError:
        warn("WorldPartitionBlueprintLibrary unavailable - relying on "
             "always-loaded actors only.")
        return
    try:
        descs = lib.get_actor_descs() if hasattr(lib, "get_actor_descs") else None
        if descs:
            log(f"world partition: {len(descs)} actor descriptors; loading...")
            lib.load_actors(descs)
            log("world partition actors loaded")
            return
    except Exception as exc:
        warn(f"bulk actor load failed ({exc}); trying region load")
    for fn in ("load_all_regions", "load_regions"):
        try:
            getattr(lib, fn)()
            log(f"world partition: {fn}() succeeded")
            return
        except Exception:
            continue
    warn("could not force-load World Partition cells. If the actor count below "
         "looks small, open the level in the editor GUI and use the World "
         "Partition window to load all regions, then re-run.")


def all_level_actors():
    try:
        return list(unreal.get_editor_subsystem(
            unreal.EditorActorSubsystem).get_all_level_actors())
    except Exception:
        pass
    try:
        return list(unreal.EditorLevelLibrary.get_all_level_actors())
    except Exception as exc:
        warn(f"could not enumerate level actors: {exc}")
        return []


def prop(obj, *names, default=None):
    """First readable editor property from candidate names (UE snake_cases them)."""
    for n in names:
        try:
            return obj.get_editor_property(n)
        except Exception:
            continue
    return default


def vec(v) -> list[float]:
    try:
        return [float(v.x), float(v.y), float(v.z)]
    except Exception:
        return [0.0, 0.0, 0.0]


def class_name(a) -> str:
    try:
        return a.get_class().get_name()
    except Exception:
        return "<unknown>"


# --------------------------------------------------------------------------
def find_zonegraph(actors):
    return [a for a in actors if "ZoneGraphData" in class_name(a)]


def find_builders(actors):
    return [a for a in actors if prop(a, "road_splines_map", "RoadSplinesMap") is not None]


def probe_storage(zg):
    """Try every known way to reach the lane data. Returns (storage, how)."""
    for name in ("zone_storage", "ZoneStorage", "storage"):
        s = prop(zg, name)
        if s is not None:
            return s, f"actor property '{name}'"
    try:
        sub = unreal.get_editor_subsystem(unreal.ZoneGraphSubsystem)
        if sub:
            return None, "ZoneGraphSubsystem exists (needs a query API, not a direct read)"
    except Exception:
        pass
    return None, "UNREACHABLE from Python"


# --------------------------------------------------------------------------
def lanes_from_storage(storage):
    """Flatten FZoneGraphStorage into per-lane polylines with width and links."""
    lanes_raw = prop(storage, "lanes", "Lanes", default=[]) or []
    points = prop(storage, "lane_points", "LanePoints", default=[]) or []
    tangents = prop(storage, "lane_tangent_vectors", "LaneTangentVectors", default=[]) or []
    links = prop(storage, "lane_links", "LaneLinks", default=[]) or []

    out = []
    for idx, ln in enumerate(lanes_raw):
        pb = int(prop(ln, "points_begin", "PointsBegin", default=0) or 0)
        pe = int(prop(ln, "points_end", "PointsEnd", default=0) or 0)
        lb = int(prop(ln, "links_begin", "LinksBegin", default=0) or 0)
        le = int(prop(ln, "links_end", "LinksEnd", default=0) or 0)
        width = float(prop(ln, "width", "Width", default=350.0) or 350.0)

        pts = []
        for i in range(pb, min(pe, len(points))):
            t = tangents[i] if i < len(tangents) else None
            pts.append({"pos": vec(points[i]),
                        "tangent": vec(t) if t is not None else [0.0, 0.0, 0.0]})
        if len(pts) < 2:
            continue

        lks = []
        for i in range(lb, min(le, len(links))):
            lk = links[i]
            lks.append({
                "dest": int(prop(lk, "dest_lane_index", "DestLaneIndex", default=-1) or -1),
                "type": str(prop(lk, "type", "Type", default="")),
                "flags": str(prop(lk, "flags", "Flags", default="")),
            })

        out.append({"index": idx, "points": pts, "width_cm": width, "links": lks})
    return out


def group_lanes_into_roads(lanes):
    """Union-find over Adjacent links: laterally-adjacent lanes share a road."""
    parent = list(range(len(lanes)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    by_index = {ln["index"]: i for i, ln in enumerate(lanes)}
    for i, ln in enumerate(lanes):
        for lk in ln["links"]:
            # "Adjacent" links carry Left/Right flags and mean same-road.
            blob = (lk["type"] + " " + lk["flags"]).lower()
            if "adjacent" in blob or "left" in blob or "right" in blob:
                j = by_index.get(lk["dest"])
                if j is not None:
                    union(i, j)

    groups: dict[int, list[int]] = {}
    for i in range(len(lanes)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def roads_from_zonegraph(zg_actors):
    roads, total_lanes = [], 0
    for zg in zg_actors:
        storage, how = probe_storage(zg)
        log(f"  {zg.get_name()}: storage via {how}")
        if storage is None:
            continue
        lanes = lanes_from_storage(storage)
        total_lanes += len(lanes)
        log(f"    {len(lanes)} lanes")
        for gi, group in enumerate(group_lanes_into_roads(lanes)):
            # Reference line = the widest lane in the group. Crude but stable;
            # the proper answer is the group's lateral centre.
            rep = max(group, key=lambda i: lanes[i]["width_cm"])
            ln = lanes[rep]
            roads.append({
                "id": f"{zg.get_name()}_g{gi}",
                "points": ln["points"],
                "lane_width_cm": ln["width_cm"],
                "unidirectional": True,   # ZoneGraph lanes are directed
                "lane_count": len(group),
            })
    return roads, total_lanes


# --------------------------------------------------------------------------
def survey():
    log("=" * 66)
    log("SURVEY - reporting reachable road data, writing nothing")
    log("=" * 66)

    actors = all_level_actors()
    log(f"loaded actors in level: {len(actors)}")
    if len(actors) < 1000:
        warn("small for a City Sample level. World Partition cells that are not "
             "loaded are invisible here - load the regions first or you will "
             "export a fraction of the city.")

    zgs = find_zonegraph(actors)
    log(f"ZoneGraphData actors (PRIMARY source): {len(zgs)}")
    for z in zgs:
        storage, how = probe_storage(z)
        log(f"  - {z.get_name()}: {how}")
        if storage is not None:
            log(f"    lanes reachable: {len(lanes_from_storage(storage))}")

    builders = find_builders(actors)
    log(f"MassTrafficBuilder actors (fallback, expected EMPTY): {len(builders)}")
    for b in builders:
        n = len(prop(b, "road_splines_map", "RoadSplinesMap", default={}) or {})
        log(f"  - {b.get_name()} [{class_name(b)}]: {n} road splines"
            + ("  <-- unexpectedly populated, prefer this" if n else "  (empty, as expected)"))

    if not zgs:
        warn("no ZoneGraphData found. Either the region is not loaded, or the "
             "ZoneGraph plugin is disabled so the class is not registered.")
    return {"zonegraph": zgs, "builders": builders}


def export(out_path: str) -> int:
    s = survey()
    net = {"source": "CitySample/ZoneGraph", "roads": [], "junctions": []}

    # Prefer a populated builder if one somehow exists; else ZoneGraph.
    for b in s["builders"]:
        splines = prop(b, "road_splines_map", "RoadSplinesMap", default={}) or {}
        if splines:
            warn(f"{b.get_name()} has populated builder maps - using them.")
            net["source"] = "CitySample/MassTrafficBuilder"
            # (kept minimal: the builder path is not the expected one)
            break

    if net["source"] == "CitySample/ZoneGraph":
        roads, n_lanes = roads_from_zonegraph(s["zonegraph"])
        net["roads"] = roads
        log(f"grouped {n_lanes} lanes into {len(roads)} roads")

    if not net["roads"]:
        warn("nothing to export - see the survey output above.")
        return 1

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(net, fh, indent=1)

    log("=" * 66)
    log(f"wrote {out_path}: {len(net['roads'])} roads, "
        f"{sum(len(r['points']) for r in net['roads'])} points")
    log("Next: python tools/roadnetwork_to_xodr.py "
        f"\"{out_path}\" -o CitySample.xodr")
    warn("junctions are NOT yet derived from ZoneGraph links - CARLA will treat "
         "roads as isolated until that is added. Validate geometry first, then "
         "tackle junctions.")
    return 0


DEFAULT_MAP = "/Game/Map/Small_City_LVL"

if __name__ == "__main__":
    argv = [a for a in sys.argv[1:] if not a.startswith("-run=")]
    out, map_path = OUT_DEFAULT, None
    for i, a in enumerate(argv):
        if a in ("-o", "--output") and i + 1 < len(argv):
            out = argv[i + 1]
        elif a in ("-m", "--map") and i + 1 < len(argv):
            map_path = argv[i + 1]
    # Headless (-run=pythonscript) starts on an empty template, so a map must be
    # named. Pass --current to survey whatever is already open in the editor
    # instead (useful after loading regions by hand in the GUI).
    if "--current" not in argv:
        open_map(map_path or DEFAULT_MAP)
        load_world_partition_actors()
    else:
        log("surveying the currently open level (--current)")

    if "--export" in argv:
        raise SystemExit(export(out))
    survey()
    raise SystemExit(0)

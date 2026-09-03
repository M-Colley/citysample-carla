#!/usr/bin/env python3
"""
ue_probe_zonegraph.py - find a readable path to the City Sample lane graph.

Round 1 established:
  - unreal.ZoneGraphData IS an exposed Python type.
  - MassTrafficZoneGraphDataModifier.zone_graph_data IS readable and returns a
    ZoneGraphData object.
  - ZoneGraphSubsystem / ZoneGraphBlueprintLibrary / ZoneGraphQuery are NOT
    exposed to Python at all.

Round 2 (this file) enumerates the ZoneGraphData object exhaustively - every
dir() member and every reflected property - to find where the lane arrays live.
"""

from __future__ import annotations

import sys

try:
    import unreal
except ImportError:
    print("Run inside the Unreal editor.", file=sys.stderr)
    raise SystemExit(2)

MAP = "/Game/Map/Small_City_LVL"


def log(m):
    unreal.log(f"[probe] {m}")


try:
    unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).load_level(MAP)
except Exception as e:
    log(f"load_level: {e}")
try:
    lib = unreal.WorldPartitionBlueprintLibrary
    d = lib.get_actor_descs()
    if d:
        lib.load_actors(d)
        log(f"loaded {len(d)} WP actor descs")
except Exception as e:
    log(f"WP load: {e}")

acts = list(unreal.get_editor_subsystem(
    unreal.EditorActorSubsystem).get_all_level_actors())

# Collect every ZoneGraphData object we can reach, by either route.
targets = []
for a in acts:
    cn = a.get_class().get_name()
    if cn == "ZoneGraphData":
        targets.append(("actor", a))
    elif cn == "MassTrafficZoneGraphDataModifier":
        try:
            z = a.get_editor_property("zone_graph_data")
            if z:
                targets.append(("via modifier", z))
        except Exception:
            pass

log(f"=== {len(targets)} ZoneGraphData object(s) ===")

for how, z in targets:
    log("-" * 66)
    log(f"[{how}] {z.get_name() if hasattr(z,'get_name') else z}  type={type(z).__name__}")

    members = [m for m in dir(z) if not m.startswith("__")]
    log(f"  dir() -> {len(members)} members")
    log(f"  {members}")

    # Try each member name as an editor property; report the ones that resolve
    # and, for containers, how big they are. This is what identifies the storage.
    #
    # Skip anything that is callable on the instance: get_editor_property() on a
    # method name reaches unreflected paths and can take the whole editor down
    # with an access violation rather than raising.
    log("  --- readable editor properties ---")
    for m in members:
        try:
            if callable(getattr(z, m, None)):
                continue
        except Exception:
            continue
        try:
            v = z.get_editor_property(m)
        except Exception:
            continue
        tn = type(v).__name__
        extra = ""
        try:
            extra = f"  len={len(v)}"
        except Exception:
            pass
        log(f"    {m}: {tn}{extra}")

# The struct type itself may be exposed even if the property is not.
log("-" * 66)
for t in ("ZoneGraphStorage", "ZoneData", "ZoneLaneData", "ZoneLaneLinkData",
          "ZoneGraphDataHandle", "ZoneGraphTag", "ZoneShapeComponent",
          "ZoneShape", "ZoneGraphRenderingComponent"):
    log(f"  unreal.{t}: {'present' if hasattr(unreal, t) else 'absent'}")
    if hasattr(unreal, t):
        st = getattr(unreal, t)
        mem = [m for m in dir(st) if not m.startswith("_")]
        log(f"     -> {mem[:30]}")

# ZoneShapeComponents are the AUTHORING source (what the builder edits) and are
# often better exposed than the baked storage. Worth knowing if they survived.
shapes = [a for a in acts if "ZoneShape" in a.get_class().get_name()]
log(f"  actors with ZoneShape in class name: {len(shapes)}")

log("=" * 70)
log("PROBE DONE")

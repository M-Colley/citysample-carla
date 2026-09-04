#!/usr/bin/env python3
"""
carla_mass_bridge.py - check the Mass bridge: does get_actors() see the traffic?

City Sample's traffic is Epic's Mass ECS, not Actors, so CARLA's registry knew
nothing about it: the camera rendered hundreds of cars while
`world.get_actors()` returned three. Every one of those was a silent false
negative in any ground-truth label.

The bridge mirrors Mass entities into CARLA as dormant, read-only proxies.
Enable it at launch:

    powershell -File tools\\Run-CarlaCity.ps1 -ExecCmds "carla.MassBridge.Enable 1"

then:

    python tools/carla_mass_bridge.py
    python tools/carla_mass_bridge.py --seconds 20   # watch them move

Checks that the proxies exist, that their transforms actually change (a frozen
proxy is worse than none - it looks like a parked car that is not there), and
that their bounding boxes are plausible car-sized.
"""

from __future__ import annotations

import argparse
import collections
import math
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")

# Both populations: MassTraffic vehicles are mirrored as
# vehicle.mass.citysample, MassCrowd pedestrians as walker.mass.citysample.
PREFIX = ("vehicle.mass", "walker.mass")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--wait", type=float, default=45.0,
                    help="how long to wait for the first proxy to appear")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="how long to watch the proxies move")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print(f"map: {world.get_map().name}")

    # Mass spawns its entities progressively after the map loads, and the
    # processor only mirrors what exists that frame - so a check run the instant
    # the port opens legitimately sees zero. Wait for the first proxy, then let
    # the population settle rather than measuring mid-ramp.
    deadline = time.time() + args.wait
    actors = world.get_actors()
    proxies = [a for a in actors if a.type_id.startswith(PREFIX)]
    while not proxies and time.time() < deadline:
        time.sleep(1.0)
        actors = world.get_actors()
        proxies = [a for a in actors if a.type_id.startswith(PREFIX)]
    if proxies:
        time.sleep(3.0)
        actors = world.get_actors()
        proxies = [a for a in actors if a.type_id.startswith(PREFIX)]

    print(f"\n=== registry ===")
    print(f"  actors total   : {len(actors):,}")
    n_veh = sum(1 for a in proxies if a.type_id.startswith("vehicle.mass."))
    n_ped = sum(1 for a in proxies if a.type_id.startswith("walker.mass."))
    print(f"  Mass proxies   : {len(proxies):,}  ({n_veh} vehicles, {n_ped} pedestrians)")
    print(f"  types: {dict(collections.Counter(a.type_id for a in actors).most_common(5))}")
    if n_ped == 0:
        print("  NOTE: no pedestrian proxies. Epic's crowd is bridged separately -")
        print("        check carla.MassBridge.MaxWalkers is not 0.")

    if not proxies:
        print("\nNO PROXIES. In order:")
        print("  - was the server launched with -ExecCmds \"carla.MassBridge.Enable 1\"?")
        print("  - is the CarlaMassBridge plugin enabled in CitySample.uproject?")
        print("  - grep the server log for 'CarlaMassBridge'")
        return 1

    ids = [a.id for a in proxies]
    print(f"  id range       : 0x{min(ids):X} .. 0x{max(ids):X}"
          f"  ({'dormant range, good' if min(ids) >= 0x80000000 else 'NOT in the dormant range'})")

    for label, pref, lo, hi in (("vehicle", "vehicle.mass.", 1.0, 20.0),
                                ("walker ", "walker.mass.", 0.2, 2.0)):
        ex = sorted(2 * p.bounding_box.extent.x
                    for p in proxies if p.type_id.startswith(pref))
        if not ex:
            continue
        med = ex[len(ex) // 2]
        print(f"  {label} size m : min {ex[0]:.2f} median {med:.2f} max {ex[-1]:.2f}"
              f"  ({'plausible' if lo < med < hi else 'IMPLAUSIBLE - check units'})")

    print(f"\n=== motion over {args.seconds:.0f}s ===")
    first = {p.id: p.get_transform().location for p in proxies}
    t0 = time.time()
    time.sleep(args.seconds)
    moved, gone, still = 0, 0, 0
    speeds = []
    for p in proxies:
        try:
            if not p.is_alive:
                gone += 1
                continue
            loc = p.get_transform().location
            a = first.get(p.id)
            d = math.dist((a.x, a.y, a.z), (loc.x, loc.y, loc.z)) if a else 0.0
            if d > 0.5:
                moved += 1
            else:
                still += 1
            v = p.get_velocity()
            speeds.append(3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
        except RuntimeError:
            gone += 1
    print(f"  moved > 0.5 m  : {moved}")
    print(f"  stationary     : {still}")
    print(f"  deregistered   : {gone}   (LOD churn and the proxy cap both cause this)")
    if speeds:
        speeds.sort()
        print(f"  km/h           : min {speeds[0]:.0f} median {speeds[len(speeds)//2]:.0f} "
              f"max {speeds[-1]:.0f}")

    ok = moved > 0
    print(f"\n{'PASS' if ok else 'FAIL'} - "
          f"{'proxies track live Mass entities' if ok else 'no proxy moved; they may be frozen'}")
    print("\nReminder: these proxies are READ-ONLY, and CARLA's Traffic Manager")
    print("must NOT be run while the bridge is on - ALSM adopts every actor whose")
    print("type id starts with 'v', with no dormant check.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

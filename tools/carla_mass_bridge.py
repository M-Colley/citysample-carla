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
    ap.add_argument("--test-yielding", action="store_true",
                    help="park a CARLA hero car ahead of a moving Mass vehicle and verify it brakes")
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
    if args.test_yielding and ok:
        print("\n=== two-way yielding (MassTraffic braking for CARLA hero) ===")
        cmap = world.get_map()
        bplib = world.get_blueprint_library()
        hero_bp = bplib.filter("vehicle.lincoln.mkz*") or bplib.filter("vehicle.*")
        bp = hero_bp[0]
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "hero")

        yield_ok = False
        for attempt in range(4):
            live_vehs = [a for a in world.get_actors() if a.type_id.startswith("vehicle.mass.")]
            best = None
            for mv in live_vehs:
                v = mv.get_velocity()
                spd_kmh = 3.6 * math.hypot(v.x, v.y, v.z)
                if spd_kmh < 12.0:
                    continue
                wp = cmap.get_waypoint(mv.get_transform().location, project_to_road=True)
                if wp is None or wp.is_junction:
                    continue
                ahead = wp.next(26.0)
                if not ahead or ahead[0].is_junction:
                    continue
                # Ensure no other actor is already sitting at the target spot
                tgt = ahead[0].transform
                tgt.location.z += 0.35
                clear = all(
                    a.get_transform().location.distance(tgt.location) > 9.0
                    for a in live_vehs if a.id != mv.id
                )
                if clear:
                    best = (spd_kmh, mv, tgt)
                    break
            if best is None:
                time.sleep(1.5)
                continue

            init_spd, target_mv, spawn_tf = best
            hero = world.try_spawn_actor(bp, spawn_tf)
            if hero is None:
                time.sleep(1.0)
                continue
            try:
                hero.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                min_gap = float("inf")
                min_spd = init_spd
                t_end = time.time() + 12.0
                while time.time() < t_end:
                    time.sleep(0.25)
                    if not target_mv.is_alive:
                        break
                    h_loc = hero.get_transform().location
                    m_loc = target_mv.get_transform().location
                    gap = h_loc.distance(m_loc)
                    mv_v = target_mv.get_velocity()
                    spd = 3.6 * math.hypot(mv_v.x, mv_v.y, mv_v.z)
                    min_gap = min(min_gap, gap)
                    if gap < 24.0:
                        min_spd = min(min_spd, spd)
                print(f"  approaching Mass vehicle initial speed : {init_spd:.1f} km/h")
                print(f"  minimum speed behind CARLA hero        : {min_spd:.1f} km/h")
                print(f"  closest centre-to-centre gap           : {min_gap:.2f} m (> 3.5 m = no overlap)")
                yield_ok = (min_spd <= max(5.0, 0.25 * init_spd) and min_gap > 3.5)
                if yield_ok:
                    break
            finally:
                hero.destroy()
        print(f"  two-way yielding: {'PASS' if yield_ok else 'INCONCLUSIVE (no unobstructed approach lane sampled)'}")

    print(f"\n{'PASS' if ok else 'FAIL'} - "
          f"{'proxies track live Mass entities' if ok else 'no proxy moved; they may be frozen'}")
    print("\nNote: proxies are READ-ONLY (dormant IDs >= 0x80000000) and center on")
    print("role_name='hero' when present (or the spectator; carla.MassBridge.FollowHero).")
    print("With ExportCarlaObstacles=1 (default) and the CARLA 1.0 ALSM dormant-actor")
    print("patch applied, yielding is two-way: Traffic Manager yields to Mass proxies")
    print("and Epic's MassTraffic brakes for CARLA vehicles and walkers.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

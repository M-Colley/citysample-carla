#!/usr/bin/env python3
"""
carla_walkers.py - can CARLA pedestrians navigate this map?

Walker AI needs a Recast navmesh. CARLA builds one automatically for
`generate_opendrive_world`, but not for a level that already exists - so on the
City Sample there was none, and the failure was silent:
`get_random_location_from_navigation()` returned None and walkers stood still
with no error.

Build and install one first (Integrate-CarlaIntoCitySample.ps1 does the install
step for you):

    python tools/xodr_to_recast_obj.py SmallCity-signals.xodr -o SmallCity.obj
    <carla>/Build/_deps/recastnavigation-build/RecastBuilder/RecastBuilder.exe SmallCity.obj 0.3
    copy SmallCity.bin "<CitySample>\\Content\\Map\\Nav\\Small_City_LVL.bin"

then:

    python tools/carla_walkers.py --walkers 30

NOTE THE DESTINATION. `Content\\<map folder>\\Nav\\`, NOT `Saved\\Nav\\`. Two
different places in CARLA look for a navmesh and they do not agree:

  * FNavigationMesh::Load (the get_navigation_mesh RPC) checks Saved/Nav first,
  * but the client actually fetches it through get_required_files("Nav"), which
    searches Saved/ ONLY for generated OpenDRIVE worlds. For a pre-existing map
    it walks the map's content folder and nothing else.

So a .bin in Saved/Nav/ is silently ignored on a map like this one, with no log
line from either side.

Checks the three things that fail independently: the navmesh answers location
queries at all, walkers spawn on it, and their AI controllers actually move
them. A walker that spawns but never moves looks identical to no navmesh.
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--walkers", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=15.0)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    print(f"map: {world.get_map().name}")

    # 1. Does the navmesh answer at all?
    print("\n=== navmesh ===")
    samples = [world.get_random_location_from_navigation() for _ in range(20)]
    good = [s for s in samples if s is not None]
    print(f"  get_random_location_from_navigation: {len(good)}/20 returned a point")
    if not good:
        print("\nNO NAVMESH. In order:")
        print("  - is Content/<map folder>/Nav/<MapName>.bin present?")
        print("    NOT Saved/Nav/ - see the note at the top of this file. A .bin")
        print("    in Saved/Nav/ is silently ignored on a pre-existing map, so")
        print("    finding one there proves nothing.")
        print("  - is it larger than 40 bytes? 40 bytes is a header with zero")
        print("    tiles: RecastBuilder ran but found no walkable surface,")
        print("    usually a face-winding problem.")
        print("  - does the .bin name match the map name exactly?")
        print("  - does the OBJ contain `usemtl sidewalk` faces? The spawn")
        print("    filter accepts sidewalk polygons only.")
        return 1
    xs = [p.x for p in good]
    ys = [p.y for p in good]
    print(f"  spread: {max(xs)-min(xs):,.0f} x {max(ys)-min(ys):,.0f} m")

    # 2. Do walkers spawn on it?
    print(f"\n=== spawning {args.walkers} walkers ===")
    bl = world.get_blueprint_library()
    walker_bps = list(bl.filter("walker.pedestrian.*"))
    controller_bp = bl.find("controller.ai.walker")
    if not walker_bps:
        print("  no walker blueprints in the library")
        return 1

    walkers, controllers = [], []
    try:
        # The sidewalk strips are synthesised from the road network, so a few
        # land where the City Sample has a building or a parked car. Spawning
        # is a collision test, so just try another point rather than losing the
        # walker - one attempt each spawned only 57% of them.
        attempts = 0
        while len(walkers) < args.walkers and attempts < args.walkers * 8:
            attempts += 1
            loc = world.get_random_location_from_navigation()
            if loc is None:
                continue
            bp = random.choice(walker_bps)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            w = world.try_spawn_actor(bp, carla.Transform(loc))
            if w:
                walkers.append(w)
        print(f"  spawned {len(walkers)}/{args.walkers} "
              f"({attempts} placement attempts)")
        if not walkers:
            print("  nothing spawned - the navmesh points may be under the map")
            return 1

        # 3. Do the AI controllers actually move them?
        for w in walkers:
            c = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=w)
            if c:
                controllers.append(c)
        print(f"  controllers attached: {len(controllers)}")

        sync = world.get_settings().synchronous_mode

        def pump():
            """Advance the pedestrian crowd by one step.

            Client-side walker AI is simulated IN THE CLIENT: Detour runs in
            this process and only steps inside Simulator::NavigationTick, which
            is reached from world.tick() and world.wait_for_tick() and nowhere
            else. A script that sleeps instead of ticking sees every walker
            spawn correctly and then stand perfectly still - which looks exactly
            like a broken navmesh and is not.
            """
            world.tick() if sync else world.wait_for_tick()

        pump()
        started = 0
        for c in controllers:
            try:
                c.start()
                dest = world.get_random_location_from_navigation()
                if dest is not None:
                    c.go_to_location(dest)
                c.set_max_speed(1.4)
                started += 1
            except RuntimeError:
                pass
        print(f"  controllers started : {started}")

        print(f"\n=== walking for {args.seconds:.0f}s ===")
        first = {w.id: w.get_location() for w in walkers if w.is_alive}
        deadline = time.time() + args.seconds
        ticks = 0
        while time.time() < deadline:
            pump()
            ticks += 1
        print(f"  ticks pumped: {ticks}")
        moved, still, gone = 0, 0, 0
        dists = []
        for w in walkers:
            try:
                if not w.is_alive:
                    gone += 1
                    continue
                a, b = first.get(w.id), w.get_location()
                d = math.dist((a.x, a.y), (b.x, b.y)) if a else 0.0
                dists.append(d)
                if d > 1.0:
                    moved += 1
                else:
                    still += 1
            except RuntimeError:
                gone += 1
        print(f"  moved > 1 m : {moved}")
        print(f"  stationary  : {still}")
        print(f"  gone        : {gone}")
        if dists:
            dists.sort()
            print(f"  distance m  : min {dists[0]:.1f} median "
                  f"{dists[len(dists)//2]:.1f} max {dists[-1]:.1f}")

        ok = moved > 0
        if ok:
            print("\nPASS - walkers navigate the map")
        else:
            print("\nFAIL - walkers spawned but never moved. Either the crowd "
                  "is not being ticked (see pump() above - your own script "
                  "must call world.tick() or world.wait_for_tick() in a loop), "
                  "or the navmesh has sidewalk polygons that connect to "
                  "nothing.")
        return 0 if ok else 1
    finally:
        for c in controllers:
            try:
                c.stop()
            except RuntimeError:
                pass
        client.apply_batch_sync(
            [carla.command.DestroyActor(a) for a in controllers + walkers], True)


if __name__ == "__main__":
    sys.exit(main())

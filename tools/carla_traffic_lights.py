#!/usr/bin/env python3
"""
carla_traffic_lights.py - report the traffic lights CARLA built from OpenDRIVE.

    python tools/carla_traffic_lights.py
    python tools/carla_traffic_lights.py --watch 30    # watch a group cycle

Counts the traffic-light actors, the OpenDRIVE landmarks behind them, and the
junction groups, then checks the two things that are easy to get wrong and
impossible to see from a count alone:

  * every light belongs to a junction GROUP - a light with a group of 1 is not
    coordinated with anything;
  * within a group, not everything is green at once. ATrafficLightGroup cycles
    its controllers, so at any instant one phase should be green and the rest
    red. All-green means the phases were not split.
"""

from __future__ import annotations

import argparse
import collections
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
    ap.add_argument("--traffic", type=int, default=0,
                    help="spawn N cars and check they stop at red")
    ap.add_argument("--watch", type=float, default=0.0,
                    help="seconds to watch one group cycle. A full cycle here "
                         "is green 10s + yellow 3s + all-red 2s per arm, so "
                         "give it 25s or more - a shorter window can sit "
                         "inside one phase and look static when it is not")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f"map: {cmap.name}")

    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    every = list(world.get_actors().filter("traffic.*"))
    print(f"\n=== actors ===")
    print(f"  traffic.traffic_light* : {len(lights):,}")
    print(f"  traffic.*              : {len(every):,}")
    if every:
        print(f"  types: {dict(collections.Counter(a.type_id for a in every))}")

    landmarks = cmap.get_all_landmarks()
    print(f"\n=== OpenDRIVE ===")
    print(f"  landmarks              : {len(landmarks):,}")
    if landmarks:
        print(f"  types: {dict(collections.Counter(l.type for l in landmarks).most_common(5))}")

    if not lights:
        print("\nNO TRAFFIC LIGHTS. Things to check, in order:")
        print("  - <validity> is not 0/0 (CARLA deletes those references)")
        print("  - type is 1000001")
        print("  - the .xodr actually deployed to Saved/OpenDrive/<Map>.xodr")
        return 1

    groups = collections.Counter()
    for t in lights:
        try:
            groups[len(t.get_group_traffic_lights())] += 1
        except RuntimeError:
            groups[0] += 1
    print(f"\n=== grouping ===")
    print(f"  group-size histogram   : {dict(sorted(groups.items()))}")
    ungrouped = groups.get(0, 0) + groups.get(1, 0)
    print(f"  ungrouped/solo lights  : {ungrouped}  "
          f"({'ok' if ungrouped < len(lights) * 0.5 else 'SUSPICIOUS'})")

    # Junction coverage. NOT via get_waypoint on the light's own location: the
    # lights sit on the APPROACH roads, so that projection returns junction_id
    # -1 for every one of them and reads as "no junctions covered". Count the
    # distinct groups instead - ATrafficLightGroup is created per junction.
    seen_groups, n_groups = set(), 0
    for t in lights:
        try:
            ids = frozenset(x.id for x in t.get_group_traffic_lights())
        except RuntimeError:
            continue
        if ids and ids not in seen_groups:
            seen_groups.add(ids)
            n_groups += 1
    print(f"  distinct junction groups: {n_groups:,}")

    t0 = lights[0]
    grp = t0.get_group_traffic_lights()
    print(f"\n=== sample group (light {t0.id}) ===")
    print(f"  size {len(grp)}   green={t0.get_green_time():.0f}s "
          f"yellow={t0.get_yellow_time():.0f}s red={t0.get_red_time():.0f}s")
    states = collections.Counter(str(x.state) for x in grp)
    print(f"  states now: {dict(states)}")
    if len(grp) > 1:
        # A single snapshot proves nothing: every correct junction spends part
        # of its cycle all-red as a clearance interval. Only a group that NEVER
        # splits is broken, so sample across a full cycle.
        span = t0.get_green_time() + t0.get_yellow_time() + t0.get_red_time() + 2.0
        split_seen, deadline = False, time.time() + min(span, 30.0)
        while time.time() < deadline and not split_seen:
            if len({str(x.state) for x in grp}) > 1:
                split_seen = True
            time.sleep(0.25)
        if split_seen:
            print(f"  phases split correctly (observed within {span:.0f}s)")
        else:
            print("  WARNING: this group never split over a full cycle - "
                  "opposing arms would run together.")

    if args.traffic > 0:
        import random
        print(f"\n=== do vehicles obey the lights? ({args.traffic} cars) ===")
        tm = client.get_trafficmanager(8000)
        tm.set_osm_mode(True)
        bps = [b for b in world.get_blueprint_library().filter("vehicle.*")
               if b.has_attribute("number_of_wheels")
               and int(b.get_attribute("number_of_wheels")) == 4]
        spawns = cmap.get_spawn_points()
        random.shuffle(spawns)
        cars = []
        for sp in spawns[:args.traffic]:
            v = world.try_spawn_actor(random.choice(bps), sp)
            if v:
                v.set_autopilot(True, tm.get_port())
                cars.append(v)
        print(f"  spawned {len(cars)}")
        time.sleep(12.0)                      # settle: they are still falling at t=0

        stopped_at_red = moving_at_green = at_red = 0
        end = time.time() + 25.0
        while time.time() < end:
            for v in cars:
                try:
                    if not v.is_alive or not v.is_at_traffic_light():
                        continue
                    tl = v.get_traffic_light()
                    if tl is None:
                        continue
                    vel = v.get_velocity()
                    spd = 3.6 * (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
                    if str(tl.state) == "Red":
                        at_red += 1
                        if spd < 1.0:
                            stopped_at_red += 1
                    elif str(tl.state) == "Green" and spd > 1.0:
                        moving_at_green += 1
                except RuntimeError:
                    pass
            time.sleep(0.5)

        print(f"  samples of a car at a RED light   : {at_red}")
        print(f"    of those, stationary            : {stopped_at_red}"
              f"{'  (' + str(round(100.0 * stopped_at_red / at_red)) + '%)' if at_red else ''}")
        print(f"  samples of a car moving on GREEN  : {moving_at_green}")
        if at_red == 0:
            print("  no car reached a red light in the sample window - "
                  "inconclusive, try more cars or a longer run")
        # Hand the cars back before destroying them: tearing down actors the
        # traffic manager still steers crashes the client at exit
        # (STATUS_STACK_BUFFER_OVERRUN) *after* all output, so a clean-looking
        # run returns non-zero.
        for v in cars:
            try:
                if v.is_alive:
                    v.set_autopilot(False, tm.get_port())
            except RuntimeError:
                pass
        tm.set_synchronous_mode(False)
        client.apply_batch_sync([carla.command.DestroyActor(v) for v in cars], True)

    if args.watch > 0:
        print(f"\n=== watching {args.watch:.0f}s ===")
        seen = set()
        end = time.time() + args.watch
        while time.time() < end:
            snap = tuple(str(x.state)[0] for x in grp)
            if snap not in seen:
                seen.add(snap)
                print(f"  {time.strftime('%H:%M:%S')}  {' '.join(snap)}")
            time.sleep(0.5)
        if len(seen) > 1:
            print(f"  {len(seen)} distinct phase patterns observed (cycling)")
        elif args.watch < 25.0:
            # Not a failure: one arm holds green for 10s, and the whole cycle
            # runs about 30s, so a short window legitimately sees one pattern.
            print(f"  1 pattern in {args.watch:.0f}s - INCONCLUSIVE, not a "
                  f"failure. Re-run with --watch 30 to see a whole cycle.")
        else:
            print(f"  1 pattern in {args.watch:.0f}s - STATIC, the lights are "
                  f"not cycling")
    return 0


if __name__ == "__main__":
    sys.exit(main())

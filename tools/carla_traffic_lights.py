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
    ap.add_argument(
        "--traffic", type=int, default=0, help="spawn N cars and check they stop at red"
    )
    ap.add_argument(
        "--watch",
        type=float,
        default=0.0,
        help="seconds to watch one group cycle. A full cycle here "
        "is green 10s + yellow 3s + all-red 2s per arm, so "
        "give it 30s or more. Exits nonzero if no phase "
        "change was observed",
    )
    ap.add_argument(
        "--verify-sync",
        action="store_true",
        help="verify MassTraffic -> CARLA signal sync (CARLA group "
        "timers frozen while Mass drives live Red/Yellow/Green "
        "transitions), out-of-the-box RGB exposure, and zero "
        "red-light violations under light CARLA traffic",
    )
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    cmap = world.get_map()
    print(f"map: {cmap.name}")

    lights = list(world.get_actors().filter("traffic.traffic_light*"))
    every = list(world.get_actors().filter("traffic.*"))
    print("\n=== actors ===")
    print(f"  traffic.traffic_light* : {len(lights):,}")
    print(f"  traffic.*              : {len(every):,}")
    if every:
        print(f"  types: {dict(collections.Counter(a.type_id for a in every))}")

    landmarks = cmap.get_all_landmarks()
    print("\n=== OpenDRIVE ===")
    print(f"  landmarks              : {len(landmarks):,}")
    if landmarks:
        print(
            f"  types: {dict(collections.Counter(l.type for l in landmarks).most_common(5))}"
        )
    # Each crossing comes back as a closed ring of five points.
    zone_points = cmap.get_crosswalks()
    print(
        f"  crosswalk zone points  : {len(zone_points):,}  "
        f"({len(zone_points) // 5:,} crossings)"
    )

    if not lights:
        print("\nNO TRAFFIC LIGHTS. Things to check, in order:")
        print("  - <validity> is not 0/0 (CARLA deletes those references)")
        print("  - type is 1000001")
        print("  - the .xodr actually deployed to Saved/OpenDrive/<Map>.xodr")
        return 1

    # Query group membership once per distinct group instead of 2 * N_lights
    # synchronous game-thread RPCs (619 calls vs 4,352 on Big_City_LVL).
    groups = collections.Counter()
    seen_groups, n_groups = set(), 0
    visited_light_ids: set[int] = set()
    frozen_count = 0
    sample = None
    for t in lights:
        if t.id in visited_light_ids:
            continue
        try:
            members = t.get_group_traffic_lights()
        except RuntimeError:
            groups[0] += 1
            visited_light_ids.add(t.id)
            continue
        if not members:
            groups[0] += 1
            visited_light_ids.add(t.id)
            if t.is_frozen():
                frozen_count += 1
            continue
        if sample is None and len(members) > 1:
            sample = t, members
        ids = frozenset(x.id for x in members)
        visited_light_ids.update(ids)
        groups[len(members)] += len(members)
        if t.is_frozen():
            frozen_count += len(members)
        if ids not in seen_groups:
            seen_groups.add(ids)
            n_groups += 1

    print("\n=== grouping ===")
    print(f"  group-size histogram   : {dict(sorted(groups.items()))}")
    ungrouped = groups.get(0, 0) + groups.get(1, 0)
    print(
        f"  ungrouped/solo lights  : {ungrouped}  "
        f"({'ok' if ungrouped < len(lights) * 0.5 else 'SUSPICIOUS'})"
    )
    print(f"  distinct junction groups: {n_groups:,}")

    t0, grp = sample if sample is not None else (lights[0], lights[:1])
    print(f"\n=== sample group (light {t0.id}) ===")
    print(
        f"  size {len(grp)}   green={t0.get_green_time():.0f}s "
        f"yellow={t0.get_yellow_time():.0f}s red={t0.get_red_time():.0f}s "
        f"mass_synced(frozen_timer)={frozen_count}/{len(lights)}"
    )
    states = collections.Counter(str(x.state) for x in grp)
    print(f"  states now: {dict(states)}")
    if len(grp) > 1:
        # A single snapshot proves nothing: every correct junction spends part
        # of its cycle all-red as a clearance interval. Only a group that NEVER
        # splits is broken, so sample across a full cycle.
        span = max(
            t0.get_green_time() + t0.get_yellow_time() + t0.get_red_time() + 2.0, 32.0
        )
        split_seen, deadline = False, time.time() + min(span, 35.0)
        while time.time() < deadline and not split_seen:
            if len({str(x.state) for x in grp}) > 1:
                split_seen = True
            time.sleep(0.25)
        if split_seen:
            print(f"  phases split correctly (observed within {span:.0f}s)")
        else:
            print(
                "  WARNING: this group never split over a full cycle - "
                "opposing arms would run together."
            )

    if args.traffic > 0:
        import random

        print(f"\n=== do vehicles obey the lights? ({args.traffic} cars) ===")
        tm = client.get_trafficmanager(8000)
        tm.set_osm_mode(True)
        bps = [
            b
            for b in world.get_blueprint_library().filter("vehicle.*")
            if b.has_attribute("number_of_wheels")
            and int(b.get_attribute("number_of_wheels")) == 4
        ]
        spawns = cmap.get_spawn_points()
        random.shuffle(spawns)
        cars = []
        for sp in spawns[: args.traffic]:
            v = world.try_spawn_actor(random.choice(bps), sp)
            if v:
                v.set_autopilot(True, tm.get_port())
                cars.append(v)
        print(f"  spawned {len(cars)}")
        time.sleep(12.0)  # settle: they are still falling at t=0

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
                    spd = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
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
        print(
            f"    of those, stationary            : {stopped_at_red}"
            f"{'  (' + str(round(100.0 * stopped_at_red / at_red)) + '%)' if at_red else ''}"
        )
        print(f"  samples of a car moving on GREEN  : {moving_at_green}")
        if at_red == 0:
            print(
                "  no car reached a red light in the sample window - "
                "inconclusive, try more cars or a longer run"
            )
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
        end = time.monotonic() + args.watch
        while time.monotonic() < end:
            snap = tuple(str(x.state)[0] for x in grp)
            if snap not in seen:
                seen.add(snap)
                print(f"  {time.strftime('%H:%M:%S')}  {' '.join(snap)}")
            time.sleep(0.5)
        if len(seen) > 1:
            print(f"  {len(seen)} distinct phase patterns observed (cycling)")
        else:
            print(f"  no phase change observed in {args.watch:.0f}s", file=sys.stderr)
            if args.watch < 30.0:
                print("  re-run with --watch 30 or longer", file=sys.stderr)
            return 1

    if args.verify_sync:
        import queue
        import random

        import numpy as np

        print(
            "\n=== verifying MassTraffic <-> CARLA sync & light-traffic AV compliance ==="
        )
        orig_settings = world.get_settings()
        orig_sync = orig_settings.synchronous_mode
        orig_dt = orig_settings.fixed_delta_seconds

        tm = client.get_trafficmanager(8000)
        cars = []
        cam = None
        try:
            s = world.get_settings()
            s.synchronous_mode = True
            s.fixed_delta_seconds = 0.05
            world.apply_settings(s)
            tm.set_synchronous_mode(True)
            tm.set_osm_mode(True)

            # Pick spawn points close to signalised approaches near the spectator's
            # already-streamed World Partition cells so light traffic reaches
            # multiple active traffic lights quickly without unstreamed-road drops.
            spec_loc = world.get_spectator().get_location()
            ll_xy = np.array(
                [[tl.get_location().x, tl.get_location().y] for tl in lights],
                dtype=np.float64,
            )
            all_spawns = cmap.get_spawn_points()
            near_spawns = [
                sp
                for sp in all_spawns
                if (sp.location.x - spec_loc.x) ** 2 + (sp.location.y - spec_loc.y) ** 2
                <= 450.0**2
            ]
            spawns = near_spawns if len(near_spawns) >= 15 else all_spawns
            spawns.sort(
                key=lambda sp: float(
                    np.min(
                        (ll_xy[:, 0] - sp.location.x) ** 2
                        + (ll_xy[:, 1] - sp.location.y) ** 2
                    )
                )
            )

            bps = [
                b
                for b in world.get_blueprint_library().filter("vehicle.*")
                if b.has_attribute("number_of_wheels")
                and int(b.get_attribute("number_of_wheels")) == 4
            ]
            hero_bp = world.get_blueprint_library().find("vehicle.lincoln.mkz")
            if hero_bp is None:
                hero_bp = bps[0]
            if hero_bp.has_attribute("role_name"):
                hero_bp.set_attribute("role_name", "hero")

            for sp in spawns[:40]:
                if len(cars) >= 6:
                    break
                bp = hero_bp if not cars else random.choice(bps)
                v = world.try_spawn_actor(bp, sp)
                if v is not None:
                    v.set_autopilot(True, tm.get_port())
                    cars.append(v)

            if not cars:
                print("  ERROR: could not spawn test vehicles", file=sys.stderr)
                return 1

            hero = cars[0]
            cam_bp = world.get_blueprint_library().find("sensor.camera.rgb")
            cam_bp.set_attribute("image_size_x", "640")
            cam_bp.set_attribute("image_size_y", "360")
            cam_q: queue.Queue = queue.Queue()
            cam = world.spawn_actor(
                cam_bp,
                carla.Transform(carla.Location(x=1.5, z=2.2)),
                attach_to=hero,
            )
            cam.listen(cam_q.put)

            # Settle physics & initial eye adaptation (40 ticks = 2.0s) WITHOUT
            # touching weather.sun_altitude_angle!
            for _ in range(40):
                world.tick()
            while not cam_q.empty():
                latest_img = cam_q.get_nowait()
            world.tick()
            latest_img = cam_q.get(timeout=5.0)
            arr = np.frombuffer(latest_img.raw_data, dtype=np.uint8).reshape(
                (latest_img.height, latest_img.width, 4)
            )[:, :, :3]
            mean_luma = float(arr.mean())
            white_sat_pct = float((arr.min(axis=2) >= 250).mean() * 100.0)
            print(
                f"  default RGB exposure   : mean={mean_luma:.1f}/255, "
                f"white_saturated={white_sat_pct:.2f}% (no sun_altitude workaround)"
            )

            # Watch the 25 nearest lights to hero + track red-light compliance
            # over 700 ticks (35.0s simulated time).
            hero_loc0 = hero.get_location()
            nearby_lights = sorted(
                lights,
                key=lambda tl: (
                    (tl.get_location().x - hero_loc0.x) ** 2
                    + (tl.get_location().y - hero_loc0.y) ** 2
                ),
            )[:25]
            initial_states = {tl.id: str(tl.state) for tl in nearby_lights}
            transitions_seen = 0
            prev_states = dict(initial_states)

            red_box_ticks = 0
            red_stopped_ticks = 0
            green_moving_ticks = 0
            red_violations = 0
            prev_at_red: dict[int, int] = {}

            for step in range(700):
                world.tick()
                while not cam_q.empty():
                    cam_q.get_nowait()

                for tl in nearby_lights:
                    st = str(tl.state)
                    if st != prev_states[tl.id]:
                        transitions_seen += 1
                        prev_states[tl.id] = st

                for v in cars:
                    if not v.is_alive:
                        continue
                    at_tl = v.is_at_traffic_light()
                    tl = v.get_traffic_light() if at_tl else None
                    vel = v.get_velocity()
                    spd_kmh = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
                    wp = cmap.get_waypoint(v.get_location(), project_to_road=True)

                    if at_tl and tl is not None:
                        st = str(tl.state)
                        if st == "Red":
                            red_box_ticks += 1
                            if spd_kmh < 2.5:
                                red_stopped_ticks += 1
                            prev_at_red[v.id] = tl.id
                        else:
                            prev_at_red.pop(v.id, None)
                            if st == "Green" and spd_kmh > 2.0:
                                green_moving_ticks += 1
                    else:
                        prev_tl_id = prev_at_red.pop(v.id, None)
                        if prev_tl_id is not None and wp is not None and wp.is_junction:
                            # Left the trigger box into the junction: check if light was STILL Red!
                            tl_actor = world.get_actor(prev_tl_id)
                            if (
                                tl_actor is not None
                                and str(tl_actor.state) == "Red"
                                and spd_kmh > 5.0
                            ):
                                red_violations += 1

            frozen_now = sum(1 for tl in nearby_lights if tl.is_frozen())
            sync_dir = (
                "Mass -> CARLA"
                if frozen_now > 0
                else "CARLA -> Mass (DisableMassTraffic fallback or mode 2)"
            )
            print(
                f"  sync direction         : {sync_dir} ({frozen_now}/{len(nearby_lights)} CARLA group timers frozen)"
            )
            print(
                f"  live transitions       : {transitions_seen} state changes across {len(nearby_lights)} nearby lights in 35s"
            )
            print(
                f"  red-light box ticks    : {red_box_ticks} (braking/stopped: {red_stopped_ticks})"
            )
            print(f"  green-light move ticks : {green_moving_ticks}")
            print(f"  red-light violations   : {red_violations}")

            if transitions_seen == 0:
                print(
                    "  FAIL: zero traffic light transitions observed in 35s!",
                    file=sys.stderr,
                )
                return 1
            if red_violations > 0:
                print(
                    f"  FAIL: observed {red_violations} red-light violation(s)!",
                    file=sys.stderr,
                )
                return 1
            if white_sat_pct > 35.0:
                print(
                    f"  FAIL: RGB image white-saturated ({white_sat_pct:.1f}%) - competing sun still active!",
                    file=sys.stderr,
                )
                return 1
            print("  SYNC VERIFICATION PASSED.")
        finally:
            if cam is not None:
                try:
                    cam.stop()
                    cam.destroy()
                except RuntimeError:
                    pass
            for v in cars:
                try:
                    if v.is_alive:
                        v.set_autopilot(False, tm.get_port())
                except RuntimeError:
                    pass
            if cars:
                client.apply_batch_sync(
                    [carla.command.DestroyActor(v) for v in cars], True
                )
            tm.set_synchronous_mode(False)
            restore = world.get_settings()
            restore.synchronous_mode = orig_sync
            restore.fixed_delta_seconds = orig_dt
            world.apply_settings(restore)
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
carla_validate_xodr.py - build a drivable CARLA world from an OpenDRIVE file and
check that the road network actually works.

This is the decisive test for the City Sample extraction. CARLA's
``generate_opendrive_world()`` builds road meshes, lane markings and a full
routing graph from an .xodr alone - no geometry import, no custom map package,
no UE 5.8 build. If the traffic manager can route vehicles on it, the extracted
network is correct and everything that remains is a content problem.

    python tools/carla_validate_xodr.py SmallCity.xodr
    python tools/carla_validate_xodr.py SmallCity.xodr --vehicles 30 --ticks 200

Start the CARLA server first:
    D:\\carla\\CARLA_0.9.16\\CarlaUE4.exe -RenderOffScreen
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
    print("carla module not installed. Use the venv:\n"
          "  python tools/carla_validate_xodr.py <file.xodr>",
          file=sys.stderr)
    raise SystemExit(2)


def read_xodr(path: str) -> str:
    """Read the .xodr, tolerating the UTF-16 our UE exporter chain can produce."""
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            text = raw.decode(enc)
            if "<OpenDRIVE" in text:
                return text
        except (UnicodeDecodeError, UnicodeError):
            continue
    raise ValueError(f"{path} does not look like OpenDRIVE in any known encoding")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--vehicles", type=int, default=20)
    ap.add_argument("--ticks", type=int, default=150)
    ap.add_argument("--no-osm-mode", dest="osm_mode", action="store_false",
                    help="disable the traffic manager dead-end mitigation")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="client timeout; generating a large world is slow")
    args = ap.parse_args()

    xodr = read_xodr(args.xodr)
    print(f"OpenDRIVE: {args.xodr}  ({len(xodr)/1e6:.1f} MB of XML)")

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    print(f"CARLA server: {client.get_server_version()}  "
          f"(client {client.get_client_version()})")

    # Mesh generation parameters. wall_height=0 keeps the invisible boundary
    # walls off so the city reads as open road rather than a bobsleigh run.
    params = carla.OpendriveGenerationParameters(
        vertex_distance=2.0,
        max_road_length=500.0,
        wall_height=0.0,
        additional_width=0.6,
        smooth_junctions=True,
        enable_mesh_visibility=True,
    )

    print("\ngenerating world from OpenDRIVE (this takes a while for a city)...")
    t0 = time.time()
    world = client.generate_opendrive_world(xodr, params)
    print(f"  world generated in {time.time()-t0:.1f}s")

    cmap = world.get_map()
    print(f"  map name: {cmap.name}")

    # --- the road network as CARLA sees it --------------------------------
    topo = cmap.get_topology()
    spawns = cmap.get_spawn_points()
    wps = cmap.generate_waypoints(5.0)
    junction_wps = [w for w in wps if w.is_junction]
    print(f"\nroad network:")
    print(f"  topology segments : {len(topo):,}")
    print(f"  waypoints @5m     : {len(wps):,}  ({len(junction_wps):,} in junctions)")
    print(f"  spawn points      : {len(spawns):,}")

    if not spawns:
        print("\nNo spawn points. CARLA parsed the file but found no drivable "
              "lanes - the network is geometrically present but not routable.",
              file=sys.stderr)
        return 1

    # --- test 1: can a vehicle physically drive the roads? ----------------
    # Done first and without the traffic manager, because the TM is the fragile
    # part on generated networks. If this passes, the geometry and collision
    # are sound regardless of what the TM does.
    settings = world.get_settings()
    # Capture BEFORE mutating. `settings` is one mutable object, so reading
    # these back after the two assignments returns True/0.05 - the values just
    # written - and the finally block then re-asserts synchronous mode instead
    # of undoing it, leaving every server this tool touches looking hung.
    was_sync = settings.synchronous_mode
    was_dt = settings.fixed_delta_seconds
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    tm = None
    try:

        bp_lib = world.get_blueprint_library()
        probe_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        probe = None
        for sp in spawns[:40]:
            probe = world.try_spawn_actor(probe_bp, sp)
            if probe:
                break
        if probe:
            # Settle first: the spawn drop alone flings the car far enough to look
            # like driving, which produced a bogus 338 m reading earlier.
            for _ in range(20):
                world.tick()
            p0 = probe.get_location()
            ctrl = carla.VehicleControl(throttle=0.6, brake=0.0, hand_brake=False)
            for _ in range(120):
                probe.apply_control(ctrl)
                world.tick()
            p1 = probe.get_location()
            print(f"\nphysical drive test (no traffic manager):")
            print(f"  straight-line travel in 6.0 s: {p1.distance(p0):.1f} m")
            wp = cmap.get_waypoint(p1, project_to_road=True)
            if wp:
                print(f"  still on a lane: road {wp.road_id} lane {wp.lane_id} "
                      f"({wp.lane_type}), {wp.transform.location.distance(p1):.2f} m off centre")
            probe.destroy()

        # --- test 2: will the traffic manager route on it? --------------------
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True)
        # CARLA's docs warn that on generated networks "roads end abruptly at the
        # borders of the map [which] will cause the Traffic Manager to crash when
        # vehicles are not able to find the next waypoint". OSM mode makes the TM
        # despawn such vehicles instead of crashing.
        if args.osm_mode:
            tm.set_osm_mode(True)
            print("\n  traffic manager: OSM mode ON (despawn at dead ends)")

        car_bps = [b for b in bp_lib.filter("vehicle.*")
                   if b.has_attribute("number_of_wheels")
                   and int(b.get_attribute("number_of_wheels")) == 4]

        random.seed(0)
        random.shuffle(spawns)
        actors = []
        for sp in spawns[:args.vehicles]:
            v = world.try_spawn_actor(random.choice(car_bps), sp)
            if v:
                v.set_autopilot(True, tm.get_port())
                actors.append(v)
        print(f"\ntraffic:")
        print(f"  spawned {len(actors)} vehicles on autopilot")

        if not actors:
            print("  nothing spawned - spawn points exist but are unusable.", file=sys.stderr)
            return 1

        # Measure SPEED, not displacement. CARLA reuses actor ids after a despawn,
        # so comparing a live actor against start[a.id] can silently diff a new
        # vehicle against a dead one's position - that produced a bogus 486 m
        # "mean travel" that implied 175 km/h.
        for _ in range(args.ticks):
            world.tick()

        speeds, alive, lost = [], 0, 0
        for a in actors:
            try:
                if not a.is_alive:
                    lost += 1
                    continue
                alive += 1
                v = a.get_velocity()
                speeds.append(3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
            except RuntimeError:
                lost += 1

        moving = sum(1 for x in speeds if x > 1.0)
        sim_s = args.ticks * settings.fixed_delta_seconds
        limit = None
        for a in actors:
            try:
                if a.is_alive:
                    limit = a.get_speed_limit()
                    break
            except RuntimeError:
                pass

        print(f"  after {sim_s:.1f}s of simulation:")
        print(f"    still alive   : {alive}/{len(actors)}   (despawned {lost})")
        print(f"    moving        : {moving}/{alive}")
        if speeds:
            speeds_sorted = sorted(speeds)
            print(f"    speed km/h    : min {min(speeds):.0f}  "
                  f"median {speeds_sorted[len(speeds)//2]:.0f}  max {max(speeds):.0f}")
        if limit is not None:
            print(f"    speed limit   : {limit:.0f} km/h")

        for a in actors:
            try:
                if a.is_alive:
                    a.destroy()
            except RuntimeError:
                pass
    finally:
        # Restore on EVERY exit, including the `return 1` when nothing spawns
        # and any exception in between. A server left in synchronous mode with
        # no client ticking it looks completely hung to whoever connects next,
        # and the cause is three tools away from the symptom.
        try:
            if tm is not None:
                tm.set_synchronous_mode(False)
        except RuntimeError:
            pass
        settings.synchronous_mode = was_sync
        settings.fixed_delta_seconds = was_dt
        try:
            world.apply_settings(settings)
        except RuntimeError:
            pass

    ok = alive >= len(actors) * 0.6 and moving >= max(1, alive // 2)
    print(f"\nVERDICT: {'PASS' if ok else 'FAIL'} - the traffic manager "
          f"{'routes vehicles on the extracted network' if ok else 'could not drive most vehicles'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

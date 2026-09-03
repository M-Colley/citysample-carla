#!/usr/bin/env python3
"""
carla_probe.py - prove that the CARLA server is running the City Sample.

Connects to a CARLA server, reports what map it is actually serving, and saves
camera frames so the city can be seen rather than taken on trust. Without the
screenshots it is easy to believe an empty grey OpenDRIVE world is "the city".

    python tools/carla_probe.py
    python tools/carla_probe.py --shots 6 --out shots/

Exit code is 0 only if the server answered and at least one frame was captured.
"""

from __future__ import annotations

import argparse
import os
import queue
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")


def spawn_camera(world, transform, width, height, fov=90.0):
    """A camera plus a queue its frames land in, so captures are synchronous.

    Note there is nothing to configure about exposure here: CARLA 0.10's RGB
    camera exposes no exposure_* attributes at all (0.9.x did). Getting a
    correctly exposed frame is therefore a lighting problem, not a camera one -
    see the weather handling in main().
    """
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(width))
    bp.set_attribute("image_size_y", str(height))
    bp.set_attribute("fov", str(fov))
    cam = world.spawn_actor(bp, transform)
    frames: "queue.Queue" = queue.Queue()
    cam.listen(frames.put)
    return cam, frames


def drive_traffic(world, client, count):
    """Spawn autopilot vehicles and measure how many actually move.

    Rendering the city proves the geometry loaded. This proves CARLA's map -
    the road network extracted from the level's own ZoneGraph - is navigable:
    the traffic manager has to find lanes, successors and junctions in it.
    """
    import random

    spawns = world.get_map().get_spawn_points()
    random.shuffle(spawns)

    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_osm_mode(True)      # despawn at map-edge dead ends rather than error

    bps = [b for b in world.get_blueprint_library().filter("vehicle.*")
           if b.has_attribute("number_of_wheels")
           and int(b.get_attribute("number_of_wheels")) == 4]

    cars = []
    for sp in spawns[:count]:
        v = world.try_spawn_actor(random.choice(bps), sp)
        if v:
            v.set_autopilot(True, tm.get_port())
            cars.append(v)
    print(f"\n=== traffic ===")
    print(f"  spawned {len(cars)}/{count}")

    # Settle first: a vehicle dropped at a spawn point is still falling for the
    # first few ticks, and its velocity then says nothing about whether the
    # traffic manager can route it.
    for _ in range(100):
        world.tick()

    speeds = []
    for v in cars:
        try:
            if v.is_alive:
                vel = v.get_velocity()
                speeds.append(3.6 * (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5)
        except RuntimeError:
            pass

    moving = [s for s in speeds if s > 1.0]
    if speeds:
        ordered = sorted(speeds)
        print(f"  alive {len(speeds)}/{len(cars)}, moving {len(moving)}")
        print(f"  km/h: min {ordered[0]:.0f}, "
              f"median {ordered[len(ordered) // 2]:.0f}, max {ordered[-1]:.0f}")
    else:
        print("  no vehicles survived")

    # Teardown order matters. Destroying vehicles that the traffic manager is
    # still steering, while the world is still in synchronous mode, crashes the
    # client at interpreter shutdown (STATUS_STACK_BUFFER_OVERRUN, 0xC0000409) -
    # after all the output, so it looks like a clean run that returns non-zero.
    # Hand the vehicles back first, then drop the TM, then destroy in one batch.
    for v in cars:
        try:
            if v.is_alive:
                v.set_autopilot(False, tm.get_port())
        except RuntimeError:
            pass
    world.tick()

    tm.set_synchronous_mode(False)
    client.apply_batch_sync(
        [carla.command.DestroyActor(v) for v in cars], True)
    return len(moving), len(speeds)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--shots", type=int, default=4, help="camera positions to photograph")
    ap.add_argument("--out", default="shots", help="directory for the PNGs")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--carla-sun", action="store_true",
                    help="keep CARLA's own sun on (blows out a level that already has lighting)")
    ap.add_argument("--traffic", type=int, default=0,
                    help="spawn N autopilot vehicles and report whether they drive")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    print("=== server ===")
    print(f"  client {client.get_client_version()}")
    print(f"  server {client.get_server_version()}")

    world = client.get_world()
    cmap = world.get_map()
    print(f"  map    {cmap.name}")

    # CARLA drives its own directional light through the weather system. On its
    # own towns that IS the lighting. The City Sample already ships a complete
    # lighting setup, so leaving CARLA's sun on gives the level two suns: the
    # engine warns "Multiple directional lights are competing to be the single
    # one used for forward shading" and every lit surface blows out to white.
    # Putting CARLA's sun below the horizon hands the level back to its own
    # lighting, which is what you actually want when hosting City Sample.
    if not args.carla_sun:
        weather = world.get_weather()
        weather.sun_altitude_angle = -90.0
        weather.fog_density = 0.0
        world.set_weather(weather)

    settings = world.get_settings()
    # Both captures BEFORE the mutations - see carla_validate_xodr.py. Reading
    # fixed_delta_seconds back after writing it just returns 0.05, so the
    # restore would pin the next client to 20 Hz.
    was_sync = settings.synchronous_mode
    was_dt = settings.fixed_delta_seconds
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    try:

        actors = world.get_actors()
        print("\n=== world ===")
        print(f"  actors        : {len(actors):,}")
        for kind in ("vehicle", "walker", "traffic_light", "static"):
            n = len(actors.filter(f"{kind}.*"))
            if n:
                print(f"    {kind:<13} : {n:,}")

        spawns = cmap.get_spawn_points()
        print(f"  spawn points  : {len(spawns):,}")
        topo = cmap.get_topology()
        print(f"  road segments : {len(topo):,}")

        # The extent of the drivable area is the cheapest signal that we are on the
        # City Sample and not on a stock CARLA town: Small City spans ~2 km.
        if spawns:
            xs = [s.location.x for s in spawns]
            ys = [s.location.y for s in spawns]
            print(f"  extent        : {max(xs)-min(xs):,.0f} x {max(ys)-min(ys):,.0f} m")

        os.makedirs(args.out, exist_ok=True)
        saved = []

        # Photograph from a few spawn points, raised and tilted down, so buildings
        # and props are in frame rather than just tarmac.
        picks = spawns[:: max(1, len(spawns) // max(1, args.shots))][: args.shots] if spawns else []
        if not picks:
            picks = [carla.Transform(carla.Location(0, 0, 60), carla.Rotation(pitch=-30))]

        print("\n=== capturing ===")
        for i, sp in enumerate(picks):
            tf = carla.Transform(
                carla.Location(sp.location.x, sp.location.y, sp.location.z + 12.0),
                carla.Rotation(pitch=-18.0, yaw=sp.rotation.yaw))
            cam, frames = spawn_camera(world, tf, args.width, args.height)
            try:
                # Let exposure, streaming and Lumen settle, or the first frames come
                # out black or with untextured proxies.
                for _ in range(40):
                    world.tick()
                    try:
                        frames.get(timeout=5.0)
                    except queue.Empty:
                        pass
                world.tick()
                img = frames.get(timeout=20.0)
                path = os.path.join(args.out, f"citysample_{i:02d}.png")
                img.save_to_disk(path)
                saved.append(path)
                print(f"  {path}  ({img.width}x{img.height})")
            except queue.Empty:
                print(f"  shot {i}: no frame arrived", file=sys.stderr)
            finally:
                cam.stop()
                cam.destroy()

        moving = alive = 0
        if args.traffic:
            moving, alive = drive_traffic(world, client, args.traffic)

    finally:
        # Both fields, and on every exit path. Restoring synchronous_mode but
        # leaving fixed_delta_seconds at 0.05 pins the next client to 20 Hz
        # with no indication why.
        settings.synchronous_mode = was_sync
        settings.fixed_delta_seconds = was_dt
        try:
            world.apply_settings(settings)
        except RuntimeError:
            pass

    print(f"\n{len(saved)}/{len(picks)} frames captured")
    if args.traffic:
        print(f"{moving}/{alive} vehicles moving")
    return 0 if saved else 1


if __name__ == "__main__":
    sys.exit(main())

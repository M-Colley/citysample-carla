#!/usr/bin/env python3
"""
carla_show_city.py - build the extracted City Sample network in CARLA and watch
traffic drive it.

THIS REPLACES THE LOADED LEVEL. It calls generate_opendrive_world(), which
builds a fresh world of bare road ribbons from the .xodr you give it - so
pointing it at a City Sample server throws the city away and serves grey
geometry instead, with nothing on screen to say why. It is a PART A tool: it
exists to drive an exported .xodr in a stock CARLA, before any of part B.

To look around the actual City Sample, use tools/carla_probe.py or
tools/carla_spectator.py instead.

Run the CARLA server WITH a window (no -RenderOffScreen), then:

    python tools/carla_show_city.py SmallCity.xodr

The spectator camera chases a vehicle so there is something to look at without
touching the mouse. Ctrl-C to stop; it restores async mode and cleans up.

    --chase N      follow the Nth spawned vehicle (default 0)
    --top-down     static overhead view of the whole network instead of a chase
    --vehicles N   how much traffic (default 80)
"""

from __future__ import annotations

import argparse
import math
import random
import sys
import time

import carla


def read_xodr(path: str) -> str:
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            t = raw.decode(enc)
            if "<OpenDRIVE" in t:
                return t
        except (UnicodeDecodeError, UnicodeError):
            continue
    raise ValueError(f"{path} is not readable OpenDRIVE")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr", nargs="?", default="SmallCity.xodr")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--vehicles", type=int, default=80)
    ap.add_argument("--chase", type=int, default=0)
    ap.add_argument("--top-down", action="store_true")
    ap.add_argument("--minutes", type=float, default=10.0)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(180.0)
    print(f"CARLA {client.get_server_version()}")

    print("generating the city from OpenDRIVE...")
    world = client.generate_opendrive_world(
        read_xodr(args.xodr),
        carla.OpendriveGenerationParameters(
            vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
            additional_width=0.6, smooth_junctions=True,
            enable_mesh_visibility=True))
    cmap = world.get_map()
    spawns = cmap.get_spawn_points()
    print(f"  {len(spawns):,} spawn points")

    # Daylight, so it is actually visible.
    world.set_weather(carla.WeatherParameters(
        cloudiness=20.0, precipitation=0.0, sun_altitude_angle=65.0,
        fog_density=0.0))

    settings = world.get_settings()
    # Keep the ORIGINAL so teardown restores what was there, rather than
    # asserting a fixed delta the next client did not ask for.
    was_sync = settings.synchronous_mode
    was_dt = settings.fixed_delta_seconds
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    tm = client.get_trafficmanager(8000)
    tm.set_synchronous_mode(True)
    tm.set_osm_mode(True)          # despawn at map-edge dead ends rather than crash

    bp_lib = world.get_blueprint_library()
    car_bps = [b for b in bp_lib.filter("vehicle.*")
               if b.has_attribute("number_of_wheels")
               and int(b.get_attribute("number_of_wheels")) == 4]

    random.seed()
    random.shuffle(spawns)
    actors = []
    for sp in spawns[:args.vehicles]:
        v = world.try_spawn_actor(random.choice(car_bps), sp)
        if v:
            v.set_autopilot(True, tm.get_port())
            actors.append(v)
    print(f"  {len(actors)} vehicles driving")

    spectator = world.get_spectator()

    if args.top_down:
        xs = [sp.location.x for sp in spawns]
        ys = [sp.location.y for sp in spawns]
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        height = max(max(xs) - min(xs), max(ys) - min(ys)) * 0.75
        spectator.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=height),
            carla.Rotation(pitch=-90.0)))
        print(f"  overhead view from {height:.0f} m")

    print(f"\nrunning for {args.minutes:.0f} min - Ctrl-C to stop")
    deadline = time.time() + args.minutes * 60
    try:
        while time.time() < deadline:
            world.tick()
            if not args.top_down and actors:
                hero = actors[min(args.chase, len(actors) - 1)]
                try:
                    if hero.is_alive:
                        tf = hero.get_transform()
                        # Chase camera: behind and above, looking down slightly.
                        yaw = math.radians(tf.rotation.yaw)
                        back = carla.Location(
                            x=tf.location.x - 8.0 * math.cos(yaw),
                            y=tf.location.y - 8.0 * math.sin(yaw),
                            z=tf.location.z + 3.5)
                        spectator.set_transform(carla.Transform(
                            back, carla.Rotation(pitch=-12.0, yaw=tf.rotation.yaw)))
                    else:
                        actors = [a for a in actors if a.is_alive]
                except RuntimeError:
                    actors = [a for a in actors if a.is_alive]
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        # Order matters, and getting it wrong hangs or crashes the client:
        # autopilot OFF first, so the traffic manager stops issuing commands
        # for actors that are about to go; then one tick to let that land; then
        # the TM out of sync mode; then the world; and only then the batch
        # destroy. Destroying autopilot vehicles while still synchronous is the
        # failure carla_probe.py and verify_sensors.py both document.
        for a in actors:
            try:
                a.set_autopilot(False, tm.get_port())
            except RuntimeError:
                pass
        try:
            world.tick()
        except RuntimeError:
            pass
        try:
            tm.set_synchronous_mode(False)
        except RuntimeError:
            pass
        settings.synchronous_mode = was_sync
        settings.fixed_delta_seconds = was_dt
        try:
            world.apply_settings(settings)
        except RuntimeError:
            pass
        try:
            client.apply_batch_sync(
                [carla.command.DestroyActor(a) for a in actors], True)
        except RuntimeError:
            pass
        print("cleaned up")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""
carla_spectator.py - move the camera around the running City Sample.

The server runs with `-game` and CARLA's game mode, so City Sample's own WASD
bindings are gone and nothing is bound to movement: the mouse looks around, but
the camera cannot translate. The spectator is a CARLA actor, so you drive it
through the API instead.

    # fly it yourself (WASD, no extra packages - uses Windows msvcrt)
    python tools/carla_spectator.py fly

    # jump somewhere
    python tools/carla_spectator.py goto --x 500 --y 518 --z 30
    python tools/carla_spectator.py goto --spawn 1200        # to a spawn point

    # look down on the whole city
    python tools/carla_spectator.py top

    # chase traffic (spawns some if the map is empty)
    python tools/carla_spectator.py follow --vehicles 40

    # where am I?
    python tools/carla_spectator.py where

Nothing here changes world settings, so it is safe to run against a server that
something else is already driving.
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


FLY_HELP = """
  W / S      forward / back           I / K   pitch up / down
  A / D      left / right             J / L   yaw left / right
  R / F      up / down                +/-     slower / faster
  SPACE      halt                     Q / ESC quit
"""


def connect(args):
    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    return client, client.get_world()


def show(spec, settle=0.0):
    # set_transform is asynchronous: get_transform right after it returns the
    # PREVIOUS pose, which makes a successful move look like it did nothing.
    if settle:
        time.sleep(settle)
    tf = spec.get_transform()
    print(f"  location  x={tf.location.x:9.2f}  y={tf.location.y:9.2f}  z={tf.location.z:7.2f}")
    print(f"  rotation  pitch={tf.rotation.pitch:7.2f}  yaw={tf.rotation.yaw:7.2f}")


def cmd_where(args, world):
    show(world.get_spectator())
    cmap = world.get_map()
    tf = world.get_spectator().get_transform()
    wp = cmap.get_waypoint(tf.location, project_to_road=True)
    if wp:
        print(f"  nearest lane: road {wp.road_id} lane {wp.lane_id} "
              f"s={wp.s:.1f} {'(junction)' if wp.is_junction else ''}")
    return 0


def cmd_goto(args, world):
    spec = world.get_spectator()
    if args.spawn is not None:
        spawns = world.get_map().get_spawn_points()
        if not spawns:
            sys.exit("map has no spawn points")
        sp = spawns[args.spawn % len(spawns)]
        loc = carla.Location(sp.location.x, sp.location.y, sp.location.z + args.z)
        rot = carla.Rotation(pitch=args.pitch, yaw=sp.rotation.yaw)
        print(f"spawn point {args.spawn % len(spawns)} of {len(spawns)}")
    else:
        loc = carla.Location(args.x, args.y, args.z)
        rot = carla.Rotation(pitch=args.pitch, yaw=args.yaw)
    spec.set_transform(carla.Transform(loc, rot))
    show(spec, settle=0.3)
    return 0


def cmd_top(args, world):
    spawns = world.get_map().get_spawn_points()
    if not spawns:
        sys.exit("map has no spawn points")
    xs = [s.location.x for s in spawns]
    ys = [s.location.y for s in spawns]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    height = max(max(xs) - min(xs), max(ys) - min(ys)) * 0.75
    world.get_spectator().set_transform(carla.Transform(
        carla.Location(cx, cy, height), carla.Rotation(pitch=-90.0)))
    print(f"overhead from {height:.0f} m over ({cx:.0f}, {cy:.0f})")
    return 0


def cmd_follow(args, world, client):
    spec = world.get_spectator()
    vehicles = list(world.get_actors().filter("vehicle.*"))

    spawned = []
    if len(vehicles) < 1 and args.vehicles > 0:
        print(f"no vehicles present - spawning {args.vehicles}")
        tm = client.get_trafficmanager(8000)
        tm.set_osm_mode(True)
        bps = [b for b in world.get_blueprint_library().filter("vehicle.*")
               if b.has_attribute("number_of_wheels")
               and int(b.get_attribute("number_of_wheels")) == 4]
        spawns = world.get_map().get_spawn_points()
        random.shuffle(spawns)
        for sp in spawns[:args.vehicles]:
            v = world.try_spawn_actor(random.choice(bps), sp)
            if v:
                v.set_autopilot(True, tm.get_port())
                spawned.append(v)
        vehicles = spawned
        print(f"  spawned {len(vehicles)}")

    if not vehicles:
        sys.exit("no vehicles to follow")

    hero = vehicles[args.index % len(vehicles)]
    print(f"following {hero.type_id} (id {hero.id}) - Ctrl-C to stop")
    try:
        while True:
            if not hero.is_alive:
                alive = [v for v in world.get_actors().filter("vehicle.*") if v.is_alive]
                if not alive:
                    print("all vehicles gone")
                    break
                hero = alive[0]
            tf = hero.get_transform()
            yaw = math.radians(tf.rotation.yaw)
            spec.set_transform(carla.Transform(
                carla.Location(tf.location.x - args.distance * math.cos(yaw),
                               tf.location.y - args.distance * math.sin(yaw),
                               tf.location.z + args.height),
                carla.Rotation(pitch=-12.0, yaw=tf.rotation.yaw)))
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        # Only clean up what this script created.
        for v in spawned:
            try:
                if v.is_alive:
                    v.destroy()
            except RuntimeError:
                pass
    return 0


def cmd_fly(args, world):
    try:
        import msvcrt                      # Windows-only, but ships with Python
    except ImportError:
        sys.exit("fly mode needs Windows (msvcrt); use goto/follow instead")

    spec = world.get_spectator()
    speed = args.speed
    print("flying the spectator." + FLY_HELP)
    print(f"  speed {speed:.1f} m/step\n")

    while True:
        ch = msvcrt.getch()
        if ch in (b"\x00", b"\xe0"):        # arrow keys send two bytes
            msvcrt.getch()
            continue
        try:
            k = ch.decode("ascii").lower()
        except UnicodeDecodeError:
            continue
        if k in ("q", "\x1b"):
            print("bye")
            return 0

        tf = spec.get_transform()
        loc, rot = tf.location, tf.rotation
        yaw = math.radians(rot.yaw)
        pitch = math.radians(rot.pitch)
        # Forward includes pitch so "W" flies where you are looking.
        fwd = carla.Location(math.cos(yaw) * math.cos(pitch),
                             math.sin(yaw) * math.cos(pitch),
                             math.sin(pitch))
        right = carla.Location(-math.sin(yaw), math.cos(yaw), 0.0)

        if k == "w":
            loc += fwd * speed
        elif k == "s":
            loc -= fwd * speed
        elif k == "d":
            loc += right * speed
        elif k == "a":
            loc -= right * speed
        elif k == "r":
            loc.z += speed
        elif k == "f":
            loc.z -= speed
        elif k == "j":
            rot.yaw -= 5.0
        elif k == "l":
            rot.yaw += 5.0
        elif k == "i":
            rot.pitch = min(89.0, rot.pitch + 5.0)
        elif k == "k":
            rot.pitch = max(-89.0, rot.pitch - 5.0)
        elif k in ("+", "="):
            speed = min(200.0, speed * 1.5)
            print(f"  speed {speed:.1f}")
        elif k in ("-", "_"):
            speed = max(0.1, speed / 1.5)
            print(f"  speed {speed:.1f}")
        elif k == " ":
            show(spec, settle=0.2)
            continue
        else:
            continue

        spec.set_transform(carla.Transform(loc, rot))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[1],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=FLY_HELP)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=60.0)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("where", help="print the spectator transform")
    sub.add_parser("top", help="overhead view of the whole map")

    g = sub.add_parser("goto", help="teleport the spectator")
    g.add_argument("--x", type=float, default=0.0)
    g.add_argument("--y", type=float, default=0.0)
    g.add_argument("--z", type=float, default=20.0)
    g.add_argument("--pitch", type=float, default=-20.0)
    g.add_argument("--yaw", type=float, default=0.0)
    g.add_argument("--spawn", type=int, default=None,
                   help="go to spawn point N instead of x/y")

    f = sub.add_parser("follow", help="chase a vehicle")
    f.add_argument("--index", type=int, default=0)
    f.add_argument("--distance", type=float, default=8.0)
    f.add_argument("--height", type=float, default=3.5)
    f.add_argument("--vehicles", type=int, default=0,
                   help="spawn this many if the map has none")

    fl = sub.add_parser("fly", help="drive the spectator with WASD")
    fl.add_argument("--speed", type=float, default=5.0, help="metres per keypress")

    args = ap.parse_args()
    client, world = connect(args)
    print(f"CARLA {client.get_server_version()}  map {world.get_map().name}")

    if args.cmd == "where":
        return cmd_where(args, world)
    if args.cmd == "goto":
        return cmd_goto(args, world)
    if args.cmd == "top":
        return cmd_top(args, world)
    if args.cmd == "follow":
        return cmd_follow(args, world, client)
    if args.cmd == "fly":
        return cmd_fly(args, world)
    return 1


if __name__ == "__main__":
    sys.exit(main())

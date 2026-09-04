#!/usr/bin/env python3
"""
record_carla.py - record a video of CARLA driving through the City Sample.

Four synchronised sensor views in one frame, because each one demonstrates a
different thing that had to be fixed to make CARLA work on a map it did not
author:

    RGB                     the City Sample itself, rendered by CARLA
    semantic segmentation   the FastGeo tagger (98.8% -> 0.5% unlabeled)
    depth                   the camera pipeline end to end
    LiDAR, top down         the collision-channel fix (0 -> 230k points)

plus a HUD with speed, the ego's traffic light, and how many vehicles and
pedestrians are within range.

    python tools/record_carla.py --seconds 20 -o citysample-carla.mp4

Runs the world in SYNCHRONOUS mode. That is not a detail: in asynchronous mode
the four sensors deliver whenever they are ready and the panels in one frame
would be from different instants, which looks like a rendering fault. Sync mode
also means the client's tick drives Detour, so walkers actually move (see
tools/carla_walkers.py). The original settings are restored on the way out,
including after an exception - leaving a server in sync mode with no ticking
client makes it look hung to the next person who connects.
"""

from __future__ import annotations

import argparse
import math
import os
import queue
import shutil
import subprocess
import sys
import tempfile

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")

try:
    import numpy as np
except ImportError:
    sys.exit("record_carla.py needs numpy: pip install numpy")

try:
    from PIL import Image, ImageDraw
except ImportError:
    sys.exit("record_carla.py needs pillow: pip install pillow")

PANEL_W, PANEL_H = 640, 360
LIGHT_COLOUR = {"Red": (220, 60, 60), "Yellow": (230, 200, 70),
                "Green": (90, 200, 110), "Off": (150, 150, 150),
                "Unknown": (150, 150, 150), "-": (150, 150, 150)}


def to_bgra(image) -> np.ndarray:
    """A carla.Image's raw_data as an (H, W, 4) uint8 view."""
    return np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
        (image.height, image.width, 4))


def rgb_of(image, converter=None) -> np.ndarray:
    if converter is not None:
        image.convert(converter)
    # BGRA -> RGB
    return to_bgra(image)[:, :, [2, 1, 0]].copy()


def lidar_topdown(measurement, span_m=60.0) -> np.ndarray:
    """Render a LiDAR sweep as a top-down image, coloured by height."""
    pts = np.frombuffer(measurement.raw_data, dtype=np.float32)
    pts = np.reshape(pts, (-1, 4))[:, :3]
    img = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    if not len(pts):
        return img
    scale = min(PANEL_W, PANEL_H) / span_m
    # CARLA lidar is x forward, y right, z up. Put x up the screen so the view
    # matches the camera panels above it.
    u = (PANEL_W * 0.5 + pts[:, 1] * scale).astype(np.int32)
    v = (PANEL_H * 0.5 - pts[:, 0] * scale).astype(np.int32)
    keep = (u >= 0) & (u < PANEL_W) & (v >= 0) & (v < PANEL_H)
    u, v, z = u[keep], v[keep], pts[keep, 2]
    if not len(u):
        return img
    # Height ramp: ground dark blue, roof level bright.
    t = np.clip((z + 2.0) / 8.0, 0.0, 1.0)
    img[v, u, 0] = (60 + 195 * t).astype(np.uint8)
    img[v, u, 1] = (90 + 150 * t).astype(np.uint8)
    img[v, u, 2] = (200 - 60 * t).astype(np.uint8)
    return img


def label_panel(arr: np.ndarray, text: str) -> Image.Image:
    im = Image.fromarray(arr)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, PANEL_W - 1, 22], fill=(0, 0, 0))
    d.text((8, 5), text, fill=(235, 235, 235))
    return im


class RouteDriver:
    """Drive a vehicle along the OpenDRIVE network with pure pursuit.

    Not carla.Vehicle.set_autopilot(). The traffic manager reproducibly gets the
    ego destroyed on this map: from the first spawn point it dies at tick 156
    every single time, with no collision recorded, its state going straight from
    Active to PendingKill. Manual control over the same stretch survives
    indefinitely, so it is the traffic manager rather than streaming, dormancy
    or geometry - see README section 11.

    Following waypoints instead sidesteps that, and demonstrates the thing this
    whole project exists for: the route comes from the exported road network, so
    if the car drives, the OpenDRIVE is right.
    """

    def __init__(self, vehicle, carla_map, step=2.0, lookahead=6.0, target_kmh=28.0):
        self.v = vehicle
        self.map = carla_map
        self.step = step
        self.lookahead = lookahead
        self.target = target_kmh
        self.route = []
        self._extend(60)

    def _extend(self, n):
        import random
        wp = (self.route[-1] if self.route
              else self.map.get_waypoint(self.v.get_location(),
                                         project_to_road=True,
                                         lane_type=carla.LaneType.Driving))
        for _ in range(n):
            if wp is None:
                break
            nxt = wp.next(self.step)
            if not nxt:
                break
            # At a junction several successors are offered; pick one at random
            # so a long recording does not loop the same block.
            wp = nxt[0] if len(nxt) == 1 else random.choice(nxt)
            self.route.append(wp)

    def tick(self):
        loc = self.v.get_location()
        # Drop waypoints already passed, then aim at one `lookahead` ahead.
        while self.route and self.route[0].transform.location.distance(loc) < self.lookahead:
            self.route.pop(0)
        if len(self.route) < 20:
            self._extend(60)
        if not self.route:
            self.v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
            return
        aim = self.route[0].transform.location

        fwd = self.v.get_transform().get_forward_vector()
        dx, dy = aim.x - loc.x, aim.y - loc.y
        n = math.hypot(dx, dy) or 1.0
        dx, dy = dx / n, dy / n
        # Signed angle between where we point and where we want to go. The z of
        # the 2-D cross product gives the sign; atan2 with the dot gives the
        # magnitude, and is stable where a bare asin is not.
        cross = fwd.x * dy - fwd.y * dx
        dot = max(-1.0, min(1.0, fwd.x * dx + fwd.y * dy))
        steer = max(-1.0, min(1.0, math.atan2(cross, dot) * 1.6))

        vel = self.v.get_velocity()
        kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        # Ease off in corners, or it understeers wide at every junction.
        want = self.target * (1.0 - 0.55 * min(1.0, abs(steer)))
        throttle = 0.0 if kmh > want else min(0.75, 0.25 + (want - kmh) * 0.05)
        brake = 0.35 if kmh > want + 6.0 else 0.0
        self.v.apply_control(carla.VehicleControl(
            throttle=throttle, steer=steer, brake=brake))


def ego_light_state(ego) -> str:
    try:
        if not ego.is_at_traffic_light():
            return "-"
        tl = ego.get_traffic_light()
        return str(tl.get_state()).split(".")[-1] if tl else "-"
    except RuntimeError:
        return "-"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("-o", "--output", default="citysample-carla.mp4")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--vehicles", type=int, default=60)
    ap.add_argument("--walkers", type=int, default=40)
    ap.add_argument("--kmh", type=float, default=28.0,
                    help="target speed for the ego's route follower")
    ap.add_argument("--settle", type=float, default=30.0,
                    help="seconds of streaming before recording starts; the "
                         "world is unlabelled and half-loaded before this")
    ap.add_argument("--no-fix-sun", dest="fix_sun", action="store_false",
                    help="leave the weather alone; by default CARLA's own sun "
                         "is put below the horizon so it does not double up "
                         "with the one City Sample already has")
    ap.add_argument("--gif", action="store_true",
                    help="also write a smaller .gif next to the .mp4")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    bl = world.get_blueprint_library()
    tm = client.get_trafficmanager()

    original = world.get_settings()
    ego = cams = None
    sensors, vehicles, walkers, controllers = [], [], [], []
    tmpdir = tempfile.mkdtemp(prefix="carla-rec-")
    try:
        # CARLA's game mode spawns its OWN directional light on top of the one
        # City Sample already has, and two suns blow every lit surface to white.
        # Putting CARLA's below the horizon leaves City Sample's lighting alone.
        # See README section 11 and ISSUES-TO-FILE.md issue 5.
        if args.fix_sun:
            w = world.get_weather()
            w.sun_altitude_angle = -90.0
            w.cloudiness = 15.0
            w.precipitation = 0.0
            w.fog_density = 0.0
            world.set_weather(w)

        spawns = world.get_map().get_spawn_points()
        if not spawns:
            print("no spawn points - is the .xodr installed?")
            return 1

        # --- ego ---------------------------------------------------------
        # bl.filter("vehicle.*")[0] is a Sprinter van, which fills two thirds
        # of a chase camera. Prefer something low.
        import random
        random.seed(7)          # same shot every time, for A/B comparisons
        ego_bp = None
        for want in ("vehicle.tesla.model3", "vehicle.audi.a2", "vehicle.nissan.micra",
                     "vehicle.bmw.grandtourer", "vehicle.dodge.charger"):
            found = bl.filter(want)
            if found:
                ego_bp = found[0]
                break
        if ego_bp is None:
            ego_bp = bl.filter("vehicle.*")[0]
        # role_name=hero makes the ego a World Partition streaming source, so
        # the city loads around it rather than around the spectator.
        if ego_bp.has_attribute("role_name"):
            ego_bp.set_attribute("role_name", "hero")
        ego_spawn = None
        for sp in random.sample(spawns, len(spawns)):
            ego = world.try_spawn_actor(ego_bp, sp)
            if ego:
                ego_spawn = sp
                break
        if ego is None:
            print("could not spawn an ego")
            return 1
        print(f"ego: {ego.type_id}")

        # --- traffic -----------------------------------------------------
        veh_bps = [b for b in bl.filter("vehicle.*") if b.id != ego_bp.id] or [ego_bp]
        # Keep CARLA traffic away from the ego's own spawn. Spawn points sit in
        # clusters, and dropping a dozen cars around a stationary ego produces a
        # pile-up in the first second - the recording then opens on a jam, with
        # a flipped car in shot. The city is already full of Epic's Mass traffic
        # anyway; these are the ones CARLA controls.
        far = [sp for sp in spawns
               if ego_spawn is None
               or sp.location.distance(ego_spawn.location) > 120.0]
        for sp in random.sample(far, min(args.vehicles, len(far))):
            v = world.try_spawn_actor(random.choice(veh_bps), sp)
            if v:
                vehicles.append(v)
        print(f"traffic: {len(vehicles)} vehicles")

        walker_bps = list(bl.filter("walker.pedestrian.*"))
        controller_bp = bl.find("controller.ai.walker")
        attempts = 0
        while len(walkers) < args.walkers and attempts < args.walkers * 8:
            attempts += 1
            loc = world.get_random_location_from_navigation()
            if loc is None:
                break
            w = world.try_spawn_actor(random.choice(walker_bps),
                                      carla.Transform(loc))
            if w:
                walkers.append(w)
        for w in walkers:
            c = world.try_spawn_actor(controller_bp, carla.Transform(), attach_to=w)
            if c:
                controllers.append(c)
        print(f"pedestrians: {len(walkers)} walkers, {len(controllers)} controllers")

        # --- synchronous mode -------------------------------------------
        # Set this AFTER spawning: try_spawn_actor in sync mode needs a tick to
        # confirm, so spawning a crowd is far slower with it on.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / args.fps
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)

        driver = RouteDriver(ego, world.get_map(), target_kmh=args.kmh)
        for v in vehicles:
            v.set_autopilot(True, tm.get_port())
        for c in controllers:
            try:
                c.start()
                dest = world.get_random_location_from_navigation()
                if dest is not None:
                    c.go_to_location(dest)
                c.set_max_speed(1.4)
            except RuntimeError:
                pass

        # --- sensors -----------------------------------------------------
        cam_tf = carla.Transform(carla.Location(x=-6.5, z=3.1),
                                 carla.Rotation(pitch=-13.0))

        def camera(kind):
            bp = bl.find(f"sensor.camera.{kind}")
            bp.set_attribute("image_size_x", str(PANEL_W))
            bp.set_attribute("image_size_y", str(PANEL_H))
            bp.set_attribute("fov", "90")
            bp.set_attribute("sensor_tick", str(1.0 / args.fps))
            return world.spawn_actor(bp, cam_tf, attach_to=ego)

        rgb = camera("rgb")
        seg = camera("semantic_segmentation")
        dep = camera("depth")

        lidar_bp = bl.find("sensor.lidar.ray_cast")
        for k, v in (("range", "60"), ("rotation_frequency", str(args.fps)),
                     ("points_per_second", "560000"), ("channels", "64"),
                     ("upper_fov", "10"), ("lower_fov", "-25"),
                     ("sensor_tick", str(1.0 / args.fps))):
            if lidar_bp.has_attribute(k):
                lidar_bp.set_attribute(k, v)
        lid = world.spawn_actor(
            lidar_bp, carla.Transform(carla.Location(z=2.4)), attach_to=ego)

        sensors = [rgb, seg, dep, lid]
        queues = {s.id: queue.Queue() for s in sensors}
        for s in sensors:
            s.listen(queues[s.id].put)

        # --- settle ------------------------------------------------------
        print(f"\nstreaming in for {args.settle:.0f}s before recording")
        spectator = world.get_spectator()
        for _ in range(int(args.settle * args.fps)):
            world.tick()
            if ego.is_alive:
                driver.tick()
                # Streaming follows the spectator, so park it on the ego from
                # the very first tick - otherwise the settle loads the world
                # around wherever the spectator happens to be instead.
                tf = ego.get_transform()
                spectator.set_transform(carla.Transform(
                    tf.location + carla.Location(z=4.0),
                    carla.Rotation(pitch=-14.0, yaw=tf.rotation.yaw)))
            for q in queues.values():          # drain, do not accumulate
                while not q.empty():
                    q.get_nowait()
        if not ego.is_alive:
            print("the ego did not survive the settle - nothing to record")
            return 1

        # --- record ------------------------------------------------------
        total = int(args.seconds * args.fps)
        print(f"recording {total} frames at {args.fps} fps")
        written = 0
        near_v = near_w = 0
        for n in range(total):
            world.tick()

            # Keep the spectator on the ego. Two reasons, and the second one is
            # not cosmetic: World Partition streams around the spectator, so an
            # ego that drives away from a parked spectator leaves the loaded
            # region - and then drops out of the episode snapshot, at which
            # point the client reports it destroyed and every ego call raises.
            # Following it keeps the geometry loaded where the sensors point.
            if not ego.is_alive:
                print(f"  frame {n}: the ego is gone, stopping early")
                break
            driver.tick()
            tf = ego.get_transform()
            fwd = tf.get_forward_vector()
            spectator.set_transform(carla.Transform(
                tf.location + carla.Location(x=-8.0 * fwd.x, y=-8.0 * fwd.y, z=4.0),
                carla.Rotation(pitch=-14.0, yaw=tf.rotation.yaw)))

            try:
                # Every sensor has sensor_tick == the world step, so each tick
                # yields exactly one frame from each. Take them in lockstep so
                # the four panels are the same instant.
                f_rgb = queues[rgb.id].get(timeout=20.0)
                f_seg = queues[seg.id].get(timeout=20.0)
                f_dep = queues[dep.id].get(timeout=20.0)
                f_lid = queues[lid.id].get(timeout=20.0)
            except queue.Empty:
                print(f"  frame {n}: a sensor did not deliver, stopping early")
                break

            panels = [
                label_panel(rgb_of(f_rgb), "sensor.camera.rgb"),
                label_panel(rgb_of(f_seg, carla.ColorConverter.CityScapesPalette),
                            "sensor.camera.semantic_segmentation"),
                label_panel(rgb_of(f_dep, carla.ColorConverter.LogarithmicDepth),
                            "sensor.camera.depth"),
                label_panel(lidar_topdown(f_lid),
                            "sensor.lidar.ray_cast  (top down, 60 m)"),
            ]
            sheet = Image.new("RGB", (PANEL_W * 2, PANEL_H * 2 + 34), (12, 12, 14))
            for i, p in enumerate(panels):
                sheet.paste(p, ((i % 2) * PANEL_W, (i // 2) * PANEL_H))

            vel = ego.get_velocity()
            kmh = 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
            light = ego_light_state(ego)
            loc = ego.get_location()
            # Count from the world, not from our own spawn lists: the traffic
            # manager destroys some of what we spawned (issue 9), and with the
            # Mass bridge on the interesting vehicles are Epic's, which we never
            # spawned at all.
            if n % 5 == 0:
                snap = world.get_actors()
                near_v = sum(1 for v in snap.filter("vehicle.*")
                             if v.get_location().distance(loc) < 80.0)
                near_w = sum(1 for w in snap.filter("walker.*")
                             if w.get_location().distance(loc) < 80.0)

            d = ImageDraw.Draw(sheet)
            y = PANEL_H * 2 + 9
            # Say WHOSE count this is: what the CARLA API can see, which is
            # not automatically what is on screen. With the Mass bridge on,
            # Epic's traffic AND crowd are both mirrored in, so these track the
            # city; with it off they count only what this script spawned.
            d.text((10, y),
                   f"Epic City Sample  served by CARLA 0.10   "
                   f"{kmh:5.1f} km/h   "
                   f"get_actors() within 80 m: {near_v:3d} vehicles, "
                   f"{near_w:3d} walkers   frame {n + 1}/{total}",
                   fill=(210, 210, 215))
            d.text((sheet.width - 150, y), f"traffic light: {light}",
                   fill=LIGHT_COLOUR.get(light, (150, 150, 150)))

            sheet.save(os.path.join(tmpdir, f"f{n:05d}.png"))
            written += 1
            if (n + 1) % (args.fps * 2) == 0:
                print(f"  {n + 1}/{total}")

        if not written:
            print("no frames recorded")
            return 1

        # --- encode ------------------------------------------------------
        print(f"\nencoding {written} frames -> {args.output}")
        cmd = ["ffmpeg", "-y", "-loglevel", "error",
               "-framerate", str(args.fps),
               "-i", os.path.join(tmpdir, "f%05d.png"),
               # yuv420p + even dimensions, or the file plays nowhere.
               "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2",
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
               args.output]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print("ffmpeg failed:\n" + (r.stderr or "")[-2000:])
            return 1
        print(f"  {args.output}  {os.path.getsize(args.output) / 1e6:.1f} MB")

        if args.gif:
            gif = os.path.splitext(args.output)[0] + ".gif"
            print(f"encoding {gif}")
            # Two passes: a palette generated from the whole clip avoids the
            # dithering swim you get from per-frame palettes.
            pal = os.path.join(tmpdir, "pal.png")
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error",
                            "-i", args.output, "-vf",
                            "fps=10,scale=800:-1:flags=lanczos,palettegen",
                            pal], check=True)
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error",
                            "-i", args.output, "-i", pal, "-lavfi",
                            "fps=10,scale=800:-1:flags=lanczos[x];[x][1:v]paletteuse",
                            gif], check=True)
            print(f"  {gif}  {os.path.getsize(gif) / 1e6:.1f} MB")
        return 0
    finally:
        print("\ncleaning up")
        for s in sensors:
            try:
                s.stop()
            except RuntimeError:
                pass
        for c in controllers:
            try:
                c.stop()
            except RuntimeError:
                pass
        # Autopilot off before the traffic manager goes async, or it keeps
        # issuing commands for actors that are being destroyed.
        for v in vehicles:
            try:
                v.set_autopilot(False, tm.get_port())
            except RuntimeError:
                pass
        try:
            tm.set_synchronous_mode(False)
        except RuntimeError:
            pass
        try:
            world.apply_settings(original)
        except RuntimeError:
            pass
        try:
            client.apply_batch_sync(
                [carla.command.DestroyActor(a)
                 for a in sensors + controllers + walkers + vehicles
                 + ([ego] if ego else [])], True)
        except RuntimeError:
            pass
        shutil.rmtree(tmpdir, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())

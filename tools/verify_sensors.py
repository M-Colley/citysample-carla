#!/usr/bin/env python3
"""
verify_sensors.py - prove CARLA's ray-cast sensors actually see the world.

Before the collision-channel fix every ray-cast sensor delivered frames and zero
points, because CARLA hardcodes ECC_GameTraceChannel2 and that slot means
"BallisticLandingTarget" (DefaultResponse=Ignore) in the City Sample project.

The pass criteria here are deliberately not "points > 0". With SensorTrace
defaulting to Block, a lidar bolted to a car returns tens of thousands of hits
off its own bodywork while every piece of world geometry still ignores the
channel. So we check:

  * an UNPARENTED lidar 10 m above the road returns points          <- the real test
  * the returned distances are spread, not collapsed into a shell
  * semantic lidar reports object ids/tags other than the ego
  * radar returns detections
  * ground_projection / cast_ray (URayTracer) come back non-empty

The ego is spawned with role_name='hero' on purpose: CarlaEpisode gives a hero a
3 km World Partition streaming ring and everything else 200 m, so a non-hero ego
measures the streaming radius rather than the collision channel.

    python tools/verify_sensors.py
"""

from __future__ import annotations

import argparse
import collections
import math
import queue
import sys

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")


def collect(world, sensor, n_frames, timeout=10.0, budget=90.0):
    """Tick until n_frames arrive, but never block longer than `budget` seconds.

    Without the overall budget a sensor that never delivers costs
    n_frames * timeout — and if the server dies mid-run the script hangs instead
    of reporting it.
    """
    import time as _t
    q: "queue.Queue" = queue.Queue()
    sensor.listen(q.put)
    got = []
    deadline = _t.time() + budget
    for _ in range(n_frames):
        if _t.time() > deadline:
            print(f"    (budget reached after {len(got)} frames)", file=sys.stderr)
            break
        try:
            world.tick()
        except RuntimeError as exc:
            print(f"    (server stopped responding: {exc})", file=sys.stderr)
            break
        try:
            got.append(q.get(timeout=timeout))
        except queue.Empty:
            pass
    sensor.stop()
    return got


def lidar_stats(frames):
    """Distances of every point, so a self-hit shell is distinguishable."""
    dists = []
    tags = collections.Counter()
    ids = set()
    for f in frames:
        for d in f:
            p = d.point
            dists.append(math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z))
            tag = getattr(d, "object_tag", None)
            if tag is not None:
                tags[tag] += 1
                ids.add(getattr(d, "object_idx", -1))
    return dists, tags, ids


def describe(name, dists):
    if not dists:
        print(f"  {name:<26} 0 points   <-- FAIL")
        return False
    dists.sort()
    n = len(dists)
    beyond5 = sum(1 for d in dists if d > 5.0) / n
    print(f"  {name:<26} {n:>8,} points   "
          f"min {dists[0]:5.1f}  median {dists[n // 2]:6.1f}  max {dists[-1]:7.1f} m   "
          f"{beyond5 * 100:5.1f}% beyond 5 m")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    bl = world.get_blueprint_library()
    cmap = world.get_map()

    settings = world.get_settings()
    was_sync, was_dt = settings.synchronous_mode, settings.fixed_delta_seconds
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    spawned = []
    ok = {}
    try:
        sp = cmap.get_spawn_points()[0]

        # Hero: 3 km streaming ring instead of 200 m.
        vbp = [b for b in bl.filter("vehicle.*")
               if b.has_attribute("number_of_wheels")
               and int(b.get_attribute("number_of_wheels")) == 4][0]
        if vbp.has_attribute("role_name"):
            vbp.set_attribute("role_name", "hero")
        ego = world.try_spawn_actor(vbp, sp)
        if ego:
            spawned.append(ego)
        print(f"ego: {ego.type_id if ego else 'FAILED'}  at "
              f"({sp.location.x:.0f}, {sp.location.y:.0f})")
        for _ in range(40):
            world.tick()

        print("\n=== semantic lidar ===")
        lbp = bl.find("sensor.lidar.ray_cast_semantic")
        lbp.set_attribute("range", "100")
        lbp.set_attribute("channels", "64")
        lbp.set_attribute("points_per_second", "500000")
        lbp.set_attribute("rotation_frequency", "20")
        lbp.set_attribute("upper_fov", "10")
        lbp.set_attribute("lower_fov", "-30")

        # THE control: unparented, 10 m up. Nothing of the ego can be hit, so any
        # point at all proves world geometry now blocks SensorTrace.
        free_tf = carla.Transform(
            carla.Location(sp.location.x, sp.location.y, sp.location.z + 10.0))
        free = world.spawn_actor(lbp, free_tf)
        spawned.append(free)
        fd, ftags, fids = lidar_stats(collect(world, free, 12))
        ok["lidar_world"] = describe("unparented (control)", fd)
        if ftags:
            print(f"    tags: {dict(ftags.most_common(8))}")
            print(f"    distinct object ids: {len(fids)}")

        # Attached, to confirm the ego-ignore actually suppresses self-hits.
        att = world.spawn_actor(lbp, carla.Transform(carla.Location(z=2.4)),
                                attach_to=ego) if ego else None
        if att:
            spawned.append(att)
            ad, atags, aids = lidar_stats(collect(world, att, 12))
            ok["lidar_ego"] = describe("attached to ego", ad)

        print("\n=== radar ===")
        rbp = bl.find("sensor.other.radar")
        rbp.set_attribute("range", "100")
        rbp.set_attribute("points_per_second", "5000")
        radar = world.spawn_actor(rbp, free_tf)
        spawned.append(radar)
        rframes = collect(world, radar, 12)
        rdet = sum(len(f) for f in rframes)
        print(f"  {'detections':<26} {rdet:>8,}")
        ok["radar"] = rdet > 0

        print("\n=== URayTracer (ground_projection / cast_ray) ===")
        probe = carla.Location(sp.location.x, sp.location.y, sp.location.z + 30.0)
        gp = world.ground_projection(probe, 100.0)
        print(f"  ground_projection          {'None -- FAIL' if gp is None else gp.location}")
        ok["ground_projection"] = gp is not None

        a = carla.Location(sp.location.x, sp.location.y, sp.location.z + 2.0)
        b = carla.Location(a.x + 150.0, a.y, a.z)
        hits = world.cast_ray(a, b)
        print(f"  cast_ray over 150 m        {len(hits):>8,} hits")
        ok["cast_ray"] = len(hits) > 1

        print("\n=== semantic segmentation encoding ===")
        cbp = bl.find("sensor.camera.semantic_segmentation")
        cbp.set_attribute("image_size_x", "400")
        cbp.set_attribute("image_size_y", "300")
        cam = world.spawn_actor(cbp, carla.Transform(
            carla.Location(sp.location.x, sp.location.y, sp.location.z + 8.0),
            carla.Rotation(pitch=-20.0)))
        spawned.append(cam)
        imgs = collect(world, cam, 25)
        if imgs:
            raw = bytes(imgs[-1].raw_data)
            r = raw[2::4]                      # BGRA -> R channel holds the label
            hist = collections.Counter(r)
            over = sum(v for k, v in hist.items() if k > 29) / len(r)
            print(f"  distinct label values      {len(hist):>8}   (valid range is 0..29)")
            print(f"  pixels with R > 29         {over * 100:>7.2f}%   (was 8.7-100%)")
            print(f"  top labels: {dict(hist.most_common(6))}")
            ok["segmentation"] = over == 0.0 and len(hist) <= 30
        else:
            print("  no frame")
            ok["segmentation"] = False
    finally:
        for a_ in spawned:
            try:
                if a_.is_alive:
                    a_.destroy()
            except RuntimeError:
                pass
        settings.synchronous_mode = was_sync
        settings.fixed_delta_seconds = was_dt
        world.apply_settings(settings)

    print("\n=== summary ===")
    for k, v in ok.items():
        print(f"  {k:<20} {'PASS' if v else 'FAIL'}")
    return 0 if all(ok.values()) else 1


if __name__ == "__main__":
    sys.exit(main())

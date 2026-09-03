#!/usr/bin/env python3
"""
segmentation_census.py - what is still Unlabeled in the semantic camera?

`verify_sensors.py` checks that segmentation is *encoded* correctly. This asks a
different question: of the pixels that come back, how many carry a label, and
what are the unlabeled ones?

The answer depends on streaming. World Partition loads FastGeo containers around
the ego, and CarlaFastGeoTagger can only tag what is loaded - so a frame grabbed
straight after spawning shows far more Unlabeled than the same view thirty
seconds later. Measuring before the world has settled is how you get a scary
number that means nothing.

    python tools/segmentation_census.py --settle 45
"""

from __future__ import annotations

import argparse
import queue
import sys
import time

try:
    import carla
except ImportError:
    sys.exit("carla is not importable - build the Python API first")

# carla.CityObjectLabel, for readable output.
LABELS = {
    0: "Unlabeled", 1: "Roads", 2: "Sidewalks", 3: "Buildings", 4: "Walls",
    5: "Fences", 6: "Poles", 7: "TrafficLight", 8: "TrafficSigns",
    9: "Vegetation", 10: "Terrain", 11: "Sky", 12: "Pedestrians", 13: "Rider",
    14: "Car", 15: "Truck", 16: "Bus", 17: "Train", 18: "Motorcycle",
    19: "Bicycle", 20: "Static", 21: "Dynamic", 22: "Other", 23: "Water",
    24: "RoadLines", 25: "Ground", 26: "Bridge", 27: "RailTrack",
    28: "GuardRail", 29: "Rock",
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--timeout", type=float, default=600.0)
    ap.add_argument("--settle", type=float, default=45.0,
                    help="seconds to let World Partition stream in around the "
                         "ego before measuring (default 45)")
    ap.add_argument("--save", metavar="PREFIX", default=None,
                    help="also write PREFIX-rgb.png and PREFIX-seg.png from "
                         "the same viewpoint - a histogram says how much is "
                         "unlabeled, the pair says WHAT is")
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    bl = world.get_blueprint_library()

    spawns = world.get_map().get_spawn_points()
    if not spawns:
        print("no spawn points - is the .xodr installed?")
        return 1

    ego, cam = None, None
    try:
        ego_bp = bl.filter("vehicle.*")[0]
        for sp in spawns:
            ego = world.try_spawn_actor(ego_bp, sp)
            if ego:
                break
        if ego is None:
            print("could not spawn an ego anywhere")
            return 1
        print(f"ego: {ego.type_id} at "
              f"({ego.get_location().x:.0f}, {ego.get_location().y:.0f})")

        print(f"\nletting the world stream in for {args.settle:.0f}s "
              "(the number is meaningless before this settles)")
        deadline = time.time() + args.settle
        while time.time() < deadline:
            world.wait_for_tick()

        cam_bp = bl.find("sensor.camera.semantic_segmentation")
        cam_bp.set_attribute("image_size_x", "800")
        cam_bp.set_attribute("image_size_y", "600")
        images = queue.Queue()
        cam = world.spawn_actor(
            cam_bp,
            carla.Transform(carla.Location(x=1.5, z=2.4)),
            attach_to=ego)
        cam.listen(images.put)

        try:
            image = images.get(timeout=30.0)
        except queue.Empty:
            print("no segmentation frame arrived in 30s")
            return 1

        # Raw BGRA; the label is the red channel.
        raw = bytes(image.raw_data)
        hist = {}
        for i in range(2, len(raw), 4):
            v = raw[i]
            hist[v] = hist.get(v, 0) + 1
        total = sum(hist.values())

        print(f"\n=== {image.width}x{image.height}, {total:,} pixels ===")
        for v, n in sorted(hist.items(), key=lambda kv: -kv[1]):
            name = LABELS.get(v, f"INVALID({v})")
            print(f"  {v:3d} {name:14s} {n:9,d}   {n * 100.0 / total:5.1f}%")

        unlabeled = hist.get(0, 0)
        print(f"\nUnlabeled: {unlabeled * 100.0 / total:.1f}%")
        print(f"distinct labels: {len(hist)}")
        out_of_range = sum(n for v, n in hist.items() if v > 29)
        bad = out_of_range * 100.0 / total
        if bad > 1.0:
            print(f"\nOUT OF RANGE (>29): {bad:.1f}% of pixels - NOT A VALID "
                  "MEASUREMENT.")
            print("Untagged FastGeo geometry renders with whatever City Sample")
            print("left in float 5 of its custom primitive data, and the")
            print("segmentation shader reads that as a label - so an untagged")
            print("primitive shows up as a nonsense label, not as Unlabeled.")
            print("\nThis is expected on the FIRST measurement after the server")
            print("starts, while the world is still streaming and shader")
            print("proxies are still warming. Just run it again: the second and")
            print("later runs settle at well under 1% Unlabeled.")
            print("\nIf it persists across runs, the tagger is not reaching the")
            print("geometry - check that carla.FastGeoTagger.Enable is 1 and")
            print("look for 'CarlaFastGeoTagger sweep' in the server log.")
            return 1
        if out_of_range:
            print(f"out of range (>29): {bad:.2f}%  (a few stragglers still "
                  "streaming in)")

        if args.save:
            image.save_to_disk(f"{args.save}-seg.png",
                               carla.ColorConverter.CityScapesPalette)
            rgb_bp = bl.find("sensor.camera.rgb")
            rgb_bp.set_attribute("image_size_x", str(image.width))
            rgb_bp.set_attribute("image_size_y", str(image.height))
            rgbs = queue.Queue()
            rgb = world.spawn_actor(
                rgb_bp, carla.Transform(carla.Location(x=1.5, z=2.4)),
                attach_to=ego)
            try:
                rgb.listen(rgbs.put)
                rgbs.get(timeout=30.0).save_to_disk(f"{args.save}-rgb.png")
                print(f"\nwrote {args.save}-rgb.png and {args.save}-seg.png")
            except queue.Empty:
                print("\nno RGB frame arrived; wrote the segmentation only")
            finally:
                rgb.stop()
                rgb.destroy()
        return 0
    finally:
        if cam is not None:
            cam.stop()
        client.apply_batch_sync(
            [carla.command.DestroyActor(a) for a in (cam, ego) if a], True)


if __name__ == "__main__":
    sys.exit(main())

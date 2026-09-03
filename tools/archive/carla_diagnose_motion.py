#!/usr/bin/env python3
"""
carla_diagnose_motion.py - is the vehicle DRIVING, or falling?

The validation run reported 338 m of travel in 6 s from a standing start, which
is ~200 km/h and impossible for a Model 3 at throttle 0.5. Either the network is
mis-scaled, or the car is not on the road at all and the "distance" is a fall.
This separates horizontal from vertical motion and prints the actual velocity so
the answer is unambiguous.
"""

from __future__ import annotations

import sys
import carla


def main() -> int:
    xodr_path = sys.argv[1] if len(sys.argv) > 1 else "SmallCity.xodr"
    with open(xodr_path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            xodr = raw.decode(enc)
            if "<OpenDRIVE" in xodr:
                break
        except UnicodeDecodeError:
            continue

    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(180.0)
    world = client.generate_opendrive_world(
        xodr, carla.OpendriveGenerationParameters(
            vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
            additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True))
    cmap = world.get_map()

    s = world.get_settings()
    s.synchronous_mode = True
    s.fixed_delta_seconds = 0.05
    world.apply_settings(s)

    spawns = cmap.get_spawn_points()
    bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]

    v = None
    for sp in spawns[:40]:
        v = world.try_spawn_actor(bp, sp)
        if v:
            break
    if not v:
        print("could not spawn", file=sys.stderr)
        return 1

    # Let it settle so we measure driving, not the spawn drop.
    for _ in range(20):
        world.tick()

    p0 = v.get_location()
    print(f"start: x={p0.x:.1f} y={p0.y:.1f} z={p0.z:.1f}")
    print()
    print(f"{'t(s)':>5} {'horiz(m)':>9} {'dz(m)':>8} {'speed(km/h)':>12} {'on-lane':>8}")

    # Re-apply every tick. In synchronous mode a single apply_control can be
    # superseded before it takes effect, and the vehicle just sits there.
    ctrl = carla.VehicleControl(throttle=0.6, steer=0.0, brake=0.0, hand_brake=False)
    for i in range(1, 7):
        for _ in range(20):          # 20 ticks = 1.0 s
            v.apply_control(ctrl)
            world.tick()
        p = v.get_location()
        vel = v.get_velocity()
        speed = 3.6 * (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
        horiz = ((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2) ** 0.5
        wp = cmap.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Driving)
        off = wp.transform.location.distance(p) if wp else float("nan")
        print(f"{i:>5} {horiz:>9.1f} {p.z-p0.z:>8.1f} {speed:>12.1f} {off:>8.2f}")

    p = v.get_location()
    print()
    print(f"end:   x={p.x:.1f} y={p.y:.1f} z={p.z:.1f}")
    dz = p.z - p0.z
    horiz = ((p.x - p0.x) ** 2 + (p.y - p0.y) ** 2) ** 0.5
    if abs(dz) > horiz * 0.5:
        print("VERDICT: the vehicle is FALLING - vertical motion dominates. "
              "The road surface is not where the spawn points are.")
    elif horiz > 200:
        print("VERDICT: horizontal motion is real but implausibly fast - "
              "suspect a SCALE error in the OpenDRIVE.")
    else:
        print("VERDICT: normal driving.")

    v.destroy()
    s.synchronous_mode = False
    world.apply_settings(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())

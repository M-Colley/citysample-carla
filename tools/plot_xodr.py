#!/usr/bin/env python3
"""
plot_xodr.py - render an OpenDRIVE road network top-down as a PNG.

The fastest way to see whether an extracted network actually looks like a city.
Evaluates each road's paramPoly3 geometry rather than just plotting the segment
origins, so curves show as curves.

    python tools/plot_xodr.py SmallCity.xodr -o SmallCity.png
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET


def read_root(path: str):
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            return ET.fromstring(raw.decode(enc))
        except (UnicodeDecodeError, ET.ParseError):
            continue
    raise ValueError(f"cannot parse {path}")


def road_points(road: ET.Element, step: float = 0.2):
    """Sample a road's reference line in world coordinates."""
    xs, ys = [], []
    for g in road.findall("planView/geometry"):
        x0, y0 = float(g.get("x")), float(g.get("y"))
        hdg = float(g.get("hdg"))
        cos_h, sin_h = math.cos(hdg), math.sin(hdg)
        pp = g.find("paramPoly3")
        if pp is None:
            xs.append(x0)
            ys.append(y0)
            continue
        bu, cu, du = (float(pp.get(k)) for k in ("bU", "cU", "dU"))
        bv, cv, dv = (float(pp.get(k)) for k in ("bV", "cV", "dV"))
        t = 0.0
        while t <= 1.0 + 1e-9:
            u = bu * t + cu * t * t + du * t ** 3
            v = bv * t + cv * t * t + dv * t ** 3
            xs.append(x0 + u * cos_h - v * sin_h)
            ys.append(y0 + u * sin_h + v * cos_h)
            t += step
    return xs, ys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--dpi", type=int, default=200)
    args = ap.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("needs matplotlib:  pip install matplotlib", file=sys.stderr)
        return 2

    root = read_root(args.xodr)
    roads = root.findall("road")
    out = args.output or args.xodr.rsplit(".", 1)[0] + ".png"

    fig, ax = plt.subplots(figsize=(14, 14))
    fig.patch.set_facecolor("#12161a")
    ax.set_facecolor("#12161a")

    n_ord = n_jn = 0
    total_len = 0.0
    for r in roads:
        xs, ys = road_points(r)
        if len(xs) < 2:
            continue
        total_len += float(r.get("length", 0))
        if r.get("junction", "-1") != "-1":
            ax.plot(xs, ys, "-", color="#e0982f", linewidth=0.45, alpha=0.85,
                    solid_capstyle="round", zorder=2)
            n_jn += 1
        else:
            ax.plot(xs, ys, "-", color="#7ea6d2", linewidth=0.7, alpha=0.9,
                    solid_capstyle="round", zorder=1)
            n_ord += 1

    ax.set_aspect("equal")
    ax.axis("off")
    name = root.find("header").get("name") if root.find("header") is not None else ""
    ax.set_title(
        f"{name}   {len(roads):,} roads · {len(root.findall('junction')):,} junctions · "
        f"{total_len/1000:.0f} km\n"
        f"blue = road   amber = junction connecting road",
        color="#e7eaec", fontsize=13, pad=18)

    fig.tight_layout()
    fig.savefig(out, dpi=args.dpi, facecolor=fig.get_facecolor())
    print(f"wrote {out}")
    print(f"  {n_ord:,} ordinary roads, {n_jn:,} connecting roads, {total_len/1000:.1f} km")
    return 0


if __name__ == "__main__":
    sys.exit(main())

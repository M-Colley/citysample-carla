#!/usr/bin/env python3
"""
xodr_to_recast_obj.py - walkable surface from an OpenDRIVE file, for RecastBuilder.

CARLA loads pedestrian navigation from a Recast `.bin` - for a pre-existing map,
`Content/<map folder>/Nav/<MapName>.bin` - built by RecastBuilder from an OBJ of
the walkable surface. CARLA generates that pair itself for
`generate_opendrive_world`, but NOT for a level that already exists - so on the
City Sample there is no navmesh, `get_random_location_from_navigation()` returns
None, and AI walker controllers fail silently.

This produces the same surface CARLA's own path does: driving lanes plus
crosswalks (CarlaEpisode.cpp builds `RoadMesh + CrosswalksMesh`).

    python tools/xodr_to_recast_obj.py SmallCity-signals.xodr -o SmallCity.obj
    <carla>/Build/_deps/recastnavigation-build/RecastBuilder/RecastBuilder.exe SmallCity.obj 0.3
    copy SmallCity.bin "<CitySample>\\Content\\Map\\Nav\\Small_City_LVL.bin"

That destination is `Content\\<map folder>\\Nav\\`, NOT `Saved\\Nav\\`: the
client fetches the navmesh through get_required_files("Nav"), which only looks
under Saved/ for generated OpenDRIVE worlds and walks the map's content folder
for anything else. A .bin in the wrong one is ignored in silence.
Integrate-CarlaIntoCitySample.ps1 installs it in the right place for you.

TWO CONVENTIONS THAT MUST MATCH CARLA (Mesh::GenerateOBJForRecast):

  * vertices are written `v x z y` - Recast is Y-up, OpenDRIVE/UE are Z-up;
  * face winding must make each normal point UP in that space. Recast discards
    any triangle outside the walkable slope angle, so the wrong winding yields a
    navmesh with ZERO tiles and no error message whatsoever - which is exactly
    what a fixed `f i1 i3 i2` produced here. The writer computes each normal and
    picks the winding rather than assuming one.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from bisect import bisect_right
from collections import defaultdict

try:
    # Optional: only mark_quads uses it, and only for speed - the samples it
    # takes are the same ones the pure-Python loop takes, bit for bit.
    import numpy as np
except ImportError:
    np = None

# Recast sizes its voxel grid from the bounding box over EVERY vertex, so one
# stray coordinate makes the build intractable. CARLA clamps at 100 km for the
# same reason; keep the identical guard.
MAX_SANE_COORD = 1.0e5

# WHY THIS TOOL SYNTHESISES SIDEWALKS
# -----------------------------------
# CARLA's own recast path meshes driving lanes and crosswalks. That suffices for
# a town whose .xodr also carries `type="sidewalk"` lanes - but the City Sample
# road network is exported from Epic's ZoneGraph, which models driving lanes and
# nothing else. Its .xodr has 3,497 driving lanes, 1,096 crosswalks, and zero
# sidewalks.
#
# That is fatal for pedestrians, because Navigation::GetRandomLocation filters
# on one area flag only (LibCarla/source/carla/nav/Navigation.cpp:1074):
#
#     filter2.setIncludeFlags(CARLA_TYPE_SIDEWALK);
#
# With no sidewalk polygons every get_random_location_from_navigation() returns
# None, so no walker can ever be placed - even though the navmesh is otherwise
# valid and walkers would move happily once put on it (the default crowd filter
# includes CARLA_TYPE_WALKABLE and then excludes road, so a walker uses
# sidewalk, crosswalk and grass). RecastBuilder reads the area from
# the `usemtl` name: road / crosswalk / sidewalk / grass, anything else is
# CARLA_AREA_BLOCK (RecastBuilder/Source/MeshLoaderObj.cpp:218).
#
# So a strip is laid along each outer carriageway edge. These are an
# approximation of where City Sample's real sidewalk meshes are, not a
# measurement of them - see --sidewalk-width / --no-sidewalks.
SIDEWALK_WIDTH = 2.5
CURB_GAP = 0.15

# ZoneGraph exports every lane as its own one-lane road, so a strip laid just
# outside road A almost always lands on road B - its neighbour in the same
# carriageway, or the opposite direction of the same street. Placing sidewalks
# at a fixed offset would put them in live traffic (and simply dropping the
# overlapping ones threw away 98% of them, leaving pedestrians only on the
# outskirts).
#
# Instead every road surface is rasterised into an occupancy grid, and each
# sidewalk sample marches outward from its lane edge until it finds space clear
# enough for the whole strip. That lands it past the full width of the street,
# wherever the real curb is, without needing to know how many lanes it crossed.
GRID_CELL = 1.0
SIDEWALK_SEARCH = 16.0    # give up beyond this - do not wander across a block
SIDEWALK_PROBE = 0.5

# A strip that stops where its road meets a junction leaves the corner bare:
# the two streets' strips end at right angles a few metres apart, and each
# block side becomes an island its walkers can never leave (one per street
# segment, measured on a synthetic grid). So at a junction end the strip runs
# on, straight, at its own offset, for as long as its whole footprint stays
# clear of road - into the corner, where the cross street's strip does the same.
CORNER_REACH = 16.0

# Where a junction's right turn hugs the corner, ITS outer edge is the kerb, and
# the strip that belongs there is the one that wraps the corner. So connecting
# roads get strips too, but only kerbside ones: the strip must fit within this
# of the lane edge. Marching the full SIDEWALK_SEARCH would carry a straight or
# left-turn lane's strip across the junction surface into whatever clear patch
# lies beyond it.
CORNER_SEARCH = 2.0

# The connectivity check rasterises at this, counting a cell only when its
# centre is inside a walkable face: close to Recast's own threshold, where a
# walker's 0.3 m radius closes anything narrower than about 0.6 m.
ISLAND_CELL = 0.5


def read_root(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            return ET.fromstring(raw.decode(enc))
        except (UnicodeDecodeError, ET.ParseError):
            continue
    raise ValueError(f"cannot parse {path}")


def load_json_any(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-16", "utf-16-le", "utf-16-be", "utf-8"):
        try:
            return json.loads(raw.decode(enc))
        except (UnicodeDecodeError, UnicodeError, json.JSONDecodeError):
            continue
    raise ValueError(f"cannot decode {path} as JSON")


def extract_zonegraph_sidewalk_lanes(zg_doc: dict) -> list[dict]:
    """Extract Epic's authored Pedestrian (non-Crosswalk) lanes directly from
    `zonegraph.json`, propagating sidewalk membership to untagged neighbours."""
    out: list[dict] = []
    for graph in zg_doc.get("zone_graphs", []):
        lanes = graph.get("lanes", [])
        if not lanes:
            continue
        by_idx = {ln["index"]: ln for ln in lanes}
        adj: dict[int, list[int]] = defaultdict(list)
        resolved: dict[int, str] = {}
        queue: list[int] = []
        for ln in lanes:
            tags = ln.get("tags", [])
            if "Crosswalk" in tags or "Pedestrian" in tags:
                resolved[ln["index"]] = "sidewalk"
                queue.append(ln["index"])
            elif any(t in tags for t in ("Vehicle", "Freeway", "Freeway Onramp", "Freeway Offramp")):
                resolved[ln["index"]] = "driving"
                queue.append(ln["index"])
            for lk in ln.get("links", []):
                v = lk.get("dest")
                if v in by_idx:
                    adj[ln["index"]].append(v)
                    adj[v].append(ln["index"])
        head = 0
        while head < len(queue):
            u = queue[head]
            head += 1
            for v in adj.get(u, ()):
                if v not in resolved:
                    resolved[v] = resolved[u]
                    queue.append(v)
        for ln in lanes:
            tags = ln.get("tags", [])
            if "Crosswalk" in tags:
                continue
            if resolved.get(ln["index"]) == "sidewalk" and len(ln.get("points", [])) >= 2:
                out.append(ln)
    return out


def poly3(a, b, c, d, t):
    return a + b * t + c * t * t + d * t ** 3


def _coeffs(pp):
    """(bU, cU, dU, bV, cV, dV) of a <paramPoly3>, parsed once per geometry."""
    return tuple(float(pp.get(k)) for k in ("bU", "cU", "dU", "bV", "cV", "dV"))


def geom_points(g, step):
    """Sample one <geometry>: yields (x, y, hdg) in OpenDRIVE metres."""
    x0, y0 = float(g.get("x")), float(g.get("y"))
    hdg = float(g.get("hdg"))
    length = float(g.get("length", 0.0))
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    pp = g.find("paramPoly3")
    if pp is not None:
        bu, cu, du, bv, cv, dv = _coeffs(pp)
    n = max(2, int(length / step) + 1)
    for i in range(n):
        t = i / (n - 1)
        if pp is None:
            u, v = length * t, 0.0
            dudt, dvdt = 1.0, 0.0
        else:
            u, v = poly3(0.0, bu, cu, du, t), poly3(0.0, bv, cv, dv, t)
            dudt = bu + 2 * cu * t + 3 * du * t * t
            dvdt = bv + 2 * cv * t + 3 * dv * t * t
        yield (x0 + u * cos_h - v * sin_h,
               y0 + u * sin_h + v * cos_h,
               hdg + math.atan2(dvdt, dudt),
               t * length)


def carla_point_at(g, ds):
    """(x, y, hdg) `ds` metres into one <geometry>, exactly as LibCarla gets it.

    The twin of roadnetwork_to_xodr.carla_polyline, which places crosswalks:
    the normalised parameter in max(int(length / 0.5), 5) equal steps, s as the
    running chord, then a straight-line interpolation of position and tangent
    between the two samples around ds. Reading an object back any other way -
    taking the nearest of a coarser set of samples, or s = p * length - puts it
    up to half a metre from where both the writer and CARLA put it.
    """
    x0, y0 = float(g.get("x")), float(g.get("y"))
    hdg = float(g.get("hdg"))
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    pp = g.find("paramPoly3")
    if pp is None:
        return x0 + ds * cos_h, y0 + ds * sin_h, hdg
    bu, cu, du, bv, cv, dv = _coeffs(pp)
    n = max(int(float(g.get("length", 0.0)) / 0.5), 5)
    s, prev = 0.0, None
    for i in range(n + 1):
        p = i / n
        cur = (((du * p + cu) * p + bu) * p, ((dv * p + cv) * p + bv) * p,
               (3.0 * du * p + 2.0 * cu) * p + bu, (3.0 * dv * p + 2.0 * cv) * p + bv)
        if prev is not None:
            seg = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            if s + seg >= ds or i == n:
                k = min(1.0, max(0.0, (ds - s) / seg)) if seg > 0.0 else 0.0
                u, v, tu, tv = (a + k * (b - a) for a, b in zip(prev, cur))
                return (x0 + u * cos_h - v * sin_h, y0 + u * sin_h + v * cos_h,
                        hdg + math.atan2(tv, tu))
            s += seg
        prev = cur
    return x0, y0, hdg


def elevation_fn(road):
    """s -> z along one road, from its elevation profile parsed once.

    Picks the last record starting at or before s, as elevation_at always did,
    but by bisection over a pre-parsed list instead of a findall() and a float
    parse of every record per sample - which was a tenth of the run time.
    """
    first = {}
    for e in road.findall("elevationProfile/elevation"):
        # setdefault: the first record at a given s wins, as it always did.
        first.setdefault(float(e.get("s", 0.0)),
                         tuple(float(e.get(k, 0.0)) for k in ("a", "b", "c", "d")))
    starts = sorted(first)
    polys = [first[s] for s in starts]

    def z(s):
        i = bisect_right(starts, s) - 1
        return 0.0 if i < 0 else poly3(*polys[i], s - starts[i])
    return z


def elevation_at(road, s):
    return elevation_fn(road)(s)


def _samples_per_side(a, b, c, d):
    span = max(math.dist(a[:2], b[:2]), math.dist(d[:2], c[:2]),
               math.dist(a[:2], d[:2]), math.dist(b[:2], c[:2]))
    return max(2, min(64, int(span / (GRID_CELL * 0.5)) + 2))


def mark_quads(cells, quads):
    """Rasterise every quad into the occupancy grid at once.

    Takes exactly the samples mark_quad takes, with the same arithmetic in the
    same order, so the grid is identical. With numpy the ~7 million samples of
    a city cost a fraction of a second instead of being half the run time.
    """
    if np is None:
        for q in quads:
            mark_quad(cells, *q)
        return
    by_n = defaultdict(list)
    for q in quads:
        by_n[_samples_per_side(*q)].append([p[:2] for p in q])
    for n, group in by_n.items():
        w = np.arange(n + 1) / n
        for k in range(0, len(group), 4096):          # bounded memory
            q = np.asarray(group[k:k + 4096], dtype=np.float64)   # (N, 4, 2)
            a, b, c, d = (q[:, i, None, None, :] for i in range(4))
            u = w[None, :, None, None]
            v = w[None, None, :, None]
            p0 = a + (d - a) * u
            p1 = b + (c - b) * u
            xy = np.floor((p0 + (p1 - p0) * v) / GRID_CELL).astype(np.int64)
            # One int64 per cell so unique() is a 1-D sort: high word x, low
            # word y. Cell indices are far inside 32 bits (MAX_SANE_COORD).
            keys = np.unique((xy[..., 0] << 32) + (xy[..., 1] & 0xFFFFFFFF))
            lo = keys & 0xFFFFFFFF
            cells.update(zip((keys >> 32).tolist(),
                             ((lo ^ 0x80000000) - 0x80000000).tolist()))


def mark_quad(cells, a, b, c, d):
    """Rasterise a planar quad (a-b-c-d in order) into the occupancy grid.

    Bilinear sampling at half a cell is plenty here: the shapes are long thin
    ribbons, and a missed cell only means one sidewalk quad survives that should
    not have.
    """
    n = _samples_per_side(a, b, c, d)
    for i in range(n + 1):
        u = i / n
        p0 = (a[0] + (d[0] - a[0]) * u, a[1] + (d[1] - a[1]) * u)
        p1 = (b[0] + (c[0] - b[0]) * u, b[1] + (c[1] - b[1]) * u)
        for j in range(n + 1):
            v = j / n
            x = p0[0] + (p1[0] - p0[0]) * v
            y = p0[1] + (p1[1] - p0[1]) * v
            cells.add((int(math.floor(x / GRID_CELL)),
                       int(math.floor(y / GRID_CELL))))


def clear_offset(cells, x, y, nx, ny, start, direction, width,
                 search=SIDEWALK_SEARCH):
    """March outward from `start` until a strip of `width` fits clear of roads.

    Returns the lateral offset of the strip's inner edge, or None if nothing
    fits within `search`. Probing the whole strip rather than just its
    inner edge stops a sidewalk from being tucked into a gap narrower than
    itself, e.g. the seam between two adjacent exported lanes.
    """
    steps = int(search / SIDEWALK_PROBE)
    probes = [w * width for w in (0.0, 0.25, 0.5, 0.75, 1.0)]
    floor = math.floor
    for i in range(steps + 1):
        t = start + direction * (CURB_GAP + i * SIDEWALK_PROBE)
        # The occupancy test, inlined: this is the inner loop of the sidewalk
        # pass, run up to 165 times per sample.
        for p in probes:
            q = t + direction * p
            if (int(floor((x + nx * q) / GRID_CELL)),
                    int(floor((y + ny * q) / GRID_CELL))) in cells:
                break
        else:
            return t
    return None


def _strip_clear(cells, x, y, nx, ny, t, direction, width):
    """Is a strip of `width` at lateral offset t clear of road cells here?
    The same five probes clear_offset uses."""
    floor = math.floor
    for w in (0.0, 0.25, 0.5, 0.75, 1.0):
        q = t + direction * w * width
        if (int(floor((x + nx * q) / GRID_CELL)),
                int(floor((y + ny * q) / GRID_CELL))) in cells:
            return False
    return True


def _meets_junction(road, tag):
    e = road.find(f"link/{tag}")
    return e is not None and e.get("elementType") == "junction"


def run_on(cells, end, sense, step, direction, width):
    """Strip samples continuing past one end of a road, nearest first.

    `end` is the (x, y, hdg, z, t) sample at that end and `sense` is +1 past
    the road's end or -1 before its start. The strip keeps its heading and its
    offset t, and stops at the first position where its footprint would touch
    road - probed every SIDEWALK_PROBE, so it reaches as close to the cross
    street as the grid allows, but sampled for quads only every `step` plus
    the last clear position.
    """
    x, y, hdg, z, t = end
    if t is None:
        return []
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    nx, ny = -sin_h, cos_h
    every = max(1, round(step / SIDEWALK_PROBE))
    out, last = [], None
    for k in range(1, int(CORNER_REACH / SIDEWALK_PROBE) + 1):
        d = sense * k * SIDEWALK_PROBE
        px, py = x + d * cos_h, y + d * sin_h
        if not _strip_clear(cells, px, py, nx, ny, t, direction, width):
            break
        last = (px, py, hdg, z, t)
        if k % every == 0:
            out.append(last)
    if last is not None and (not out or out[-1] is not last):
        out.append(last)
    return out


def _quad_centre_cells(quads, cell, as_array=False):
    """Cells of size `cell` whose CENTRE lies inside any of the convex quads.

    A set of (i, j), or with `as_array` (numpy only) an (N, 2) int array that
    may repeat cells - cheaper when the caller only fills a grid with them.
    """
    cells = set()
    found = []
    rest = quads
    if np is not None and quads:
        # Test a block of cell centres from each quad's lower corner, all quads
        # of a size at once: 8x8 covers a sidewalk quad, 64x64 a crossing.
        QA = np.asarray([[p[:2] for p in q] for q in quads], dtype=np.float64)
        LO = np.floor(QA.min(axis=1) / cell).astype(np.int64)             # (N, 2)
        SPAN = (np.floor(QA.max(axis=1) / cell).astype(np.int64) - LO).max(axis=1)
        rest = [q for q, s in zip(quads, SPAN) if s >= 128]
        for size in (8, 16, 32, 64, 128):
            pick = (SPAN < size) & (SPAN >= size // 2 if size > 8 else True)
            k = np.arange(size)
            chunk = max(1, (1 << 20) // (size * size))        # ~1M centres at a time
            QP, LP = QA[pick], LO[pick]
            for c0 in range(0, len(QP), chunk):
                Q, lo = QP[c0:c0 + chunk], LP[c0:c0 + chunk]
                ci = lo[:, 0, None, None] + k[None, :, None]              # (N, size, 1)
                cj = lo[:, 1, None, None] + k[None, None, :]              # (N, 1, size)
                cx, cy = (ci + 0.5) * cell, (cj + 0.5) * cell
                a, b = Q, np.roll(Q, -1, axis=1)
                area = ((a[:, :, 0] * b[:, :, 1]) - (b[:, :, 0] * a[:, :, 1])).sum(axis=1)
                sgn = np.sign(area)[:, None, None]
                inside = (area != 0.0)[:, None, None]
                for e in range(4):
                    ex0, ey0 = a[:, e, 0, None, None], a[:, e, 1, None, None]
                    ex = (b[:, e, 0] - a[:, e, 0])[:, None, None]
                    ey = (b[:, e, 1] - a[:, e, 1])[:, None, None]
                    inside = inside & (sgn * (ex * (cy - ey0) - ey * (cx - ex0)) >= 0.0)
                n_idx, i_idx, j_idx = np.nonzero(inside)
                ij = np.stack([lo[n_idx, 0] + i_idx, lo[n_idx, 1] + j_idx], axis=1)
                if as_array:
                    found.append(ij)
                else:
                    cells.update(zip(ij[:, 0].tolist(), ij[:, 1].tolist()))
    for q in rest:
        pts = [p[:2] for p in q]
        edges = list(zip(pts, pts[1:] + pts[:1]))
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in edges)
        if area == 0.0:
            continue
        sgn = 1.0 if area > 0.0 else -1.0
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        for i in range(math.floor(min(xs) / cell), math.floor(max(xs) / cell) + 1):
            cx = (i + 0.5) * cell
            for j in range(math.floor(min(ys) / cell), math.floor(max(ys) / cell) + 1):
                cy = (j + 0.5) * cell
                if all(sgn * ((x1 - x0) * (cy - y0) - (y1 - y0) * (cx - x0)) >= 0.0
                       for (x0, y0), (x1, y1) in edges):
                    cells.add((i, j))
    if as_array:
        if cells:
            found.append(np.asarray(sorted(cells), dtype=np.int64))
        return (np.concatenate(found) if found
                else np.zeros((0, 2), dtype=np.int64))
    return cells


def walkable_islands(obj):
    """(islands, share of the area in the largest) of what a walker may use.

    Sidewalk and crosswalk faces only - the crowd filter excludes road - with
    a cell walkable only when its centre is inside one (ISLAND_CELL), joined
    4-ways. An approximation of Recast's own connectivity, but it tells one
    network from a city of islands, which nothing else here does: a walker on
    an island walks, and the walker test passes.
    """
    bounds = [g[0] for g in obj.groups[1:]] + [len(obj.faces)]
    quads = []
    for (start, name), end in zip(obj.groups, bounds):
        if name in ("sidewalk", "crosswalk"):
            for f in range(start, end, 2):
                b = obj.faces[f][0]
                quads.append(tuple(obj.verts[b - 1 + k] for k in range(4)))
    if np is not None:
        try:
            from scipy import ndimage
        except ImportError:
            ndimage = None
        if ndimage is not None:
            # Same cells, labelled on a dense grid instead of flood-filled
            # one tuple at a time: a city is ~700,000 cells at ISLAND_CELL.
            ij = _quad_centre_cells(quads, ISLAND_CELL, as_array=True)
            if not len(ij):
                return 0, 0.0
            ij = ij - ij.min(axis=0)
            grid = np.zeros(ij.max(axis=0) + 1, dtype=bool)
            grid[ij[:, 0], ij[:, 1]] = True
            labels, n = ndimage.label(grid)          # 4-connected, as below
            sizes = np.bincount(labels.ravel())[1:]
            return int(n), float(sizes.max() / sizes.sum())
    cells = _quad_centre_cells(quads, ISLAND_CELL)
    seen, sizes = set(), []
    for c in cells:
        if c in seen:
            continue
        seen.add(c)
        stack, n = [c], 0
        while stack:
            cx, cy = stack.pop()
            n += 1
            for nb in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                if nb in cells and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        sizes.append(n)
    return len(sizes), (max(sizes) / sum(sizes) if sizes else 0.0)


def carriageway_edges(road, lane_type="driving"):
    """(laneOffset, width of the left lanes, width of the right lanes), or None.

    OpenDRIVE lane t grows to the left, so the surface spans
    [offset - right_w, offset + left_w] about the reference line. Summing both
    sides into one width - as an earlier version did - is only right for the
    one-sided roads ZoneGraph happens to emit, and puts the sidewalk inside the
    carriageway on any road that has lanes on both sides. Only lanes of
    `lane_type` count.
    """
    ls = road.find("lanes/laneSection")
    if ls is None:
        return None

    def side_width(tag):
        total = 0.0
        for lane in ls.findall(f"{tag}/lane"):
            if lane.get("type") != lane_type:
                continue
            w = lane.find("width")
            total += float(w.get("a")) if w is not None else 3.5
        return total

    left_w, right_w = side_width("left"), side_width("right")
    if left_w <= 0.0 and right_w <= 0.0:
        return None
    lo = road.find("lanes/laneOffset")
    offset = float(lo.get("a")) if lo is not None else 0.0
    return offset, left_w, right_w


class ObjWriter:
    def __init__(self):
        self.verts = []
        self.faces = []          # (i1, i2, i3) 1-based, pre-winding
        self.groups = []         # (start_face_index, material)
        self.sanitized = 0
        self.flipped = 0

    def material(self, name):
        self.groups.append((len(self.faces), name))

    def add_quad(self, p0, p1, p2, p3):
        base = len(self.verts) + 1
        lim = MAX_SANE_COORD
        for p in (p0, p1, p2, p3):
            if -lim <= p[0] <= lim and -lim <= p[1] <= lim and -lim <= p[2] <= lim:
                self.verts.append(p)
            else:
                self.verts.append((0.0, 0.0, 0.0))
                self.sanitized += 1
        self.faces.append((base, base + 1, base + 2))
        self.faces.append((base, base + 2, base + 3))

    def write(self, path):
        # Built as one string and written once: a city is a million lines, and
        # a write() call per line was most of this tool's run time.
        out = ["# Walkable surface for RecastBuilder\n",
               f"# {len(self.verts)} vertices, {len(self.faces)} faces\n\n"]
        # Recast is Y-up: swap y and z, exactly as Mesh::GenerateOBJForRecast does.
        out.extend([f"v {x:.6f} {z:.6f} {y:.6f}\n" for x, y, z in self.verts])
        out.append("\n# Polygonal face element.\n")
        gi = 0
        for fi, (a, b, c) in enumerate(self.faces):
            while gi < len(self.groups) and self.groups[gi][0] == fi:
                out.append(f"\nusemtl {self.groups[gi][1]}\n")
                gi += 1
            # Winding decides the normal, and Recast discards any triangle
            # whose normal is not within the walkable-slope angle of up - so
            # a road ribbon wound the wrong way yields a navmesh with ZERO
            # tiles and no error at all. Rather than assume an order,
            # compute the normal in Recast space (X=x, Y=z, Z=y) and emit
            # whichever winding points it upwards.
            pa, pb, pc = self.verts[a - 1], self.verts[b - 1], self.verts[c - 1]
            # Recast space is (X, Y, Z) = (x, z, y): the up axis is Y, and
            # the ground plane is X/Z = our x/y. The up component of
            # (b-a) x (c-a) is therefore u_Z*v_X - u_X*v_Z, i.e. the
            # determinant taken in that order:
            udx, udy = pb[0] - pa[0], pb[1] - pa[1]
            vdx, vdy = pc[0] - pa[0], pc[1] - pa[1]
            ny = udy * vdx - udx * vdy
            if ny >= 0.0:
                out.append(f"f {a} {b} {c}\n")
            else:
                out.append(f"f {a} {c} {b}\n")
                self.flipped += 1
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("".join(out))


def road_ribbon(road, step, t_inner, t_outer):
    """Quads for the strip between two lateral offsets along a road.

    Both the carriageway and a sidewalk are the same shape - a ribbon swept
    along the reference line between two t values - so they share one routine.
    Yields (near_inner, near_outer, far_outer, far_inner) in winding order.
    """
    prev = None
    elevation = elevation_fn(road)
    for g in road.findall("planView/geometry"):
        gs = float(g.get("s", 0.0))
        for (x, y, hdg, ds) in geom_points(g, step):
            z = elevation(gs + ds)
            nx, ny = -math.sin(hdg), math.cos(hdg)
            inner = (x + nx * t_inner, y + ny * t_inner, z)
            outer = (x + nx * t_outer, y + ny * t_outer, z)
            if prev is not None:
                yield prev[0], inner, outer, prev[1]
            prev = (inner, outer)


def build(root, step=2.0, verbose=False, sidewalk_width=SIDEWALK_WIDTH,
          corners=True, zonegraph_doc: dict | None = None):
    obj = ObjWriter()
    obj.material("road")

    # Roads first: the sidewalk pass needs the finished occupancy grid, and the
    # writer groups faces by the material in force when they were added.
    road_cells = set()
    road_quads = []
    n_roads = 0
    meshed = []
    for road in root.findall("road"):
        edges = carriageway_edges(road)
        if edges is None:
            continue
        offset, left_w, right_w = edges
        # Emitting one ribbon per road rather than per lane keeps the triangle
        # count sane; Recast wants a surface, not lane fidelity.
        for quad in road_ribbon(road, step, offset + left_w, offset - right_w):
            obj.add_quad(*quad)
            road_quads.append(quad)
        meshed.append((road, offset, left_w, right_w))
        n_roads += 1
    mark_quads(road_cells, road_quads)

    # Measured sidewalks: either directly from Epic's authored ZoneGraph
    # Pedestrian lanes (--zonegraph, keeping the .xodr compact) or from
    # sidewalk-typed lanes inside the .xodr (--keep-sidewalks). When measured
    # sidewalks exist, synthesising 2.5 m strips on top would only add guesses.
    zg_sidewalks = extract_zonegraph_sidewalk_lanes(zonegraph_doc) if zonegraph_doc else []
    measured = []
    for road in root.findall("road"):
        edges = carriageway_edges(road, "sidewalk")
        if edges is not None:
            measured.append((road, edges))
    synthesise = sidewalk_width > 0.0 and not measured and not zg_sidewalks

    n_sw = n_sw_measured = n_corner = 0
    n_sw_dropped = 0
    if zg_sidewalks or measured or synthesise:
        obj.material("sidewalk")
    for ln in zg_sidewalks:
        # Epic authors Pedestrian lanes at 1.0 m width; pad single-lane ribbons
        # slightly (1.6 m total, 0.8 m half-width) so Recast's 2 x 0.3 m agent
        # radius erosion leaves a solid navigable corridor around every corner.
        half_w = max(0.5 * float(ln.get("width_cm", 100.0)) * 0.01, 0.8)
        pts = [(p["pos"][0] * 0.01, -p["pos"][1] * 0.01, p["pos"][2] * 0.01)
               for p in ln["points"]]
        prev = None
        for i, (x, y, z) in enumerate(pts):
            ax, ay, _ = pts[max(0, i - 1)]
            bx, by, _ = pts[min(len(pts) - 1, i + 1)]
            hdg = math.atan2(by - ay, bx - ax)
            nx, ny = -math.sin(hdg), math.cos(hdg)
            cur = ((x + nx * half_w, y + ny * half_w, z),
                   (x - nx * half_w, y - ny * half_w, z))
            if prev is not None:
                obj.add_quad(prev[0], cur[0], cur[1], prev[1])
                n_sw_measured += 1
            prev = cur
    for road, (offset, left_w, right_w) in measured:
        for quad in road_ribbon(road, step, offset + left_w, offset - right_w):
            obj.add_quad(*quad)
            n_sw_measured += 1
    if synthesise:
        for road, offset, left_w, right_w in meshed:
            # Connecting roads get kerbside strips only (CORNER_SEARCH): the
            # corner a right turn wraps. Streets get the full outward march,
            # and run on into the junction at their ends (run_on).
            # corners=False is the old behaviour, kept for the selftest: no
            # strips in junctions, none running on, every block side an island.
            in_junction = road.get("junction", "-1") != "-1"
            if in_junction and not corners:
                continue
            search = CORNER_SEARCH if in_junction else SIDEWALK_SEARCH
            elevation = elevation_fn(road)
            at_junction = ((False, False) if in_junction or not corners else
                           (_meets_junction(road, "predecessor"),
                            _meets_junction(road, "successor")))
            for edge, direction in ((offset + left_w, +1.0),
                                    (offset - right_w, -1.0)):
                samples = []
                for g in road.findall("planView/geometry"):
                    gs = float(g.get("s", 0.0))
                    for (x, y, hdg, ds) in geom_points(g, step):
                        z = elevation(gs + ds)
                        nx, ny = -math.sin(hdg), math.cos(hdg)
                        t = clear_offset(road_cells, x, y, nx, ny, edge,
                                         direction, sidewalk_width, search)
                        if t is None and not in_junction:
                            n_sw_dropped += 1
                        samples.append((x, y, hdg, z, t))
                if not samples:
                    continue
                head = (run_on(road_cells, samples[0], -1.0, step, direction,
                               sidewalk_width) if at_junction[0] else [])
                tail = (run_on(road_cells, samples[-1], +1.0, step, direction,
                               sidewalk_width) if at_junction[1] else [])
                n_corner += len(head) + len(tail)
                prev = None
                for (x, y, hdg, z, t) in head[::-1] + samples + tail:
                    if t is None:
                        # Boxed in on this side for this stretch. Break the
                        # ribbon rather than bridging the gap, which would
                        # lay a quad straight back over the carriageway.
                        prev = None
                        continue
                    nx, ny = -math.sin(hdg), math.cos(hdg)
                    far = t + direction * sidewalk_width
                    inner = (x + nx * t, y + ny * t, z)
                    outer = (x + nx * far, y + ny * far, z)
                    if prev is not None:
                        obj.add_quad(prev[0], inner, outer, prev[1])
                        n_sw += 1
                    prev = (inner, outer)

    n_cw = 0
    obj.material("crosswalk")
    for road in root.findall("road"):
        geoms = road.findall("planView/geometry")
        if not geoms:
            continue
        for o in road.findall("objects/object"):
            if o.get("type") != "crosswalk":
                continue
            corners = o.findall("outline/cornerLocal")
            if len(corners) < 4:
                continue
            s_pos = float(o.get("s", 0.0))
            t_pos = float(o.get("t", 0.0))
            hdg_rel = float(o.get("hdg", 0.0))
            # Locate (s,t) on the road, then place the local outline there.
            #
            # The geometry is the last one starting at or before s, which is
            # how CARLA picks it, and the point is evaluated the way CARLA
            # evaluates it (carla_point_at) - not snapped to the nearest of a
            # set of samples, which moved crossings on long curved segments by
            # up to half a metre and turned them by the local curvature.
            g = None
            for cand in geoms:
                if float(cand.get("s", 0.0)) <= s_pos or g is None:
                    g = cand
            bx, by, bhdg = carla_point_at(g, s_pos - float(g.get("s", 0.0)))
            nx, ny = -math.sin(bhdg), math.cos(bhdg)
            ox, oy = bx + nx * t_pos, by + ny * t_pos
            z = elevation_at(road, s_pos) + 0.02
            th = bhdg + hdg_rel
            pts = []
            for c in corners[:4]:
                u, v = float(c.get("u")), float(c.get("v"))
                pts.append((ox + u * math.cos(th) - v * math.sin(th),
                            oy + u * math.sin(th) + v * math.cos(th), z))
            obj.add_quad(*pts)
            n_cw += 1

    if verbose:
        print(f"  roads meshed     : {n_roads:,}")
        if zg_sidewalks:
            print(f"  sidewalk quads   : {n_sw_measured:,} from {len(zg_sidewalks):,} "
                  "measured ZoneGraph pedestrian lanes (none synthesised)")
        elif measured:
            print(f"  sidewalk quads   : {n_sw_measured:,} from {len(measured):,} "
                  "sidewalk lanes in the .xodr (none synthesised)")
        else:
            print(f"  sidewalk quads   : {n_sw:,} synthesised "
                  f"({n_sw_dropped:,} samples boxed in, "
                  f"{n_corner:,} run on into junction corners)")
        print(f"  crosswalks meshed: {n_cw:,}")
    return obj, n_roads, n_cw, n_sw + n_sw_measured


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("-o", "--output", default=None)
    ap.add_argument("--zonegraph", default=None,
                    help="optional zonegraph.json to mesh Epic's measured "
                         "Pedestrian lanes directly instead of synthesising "
                         "2.5 m strips (keeps .xodr free of 60k sidewalk roads)")
    ap.add_argument("--step", type=float, default=2.0,
                    help="sampling step along the reference line, metres")
    ap.add_argument("--sidewalk-width", type=float, default=SIDEWALK_WIDTH,
                    help="width of the synthesised sidewalk strips, metres "
                         f"(default {SIDEWALK_WIDTH}); these are the ONLY "
                         "polygons get_random_location_from_navigation() will "
                         "return, so 0 leaves walkers unspawnable")
    ap.add_argument("--no-sidewalks", action="store_true",
                    help="mesh only what CARLA's own exporter does")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.xodr:
        ap.error("an .xodr is required (or --selftest)")

    root = read_root(args.xodr)
    zg_doc = load_json_any(args.zonegraph) if args.zonegraph else None
    obj, n_roads, n_cw, n_sw = build(
        root, args.step, verbose=True,
        sidewalk_width=0.0 if args.no_sidewalks else args.sidewalk_width,
        zonegraph_doc=zg_doc)
    out = args.output or args.xodr.rsplit(".", 1)[0] + ".obj"
    obj.write(out)
    if not obj.faces:
        # Before the extent maths, which would raise on an empty vertex list
        # and hide this message behind a traceback.
        print("  NO FACES - nothing for Recast to walk on", file=sys.stderr)
        return 1
    xs = [v[0] for v in obj.verts]
    ys = [v[1] for v in obj.verts]
    print(f"wrote {out}")
    print(f"  vertices : {len(obj.verts):,}")
    print(f"  faces    : {len(obj.faces):,}")
    print(f"  extent   : {max(xs)-min(xs):,.0f} x {max(ys)-min(ys):,.0f} m")
    print(f"  flipped  : {obj.flipped:,} faces re-wound to face up")
    islands, largest = walkable_islands(obj)
    print(f"  walkable : {islands:,} island(s), the largest {largest:.0%} of the "
          "sidewalk and crosswalk area")
    if islands > 1 and largest < 0.5:
        print("  NO ISLAND HOLDS HALF THE WALKABLE AREA - a walker can only reach "
              "the island it was spawned on", file=sys.stderr)
    if obj.sanitized:
        print(f"  sanitized {obj.sanitized} out-of-range vertices", file=sys.stderr)
    if not obj.faces:
        print("  NO FACES - nothing for Recast to walk on", file=sys.stderr)
        return 1
    if n_sw == 0 and not args.no_sidewalks:
        print("  NO SIDEWALK QUADS - the navmesh will build, but "
              "get_random_location_from_navigation() will return None for "
              "every call and no walker can be placed", file=sys.stderr)
    return 0


def selftest() -> int:
    """One straight road, one crosswalk; check the axis swap and the winding."""
    fails = []
    # A 10 m straight at elevation 5, one 4 m right lane, laneOffset +2 - so
    # the carriageway spans t in [-2, +2] and the reference line is y = 0.
    ROAD = """
      <road name="r" length="10.0" id="{id}" junction="-1">
        <planView><geometry s="0.0" x="0.0" y="{y}" hdg="0.0" length="10.0">
          <paramPoly3 aU="0.0" bU="10.0" cU="0.0" dU="0.0" aV="0.0" bV="0.0" cV="0.0" dV="0.0" pRange="normalized"/>
        </geometry></planView>
        <elevationProfile><elevation s="0.0" a="5.0" b="0.0" c="0.0" d="0.0"/></elevationProfile>
        <lanes><laneOffset s="0.0" a="2.0" b="0.0" c="0.0" d="0.0"/>
          <laneSection s="0.0"><right><lane id="-1" type="driving" level="false">
          <width sOffset="0.0" a="4.0" b="0.0" c="0.0" d="0.0"/></lane></right></laneSection>
        </lanes>
      </road>"""
    xodr = "<OpenDRIVE>" + ROAD.format(id=1, y=0.0) + "</OpenDRIVE>"
    obj, n_roads, _, n_sw = build(ET.fromstring(xodr), step=5.0)
    if n_roads != 1:
        fails.append(f"expected 1 road meshed, got {n_roads}")
    if not obj.faces:
        fails.append("no faces emitted")
    if n_sw == 0:
        fails.append("no sidewalk quads - walkers would be unspawnable")

    # An isolated road must keep both strips; the drop rule only fires where a
    # second carriageway is already there.
    if "sidewalk" not in [g[1] for g in obj.groups]:
        fails.append("no `usemtl sidewalk` group - RecastBuilder maps the area "
                     "from that name, so the polys would be CARLA_AREA_BLOCK")

    def sidewalk_ys(o):
        """Lateral positions of every sidewalk vertex (the roads run along x)."""
        start = next(g[0] for g in o.groups if g[1] == "sidewalk")
        end = min((g[0] for g in o.groups if g[0] > start), default=len(o.faces))
        return [o.verts[i - 1][1] for f in o.faces[start:end] for i in f]

    # The strips must lie OUTSIDE the carriageway. laneOffset a=2.0 with one
    # 4 m right lane puts the surface across t in [-2, 2], and the reference
    # line runs along y=0, so every sidewalk vertex must clear |y| = 2.
    inside = [v for v in sidewalk_ys(obj) if abs(v) < 2.0 - 1e-6]
    if inside:
        fails.append(f"{len(inside)} sidewalk vertices lie on the carriageway")

    # The whole point of the outward march: a second carriageway alongside the
    # first must push the facing strips clear of BOTH, not lay one down the
    # middle of the street. Road 2 sits at y=-4, so together the two surfaces
    # occupy y in [-6, +2] and no sidewalk vertex may fall inside that.
    twin = ("<OpenDRIVE>" + ROAD.format(id=1, y=0.0)
            + ROAD.format(id=2, y=-4.0) + "</OpenDRIVE>")
    obj_twin, n_roads_twin, _, n_sw_twin = build(ET.fromstring(twin), step=5.0)
    if n_roads_twin != 2:
        fails.append(f"twin fixture is malformed: {n_roads_twin} roads meshed")
    elif n_sw_twin == 0:
        fails.append("two adjacent roads produced no sidewalk at all")
    else:
        ys = sidewalk_ys(obj_twin)
        between = [v for v in ys if -6.0 + 1e-6 < v < 2.0 - 1e-6]
        if between:
            fails.append(f"{len(between)} sidewalk vertices sit between two "
                         "carriageways - pedestrians would spawn in traffic")
        # ...and both far kerbs must still be served. Merely DROPPING the
        # strips that overlap keeps the invariant above while losing one whole
        # side of the street, which is what cost 98% of the city's sidewalks
        # before the march was added - so assert the coverage too.
        if not any(v > 2.0 for v in ys):
            fails.append("no sidewalk outside the first carriageway")
        if not any(v < -6.0 for v in ys):
            fails.append("no sidewalk outside the second carriageway - the "
                         "outward march did not push past the neighbour")

    # A crossing as roadnetwork_to_xodr writes one: ON the reference line
    # (t=0), turned 90 degrees, its ends at u=-3 and u=+4 - not symmetric about
    # the anchor. Worked by hand: u runs along +y and v along -x from (5, 0),
    # so corner (u, v) lands at (5 - v, u).
    xcw = ("<OpenDRIVE>" + ROAD.format(id=1, y=0.0).replace(
        "</lanes>",
        "</lanes><objects><object type=\"crosswalk\" s=\"5.0\" t=\"0.0\" "
        f"hdg=\"{math.pi / 2!r}\"><outline>"
        + "".join(f'<cornerLocal u="{u}" v="{v}"/>'
                  for u, v in ((4, 2), (4, -2), (-3, -2), (-3, 2), (4, 2)))
        + "</outline></object></objects>") + "</OpenDRIVE>")
    obj_cw, _, n_cw, _ = build(ET.fromstring(xcw), step=5.0, sidewalk_width=0.0)
    start = next(g[0] for g in obj_cw.groups if g[1] == "crosswalk")
    got = [obj_cw.verts[i - 1] for i in (obj_cw.faces[start] + obj_cw.faces[start + 1][2:])]
    want = [(3.0, 4.0), (7.0, 4.0), (7.0, -3.0), (3.0, -3.0)]
    if n_cw != 1 or any(math.dist(g[:2], w) > 1e-6 for g, w in zip(got, want)):
        fails.append(f"crosswalk corners {[tuple(round(c, 3) for c in g[:2]) for g in got]}, "
                     f"expected {want}")

    # A right-turn corner: street 1 eastbound into junction 9, road 2 turning
    # right through a quarter circle, street 3 southbound out. Each side of the
    # L - inside the turn and outside it - must be ONE walkable network round
    # the corner, and a crossing over street 1 must join the two. With strips
    # stopping at every junction, each kerb was an island a walker never left.
    def road_xml(rid, junction, p0, h0, p1, h1, link, extra=""):
        chord = math.dist(p0, p1)
        c, s = math.cos(-h0), math.sin(-h0)
        lx = (p1[0] - p0[0]) * c - (p1[1] - p0[1]) * s
        ly = (p1[0] - p0[0]) * s + (p1[1] - p0[1]) * c
        m1x, m1y = chord * math.cos(h1 - h0), chord * math.sin(h1 - h0)
        k = (chord, 3 * lx - 2 * chord - m1x, -2 * lx + chord + m1x,
             0.0, 3 * ly - m1y, -2 * ly + m1y)
        pts = [(sum(k[i] * t ** (i + 1) for i in range(3)),
                sum(k[3 + i] * t ** (i + 1) for i in range(3)))
               for t in (j / 64 for j in range(65))]
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        return (f'<road id="{rid}" junction="{junction}" length="{length}"><link>{link}</link>'
                f'<planView><geometry s="0" x="{p0[0]}" y="{p0[1]}" hdg="{h0}" length="{length}">'
                f'<paramPoly3 aU="0" bU="{k[0]}" cU="{k[1]}" dU="{k[2]}" aV="0" bV="{k[3]}" '
                f'cV="{k[4]}" dV="{k[5]}" pRange="normalized"/></geometry></planView>'
                '<lanes><laneOffset s="0" a="2.0" b="0" c="0" d="0"/><laneSection s="0">'
                '<right><lane id="-1" type="driving"><width sOffset="0" a="4.0" b="0" c="0" d="0"/>'
                f'</lane></right></laneSection></lanes>{extra}</road>')
    south = -math.pi / 2

    def corner_xml(crossing):
        cw = ('<objects><object type="crosswalk" s="15" t="0" hdg="1.5707963267948966">'
              '<outline>' + "".join(f'<cornerLocal u="{u}" v="{v}"/>' for u, v in
                                    ((7, 2), (7, -2), (-7, -2), (-7, 2), (7, 2)))
              + '</outline></object></objects>') if crossing else ""
        return ("<OpenDRIVE>"
                + road_xml(1, -1, (-40.0, 0.0), 0.0, (-10.0, 0.0), 0.0,
                           '<successor elementType="junction" elementId="9"/>', cw)
                + road_xml(2, 9, (-10.0, 0.0), 0.0, (0.0, -10.0), south,
                           '<predecessor elementType="road" elementId="1" contactPoint="end"/>'
                           '<successor elementType="road" elementId="3" contactPoint="start"/>')
                + road_xml(3, -1, (0.0, -10.0), south, (0.0, -40.0), south,
                           '<predecessor elementType="junction" elementId="9"/>')
                + '<junction id="9"/></OpenDRIVE>')

    def islands(xml, **kw):
        return walkable_islands(build(ET.fromstring(xml), step=2.0, **kw)[0])[0]
    got = (islands(corner_xml(False), corners=False), islands(corner_xml(False)),
           islands(corner_xml(True)))
    if got != (4, 2, 1):
        fails.append(f"walkable islands at a corner: {got[0]} kerbs alone, {got[1]} "
                     f"joined round it, {got[2]} with a crossing - expected 4, 2, 1")

    # Epic's own pedestrian lanes (zonegraph_to_roadnetwork --keep-sidewalks)
    # arrive typed "sidewalk". They are the sidewalks, so they are meshed as
    # such and nothing is synthesised on top of them.
    walk_lane = ROAD.format(id=2, y=-10.0).replace('type="driving"', 'type="sidewalk"')
    obj_m, n_roads_m, _, n_sw_m = build(
        ET.fromstring("<OpenDRIVE>" + ROAD.format(id=1, y=0.0) + walk_lane + "</OpenDRIVE>"),
        step=5.0)
    ribbon = sum(1 for _ in road_ribbon(ET.fromstring(walk_lane), 5.0, 2.0, -2.0))
    if (n_roads_m, n_sw_m) != (1, ribbon):
        fails.append(f"a sidewalk lane should be meshed as {ribbon} sidewalk quads with "
                     f"none synthesised; got {n_sw_m} quads, {n_roads_m} road(s) meshed")

    # numpy only changes how fast the occupancy grid is filled, never what is
    # in it: the twin fixture must come out identical without it.
    global np
    if np is not None:
        fast = [walkable_islands(build(ET.fromstring(corner_xml(c)), step=2.0)[0])
                for c in (False, True)]
        saved, np = np, None
        try:
            plain, _, _, _ = build(ET.fromstring(twin), step=5.0)
            slow = [walkable_islands(build(ET.fromstring(corner_xml(c)), step=2.0)[0])
                    for c in (False, True)]
        finally:
            np = saved
        if (plain.verts, plain.faces) != (obj_twin.verts, obj_twin.faces):
            fails.append("numpy and pure-Python rasterisation disagree")
        if [n for n, _ in fast] != [n for n, _ in slow] or any(
                abs(a - b) > 1e-12 for (_, a), (_, b) in zip(fast, slow)):
            fails.append(f"island counts disagree: grid labelling {fast}, "
                         f"pure Python {slow}")

    import io as _io
    import os
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "t.obj")
        obj.write(p)
        text = _io.open(p, encoding="utf-8").read()
    vlines = [l for l in text.splitlines() if l.startswith("v ")]
    flines = [l for l in text.splitlines() if l.startswith("f ")]
    if not vlines:
        fails.append("no vertex lines")
    else:
        # Elevation 5.0 must land in the SECOND slot (y), not the third.
        parts = vlines[0].split()
        if abs(float(parts[2]) - 5.0) > 1e-6:
            fails.append(f"axis swap wrong: expected z=5.0 in slot 2, got {parts[2]}")
    if not flines:
        fails.append("no face lines")
    else:
        # Every emitted face must have an upward normal in Recast space, or
        # Recast walks away with zero tiles.
        vs = [tuple(float(x) for x in l.split()[1:4]) for l in vlines]
        bad = 0
        for l in flines:
            i, j, k = (int(x) - 1 for x in l.split()[1:4])
            (ax, ay, az), (bx, by, bz), (cx, cy, cz) = vs[i], vs[j], vs[k]
            # Slot 0 is X and slot 2 is Z in Recast space; up is Y. The up
            # component of (b-a) x (c-a) is u_Z*v_X - u_X*v_Z. Deriving this
            # the other way round gives a test that agrees with an equally
            # inverted writer and passes while Recast builds zero tiles - which
            # is exactly what happened here.
            n = (bz - az) * (cx - ax) - (bx - ax) * (cz - az)
            if n < 0:
                bad += 1
        if bad:
            fails.append(f"{bad}/{len(flines)} faces have downward normals")

    if fails:
        print("SELFTEST FAILED")
        for f in fails:
            print("  -", f)
        return 1
    print("selftest passed - road meshed, Y-up axis swap, upward winding")
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
import math
import sys
import xml.etree.ElementTree as ET

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


def read_root(path):
    with open(path, "rb") as fh:
        raw = fh.read()
    for enc in ("utf-8-sig", "utf-8", "utf-16", "utf-16-le"):
        try:
            return ET.fromstring(raw.decode(enc))
        except (UnicodeDecodeError, ET.ParseError):
            continue
    raise ValueError(f"cannot parse {path}")


def poly3(a, b, c, d, t):
    return a + b * t + c * t * t + d * t ** 3


def geom_points(g, step):
    """Sample one <geometry>: yields (x, y, hdg) in OpenDRIVE metres."""
    x0, y0 = float(g.get("x")), float(g.get("y"))
    hdg = float(g.get("hdg"))
    length = float(g.get("length", 0.0))
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    pp = g.find("paramPoly3")
    n = max(2, int(length / step) + 1)
    for i in range(n):
        t = i / (n - 1)
        if pp is None:
            u, v = length * t, 0.0
            dudt, dvdt = 1.0, 0.0
        else:
            bu, cu, du = (float(pp.get(k)) for k in ("bU", "cU", "dU"))
            bv, cv, dv = (float(pp.get(k)) for k in ("bV", "cV", "dV"))
            u, v = poly3(0.0, bu, cu, du, t), poly3(0.0, bv, cv, dv, t)
            dudt = bu + 2 * cu * t + 3 * du * t * t
            dvdt = bv + 2 * cv * t + 3 * dv * t * t
        yield (x0 + u * cos_h - v * sin_h,
               y0 + u * sin_h + v * cos_h,
               hdg + math.atan2(dvdt, dudt),
               t * length)


def elevation_at(road, s):
    best = None
    for e in road.findall("elevationProfile/elevation"):
        es = float(e.get("s", 0.0))
        if es <= s and (best is None or es > float(best.get("s", 0.0))):
            best = e
    if best is None:
        return 0.0
    ds = s - float(best.get("s", 0.0))
    return poly3(*(float(best.get(k, 0.0)) for k in ("a", "b", "c", "d")), ds)


def mark_quad(cells, a, b, c, d):
    """Rasterise a planar quad (a-b-c-d in order) into the occupancy grid.

    Bilinear sampling at half a cell is plenty here: the shapes are long thin
    ribbons, and a missed cell only means one sidewalk quad survives that should
    not have.
    """
    span = max(math.dist(a[:2], b[:2]), math.dist(d[:2], c[:2]),
               math.dist(a[:2], d[:2]), math.dist(b[:2], c[:2]))
    n = max(2, min(64, int(span / (GRID_CELL * 0.5)) + 2))
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


def occupied(cells, x, y):
    return (int(math.floor(x / GRID_CELL)), int(math.floor(y / GRID_CELL))) in cells


def clear_offset(cells, x, y, nx, ny, start, direction, width):
    """March outward from `start` until a strip of `width` fits clear of roads.

    Returns the lateral offset of the strip's inner edge, or None if nothing
    fits within SIDEWALK_SEARCH. Probing the whole strip rather than just its
    inner edge stops a sidewalk from being tucked into a gap narrower than
    itself, e.g. the seam between two adjacent exported lanes.
    """
    steps = int(SIDEWALK_SEARCH / SIDEWALK_PROBE)
    probes = [w * width for w in (0.0, 0.25, 0.5, 0.75, 1.0)]
    for i in range(steps + 1):
        t = start + direction * (CURB_GAP + i * SIDEWALK_PROBE)
        if all(not occupied(cells, x + nx * (t + direction * p),
                            y + ny * (t + direction * p))
               for p in probes):
            return t
    return None


def carriageway_edges(road):
    """(laneOffset, width of the left lanes, width of the right lanes), or None.

    OpenDRIVE lane t grows to the left, so the surface spans
    [offset - right_w, offset + left_w] about the reference line. Summing both
    sides into one width - as an earlier version did - is only right for the
    one-sided roads ZoneGraph happens to emit, and puts the sidewalk inside the
    carriageway on any road that has lanes on both sides.
    """
    ls = road.find("lanes/laneSection")
    if ls is None:
        return None

    def side_width(tag):
        total = 0.0
        for lane in ls.findall(f"{tag}/lane"):
            if lane.get("type") != "driving":
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
        for p in (p0, p1, p2, p3):
            if max(abs(p[0]), abs(p[1]), abs(p[2])) > MAX_SANE_COORD:
                self.verts.append((0.0, 0.0, 0.0))
                self.sanitized += 1
            else:
                self.verts.append(p)
        self.faces.append((base, base + 1, base + 2))
        self.faces.append((base, base + 2, base + 3))

    def write(self, path):
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("# Walkable surface for RecastBuilder\n")
            fh.write(f"# {len(self.verts)} vertices, {len(self.faces)} faces\n\n")
            for x, y, z in self.verts:
                # Recast is Y-up: swap y and z, exactly as
                # Mesh::GenerateOBJForRecast does.
                fh.write(f"v {x:.6f} {z:.6f} {y:.6f}\n")
            fh.write("\n# Polygonal face element.\n")
            gi = 0
            for fi, (a, b, c) in enumerate(self.faces):
                while gi < len(self.groups) and self.groups[gi][0] == fi:
                    fh.write(f"\nusemtl {self.groups[gi][1]}\n")
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
                    fh.write(f"f {a} {b} {c}\n")
                else:
                    fh.write(f"f {a} {c} {b}\n")
                    self.flipped += 1


def road_ribbon(road, step, t_inner, t_outer):
    """Quads for the strip between two lateral offsets along a road.

    Both the carriageway and a sidewalk are the same shape - a ribbon swept
    along the reference line between two t values - so they share one routine.
    Yields (near_inner, near_outer, far_outer, far_inner) in winding order.
    """
    prev = None
    for g in road.findall("planView/geometry"):
        gs = float(g.get("s", 0.0))
        for (x, y, hdg, ds) in geom_points(g, step):
            z = elevation_at(road, gs + ds)
            nx, ny = -math.sin(hdg), math.cos(hdg)
            inner = (x + nx * t_inner, y + ny * t_inner, z)
            outer = (x + nx * t_outer, y + ny * t_outer, z)
            if prev is not None:
                yield prev[0], inner, outer, prev[1]
            prev = (inner, outer)


def build(root, step=2.0, verbose=False, sidewalk_width=SIDEWALK_WIDTH):
    obj = ObjWriter()
    obj.material("road")

    # Roads first: the sidewalk pass needs the finished occupancy grid, and the
    # writer groups faces by the material in force when they were added.
    road_cells = set()
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
            mark_quad(road_cells, *quad)
        meshed.append((road, offset, left_w, right_w))
        n_roads += 1

    n_sw = 0
    n_sw_dropped = 0
    if sidewalk_width > 0.0:
        obj.material("sidewalk")
        for road, offset, left_w, right_w in meshed:
            # No sidewalk slabs across intersections - the crosswalks already
            # carry pedestrians over those, and a slab here would sit on the
            # junction surface itself.
            if road.get("junction", "-1") != "-1":
                continue
            for edge, direction in ((offset + left_w, +1.0),
                                    (offset - right_w, -1.0)):
                prev = None
                for g in road.findall("planView/geometry"):
                    gs = float(g.get("s", 0.0))
                    for (x, y, hdg, ds) in geom_points(g, step):
                        z = elevation_at(road, gs + ds)
                        nx, ny = -math.sin(hdg), math.cos(hdg)
                        t = clear_offset(road_cells, x, y, nx, ny, edge,
                                         direction, sidewalk_width)
                        if t is None:
                            # Boxed in on this side for this stretch. Break the
                            # ribbon rather than bridging the gap, which would
                            # lay a quad straight back over the carriageway.
                            prev = None
                            n_sw_dropped += 1
                            continue
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
            # Sample at `step` like everything else, NOT at gl/8: a 100 m
            # geometry sampled eight times puts the nearest sample up to 6.25 m
            # from the crosswalk's real s, and the whole quad moves with it.
            # Long geometries are exactly where the error is largest, so the
            # coarse rule was worst precisely where it mattered.
            base = None
            for g in geoms:
                gs = float(g.get("s", 0.0))
                gl = float(g.get("length", 0.0))
                if gs <= s_pos <= gs + gl:
                    for (x, y, hdg, ds) in geom_points(g, min(step, 1.0)):
                        if base is None or abs(gs + ds - s_pos) < base[3]:
                            base = (x, y, hdg, abs(gs + ds - s_pos))
                    break
            if base is None:
                continue
            bx, by, bhdg, _ = base
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
        print(f"  sidewalk quads   : {n_sw:,} "
              f"({n_sw_dropped:,} samples boxed in)")
        print(f"  crosswalks meshed: {n_cw:,}")
    return obj, n_roads, n_cw, n_sw


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("xodr", nargs="?")
    ap.add_argument("-o", "--output", default=None)
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
    obj, n_roads, n_cw, n_sw = build(
        root, args.step, verbose=True,
        sidewalk_width=0.0 if args.no_sidewalks else args.sidewalk_width)
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

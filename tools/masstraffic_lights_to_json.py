#!/usr/bin/env python3
"""
masstraffic_lights_to_json.py - harvest Epic's authored traffic lights out of a
UMassTrafficLightInstancesDataAsset .uasset, with no editor and no C++ rebuild.

    python tools/masstraffic_lights_to_json.py \\
        "<CitySample>/Content/AI/Traffic/TrafficLights/CitySampleSmallCityTrafficLights.uasset" \\
        -o lights-small-city.json
    python tools/masstraffic_lights_to_json.py --selftest

Why this exists: City Sample's intersections are NOT described by the ZoneGraph.
The lane graph carries only an "Intersection" tag; which sides are signalised is
decided at spawn time by UMassTrafficIntersectionSpawnDataGenerator, which reads
a separate data asset. That asset is the only place the signal layout exists, so
it is the only thing worth extracting.

The asset stores TArray<FMassTrafficLightInstanceDesc> with ordinary UE tagged
property serialisation, which is stable and trivially walkable:

    FMassTrafficLightInstanceDesc {
        FVector Position;                            // pole base, UE cm
        float   ZRotation;                           // pole yaw, degrees
        FVector ControlledIntersectionSideMidpoint;  // UE cm - the match key
        int16   TrafficLightTypeIndex;
    }

Coordinates are left untouched (UNREAL cm, left-handed). The single cm->m and
y-negation conversion happens downstream, exactly as it does for lanes.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys

# Small City ships exactly this many authored lights. A different number means
# either the wrong asset or a changed level, both worth shouting about.
EXPECTED_SMALL_CITY = 358


def _read_summary(d: bytes):
    """Return (names, total_header_size).

    Only the fields up to the name table are needed, so this stops there.
    """
    o = 0

    def i32():
        nonlocal o
        v, = struct.unpack_from("<i", d, o)
        o += 4
        return v

    def fstr():
        nonlocal o
        n = i32()
        if n >= 0:
            s = d[o:o + n - 1].decode("latin1")
            o += n
        else:
            n = -n
            s = d[o:o + 2 * n - 2].decode("utf-16-le")
            o += 2 * n
        return s

    tag, = struct.unpack_from("<I", d, 0)
    o = 4
    if tag != 0x9E2A83C1:
        raise ValueError("not a .uasset (bad package tag)")
    legacy = i32()
    i32()                       # LegacyUE3Version
    i32()                       # FileVersionUE4
    if legacy <= -8:
        i32()                   # FileVersionUE5
    i32()                       # FileVersionLicenseeUE4

    # Consume the count FIRST. `o += i32() * 20` binds `o` before evaluating the
    # right-hand side, so it silently loses the 4 bytes of the count itself and
    # every subsequent offset is wrong - the name table then reads as empty.
    num_custom_versions = i32()
    o += num_custom_versions * 20        # CustomVersions: FGuid + int32 each

    total_header = i32()
    fstr()                      # FolderName
    o += 4                      # PackageFlags
    name_count = i32()
    name_offset = i32()

    o = name_offset
    names = []
    for _ in range(name_count):
        names.append(fstr())
        o += 4                  # NonCasePreservingHash + CasePreservingHash

    if not names:
        raise ValueError("empty name table - FPackageFileSummary layout changed; "
                         "re-check the field order in _read_summary")
    return names, total_header


def parse(path: str) -> list[dict]:
    with open(path, "rb") as fh:
        d = fh.read()
    names, header_end = _read_summary(d)
    try:
        want = names.index("TrafficLights")
        arr = names.index("ArrayProperty")
    except ValueError:
        raise ValueError(f"{path} has no TrafficLights ArrayProperty - is it a "
                         "UMassTrafficLightInstancesDataAsset?")

    tag = struct.pack("<iiii", want, 0, arr, 0)
    at = d.find(tag, header_end)
    if at < 0:
        raise ValueError("TrafficLights property tag not found in export data")

    o = at + 16
    o += 8                                       # PropertySize, ArrayIndex
    o += 8                                       # inner type FName (StructProperty)
    o += 1                                       # HasPropertyGuid
    count, = struct.unpack_from("<i", d, o)
    o += 4
    o += 8 + 8 + 4 + 4 + 8 + 16 + 1              # inner struct tag + FName + guid

    def fname():
        nonlocal o
        a, _ = struct.unpack_from("<ii", d, o)
        o += 8
        return names[a]

    out = []
    for _ in range(count):
        rec = {}
        while True:
            n = fname()
            if n == "None":
                break
            t = fname()
            size, = struct.unpack_from("<i", d, o)
            o += 8                               # PropertySize, ArrayIndex
            struct_name = None
            if t == "StructProperty":
                struct_name = fname()
                o += 16                          # struct guid
            o += 1                               # HasPropertyGuid
            if t == "StructProperty" and struct_name == "Vector":
                rec[n] = list(struct.unpack_from("<ddd", d, o))
            elif t == "FloatProperty":
                rec[n], = struct.unpack_from("<f", d, o)
            elif t == "Int16Property":
                rec[n], = struct.unpack_from("<h", d, o)
            o += size
        out.append({
            "position": rec.get("Position", [0.0, 0.0, 0.0]),
            "z_rotation": rec.get("ZRotation", 0.0),
            "controlled_side_midpoint":
                rec.get("ControlledIntersectionSideMidpoint", [0.0, 0.0, 0.0]),
            "type_index": rec.get("TrafficLightTypeIndex", -1),
        })
    return out


def selftest() -> int:
    """Synthesise the exact byte layout the parser expects and read it back."""
    import os
    import tempfile

    fails = []
    names = ["None", "ArrayProperty", "StructProperty", "FloatProperty",
             "Int16Property", "Vector", "MassTrafficLightInstanceDesc",
             "Position", "ZRotation", "ControlledIntersectionSideMidpoint",
             "TrafficLightTypeIndex", "TrafficLights"]
    ix = {n: i for i, n in enumerate(names)}

    def fname(n):
        return struct.pack("<ii", ix[n], 0)

    lights = [([1.0, 2.0, 3.0], 90.0, [4.0, 5.0, 6.0], 2),
              ([-7.5, 8.25, 9.0], -45.0, [10.0, 11.0, 12.0], 0)]
    body = b""
    for pos, zr, mid, ti in lights:
        body += (fname("Position") + fname("StructProperty")
                 + struct.pack("<ii", 24, 0) + fname("Vector") + b"\0" * 16 + b"\0"
                 + struct.pack("<ddd", *pos))
        body += (fname("ZRotation") + fname("FloatProperty")
                 + struct.pack("<ii", 4, 0) + b"\0" + struct.pack("<f", zr))
        body += (fname("ControlledIntersectionSideMidpoint") + fname("StructProperty")
                 + struct.pack("<ii", 24, 0) + fname("Vector") + b"\0" * 16 + b"\0"
                 + struct.pack("<ddd", *mid))
        body += (fname("TrafficLightTypeIndex") + fname("Int16Property")
                 + struct.pack("<ii", 2, 0) + b"\0" + struct.pack("<h", ti))
        body += fname("None")

    inner = (fname("TrafficLights") + fname("StructProperty")
             + struct.pack("<ii", len(body), 0)
             + fname("MassTrafficLightInstanceDesc") + b"\0" * 16 + b"\0")
    payload = struct.pack("<i", len(lights)) + inner + body
    export = (fname("TrafficLights") + fname("ArrayProperty")
              + struct.pack("<ii", len(payload), 0) + fname("StructProperty")
              + b"\0" + payload)

    name_blob = b""
    for n in names:
        name_blob += struct.pack("<i", len(n) + 1) + n.encode() + b"\0" + b"\0" * 4

    # tag, legacy(-8), LegacyUE3, FileVersionUE4, FileVersionUE5,
    # FileVersionLicenseeUE4, then the custom-version COUNT.
    head = (struct.pack("<I", 0x9E2A83C1)
            + struct.pack("<iiiii", -8, 0, 522, 1004, 0)
            + struct.pack("<i", 0))
    folder = struct.pack("<i", 5) + b"None\0"
    prefix_len = len(head) + 4 + len(folder) + 4 + 4 + 4
    name_off = prefix_len
    head_total = name_off + len(name_blob)
    d = (head + struct.pack("<i", head_total) + folder + struct.pack("<I", 0)
         + struct.pack("<ii", len(names), name_off) + name_blob + export)

    with tempfile.TemporaryDirectory() as td:
        p = os.path.join(td, "t.uasset")
        with open(p, "wb") as fh:
            fh.write(d)
        got = parse(p)

    if len(got) != 2:
        fails.append(f"expected 2 lights, got {len(got)}")
    else:
        if got[0]["position"] != [1.0, 2.0, 3.0]:
            fails.append(f"position wrong: {got[0]['position']}")
        if abs(got[0]["z_rotation"] - 90.0) > 1e-6:
            fails.append(f"z_rotation wrong: {got[0]['z_rotation']}")
        if got[1]["controlled_side_midpoint"] != [10.0, 11.0, 12.0]:
            fails.append(f"midpoint wrong: {got[1]['controlled_side_midpoint']}")
        if got[0]["type_index"] != 2:
            fails.append(f"type_index wrong: {got[0]['type_index']}")

    if fails:
        print("SELFTEST FAILED")
        for f in fails:
            print("  -", f)
        return 1
    print("selftest passed - 2 synthetic light descs round-tripped")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("input", nargs="?", help="CitySample*TrafficLights.uasset")
    ap.add_argument("-o", "--output", default="lights.json")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.input:
        ap.error("an input .uasset is required (or --selftest)")

    lights = parse(args.input)
    if "SmallCity" in args.input.replace("_", "") and len(lights) != EXPECTED_SMALL_CITY:
        print(f"WARNING: expected {EXPECTED_SMALL_CITY} lights for Small City, "
              f"got {len(lights)}. Wrong asset, or the level changed.",
              file=sys.stderr)

    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump({"lights": lights}, fh, indent=1)

    print(f"wrote {args.output}")
    print(f"  {len(lights):,} traffic lights")
    if lights:
        xs = [l["position"][0] for l in lights]
        ys = [l["position"][1] for l in lights]
        print(f"  extent (UE cm): x {min(xs):,.0f}..{max(xs):,.0f}  "
              f"y {min(ys):,.0f}..{max(ys):,.0f}")
        mids = {tuple(round(c, 2) for c in l["controlled_side_midpoint"]) for l in lights}
        print(f"  distinct controlled intersection sides: {len(mids):,}")
        types = {}
        for l in lights:
            types[l["type_index"]] = types.get(l["type_index"], 0) + 1
        print(f"  type_index histogram: {dict(sorted(types.items()))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

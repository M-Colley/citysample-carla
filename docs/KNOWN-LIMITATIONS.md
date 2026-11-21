# Known Limitations & Technical Caveats (`v1.1.0`)

Every limitation listed here was measured on the running CARLA `1.0` server
(`Small_City_LVL` and `Big_City_LVL`), not inferred. Upstream CARLA defects
found during integration are documented with minimal reproducers in
[ISSUES-TO-FILE.md](../ISSUES-TO-FILE.md); stage-by-stage design notes live in
[IMPLEMENTATION-PLAN.md](IMPLEMENTATION-PLAN.md).

---

## 1. Active Runtime & Map Caveats

### 1.1 First semantic segmentation capture after server start
While World Partition is still streaming initial cells after server launch,
newly loaded `FastGeoStreaming` primitives briefly carry City Sample's own
material-variation values in custom primitive data float 5 before the tagger
sweeps them. CARLA's segmentation shader reads that float as a class ID (`> 29`).
- **Impact**: An immediate post-boot capture contains out-of-range label IDs.
  `tools/segmentation_census.py` detects `R > 29` and refuses to report a bogus
  percentage.
- **Mitigation**: Let the world settle (~25 s) and take a second capture. From
  the second run onward, `Unlabeled` settles at **`0.2 %`** on `Small_City_LVL`
  and **`0.4 %`** on `Big_City_LVL` (`0.00 %` out of range), and completed
  `UFastGeoContainer` objects stay cached via `FObjectKey`.

### 1.2 Speed limits and yield signs are inferred from ZoneGraph metadata
Epic's City Sample drives traffic off ZoneGraph lane tags and authors traffic
light data assets, but does not place speed-limit or yield sign actors.
- **How it is handled**:
  - `zonegraph_to_roadnetwork.py` and `roadnetwork_to_xodr.py` assign road-class
    `<speed>` limits from ZoneGraph tags: `90 km/h` (`Trunk` freeway),
    `80 km/h` (`Freeway`), `60 km/h` (ramps), and `50 / 40 / 30 km/h` across
    urban density tiers.
  - `add_signals_to_xodr.py` emits curbside US MUTCD `traffic.speed_limit.*`
    signs (`274`, `country="US"`) with `<signalReference>` across parallel lanes
    so `USpeedLimitComponent` trigger boxes update `vehicle.get_speed_limit()`
    live, plus curbside `traffic.yield` signs (`205`) at unsignalised freeway/ramp
    merges, acute slip lanes, and minor-to-major T-junction stems.
- **Caveat**: Because these limits and yield priorities are derived heuristically
  from road class and junction geometry rather than painted street signs in Epic's
  art assets, visual City Sample street props may not always show a matching sign
  mesh at the exact same spot (though CARLA spawns its own `BP_SpeedLimit*` and
  yield sign actors on the curb).

### 1.3 Mass ECS traffic is read-only and one-way aware
City Sample's ambient vehicles and crowd run in Epic's **Mass ECS**, not as
standard UE `AActor`s. When `-ExecCmds "carla.MassBridge.Enable 1"` is passed:
- **Window, not whole-city census**: `CarlaMassBridge` mirrors up to `150`
  vehicles (`carla.MassBridge.MaxProxies`) and `100` pedestrians
  (`carla.MassBridge.MaxWalkers`) nearest `role_name="hero"` (or nearest the
  spectator when no `hero` exists; `carla.MassBridge.FollowHero`).
- **Read-only proxies**: Proxies are registered in CARLA's dormant ID range
  (`>= 0x80000000`). Calling `apply_control()` or `set_autopilot()` on a `.mass.`
  actor does not steer Epic's Mass entity.
- **Asymmetric yielding with CARLA Traffic Manager**: With the `ALSM.cpp` patch
  ([carla#10366](https://github.com/carla-simulator/carla/issues/10366)),
  CARLA's Traffic Manager safely coexists with `CarlaMassBridge` and yields to
  mirrored Mass vehicles within `60 m`. However, Epic's Mass ECS vehicles do not
  know about CARLA-spawned actors and will not brake for a CARLA vehicle stopped
  in their lane. For purely CARLA-controlled traffic, leave `MassBridge` off and
  disable `Auto Spawn On Begin Play` on the level's `BP_Mass*Spawner` actors
  (keep `BP_MassTrafficIntersectionSpawner` enabled if you need visual traffic
  light meshes).

### 1.4 One-lane-per-road OpenDRIVE topology (no automatic lane changes)
Each ZoneGraph vehicle lane is exported as its own single-lane OpenDRIVE `<road>`
with a `<laneOffset>` of half the lane width (`0.04 m` centreline accuracy, zero
overlapping road meshes).
- **Impact**: Because parallel lanes on a multi-lane avenue are separate OpenDRIVE
  `<road>` elements rather than sibling `<lane>` elements under one reference
  line, CARLA's `Waypoint.get_left_lane()` / `get_right_lane()` return `None`,
  and Traffic Manager vehicles stay in their chosen lane between junctions rather
  than making mid-block lane changes.

### 1.5 `map.get_crosswalks()` lateral offset vs. navmesh crosswalks
CARLA's `Map::GetAllCrosswalkZones()` evaluates `<object type="crosswalk">`
outlines from lane `0` *after* applying `<laneOffset>` rather than from the raw
OpenDRIVE reference line. On single-lane roads with a half-lane `<laneOffset>`
(`1.75 m`), `world.get_map().get_crosswalks()` returns polygon coordinates shifted
by half a lane width along the crossing axis.
- **Impact**: Affects callers of `map.get_crosswalks()` only. `xodr_to_recast_obj.py`
  evaluates the OpenDRIVE reference line directly, so pedestrian navmesh
  crosswalk tiles align with the painted crossings.

### 1.6 Ray-cast queries beyond the World Partition streaming radius
City Sample streams building and road meshes around active streaming sources
(the spectator and `role_name="hero"`, defaulting to `carla.Streaming.HeroRadiusCm`
/ `world_settings.tile_stream_distance`).
- **Impact**: Calling `world.cast_ray()` or `ground_projection()` at coordinates
  outside the currently streamed radius hits unloaded space (or a flat `z = 0`
  fallback). Move the spectator or `hero` vehicle to the target area (and tick
  briefly in synchronous mode) before querying distant locations.

### 1.7 `Integrate-CarlaIntoCitySample.ps1 -Undo` scope
Running `tools\Integrate-CarlaIntoCitySample.ps1 -Undo` removes all copied
plugins, the `Content\Carla` junction, deployed `.xodr` / `.bin` map files, and
restores all backed-up `.uproject` / `.ini` / `.Target.cs` files inside the City
Sample project:
- Pass `-Undo -RevertCarlaPatch` to also reverse `patches/carla-1.0-citysample.patch`
  inside `$CARLA_DIR` (left untouched by default in case `$CARLA_DIR` is shared).
- The Windows `HKCU:\Software\Epic Games\Unreal Engine\Builds` registration entry
  for your CARLA UE 5.8 engine fork stays registered unless removed manually.

---

## 2. Implementation Traps & Engineering Notes

If you modify the C++ plugins or Python converters, keep these hard-won
invariants in mind:

1. **Edit `plugins/CarlaMassBridge/` or `$CARLA_DIR`, never `<CitySample>/Plugins/`**:
   The City Sample project holds copies overwritten by `Integrate-CarlaIntoCitySample.ps1`.
2. **Never cache FastGeo tagging state by raw pointer**: World Partition unloads
   and reloads `UFastGeoContainer` instances and the allocator reuses memory
   addresses. `CarlaFastGeoTagger` only memoizes completed containers and mesh
   assets via UE's `FObjectKey` (`ObjectIndex + SerialNumber`), which a reused
   pointer can never match.
3. **Never call `MarkRenderStateDirty(true)` on a `FFastGeoPrimitiveComponent`**:
   Forcing proxy recreation while `ProxyState == Creating` triggers an engine
   assertion (`FastGeoPrimitiveComponent.cpp:331`). Writing floats 4 and 5 of
   custom primitive data (`SetCustomPrimitiveDataVector4(4, ...)`) before proxy
   creation is picked up automatically when the render proxy initializes.
4. **OpenDRIVE junction IDs must be disjoint from road IDs**: CARLA's
   `RoadParser` ignores `elementType="junction"` on road `<link>` elements whenever
   a `<road>` shares the same numeric `id` ([carla#9869](https://github.com/carla-simulator/carla/issues/9869)).
   `roadnetwork_to_xodr.py` offsets all junction IDs above the maximum road ID
   and `--selftest` enforces this.
5. **Pedestrian navmesh path & client ticking**:
   - CARLA's client fetches `Content/Map/Nav/<MapName>.bin` (not `Saved/Nav/`)
     for existing UE levels ([carla#9870](https://github.com/carla-simulator/carla/issues/9870)).
   - Detour crowd simulation runs in the **Python client** inside `world.tick()` /
     `world.wait_for_tick()`; scripts that only `time.sleep()` will see walkers
     spawn and stand still.

---

## 3. Summary of Previously Known Gaps Resolved Through `v1.1.0`

| Previously broken / missing | Resolution in `v1.0.0` – `v1.1.0` | Verification tool |
|---|---|---|
| Ray-cast sensors (LiDAR, radar, `cast_ray`) returned 0 hits | Added dedicated collision channels 8–10 (`SensorObject`, `SensorTrace`, `OverlapChannel`) & ego-ignore ([carla#10365](https://github.com/carla-simulator/carla/issues/10365)) | `tools/verify_sensors.py` |
| `URayTracer` crashed on `FastGeoStreaming` hits (`Hit.GetComponent() == nullptr`) | Null-safe `Hit.GetComponent()` handling + FastGeo physics hit resolver ([carla#10364](https://github.com/carla-simulator/carla/issues/10364)) | `tools/verify_sensors.py` |
| Camera segmentation `98.8 %` `Unlabeled` & `Sky` (`6.9 %` on `Big_City_LVL`) `Unlabeled` | `CarlaFastGeoTagger` enabled by default (`carla.FastGeoTagger.Enable = 1`) + `ATagger` dot-boundary fix (`SM_dome.SM_dome` -> `Sky` `11`): `0.2 %` / `0.4 %` `Unlabeled` | `tools/segmentation_census.py` |
| Semantic LiDAR `99.1 %` -> `3.3 %` `Unlabeled` | FastGeo hit-label resolver + expanded `ATagger` rules (`Underpass` -> `Bridge`, `Map/Environment` -> `Terrain`): `1.26 %` `Unlabeled` on both maps | `tools/verify_sensors.py` |
| No traffic lights, stop signs, yield signs, or speed limits (fixed `30 km/h`) | Harvested `.uasset` lights (`355` / `2,176`) + curbside stop (`193` / `2,107`), yield (`2` / `7`), and US MUTCD speed-limit signs (`197` / `1,356`, `30–90 km/h`, `0` poles in lanes) | `tools/carla_traffic_lights.py` |
| Synthesised `2.5 m` sidewalk approximations | `xodr_to_recast_obj.py --zonegraph` meshes Epic's `7,846` / `59,384` authored pedestrian lanes (`1` connected island, `30/30` walkers moving) | `tools/carla_walkers.py` |
| `CarlaMassBridge` spectator-only & incompatible with Traffic Manager | `FollowHero` (`role_name="hero"`) + `ALSM.cpp` dormant-actor fix ([carla#10366](https://github.com/carla-simulator/carla/issues/10366)) | `tools/carla_mass_bridge.py` |
| `Big_City_LVL` unexported (`17` untagged sidewalk lanes leaked into roads) | Propagated lane types across ZoneGraph links; full `23,594`-road, `1,197.5 km` export & runtime validation | `tools/Run-CarlaCity.ps1 -Map Big_City_LVL` |

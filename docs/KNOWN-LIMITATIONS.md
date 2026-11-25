# Known Limitations & Technical Caveats

Every caveat and measurement listed here was verified on the running CARLA `1.0`
server (`Small_City_LVL` and `Big_City_LVL`), not inferred. Upstream CARLA bugs
and pull requests are tracked in [ISSUES-TO-FILE.md](../ISSUES-TO-FILE.md);
stage-by-stage design notes live in [IMPLEMENTATION-PLAN.md](IMPLEMENTATION-PLAN.md).

---

## 1. Remaining Architectural Caveats

### 1.1 World Partition streaming horizon & initial cell streaming
City Sample streams building, prop, and road geometry around active World
Partition streaming sources (`role_name="hero"` / `ego_vehicle` and the CARLA
spectator pawn, defaulting to `carla.Streaming.HeroRadiusCm` /
`world_settings.tile_stream_distance`).
- **Semantic segmentation during initial streaming**: Distant or not-yet-streamed
  World Partition HLOD proxies (`AWorldPartitionHLOD` and `FastGeoStreaming`
  HLOD1 meshes) explicitly carry `CityObjectLabel::None` (`0`, `Unlabeled`)
  rather than invalid class IDs (`0.00 %` `R > 29` from the very first run;
  `tools/segmentation_census.py` also waits for UE's `GTMaterial` post-process
  PSO to finish compiling on cold boot). While initial cells around `hero` are
  still streaming in, `Unlabeled` starts high on `Big_City_LVL` and settles at
  **`0.0 %–0.2 %`** on `Small_City_LVL` and **`0.4 %`** on `Big_City_LVL` once
  nearby cells load (~20–25 s).
- **Ray-cast queries outside streamed cells**: Calling `world.cast_ray()` or
  `ground_projection()` at coordinates beyond the active streaming radius hits
  unloaded space (or a flat fallback). Move `hero` or the spectator to the target
  area and tick briefly before querying distant coordinates.

### 1.2 Mass ECS proxies are read-only and bounded around `hero`
City Sample's ambient vehicles and crowd run in Epic's **Mass ECS**, not as
standard UE `AActor`s:
- **Spatial window**: When `-ExecCmds "carla.MassBridge.Enable 1"` is passed,
  `CarlaMassBridge` mirrors up to `150` vehicles (`carla.MassBridge.MaxProxies`)
  and `100` pedestrians (`carla.MassBridge.MaxWalkers`) nearest `role_name="hero"`
  (or nearest the spectator when no `hero` exists; `carla.MassBridge.FollowHero`).
- **Read-only control**: Proxies live in CARLA's dormant ID range
  (`>= 0x80000000`). Calling `apply_control()` or `set_autopilot()` on a `.mass.`
  actor does not steer Epic's Mass entity.
- **Two-way obstacle yielding & pure-CARLA traffic mode**:
  - By default (`carla.MassBridge.ExportCarlaObstacles 1`), live CARLA vehicles
    and walkers are published into Mass as `FMassTrafficObstacleTag` entities so
    ambient MassTraffic vehicles brake to `0 km/h` behind CARLA actors, while
    CARLA's Traffic Manager yields to nearby `.mass.` proxies ([carla#10366](https://github.com/carla-simulator/carla/issues/10366) / [PR #10369](https://github.com/carla-simulator/carla/pull/10369)).
  - To run purely CARLA-controlled traffic or walkers without Epic's ambient
    populations, pass `-ExecCmds "carla.MassBridge.DisableMassTraffic 1"` (and/or
    `"carla.MassBridge.DisableMassCrowd 1"`), which despawns Mass vehicle/crowd
    spawners while keeping intersection traffic-light visuals intact.

### 1.3 Speed-limit and yield signs vs. City Sample visual street props
Epic's City Sample drives traffic off ZoneGraph lane tags and authors traffic
light data assets, but does not place 3D speed-limit or yield sign props in its
street art.
- `zonegraph_to_roadnetwork.py`, `roadnetwork_to_xodr.py`, and
  `add_signals_to_xodr.py` derive road-class `<speed>` limits (`30–90 km/h`) and
  spawn functional CARLA `traffic.speed_limit.*` (`274`, `country="US"`) and
  `traffic.yield` (`205`) sign actors on the curb (`0` poles inside driving
  lanes), so `vehicle.get_speed_limit()` and Traffic Manager obey them even where
  Epic's static environment art had no painted sign mesh.

### 1.4 Split, merge, and junction lanes stay single-lane `<road>`s
`zonegraph_to_roadnetwork.py` groups co-parallel same-direction non-junction
vehicle lanes (matching vertex counts within `15 cm` of lateral offset and
compatible endpoints) into multi-lane OpenDRIVE `<road>`s (`236` multi-lane
roads / `23,536` lane-change waypoints on `Small_City_LVL`; `1,312` multi-lane
roads / `158,358` lane-change waypoints on `Big_City_LVL`), enabling native
`Waypoint.get_left_lane()` / `get_right_lane()` and Traffic Manager mid-block
lane changes.
- Diverging ramp tapers, turn-pocket flares (`> 15 cm` lateral divergence), and
  junction connecting roads intentionally remain 1-lane `<road>` elements so
  adjacent road meshes never overlap (`--single-lane-roads` restores legacy
  1-lane-per-road export across the entire map if desired).

---

## 2. Implementation Traps & Engineering Notes

If you modify the C++ plugins or Python converters, keep these invariants in mind:

1. **Edit `plugins/CarlaMassBridge/` or `$CARLA_DIR`, never `<CitySample>/Plugins/`**:
   The City Sample project holds copies overwritten by `Integrate-CarlaIntoCitySample.ps1`.
2. **Never cache FastGeo tagging state by raw pointer**: World Partition unloads
   and reloads `UFastGeoContainer` instances and the allocator reuses memory
   addresses. `CarlaFastGeoTagger` only memoizes completed containers and mesh
   assets via UE's `FObjectKey` (`ObjectIndex + SerialNumber`), which a reused
   pointer can never match.
3. **Never call `MarkRenderStateDirty(true)` on a `FFastGeoPrimitiveComponent`**:
   Forcing proxy recreation while `ProxyState == Creating` triggers an engine
   assertion (`FastGeoPrimitiveComponent.cpp:331`). `SetCustomPrimitiveData`
   marks render state dirty safely without forcing mid-creation proxy teardown.
4. **Mutate `FMassEntityManager` outside Mass processing scopes**: Synchronous
   `CreateEntity` / `DestroyEntity` calls assert `!EntityManager.IsProcessing()`.
   `UCarlaMassBridgeSubsystem::Tick` syncs CARLA obstacle entities on the game
   thread outside Mass processor execution and guards `IsProcessing()`.
5. **OpenDRIVE junction IDs vs. road IDs**: Stock CARLA's `RoadParser` ignores `elementType="junction"` on road `<link>` elements whenever a `<road>` shares the same numeric `id` ([carla#9869](https://github.com/carla-simulator/carla/issues/9869)). Both `patches/carla-1.0-citysample.patch` (preserving `elementType` in `RoadParser` and `MapBuilder`) and `roadnetwork_to_xodr.py` (offsetting junction IDs above the maximum road ID for unpatched stock CARLA) enforce this invariant.
6. **Pedestrian navmesh path & client ticking**:
   - Stock CARLA's `get_required_files("Nav")` ignores `Saved/Nav/<MapName>.bin` on pre-existing UE levels ([carla#9870](https://github.com/carla-simulator/carla/issues/9870)); `patches/carla-1.0-citysample.patch` fixes `get_required_files` to check `Saved/` first, and `Integrate-CarlaIntoCitySample.ps1` also deploys to `Content/Map/Nav/<MapName>.bin`.
   - Detour crowd simulation runs in the **Python client** inside `world.tick()` / `world.wait_for_tick()`; scripts that only `time.sleep()` will see walkers spawn and stand still.
7. **MassTraffic processor group ordering**: `UMassTrafficUpdateIntersectionsProcessor` runs inside `UE::MassTraffic::ProcessorGroupNames::EndPhysicsIntersectionBehavior`; any processor that actuates `bIsVehicleReadyToUseLane` before intersection evaluation must join `EndPhysicsIntersectionBehavior` alongside `ExecuteBefore` so UE's Mass dependency solver orders it ahead of the group's internal execution.

---

## 3. Previously Known Limitations Now Resolved

| Previously broken / limited | Resolution | Verification tool |
|---|---|---|
| AVs ran visible red lights in light traffic (`MassTraffic` vs. CARLA signal desync, skipped Yellow phases, and `3.0 m` stop-line boxes on `50-90 km/h` arterials) | `UCarlaTrafficLightActuationProcessor` (`EndPhysicsIntersectionBehavior`) repairs `RetimeIntersectionPeriods` flags and actuates signalised phases; `UCarlaTrafficLightSyncProcessor` (`SyncTrafficLights=1` default) freezes CARLA timers, mirrors live `FMassTrafficLight` states (`355/355` on `Small_City_LVL`, `2,175/2,176` across `619` groups on `Big_City_LVL`), extends stop-line trigger boxes to `[s_sig - 10 m, s_sig + 1 m]`, and falls back to `CARLA -> Mass` when `DisableMassTraffic 1` is set: **`0` red-light violations** | `tools/carla_traffic_lights.py --verify-sync` |
| Blinding-white RGB viewport from competing `BP_Carla_Sky` sun ([carla#9866](https://github.com/carla-simulator/carla/issues/9866)) | `ACarlaGameModeBase` detects existing non-CARLA `UDirectionalLightComponent` suns and suppresses `BP_Carla_Sky` automatically (`KeepFallbackSkyOnLitMaps=0` default): **`0.00 %` white saturation** out of the box | `tools/carla_traffic_lights.py --verify-sync` |
| `AMassVisualizer` ISM vehicles/pedestrians untagged, dormant `.mass.` proxies accepted `destroy()`, and `CarlaFastGeoTagger` re-swept unregistered containers | `UCarlaMassBridgeSubsystem` tags `AMassVisualizer` ISM components every `0.5 s`; `ActorDispatcher` and `CarlaRecorder` guard `>= DORMANT_ID_BASE`; `CarlaFastGeoTagger` filters `Container->IsRegistered()` | `tools/segmentation_census.py` |
| `get_required_files` ignored `Saved/Nav/`, `GetRandomLocation` failed without sidewalk polys, and `RoadParser` dropped `<link>` `elementType` ([carla#9869](https://github.com/carla-simulator/carla/issues/9869), [#9870](https://github.com/carla-simulator/carla/issues/9870), [#9871](https://github.com/carla-simulator/carla/issues/9871)) | All three fixed in `patches/carla-1.0-citysample.patch` (`CarlaServer.cpp`, `Navigation.cpp`, `RoadParser.cpp`, `MapBuilder.cpp`) | `tools/carla_walkers.py` |
| Ray-cast sensors (LiDAR, radar, `cast_ray`) returned 0 hits or ego self-hits | Named collision channels 8–10 (`SensorObject`, `SensorTrace`, `OverlapChannel`) & `GetOwner()` ignore ([carla#10365](https://github.com/carla-simulator/carla/issues/10365), [PR #10371](https://github.com/carla-simulator/carla/pull/10371)) | `tools/verify_sensors.py` |
| `URayTracer` crashed on `FastGeoStreaming` hits (`Hit.GetComponent() == nullptr`) | Null-safe `ATagger::GetTagOfHit` + FastGeo physics hit resolver ([carla#10364](https://github.com/carla-simulator/carla/issues/10364), [PR #10370](https://github.com/carla-simulator/carla/pull/10370)) | `tools/verify_sensors.py` |
| First-run segmentation `R > 29` (`INVALID` class IDs) & `98.8 %` `Unlabeled` | `CarlaFastGeoTagger` + `ATagger` clear HLOD `CustomPrimitiveData[4..5]` to `0`, `SM_dome` -> `Sky` (`11`), `GTMaterial` PSO warmup: **`0.00 %` `R > 29` on Run 1**, **`0.0 %–0.2 %` / `0.4 %`** settled `Unlabeled` | `tools/segmentation_census.py` |
| Semantic LiDAR `99.1 %` -> `3.3 %` `Unlabeled` | FastGeo hit-label resolver + expanded `ATagger` rules (`Underpass` -> `Bridge`, `Map/Environment` -> `Terrain`): **`1.26 %` `Unlabeled`** on both maps | `tools/verify_sensors.py` |
| No traffic lights, stop signs, yield signs, or speed limits (fixed `30 km/h`) | Harvested `.uasset` lights (`355` / `2,176`) + curbside stop (`193` / `2,109`), yield (`2` / `7`), and US MUTCD speed-limit signs (`197` / `1,357`, `30–90 km/h`, `0` poles in lanes) | `tools/carla_traffic_lights.py` |
| One-lane-per-road OpenDRIVE (`get_left_lane()` / `get_right_lane()` returned `None`, no lane changes) | Co-parallel same-direction lanes grouped into `236` / `1,312` multi-lane `<road>`s (`23,536` / `158,358` lane-change waypoints, `0` junction dead ends, live TM mid-block lane changes) | `tools/zonegraph_to_roadnetwork.py` |
| `map.get_crosswalks()` shifted by `1.75 m` (`<laneOffset>`) | `Map::GetAllCrosswalkZones()` evaluates `GetDirectedPointInNoLaneOffset` + `ApplyLateralOffset(-t)`: **`0.0000 m` median error** across `1,096` / `8,310` crosswalks ([carla#10367](https://github.com/carla-simulator/carla/issues/10367), [PR #10368](https://github.com/carla-simulator/carla/pull/10368)) | `tools/roadnetwork_to_xodr.py` |
| Asymmetric Mass / CARLA yielding & editor-only Mass traffic disable | `ExportCarlaObstacles=1` publishes CARLA actors as `FMassTrafficObstacleTag` entities (Mass brakes `50.2 -> 0.0 km/h` behind `hero`) + `DisableMassTraffic` / `DisableMassCrowd` CVars + `ALSM.cpp` fix ([carla#10366](https://github.com/carla-simulator/carla/issues/10366), [PR #10369](https://github.com/carla-simulator/carla/pull/10369)) | `tools/carla_mass_bridge.py --test-yielding` |
| Synthesised `2.5 m` sidewalk approximations | `xodr_to_recast_obj.py --zonegraph` meshes Epic's `7,846` / `59,384` authored pedestrian lanes (`1` connected island, `30/30` walkers moving) | `tools/carla_walkers.py` |
| `Integrate-CarlaIntoCitySample.ps1 -Undo` left CARLA patch & HKCU engine entry | Added `-RevertCarlaPatch` and `-RemoveEngineRegistration` switches to `-Undo` | `tools/Integrate-CarlaIntoCitySample.ps1` |

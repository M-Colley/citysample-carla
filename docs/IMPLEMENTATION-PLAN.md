> ## STATUS — all stages implemented
>
> | stage | state | evidence |
> |---|---|---|
> | **0** OpenDRIVE signals & crosswalks | **done** | 353 traffic lights / 106 junctions; 1,096 crosswalks -> 5,480 polygon points |
> | **1** Collision channels + segmentation CVar + Tagger | **done** | lidar 0 -> 232,584 points; radar 0 -> 1,980; segmentation out-of-range 8.7-100% -> 0.00% |
> | **2** Deploy the .xodr | **done** | `Saved/OpenDrive/Small_City_LVL.xodr` |
> | **3** Tagger table refinement | **done** | 40 rules; only 6 unmatched paths, all `/Engine/` meshes |
> | **4** Mass entity bridge | **done** | 248 dormant proxies in `get_actors()` - 148 MassTraffic vehicles and 100 MassCrowd pedestrians, ids `0x8000....`, 142 tracking motion |
> | **5** FastGeo tagging (beyond the plan) | **done** | every loaded FastGeo primitive tagged, 0 unmatched; unlabeled pixels 98.8% -> 0.5% |
>
> **One finding superseded the plan, and was then fixed.** Stage 3 assumed the
> remaining unlabeled geometry was HLOD proxies. It was not: City Sample streams
> its world through Epic's **FastGeoStreaming** plugin as
> `FFastGeoPrimitiveComponent` — plain C++ objects implementing
> `IPrimitiveComponent`, not `UPrimitiveComponent` UObjects — so CARLA's tagger
> could not reach them by construction. A census proved it: 7 static mesh
> components across the whole city. `CarlaFastGeoTagger` now walks the FastGeo
> containers directly and labels them.

---

All load-bearing facts re-verified against the live system. Every baseline reproduces; nothing was modified; the server is still asynchronous with 3 actors.

---

# THE IMPLEMENTATION PLAN

## 0. VERIFICATION LEDGER (commands run for this plan, output quoted)

```
$ powershell Get-CimInstance Win32_Process -Filter "Name='UnrealEditor.exe'" | Select CommandLine
"C:\carla-ue58\UnrealEngine5_carla\Engine\Binaries\Win64\UnrealEditor.exe"
"C:\Users\localadmin\Documents\Unreal Projects\CitySample\CitySample.uproject"
/Game/Map/Small_City_LVL?game=/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C
-game -carla-server -carla-rpc-port=2000 -quality-level=Epic -nosound -NoLiveCoding
-nosplash -unattended -stdout -FullStdOutLogOutput -windowed -ResX=1600 -ResY=900

$ python -c "...carla baseline..."
map Map/Small_City_LVL sync False dt 0.05
actors 3 Counter({'static.prop.mesh': 2, 'spectator': 1})
crosswalks 0
envobj 56 Counter({'NONE': 56})

$ grep -rn "ECC_GameTraceChannel" <project>/Plugins/Carla/Source/Carla/
BlueprintLibary/MeshToLandscape.cpp:390 / Sensor/Radar.cpp:169 /
Sensor/RayCastSemanticLidar.cpp:272 / Util/RayTracer.cpp:23 / Util/RayTracer.cpp:54 /
Vehicle/MovementComponents/ChronoMovementComponent.cpp:107      -> SIX sites

$ sed -n '18p' <project>/Plugins/Carla/Source/Carla/Actor/ActorRegistry.h
class FActorRegistry                       # NO CARLA_API

$ cmp <project>/.../Game/Tagger.cpp <repo>/.../Game/Tagger.cpp   -> IDENTICAL (both files)
$ grep -n "ue-header-guard" Tagger.cpp     -> 11: begin   18: end
$ grep -c "^\[/Script/Engine.CollisionProfile\]" Config/DefaultEngine.ini  -> 1
$ sed -n '137,138p' Config/DefaultEngine.ini
[SystemSettings]
r.DistanceFields.MaxPerMeshResolution=256

$ python <fixed harvester> CitySampleSmallCityTrafficLights.uasset
names 31 header_end 2049 / tag offset 2078 / ARRAY COUNT = 358 / parsed 358
first {'Position': [70753.335938, 31469.78710900001, 63.000000000007276],
 'ZRotation': 116.57168579101562,
 'ControlledIntersectionSideMidpoint': [67848.125, 33729.0234375, 70.0],
 'TrafficLightTypeIndex': 3}
distinct midpoints 358

$ python -c "o=24; def i32(): global o; o+=4; return 1; o += i32()*20"
spec form leaves o = 44 (correct is 48)          # the harvester bug, confirmed

$ sed -n '396,400p' Engine/.../DeferredShadingRenderer.cpp
static TAutoConsoleVariable<int32> CVarEnableSemSegRendering(
	TEXT("r.CARLA.EnableSegmentationRendering"), 0, ...  ECVF_RenderThreadSafe);
$ grep -n "EnableSegmentationRendering\|^\[SystemSettings\]" <carla repo>/Config/DefaultEngine.ini
21:[/Script/Engine.RendererSettings]   114:r.CARLA...=1   123:[SystemSettings]

$ grep -o "/Game/__ExternalActors__[^ \"']*HLOD[^ \"']*" logs/run-carlacity.log | sort -u | head -1
/Game/__ExternalActors__/Map/Small_City_LVL/01/AQ/AXN5Q1N9155J459BZ5HDK.StaticMesh_CitySample_HLOD1_HighQuality_0
$ grep -c "LogCollisionProfile" logs/run-carlacity.log   -> 0

$ sed -n '78,82p' Tagger.cpp  (SetStencilValue body, last statement)
  Component.SetCustomPrimitiveDataVector4(4, FVector4((float) ActorID, (float) CastEnum(Label), 0.0f, 0.0f));
      # writes ONLY custom primitive data; bSetRenderCustomDepth is ignored

$ grep -n "class AWorldPartitionHLOD" Engine/Source/Runtime/Engine/Public/WorldPartition/HLOD/HLODActor.h
69:class AWorldPartitionHLOD : public AActor, public IWorldPartitionHLODObject

$ sed -n '25,32p' CityRoadExport/Source/CityRoadExport/CityRoadExport.Build.cs
30: "Projects",   31: });   32: // World Partition types ...       # insert at 31, not 32
$ grep -n "class Road\|is_freeway\|def to_xodr" tools/roadnetwork_to_xodr.py
105-119 Road dataclass (115 is_freeway, 118 junction, 119 lane_offset_m); 256 def to_xodr; 326 n_side
$ grep -n "Junction records\|return {" tools/zonegraph_to_roadnetwork.py -> 233, 259
```

Two corrections to the critiques themselves, from the same reads:
- City Sample declares **22** collision profiles at `DefaultEngine.ini:301-322`, not twelve. The `EditProfiles` suppression list below is sized to that.
- `AWorldPartitionHLOD` exists and is public Engine API — so HLOD exclusion is implementable, contrary to "confirm the class name first".

---

## 1. ORDER — and why

| # | Stage | Build? | Restart? | Why here |
|---|---|---|---|---|
| **0** | OpenDRIVE offline pipeline (harvester + emitters + `SmallCity.xodr`) | no | no | Pure Python. Fully validatable with zero server and zero compile. Do it **while the build in Stage 1 runs.** Nothing else can start this early. |
| **1** | **Collision channels + segmentation CVar + Tagger classification** — one ini pass, one C++ pass, **one build** | **yes (build #1)** | yes | These three are independent at *verification* time but share a build. Bundling saves a full 15-min plugin rebuild. Lidar/radar is the biggest single capability unlock (they return literally zero today) and has no dependency on anything else. |
| **2** | Deploy the Stage-0 `.xodr` | no | yes | Data-only. Costs one map load, no compile. Deliberately *after* Stage 1's verification so 353 new light actors and a red-light regime do not confound the lidar and label measurements. |
| **3** | Tagger table refinement from the paths Stage 1 logged | yes (cheap) | yes | Stage 1 ships instrumentation that prints every unmatched asset path once. You cannot write the final prefix table without that output — do not try. |
| **4** | Mass entity bridge, Phase 1 | yes (multi-day) | yes | Genuinely large, touches the Carla plugin in four places, and its Phase-2 half depends on the Tagger work landing first. Last. |

**Why tagging and the Mass bridge interact, concretely:** the Mass bridge's "Phase 2" (spec 4, change 10) proposes calling `ATagger::SetStencilValue` on Mass ISM components. That is the wrong entry point — `SetStencilValue` (Tagger.cpp:82) writes only `SetCustomPrimitiveDataVector4(4, ...)`, which is **per-component**, and a Mass ISMC holds hundreds of vehicles. Per-instance tagging needs `UInstancedStaticMeshComponent::SetCustomDataValue` and a slot budget agreed with the Tagger workstream. So: land Stage 1/3 first, settle the custom-primitive-data slot allocation there, then build Phase 2 against it. Doing Phase 2 first guarantees rework.

**Why not do the OpenDRIVE work last** even though it is "just data": it is the only one of the four that needs no compiler at all, so it is free parallel work during Stage 1's build.

---

## 2. THE EDITS, IN ORDER

### ═══ STAGE 0 — OpenDRIVE signals & crosswalks (offline, no build, no restart) ═══

**0.1 [create]** `C:/Users/localadmin/Desktop/unreal city/tools/masstraffic_lights_to_json.py`

Use the spec's file **with these three mandatory fixes**:

**(a) The header walk.** In `_read_summary`, replace

```python
    o += i32() * 20             # CustomVersions (FGuid + int32)
```

with

```python
    num_custom_versions = i32()      # consume the count FIRST:
    o += num_custom_versions * 20    # `o += i32()*20` binds `o` before
                                     # evaluating the RHS and loses 4 bytes
```

Proven above: the spec form leaves `o = 44`, the correct value is 48; with the spec form the real asset yields `names 0, total_header 40` and raises `'TrafficLights' is not in list`. With the fix: `names 31, header_end 2049, ARRAY COUNT = 358`.

**(b) Escaped nulls.** Every `b"\\0"` in `selftest()` is a two-byte backslash-zero. Replace all occurrences with `b"\0"` (`b"None\0"`, `b"\0"*16`, the name-table padding). Also delete the dead first assignment to `head` (`struct.pack("<IiiiiiiI", ...)`), which is overwritten on the next line.

**(c) Guard rail.** Immediately after the name-table loop in `_read_summary`:

```python
    if not names:
        raise ValueError(f"{path}: empty name table - FPackageFileSummary "
                         "layout changed; re-check field order")
```

and in `main()`, after `lights = parse(args.input)`:

```python
    if len(lights) != 358:
        print(f"WARNING: expected 358 lights for Small_City, got {len(lights)}. "
              "Wrong asset, or the level changed.", file=sys.stderr)
```

**0.2 [edit]** `tools/zonegraph_to_roadnetwork.py` — crosswalk extraction

- After line 136 (`stats = defaultdict(int)`), add `crosswalks: list[dict] = []`.
- Insert the spec's crosswalk block **immediately before line 233** (`# Junction records, with connections resolved lane-by-lane.`), with `"corners"` removed (dead field, never read downstream).
- In the return dict beginning at line 259, add `"crosswalks": crosswalks,` next to `"junctions": junctions,`.

**0.3 [edit]** `tools/roadnetwork_to_xodr.py` — `Road` dataclass, **after line 119** (`lane_offset_m: float = 0.0`), inside the dataclass at lines 105-119:

```python
    # junction-end pose, kept in both frames: UE cm for matching against Epic's
    # light asset (authored in UE space), OpenDRIVE m for placement.
    end_ue: tuple = (0.0, 0.0, 0.0)
    end_odr: tuple = (0.0, 0.0, 0.0)
    bearing: float = 0.0
```

Populate all three in `build_road` (line 170) from the source lane's last point.

**0.4 [add]** `tools/roadnetwork_to_xodr.py` — the signal/crosswalk block, inserted **immediately before line 256** (`def to_xodr(`). Use the spec's block with these six changes:

1. **Namespace the ids.** `sid = f"s{jnum}_{si}"`, `cid = f"c{jnum}_{pi}"`, and matching `name=` attributes. Keeps `MapBuilder::CreateController`'s latent `_controllers.insert(signal)` bug (MapBuilder.cpp:980) loud rather than silently plausible.
2. **Cover every lane of a side with `<signalReference>`.** This is the behavioural fix, not a nicety: with one `<signal>` per side, only the host road gets a `RoadInfoSignal`, so `UTrafficLightComponent::InitializeSign` builds **no trigger box** on the paired lane, and `TrafficLightStage.cpp:47-77` then routes that vehicle to `AddActorToNonSignalisedJunction` — it crosses on red. ~346 of 699 approach lanes are affected. After the `<signal>` is written on `host`:

```python
        for r in members:
            if r is host:
                continue
            e = road_elems.get(r.numeric_id)
            if e is None:
                continue
            srs = e.find("signals")
            if srs is None:
                srs = ET.SubElement(e, "signals")
            n_r = 2 if r.is_freeway else 1
            sr = ET.SubElement(
                srs, "signalReference",
                s=f"{max(0.0, r.length - STOPLINE_SETBACK_M):.6f}",
                t=f"{r.lane_offset_m - n_r * r.lane_width_m - POLE_CLEAR_M:.6f}",
                id=sid, orientation="-")
            ET.SubElement(sr, "validity", fromLane=f"-{n_r}", toLane="-1")
```

3. **Fix `t` for the 160 freeway roads.** Replace `t = -(host.lane_width_m / 2.0 + POLE_CLEAR_M)` with

```python
            n_side = 2 if host.is_freeway else 1
            t = host.lane_offset_m - n_side * host.lane_width_m - POLE_CLEAR_M
```

(`lane_offset_cm=200`, `lane_width_cm=400` on every road, so `n_side=1` still gives −3.8 m; `n_side=2` gives −7.8 m instead of putting the pole 0.2 m from lane −2's centre and tripping `CheckSignalsOnRoads`' 0.7·lane_width displacement loop.) Change the signal's own validity to `fromLane=f"-{n_side}" toLane="-1"`.
4. **Close the crosswalk outline.** Replace the 4-corner loop with a 5-corner ring, first corner repeated:

```python
        ring = ((hu, hv), (hu, -hv), (-hu, -hv), (-hu, hv), (hu, hv))
        for u, v in ring:
            ET.SubElement(out, "cornerLocal", u=f"{u:.6f}", v=f"{v:.6f}", z="0.000000")
```

`Map::GetAllCrosswalkMesh` (Map.cpp:1514-1546) ends a triangle fan only on `crosswalk_vertex[start] == crosswalk_vertex[i]`; four unrepeated corners give a "crosswalk"-material mesh with **zero triangles**, and `SplitCrosswalkPolygons` warns and flushes at its 16-corner cap. Town10HD writes 5 on all 16 objects.
5. **Project onto the geometry, don't measure to its origin.** `emit_crosswalks`' `best` currently uses distance from crosswalk centre to `(g.x, g.y)`. Geometry median length is 6.13 m but p90 is 37.6 m and max 198.5 m, so a crosswalk mid-segment is up to ~99 m from that origin and gets bound to the wrong road or dropped at the 25 m cutoff. Compute `s_local` first, clamp to `[0, g.length]`, and rank on the residual `hypot(dx - s*cos(hdg), dy - s*sin(hdg))`. Grid-index the geometry origins on 50 m cells; as written it is 548 × 9891 = 5.4 M evaluations.
6. **Drop the dead code**: the `roads_by_id` parameter of `emit_signals` (never read), and the `acc` accumulator in `emit_crosswalks`.

**0.5 [edit]** `tools/roadnetwork_to_xodr.py` — `to_xodr()` wiring at line 256

Signature `def to_xodr(net, name="CitySample", lights=None, inertial=False)`. Record `road_elems[r.numeric_id] = rd` in the road loop and `junction_elems[jid] = je` in the junction loop. Then, after the junction loop and **before** `ET.indent`:

```python
    n_cw = emit_crosswalks(road_elems, roads, net.get("crosswalks", []))
    n_sig = n_ctrl = 0
    if lights:
        sides = assign_sides(roads, lights)
        n_sig, n_ctrl = emit_signals(road_elems, sides, lights,
                                     jn_map, root, junction_elems, inertial)

    # Hard gate: a <control signalId> naming a missing signal is
    # `Signals.at(SignalId)` in ATrafficLightManager::SpawnTrafficLights
    # (TrafficLightManager.cpp:617-621) -> std::out_of_range during map load,
    # i.e. the server dies. MapBuilder.cpp:877-880 also derefs find() with no
    # end() check. This must never ship.
    sig_ids = {s.get("id") for s in root.findall(".//signal")}
    ctrl_ids = {c.get("id") for c in root.findall("controller")}
    for c in root.findall("controller"):
        for ctl in c.findall("control"):
            assert ctl.get("signalId") in sig_ids, \
                f"controller {c.get('id')} -> missing signal {ctl.get('signalId')}"
    for j in root.findall("junction"):
        for cr in j.findall("controller"):
            assert cr.get("id") in ctrl_ids, \
                f"junction {j.get('id')} -> missing controller {cr.get('id')}"
    net["_signal_stats"] = {"signals": n_sig, "controllers": n_ctrl, "crosswalks": n_cw}
```

Note `emit_crosswalks` runs **first** so `<objects>` precedes `<signals>` inside each `<road>` (ASAM 1.4 order), and emit the top-level `<controller>` elements before the `<junction>` block for the same reason.

**0.6 [edit]** `tools/roadnetwork_to_xodr.py` — `main()` at line 469 and `selftest()` at line 369

Add `--lights` and `--signal-inertial` per the spec; print the three new counts. In `selftest()`, assert **5** `cornerLocal` per outline with first == last, and assert the two dangling-reference gates fire on a deliberately broken tree.

**RUN IT:**

```
cd "C:/Users/localadmin/Desktop/unreal city"
PY="C:/Users/localadmin/AppData/Local/Programs/Python/Python313/python.exe"
"$PY" tools/masstraffic_lights_to_json.py --selftest
"$PY" tools/masstraffic_lights_to_json.py "C:/Users/localadmin/Documents/Unreal Projects/CitySample/Content/AI/Traffic/TrafficLights/CitySampleSmallCityTrafficLights.uasset" -o lights-small-city.json
"$PY" tools/zonegraph_to_roadnetwork.py zonegraph-small-city.json -o roadnetwork-small-city.json
"$PY" tools/roadnetwork_to_xodr.py roadnetwork-small-city.json --name SmallCity --lights lights-small-city.json -o SmallCity.xodr
```

**EXPECT, exactly:**
```
selftest passed - 2 synthetic light descs round-tripped
wrote lights-small-city.json: 358 traffic lights
roads: 3337  junctions: 172  crosswalks: 548
3337 roads, 172 junctions
  signals   : 353
  controllers: 216
  crosswalks: 548
```
(353 sides over 106 junctions — 4×1-sided, 61×3, 40×4, 1×6 — consuming 353 of 358 lights; 216 controllers. These are measurements, not invariants: the matching uses a 2-D 400 cm radius against every approach-lane end, whereas MassTraffic uses an axis-aligned `FBox::BuildAABB(LeftMostIntersectionLaneBeginPoint, FVector(400))` with no distance cutoff. Do **not** describe this as "Epic's own association"; it is a tighter proxy for it. A drift of a few is fine; a drift of tens means the geometry changed.)

**Structural gate — run before deploying:**

```
"$PY" -c "
import xml.etree.ElementTree as ET, collections
r=ET.parse('SmallCity.xodr').getroot()
sig=r.findall('.//signal'); sref=r.findall('.//signalReference'); ctrl=r.findall('controller')
cw=r.findall('.//object[@type=\"crosswalk\"]')
print('signals',len(sig),'signalRefs',len(sref),'controllers',len(ctrl),'crosswalks',len(cw))
print('types',collections.Counter(s.get('type') for s in sig))
print('orientations',collections.Counter(s.get('orientation') for s in sig))
bad=[s.get('id') for s in sig+sref if not s.findall('validity') or all(v.get('fromLane')=='0' and v.get('toLane')=='0' for v in s.findall('validity'))]
print('refs CARLA would delete (must be 0):',len(bad))
print('signals on junction roads (must be 0):',len([1 for rd in r.findall('road') if rd.get('junction')!='-1' and rd.findall('signals/signal')]))
ids={s.get('id') for s in sig}; refs={c.get('signalId') for x in ctrl for c in x.findall('control')}
print('dangling control refs (must be 0):',len(refs-ids))
cids={c.get('id') for c in ctrl}; jrefs={c.get('id') for j in r.findall('junction') for c in j.findall('controller')}
print('controllers not referenced by a junction (must be 0):',len(cids-jrefs))
uc=[o.get('id') for o in cw if len(o.findall('outline/cornerLocal'))!=5 or (lambda c:(c[0].get('u'),c[0].get('v'))!=(c[-1].get('u'),c[-1].get('v')))(o.findall('outline/cornerLocal'))]
print('crosswalk outlines not closed (must be 0):',len(uc))
"
```
EXPECT: `signals 353 signalRefs ~346 controllers 216 crosswalks 548`; `types Counter({'1000001': 353})`; `orientations Counter({'-': 353})`; all five "must be 0" lines = 0.

---

### ═══ STAGE 1 — one build: collision channels + segmentation CVar + Tagger ═══

**STOP THE SERVER FIRST.** `Get-Process UnrealEditor | Stop-Process`. The running process has `UnrealEditor-Carla.dll` (11,262,464 bytes) mapped; building over it is `LNK1168`. Editing `Config/DefaultEngine.ini` while an editor is live also risks that editor rewriting the file on exit and dropping the hand-inserted comments.

#### 1.1 [edit] `<project>/Config/DefaultEngine.ini` — insert after line 331, before line 332

Line 331 is `+EditProfiles=(Name="CharacterMesh",CustomResponses=((Channel="Camera",Response=ECR_Ignore)))`; line 332 is `-ProfileRedirects=(OldName="BlockingVolume",NewName="InvisibleWall")`. Section header is line 282.

```ini
; --- CARLA ray-cast sensor collision channels (CARLA 0.10 / City Sample merge) ---
; City Sample owns GameTraceChannel1..7 (Ballistic, BallisticLandingTarget,
; Interactions, Footstep, VehicleComponent, VehicleFrame, VehicleWheels), so
; CARLA's own slots 1..3 are unavailable here. 8/9/10 are the next free slots.
; They must be >7: CollisionProfile.cpp sorts DefaultChannelResponses ascending
; before filling ObjectTypeMapping/TraceTypeMapping, so appending at 8/9/10
; leaves every baked EObjectTypeQuery / ETraceTypeQuery index in City Sample's
; Blueprints unchanged. Inserting below 7 would silently rewire them.
; SensorTrace defaults to ECR_Block (CARLA's own ini uses ECR_Ignore + a BlockAll
; edit); Block-by-default is what City Sample's mesh profiles actually respond to.
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel8,DefaultResponse=ECR_Ignore,bTraceType=False,bStaticObject=False,Name="SensorObject")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel9,DefaultResponse=ECR_Block,bTraceType=True,bStaticObject=False,Name="SensorTrace")
+DefaultChannelResponses=(Channel=ECC_GameTraceChannel10,DefaultResponse=ECR_Overlap,bTraceType=True,bStaticObject=False,Name="OverlapChannel")
+Profiles=(Name="CustomSensorCollision",CollisionEnabled=QueryOnly,bCanModify=True,ObjectTypeName="SensorObject",CustomResponses=((Channel="WorldStatic",Response=ECR_Ignore),(Channel="WorldDynamic",Response=ECR_Ignore),(Channel="Pawn",Response=ECR_Ignore),(Channel="Visibility",Response=ECR_Ignore),(Channel="Camera",Response=ECR_Ignore),(Channel="PhysicsBody",Response=ECR_Ignore),(Channel="Vehicle",Response=ECR_Ignore),(Channel="Destructible",Response=ECR_Ignore),(Channel="Ballistic",Response=ECR_Ignore),(Channel="BallisticLandingTarget",Response=ECR_Ignore),(Channel="Interactions",Response=ECR_Ignore),(Channel="Footstep",Response=ECR_Ignore),(Channel="VehicleComponent",Response=ECR_Ignore),(Channel="VehicleFrame",Response=ECR_Ignore),(Channel="VehicleWheels",Response=ECR_Ignore),(Channel="SensorObject",Response=ECR_Block),(Channel="SensorTrace",Response=ECR_Block)),HelpMessage="Invisible proxy geometry that only CARLA ray-cast sensors can see")
; A channel with DefaultResponse=ECR_Block is Block on EVERY profile that does not
; name it (CollisionProfile.cpp:675 assigns the global DefaultResponseContainer to
; each profile before CustomResponses). Suppress SensorTrace AND OverlapChannel on
; the non-physical query profiles: Ignore, never Overlap - LineTraceMultiByChannel
; (URayTracer::CastRay) COLLECTS overlap touches, so Overlap is not suppression.
+EditProfiles=(Name="OverlapAll",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="OverlapAllDynamic",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="OverlapOnlyPawn",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="Trigger",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="UI",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="Spectator",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="InvisibleWall",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="InvisibleWallDynamic",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="IgnoreAll",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="CapsuleWhileRagdolled",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="PawnInteraction",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="CitySampleInteraction",CustomResponses=((Channel="SensorTrace",Response=ECR_Ignore),(Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="IgnoreOnlyPawn",CustomResponses=((Channel="OverlapChannel",Response=ECR_Ignore)))
+EditProfiles=(Name="Ragdoll",CustomResponses=((Channel="OverlapChannel",Response=ECR_Ignore)))
```

Never add a **second** `EditProfiles` entry for a name that already appears — `CollisionProfile.cpp:681-688` `break`s on the first match and silently drops the rest. `Pawn` and `CharacterMesh` already have entries at lines 330-331; if they ever need SensorTrace suppression, **merge into those lines**. `Pawn` is deliberately left at Block: City Sample crowd agents are Mass-driven and may only have capsule collision, so ignoring Pawn could make every pedestrian invisible. Cost is capsule-shaped returns; tune after measurement, not before.

#### 1.2 [edit] `<project>/Config/DefaultEngine.ini` — line 137/138, `[SystemSettings]`

Insert as the first key of the section, between line 137 (`[SystemSettings]`) and line 138 (`r.DistanceFields.MaxPerMeshResolution=256`):

```ini
; CARLA segmentation sensors. The engine fork declares
; r.CARLA.EnableSegmentationRendering with a DEFAULT OF 0
; (Engine/Source/Runtime/Renderer/Private/DeferredShadingRenderer.cpp:396-400)
; and gates AddSegmentationSensorPass on it at :4213. With it at 0 the pass never
; clears or draws into SceneTextures.SegmentationBuffer, but the texture is still
; created and still bound to PPI_PostProcessInput5, so GTMaterial samples stale
; aliased RDG memory. Must live in [SystemSettings]: this is a plain
; TAutoConsoleVariable, not a URendererSettings UPROPERTY, so
; [/Script/Engine.RendererSettings] would silently drop it.
r.CARLA.EnableSegmentationRendering=1
```

This survives a re-run of `Integrate-CarlaIntoCitySample.ps1`: its step 7 (lines 291-311) only line-replaces `GameInstanceClass`, `GlobalDefaultGameMode` and `WorldSettingsClassName`.

#### 1.3 [create] `<project>/Plugins/Carla/Source/Carla/Util/CarlaTraceChannels.h`

```cpp
// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "Carla.h"                       // LogCarla

#include <util/ue-header-guard-begin.h>
#include "Engine/EngineTypes.h"
#include "Engine/CollisionProfile.h"
#include "UObject/NameTypes.h"
#include <util/ue-header-guard-end.h>

/// Collision channels used by CARLA's ray-cast sensors.
///
/// CARLA hardcoded ECC_GameTraceChannel2 / 3. Those numeric slots are correct
/// only for projects shipping CARLA's own DefaultEngine.ini; Epic's City Sample
/// already owns slots 1..7. Resolve by the NAME configured in
/// [/Script/Engine.CollisionProfile] instead - names are stable across projects -
/// and fall back to CARLA's historical slot when the name is absent.
///
/// The name lookup works in cooked and -game builds: CollisionProfile.cpp:428
/// assigns ChannelDisplayNames[EnumIndex] OUTSIDE the WITH_EDITOR guard.
/// The loop starts at GameTraceChannel1 by choice; IS_VALID_COLLISIONCHANNEL
/// (`> ECC_Destructible`) would also admit EngineTraceChannel1..6, which are not
/// project-configurable and are deliberately skipped. This fork declares
/// GameTraceChannel1..50 (EngineTypes.h:1119-1157), so 8/9/10 leaves ample room.
namespace CarlaTraceChannels
{
  inline ECollisionChannel ResolveByName(const FName& Name, ECollisionChannel Fallback)
  {
    if (const UCollisionProfile* Profile = UCollisionProfile::Get())
    {
      for (int32 Index = static_cast<int32>(ECC_GameTraceChannel1);
           Index < static_cast<int32>(ECC_OverlapAll_Deprecated);
           ++Index)
      {
        if (Profile->ReturnChannelNameFromContainerIndex(Index) == Name)
        {
          UE_LOG(LogCarla, Log, TEXT("CarlaTraceChannels: '%s' -> GameTraceChannel%d"),
                 *Name.ToString(),
                 Index - static_cast<int32>(ECC_GameTraceChannel1) + 1);
          return static_cast<ECollisionChannel>(Index);
        }
      }
    }
    UE_LOG(LogCarla, Warning,
           TEXT("CarlaTraceChannels: collision channel '%s' is NOT declared in "
                "[/Script/Engine.CollisionProfile]; falling back to the historical "
                "slot. Ray-cast sensors will return no points unless that slot "
                "happens to be correct."),
           *Name.ToString());
    return Fallback;
  }

  /// Blocking trace channel: RayCastLidar, RayCastSemanticLidar, HSSLidar,
  /// Radar, URayTracer::ProjectPoint. Declared as "SensorTrace".
  inline ECollisionChannel SensorTrace()
  {
    static const ECollisionChannel Channel =
        ResolveByName(FName(TEXT("SensorTrace")), ECC_GameTraceChannel2);
    return Channel;
  }

  /// Overlap trace channel: URayTracer::CastRay. Declared as "OverlapChannel".
  inline ECollisionChannel SensorOverlap()
  {
    static const ECollisionChannel Channel =
        ResolveByName(FName(TEXT("OverlapChannel")), ECC_GameTraceChannel3);
    return Channel;
  }
}
```

The `UE_LOG` is not decoration — it is the only positive test that works in `-game` mode (see 1.9). The silent fallback in the original spec is unobservable and returns the *broken* channel, making "config not applied" indistinguishable from "C++ not applied".

#### 1.4 [edit] six call sites (all in `<project>/Plugins/Carla/Source/Carla/`)

| File | Line | Replace the whole line with | Indent |
|---|---|---|---|
| `Sensor/RayCastSemanticLidar.cpp` | 272 | `CarlaTraceChannels::SensorTrace(),` | 4 spaces |
| `Sensor/Radar.cpp` | 169 | `CarlaTraceChannels::SensorTrace(),` | 8 spaces |
| `Util/RayTracer.cpp` | 23 | `CarlaTraceChannels::SensorOverlap(), // "OverlapChannel"` | 6 spaces |
| `Util/RayTracer.cpp` | 54 | `CarlaTraceChannels::SensorTrace(), // "SensorTrace"` | 6 spaces |
| `Vehicle/MovementComponents/ChronoMovementComponent.cpp` | 107 | `CarlaTraceChannels::SensorTrace(), // "SensorTrace"` | 6 spaces |
| `BlueprintLibary/MeshToLandscape.cpp` | 390 | `CarlaTraceChannels::SensorTrace(),` | **5 tabs** (this file is tab-indented) |

Add `#include "Carla/Util/CarlaTraceChannels.h"` after `RayCastSemanticLidar.cpp:10`, `Radar.cpp:9`, `RayTracer.cpp:10`, `ChronoMovementComponent.cpp:11`, and alongside the other `Carla/` includes at the top of `MeshToLandscape.cpp`.

`MeshToLandscape.cpp:390` is an offline mesh-to-landscape utility, not a sensor — but it compiles into the runtime module and leaving it means half the module traces `SensorTrace` and half traces `BallisticLandingTarget`, which is precisely the inconsistency this change exists to remove. Fixing all six also makes the grep a clean zero, which is the maintainable end state.

Fixing `RayCastSemanticLidar.cpp:272` covers all three lidars: `ShootLaser` is declared once (`RayCastSemanticLidar.h:51`), never overridden, and `HSSLidar.cpp:259` calls the inherited one. `ObstacleDetectionSensor` (ECC_WorldStatic) and `V2X/PathLossModel.cpp:150` (`LineTraceMultiByObjectType`) are unaffected — the sensor sweep is complete.

#### 1.5 [edit] warm the resolver on the game thread

`UCollisionProfile::Get()` (CollisionProfile.cpp:57-70) guards `LoadProfileConfig()` with a plain non-atomic `static bool bInitialized`. The magic static in `CarlaTraceChannels` is fine, but its *initializer* would first run inside `ParallelFor` (`RayCastSemanticLidar.cpp:137`). One line at the end of `ARayCastSemanticLidar::Set(...)` and `ARadar::Set(...)`:

```cpp
  (void)CarlaTraceChannels::SensorTrace();
```

#### 1.6 [edit] suppress ego self-occlusion

`RayCastSemanticLidar.cpp:141` and `Radar.cpp:34` build `FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this)` — the ignored actor is the **sensor**, not the vehicle. With `SensorTrace` at `ECR_Block` the ego's own body and wheels block the channel, and a roof-mounted lidar returns a shell of its own trunk (measured live: a spawned vehicle blocks a Block-default channel from all four sides at 5.61/6.17/7.18/7.20 m, label `Car`; 1/48 rays from a 2.4 m mount, rising sharply for `lower_fov=-45`). Immediately after each `TraceParams` construction:

```cpp
  if (AActor* Parent = GetOwner())
  {
    TraceParams.AddIgnoredActor(Parent);
  }
```

This is a no-op on stock CARLA towns (the `Vehicle` profile has empty `CustomResponses`, so under CARLA's `ECR_Ignore` default it never blocked SensorTrace anyway) — but it is a deliberate divergence, so **STEP 5's claim becomes "identical on stock towns except that ego self-returns are suppressed"**, not "bit-identical". Revert is one line.

#### 1.7 [edit] `<project>/Plugins/Carla/Source/Carla/Game/Tagger.cpp` — includes

Lines 11-18 are the `ue-header-guard` block. Add, in alphabetical position, before line 18:

```cpp
#include "HAL/IConsoleManager.h"
#include "Misc/ConfigCacheIni.h"
#include "Misc/Parse.h"
#include "WorldPartition/HLOD/HLODActor.h"
```

`FParse::Value` lives in `Misc/Parse.h`; relying on `ConfigCacheIni.h` to pull it transitively is how IWYU builds break. And add, in the `.cpp`-only includes above the guard block:

```cpp
#include "Carla/Game/CarlaGameModeBase.h"
```

#### 1.8 [edit] `<project>/Plugins/Carla/Source/Carla/Game/Tagger.cpp` — lines 28-61

Replace the whole `GetLabelByFolderName` if/else chain with the spec's `TMap` version verbatim. This is behaviour-identical including case: `FString::operator==` is `Stricmp` and `GetTypeHash(FString)` is `FCrc::Strihash_DEPRECATED`, so `TMap<FString,...>::Find` matches case-insensitively exactly as the chain did. Same 31 keys, same `None` fallback. Say so in a comment so a reviewer does not flag it.

#### 1.9 [add] `<project>/.../Game/Tagger.cpp` — after old line 61, before `SetStencilValue`

Insert the spec's data-driven block, with these changes:

- **Delete** `{ TEXT("/Game/Map/HLOD"), crp::CityObjectLabel::Static },`. `Content/Map/HLOD` holds exactly five `UHLODLayer` **settings** assets and not one static mesh; `GetLabelByPath` only ever sees a `UStaticMesh` (Tagger.cpp:142) or a `UPhysicsAsset` (:161), so that rule can never fire. The proxy meshes are subobjects of hashed external-actor packages — verified from the log: `/Game/__ExternalActors__/Map/Small_City_LVL/01/AQ/AXN5Q1N9155J459BZ5HDK.StaticMesh_CitySample_HLOD1_HighQuality_0`.
- **Add**, at the very end of the array (shortest prefixes last):
```cpp
  // World Partition HLOD proxy meshes live in the HLOD actor's own hashed
  // external package, NOT in /Game/Map/HLOD (which holds only the five
  // UHLODLayer settings assets). One merged proxy spans many semantic classes,
  // so any single label is wrong at the silhouette; Static is least-wrong, and
  // is only a fallback for standalone HLOD - TagActor skips AWorldPartitionHLOD.
  { TEXT("/Game/__ExternalActors__"),           crp::CityObjectLabel::Static     },
```
- **Delete** three rules that can never fire or are redundant: `/Game/Megascans/Surfaces` (a surface is a material, never a `UStaticMesh`), `/Game/Prop/Kit_border_RR` (already matched case-insensitively by `/Game/Prop/Kit_Border_`), and keep `/Game/Vehicle/vehTruck_` / `vehBus_` but **comment them as usually-dead**: skeletal vehicles resolve through the *physics asset* path (Tagger.cpp:161), and `Content/Vehicle/Physics` is a sibling of `vehTruck_*`, so they fall into the `/Game/Vehicle -> Car` catch-all. Label vehicles off the owning actor, not the asset path, if truck/bus distinction matters.
- **Anchor prefixes at folder boundaries.** In `GetLabelByAssetPath`, replace bare `StartsWith` with:
```cpp
      if (Path.Equals(Rule.Key, ESearchCase::IgnoreCase) ||
          Path.StartsWith(Rule.Key + TEXT("/"), ESearchCase::IgnoreCase) ||
          Path.StartsWith(Rule.Key, ESearchCase::IgnoreCase))
```
  — or better, store each prefix already normalised and require a `/` or `_` boundary. As written, `/Game/Prop` also matches a future `/Game/Props`, `/Game/City` matches `/Game/CityAnything`, `/Game/Effect` matches `/Game/Effects`. (Note the deliberate `_`-suffixed rules like `/Game/Prop/Kit_Tree_` need the raw-prefix branch.)
- **Total the sort comparator** so rule resolution is deterministic across builds (`TArray::Sort` is an unstable introsort):
```cpp
    return A.Key.Len() != B.Key.Len() ? A.Key.Len() > B.Key.Len() : A.Key < B.Key;
```
- **Use `CreateLambda`, not `CreateStatic`,** for the console command. Both compile; `CreateLambda` is idiomatic and does not break if anyone adds a capture.
- **Make `carla.RetagWorld` actually testable** by re-registering the environment-object snapshot. `get_environment_objects` reads `UObjectRegister::EnvironmentObjects`, built once by `ACarlaGameModeBase::RegisterEnvironmentObjects()` (CarlaGameModeBase.cpp:789, called at BeginPlay:326). Move that declaration from the `private:` block at `CarlaGameModeBase.h:164` into the `public:` block beside `GetEnvironmentObjects` (:78), and extend the lambda:
```cpp
          ATagger::ReloadPathLabelRules();
          ATagger::TagActorsInLevel(*World, true);
          if (ACarlaGameModeBase* GM = Cast<ACarlaGameModeBase>(World->GetAuthGameMode()))
          {
            GM->RegisterEnvironmentObjects();
          }
```
  Without this the command's log line prints while `get_environment_objects` returns stale labels — a test that passes and tells you nothing.
- **Add a `bUseBuiltinCitySampleRules` escape hatch.** These rules ship to every CARLA project through the shared plugin, and `ReloadPathLabelRules` only replaces an *exact* prefix, so a non-City-Sample project would have to override 16 `/Game/Prop/...` rules by hand. Read `GConfig->GetBool(TEXT("CarlaTagger"), TEXT("bUseBuiltinCitySampleRules"), ...)` (default `true`) and skip the compiled-in loop when false.

#### 1.10 [add] `<project>/.../Game/Tagger.cpp` — the instrumentation that makes Stage 3 possible

At the top of `TagActor` (line 130) add the HLOD skip, and after each `auto Label = GetLabelByPath(...)` add a once-per-distinct-path log for unmatched assets:

```cpp
void ATagger::TagActor(const AActor &Actor, bool bTagForSemanticSegmentation)
{
  // World Partition HLOD proxies merge many semantic classes into one mesh, so
  // any single label is wrong at every silhouette boundary. Leaving them
  // untagged lets the semantic camera fall through to the real streamed meshes.
  if (Actor.IsA(AWorldPartitionHLOD::StaticClass()))
  {
    return;
  }
```

and, next to each `SetStencilValue` call:

```cpp
    if (Label == crp::CityObjectLabel::None)
    {
      static TSet<FString> Reported;
      const FString P = Component->GetStaticMesh()
          ? Component->GetStaticMesh()->GetPathName() : TEXT("<null>");
      bool bAlready = false;
      Reported.Add(P, &bAlready);
      if (!bAlready)
      {
        UE_LOG(LogCarla, Log, TEXT("CarlaTagger: UNMATCHED \"%s\""), *P);
      }
    }
```

(same for the skeletal loop with `GetPhysicsAsset()`). Bounded by distinct paths — a few hundred lines, not thousands — and **not** behind `CARLA_TAGGER_EXTRA_LOG`, so no second build is needed to obtain it. This is the input to Stage 3.

#### 1.11 [add] `<project>/Config/DefaultGame.ini` — new section at end of file

The spec's `[CarlaTagger]` block, with the `/Game/Map/HLOD` example line deleted and `bUseBuiltinCitySampleRules=true` documented.

#### 1.12 [mirror] the repo copy

Copy **file by file** into `C:/carla-ue58/carla/Unreal/CarlaUnreal/Plugins/Carla/Source/Carla/`: the new `Util/CarlaTraceChannels.h`, plus `Sensor/RayCastSemanticLidar.cpp`, `Sensor/Radar.cpp`, `Util/RayTracer.cpp`, `Vehicle/MovementComponents/ChronoMovementComponent.cpp`, `BlueprintLibary/MeshToLandscape.cpp`, `Game/Tagger.cpp`, `Game/Tagger.h`, `Game/CarlaGameModeBase.h` — **eight files plus one new header**. All are byte-identical between the trees today (`cmp` confirms for Tagger.cpp/.h), so line numbers transfer unchanged.

Do **NOT** add the collision block to `C:/carla-ue58/carla/Unreal/CarlaUnreal/Config/DefaultEngine.ini` — it must keep SensorObject/SensorTrace/OverlapChannel on GameTraceChannel1/2/3 for CARLA's own towns. Do **NOT** re-run `Integrate-CarlaIntoCitySample.ps1` to propagate: line 187 `Remove-Tree $dst` wipes `Plugins\Carla` wholesale and lines 189-191 delete `Binaries`/`Intermediate`, forcing a full from-scratch rebuild.

Separately, `C:/carla-ue58/carla/Unreal/CarlaUnreal/Config/DefaultEngine.ini:114` has `r.CARLA.EnableSegmentationRendering=1` sitting inside `[/Script/Engine.RendererSettings]` (opens at :21) where it is silently dropped; `[SystemSettings]` opens at :123. **Moving it is a separate change with its own regression run** — it would switch on a never-executed render pass for Town01-15 for the first time, and the fork already carries a defensive comment about that path (`SegmentationSensorRendering.cpp:426-428`, skipping the Nanite raster block to avoid a NULL-buffer assertion). Do not bundle it with the City Sample fix.

#### 1.13 BUILD AND RESTART

```
"C:/carla-ue58/UnrealEngine5_carla/Engine/Build/BatchFiles/Build.bat" CitySampleEditor Win64 Development -Project="C:/Users/localadmin/Documents/Unreal Projects/CitySample/CitySample.uproject" -WaitMutex
```
Expect `Build succeeded`. Then relaunch: `powershell -File "C:/Users/localadmin/Desktop/unreal city/tools/Run-CarlaCity.ps1"`.

#### 1.14 VERIFY — three independent suites

**V1a — did the channels register?** (`grep -c "LogCollisionProfile" logs/run-carlacity.log` is **0** today, so "expect no warnings" is a null test. This is the positive test:)
```
grep -n "CarlaTraceChannels" "C:/Users/localadmin/Desktop/unreal city/logs/run-carlacity.log"
```
EXPECT exactly two `Log` lines: `'SensorTrace' -> GameTraceChannel9` and `'OverlapChannel' -> GameTraceChannel10`. A `Warning` line means the ini half did not take. Also assert still zero of `Cannot map multiple responses to the same collision channel` and `Custom Channel Name = ... hasn't been found`.

**V1b — do the sensors return data?** Spawn the ego as `role_name='hero'`:
```python
bp = bp_lib.filter('vehicle.*')[0]; bp.set_attribute('role_name','hero')
```
`CarlaEpisode.cpp:564-590` gives a hero a `TileStreamingDistance` = 3 km streaming ring (`EpisodeSettings.h:36`); a non-hero gets 200 m. The old baseline (`0/36 rays at +600 m from the spectator`) measured the *spectator's* bubble with no hero present — Risk 1 downgrades from "streaming caps range" to "streaming caps range for non-hero actors".

Pass criteria — **not** `points > 0`, which the ego's own body would satisfy:
- **Distance histogram**: a substantial fraction of points beyond ~5 m. A cloud collapsed into a 1-3 m shell is the ego car and means world geometry still ignores SensorTrace. (With 1.6 applied a shell is impossible, which makes this an even cleaner signal.)
- **Distinct object ids** from semantic lidar's object-id channel: ids other than the ego vehicle's.
- **Control**: same script with the lidar unparented, 10 m above the road. Nonzero ⇒ world geometry blocks.
- Expect `total_points` on the order of 10⁵-10⁶ per frame at default 56k pts/s over ~130 frames; `RADAR total_detections > 0` (was 0).
- **3b — over-blocking check**: histogram by distance and look for a wall of points at a fixed radius. That is the signature of an invisible proxy volume, not real geometry.

**V1c — the RayTracer half:**
```python
w.ground_projection(carla.Location(x, y, z+30), 100.0)   # EXPECT a LabelledPoint, not None (was None, 0/81)
len(w.cast_ray(a, b))                                    # 150 m horizontal ray: EXPECT > 1 (today exactly 1)
```

**V1d — segmentation encoding.** Re-run the existing probe:
```
"C:/Users/localadmin/AppData/Local/Programs/Python/Python313/python.exe" <scratchpad>/seg_probe7.py
```
BEFORE (measured, CVar at 0): `pct>29` was 8.66-15.11% at 256-512 px and **100.00%** at 600-1024 px, `distinct` 217-256 at every resolution.
PASS: `pct(R>29) == 0.00%` at every resolution **and** `distinct <= 30`, **and** two consecutive frames from a static camera are byte-identical (today 3080 pixels differ at 400x300).
**Sky will read 0, not 11.** `BP_Sky_Sphere_C_0_SM_0` is a real opaque 16384³ `UStaticMeshComponent` centred on the origin; `FSegmentationMeshPassProcessor::AddMeshBatch` (SegmentationSensorRendering.cpp:158-166) admits any non-translucent batch, so the dome rasterises over the 11/255 clear. Its mesh is under `/Engine/EngineSky/...`, which fails `Path.StartsWith(TEXT("/Game/"))` at Tagger.h:87 and has no `Static` folder, so its tag is 0. **Reading 11 is not required and its absence is not evidence of failure.**

**V1e — segmentation labels, CPU side, no rendering:**
```
python -c "import carla,collections;c=carla.Client('127.0.0.1',2000);c.set_timeout(120.0);o=c.get_world().get_environment_objects(carla.CityObjectLabel.Any);print(len(o));print(collections.Counter(str(x.type) for x in o))"
```
BEFORE (measured just now): `56` / `Counter({'NONE': 56})`.
**Realistic pass criterion**: 52 of those 56 are `CitySample_HLOD1_HighQuality_*`, which 1.10 now skips entirely. So EXPECT the 3 `StaticMeshActor_UAID_*` objects to acquire non-`NONE` labels and the count of `NONE` to drop to roughly 53/56. **Do not use "NONE well under half" — it is unreachable.** This is a weak metric by construction (the snapshot is built at BeginPlay before World Partition streams, and `OnLevelAddedToWorld` at CarlaGameModeBase.cpp:495 tags streamed cells but never re-registers them). The strong metric is V1f.

**V1f — the real classification check:**
```
grep -c "CarlaTagger: UNMATCHED" logs/run-carlacity.log
grep "CarlaTagger: UNMATCHED" logs/run-carlacity.log | sed 's/.*UNMATCHED //' | sort -u | head -60
```
Every `/Game/Road/*`, `/Game/Prop/Kit_*` and `/Game/Building/*` mesh must be **absent** from this list. What remains is the Stage 3 worklist.

**V1g — regressions:** RGB and depth at 800×600 from a fixed transform, unchanged and still valid packed depth (both work today). Record `stat unit` / server tick with no sensors, one default lidar, and one lidar at `points_per_second=1300000` — **capture this before the rebuild too**, or the regression is not attributable. Today lidar rays hit nothing and cost broadphase only; after the fix every ray does narrow-phase against complex collision on Nanite geometry (`bTraceComplex = true` at RayCastSemanticLidar.cpp:141, Radar.cpp:35). If points look suspiciously sparse, flipping those two lines to `false` isolates "Use Complex as Simple with no cooked trimesh" in one build.

**V1h — stock-town regression, as a desk check** (Town10HD is not runnable here: there is no CARLA town content in this project and the build serves `/Game/Map/Small_City_LVL`): CARLA's own ini declares `Name="SensorTrace"` on `ECC_GameTraceChannel2` (`<carla repo>/Config/DefaultEngine.ini:366`), so `ResolveByName` returns exactly the previously hardcoded value. On any CARLA-town build the V1a log line must read `'SensorTrace' -> GameTraceChannel2`. That is a one-glance regression test needing no baseline capture.

---

### ═══ STAGE 2 — deploy the .xodr (no build, one restart) ═══

```
cp "C:/Users/localadmin/Desktop/unreal city/SmallCity.xodr" \
   "C:/Users/localadmin/Documents/Unreal Projects/CitySample/Saved/OpenDrive/Small_City_LVL.xodr"
```
Restart the server — the `.xodr` is read once at map load; there is no live reload.

**VERIFY:**
```python
import carla, collections
c=carla.Client('127.0.0.1',2000); c.set_timeout(120.0)
w=c.get_world(); m=w.get_map()
tls=[a for a in w.get_actors() if a.type_id=='traffic.traffic_light']
print('traffic.traffic_light actors:', len(tls))
print('distinct opendrive ids:', len({t.get_opendrive_id() for t in tls}))
print('distinct groups:', len({tuple(sorted(g.id for g in t.get_group_traffic_lights())) for t in tls}))
cw=m.get_crosswalks(); print('crosswalk polygon points:', len(cw))
print('polygons closed:', sum(1 for i in range(0,len(cw),5) if cw[i]==cw[i+4]))
lm=tl=0
for p in m.get_spawn_points()[:400]:
    for l in m.get_waypoint(p.location).get_landmarks(120.0):
        lm+=1; tl += (l.type=='1000001')
print('landmarks:', lm, 'of which traffic lights:', tl)
print('states:', collections.Counter(str(t.state) for t in tls))
```
PASS: actors `== 353`; distinct opendrive ids `== 353`; distinct groups `== 106`; **crosswalk polygon points `== 2740`** (548 × 5, first corner repeated — *not* 2192); polygons closed `== 548`; traffic-light landmarks `> 0`; states show a mix of Red/Green/Yellow. Re-run after 30 s: the state histogram must change (`{10 s Green, 3 s Yellow, 2 s Red}` per controller, `TrafficLightController.h:150-154`, round-robin in `ATrafficLightGroup::Tick`). Baseline for all of these is 0.

**Per-lane coverage — the check that catches the trigger-box bug:**
```python
for jid in signalised_junction_ids:
    for road in approach_roads_of(jid):
        assert len(m.get_waypoint_xodr(road, -1, road_len-1.0).get_landmarks(5.0)) > 0
```
Actor count alone cannot see a missing trigger box.

**Behavioural:** a vehicle stopped at red reports `is_at_traffic_light() == True` and `get_traffic_light_state()` Red.

**Expect a visibly slower first map load** — `GetClosestTrafficSignActor` calls `UGameplayStatics::GetAllActorsOfClass` once per signal in each of `SpawnTrafficLights`' two loops **and** again in `SpawnSignals` (~3 × 353 sweeps of ~12,100 actors), plus 353 `GetClosestWaypointOnRoad` over 3337 roads, plus `GetAllReferencesToThisSignal` walking every road per light. A long BeginPlay is expected, not a hang.

**Expect `ATrafficLightManager::GetTrafficSign(id)` to return the wrong component.** `SpawnSignals` (TrafficLightManager.cpp:742) re-matches each just-spawned light (within 0.25 m, `MatchSignalAndActor` returns true for traffic lights), attaches a plain `USignComponent` under the same `SignId`, and overwrites the `UTrafficLightComponent`. Pre-existing CARLA behaviour, not introduced here — do not use it as a probe. `w.get_actors()` filtered on `traffic.traffic_light` and `get_group_traffic_lights()` are unaffected.

**Also note the per-frame cost that is exactly zero today:** `UpdateSignalGroundDormancy` returns immediately while `TrafficSigns` is empty (TrafficLightManager.cpp:473-477). With 353+ entries it runs 150 `LineTraceSingleByChannel(ECC_Visibility)` 20 m deep every frame plus `SetActorHiddenInGame` toggles, and 106 always-ticking `ATrafficLightGroup` actors. There is no `TrafficLights` streaming level in this map, so all 353 poles land in the persistent WP level and never stream out.

**And re-baseline traffic throughput.** 106 of 172 junctions switch from all-way-stop to a 30-45 s signal cycle. The existing "median 30 km/h" number will move. Measure speed and collision counts, not just actor counts.

---

### ═══ STAGE 3 — Tagger table refinement (cheap build, one restart) ═══

Read the `CarlaTagger: UNMATCHED` set from Stage 1, add one `+PathLabel=(Prefix="...", Label="...")` line to `[CarlaTagger]` in `DefaultGame.ini` **per unmatched folder** — no rebuild needed for ini-driven rules — and run `carla.RetagWorld` if you have an editor, or `client.reload_world(False)` otherwise (it re-runs BeginPlay → `TagActorsInLevel` at CarlaGameModeBase.cpp:281 **and** `RegisterEnvironmentObjects` at :326). Promote a rule into `GCitySampleDefaultRules` only when it has proven itself; that is the only part needing a rebuild.

Judgement calls in the shipped table that should be reviewed against a rendered semantic frame, not argued about in advance. As actually shipped in `GCitySampleDefaultRules`: vans → `Car` (Cityscapes convention) not `Truck`; `Kit_Border_*` → `Fences`; `Kit_Barricade_*` → `Fences`; manhole covers → `Ground`. An earlier draft of this paragraph named `Sidewalks`, `GuardRail` and `Roads` for those three - it described an intention, not the table.

**Still-open after Stage 3, and it must not be claimed as fixed:** `ATagger::TagActor` walks only `UStaticMeshComponent` and `USkeletalMeshComponent` **on AActors**. City Sample's traffic and crowd are MassEntity ISM instances with no actor at all. The ~37-53 autopilot vehicles and every pedestrian stay label 0 after both halves of the segmentation fix. That is Stage 4's Phase 2.

---

### ═══ STAGE 4 — Mass entity bridge, Phase 1 (multi-day) ═══

Apply the spec's ten changes with these ten corrections. Full text for the two non-obvious ones:

**4.1 [edit] `<carla repo>/.../Actor/ActorRegistry.h:18` — the change without which nothing links.**
```cpp
class CARLA_API FActorRegistry
```
`dumpbin /EXPORTS UnrealEditor-Carla.dll` lists 4548 exports and **zero** matching `FActorRegistry` or `FCarlaActor`; `UCarlaEpisode::GetActorRegistry` and `FindCarlaActor` *are* exported because `UCarlaEpisode` is `CARLA_API`. The bridge calls two out-of-line members — the new `RegisterDormant` and the existing `Deregister(IdType)` at ActorRegistry.cpp:142 — so both are `LNK2019` at the end of a multi-hour build. If a whole-class export drags in errors from the private `MakeCarlaActor`/`MakeFakeActor`, annotate only those two members instead. Either way, **the spec's claim "adds the missing door without touching any existing code path" is now false** — line 18 (or 42) changes.

Also add `#include "Carla/Util/BoundingBox.h"` next to ActorRegistry.h:9. `ActorInfo.h:25` declares `FBoundingBox BoundingBox;` and compiles today only through unity/PCH ordering; no header in the `ActorRegistry.h → CarlaActor.h → ActorInfo.h` chain includes it.

**4.2 [edit] `RegisterDormant` — take the proxies out of the replay id space.** Use a second counter, `static IdType MASS_ID_COUNTER;` initialised to `0x80000000u`, and **drop the `DesiredId` parameter entirely**. `ActorRegistry.cpp:70-81` (top of `Register`) adopts an existing dormant entry by grafting the incoming `AActor` onto it; the replayer supplies `DesiredId` (`CarlaReplayerHelper.cpp:75` → `ActorDispatcher.cpp:224`). Sharing `ID_COUNTER` means a replayed id can land on a live bridge proxy, and one frame later `EndFrame` deregisters the entry for a live actor, leaving an orphan in the world. A separate high range removes this outright and makes proxies recognisable client-side.

**4.3 [edit] the header include.** `MassEntityHandle.h` in this fork is a deprecated stub (`UE_DEPRECATED_HEADER(5.8, "...moved to MassCore and renamed to Mass/EntityHandle.h")`), and `MassEntityTypes.h` only pulls it inside `#if UE_ENABLE_INCLUDE_ORDER_DEPRECATED_IN_5_6`, which is compiled out because `CitySampleEditor.Target.cs` sets `IncludeOrderVersion = EngineIncludeOrderVersion.Latest`. Use `#include "Mass/EntityHandle.h"` and add `"MassCore"` to the Build.cs.

**4.4 [edit] the query setup.** `FMassEntityQuery::Initialize()` **does not exist** — the only setup paths are the constructors at `MassEntityQuery.h:80-84`. Bind in the constructor initialiser list (`: VehicleQuery(*this)`, exactly as `MassTrafficVehicleVisualizationProcessor.cpp:176` does) and delete both `VehicleQuery.Initialize(EntityManager);` and `VehicleQuery.RegisterWithProcessor(*this);` from `ConfigureQueries` — that function can be called more than once. Call `Super::ConfigureQueries(EntityManager);` first. The *signature* is right and needs no hedging: `MassProcessor.h:225` is `virtual void ConfigureQueries(const TSharedRef<FMassEntityManager>&)`; the no-arg flavour is `final` + deprecated at :353-354, so overriding it is a hard error. Delete the spec's fallback advice.

**4.5 [edit] the CVar path — the spec's own STEP 4 cannot pass as written.** Hoist the world/bridge lookup above the CVar check and always mark-and-sweep:
```cpp
  UWorld* World = Context.GetWorld();        // MassExecutionContext.h:431, not GetWorld()
  UCarlaMassBridgeSubsystem* Bridge =
      World ? World->GetSubsystem<UCarlaMassBridgeSubsystem>() : nullptr;
  if (Bridge == nullptr) { return; }

  Bridge->BeginFrame();
  if (CVarCarlaMassBridge.GetValueOnGameThread() != 0)
  {
    VehicleQuery.ForEachEntityChunk(Context, [Bridge](FMassExecutionContext& ChunkContext) { /* unchanged */ });
  }
  Bridge->EndFrame();
```
Also add a `FConsoleVariableDelegate` sink calling a new `UCarlaMassBridgeSubsystem::RemoveAll()`, so the sweep happens even if Mass stops ticking.

**4.6 [edit] risk #3 is factually inverted, and the truth is worse.** Control calls do **not** raise on dormant proxies; they silently return `Success`. `FVehicleActor::SetActorAutopilot` has a literally empty `if (IsDormant()) { }` branch; `ApplyControlToVehicle` writes `FVehicleData::Control`, which nothing consumes; `FCarlaActor::SetActorGlobalTransform` (CarlaActor.cpp:346-354) writes `FActorData` that the bridge overwrites next frame. The three `FunctionNotAvailableWhenDormant` guards the spec cites are real but on other paths (CarlaServer.cpp:1316-1320 is the V2X `send` RPC; :128 is `ResolveWalkerNavController`). Add `bool bIsMassProxy` to `FCarlaActor`, set it in `RegisterDormant`, and reject in those three methods. This raises the in-Carla footprint from "one ~45-line function" to four touched files — say so.

**4.7 [edit] the Traffic Manager will adopt every proxy, and there is no client-side-free fix.** `ALSM.cpp:60` calls `world.GetActors()` every cycle; `:121-140 IdentifyNewActors` has no type filter and no dormant check; `:290-340` then does a waypoint-vicinity search per bounding-box corner into the collision grid. `vehicle.mass.*` does not help — the factory keys on the `vehicle.` prefix and `ALSM.cpp:126` tests `GetTypeId().front() == 'v'`. Choose explicitly: (i) ship Phase 1 capped at 150 proxies with a documented "do not run TM with the bridge on", or (ii) budget the LibCarla + PythonAPI egg rebuild for a prefix skip at `ALSM.cpp:135`. Do not leave this as "untested".

**4.8 [edit] guard the recorder and `destroy_actor`.** In `ACarlaRecorder::Tick` (CarlaRecorder.cpp:109-110), `AddExistingActors` (:756-758) and `FFrameData::GetFrameData` (FrameData.cpp:38-40), `continue` on `View->IsMassProxy()`. While there, change `FActorRegistry Registry = Episode->GetActorRegistry();` (CarlaRecorder.cpp:755, FrameData.cpp:1223) to a `const&` — a by-value deep copy of three TMaps is pointless today and pathological at 500-3000 entries. In `UActorDispatcher::DestroyActor` (ActorDispatcher.cpp:175-186), refuse on `IsMassProxy()`: today it skips `Actor->Destroy()` when `GetActor()` is null and returns `true`, so the standard `apply_batch([DestroyActor(x) for x in world.get_actors().filter('vehicle.*')])` cleanup reports success on hundreds of proxies which reappear next frame with fresh ids.

**4.9 [edit] Build.cs and the coordinate contract.** Add `bUseRTTI = true;` next to `bEnableExceptions` — `CarlaTools.Build.cs:25-26` sets both, and `FCarlaActor::GetActorData<T>()` (CarlaActor.h:168-176) is a `dynamic_cast`. Document that `FActorData::Location` is **global** (large-map) space: `CarlaActor.cpp:190-195` subtracts the large-map origin on the read path. Correct today (zero `LargeMapManager` hits in the 1.7 MB log), wrong the moment a large map is used.

**4.10 [rewrite] the Phase 2 stub.** Delete the `CustomDepthStencilValue != 0` guard (dead code — `SetStencilValue` never writes that field, verified above) and the inverted LIMITATION paragraph. Per-**instance** ids on a Mass ISMC are reachable via `NumCustomDataFloats` + `UInstancedStaticMeshComponent::SetCustomDataValue`; `SetCustomPrimitiveDataVector4` is the per-component API and is the wrong entry point. First verify City Sample's Mass vehicle materials are not already consuming those slots, and reconcile slot indices with Stage 1/3.

**Bandwidth, pinned exactly** (not "roughly 130 bytes"): `ActorDynamicState.h:147-152` `static_assert`s `sizeof == 119u` and `WorldObserver.cpp:315` sizes the buffer as `Registry.Num() * 119`. 500 proxies @ 20 Hz = 1.19 MB/s; 3000 = 7.14 MB/s, memcpy'd on the game thread each tick, up from ~400 B/s.

**Client-side leak, verified:** `CachedActorList.h:29` carries the literal comment `/// @todo Dead actors are never removed from the list.`, and only `OnEpisodeStarted()` clears it. Every recycled Mass entity mints a new id, so at ~10² recycles/s that is ~360k permanently-retained `rpc::Actor` records per hour per client.

**Resolve the internal contradiction:** the spec's RISKS promise a 500-proxy cap and a `carla.MassBridge.MaxProxies` radius filter that appear in none of changes 5-8, while STEP 3 asserts `len(v) > 200`. Either implement the cap and assert `200 < len(v) <= cap`, or delete the promise.

**Rewrite STEP 3's motion assert** — a single actor can be stopped at a red light, and off-LOD entities update under `SetChunkFilter(...ShouldTickChunkThisFrame)` (MassTrafficInterpolationProcessor.cpp:45). Use an aggregate over 200 sampled proxies with `try/except` around each `get_transform` (ids churn), passing at >25% moved.

---

## 3. WHAT NOT TO DO

**Do not use `grep LogCollisionProfile` as the channel-registration test.** There are zero such lines in the current, broken log, so "expect no errors" passes unchanged. Worse, `CollisionProfile.cpp:715` (`Custom Channel Name '%ls' hasn't been found`) fires only for a *CustomResponses* entry naming an unknown channel; a malformed `+DefaultChannelResponses` line logs nothing at all. Use the `CarlaTraceChannels:` log line instead.

**Do not fall back to "check Project Settings > Engine > Collision in the editor."** The server runs `-game`; there is no editor UI in the process under test, and a separate editor launch is a step nobody budgeted.

**Do not use `points > 0` as the lidar pass criterion.** With `SensorTrace` at `ECR_Block` the ego's own body blocks the channel unconditionally (the ignore-actor is `this`, the sensor, not the vehicle), so a lidar bolted to a car reports tens of thousands of self-hits while every piece of City Sample world geometry still ignores the channel.

**Do not renumber the collision channels, or insert below slot 7.** `DefaultChannelResponses` is sorted ascending before filling `ObjectTypeMapping`/`TraceTypeMapping`, and City Sample's 1-7 are all `bTraceType=False`. 8/9/10 append cleanly; anything else silently rewires every baked `EObjectTypeQuery` index in City Sample's Blueprints.

**Do not use `ECR_Overlap` to "suppress" a channel.** It is invisible to `LineTraceSingleByChannel` but is exactly what `LineTraceMultiByChannel` (`URayTracer::CastRay`) collects. `Ignore` is the suppression verb for both `SensorTrace` and `OverlapChannel`. Leaving `OverlapAll` at Overlap makes `cast_ray` return CARLA's own `RoutePlanner` (RoutePlanner.cpp:60) and `FrictionTrigger` (FrictionTrigger.cpp:19) volumes plus every City Sample trigger, with an `ATagger` call per hit.

**Do not expect `EditProfiles` to catch everything.** Components with `CollisionProfileName = "Custom"` are rebased from the global `DefaultResponseContainer` at `BodyInstance.cpp:300` and get `SensorTrace = Block` with **no** `EditProfiles` recourse — only per-asset editing. City Sample uses Custom collision extensively for audio/nav/camera proxies. Symptom: a wall of points at a fixed radius.

**Do not add a second `EditProfiles` entry for a name that already has one.** `CollisionProfile.cpp:681-688` `break`s on the first match and silently drops the rest. Merge into the existing line.

**Do not write `/Game/Map/HLOD` as a Tagger prefix.** That folder holds five `UHLODLayer` settings assets and no mesh; `GetLabelByPath` only ever sees a `UStaticMesh` or a `UPhysicsAsset`. The rule can never fire, and 52 of the 56 environment objects depend on it.

**Do not accept "NONE well under half" as the label pass criterion.** Unreachable: 93% of the measured population is HLOD proxies which the plan now deliberately leaves untagged.

**Do not treat "sky reads 11" as proof the segmentation pass runs.** `BP_Sky_Sphere_C_0` is a real opaque mesh that rasterises over the clear with tag 0. The spec's own RISK 2 then sends you to RenderDoc for a problem that does not exist.

**Do not write `<validity fromLane="0" toLane="0"/>`.** Every stock town does (Town03 38/38, Town05 54/54 traffic-light signals, Town10HD 19/21), and `MapBuilder::RemoveZeroLaneValiditySignalReferences` (MapBuilder.cpp:1059) deletes exactly those, taking the trigger box and every landmark with them. Stock towns get away with it because their lights are hand-placed actors that `SpawnTrafficLights` adopts. And do not *omit* `<validity>` either: `MapBuilder.cpp:1070-1072` spares an empty set, but `TrafficLightComponent.cpp:37` then iterates nothing and builds no box, and `GenerateDefaultValiditiesForSignalReferences` (MapBuilder.cpp:1046) is an **empty stub**. Write `-1..-1` explicitly.

**Do not copy a stock Town `.xodr` as a template.** It is the most likely way to reintroduce the 0/0 validity bug.

**Do not put a signal on a junction (connecting) road.** `SpawnTrafficLights`' fallback loop requires `!IsJunction(Signal->GetRoadId())`, and the controller loop is the only other path.

**Do not use `"positive"`/`"negative"`/`"none"` for signal orientation.** `RoadInfoSignal::GetOrientation` compares against the literal `"+"` and `"-"`; anything else falls through to Both and generates validities for lanes that do not exist.

**Do not "fix" `MapBuilder::CreateController`'s `it->second->_controllers.insert(signal)` (MapBuilder.cpp:980) without also fixing parse order, or vice versa.** It inserts the *SignId* where a ContId belongs, and compiles because both are `std::string`. It is inert today only because `AddSignal` writes `_temp_signal_container` (MapBuilder.cpp:273) and `_map_data._signals` is filled only at `Build()` (:865), after `ControllerParser`. Reorder either and `GetControllers().at(ControllerId)` throws `std::out_of_range` and takes the server down.

**Do not build proxy AActors for Mass entities.** Strictly dominated: identical per-frame fragment→transform copy, plus UE actor overhead, plus ~10² `SpawnActor`/`Destroy` per second from MassTraffic recycling. Memory is not the problem (~4 MB for 3000); churn is.

**Do not register Mass *AActors*.** `UMassTrafficVehicleVisualizationTrait` gives at most ~50 real pawns (`LODMaxCount` High=10, Medium=40) and which 50 churns every few frames. Mirror by **entity**, never by actor — and keep it that way, or anything that later adds an `OnActorSpawned` auto-register double-counts.

**Do not run `Integrate-CarlaIntoCitySample.ps1` to re-sync a source edit.** Line 187 `Remove-Tree $dst` deletes `Plugins\Carla` wholesale, lines 189-191 delete `Binaries`/`Intermediate`, and the four generated `.def` files that `Carla.Build.cs` reads from `PluginDirectory` (lines 53-93) go with it. Copy the edited files by hand. It also fails outright on locked files while the server is up.

**Do not edit the `Desktop/unreal city/CityRoadExport` tree expecting it to build.** It is a manual byte-identical mirror of `<project>/Plugins/CityRoadExport` (confirmed by `cmp`), and the integrator has zero references to it.

**Do not add `"MassTraffic"` to `CityRoadExport.Build.cs` without adding `{"Name": "Traffic", "Enabled": true}` to `CityRoadExport.uplugin`'s `Plugins` array** — its current array holds only `ZoneGraph`. Missing plugin dependency is a UBT error, not a link error. And insert the module at line **31** (before `});`), not line 32 which is a comment.

**Do not take the commandlet route for the traffic lights at all in this pass.** The Python harvester is verified working against the real asset (358 lights, exact first record), needs no rebuild, and the byte layout is plain UE5 tagged-property serialisation with a stable 49-byte inner struct tag. Keep the commandlet as a documented fallback for when the asset changes shape.

**Do not use `ATrafficLightManager::GetTrafficSign(id)` as a probe** after Stage 2 — `SpawnSignals` overwrites the `UTrafficLightComponent` with a plain `USignComponent` under the same key.

**Do not call `ATagger::SetStencilValue` on a Mass ISM component.** It writes `SetCustomPrimitiveDataVector4(4, ...)`, which is per-component; every vehicle instance in that ISMC would get one label.

**Do not claim vehicle or pedestrian segmentation works** after either segmentation fix. Mass agents are ISM instances with no AActor and `TagActor` never sees them.

**Do not enable synchronous mode to test any of this.** The server is asynchronous (`sync False, dt 0.05`); leave it that way.

---

## 4. REBUILD / RESTART POINTS

| Point | Rebuild | Restart | Why |
|---|---|---|---|
| Stage 0 (all of it) | no | no | Pure Python + a `.uasset` read. Runs against a live server it never touches. |
| **Before Stage 1's ini edits** | — | **stop server** | `UnrealEditor-Carla.dll` is mapped by PID; a live editor may also rewrite `DefaultEngine.ini` on exit and drop hand-inserted comments. |
| **Stage 1** | **BUILD #1** — `Build.bat CitySampleEditor Win64 Development -Project=... -WaitMutex` | **yes** | Config collision channels are read once by `UCollisionProfile` at startup; `r.CARLA.EnableSegmentationRendering` is applied from `[SystemSettings]` at startup; the six trace sites and the Tagger are C++. `-NoLiveCoding` is already on the launcher's command line, so "Live Coding is not enough" is correct but moot here. |
| Stage 2 | **no** | **yes** | The `.xodr` is read once at map load; there is no live reload path. |
| Stage 3, ini-only rules | no | `carla.RetagWorld` (needs an editor) or `client.reload_world(False)` | `reload_world` re-runs BeginPlay → `TagActorsInLevel` **and** `RegisterEnvironmentObjects`. |
| Stage 3, promoting a rule into `GCitySampleDefaultRules` | yes (Carla module only) | yes | C++ table. |
| Stage 4 | BUILD #2+ (Carla module + new plugin) | yes | New module, new `.uproject` plugin entry, changed Carla headers. |

There is **no `-ExecCmds` and no interactive console** in this deployment — `Run-CarlaCity.ps1` builds `$args` at lines 75-92 with `-game -carla-server ... -NoLiveCoding` and redirects stdout to a file. Any CVar this plan needs at runtime must go in `Config/DefaultEngine.ini [SystemSettings]`, which is why 1.2 is written that way and why "type it in the editor console" is not a step anyone can execute here. If a launch-time override is ever wanted, add `"-ExecCmds=<cvar> <value>",` to that `$args` array after `"-NoLiveCoding",`.

**Total: two builds and four restarts for Stages 0-3.** Stage 4 adds its own.

---

## 5. HONEST SCOPE

**Small and safe — do these first.**
- **LiDAR/radar channel remap**: ~30 lines of ini, one new 60-line header, six one-line C++ substitutions plus five includes, two `AddIgnoredActor` calls. Blast radius on CARLA internals is genuinely tiny — `grep -rn "URayTracer::"` finds callers only at `CarlaServer.cpp:3633` and `:3652`, so nothing in spawning, the traffic manager or the recorder can regress. Half a day plus one build. **This is the highest value-per-line item in the whole set** and it is the reason it goes first.
- **Segmentation CVar**: eleven lines of ini. Costs the main viewport, RGB and depth exactly zero — the gate is `CVar && ViewFamily.bRequiresSegmentationPass`, and that flag is set in exactly two places (`SemanticSegmentationCamera.cpp:32`, `InstanceSegmentationCamera.cpp:37`). No new shader compiles and no new mesh pass: `EMeshPass::SegmentationPass` draw commands are already built for every view regardless (`SceneVisibility.cpp:1779, 1921, 2446-2447`). Substrate is not a hazard (`SegmentationBuffer` is created under `bIsUsingGBuffers`, which is `!IsForwardShadingEnabled`). Ten minutes plus a restart. **Its one real uncertainty is that no completed A/B exists** — the "disabled pass is the sole cause" conclusion is a strong inference from six converging measurements. If V1d does not collapse the histogram, the next move is a RenderDoc capture of one segmentation frame to see what `PPI_PostProcessInput5` actually resolves to.

**Medium.**
- **Tagger classification**: a ~250-line data-driven replacement, an HLOD skip, and instrumentation. One to two days including the two build cycles and the table-tuning pass. The table itself encodes reviewable judgement calls, and 93% of the obvious metric is HLOD noise — budget time for looking at rendered semantic frames, not for arguing about prefixes.
- **OpenDRIVE signals and crosswalks**: one new 250-line Python file, ~200 lines added to two existing tools. All of it validatable offline in under 5 seconds of runtime (the full 548 × 9891 crosswalk sweep is ~2 s unindexed). One to two days, **zero builds**. The genuine risks are downstream and behavioural, not in the tooling: the per-lane `<signalReference>` coverage (without it ~50% of approach lanes cross on red), the closed 5-corner outline (without it every crosswalk mesh has zero triangles), and the load-time / throughput changes at 106 junctions.

**Genuinely large — the Mass bridge.** The spec's "2-3 days for Phase 1" is optimistic and its framing that "the only in-Carla edit is generic and upstreamable" no longer holds. With the corrections, Phase 1 touches the Carla plugin in **five** places, not one: `ActorRegistry.h/.cpp` (export + `RegisterDormant` + separate id counter), `CarlaActor.h/.cpp` (`bIsMassProxy` + three control-path rejections), `ActorDispatcher.cpp` (destroy guard), `CarlaRecorder.cpp` + `FrameData.cpp` (skip + the by-value registry copies), and — if you want the Traffic Manager to behave — **LibCarla plus a new Python egg**. Call it **4-6 days**, and the TM decision is a fork in the road that must be made before you start, not after.

**Smallest useful first increment for the Mass bridge**, which I would ship on its own and stop:

> `RegisterDormant` with its **own** id counter starting at `0x80000000`, `bIsMassProxy`, the plugin, the processor and the subsystem, **vehicles only**, **read-only**, **capped at 150 proxies** with a radius filter around the LOD viewer, CVar-defaulted **off**, with the always-sweep fix so turning it off truly returns the registry to baseline, and with the recorder and `destroy_actor` guards in place. Ship it with the documented constraint "**do not run the Traffic Manager with the bridge enabled**" rather than paying for the LibCarla change up front.

That alone turns `world.get_actors()` returning 3 into "the traffic you can see, with correct transforms, velocities, bounding boxes and `Car` labels" — which is the entire point of the workstream — for about 40% of the full cost, and it leaves the TM integration, pedestrians, per-instance ISM tagging and the bounding-box sensor as clean, separately-schedulable follow-ons.

**One thing to say plainly about the fourth workstream that the spec does not:** even fully delivered, CARLA's traffic lights and MassTraffic's traffic lights remain two independent systems reading the same authored data and showing, in general, different phases at the same intersection. Nothing in this plan reconciles them. That is a fifth problem.
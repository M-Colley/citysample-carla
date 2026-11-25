# AGENTS.md

Notes for an AI agent working in this repository. Written by one, after the
mistakes below were made and paid for. Read this before editing anything.

## What this repository is

Tooling that runs Epic's **City Sample** (Unreal Engine 5.8) as a **CARLA
1.0** server, and extracts its road network as multi-lane ASAM OpenDRIVE. It
contains no Epic content and no data derived from it; see [LICENSE](LICENSE).

The three trees you will touch:

| tree | what it is | authoritative? |
|---|---|---|
| this repo | tools, docs, `plugins/CarlaMassBridge`, `patches/` | **yes** |
| your CARLA checkout (`$CARLA_DIR`) | CARLA source; the patch is applied *to* it | yes, for CARLA's own files |
| the City Sample project (`$CITYSAMPLE_DIR`) | where everything is **copied to** and built | **no — it is a build output** |

## The mistake that will cost you the most time

**The City Sample project holds COPIES.** `Integrate-CarlaIntoCitySample.ps1`
copies the CARLA plugins and `plugins/CarlaMassBridge` into it. Edit a file
under `<CitySample>/Plugins/...` and the next integrate silently overwrites it —
or worse, you edit there, rebuild, and the build *succeeds* while your change is
absent, because you edited a copy that was never compiled.

This happened three times during the original work. Twice it was diagnosed as a
bug in the code being edited.

| to change | edit here | then |
|---|---|---|
| traffic-light sync, Mass bridge, FastGeo tagger | `plugins/CarlaMassBridge/` | re-integrate, **rebuild** |
| anything in CARLA's `Carla` plugin or `LibCarla` | your CARLA checkout, then regenerate `patches/carla-1.0-citysample.patch` | re-integrate, **rebuild** |
| a Python tool | `tools/` | nothing, it runs from here |
| a PowerShell script | `tools/` | nothing |

`Run-CarlaCity.ps1` warns when either plugin source is newer than the project's
copy. Do not ignore that warning.

## The loop

```bash
powershell -File tools\Integrate-CarlaIntoCitySample.ps1   # copies + patches (upgrades older patches cleanly)
powershell -File tools\Build-CarlaCity.ps1                 # ~45 s for CarlaMassBridge, ~15 min for full Carla
powershell -File tools\Run-CarlaCity.ps1 -ExecCmds "carla.MassBridge.Enable 1"
```

Three things about this loop:

- **Stop the server before integrating.** The running editor holds
  `UnrealEditor-Carla.dll` open; the delete fails halfway and leaves the project
  half-integrated.
- **Integrating deletes the plugins' `Binaries/`.** You must rebuild afterwards,
  even if you changed nothing in C++. Skipping it gives you an editor that
  cannot load the `Carla` module, and `-unattended` suppresses the dialog that
  would tell you.
- **A C++ change is a ~15 minute round trip.** Batch your edits. Before
  rebuilding, re-read what you changed and check it against the headers you are
  calling — a build spent on a typo is fifteen minutes gone.

## Verifying

Never claim something works without running the tool that measures it. Each of
these prints numbers, not a green tick:

```bash
python tools/verify_sensors.py                           # lidar, radar, raytracer, segmentation
python tools/carla_walkers.py --walkers 30               # navmesh, spawning, actual movement
python tools/carla_traffic_lights.py --watch 32 --verify-sync
python tools/carla_mass_bridge.py --test-yielding        # Epic's traffic/crowd + two-way braking
python tools/segmentation_census.py --settle 25          # RUN IT TWICE, see below
python tools/carla_probe.py                              # what the server is actually serving
```

Every Python converter has `--selftest` and needs no server.

**Measurements that lie if you take them once:**

- **`segmentation_census.py`**: World Partition cells around `hero` take ~20-25 s
  to finish streaming on cold boot (`Unlabeled` starts high on `Big_City_LVL` and
  settles at `0.0 %-0.4 %` once cells register, with `0.00 %` `R > 29` from Run 1).
  Run it twice.
- **`carla_traffic_lights.py --watch`**: a window under ~25 s can sit inside
  one phase and report "static" for lights that are cycling.
- **Anything spatial**: the Mass bridge mirrors the entities nearest
  `role_name="hero"` (falling back to the spectator). Keep `hero` or the spectator
  parked in a streamed district before counting.
- **Walkers that move** — `carla_walkers.py` passes when walkers walk, and a
  walker on an island walks. Whether pedestrians can reach the rest of the city
  is `xodr_to_recast_obj.py`'s `walkable : N island(s)` line; sidewalks that
  stopped at every junction once made each block side its own island.
- **Traffic survival over a short run** — a vehicle dies only when its path
  reaches a dead end, so a short `carla_validate_xodr.py` run (7.5 s by
  default) passed a network in which no junction could be entered. Read its `junction entries`
  and `route reach` lines, not just who is alive.

## Traps that have already been paid for

Each of these produced a wrong diagnosis before the real cause was found. They
are in the code comments too; this is the index.

- **`-ExecCmds` splits on commas, not `|`.** Joining with `|` does not error: UE
  runs the whole string as one command, sets the first CVar from a garbage
  argument that `atoi`s to 1, and silently drops the rest. Symptom: one feature
  works, the other is absent, nothing logged.
- **Never cache "already done" against a raw pointer.** World Partition unloads
  and reloads containers, and the allocator reuses addresses. A `TSet` of raw
  component or container pointers reports new, untagged geometry as finished.
  `CarlaFastGeoTagger` only caches registered containers and static mesh labels
  by `FObjectKey` (`ObjectIndex + SerialNumber`), which a reused address cannot
  match.
- **`MarkRenderStateDirty(true)` on a FastGeo component crashes the server.** It
  queues a component whose `ProxyState` is `Creating` for recreation and trips an
  assertion. It is also unnecessary. Do not "fix" its absence.
- **Native stderr redirection is terminating under `$ErrorActionPreference =
  'Stop'`.** `& git ... 2>$null` in PowerShell 5.1 wraps stderr in an
  ErrorRecord and kills the script before `$LASTEXITCODE` is read. Wrap native
  calls in a temporary `'Continue'`.
- **Capture world settings BEFORE mutating them.** `settings` is one mutable
  object; reading `synchronous_mode` back after writing it returns what you just
  wrote, and the "restore" then re-asserts the state it was meant to undo.
- **Walker AI is simulated in the CLIENT.** Detour steps inside
  `world.tick()` / `world.wait_for_tick()` and nowhere else. A script that
  `sleep`s sees every walker spawn and then stand still — indistinguishable from
  a broken navmesh.
- **A junction id must never equal a road id.** CARLA's `RoadParser` keeps a
  link's `elementId`, drops its `elementType`, and then treats the link as a
  junction only if no ROAD has that id. Junctions numbered 0..171 beside roads
  0..3336 made every junction approach a dead end, and the traffic manager
  silently destroys any vehicle whose path reaches one (`osm_mode` defaults to
  on). That looked like "`set_autopilot(True)` destroys the vehicle" and was
  reported upstream as a CARLA bug (issue 9) before the cause was found.
  `roadnetwork_to_xodr.py --selftest` now checks it.

## Rules

1. **Never commit City Sample-derived data.** No `zonegraph*.json`,
   `roadnetwork*.json`, `lights*.json`, `*.xodr`, `*.obj`, `*.bin`. They are
   Epic's content once derived. `.gitignore` covers them; check `git status`
   before committing anyway. The one deliberate exception is
   `docs/citysample-carla.{mp4,gif}`, which the owner chose to include.
2. **Regenerate `patches/carla-1.0-citysample.patch` with
   `git diff HEAD --output=...`** from the CARLA `1.0` (`3ea166118d2c`) checkout
   (never use PowerShell `>` redirection, which writes UTF-16LE BOM), and verify
   with `git apply --reverse --check`. `.gitattributes` marks `*.patch` `-text`
   so git never rewrites its line endings.
3. **Keep the vendored plugin in sync.** After editing
   `plugins/CarlaMassBridge/`, the project copy is downstream. `diff -r` them if
   a change seems not to take effect.
4. **Do not widen a claim past its measurement.** "0.4 % unlabeled" is a number
   from one viewpoint at one moment. Say which, or say "about".
5. **Report failures as failures.** The README's value is that its numbers are
   real. If a check fails, say so with the output.

## Where things are

```
tools/                     Python converters and CARLA client checks; PowerShell drivers
plugins/CarlaMassBridge/   traffic-light sync, Mass->CARLA bridge, and FastGeo semantic tagger
CityRoadExport/            UE plugin, -run=ExportZoneGraph
patches/                   CARLA 1.0 patch (carla-1.0-citysample.patch) + legacy 0.10 patch
config/                    [CarlaTagger] label rules, appended to the project's DefaultGame.ini
docs/                      KNOWN-LIMITATIONS.md, IMPLEMENTATION-PLAN.md, showcase videos
ISSUES-TO-FILE.md          13 CARLA issues, 12 filed upstream (#9869 corrected)
```

Read `README.md` §8 and `docs/KNOWN-LIMITATIONS.md` for remaining caveats before
assuming you found something new.

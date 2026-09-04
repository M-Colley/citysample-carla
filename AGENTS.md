# AGENTS.md

Notes for an AI agent working in this repository. Written by one, after the
mistakes below were made and paid for. Read this before editing anything.

## What this repository is

Tooling that runs Epic's **City Sample** (Unreal Engine 5.8) as a **CARLA
0.10.0** server, and extracts its road network as ASAM OpenDRIVE. It contains
no Epic content and no data derived from it — see [LICENSE](LICENSE).

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
| Mass bridge, FastGeo tagger | `plugins/CarlaMassBridge/` | re-integrate, **rebuild** |
| anything in CARLA's `Carla` plugin | your CARLA checkout, then regenerate `patches/` | re-integrate, **rebuild** |
| a Python tool | `tools/` | nothing, it runs from here |
| a PowerShell script | `tools/` | nothing |

`Run-CarlaCity.ps1` warns when either plugin source is newer than the project's
copy. Do not ignore that warning.

## The loop

```bash
powershell -File tools\Integrate-CarlaIntoCitySample.ps1   # copies + patches
powershell -File tools\Build-CarlaCity.ps1                 # ~15 min for C++
powershell -File tools\Run-CarlaCity.ps1 -ExecCmds "carla.FastGeoTagger.Enable 1","carla.MassBridge.Enable 1"
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
python tools/verify_sensors.py            # lidar, radar, raytracer, segmentation
python tools/carla_walkers.py             # navmesh, spawning, actual movement
python tools/carla_traffic_lights.py --watch 30
python tools/carla_mass_bridge.py         # Epic's traffic and crowd
python tools/segmentation_census.py --settle 25   # RUN IT TWICE, see below
python tools/carla_probe.py               # what the server is actually serving
```

Every Python converter has `--selftest` and needs no server.

**Measurements that lie if you take them once:**

- **`segmentation_census.py`** — the first run after a server start is
  meaningless and the tool says so. World Partition is still streaming, and
  untagged geometry renders with whatever City Sample left in its custom
  primitive data, which the shader reads as a nonsense label. Run it twice.
- **`carla_traffic_lights.py --watch`** — a window under ~25 s can sit inside
  one phase and report "static" for lights that are cycling.
- **Anything spatial** — the Mass bridge mirrors the entities nearest the
  *spectator*. Park the spectator before counting.

## Traps that have already been paid for

Each of these produced a wrong diagnosis before the real cause was found. They
are in the code comments too; this is the index.

- **`-ExecCmds` splits on commas, not `|`.** Joining with `|` does not error: UE
  runs the whole string as one command, sets the first CVar from a garbage
  argument that `atoi`s to 1, and silently drops the rest. Symptom: one feature
  works, the other is absent, nothing logged.
- **Never cache "already done" against a pointer.** World Partition unloads and
  reloads containers, and the allocator reuses addresses. A `TSet` of tagged
  component pointers reports new, untagged geometry as finished. One viewpoint
  measured between 0 % and 92 % unlabeled depending on how streaming had cycled.
  The FastGeo sweep is deliberately stateless; leave it that way.
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
- **`set_autopilot(True)` destroys the vehicle** on this map, reproducibly, with
  no collision and nothing logged. Drive waypoints instead; see `RouteDriver` in
  `tools/record_carla.py`. Reported upstream.

## Rules

1. **Never commit City Sample-derived data.** No `zonegraph*.json`,
   `roadnetwork*.json`, `lights*.json`, `*.xodr`, `*.obj`, `*.bin`. They are
   Epic's content once derived. `.gitignore` covers them; check `git status`
   before committing anyway. The one deliberate exception is
   `docs/citysample-carla.{mp4,gif}`, which the owner chose to include.
2. **Regenerate `patches/carla-ue58-plugin-build-fixes.patch` with
   `git diff HEAD`** from the CARLA checkout, and verify with
   `git apply --reverse --check`. It must stay byte-exact — `.gitattributes`
   marks it `-text` so git never rewrites its line endings.
3. **Keep the vendored plugin in sync.** After editing
   `plugins/CarlaMassBridge/`, the project copy is downstream. `diff -r` them if
   a change seems not to take effect.
4. **Do not widen a claim past its measurement.** "0.4 % unlabeled" is a number
   from one viewpoint at one moment. Say which, or say "about".
5. **Report failures as failures.** The README's value is that its numbers are
   real. If a check fails, say so with the output.

## Where things are

```
tools/            Python converters and CARLA client checks; PowerShell drivers
plugins/CarlaMassBridge/   the Mass->CARLA bridge and the FastGeo semantic tagger
CityRoadExport/   UE plugin, -run=ExportZoneGraph
patches/          19 files of CARLA fixes, applied to your CARLA checkout
config/           [CarlaTagger] label rules, appended to the project's DefaultGame.ini
docs/             the implementation plan and the showcase video
ISSUES-TO-FILE.md nine CARLA bugs, four filed upstream
```

Read `README.md` §10 for what is known-broken before assuming you found
something new.

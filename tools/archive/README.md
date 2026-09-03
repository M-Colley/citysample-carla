# Archive

Kept for the record, not part of the pipeline.

| File | Why it is here |
|---|---|
| `ue_export_roadnetwork.py` | First attempt at extracting the road network from editor Python. Superseded by the `CityRoadExport` C++ plugin: `FZoneGraphStorage` is a plain `UPROPERTY()` USTRUCT, so Python cannot reach it. |
| `ue_probe_zonegraph.py` | The probe that established the above. Run against a live UE 5.8.2 editor. |
| `carla_diagnose_motion.py` | One-off: separated horizontal from vertical motion to prove an early "338 m in 6 s" reading was the spawn drop, not driving. |
| `install-netfx-sdk.bat` | Broken. `cmd` expands `%VAR%` when it parses an `if (...)` block, so the `)` inside `C:\Program Files (x86)\...` closed the block early. Replaced by `Install-NetFxSdk.ps1`. |

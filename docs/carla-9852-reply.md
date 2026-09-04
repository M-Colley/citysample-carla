# Reply for carla#9852

Paste as a comment on
<https://github.com/carla-simulator/carla/issues/9852>. Trim the tail if it
reads long — the first three paragraphs carry it.

---

I have this working, and the tooling is public:
**https://github.com/M-Colley/citysample-carla** (MIT).

It runs Epic's City Sample as a CARLA 0.10.0 server against the CARLA fork of
UE 5.8, and separately exports the city's road network to OpenDRIVE so it can
be driven in any CARLA. No Epic content is redistributed — the tools regenerate
everything from your own City Sample install in a few minutes.

**Road network.** City Sample has no OpenDRIVE, and its road data is not in a
form CARLA reads: the drivable layout lives in Epic's ZoneGraph. A small UE
plugin exports it (`-run=ExportZoneGraph`), and Python converts lanes → roads +
junctions → `.xodr`. On `Small_City_LVL` that is **3,337 roads, 172 junctions,
145 km, 2,861 spawn points**, with `paramPoly3` endpoint continuity checked to
2.45 × 10⁻⁵ m across 5,452 joins. The traffic manager routes on it.

Two things mattered more than expected. One road per *lane*, not per lane group
— grouping them made adjacent roads' generated meshes overlap and trapped
vehicles, costing about half the spawn points. And lane-level `<link>` elements,
not just road-level: road links alone made things worse (34/40 vehicles
despawned); adding `<lane><link><successor>` took survival to 41/58.

**Hosting the level itself** needed rather more, because several CARLA features
assume a map CARLA authored:

- *Ray-cast sensors returned nothing.* CARLA hardcodes `ECC_GameTraceChannel2`,
  which in City Sample is a different channel with `DefaultResponse=Ignore`, so
  every ray passed through the world. Resolving the channel **by name** gives
  232,584 lidar points where there were 0, and 1,980 radar detections where
  there were 0.
- *Semantic segmentation labelled almost nothing.* City Sample streams its
  geometry through Epic's FastGeoStreaming as `FFastGeoPrimitiveComponent` —
  plain C++ objects, not `UPrimitiveComponent` UObjects, not owned by actors —
  so `ATagger` cannot reach them by construction. A census inside `ATagger`
  found **seven** static mesh components in the entire city. Walking the
  FastGeo containers directly took the camera from 98.8 % `Unlabeled` to
  **0.5 %**.
- *Semantic LiDAR was still unlabeled* for a related but distinct reason: a
  ray-cast sensor reads the hit's `UPrimitiveComponent` tag, and FastGeo owns
  its `FBodyInstance`s through `IPhysicsBodyInstanceOwner` instead.
  `FFastGeoPhysicsBodyInstanceOwner::FromHitResult` resolves it. 99.1 % → 3.3 %.
- *No traffic lights.* The signal layout is not in the ZoneGraph; it is a
  separate data asset. Parsing it yields **353 lights across 106 junctions**,
  phased so opposing approaches share a green, plus **1,096 crosswalks**. The
  other 66 junctions are stop-sign intersections — MassTraffic's own
  `bHasTrafficLights` distinguishes them — so those get **229 stop signs**.
- *`world.get_actors()` could not see the traffic.* City Sample's vehicles and
  pedestrians are Mass ECS entities, not actors, so the camera renders hundreds
  of cars the API cannot see. A bridge mirrors the nearest ones in as dormant
  proxies.
- *Pedestrians could not spawn or move*, silently, for three independent
  reasons — see the issues below.

**Bugs found on the way.** Four are already filed (#9863, #9864, #9865, #9866).
The rest are written up in the repo, and two are worth a maintainer's attention
because together they make walker navigation fail silently on *any* pre-existing
map:

- `HasServerSideNavigation()` returns true for a `RecastNavMesh` with zero
  tiles, and the client cannot fall back — so a perfectly valid
  `Saved/Nav/<Map>.bin` becomes unreachable and every walker API returns
  nothing, with no log line on either side.
- `FNavigationMesh::Load` and `get_required_files("Nav")` search different
  directories, so a navmesh in the place the first documents is ignored by the
  path that actually runs.

Also: `Navigation::GetRandomLocation` filters on `CARLA_TYPE_SIDEWALK` alone,
so a navmesh built from an OpenDRIVE file without `type="sidewalk"` lanes is
valid, has hundreds of tiles, and still cannot place a single pedestrian.

Happy to open a PR for any of the CARLA-side fixes if that is useful — they are
in the repo as a single patch against `ue58-dev-carla`, and none of them are
City Sample-specific except by motivation.

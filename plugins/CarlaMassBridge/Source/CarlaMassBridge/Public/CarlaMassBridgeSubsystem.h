// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
// FMassEntityHandle moved to MassCore in 5.8: MassEntityHandle.h and
// MassEntityTypes.h are deprecated stubs that compile out entirely at
// IncludeOrderVersion.Latest, which this project uses.
#include "Mass/EntityHandle.h"
#include "MassArchetypeTypes.h"
#include "UObject/ObjectKey.h"

#include "CarlaMassBridgeSubsystem.generated.h"

class UCarlaEpisode;

/// Mirrors Epic's MassTraffic entities into CARLA's actor registry.
///
/// WHY THIS EXISTS
/// ---------------
/// City Sample's traffic is Mass ECS entities, not Actors.
/// UMassTrafficVehicleVisualizationTrait only ever gives ~50 of them a real
/// AActor (LODMaxCount High=10, Medium=40), and *which* 50 churns every few
/// frames as the LOD viewer moves. So `world.get_actors()` reported 3 actors
/// while the camera rendered hundreds of cars: every one of those was a silent
/// false negative in any ground-truth label a student generated.
///
/// HOW
/// ---
/// CARLA's registry does not need an AActor for the *lifetime* of an entry,
/// only at creation. FCarlaActor already supports a fully actorless
/// representation - carla::rpc::ActorState::Dormant with TheActor == nullptr -
/// and FWorldObserver::BroadcastTick already serialises transform, velocity and
/// vehicle data straight out of FActorData for those. So we register one
/// dormant proxy per Mass entity and refresh its FActorData each frame.
///
/// LIMITS (read before enabling)
/// -----------------------------
///  * These proxies are READ-ONLY. Control calls on them do not fail loudly:
///    FVehicleActor::SetActorAutopilot has an empty dormant branch and still
///    returns Success, and ApplyControlToVehicle writes to FActorData that this
///    subsystem overwrites on the next frame. Treat them as observations.
///  * Disabled by default. Enable with `carla.MassBridge.Enable 1`, which is
///    read every frame so it can be flipped at runtime; turning it off
///    deregisters every proxy, returning the registry to its baseline.
UCLASS()
class CARLAMASSBRIDGE_API UCarlaMassBridgeSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    //~ Begin USubsystem
    virtual void Deinitialize() override;
    //~ End USubsystem

    //~ Begin FTickableGameObject
    /// Game-thread tick outside Mass processing scopes: syncs live CARLA vehicles
    /// and walkers into MassTraffic obstacle entities (`carla.MassBridge.ExportCarlaObstacles`)
    /// so Epic's ambient Mass traffic brakes for CARLA actors, and applies runtime
    /// `carla.MassBridge.DisableMassTraffic` / `DisableMassCrowd` toggles.
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(UCarlaMassBridgeSubsystem, STATGROUP_Tickables);
    }
    virtual bool IsTickable() const override;
    //~ End FTickableGameObject

    /// Where to measure "near" from, in world space.
    ///
    /// Prefers an active `role_name=hero` / `ego_vehicle` actor when
    /// `carla.MassBridge.FollowHero` is non-zero (default), falling back to the
    /// CARLA spectator pawn and then the world origin.
    FVector GetObserverLocation() const;

    /// Called by the processor before it sweeps entities.
    void BeginFrame();

    /// What kind of CARLA actor an entity is mirrored as. City Sample runs two
    /// independent Mass populations - MassTraffic vehicles and the MassCrowd
    /// pedestrians - and the CARLA client builds a different Python class from
    /// each type-id prefix, so the two cannot share one description.
    enum class EProxyKind : uint8
    {
        Vehicle,
        Walker,
    };

    /// Register (or refresh) the proxy for one Mass entity.
    void SyncEntity(FMassEntityHandle Entity, const FTransform &Transform,
                    const FVector &Velocity,
                    EProxyKind Kind = EProxyKind::Vehicle);

    /// Deregister proxies whose entity was not seen this frame.
    void EndFrame();

    /// Drop every proxy. Called when the CVar goes to 0 and on Deinitialize,
    /// so switching the bridge off truly restores the baseline registry rather
    /// than freezing hundreds of stale actors in it.
    void RemoveAllProxies();

    int32 NumProxies() const { return EntityToActorId.Num(); }
    int32 NumExportedObstacles() const { return CarlaActorToObstacleEntity.Num(); }

private:
    UCarlaEpisode *GetEpisode() const;
    void SyncCarlaObstaclesToMass();
    void RemoveAllObstacleEntities();
    void ApplyMassPopulationToggles();
    void TagDynamicMassVisualizers();

    /// Mass entity -> CARLA actor id.
    TMap<FMassEntityHandle, uint32> EntityToActorId;

    /// Live CARLA actor id -> Mass obstacle entity (`FMassTrafficObstacleTag`).
    TMap<uint32, FMassEntityHandle> CarlaActorToObstacleEntity;
    FMassArchetypeHandle ObstacleArchetype;

    bool bMassTrafficDisabledApplied = false;
    bool bMassCrowdDisabledApplied = false;

    /// Last known pose, used to differentiate a velocity. MassTraffic vehicles
    /// have no FMassVelocityFragment, so there is nothing to read directly.
    struct FMassProxyMotion
    {
        FVector Location = FVector::ZeroVector;
        double Time = 0.0;
    };
    TMap<FMassEntityHandle, FMassProxyMotion> EntityMotion;

    /// Entities seen during the current sweep.
    TSet<FMassEntityHandle> SeenThisFrame;

    /// Cached active hero actor id for GetObserverLocation().
    mutable uint32 CachedHeroId = 0;
    mutable double LastHeroScanTime = -1.0;

    /// Component counts of AMassVisualizer actors already tagged for semantic segmentation.
    TMap<FObjectKey, int32> TaggedMassVisualizers;
    double LastVisualizerTagTime = -10.0;

    /// Logged once when the bridge begins mirroring entities.
    bool bWarnedAboutTrafficManager = false;
};

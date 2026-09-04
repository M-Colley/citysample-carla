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
///  * DO NOT run CARLA's Traffic Manager with the bridge enabled. The TM calls
///    world.GetActors() every cycle and ALSM adopts any unregistered actor whose
///    type id starts with 'v', with no dormant check - so it would take every
///    proxy as an unregistered vehicle and run a waypoint search per bounding
///    box corner for each one.
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

    //~ Begin FTickableGameObject - the processor drives the sweep, not Tick.
    virtual void Tick(float DeltaTime) override {}
    virtual TStatId GetStatId() const override
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT(UCarlaMassBridgeSubsystem, STATGROUP_Tickables);
    }
    virtual bool IsTickable() const override { return false; }
    //~ End FTickableGameObject

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

private:
    UCarlaEpisode *GetEpisode() const;

    /// Mass entity -> CARLA actor id.
    TMap<FMassEntityHandle, uint32> EntityToActorId;

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

    /// Logged once, so a student who forgets the Traffic Manager caveat sees it.
    bool bWarnedAboutTrafficManager = false;
};

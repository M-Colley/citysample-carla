// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Mass/EntityHandle.h"
#include "MassProcessor.h"
#include "UObject/ObjectKey.h"
#include "UObject/WeakObjectPtr.h"

#include "CarlaTrafficLightSyncProcessor.generated.h"

class ATrafficLightBase;
class ATrafficLightGroup;
class UTrafficLightComponent;
struct FZoneGraphTrafficLaneData;

/// Runs in EndPhysics immediately BEFORE UMassTrafficUpdateIntersectionsProcessor
/// (after MassTrafficMovement cleared bIsVehicleReadyToUseLane in DuringPhysics)
/// so signalised intersections never skip their 3 s Yellow clearance phase or
/// strobe across periods in light/empty ambient Mass traffic.
UCLASS()
class CARLAMASSBRIDGE_API UCarlaTrafficLightActuationProcessor : public UMassProcessor
{
    GENERATED_BODY()

public:
    UCarlaTrafficLightActuationProcessor();

protected:
    virtual void ConfigureQueries(const TSharedRef<FMassEntityManager> &EntityManager) override;
    virtual void Execute(FMassEntityManager &EntityManager,
                         FMassExecutionContext &Context) override;

private:
    FMassEntityQuery IntersectionQuery;
    TSet<FMassEntityHandle> RepairedIntersections;
    TWeakObjectPtr<UWorld> ActuatedWorld;
};

/// Synchronises Epic's MassTraffic intersection signals (`FMassTrafficIntersectionFragment`
/// and `FMassTrafficLight`) with CARLA's OpenDRIVE traffic light actors (`ATrafficLightBase`,
/// `UTrafficLightComponent`, and `ATrafficLightGroup`).
///
/// WHY THIS EXISTS
/// ---------------
/// City Sample renders its overhead traffic lights through Mass ECS (`UMassTrafficUpdateIntersectionsProcessor`
/// and `UMassTrafficLightUpdateCustomVisualizationProcessor`), whereas CARLA drives vehicles
/// and `vehicle.get_traffic_light_state()` from `ATrafficLightGroup` timers spawned out of
/// OpenDRIVE `<signal type="1000001">` records. Worse, Epic's intersections are demand-actuated
/// (`FZoneGraphTrafficLaneData::bIsVehicleReadyToUseLane`) and ignore CARLA vehicles, so in
/// light ambient traffic Epic's visible overhead lights stay red or skip phases while CARLA's
/// independent timer turns green.
///
/// MODES (`carla.MassBridge.SyncTrafficLights`, default 1)
/// -------------------------------------------------------
///   0 : Off (independent MassTraffic and CARLA timers).
///   1 : Mass -> CARLA (default). Freezes CARLA's `ATrafficLightGroup` timers, actuates
///       signalised Mass intersections for approaching CARLA vehicles and within camera
///       range so phases cycle cleanly even in light traffic, and mirrors Epic's live
///       `Red` / `Yellow` / `Green` state onto CARLA's `UTrafficLightComponent` every frame.
///       Automatically falls back to CARLA -> Mass visual sync when MassTraffic vehicles
///       are despawned (`carla.MassBridge.DisableMassTraffic 1`).
///   2 : CARLA -> Mass. Unfreezes CARLA's `ATrafficLightGroup` timers (and honours Python
///       `traffic_light.set_state(...)` / `freeze(...)` calls) while writing CARLA's live
///       signal state onto Epic's overhead `FMassTrafficLight` instances and intersection lanes.
UCLASS()
class CARLAMASSBRIDGE_API UCarlaTrafficLightSyncProcessor : public UMassProcessor
{
    GENERATED_BODY()

public:
    UCarlaTrafficLightSyncProcessor();

protected:
    virtual void ConfigureQueries(const TSharedRef<FMassEntityManager> &EntityManager) override;
    virtual void Execute(FMassEntityManager &EntityManager,
                         FMassExecutionContext &Context) override;

private:
    struct FSignalBinding
    {
        TWeakObjectPtr<ATrafficLightBase> CarlaLight;
        TWeakObjectPtr<UTrafficLightComponent> CarlaComponent;
        FMassEntityHandle IntersectionEntity;
        int32 TrafficLightIndex = INDEX_NONE;
        TArray<FZoneGraphTrafficLaneData *> ControlledLanes;
        double YellowStartedWorldTime = -100.0;
        double HoldYellowUntilWorldTime = -1.0;
    };

    void RebuildBindingsIfNeeded(UWorld *World,
                                 FMassEntityManager &EntityManager,
                                 FMassExecutionContext &Context);
    void SetManagedGroupsFrozen(bool bFreeze);
    void ApplyDuplicatePoleVisibility(bool bHideDuplicates);

    FMassEntityQuery IntersectionQuery;

    TArray<FSignalBinding> Bindings;
    TSet<TWeakObjectPtr<ATrafficLightGroup>> ManagedGroups;
    TSet<FObjectKey> ResizedTriggerBoxes;
    TWeakObjectPtr<UWorld> BoundWorld;
    int32 BoundCarlaLightCount = 0;
    int32 BoundMassIntersectionCount = 0;
    double LastRebuildAttemptTime = -10.0;
    bool bGroupsFrozenBySync = false;
    bool bDuplicatesHidden = false;
};

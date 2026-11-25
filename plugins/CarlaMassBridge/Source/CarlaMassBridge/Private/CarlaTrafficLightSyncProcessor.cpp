// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaTrafficLightSyncProcessor.h"

#include "CarlaCitySampleLog.h"
#include "CarlaMassBridgeSubsystem.h"

#include "Carla/Actor/ActorRegistry.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Traffic/TrafficLightBase.h"
#include "Carla/Traffic/TrafficLightComponent.h"
#include "Carla/Traffic/TrafficLightGroup.h"
#include "Carla/Traffic/TrafficLightState.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include "Components/BoxComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/IConsoleManager.h"

#include "MassCommonFragments.h"
#include "MassEntityManager.h"
#include "MassExecutionContext.h"
#include "MassTraffic.h"
#include "MassTrafficFragments.h"
#include "MassTrafficSettings.h"
#include "MassTrafficSubsystem.h"
#include "MassTrafficUpdateIntersectionsProcessor.h"
#include "ZoneGraphSubsystem.h"
#include "ZoneGraphTypes.h"

namespace
{
  static TAutoConsoleVariable<int32> CVarCarlaSyncTrafficLights(
      TEXT("carla.MassBridge.SyncTrafficLights"),
      1,
      TEXT("Synchronise Epic's MassTraffic intersection lights and CARLA's traffic.traffic_light actors.\n"
           "  0: off (independent timers)\n"
           "  1: Mass -> CARLA with CARLA-vehicle demand actuation (default; auto-falls back to CARLA -> Mass when MassTraffic vehicles are disabled)\n"
           "  2: CARLA -> Mass (CARLA controllers and Python set_state() drive City Sample overhead lights and lanes)"),
      ECVF_Default);

  static TAutoConsoleVariable<int32> CVarCarlaActuateSignalisedIntersections(
      TEXT("carla.MassBridge.ActuateSignalisedIntersections"),
      1,
      TEXT("Keep signalised MassTraffic intersections actuated before EndPhysics so "
           "every phase executes its full Green -> Yellow -> Red cycle even in light "
           "or empty ambient Mass traffic.\n"
           "  0: off\n"
           "  1: on (default)"),
      ECVF_Default);

  static TAutoConsoleVariable<int32> CVarCarlaHideDuplicateTrafficLights(
      TEXT("carla.MassBridge.HideDuplicateCarlaTrafficLights"),
      0,
      TEXT("Hide CARLA's OpenDRIVE-spawned traffic light pole meshes when bound to an authored "
           "City Sample overhead MassTraffic light (preserving stop-line trigger boxes).\n"
           "  0: keep CARLA poles visible and state-synced (default)\n"
           "  1: hide bound CARLA pole meshes"),
      ECVF_Default);

  struct FApproachBoxSample
  {
    FVector Pos = FVector::ZeroVector;
    FVector Dir2D = FVector::ForwardVector;
  };

  bool ExtractApproachSamples(const UTrafficLightComponent *Comp,
                              const ATrafficLightBase *Light,
                              TArray<FApproachBoxSample> &OutSamples,
                              FVector &OutMeanPos,
                              FVector &OutMeanDir2D)
  {
    OutSamples.Reset();
    OutMeanPos = FVector::ZeroVector;
    OutMeanDir2D = FVector::ZeroVector;

    if (Comp != nullptr)
    {
      const TArray<UBoxComponent *> Boxes = Comp->GetEffectTriggerVolume();
      for (const UBoxComponent *Box : Boxes)
      {
        if (!IsValid(Box))
        {
          continue;
        }
        const FVector Loc = Box->GetComponentLocation();
        FVector Dir = Box->GetForwardVector();
        Dir.Z = 0.0;
        if (!Dir.Normalize())
        {
          continue;
        }
        OutSamples.Add(FApproachBoxSample{Loc, Dir});
        OutMeanPos += Loc;
        OutMeanDir2D += Dir;
      }
    }

    if (OutSamples.Num() > 0)
    {
      OutMeanPos /= static_cast<double>(OutSamples.Num());
      if (!OutMeanDir2D.Normalize())
      {
        OutMeanDir2D = OutSamples[0].Dir2D;
      }
      return true;
    }

    if (IsValid(Light))
    {
      OutMeanPos = Light->GetActorLocation();
      // ATrafficLightManager::SpawnTrafficLights adds +90 deg yaw to OpenDRIVE signal orientation.
      FVector Dir = (Light->GetActorRotation() - FRotator(0.0f, 90.0f, 0.0f)).Vector();
      Dir.Z = 0.0;
      if (Dir.Normalize())
      {
        OutMeanDir2D = Dir;
        OutSamples.Add(FApproachBoxSample{OutMeanPos, OutMeanDir2D});
        return true;
      }
    }
    return false;
  }

  bool GetZoneLaneStartAndDir(const FZoneGraphStorage &Storage,
                              int32 LaneIndex,
                              FVector &OutStart,
                              FVector &OutDir2D)
  {
    if (!Storage.Lanes.IsValidIndex(LaneIndex))
    {
      return false;
    }
    const FZoneLaneData &Lane = Storage.Lanes[LaneIndex];
    if (Lane.PointsEnd - Lane.PointsBegin < 2 ||
        !Storage.LanePoints.IsValidIndex(Lane.PointsBegin) ||
        !Storage.LanePoints.IsValidIndex(Lane.PointsBegin + 1))
    {
      return false;
    }
    OutStart = Storage.LanePoints[Lane.PointsBegin];
    FVector RawDir = Storage.LanePoints[Lane.PointsEnd - 1] - OutStart;
    if (Lane.PointsEnd - Lane.PointsBegin >= 2)
    {
      const FVector SegDir = Storage.LanePoints[Lane.PointsBegin + 1] - OutStart;
      if (SegDir.SizeSquared2D() > 1.0)
      {
        RawDir = SegDir;
      }
    }
    RawDir.Z = 0.0;
    if (!RawDir.Normalize())
    {
      return false;
    }
    OutDir2D = RawDir;
    return true;
  }

  ETrafficLightState MassFlagsToCarlaState(EMassTrafficLightStateFlags Flags,
                                           const TArray<FZoneGraphTrafficLaneData *> &ControlledLanes)
  {
    if ((Flags & EMassTrafficLightStateFlags::VehiclePrepareToStop) !=
        EMassTrafficLightStateFlags::None)
    {
      return ETrafficLightState::Yellow;
    }
    if ((Flags & EMassTrafficLightStateFlags::VehicleGo) !=
        EMassTrafficLightStateFlags::None)
    {
      // Guard against the 1-frame AdvancePeriod transition on un-actuated distant
      // intersections where UpdateTrafficLightsForCurrentPeriod sets VehicleGo
      // while SoftClose kept every controlled vehicle lane closed.
      if (ControlledLanes.Num() > 0)
      {
        bool bAnyOpen = false;
        bool bAnyAboutToClose = false;
        for (const FZoneGraphTrafficLaneData *Lane : ControlledLanes)
        {
          if (Lane != nullptr && Lane->bIsOpen)
          {
            bAnyOpen = true;
            bAnyAboutToClose |= Lane->bIsAboutToClose;
          }
        }
        if (!bAnyOpen)
        {
          return ETrafficLightState::Red;
        }
        if (bAnyAboutToClose)
        {
          return ETrafficLightState::Yellow;
        }
      }
      return ETrafficLightState::Green;
    }
    return ETrafficLightState::Red;
  }

  ETrafficLightState MassLanesToCarlaState(
      const TArray<FZoneGraphTrafficLaneData *> &ControlledLanes)
  {
    bool bAnyOpen = false;
    bool bAllOpenAboutToClose = true;
    for (const FZoneGraphTrafficLaneData *Lane : ControlledLanes)
    {
      if (Lane != nullptr && Lane->bIsOpen)
      {
        bAnyOpen = true;
        if (!Lane->bIsAboutToClose)
        {
          bAllOpenAboutToClose = false;
        }
      }
    }
    if (!bAnyOpen)
    {
      return ETrafficLightState::Red;
    }
    return bAllOpenAboutToClose ? ETrafficLightState::Yellow : ETrafficLightState::Green;
  }

  EMassTrafficLightStateFlags CarlaStateToMassFlags(
      ETrafficLightState CarlaState,
      EMassTrafficLightStateFlags ExistingPedestrianBits)
  {
    const EMassTrafficLightStateFlags PedMask =
        ExistingPedestrianBits & EMassTrafficLightStateFlags::PedestrianGo;
    switch (CarlaState)
    {
      case ETrafficLightState::Green:
        return EMassTrafficLightStateFlags::VehicleGo;
      case ETrafficLightState::Yellow:
        return EMassTrafficLightStateFlags::VehiclePrepareToStop;
      case ETrafficLightState::Red:
      default:
        return PedMask;
    }
  }
  void RepairRetimedIntersectionPeriods(FMassTrafficIntersectionFragment &Frag)
  {
    const int32 NumPeriods = Frag.Periods.Num();
    const int32 NumLights = Frag.TrafficLights.Num();
    if (!Frag.bHasTrafficLights || NumPeriods <= 0)
    {
      return;
    }

    // UMassTrafficRetimeIntersectionPeriodsFieldOperation removes 0-duration periods
    // AFTER FMassTrafficIntersectionFragment::Finalize() already baked
    // VehicleLaneIndices_ClosedInNextPeriod and bWillAllVehicleLanesCloseInNextPeriodForThisTrafficLight,
    // leaving 4-way intersections with false flags that jump Green -> Red with 0 s of Yellow.
    // Re-finalize every surviving period against its actual successor period.
    for (int32 P = 0; P < NumPeriods; ++P)
    {
      FMassTrafficPeriod &ThisPeriod = Frag.Periods[P];
      FMassTrafficPeriod &NextPeriod = Frag.Periods[(P + 1) % NumPeriods];

      if (ThisPeriod.VehicleLanes.IsEmpty())
      {
        for (int32 L = 0; L < NumLights; ++L)
        {
          if (FMassTrafficLightControl *Ctrl = ThisPeriod.GetTrafficLightControl(static_cast<int8>(L)))
          {
            Ctrl->TrafficLightStateFlags =
                Ctrl->TrafficLightStateFlags & EMassTrafficLightStateFlags::PedestrianGo;
          }
        }
        continue;
      }

      if (float(ThisPeriod.Duration) < 6.0f)
      {
        ThisPeriod.Duration = 6.0f;
      }

      ThisPeriod.VehicleLaneIndices_ClosedInNextPeriod.Reset();
      for (int32 I = 0; I < ThisPeriod.VehicleLanes.Num() && I <= 255; ++I)
      {
        FZoneGraphTrafficLaneData *Lane = ThisPeriod.VehicleLanes[I];
        if (Lane != nullptr && !NextPeriod.VehicleLanes.Contains(Lane))
        {
          ThisPeriod.VehicleLaneIndices_ClosedInNextPeriod.Add(static_cast<uint8>(I));
        }
      }

      for (int32 L = 0; L < NumLights; ++L)
      {
        const int8 LightIdx = static_cast<int8>(L);
        FMassTrafficLightControl *ThisCtrl = ThisPeriod.GetTrafficLightControl(LightIdx);
        if (ThisCtrl == nullptr ||
            (ThisCtrl->TrafficLightStateFlags & EMassTrafficLightStateFlags::VehicleGo) ==
                EMassTrafficLightStateFlags::None)
        {
          continue;
        }

        FMassTrafficLightControl *NextCtrl = NextPeriod.GetTrafficLightControl(LightIdx);
        if (NextCtrl == nullptr)
        {
          NextPeriod.AddTrafficLightControl(LightIdx, EMassTrafficLightStateFlags::None);
          NextCtrl = NextPeriod.GetTrafficLightControl(LightIdx);
        }

        const bool bStaysGreenInNext =
            !NextPeriod.VehicleLanes.IsEmpty() &&
            (NextCtrl != nullptr) &&
            ((NextCtrl->TrafficLightStateFlags & EMassTrafficLightStateFlags::VehicleGo) !=
             EMassTrafficLightStateFlags::None);
        ThisCtrl->bWillAllVehicleLanesCloseInNextPeriodForThisTrafficLight = !bStaysGreenInNext;
      }
    }

    // Ensure the initially active vehicle period starts open so Frame 0 never
    // zoom-skips an un-actuated startup period without a Yellow interval.
    FMassTrafficPeriod &Active = Frag.GetCurrentPeriod();
    if (!Active.VehicleLanes.IsEmpty())
    {
      if (float(Frag.PeriodTimeRemaining) < 3.5f)
      {
        Frag.PeriodTimeRemaining = Active.Duration;
      }
      for (FZoneGraphTrafficLaneData *Lane : Active.VehicleLanes)
      {
        if (Lane != nullptr && Lane->ConstData.bIsIntersectionLane)
        {
          Lane->bIsOpen = true;
          Lane->bIsAboutToClose = false;
        }
      }
      Frag.LastVehicleLanesActionAppliedToCurrentPeriod = EMassTrafficPeriodLanesAction::Open;
      Frag.UpdateTrafficLightsForCurrentPeriod();
    }
  }
}

UCarlaTrafficLightActuationProcessor::UCarlaTrafficLightActuationProcessor()
    : IntersectionQuery(*this)
{
    ExecutionFlags = static_cast<int32>(EProcessorExecutionFlags::All);
    ProcessingPhase = EMassProcessingPhase::EndPhysics;
    bRequiresGameThreadExecution = true;
    ExecutionOrder.ExecuteInGroup = UE::MassTraffic::ProcessorGroupNames::EndPhysicsIntersectionBehavior;
    ExecutionOrder.ExecuteBefore.Add(UMassTrafficUpdateIntersectionsProcessor::StaticClass()->GetFName());
}

void UCarlaTrafficLightActuationProcessor::ConfigureQueries(
    const TSharedRef<FMassEntityManager> &EntityManager)
{
    IntersectionQuery.AddRequirement<FMassTrafficIntersectionFragment>(EMassFragmentAccess::ReadWrite);
}

void UCarlaTrafficLightActuationProcessor::Execute(FMassEntityManager &EntityManager,
                                                   FMassExecutionContext &Context)
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    if (CVarCarlaSyncTrafficLights.GetValueOnGameThread() != 1 ||
        CVarCarlaActuateSignalisedIntersections.GetValueOnGameThread() == 0)
    {
        return;
    }
    UMassTrafficSubsystem *MassTrafficSub = World->GetSubsystem<UMassTrafficSubsystem>();
    if (MassTrafficSub == nullptr || !MassTrafficSub->HasTrafficVehicleAgents())
    {
        return;
    }

    if (ActuatedWorld.Get() != World)
    {
        RepairedIntersections.Reset();
        ActuatedWorld = World;
    }

    if (UMassTrafficSettings *Settings = GetMutableDefault<UMassTrafficSettings>())
    {
        if (Settings->StandardTrafficPrepareToStopSeconds < 3.0f)
        {
            Settings->StandardTrafficPrepareToStopSeconds = 3.0f;
        }
    }

    // MassTrafficMovement clears NewCurrentLane.bIsVehicleReadyToUseLane during
    // PrePhysics/DuringPhysics whenever a Mass vehicle enters a lane. Setting it
    // here in EndPhysics immediately before UMassTrafficUpdateIntersectionsProcessor
    // ensures signalised intersections never skip their 3 s Yellow clearance period
    // or strobe through empty phases in 1 frame under light traffic.
    IntersectionQuery.ForEachEntityChunk(Context,
        [this](FMassExecutionContext &ChunkCtx)
        {
            const TArrayView<FMassTrafficIntersectionFragment> Frags =
                ChunkCtx.GetMutableFragmentView<FMassTrafficIntersectionFragment>();
            for (int32 i = 0; i < ChunkCtx.GetNumEntities(); ++i)
            {
                FMassTrafficIntersectionFragment &Frag = Frags[i];
                if (!Frag.bHasTrafficLights || Frag.Periods.IsEmpty())
                {
                    continue;
                }
                const FMassEntityHandle Entity = ChunkCtx.GetEntity(i);
                if (!RepairedIntersections.Contains(Entity))
                {
                    RepairRetimedIntersectionPeriods(Frag);
                    RepairedIntersections.Add(Entity);
                }
                for (FMassTrafficPeriod &Period : Frag.Periods)
                {
                    for (FZoneGraphTrafficLaneData *Lane : Period.VehicleLanes)
                    {
                        if (Lane != nullptr)
                        {
                            Lane->bIsVehicleReadyToUseLane = true;
                        }
                    }
                }
            }
        });
}

UCarlaTrafficLightSyncProcessor::UCarlaTrafficLightSyncProcessor()
    : IntersectionQuery(*this)
{
    ExecutionFlags = static_cast<int32>(EProcessorExecutionFlags::All);
    ProcessingPhase = EMassProcessingPhase::PostPhysics;
    bRequiresGameThreadExecution = true;
}

void UCarlaTrafficLightSyncProcessor::ConfigureQueries(
    const TSharedRef<FMassEntityManager> &EntityManager)
{
    IntersectionQuery.AddRequirement<FMassTrafficIntersectionFragment>(EMassFragmentAccess::ReadWrite);
    IntersectionQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
}

void UCarlaTrafficLightSyncProcessor::SetManagedGroupsFrozen(bool bFreeze)
{
    if (bGroupsFrozenBySync == bFreeze)
    {
        return;
    }
    for (const TWeakObjectPtr<ATrafficLightGroup> &WeakGroup : ManagedGroups)
    {
        if (ATrafficLightGroup *Group = WeakGroup.Get())
        {
            Group->SetFrozenGroup(bFreeze);
        }
    }
    bGroupsFrozenBySync = bFreeze;
}

void UCarlaTrafficLightSyncProcessor::ApplyDuplicatePoleVisibility(bool bHideDuplicates)
{
    if (bDuplicatesHidden == bHideDuplicates)
    {
        return;
    }
    for (const FSignalBinding &B : Bindings)
    {
        ATrafficLightBase *Light = B.CarlaLight.Get();
        if (!IsValid(Light) || B.TrafficLightIndex == INDEX_NONE)
        {
            continue;
        }
        Light->SetActorHiddenInGame(bHideDuplicates);
        TArray<UPrimitiveComponent *> Primitives;
        Light->GetComponents(Primitives);
        for (UPrimitiveComponent *Prim : Primitives)
        {
            if (IsValid(Prim) && !Prim->IsA<UBoxComponent>())
            {
                Prim->SetCollisionEnabled(
                    bHideDuplicates ? ECollisionEnabled::NoCollision
                                    : ECollisionEnabled::QueryOnly);
            }
        }
    }
    bDuplicatesHidden = bHideDuplicates;
}

void UCarlaTrafficLightSyncProcessor::RebuildBindingsIfNeeded(
    UWorld *World,
    FMassEntityManager &EntityManager,
    FMassExecutionContext &Context)
{
    const UZoneGraphSubsystem *ZoneGraphSub = World->GetSubsystem<UZoneGraphSubsystem>();
    if (ZoneGraphSub == nullptr)
    {
        return;
    }

    if (BoundWorld.Get() != World)
    {
        Bindings.Reset();
        ManagedGroups.Reset();
        ResizedTriggerBoxes.Reset();
        BoundWorld = World;
        BoundCarlaLightCount = 0;
        BoundMassIntersectionCount = 0;
        LastRebuildAttemptTime = -10.0;
        bGroupsFrozenBySync = false;
        bDuplicatesHidden = false;
    }

    int32 LiveCarlaLights = 0;
    for (TActorIterator<ATrafficLightBase> It(World); It; ++It)
    {
        if (IsValid(*It))
        {
            ++LiveCarlaLights;
        }
    }

    int32 LiveMassIntersections = 0;
    IntersectionQuery.ForEachEntityChunk(Context,
        [&LiveMassIntersections](FMassExecutionContext &ChunkCtx)
        {
            const TConstArrayView<FMassTrafficIntersectionFragment> Frags =
                ChunkCtx.GetFragmentView<FMassTrafficIntersectionFragment>();
            for (int32 i = 0; i < ChunkCtx.GetNumEntities(); ++i)
            {
                if (Frags[i].bHasTrafficLights && !Frags[i].Periods.IsEmpty())
                {
                    ++LiveMassIntersections;
                }
            }
        });

    if (LiveCarlaLights == 0 || LiveMassIntersections == 0)
    {
        return;
    }

    const bool bCountsMatch =
        (LiveCarlaLights == BoundCarlaLightCount &&
         LiveMassIntersections == BoundMassIntersectionCount &&
         Bindings.Num() > 0);
    if (bCountsMatch)
    {
        return;
    }

    const double Now = World->GetTimeSeconds();
    if (Now - LastRebuildAttemptTime < 1.0)
    {
        return;
    }
    LastRebuildAttemptTime = Now;

    if (bDuplicatesHidden)
    {
        ApplyDuplicatePoleVisibility(false);
    }
    if (bGroupsFrozenBySync)
    {
        SetManagedGroupsFrozen(false);
    }

    struct FMassLaneSample
    {
        FZoneGraphTrafficLaneData *LanePtr = nullptr;
        FVector StartPos = FVector::ZeroVector;
        FVector Dir2D = FVector::ForwardVector;
    };

    struct FMassIntersectionCandidate
    {
        FMassEntityHandle Entity;
        FVector Center = FVector::ZeroVector;
        TArray<FMassLaneSample> Lanes;
    };

    TArray<FMassIntersectionCandidate> Intersections;
    Intersections.Reserve(LiveMassIntersections);

    IntersectionQuery.ForEachEntityChunk(Context,
        [&Intersections, ZoneGraphSub](FMassExecutionContext &ChunkCtx)
        {
            const TConstArrayView<FMassTrafficIntersectionFragment> Frags =
                ChunkCtx.GetFragmentView<FMassTrafficIntersectionFragment>();
            const TConstArrayView<FTransformFragment> Transforms =
                ChunkCtx.GetFragmentView<FTransformFragment>();

            for (int32 i = 0; i < ChunkCtx.GetNumEntities(); ++i)
            {
                const FMassTrafficIntersectionFragment &Frag = Frags[i];
                if (!Frag.bHasTrafficLights || Frag.Periods.IsEmpty())
                {
                    continue;
                }
                const FZoneGraphStorage *Storage =
                    ZoneGraphSub->GetZoneGraphStorage(Frag.ZoneGraphDataHandle);
                if (Storage == nullptr)
                {
                    continue;
                }

                FMassIntersectionCandidate Cand;
                Cand.Entity = ChunkCtx.GetEntity(i);
                Cand.Center = Transforms[i].GetTransform().GetLocation();

                TSet<FZoneGraphTrafficLaneData *> SeenLanes;
                for (const FMassTrafficPeriod &Period : Frag.Periods)
                {
                    for (FZoneGraphTrafficLaneData *LaneData : Period.VehicleLanes)
                    {
                        if (LaneData == nullptr || SeenLanes.Contains(LaneData))
                        {
                            continue;
                        }
                        FVector StartPos, Dir2D;
                        if (GetZoneLaneStartAndDir(*Storage, LaneData->LaneHandle.Index, StartPos, Dir2D))
                        {
                            SeenLanes.Add(LaneData);
                            Cand.Lanes.Add(FMassLaneSample{LaneData, StartPos, Dir2D});
                        }
                    }
                }
                if (Cand.Lanes.Num() > 0)
                {
                    Intersections.Add(MoveTemp(Cand));
                }
            }
        });

    Bindings.Reset();
    ManagedGroups.Reset();

    int32 DirectLightMatches = 0;
    int32 LaneStateMatches = 0;
    int32 UnmatchedCarlaLights = 0;

    for (TActorIterator<ATrafficLightBase> It(World); It; ++It)
    {
        ATrafficLightBase *CarlaLight = *It;
        if (!IsValid(CarlaLight))
        {
            continue;
        }
        UTrafficLightComponent *Comp = CarlaLight->GetTrafficLightComponent();
        if (Comp == nullptr)
        {
            continue;
        }

        TArray<FApproachBoxSample> BoxSamples;
        FVector MeanBoxPos, MeanBoxDir2D;
        if (!ExtractApproachSamples(Comp, CarlaLight, BoxSamples, MeanBoxPos, MeanBoxDir2D))
        {
            ++UnmatchedCarlaLights;
            continue;
        }

        int32 BestIntersectionIdx = INDEX_NONE;
        int32 DirectPoleFallbackLightIdx = INDEX_NONE;
        double BestScore = TNumericLimits<double>::Max();

        for (int32 Idx = 0; Idx < Intersections.Num(); ++Idx)
        {
            const FMassIntersectionCandidate &Cand = Intersections[Idx];
            if (FVector::DistSquared2D(MeanBoxPos, Cand.Center) > FMath::Square(12000.0))
            {
                continue;
            }

            for (const FMassLaneSample &Lane : Cand.Lanes)
            {
                for (const FApproachBoxSample &Box : BoxSamples)
                {
                    if (FMath::Abs(Lane.StartPos.Z - Box.Pos.Z) > 1500.0)
                    {
                        continue;
                    }
                    const double Dot = FVector::DotProduct(Box.Dir2D, Lane.Dir2D);
                    if (Dot < 0.65)
                    {
                        continue;
                    }
                    const FVector Delta = Lane.StartPos - Box.Pos;
                    const double ForwardCm = FVector::DotProduct(FVector(Delta.X, Delta.Y, 0.0), Box.Dir2D);
                    if (ForwardCm < -1000.0 || ForwardCm > 7000.0)
                    {
                        continue;
                    }
                    const double LateralCm =
                        FMath::Abs(Box.Dir2D.X * Delta.Y - Box.Dir2D.Y * Delta.X);
                    if (LateralCm > 3000.0)
                    {
                        continue;
                    }
                    const double Score =
                        LateralCm + 0.15 * FMath::Abs(ForwardCm - 500.0) + (1.0 - Dot) * 400.0;
                    if (Score < BestScore)
                    {
                        BestScore = Score;
                        BestIntersectionIdx = Idx;
                    }
                }
            }
        }

        // Fallback for wide avenues or approaches without trigger boxes: match CARLA's pole
        // directly against Epic's authored FMassTrafficLight positions (and intersection centers).
        if (BestIntersectionIdx == INDEX_NONE)
        {
            const FVector PoleLoc = CarlaLight->GetActorLocation();
            double BestPoleDistSq = FMath::Square(8000.0);
            for (int32 Idx = 0; Idx < Intersections.Num(); ++Idx)
            {
                const FMassIntersectionCandidate &Cand = Intersections[Idx];
                if (!EntityManager.IsEntityActive(Cand.Entity))
                {
                    continue;
                }
                const FMassTrafficIntersectionFragment &CandFrag =
                    EntityManager.GetFragmentDataChecked<FMassTrafficIntersectionFragment>(Cand.Entity);
                for (int32 L = 0; L < CandFrag.TrafficLights.Num(); ++L)
                {
                    if (FMath::Abs(PoleLoc.Z - CandFrag.TrafficLights[L].Position.Z) > 2000.0)
                    {
                        continue;
                    }
                    const double D2 = FVector::DistSquared2D(PoleLoc, CandFrag.TrafficLights[L].Position);
                    if (D2 < BestPoleDistSq)
                    {
                        BestPoleDistSq = D2;
                        BestIntersectionIdx = Idx;
                        DirectPoleFallbackLightIdx = L;
                    }
                }
                const double CenterD2 = FVector::DistSquared(PoleLoc, Cand.Center);
                if (CenterD2 < BestPoleDistSq)
                {
                    BestPoleDistSq = CenterD2;
                    BestIntersectionIdx = Idx;
                }
            }
        }

        if (BestIntersectionIdx == INDEX_NONE)
        {
            ++UnmatchedCarlaLights;
            continue;
        }

        const FMassIntersectionCandidate &Matched = Intersections[BestIntersectionIdx];
        if (!EntityManager.IsEntityActive(Matched.Entity))
        {
            ++UnmatchedCarlaLights;
            continue;
        }
        FMassTrafficIntersectionFragment &Frag =
            EntityManager.GetFragmentDataChecked<FMassTrafficIntersectionFragment>(Matched.Entity);

        FSignalBinding Binding;
        Binding.CarlaLight = CarlaLight;
        Binding.CarlaComponent = Comp;
        Binding.IntersectionEntity = Matched.Entity;

        // Collect all intersection lanes entering from this approach side.
        TSet<FZoneGraphTrafficLaneData *> ApproachLaneSet;
        for (const FMassLaneSample &Lane : Matched.Lanes)
        {
            if (FVector::DotProduct(MeanBoxDir2D, Lane.Dir2D) < 0.65)
            {
                continue;
            }
            bool bNearBox = false;
            for (const FApproachBoxSample &Box : BoxSamples)
            {
                const FVector Delta = Lane.StartPos - Box.Pos;
                const double ForwardCm = FVector::DotProduct(FVector(Delta.X, Delta.Y, 0.0), Box.Dir2D);
                const double LateralCm = FMath::Abs(Box.Dir2D.X * Delta.Y - Box.Dir2D.Y * Delta.X);
                if (ForwardCm >= -1000.0 && ForwardCm <= 7200.0 && LateralCm <= 3200.0)
                {
                    bNearBox = true;
                    break;
                }
            }
            if (bNearBox)
            {
                ApproachLaneSet.Add(Lane.LanePtr);
                Binding.ControlledLanes.Add(Lane.LanePtr);
            }
        }

        // Identify the FMassTrafficLight index governing these lanes.
        const int32 NumMassLights = Frag.TrafficLights.Num();
        if (NumMassLights > 0 && ApproachLaneSet.Num() > 0)
        {
            TArray<int32> ExclusiveHits;
            TArray<int32> SharedHits;
            ExclusiveHits.Init(0, NumMassLights);
            SharedHits.Init(0, NumMassLights);

            for (FMassTrafficPeriod &Period : Frag.Periods)
            {
                bool bPeriodUsesApproach = false;
                for (FZoneGraphTrafficLaneData *PLane : Period.VehicleLanes)
                {
                    if (ApproachLaneSet.Contains(PLane))
                    {
                        bPeriodUsesApproach = true;
                        break;
                    }
                }
                if (!bPeriodUsesApproach)
                {
                    continue;
                }

                TArray<int32> GoLightsInPeriod;
                for (int32 L = 0; L < NumMassLights; ++L)
                {
                    if (const FMassTrafficLightControl *Ctrl =
                            Period.GetTrafficLightControl(static_cast<int8>(L)))
                    {
                        if ((Ctrl->TrafficLightStateFlags & EMassTrafficLightStateFlags::VehicleGo) !=
                            EMassTrafficLightStateFlags::None)
                        {
                            GoLightsInPeriod.Add(L);
                        }
                    }
                }
                if (GoLightsInPeriod.Num() == 1)
                {
                    ++ExclusiveHits[GoLightsInPeriod[0]];
                }
                for (int32 L : GoLightsInPeriod)
                {
                    ++SharedHits[L];
                }
            }

            int32 ChosenLight = INDEX_NONE;
            int32 BestExcl = 0;
            for (int32 L = 0; L < NumMassLights; ++L)
            {
                if (ExclusiveHits[L] > BestExcl)
                {
                    BestExcl = ExclusiveHits[L];
                    ChosenLight = L;
                }
            }
            if (ChosenLight == INDEX_NONE)
            {
                int32 BestShared = 0;
                double BestDistSq = TNumericLimits<double>::Max();
                for (int32 L = 0; L < NumMassLights; ++L)
                {
                    if (SharedHits[L] <= 0)
                    {
                        continue;
                    }
                    const double D2 = FVector::DistSquared2D(
                        Frag.TrafficLights[L].Position, MeanBoxPos);
                    if (SharedHits[L] > BestShared ||
                        (SharedHits[L] == BestShared && D2 < BestDistSq))
                    {
                        BestShared = SharedHits[L];
                        BestDistSq = D2;
                        ChosenLight = L;
                    }
                }
            }
            Binding.TrafficLightIndex = ChosenLight;
        }
        else if (DirectPoleFallbackLightIdx != INDEX_NONE &&
                 Frag.TrafficLights.IsValidIndex(DirectPoleFallbackLightIdx))
        {
            Binding.TrafficLightIndex = DirectPoleFallbackLightIdx;
        }
        else if (NumMassLights > 0)
        {
            const FVector PoleLoc = CarlaLight->GetActorLocation();
            double BestD2 = FMath::Square(6000.0);
            for (int32 L = 0; L < NumMassLights; ++L)
            {
                const double D2 = FVector::DistSquared2D(PoleLoc, Frag.TrafficLights[L].Position);
                if (D2 < BestD2)
                {
                    BestD2 = D2;
                    Binding.TrafficLightIndex = L;
                }
            }
        }

        if (Binding.TrafficLightIndex != INDEX_NONE)
        {
            ++DirectLightMatches;
        }
        else if (Binding.ControlledLanes.Num() > 0)
        {
            ++LaneStateMatches;
        }
        else
        {
            ++UnmatchedCarlaLights;
            continue;
        }

        // CARLA's UTrafficLightComponent::InitializeSign hardcodes a 3.0 m trigger box
        // (BoxExtent.X = 150 cm, ending 1.5 m upstream of s_signal) tuned for 30 km/h
        // stock towns. On 50-90 km/h City Sample arterials, stopping distance exceeds
        // 3.0 m, so a braking Traffic Manager vehicle slides past the front of the box,
        // clears OnEndOverlapTriggerBox, and releases its brakes into the red light.
        // Extend each trigger box once to [s_sig - 10.0 m, s_sig + 1.0 m] (550 cm half-X)
        // unless another intersection sits immediately upstream.
        for (UBoxComponent *Box : Comp->GetEffectTriggerVolume())
        {
            if (!IsValid(Box))
            {
                continue;
            }
            const FObjectKey BoxKey(Box);
            if (ResizedTriggerBoxes.Contains(BoxKey))
            {
                continue;
            }
            ResizedTriggerBoxes.Add(BoxKey);

            const FVector Extent = Box->GetUnscaledBoxExtent();
            if (Extent.X >= 450.0)
            {
                continue;
            }
            FVector Fwd = Box->GetForwardVector();
            Fwd.Z = 0.0;
            if (!Fwd.Normalize())
            {
                continue;
            }
            const FVector OrigLoc = Box->GetComponentLocation();
            double MaxUpstreamExtraCm = 550.0;
            for (int32 OtherIdx = 0; OtherIdx < Intersections.Num(); ++OtherIdx)
            {
                if (OtherIdx == BestIntersectionIdx)
                {
                    continue;
                }
                const FVector ToOther = Intersections[OtherIdx].Center - OrigLoc;
                if (FMath::Abs(ToOther.Z) > 1200.0)
                {
                    continue;
                }
                const double UpstreamCm = -FVector::DotProduct(FVector(ToOther.X, ToOther.Y, 0.0), Fwd);
                const double LatCm = FMath::Abs(Fwd.X * ToOther.Y - Fwd.Y * ToOther.X);
                if (UpstreamCm > 200.0 && UpstreamCm < 2200.0 && LatCm < 2000.0)
                {
                    MaxUpstreamExtraCm = FMath::Min(MaxUpstreamExtraCm, FMath::Max(100.0, UpstreamCm - 1200.0));
                }
            }
            const double DownstreamExtraCm = 250.0;
            const double NewHalfX = Extent.X + 0.5 * (MaxUpstreamExtraCm + DownstreamExtraCm);
            const double ShiftCm = 0.5 * (DownstreamExtraCm - MaxUpstreamExtraCm);
            Box->SetBoxExtent(FVector(NewHalfX, Extent.Y, Extent.Z), true);
            Box->SetWorldLocation(OrigLoc + Fwd * ShiftCm, false, nullptr, ETeleportType::TeleportPhysics);
        }

        if (ATrafficLightGroup *Group = Comp->GetGroup())
        {
            ManagedGroups.Add(Group);
        }
        Bindings.Add(MoveTemp(Binding));
    }

    BoundCarlaLightCount = LiveCarlaLights;
    BoundMassIntersectionCount = LiveMassIntersections;

    UE_LOG(LogCarlaCitySample, Log,
           TEXT("CarlaTrafficLightSync: bound %d/%d CARLA traffic lights across %d Mass intersections "
                "(%d direct FMassTrafficLight, %d lane-state, %d unmatched, %d groups)"),
           Bindings.Num(), LiveCarlaLights, Intersections.Num(),
           DirectLightMatches, LaneStateMatches, UnmatchedCarlaLights, ManagedGroups.Num());
}

void UCarlaTrafficLightSyncProcessor::Execute(FMassEntityManager &EntityManager,
                                              FMassExecutionContext &Context)
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }

    const int32 SyncMode = CVarCarlaSyncTrafficLights.GetValueOnGameThread();
    if (SyncMode <= 0)
    {
        if (bDuplicatesHidden)
        {
            ApplyDuplicatePoleVisibility(false);
        }
        if (bGroupsFrozenBySync)
        {
            SetManagedGroupsFrozen(false);
        }
        return;
    }

    RebuildBindingsIfNeeded(World, EntityManager, Context);
    if (Bindings.IsEmpty())
    {
        return;
    }

    ApplyDuplicatePoleVisibility(
        CVarCarlaHideDuplicateTrafficLights.GetValueOnGameThread() != 0);

    UMassTrafficSubsystem *MassTrafficSub = World->GetSubsystem<UMassTrafficSubsystem>();
    const bool bMassVehiclesActive =
        (MassTrafficSub != nullptr && MassTrafficSub->HasTrafficVehicleAgents());

    // Mode 1 drives CARLA from MassTraffic when MassTraffic vehicles are running; if the user
    // despawned MassTraffic vehicles (`DisableMassTraffic 1`), UMassTrafficUpdateIntersectionsProcessor
    // stops ticking intersections, so Mode 1 automatically falls back to driving Epic's visual
    // overhead lights from CARLA's controllers.
    const bool bMassDrivesCarla = (SyncMode == 1 && bMassVehiclesActive);

    if (bMassDrivesCarla)
    {
        SetManagedGroupsFrozen(true);
        const double Now = World->GetTimeSeconds();
        constexpr double kMinYellowSeconds = 2.5;

        // Copy Epic's live signal state onto every bound CARLA traffic light component,
        // guaranteeing a minimum Yellow clearance interval if a split-phase or preempted
        // Mass period ever transitions Green -> Red directly.
        for (FSignalBinding &B : Bindings)
        {
            UTrafficLightComponent *Comp = B.CarlaComponent.Get();
            if (!IsValid(Comp) || !EntityManager.IsEntityActive(B.IntersectionEntity))
            {
                continue;
            }
            const FMassTrafficIntersectionFragment &Frag =
                EntityManager.GetFragmentDataChecked<FMassTrafficIntersectionFragment>(B.IntersectionEntity);

            ETrafficLightState RawTarget = ETrafficLightState::Red;
            if (B.TrafficLightIndex != INDEX_NONE &&
                Frag.TrafficLights.IsValidIndex(B.TrafficLightIndex))
            {
                RawTarget = MassFlagsToCarlaState(
                    Frag.TrafficLights[B.TrafficLightIndex].TrafficLightStateFlags,
                    B.ControlledLanes);
            }
            else
            {
                RawTarget = MassLanesToCarlaState(B.ControlledLanes);
            }

            const ETrafficLightState CurrentState = Comp->GetLightState();
            ETrafficLightState TargetState = RawTarget;

            if (RawTarget == ETrafficLightState::Green)
            {
                B.HoldYellowUntilWorldTime = -1.0;
            }
            else if (RawTarget == ETrafficLightState::Yellow)
            {
                if (CurrentState != ETrafficLightState::Yellow)
                {
                    B.YellowStartedWorldTime = Now;
                }
            }
            else if (RawTarget == ETrafficLightState::Red)
            {
                if (CurrentState == ETrafficLightState::Green)
                {
                    // Never snap a live CARLA light directly from Green to Red with 0 s of Yellow.
                    B.YellowStartedWorldTime = Now;
                    B.HoldYellowUntilWorldTime = Now + kMinYellowSeconds;
                    TargetState = ETrafficLightState::Yellow;
                }
                else if (CurrentState == ETrafficLightState::Yellow)
                {
                    const double ElapsedYellow = Now - B.YellowStartedWorldTime;
                    if (B.HoldYellowUntilWorldTime > Now)
                    {
                        TargetState = ETrafficLightState::Yellow;
                    }
                    else if (ElapsedYellow >= 0.0 && ElapsedYellow < kMinYellowSeconds - 0.25)
                    {
                        B.HoldYellowUntilWorldTime = B.YellowStartedWorldTime + kMinYellowSeconds;
                        TargetState = ETrafficLightState::Yellow;
                    }
                }
            }

            if (CurrentState != TargetState)
            {
                Comp->SetLightState(TargetState);
            }
        }
    }
    else
    {
        // CARLA -> Mass mode (either explicit SyncTrafficLights == 2 or pure-CARLA fallback).
        SetManagedGroupsFrozen(false);

        for (const FSignalBinding &B : Bindings)
        {
            const UTrafficLightComponent *Comp = B.CarlaComponent.Get();
            if (!IsValid(Comp) || !EntityManager.IsEntityActive(B.IntersectionEntity))
            {
                continue;
            }
            FMassTrafficIntersectionFragment &Frag =
                EntityManager.GetFragmentDataChecked<FMassTrafficIntersectionFragment>(B.IntersectionEntity);

            const ETrafficLightState CarlaState = Comp->GetLightState();
            if (B.TrafficLightIndex != INDEX_NONE &&
                Frag.TrafficLights.IsValidIndex(B.TrafficLightIndex))
            {
                FMassTrafficLight &MassLight = Frag.TrafficLights[B.TrafficLightIndex];
                MassLight.TrafficLightStateFlags = CarlaStateToMassFlags(
                    CarlaState, MassLight.TrafficLightStateFlags);
            }

            if (bMassVehiclesActive)
            {
                const bool bOpen = (CarlaState == ETrafficLightState::Green ||
                                    CarlaState == ETrafficLightState::Yellow);
                const bool bClosing = (CarlaState == ETrafficLightState::Yellow);
                for (FZoneGraphTrafficLaneData *Lane : B.ControlledLanes)
                {
                    if (Lane != nullptr && Lane->ConstData.bIsIntersectionLane)
                    {
                        Lane->bIsOpen = bOpen;
                        Lane->bIsAboutToClose = bClosing;
                    }
                }
            }
        }
    }
}

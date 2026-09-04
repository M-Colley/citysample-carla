// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaMassBridgeProcessor.h"
#include "CarlaMassBridgeSubsystem.h"

#include "MassCommonFragments.h"
#include "MassExecutionContext.h"
#include "MassEntityManager.h"
#include "MassMovementFragments.h"
#include "MassTrafficFragments.h"
#include "MassCrowdFragments.h"

#include "Mass/EntityFragments.h"

#include "Engine/World.h"
#include "HAL/IConsoleManager.h"

namespace
{
  // Off by default. Read every frame so it can be flipped at runtime, and so
  // that flipping it OFF gets a chance to tear the proxies down rather than
  // freezing them in the registry.
  //
  // There is no interactive console on the server this project runs
  // (Run-CarlaCity.ps1 launches -game with stdout redirected), so set it from
  // Config/DefaultEngine.ini under [SystemSettings], or pass
  // -ExecCmds="carla.MassBridge.Enable 1" via Run-CarlaCity.ps1 -ExecCmds.
  static TAutoConsoleVariable<int32> CVarCarlaMassBridge(
      TEXT("carla.MassBridge.Enable"),
      0,
      TEXT("Mirror MassTraffic entities into CARLA's actor registry as dormant "
           "proxies so world.get_actors() sees them.\n"
           "  0: off (default)\n"
           "  1: on - DO NOT run CARLA's Traffic Manager at the same time; ALSM "
           "adopts every proxy as an unregistered vehicle."),
      ECVF_Default);

  // A hard ceiling. Every proxy costs a registry entry and a slot in every
  // world snapshot the server broadcasts, so an uncapped sweep on Big City
  // would be a self-inflicted denial of service.
  static TAutoConsoleVariable<int32> CVarCarlaMassBridgeMax(
      TEXT("carla.MassBridge.MaxProxies"),
      150,
      TEXT("Maximum number of Mass VEHICLES mirrored per frame."),
      ECVF_Default);

  // Pedestrians get their own budget rather than sharing one. City Sample runs
  // far more crowd entities than vehicles near the camera, so a shared cap
  // filled up with pedestrians and the traffic vanished from get_actors().
  static TAutoConsoleVariable<int32> CVarCarlaMassBridgeMaxWalkers(
      TEXT("carla.MassBridge.MaxWalkers"),
      100,
      TEXT("Maximum number of MassCrowd pedestrians mirrored per frame."),
      ECVF_Default);
}

UCarlaMassBridgeProcessor::UCarlaMassBridgeProcessor()
    : VehicleQuery(*this), CrowdQuery(*this)
{
    ExecutionFlags = static_cast<int32>(EProcessorExecutionFlags::All);
    ProcessingPhase = EMassProcessingPhase::PostPhysics;
    bRequiresGameThreadExecution = true;
}

void UCarlaMassBridgeProcessor::ConfigureQueries(
    const TSharedRef<FMassEntityManager> &EntityManager)
{
    // Same shape MassTrafficVehicleVisualizationProcessor uses.
    VehicleQuery.AddTagRequirement<FMassTrafficVehicleTag>(EMassFragmentPresence::All);
    VehicleQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
    VehicleQuery.AddRequirement<FMassVelocityFragment>(
        EMassFragmentAccess::ReadOnly, EMassFragmentPresence::Optional);

    // Epic's pedestrians. Same shape, different tag: MassCrowd marks its
    // entities with FMassCrowdTag, and they carry the same transform and
    // velocity fragments the vehicles do.
    CrowdQuery.AddTagRequirement<FMassCrowdTag>(EMassFragmentPresence::All);
    CrowdQuery.AddRequirement<FTransformFragment>(EMassFragmentAccess::ReadOnly);
    CrowdQuery.AddRequirement<FMassVelocityFragment>(
        EMassFragmentAccess::ReadOnly, EMassFragmentPresence::Optional);
}

void UCarlaMassBridgeProcessor::Execute(FMassEntityManager &EntityManager,
                                        FMassExecutionContext &Context)
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    UCarlaMassBridgeSubsystem *Bridge =
        World->GetSubsystem<UCarlaMassBridgeSubsystem>();
    if (Bridge == nullptr)
    {
        return;
    }

    // The off path must TEAR DOWN, not just skip. Returning early here would
    // leave every proxy registered at its last transform, so `Enable 0` would
    // appear to leak hundreds of frozen actors instead of restoring baseline.
    if (CVarCarlaMassBridge.GetValueOnGameThread() == 0)
    {
        if (Bridge->NumProxies() > 0)
        {
            Bridge->RemoveAllProxies();
        }
        return;
    }

    const int32 MaxProxies = FMath::Max(0, CVarCarlaMassBridgeMax.GetValueOnGameThread());
    if (MaxProxies == 0)
    {
        Bridge->RemoveAllProxies();
        return;
    }

    Bridge->BeginFrame();

    // Mirror the entities NEAREST the observer, not the first N the query
    // happens to yield.
    //
    // Chunk order is archetype order, which has nothing to do with geography,
    // so a first-N cap scattered its proxies across the whole city: a camera
    // could sit in dense traffic and see one bridged vehicle within 80 m while
    // the budget was spent on entities kilometres away. Collecting candidates
    // and taking the closest costs one pass and a partial sort over a few
    // thousand entries, which is nothing next to the registry work it feeds.
    const FVector Origin = Bridge->GetObserverLocation();

    struct FCandidate
    {
        FMassEntityHandle Entity;
        FTransform Transform;
        FVector Velocity;
        double DistSq;
    };

    const auto Sweep = [&Bridge, &Origin](FMassEntityQuery &Query,
                                          FMassExecutionContext &Ctx0,
                                          int32 Budget,
                                          UCarlaMassBridgeSubsystem::EProxyKind Kind)
    {
        if (Budget <= 0)
        {
            return 0;
        }
        TArray<FCandidate> Candidates;
        Query.ForEachEntityChunk(Ctx0,
            [&Candidates, &Origin](FMassExecutionContext &Ctx)
            {
                const int32 Num = Ctx.GetNumEntities();
                const TConstArrayView<FTransformFragment> Transforms =
                    Ctx.GetFragmentView<FTransformFragment>();
                const TConstArrayView<FMassVelocityFragment> Velocities =
                    Ctx.GetFragmentView<FMassVelocityFragment>();
                const bool bHasVelocity = Velocities.Num() == Num;

                Candidates.Reserve(Candidates.Num() + Num);
                for (int32 i = 0; i < Num; ++i)
                {
                    const FTransform &T = Transforms[i].GetTransform();
                    Candidates.Add(FCandidate{
                        Ctx.GetEntity(i),
                        T,
                        bHasVelocity ? Velocities[i].Value : FVector::ZeroVector,
                        FVector::DistSquared(T.GetLocation(), Origin)});
                }
            });

        if (Candidates.Num() > Budget)
        {
            Candidates.Sort([](const FCandidate &A, const FCandidate &B)
                {
                    return A.DistSq < B.DistSq;
                });
            Candidates.SetNum(Budget, EAllowShrinking::No);
        }
        for (const FCandidate &C : Candidates)
        {
            Bridge->SyncEntity(C.Entity, C.Transform, C.Velocity, Kind);
        }
        return Candidates.Num();
    };

    Sweep(VehicleQuery, Context, MaxProxies,
          UCarlaMassBridgeSubsystem::EProxyKind::Vehicle);
    Sweep(CrowdQuery, Context,
          FMath::Max(0, CVarCarlaMassBridgeMaxWalkers.GetValueOnGameThread()),
          UCarlaMassBridgeSubsystem::EProxyKind::Walker);

    Bridge->EndFrame();
}

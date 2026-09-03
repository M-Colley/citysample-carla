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
      TEXT("Maximum number of Mass entities mirrored per frame."),
      ECVF_Default);
}

UCarlaMassBridgeProcessor::UCarlaMassBridgeProcessor()
    : VehicleQuery(*this)
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

    int32 Emitted = 0;
    VehicleQuery.ForEachEntityChunk(Context,
        [&Bridge, &Emitted, MaxProxies](FMassExecutionContext &Ctx)
        {
            const int32 Num = Ctx.GetNumEntities();
            const TConstArrayView<FTransformFragment> Transforms =
                Ctx.GetFragmentView<FTransformFragment>();
            const TConstArrayView<FMassVelocityFragment> Velocities =
                Ctx.GetFragmentView<FMassVelocityFragment>();
            const bool bHasVelocity = Velocities.Num() == Num;

            for (int32 i = 0; i < Num; ++i)
            {
                if (Emitted >= MaxProxies)
                {
                    return;
                }
                Bridge->SyncEntity(
                    Ctx.GetEntity(i),
                    Transforms[i].GetTransform(),
                    bHasVelocity ? Velocities[i].Value : FVector::ZeroVector);
                ++Emitted;
            }
        });

    Bridge->EndFrame();
}

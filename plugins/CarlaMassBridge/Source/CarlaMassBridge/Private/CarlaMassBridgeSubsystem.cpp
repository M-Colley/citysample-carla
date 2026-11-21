// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaMassBridgeSubsystem.h"

#include "CarlaCitySampleLog.h"

#include "Carla/Actor/ActorData.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Actor/ActorInfo.h"
#include "Carla/Actor/ActorRegistry.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Util/BoundingBox.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "HAL/IConsoleManager.h"

#include "Mass/EntityFragments.h"
#include "MassCommonFragments.h"
#include "MassEntityManager.h"
#include "MassEntitySubsystem.h"
#include "MassMovementFragments.h"
#include "MassSpawner.h"
#include "MassTrafficFragments.h"
#include "MassTrafficSubsystem.h"

namespace
{
  namespace crp = carla::rpc;

  static TAutoConsoleVariable<int32> CVarCarlaMassBridgeFollowHero(
      TEXT("carla.MassBridge.FollowHero"),
      1,
      TEXT("Center the Mass traffic/crowd proxy window on an active role_name=hero "
           "vehicle when present, falling back to the spectator pawn.\n"
           "  0: always follow spectator\n"
           "  1: follow hero vehicle if spawned, else spectator (default)"),
      ECVF_Default);

  static TAutoConsoleVariable<int32> CVarCarlaExportObstaclesToMass(
      TEXT("carla.MassBridge.ExportCarlaObstacles"),
      1,
      TEXT("Publish live CARLA vehicles and walkers into Epic's Mass ECS as "
           "FMassTrafficObstacleTag entities so ambient MassTraffic cars brake "
           "and yield instead of driving through CARLA actors.\n"
           "  0: off\n"
           "  1: on (default)"),
      ECVF_Default);

  static TAutoConsoleVariable<int32> CVarCarlaDisableMassTraffic(
      TEXT("carla.MassBridge.DisableMassTraffic"),
      0,
      TEXT("Despawn City Sample's ambient MassTraffic vehicles (leaving visual "
           "intersection traffic lights intact) so pure CARLA Traffic Manager "
           "runs can be launched via -ExecCmds without editing level spawners.\n"
           "  0: keep MassTraffic enabled (default)\n"
           "  1: despawn MassTraffic & parked Mass vehicles"),
      ECVF_Default);

  static TAutoConsoleVariable<int32> CVarCarlaDisableMassCrowd(
      TEXT("carla.MassBridge.DisableMassCrowd"),
      0,
      TEXT("Despawn City Sample's ambient MassCrowd pedestrians for pure CARLA "
           "walker runs.\n"
           "  0: keep MassCrowd enabled (default)\n"
           "  1: despawn MassCrowd spawners"),
      ECVF_Default);

  bool IsHeroActor(const FCarlaActor *CarlaActor)
  {
    if (CarlaActor == nullptr || CarlaActor->IsDormant() ||
        CarlaActor->GetActorType() != FCarlaActor::ActorType::Vehicle ||
        CarlaActor->GetActor() == nullptr)
    {
      return false;
    }
    const FActorInfo *Info = CarlaActor->GetActorInfo();
    if (Info == nullptr)
    {
      return false;
    }
    if (const FActorAttribute *Role = Info->Description.Variations.Find(TEXT("role_name")))
    {
      return Role->Value.Contains(TEXT("hero")) || Role->Value.Contains(TEXT("ego_vehicle"));
    }
    return false;
  }

  // A generic saloon-ish box, in UNREAL CENTIMETRES. FBoundingBox is stored in
  // UE units and converted to metres for the client, so metres here would ship
  // a 2.4 cm car - which is exactly what the first run reported.
  const FVector kDefaultVehicleExtent(240.0f, 100.0f, 80.0f);

  // Half-extents of an adult, also in centimetres: 0.6 m across, 1.8 m tall.
  const FVector kDefaultWalkerExtent(30.0f, 30.0f, 90.0f);

  FActorDescription MakeMassProxyDescription(
      UCarlaMassBridgeSubsystem::EProxyKind Kind)
  {
    FActorDescription Description;
    // The prefix is load-bearing on the CLIENT: ActorFactory dispatches on the
    // type id, so "vehicle." builds a carla.Vehicle and "walker." a
    // carla.Walker. It is also exactly why the Traffic Manager adopts the
    // vehicles - see the warning in the header. Walkers are not adopted by the
    // TM, but they are still read-only here: Epic's crowd owns their motion.
    Description.Id = (Kind == UCarlaMassBridgeSubsystem::EProxyKind::Walker)
                         ? TEXT("walker.mass.citysample")
                         : TEXT("vehicle.mass.citysample");
    Description.UId = 0u;
    return Description;
  }
}

UCarlaEpisode *UCarlaMassBridgeSubsystem::GetEpisode() const
{
    return UCarlaStatics::GetCurrentEpisode(GetWorld());
}

FVector UCarlaMassBridgeSubsystem::GetObserverLocation() const
{
    if (const UCarlaEpisode *Episode = GetEpisode())
    {
        if (CVarCarlaMassBridgeFollowHero.GetValueOnGameThread() != 0)
        {
            if (CachedHeroId != 0)
            {
                const FCarlaActor *Hero = Episode->GetActorRegistry().FindCarlaActor(CachedHeroId);
                if (IsHeroActor(Hero))
                {
                    return Hero->GetActor()->GetActorLocation();
                }
                CachedHeroId = 0;
            }
            const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
            if (Now - LastHeroScanTime >= 0.5)
            {
                LastHeroScanTime = Now;
                for (const auto &Pair : Episode->GetActorRegistry())
                {
                    const FCarlaActor *Candidate = Pair.Value.Get();
                    if (IsHeroActor(Candidate))
                    {
                        CachedHeroId = Candidate->GetActorId();
                        return Candidate->GetActor()->GetActorLocation();
                    }
                }
            }
        }
        if (const APawn *Spectator = Episode->GetSpectatorPawn())
        {
            return Spectator->GetActorLocation();
        }
    }
    // No spectator yet (very early frames). The origin is a poor guess, but it
    // is stable, and the next frame will have one.
    return FVector::ZeroVector;
}

void UCarlaMassBridgeSubsystem::BeginFrame()
{
    SeenThisFrame.Reset();

    if (!bWarnedAboutTrafficManager)
    {
        bWarnedAboutTrafficManager = true;
        UE_LOG(LogCarlaCitySample, Log,
               TEXT("CarlaMassBridge: ENABLED. Mass entities appear in "
                    "world.get_actors() as dormant, READ-ONLY proxies centered "
                    "on the hero vehicle (or spectator)."));
    }
}

void UCarlaMassBridgeSubsystem::SyncEntity(FMassEntityHandle Entity,
                                           const FTransform &Transform,
                                           const FVector &Velocity,
                                           EProxyKind Kind)
{
    UCarlaEpisode *Episode = GetEpisode();
    if (Episode == nullptr)
    {
        return;
    }
    SeenThisFrame.Add(Entity);

    uint32 *Existing = EntityToActorId.Find(Entity);
    FCarlaActor *CarlaActor = nullptr;

    if (Existing != nullptr)
    {
        CarlaActor = Episode->FindCarlaActor(*Existing);
        if (CarlaActor == nullptr)
        {
            // Something else deregistered it; fall through and re-create.
            EntityToActorId.Remove(Entity);
            Existing = nullptr;
        }
    }

    if (Existing == nullptr)
    {
        const bool bWalker = (Kind == EProxyKind::Walker);

        TSet<crp::CityObjectLabel> Tags;
        Tags.Add(bWalker ? crp::CityObjectLabel::Pedestrians
                         : crp::CityObjectLabel::Car);

        const FBoundingBox Box{
            FVector::ZeroVector,
            bWalker ? kDefaultWalkerExtent : kDefaultVehicleExtent,
            FRotator::ZeroRotator};

        CarlaActor = Episode->GetActorRegistry().RegisterDormant(
            MakeMassProxyDescription(Kind),
            bWalker ? FCarlaActor::ActorType::Walker
                    : FCarlaActor::ActorType::Vehicle,
            Tags,
            Box,
            GetWorld());
        if (CarlaActor == nullptr)
        {
            return;
        }
        EntityToActorId.Add(Entity, CarlaActor->GetActorId());
    }

    // Refresh the dormant state. Every read path - GetActorLocalTransform,
    // GetActorVelocity, FWorldObserver_GetDormantActorState - serves from here
    // when the actor is dormant, so this is the whole update.
    if (FActorData *Data = CarlaActor->GetActorData())
    {
        const FVector NewLocation = Transform.GetLocation();

        // MassTraffic vehicles do not carry FMassVelocityFragment - that lives
        // on obstacles - so the query's optional read comes back empty and the
        // reported speed was a flat 0 km/h. Differentiate the transform
        // instead, which works whatever fragments an entity happens to have.
        FVector DerivedVelocity = Velocity;
        if (DerivedVelocity.IsNearlyZero())
        {
            const double Now = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0;
            if (const FMassProxyMotion *Prev = EntityMotion.Find(Entity))
            {
                const double Dt = Now - Prev->Time;
                // Reject nonsense rather than publishing it. Mass recycles
                // entity handles and LOD churn teleports an entity's transform,
                // so a naive delta produces spikes of hundreds of km/h. Only
                // trust a sane frame interval and a plausible displacement.
                constexpr double kMaxSpeedCmS = 4000.0;   // 144 km/h
                if (Dt > 1.0 / 240.0 && Dt < 0.5)
                {
                    const FVector Delta = NewLocation - Prev->Location;
                    if (Delta.Size() < kMaxSpeedCmS * Dt)
                    {
                        DerivedVelocity = Delta / Dt;
                    }
                }
            }
            EntityMotion.Add(Entity, FMassProxyMotion{NewLocation, Now});
        }

        Data->Location = FDVector(NewLocation);
        Data->Rotation = Transform.GetRotation();   // FActorData::Rotation is FQuat
        Data->Scale = Transform.GetScale3D();
        Data->Velocity = DerivedVelocity;           // UE cm/s, converted client-side
    }
}

void UCarlaMassBridgeSubsystem::EndFrame()
{
    UCarlaEpisode *Episode = GetEpisode();
    if (Episode == nullptr)
    {
        return;
    }

    // Entities that vanished this frame (LOD churn, despawn, or simply beyond
    // the proxy cap) must not linger as frozen actors.
    TArray<FMassEntityHandle> Stale;
    for (const TPair<FMassEntityHandle, uint32> &Pair : EntityToActorId)
    {
        if (!SeenThisFrame.Contains(Pair.Key))
        {
            Stale.Add(Pair.Key);
        }
    }
    for (const FMassEntityHandle &Entity : Stale)
    {
        if (const uint32 *Id = EntityToActorId.Find(Entity))
        {
            Episode->GetActorRegistry().Deregister(*Id);
        }
        EntityToActorId.Remove(Entity);
        EntityMotion.Remove(Entity);
    }
}

void UCarlaMassBridgeSubsystem::RemoveAllProxies()
{
    if (EntityToActorId.IsEmpty())
    {
        return;
    }
    if (UCarlaEpisode *Episode = GetEpisode())
    {
        for (const TPair<FMassEntityHandle, uint32> &Pair : EntityToActorId)
        {
            Episode->GetActorRegistry().Deregister(Pair.Value);
        }
    }
    const int32 Removed = EntityToActorId.Num();
    EntityToActorId.Reset();
    EntityMotion.Reset();
    SeenThisFrame.Reset();
    UE_LOG(LogCarlaCitySample, Log, TEXT("CarlaMassBridge: removed %d proxies"), Removed);
}

bool UCarlaMassBridgeSubsystem::IsTickable() const
{
    if (IsTemplate())
    {
        return false;
    }
    const UWorld *World = GetWorld();
    return World != nullptr &&
           (World->WorldType == EWorldType::Game || World->WorldType == EWorldType::PIE);
}

void UCarlaMassBridgeSubsystem::Tick(float DeltaTime)
{
    ApplyMassPopulationToggles();
    SyncCarlaObstaclesToMass();
}

void UCarlaMassBridgeSubsystem::ApplyMassPopulationToggles()
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    UMassEntitySubsystem *EntitySubsystem = World->GetSubsystem<UMassEntitySubsystem>();
    if (EntitySubsystem != nullptr && EntitySubsystem->GetEntityManager().IsProcessing())
    {
        return;
    }

    const bool bWantNoTraffic = (CVarCarlaDisableMassTraffic.GetValueOnGameThread() != 0);
    if (bWantNoTraffic)
    {
        if (!bMassTrafficDisabledApplied)
        {
            if (IConsoleVariable *VScale = IConsoleManager::Get().FindConsoleVariable(TEXT("MassTraffic.NumTrafficVehiclesScale")))
            {
                VScale->Set(0.0f, ECVF_SetByCode);
            }
            if (IConsoleVariable *PScale = IConsoleManager::Get().FindConsoleVariable(TEXT("MassTraffic.NumParkedVehiclesScale")))
            {
                PScale->Set(0.0f, ECVF_SetByCode);
            }
        }

        int32 DespawnedSpawners = 0;
        UMassTrafficSubsystem *TrafficSub = World->GetSubsystem<UMassTrafficSubsystem>();
        const bool bStillHasVehicles =
            !bMassTrafficDisabledApplied ||
            (TrafficSub != nullptr &&
             (TrafficSub->HasTrafficVehicleAgents() || TrafficSub->HasParkedVehicleAgents()));

        if (bStillHasVehicles)
        {
            for (TActorIterator<AMassSpawner> It(World); It; ++It)
            {
                AMassSpawner *Spawner = *It;
                if (!IsValid(Spawner))
                {
                    continue;
                }
                const FString Name = Spawner->GetName();
                if (Name.Contains(TEXT("Intersection")) || Name.Contains(TEXT("Crowd")))
                {
                    continue;
                }
                Spawner->ScaleSpawningCount(0.0f);
                Spawner->DoDespawning();
                ++DespawnedSpawners;
            }
            if (TrafficSub != nullptr)
            {
                TrafficSub->ClearAllTrafficLanes();
            }
            RemoveAllProxies();
            if (!bMassTrafficDisabledApplied && DespawnedSpawners > 0)
            {
                UE_LOG(LogCarlaCitySample, Log,
                       TEXT("CarlaMassBridge: disabled %d MassTraffic vehicle spawner(s)"),
                       DespawnedSpawners);
                bMassTrafficDisabledApplied = true;
            }
        }
    }
    else
    {
        bMassTrafficDisabledApplied = false;
    }

    const bool bWantNoCrowd = (CVarCarlaDisableMassCrowd.GetValueOnGameThread() != 0);
    if (bWantNoCrowd)
    {
        if (!bMassCrowdDisabledApplied)
        {
            int32 DespawnedCrowd = 0;
            for (TActorIterator<AMassSpawner> It(World); It; ++It)
            {
                AMassSpawner *Spawner = *It;
                if (!IsValid(Spawner) || !Spawner->GetName().Contains(TEXT("Crowd")))
                {
                    continue;
                }
                Spawner->ScaleSpawningCount(0.0f);
                Spawner->DoDespawning();
                ++DespawnedCrowd;
            }
            if (DespawnedCrowd > 0)
            {
                UE_LOG(LogCarlaCitySample, Log,
                       TEXT("CarlaMassBridge: disabled %d MassCrowd spawner(s)"),
                       DespawnedCrowd);
                bMassCrowdDisabledApplied = true;
            }
        }
    }
    else
    {
        bMassCrowdDisabledApplied = false;
    }
}

void UCarlaMassBridgeSubsystem::SyncCarlaObstaclesToMass()
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    UMassEntitySubsystem *EntitySubsystem = World->GetSubsystem<UMassEntitySubsystem>();
    if (EntitySubsystem == nullptr)
    {
        return;
    }
    FMassEntityManager &EntityManager = EntitySubsystem->GetMutableEntityManager();
    if (EntityManager.IsProcessing())
    {
        return;
    }

    if (CVarCarlaExportObstaclesToMass.GetValueOnGameThread() == 0)
    {
        RemoveAllObstacleEntities();
        return;
    }

    const UCarlaEpisode *Episode = GetEpisode();
    if (Episode == nullptr)
    {
        return;
    }

    if (!ObstacleArchetype.IsValid())
    {
        const UScriptStruct *Elements[] = {
            FMassTrafficObstacleTag::StaticStruct(),
            FTransformFragment::StaticStruct(),
            FMassVelocityFragment::StaticStruct(),
            FAgentRadiusFragment::StaticStruct(),
        };
        ObstacleArchetype = EntityManager.CreateArchetype(MakeConstArrayView(Elements));
        if (!ObstacleArchetype.IsValid())
        {
            return;
        }
    }

    TSet<uint32> SeenCarlaActors;
    for (const auto &Pair : Episode->GetActorRegistry())
    {
        const FCarlaActor *CarlaActor = Pair.Value.Get();
        if (CarlaActor == nullptr || CarlaActor->IsDormant())
        {
            continue;
        }
        const FCarlaActor::ActorType Type = CarlaActor->GetActorType();
        if (Type != FCarlaActor::ActorType::Vehicle &&
            Type != FCarlaActor::ActorType::Walker)
        {
            continue;
        }
        const AActor *Actor = CarlaActor->GetActor();
        if (!IsValid(Actor))
        {
            continue;
        }

        const uint32 CarlaId = CarlaActor->GetActorId();
        SeenCarlaActors.Add(CarlaId);

        FMassEntityHandle ObsEntity;
        if (const FMassEntityHandle *Existing = CarlaActorToObstacleEntity.Find(CarlaId))
        {
            ObsEntity = *Existing;
        }
        if (!ObsEntity.IsSet() || !EntityManager.IsEntityActive(ObsEntity))
        {
            ObsEntity = EntityManager.CreateEntity(ObstacleArchetype);
            if (!ObsEntity.IsSet())
            {
                continue;
            }
            CarlaActorToObstacleEntity.Add(CarlaId, ObsEntity);
        }

        float RadiusCm = (Type == FCarlaActor::ActorType::Walker) ? 50.0f : 240.0f;
        if (const FActorInfo *Info = CarlaActor->GetActorInfo())
        {
            const float BoxRadius = FMath::Max(Info->BoundingBox.Extent.X, Info->BoundingBox.Extent.Y);
            if (BoxRadius > 10.0f)
            {
                RadiusCm = FMath::Clamp(BoxRadius, 40.0f, 600.0f);
            }
        }

        EntityManager.GetFragmentDataChecked<FTransformFragment>(ObsEntity)
            .SetTransform(Actor->GetActorTransform());
        EntityManager.GetFragmentDataChecked<FMassVelocityFragment>(ObsEntity).Value =
            Actor->GetVelocity();
        EntityManager.GetFragmentDataChecked<FAgentRadiusFragment>(ObsEntity).Radius =
            RadiusCm;
    }

    TArray<uint32> StaleIds;
    for (const TPair<uint32, FMassEntityHandle> &Pair : CarlaActorToObstacleEntity)
    {
        if (!SeenCarlaActors.Contains(Pair.Key))
        {
            StaleIds.Add(Pair.Key);
        }
    }
    for (uint32 StaleId : StaleIds)
    {
        if (const FMassEntityHandle *Handle = CarlaActorToObstacleEntity.Find(StaleId))
        {
            if (Handle->IsSet() && EntityManager.IsEntityActive(*Handle))
            {
                EntityManager.DestroyEntity(*Handle);
            }
        }
        CarlaActorToObstacleEntity.Remove(StaleId);
    }
}

void UCarlaMassBridgeSubsystem::RemoveAllObstacleEntities()
{
    if (CarlaActorToObstacleEntity.IsEmpty())
    {
        return;
    }
    if (UWorld *World = GetWorld())
    {
        if (UMassEntitySubsystem *EntitySubsystem = World->GetSubsystem<UMassEntitySubsystem>())
        {
            FMassEntityManager &EntityManager = EntitySubsystem->GetMutableEntityManager();
            if (!EntityManager.IsProcessing())
            {
                for (const TPair<uint32, FMassEntityHandle> &Pair : CarlaActorToObstacleEntity)
                {
                    if (Pair.Value.IsSet() && EntityManager.IsEntityActive(Pair.Value))
                    {
                        EntityManager.DestroyEntity(Pair.Value);
                    }
                }
            }
        }
    }
    CarlaActorToObstacleEntity.Reset();
}

void UCarlaMassBridgeSubsystem::Deinitialize()
{
    RemoveAllObstacleEntities();
    ObstacleArchetype.Reset();
    RemoveAllProxies();
    Super::Deinitialize();
}

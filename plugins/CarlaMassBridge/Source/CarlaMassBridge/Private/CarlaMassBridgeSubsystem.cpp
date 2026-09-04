// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaCitySampleLog.h"
#include "CarlaMassBridgeSubsystem.h"

#include "Carla/Actor/ActorData.h"
#include "Carla/Actor/ActorDescription.h"
#include "Carla/Actor/ActorInfo.h"
#include "Carla/Actor/ActorRegistry.h"
#include "Carla/Actor/CarlaActor.h"
#include "Carla/Game/CarlaEpisode.h"
#include "Carla/Game/CarlaStatics.h"
#include "Carla/Util/BoundingBox.h"

#include "Engine/World.h"
#include "GameFramework/Pawn.h"

namespace
{
  namespace crp = carla::rpc;

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
        UE_LOG(LogCarlaCitySample, Warning,
               TEXT("CarlaMassBridge: ENABLED. Mass entities will appear in "
                    "world.get_actors() as dormant, READ-ONLY proxies. Do NOT run "
                    "CARLA's Traffic Manager while this is on: ALSM adopts every "
                    "actor whose type id starts with 'v' and has no dormant check."));
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

void UCarlaMassBridgeSubsystem::Deinitialize()
{
    RemoveAllProxies();
    Super::Deinitialize();
}

// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaFastGeoTagger.h"

#include "CarlaCitySampleLog.h"

#include "Carla/Game/Tagger.h"

#include "FastGeoContainer.h"
#include "FastGeoPrimitiveComponent.h"
#include "FastGeoStaticMeshComponent.h"
#include "FastGeoInstancedStaticMeshComponent.h"
#include "FastGeoSkinnedMeshComponent.h"
#include "FastGeoPhysicsBodyInstanceOwner.h"

#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "HAL/IConsoleManager.h"
#include "UObject/UObjectIterator.h"
#include "Engine/HitResult.h"
#include "WorldPartition/WorldPartitionSubsystem.h"

namespace crp = carla::rpc;

namespace
{
  static TAutoConsoleVariable<int32> CVarFastGeoTagger(
      TEXT("carla.FastGeoTagger.Enable"),
      1,
      TEXT("Apply CARLA semantic labels to City Sample's FastGeo geometry.\n"
           "FastGeo primitives are not UObjects, so CARLA's ATagger cannot reach\n"
           "them and ~98%% of every semantic frame comes back Unlabeled without\n"
           "this.\n"
           "  0: off\n"
           "  1: on (default)"),
      ECVF_Default);

  /// Shortest gap between sweeps. With CompletedContainers keyed on FObjectKey,
  /// steady-state sweeps skip already-verified containers in < 0.05 ms while
  /// tagging newly streamed cells within one 20 Hz frame.
  constexpr double kSweepIntervalSeconds = 0.05;

  /// Where CARLA keeps the label in custom primitive data.
  ///
  /// ATagger calls SetCustomPrimitiveDataVector4(4, {ActorID, Label, 0, 0}).
  /// UPrimitiveComponent's DataIndex is a FLOAT index, not a float4 index -
  /// SetPrimitiveData does `Memcpy(&Data[DataIndex], ...)` - so that writes
  /// floats 4,5,6,7. The segmentation shader agrees:
  ///
  ///   SegmentationSensor.usf:110
  ///     uint ActorID    = (uint)PrimitiveData.CustomPrimitiveData[1].x;
  ///     uint ActorLabel = (uint)PrimitiveData.CustomPrimitiveData[1].y;
  ///
  /// float4 #1 is floats 4..7, so .x is float 4 and .y is float 5. Writing to
  /// float 16 (i.e. reading "4" as a float4 index) puts the label in a slot
  /// nothing samples, and every pixel comes back 0 with no other symptom.
  constexpr int32 kCarlaLabelFloatIndex = 4;
  constexpr int32 kRequiredFloats = kCarlaLabelFloatIndex + 4;   // 8

  /// FFastGeoPrimitiveComponent::CustomPrimitiveData is protected and the class
  /// exposes no getter - only a whole-array setter. Blindly writing a fresh
  /// array would zero floats 0..15, which City Sample uses for per-primitive
  /// material variation.
  ///
  /// Forming a pointer-to-member through a derived class is the standard-
  /// sanctioned way to reach an inherited protected member: the access check
  /// happens in the derived class (which is allowed), while the resulting
  /// pointer has type `FCustomPrimitiveData FFastGeoPrimitiveComponent::*` and
  /// so applies to any base object. No casting, no UB.
  struct FFastGeoCustomDataAccess : public FFastGeoPrimitiveComponent
  {
    static const FCustomPrimitiveData &Read(const FFastGeoPrimitiveComponent &Component)
    {
      return Component.*(&FFastGeoCustomDataAccess::CustomPrimitiveData);
    }
  };

  /// Label for a ray-cast hit on FastGeo geometry.
  ///
  /// This is what makes SEMANTIC LIDAR work on this map. The camera reads its
  /// label on the GPU out of custom primitive data; a ray-casting sensor has
  /// only an FHitResult, and asks the hit's UPrimitiveComponent for a tag.
  /// FastGeo geometry has no such component - it owns its FBodyInstances and
  /// resolves them through IPhysicsBodyInstanceOwner - so every lidar point on
  /// the city came back Unlabeled while the camera was fully labelled.
  ///
  /// FromHitResult walks the hit's Chaos physics object back to the FastGeo
  /// body owner, which hands over the component. From there the label is the
  /// same float the tagger already wrote, so the two sensors cannot disagree.
  ///
  /// Called once per lidar point - a 64-beam sweep is ~230,000 of them per
  /// frame - so it does no string work and no allocation: a resolve, a pointer
  /// chase and one float compare.
  crp::CityObjectLabel ResolveFastGeoHitLabel(const FHitResult &Hit)
  {
    const FFastGeoPhysicsBodyInstanceOwner *Owner =
        FFastGeoPhysicsBodyInstanceOwner::FromHitResult(Hit);
    if (Owner == nullptr)
    {
      return crp::CityObjectLabel::None;
    }
    const FFastGeoPrimitiveComponent *Component = Owner->GetOwnerComponent();
    if (Component == nullptr)
    {
      return crp::CityObjectLabel::None;
    }
    const FCustomPrimitiveData &Data = FFastGeoCustomDataAccess::Read(*Component);
    if (Data.Data.Num() <= kCarlaLabelFloatIndex + 1)
    {
      // Not tagged yet - the sweep has not reached it, or the tagger is off.
      return crp::CityObjectLabel::None;
    }
    const float Raw = Data.Data[kCarlaLabelFloatIndex + 1];
    if (Raw <= 0.0f || Raw > 255.0f)
    {
      return crp::CityObjectLabel::None;
    }
    return static_cast<crp::CityObjectLabel>(static_cast<uint8>(Raw));
  }
}

bool UCarlaFastGeoTaggerSubsystem::DoesSupportWorldType(const EWorldType::Type WorldType) const
{
    return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
}

void UCarlaFastGeoTaggerSubsystem::Initialize(FSubsystemCollectionBase &Collection)
{
    Super::Initialize(Collection);

    // FastGeo containers arrive with World Partition cells, so the sweep has to
    // re-run as the world streams rather than once at BeginPlay.
    if (UWorld *World = GetWorld())
    {
        if (UWorldPartitionSubsystem *WP = World->GetSubsystem<UWorldPartitionSubsystem>())
        {
            StreamingStateUpdatedHandle = WP->OnStreamingStateUpdated().AddUObject(
                this, &UCarlaFastGeoTaggerSubsystem::OnStreamingStateUpdated);
        }

        // Streaming updates stop firing once the world settles, but containers
        // keep arriving for a while after that - and the very first cells load
        // before this subsystem exists. Run an immediate pass plus a 10 Hz timer.
        World->GetTimerManager().SetTimer(
            SweepTimerHandle, this,
            &UCarlaFastGeoTaggerSubsystem::OnStreamingStateUpdated,
            0.1f, true, 0.05f);

        if (CVarFastGeoTagger.GetValueOnGameThread() != 0)
        {
            TagFastGeoGeometry();
        }
    }

    // Teach CARLA's ray-casting sensors how to label FastGeo geometry. Without
    // this the camera is labelled and the semantic lidar is not, which is a
    // confusing pair of results to hand anyone.
    ATagger::SetHitLabelResolver(
        [](const FHitResult &Hit) { return ResolveFastGeoHitLabel(Hit); });
}

void UCarlaFastGeoTaggerSubsystem::Deinitialize()
{
    if (StreamingStateUpdatedHandle.IsValid())
    {
        if (UWorld *World = GetWorld())
        {
            if (UWorldPartitionSubsystem *WP = World->GetSubsystem<UWorldPartitionSubsystem>())
            {
                WP->OnStreamingStateUpdated().Remove(StreamingStateUpdatedHandle);
            }
        }
        StreamingStateUpdatedHandle.Reset();
    }
    if (UWorld *World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(SweepTimerHandle);
    }
    // The resolver captures nothing that outlives this module, but leaving a
    // dangling TFunction behind a static would still be a trap on hot reload.
    ATagger::SetHitLabelResolver(nullptr);
    MeshLabels.Reset();
    CompletedContainers.Reset();
    Super::Deinitialize();
}

void UCarlaFastGeoTaggerSubsystem::OnStreamingStateUpdated()
{
    if (CVarFastGeoTagger.GetValueOnGameThread() == 0)
    {
        CompletedContainers.Reset();
        return;
    }
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    // Rate-limited, but only mildly.
    //
    // Streaming state updates fire whenever a streaming source moves, which is
    // several times per frame - and each sweep walks every loaded component
    // (~45,000 here). Running on every update cost 76,606 sweeps in one
    // session and dropped the server from ~30 Hz to 6.7 Hz.
    //
    // An earlier version had no limit because a 1 Hz limit had been blamed for
    // wild run-to-run variance in the semantic output. That diagnosis was
    // wrong: the variance came from a pointer-keyed "already tagged" cache
    // going stale when World Partition recycled addresses (see the long note
    // in TagFastGeoGeometry). With that cache gone the sweep is stateless and
    // self-correcting, so a limit costs only the window between a container
    // loading and the next sweep - a quarter of a second, far too short to see
    // in a settled measurement.
    const double Now = World->GetTimeSeconds();
    if (Now - LastSweepTime < kSweepIntervalSeconds)
    {
        return;
    }
    LastSweepTime = Now;

    TagFastGeoGeometry();
}

int32 UCarlaFastGeoTaggerSubsystem::TagFastGeoGeometry()
{
    UWorld *World = GetWorld();
    if (World == nullptr)
    {
        return 0;
    }

    int32 NewlyTagged = 0;
    int32 Unmatched = 0;
    int32 Seen = 0;
    int32 Containers = 0;
    int32 SkippedContainers = 0;
    int32 Deferred = 0;
    int32 AlreadyTagged = 0;
    int32 SkippedSkinned = 0, SkippedOther = 0;

    auto ReportUnmatchedFastGeo = [](const FString &AssetPath)
    {
        static TSet<FString> Reported;
        bool bAlready = false;
        Reported.Add(AssetPath, &bAlready);
        if (!bAlready)
        {
            UE_LOG(LogCarlaCitySample, Log, TEXT("CarlaFastGeoTagger: UNMATCHED \"%s\""), *AssetPath);
        }
    };

    for (TObjectIterator<UFastGeoContainer> It; It; ++It)
    {
        UFastGeoContainer *Container = *It;
        if (Container == nullptr || Container->GetWorld() != World)
        {
            continue;
        }

        ++Containers;

        // NEVER key completion on a raw pointer (World Partition unloads and
        // reloads cells, and FMalloc reuses addresses). FObjectKey combines
        // GUObjectArray index + monotonically increasing SerialNumber, so a
        // newly streamed UFastGeoContainer can NEVER match an unloaded one.
        // Furthermore, a container is only recorded in CompletedContainers once
        // an entire verifying pass saw NewlyTagged == 0 and Deferred == 0 (all
        // render proxies created and carrying their target label).
        const FObjectKey ContainerKey(Container);
        const auto &Primitives = Container->GetPrimitiveComponents();
        const int32 TotalPrims = Primitives.Num();
        if (const FCompletedContainerInfo *Done = CompletedContainers.Find(ContainerKey))
        {
            if (Done->TotalPrimitives == TotalPrims)
            {
                Seen += Done->SeenStatic;
                AlreadyTagged += Done->Tagged;
                Unmatched += Done->Unmatched;
                ++SkippedContainers;
                continue;
            }
            CompletedContainers.Remove(ContainerKey);
        }

        int32 ContainerSeen = 0;
        int32 ContainerNewlyTagged = 0;
        int32 ContainerAlready = 0;
        int32 ContainerUnmatched = 0;
        int32 ContainerDeferred = 0;

        // Iterate PRIMITIVES, not GetStaticMeshComponents(). That array is
        // filled with `CastTo<FFastGeoStaticMeshComponent>()` - the concrete
        // non-instanced class - so it silently excludes
        // FFastGeoInstancedStaticMeshComponent, the sibling that a modular city
        // is almost entirely built from. Using it found 184 components across
        // 58 containers; the instanced ones are the rest.
        for (IPrimitiveComponent *PrimitiveComponent : Primitives)
        {
            if (PrimitiveComponent == nullptr)
            {
                continue;
            }
            FFastGeoPrimitiveComponent *Prim =
                static_cast<FFastGeoPrimitiveComponent *>(PrimitiveComponent);

            // CastTo<> tests FFastGeoElementType::IsA, so the Base type matches
            // both plain and instanced static mesh components. Non-static or
            // unmatched FastGeo primitives (including HLOD1 proxies) still MUST
            // have CustomPrimitiveData[4..5] cleared if City Sample populated
            // material-variation floats there, or SegmentationSensor.usf reads
            // float 5 as an invalid class ID (> 29).
            crp::CityObjectLabel Label = crp::CityObjectLabel::None;
            FFastGeoStaticMeshComponentBase *FastGeoComponent =
                Prim->CastTo<FFastGeoStaticMeshComponentBase>();
            if (FastGeoComponent == nullptr)
            {
                if (Prim->CastTo<FFastGeoSkinnedMeshComponentBase>() != nullptr)
                {
                    ++SkippedSkinned;
                }
                else
                {
                    ++SkippedOther;
                }
            }
            else
            {
                ++Seen;
                ++ContainerSeen;

                if (const UStaticMesh *Mesh = FastGeoComponent->GetStaticMesh())
                {
                    const FObjectKey MeshKey(Mesh);
                    if (const uint8 *Known = MeshLabels.Find(MeshKey))
                    {
                        Label = static_cast<crp::CityObjectLabel>(*Known);
                    }
                    else
                    {
                        const FString AssetPath = Mesh->GetPathName();
                        Label = ATagger::GetLabelByAssetPath(AssetPath);
                        MeshLabels.Add(MeshKey, static_cast<uint8>(Label));
                        if (Label == crp::CityObjectLabel::None)
                        {
                            ReportUnmatchedFastGeo(AssetPath);
                        }
                    }
                }
                if (Label == crp::CityObjectLabel::None)
                {
                    ++Unmatched;
                    ++ContainerUnmatched;
                }
            }

            const float TargetLabel = static_cast<float>(Label);
            const FCustomPrimitiveData &Existing =
                FFastGeoCustomDataAccess::Read(*Prim);

            // When Label == None and CustomPrimitiveData does not reach slot 4/5,
            // UE already supplies 0.0f to the shader without allocating or dirtying.
            if (Label == crp::CityObjectLabel::None &&
                Existing.Data.Num() <= kCarlaLabelFloatIndex)
            {
                continue;
            }

            if (Existing.Data.Num() >= kRequiredFloats &&
                Existing.Data[kCarlaLabelFloatIndex + 0] == 0.0f &&
                Existing.Data[kCarlaLabelFloatIndex + 1] == TargetLabel)
            {
                if (!Prim->IsRenderStateCreated())
                {
                    ++Deferred;
                    ++ContainerDeferred;
                }
                if (Label != crp::CityObjectLabel::None)
                {
                    ++AlreadyTagged;
                    ++ContainerAlready;
                }
                continue;
            }

            TArray<float> Data(Existing.Data);
            if (Data.Num() < kRequiredFloats)
            {
                Data.SetNumZeroed(kRequiredFloats);
            }
            Data[kCarlaLabelFloatIndex + 0] = 0.0f;         // actor id
            Data[kCarlaLabelFloatIndex + 1] = TargetLabel;  // label (0.0f when None)
            Prim->SetCustomPrimitiveData(Data);

            // DO NOT force MarkRenderStateDirty(true) here: see FastGeoPrimitiveComponent.cpp:331.
            if (!Prim->IsRenderStateCreated())
            {
                ++Deferred;
                ++ContainerDeferred;
            }
            if (Label != crp::CityObjectLabel::None)
            {
                ++NewlyTagged;
                ++ContainerNewlyTagged;
            }
        }

        if (ContainerNewlyTagged == 0 && ContainerDeferred == 0)
        {
            CompletedContainers.Add(
                ContainerKey,
                FCompletedContainerInfo{
                    TotalPrims, ContainerSeen, ContainerAlready, ContainerUnmatched});
        }
    }

    if (CompletedContainers.Num() > Containers * 2 + 64)
    {
        for (auto MapIt = CompletedContainers.CreateIterator(); MapIt; ++MapIt)
        {
            if (MapIt.Key().ResolveObjectPtr() == nullptr)
            {
                MapIt.RemoveCurrent();
            }
        }
    }

    if (Seen > 0 && (NewlyTagged > 0 || Deferred > 0 || Seen != LastSweepSeen ||
                     Containers != LastLoggedContainers))
    {
        UE_LOG(LogCarlaCitySample, Log,
               TEXT("CarlaFastGeoTagger sweep: %d containers (%d cached), %d seen, %d written, "
                    "%d already, %d unmatched, %d awaiting render state"),
               Containers, SkippedContainers, Seen, NewlyTagged, AlreadyTagged, Unmatched, Deferred);
        UE_LOG(LogCarlaCitySample, Verbose,
               TEXT("CarlaFastGeoTagger skipped: %d skinned, %d other"),
               SkippedSkinned, SkippedOther);
        LastLoggedContainers = Containers;
    }
    LastSweepSeen = Seen;
    LastSweepTaggedTotal = AlreadyTagged + NewlyTagged;
    return NewlyTagged;
}

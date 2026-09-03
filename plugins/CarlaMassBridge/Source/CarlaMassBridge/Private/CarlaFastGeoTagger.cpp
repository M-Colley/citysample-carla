// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaCitySampleLog.h"
#include "CarlaFastGeoTagger.h"

#include "Carla/Game/Tagger.h"

#include "FastGeoContainer.h"
#include "FastGeoPrimitiveComponent.h"
#include "FastGeoStaticMeshComponent.h"
#include "FastGeoInstancedStaticMeshComponent.h"
#include "FastGeoSkinnedMeshComponent.h"

#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "TimerManager.h"
#include "HAL/IConsoleManager.h"
#include "UObject/UObjectIterator.h"
#include "WorldPartition/WorldPartitionSubsystem.h"

namespace crp = carla::rpc;

namespace
{
  static TAutoConsoleVariable<int32> CVarFastGeoTagger(
      TEXT("carla.FastGeoTagger.Enable"),
      0,
      TEXT("Apply CARLA semantic labels to City Sample's FastGeo geometry.\n"
           "FastGeo primitives are not UObjects, so CARLA's ATagger cannot reach\n"
           "them and ~98%% of every semantic frame comes back Unlabeled without\n"
           "this.\n"
           "  0: off (default)\n"
           "  1: on"),
      ECVF_Default);

  /// Shortest gap between sweeps. Small enough that newly streamed geometry is
  /// correct well within one frame of arriving at 20 Hz, large enough that the
  /// sweep is not the thing setting the frame rate.
  constexpr double kSweepIntervalSeconds = 0.25;

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
        // before this subsystem exists. A slow repeating timer closes both gaps
        // without making the streaming hook any hotter.
        World->GetTimerManager().SetTimer(
            SweepTimerHandle, this,
            &UCarlaFastGeoTaggerSubsystem::OnStreamingStateUpdated,
            0.5f, true, 0.5f);
    }
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
    Super::Deinitialize();
}

void UCarlaFastGeoTaggerSubsystem::OnStreamingStateUpdated()
{
    if (CVarFastGeoTagger.GetValueOnGameThread() == 0)
    {
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
    int32 Deferred = 0;
    int32 AlreadyTagged = 0;
    int32 SkippedSkinned = 0, SkippedOther = 0;

    for (TObjectIterator<UFastGeoContainer> It; It; ++It)
    {
        UFastGeoContainer *Container = *It;
        if (Container == nullptr || Container->GetWorld() != World)
        {
            continue;
        }

        ++Containers;

        // NO SKIPPING BY POINTER, at either level.
        //
        // Two earlier versions of this sweep tried to avoid re-walking the
        // world: a TSet of component pointers already tagged, and a TMap of
        // containers whose primitive count had not changed. Both are wrong for
        // the same reason. World Partition unloads a container when the
        // streaming source moves away and loads it again when it comes back,
        // and the allocator hands out the SAME ADDRESSES for the replacements.
        // A pointer-keyed table then reports brand-new, untagged components as
        // already done, and the geometry renders with whatever happened to be
        // in its custom primitive data - which the segmentation shader reads as
        // a label. Measured at one fixed viewpoint, the same scene came back
        // anywhere between 0% and 92% Unlabeled depending on how the streaming
        // had cycled.
        //
        // So the sweep is stateless and verifies against the component's own
        // data instead. Reading one float and comparing it cannot go stale.
        // Walking ~50,000 components is not free, though, which is why
        // OnStreamingStateUpdated rate-limits to kSweepIntervalSeconds rather
        // than sweeping on every update.
        // Iterate PRIMITIVES, not GetStaticMeshComponents(). That array is
        // filled with `CastTo<FFastGeoStaticMeshComponent>()` - the concrete
        // non-instanced class - so it silently excludes
        // FFastGeoInstancedStaticMeshComponent, the sibling that a modular city
        // is almost entirely built from. Using it found 184 components across
        // 58 containers; the instanced ones are the rest.
        for (IPrimitiveComponent *PrimitiveComponent : Container->GetPrimitiveComponents())
        {
            if (PrimitiveComponent == nullptr)
            {
                continue;
            }
            // CastTo<> tests FFastGeoElementType::IsA, so the Base type matches
            // both the plain and the instanced concrete classes.
            FFastGeoStaticMeshComponentBase *FastGeoComponent =
                static_cast<FFastGeoPrimitiveComponent *>(PrimitiveComponent)
                    ->CastTo<FFastGeoStaticMeshComponentBase>();
            if (FastGeoComponent == nullptr)
            {
                // Census the misses so "still unlabeled" points at a type
                // rather than a mystery. Only two buckets are possible: skinned
                // meshes (Epic's crowd, which the Mass bridge covers
                // separately) and everything else. Procedural ISM does NOT
                // appear here - FFastGeoProceduralISMComponent derives from
                // FFastGeoStaticMeshComponentBase and its type token is
                // parented to it, so CastTo above already matched it and the
                // road surface is tagged like any other mesh.
                FFastGeoPrimitiveComponent *Prim =
                    static_cast<FFastGeoPrimitiveComponent *>(PrimitiveComponent);
                if (Prim->CastTo<FFastGeoSkinnedMeshComponentBase>() != nullptr)
                {
                    ++SkippedSkinned;
                }
                else
                {
                    ++SkippedOther;
                }
                continue;
            }
            ++Seen;

            const UStaticMesh *Mesh = FastGeoComponent->GetStaticMesh();
            if (Mesh == nullptr)
            {
                continue;
            }

            const crp::CityObjectLabel Label =
                ATagger::GetLabelByAssetPath(Mesh->GetPathName());
            if (Label == crp::CityObjectLabel::None)
            {
                // No rule matches this path, so leave the data untouched
                // rather than writing 0 over it.
                ++Unmatched;
                continue;
            }

            // Preserve whatever the material already relies on: copy the
            // existing floats, grow to reach CARLA's slot, then overwrite only
            // the two the segmentation shader reads.
            const FCustomPrimitiveData &Existing =
                FFastGeoCustomDataAccess::Read(*FastGeoComponent);

            // The component's own data is the authority on whether it is done.
            // This is the check that makes the sweep safe to repeat and immune
            // to a recycled address.
            if (Existing.Data.Num() >= kRequiredFloats &&
                Existing.Data[kCarlaLabelFloatIndex + 1] == static_cast<float>(Label))
            {
                ++AlreadyTagged;
                continue;
            }

            TArray<float> Data(Existing.Data);
            if (Data.Num() < kRequiredFloats)
            {
                Data.SetNumZeroed(kRequiredFloats);
            }
            Data[kCarlaLabelFloatIndex + 0] = 0.0f;                       // actor id
            Data[kCarlaLabelFloatIndex + 1] = static_cast<float>(Label);  // label
            FastGeoComponent->SetCustomPrimitiveData(Data);

            // DO NOT force MarkRenderStateDirty(true) here.
            //
            // SetCustomPrimitiveData already marks the state dirty with
            // bEvenIfNotCreated = false, which looks like a gap: a component
            // tagged before its proxy exists is never queued for recreation.
            // It is not a gap. CreateRenderState builds the proxy from the
            // component's CURRENT data, so a label written beforehand is
            // simply picked up when the proxy is eventually created - there is
            // nothing to invalidate.
            //
            // Forcing the flag does actively break things. It queues a
            // component whose ProxyState is `Creating` for recreation, and the
            // recreate path re-enters CreateRenderState, which asserts:
            //
            //   Assertion failed: ProxyState == EProxyCreationState::None ||
            //                     ProxyState == EProxyCreationState::Delayed
            //   FastGeoPrimitiveComponent.cpp:331
            //
            // That is a race against World Partition streaming, so it survives
            // whole sessions and then takes the server down mid-run.
            if (!FastGeoComponent->IsRenderStateCreated())
            {
                ++Deferred;
            }
            ++NewlyTagged;
        }
    }

    // At Log, not Verbose: "written N" is meaningless without the denominator
    // and the unmatched count. This is one line per sweep, not per component.
    // In the steady state `written` is 0 and `already` is everything, which is
    // the signal that the world is fully labelled - and it drops back as soon
    // as streaming brings new geometry in.
    // Only when something actually changed. One line per sweep produced 76,606
    // log lines in a single session, which is both noise and real I/O cost.
    if (Seen > 0 && (NewlyTagged > 0 || Unmatched > 0 || Seen != LastSweepSeen))
    {
        UE_LOG(LogCarlaCitySample, Log,
               TEXT("CarlaFastGeoTagger sweep: %d containers, %d seen, %d written, "
                    "%d already, %d unmatched, %d awaiting render state"),
               Containers, Seen, NewlyTagged, AlreadyTagged, Unmatched, Deferred);
        UE_LOG(LogCarlaCitySample, Verbose,
               TEXT("CarlaFastGeoTagger skipped: %d skinned, %d other"),
               SkippedSkinned, SkippedOther);
    }
    LastSweepSeen = Seen;
    LastSweepTaggedTotal = AlreadyTagged + NewlyTagged;
    return NewlyTagged;
}

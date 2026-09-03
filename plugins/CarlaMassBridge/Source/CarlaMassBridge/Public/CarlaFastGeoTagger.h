// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "CarlaFastGeoTagger.generated.h"

/// Applies CARLA semantic labels to City Sample's FastGeo geometry.
///
/// WHY CARLA'S OWN TAGGER CANNOT DO THIS
/// -------------------------------------
/// City Sample enables Epic's FastGeoStreaming plugin, which converts a
/// partitioned world's geometry into `FFastGeoPrimitiveComponent` - note the
/// `F`: these are plain C++ objects implementing IPrimitiveComponent, NOT
/// UPrimitiveComponent UObjects, and they are not owned by ordinary actors. So
/// ATagger cannot see them by construction:
///
///   * TActorIterator<AActor> never yields them,
///   * GetComponents<UStaticMeshComponent>() never returns them,
///   * ATagger::SetStencilValue(UPrimitiveComponent&) could not even take one.
///
/// A census inside ATagger measured the result: seven static mesh components in
/// the entire city, of which most belonged to CARLA's own spawned actors. That
/// is why ~98% of every semantic frame came back Unlabeled even after the label
/// rules and the segmentation render pass were both fixed.
///
/// This subsystem walks the FastGeo containers directly, classifies each mesh
/// with the same ATagger::GetLabelByAssetPath rule table used everywhere else,
/// and writes the label into custom primitive data in CARLA's slot.
///
/// Off by default: `carla.FastGeoTagger.Enable 1`.
UCLASS()
class CARLAMASSBRIDGE_API UCarlaFastGeoTaggerSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()

public:
    //~ Begin USubsystem
    virtual void Initialize(FSubsystemCollectionBase &Collection) override;
    virtual void Deinitialize() override;
    virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override;
    //~ End USubsystem

    /// Tag every FastGeo static mesh component in this world that has not been
    /// tagged yet. Returns how many were tagged by this call.
    int32 TagFastGeoGeometry();

    /// Components carrying a correct label as of the last sweep.
    int32 NumTagged() const { return LastSweepTaggedTotal; }

    /// Static-mesh FastGeo components the last sweep walked.
    int32 NumSeen() const { return LastSweepSeen; }

private:
    void OnStreamingStateUpdated();

    FDelegateHandle StreamingStateUpdatedHandle;

    /// Counters from the last sweep, for NumTagged()/NumSeen(). The sweep
    /// itself keeps NO per-component or per-container state: World Partition
    /// recycles addresses when a container unloads and reloads, so any table
    /// keyed on a pointer eventually reports new, untagged geometry as done.
    /// The component's own custom primitive data is the authority instead.
    int32 LastSweepSeen = 0;
    int32 LastSweepTaggedTotal = 0;

    /// World time of the last sweep, for the rate limit.
    double LastSweepTime = -1.0;

    FTimerHandle SweepTimerHandle;
};

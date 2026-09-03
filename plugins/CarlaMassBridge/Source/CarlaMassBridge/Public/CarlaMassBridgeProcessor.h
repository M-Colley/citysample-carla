// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "MassProcessor.h"

#include "CarlaMassBridgeProcessor.generated.h"

/// Sweeps MassTraffic vehicle entities once per frame and mirrors them into
/// CARLA's actor registry through UCarlaMassBridgeSubsystem.
///
/// Runs on the game thread in PostPhysics: the registry is not thread-safe and
/// transforms are final by then.
UCLASS()
class CARLAMASSBRIDGE_API UCarlaMassBridgeProcessor : public UMassProcessor
{
    GENERATED_BODY()

public:
    UCarlaMassBridgeProcessor();

protected:
    virtual void ConfigureQueries(const TSharedRef<FMassEntityManager> &EntityManager) override;
    virtual void Execute(FMassEntityManager &EntityManager,
                         FMassExecutionContext &Context) override;

private:
    /// Constructor-initialised, as every MassTraffic processor does
    /// (`: EntityQuery(*this)`). There is no FMassEntityQuery::Initialize().
    FMassEntityQuery VehicleQuery;
};

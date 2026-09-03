// Copyright (c) 2026. MIT licensed.
//
// Exports the ZoneGraph lane network of a level to JSON, for conversion to
// ASAM OpenDRIVE and import into CARLA.
//
// FZoneGraphStorage is a plain UPROPERTY() USTRUCT - not BlueprintType - so it
// is invisible to editor Python. This commandlet exists solely to reach it.
//
// Usage:
//   UnrealEditor-Cmd.exe <project>.uproject -run=ExportZoneGraph \
//       -Map=/Game/Map/Small_City_LVL -Out=C:/path/zonegraph.json \
//       -unattended -nosound -nullrhi
//
// The output is the RAW lane graph. Grouping lanes into roads and emitting
// OpenDRIVE happens in Python (tools/zonegraph_to_roadnetwork.py, then
// tools/roadnetwork_to_xodr.py), so that iterating on the road semantics does
// not require a C++ rebuild.

#pragma once

#include "Commandlets/Commandlet.h"
#include "ExportZoneGraphCommandlet.generated.h"

UCLASS()
class UExportZoneGraphCommandlet : public UCommandlet
{
	GENERATED_BODY()

public:
	UExportZoneGraphCommandlet();

	virtual int32 Main(const FString& Params) override;
};

// Copyright (c) 2026. MIT licensed.

#include "ExportZoneGraphCommandlet.h"

#include "EngineUtils.h"
#include "EditorWorldUtils.h"
#include "Editor.h"
#include "Engine/World.h"
#include "WorldPartition/WorldPartitionHelpers.h"
#include "HAL/FileManager.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/PackageName.h"
#include "Serialization/JsonWriter.h"
#include "Serialization/JsonSerializer.h"

#include "ZoneGraphData.h"
#include "ZoneGraphTypes.h"
#include "ZoneGraphSettings.h"

DEFINE_LOG_CATEGORY_STATIC(LogCityRoadExport, Log, All);

namespace
{
	/// Resolve a tag mask to the human names configured in
	/// [/Script/ZoneGraph.ZoneGraphSettings]. The City Sample defines 25 of
	/// them - Vehicle, Pedestrian, Intersection, Freeway, Crosswalk, Trunk
	/// Road, Freeway Onramp/Offramp and so on - and they carry the semantics
	/// OpenDRIVE needs to type each lane. Without them every lane is just
	/// "some polyline".
	TArray<FString> TagMaskToNames(const FZoneGraphTagMask Mask)
	{
		TArray<FString> Names;
		const UZoneGraphSettings* Settings = GetDefault<UZoneGraphSettings>();
		if (!Settings)
		{
			return Names;
		}

		TConstArrayView<FZoneGraphTagInfo> Infos = Settings->GetTagInfos();
		for (const FZoneGraphTagInfo& Info : Infos)
		{
			if (!Info.IsValid())
			{
				continue;
			}
			// FZoneGraphTagMask::Contains takes a tag, not another mask.
			if (Mask.Contains(Info.Tag))
			{
				Names.Add(Info.Name.ToString());
			}
		}
		return Names;
	}

	FString LinkTypeToString(const EZoneLaneLinkType Type)
	{
		switch (Type)
		{
		case EZoneLaneLinkType::Outgoing: return TEXT("Outgoing");
		case EZoneLaneLinkType::Incoming: return TEXT("Incoming");
		case EZoneLaneLinkType::Adjacent: return TEXT("Adjacent");
		case EZoneLaneLinkType::All:      return TEXT("All");
		default:                          return TEXT("None");
		}
	}

	/// Adjacency flags say whether a neighbouring lane is left/right and
	/// whether it runs the other way - which is exactly what decides if two
	/// lanes belong to the same OpenDRIVE road.
	TArray<FString> LinkFlagsToNames(const uint8 Flags)
	{
		TArray<FString> Out;
		auto Has = [Flags](EZoneLaneLinkFlags F) { return (Flags & (uint8)F) != 0; };
		if (Has(EZoneLaneLinkFlags::Left))              { Out.Add(TEXT("Left")); }
		if (Has(EZoneLaneLinkFlags::Right))             { Out.Add(TEXT("Right")); }
		if (Has(EZoneLaneLinkFlags::Splitting))         { Out.Add(TEXT("Splitting")); }
		if (Has(EZoneLaneLinkFlags::Merging))           { Out.Add(TEXT("Merging")); }
		if (Has(EZoneLaneLinkFlags::OppositeDirection)) { Out.Add(TEXT("OppositeDirection")); }
		return Out;
	}

	void WriteVector(const TSharedRef<TJsonWriter<>>& W, const FVector& V)
	{
		W->WriteArrayStart();
		W->WriteValue(V.X);
		W->WriteValue(V.Y);
		W->WriteValue(V.Z);
		W->WriteArrayEnd();
	}
}

UExportZoneGraphCommandlet::UExportZoneGraphCommandlet()
{
	IsClient = false;
	IsServer = false;
	IsEditor = true;
	LogToConsole = true;
}

int32 UExportZoneGraphCommandlet::Main(const FString& Params)
{
	TArray<FString> Tokens;
	TArray<FString> Switches;
	TMap<FString, FString> ParamsMap;
	ParseCommandLine(*Params, Tokens, Switches, ParamsMap);

	const FString MapName = ParamsMap.FindRef(TEXT("Map"));
	FString OutPath = ParamsMap.FindRef(TEXT("Out"));

	if (MapName.IsEmpty())
	{
		UE_LOG(LogCityRoadExport, Error,
			TEXT("-Map=/Game/Map/Small_City_LVL is required."));
		return 1;
	}
	if (OutPath.IsEmpty())
	{
		OutPath = FPaths::ProjectSavedDir() / TEXT("zonegraph.json");
	}

	// Resolve whatever was typed to a long package name. SearchForPackageOnDisk
	// accepts a bare level name too, which is friendlier than requiring the
	// full /Game/Map/... path.
	FString LongPackageName = MapName;
	if (!FPackageName::IsValidLongPackageName(LongPackageName))
	{
		FString Found;
		if (FPackageName::SearchForPackageOnDisk(MapName, &Found))
		{
			LongPackageName = Found;
		}
		else
		{
			UE_LOG(LogCityRoadExport, Error, TEXT("Could not find a package for %s"), *MapName);
			return 1;
		}
	}

	// Deliberately NOT UEditorLoadingAndSavingUtils::LoadMap - that drives the
	// interactive editor map-change path (progress dialogs, save prompts,
	// transaction reset), which is wrong under -unattended. FScopedEditorWorld
	// is the pattern UWorldPartitionBuilderCommandlet uses: it loads the
	// package, initialises the world as an editor world, roots it, and sets
	// GWorld + the editor world context for the scope's lifetime.
	UWorld::InitializationValues IVS;
	IVS.RequiresHitProxies(false);
	IVS.ShouldSimulatePhysics(false);
	IVS.EnableTraceCollision(false);
	IVS.CreateNavigation(false);
	IVS.CreateAISystem(false);
	IVS.AllowAudioPlayback(false);
	IVS.CreatePhysicsScene(true);

	UE_LOG(LogCityRoadExport, Display, TEXT("Loading world %s"), *LongPackageName);
	FScopedEditorWorld ScopedWorld(LongPackageName, IVS);

	UWorld* World = ScopedWorld.GetWorld();
	if (!World)
	{
		UE_LOG(LogCityRoadExport, Error, TEXT("Failed to load world %s"), *LongPackageName);
		return 1;
	}

	// World Partition initialises its always-loaded set during world init, but
	// actor registration and the GC purge are flushed on a tick. Without this
	// the iterator below can miss freshly-loaded actors.
	FWorldPartitionHelpers::FakeEngineTick(World);

	// AZoneGraphData is not spatially loaded, so it comes in with the
	// persistent level and a plain actor iterator finds it even though the
	// World Partition cells around it stay unloaded. (Confirmed empirically:
	// only 26 of 12,102 actors materialise headless, and both ZoneGraphData
	// actors are among them.)
	TArray<AZoneGraphData*> Datas;
	for (TActorIterator<AZoneGraphData> It(World); It; ++It)
	{
		if (AZoneGraphData* D = *It)
		{
			Datas.Add(D);
		}
	}

	UE_LOG(LogCityRoadExport, Display, TEXT("Found %d AZoneGraphData actor(s)"), Datas.Num());
	if (Datas.Num() == 0)
	{
		UE_LOG(LogCityRoadExport, Error,
			TEXT("No ZoneGraphData in %s. Is the ZoneGraph plugin enabled, and has the "
			     "lane graph been built for this level?"), *MapName);
		return 1;
	}

	TUniquePtr<FArchive> Ar(IFileManager::Get().CreateFileWriter(*OutPath));
	if (!Ar)
	{
		UE_LOG(LogCityRoadExport, Error, TEXT("Cannot write %s"), *OutPath);
		return 1;
	}

	// Stream the JSON: Big_City_LVL carries 128.8 MB of lane data, so the
	// document must never be assembled in memory.
	TSharedRef<TJsonWriter<>> W = TJsonWriterFactory<>::Create(Ar.Get());

	int64 TotalLanes = 0;
	int64 TotalPoints = 0;

	W->WriteObjectStart();
	W->WriteValue(TEXT("source"), TEXT("CitySample/ZoneGraph"));
	W->WriteValue(TEXT("map"), MapName);
	W->WriteValue(TEXT("units"), TEXT("unreal_cm_left_handed"));

	W->WriteArrayStart(TEXT("zone_graphs"));
	for (const AZoneGraphData* Data : Datas)
	{
		const FZoneGraphStorage& S = Data->GetStorage();

		W->WriteObjectStart();
		W->WriteValue(TEXT("actor"), Data->GetName());
		W->WriteValue(TEXT("num_zones"), S.Zones.Num());
		W->WriteValue(TEXT("num_lanes"), S.Lanes.Num());
		W->WriteValue(TEXT("num_lane_points"), S.LanePoints.Num());

		// --- zones -----------------------------------------------------
		W->WriteArrayStart(TEXT("zones"));
		for (int32 zi = 0; zi < S.Zones.Num(); ++zi)
		{
			const FZoneData& Z = S.Zones[zi];
			W->WriteObjectStart();
			W->WriteValue(TEXT("index"), zi);
			W->WriteValue(TEXT("lanes_begin"), Z.LanesBegin);
			W->WriteValue(TEXT("lanes_end"), Z.LanesEnd);
			W->WriteArrayStart(TEXT("tags"));
			for (const FString& N : TagMaskToNames(Z.Tags)) { W->WriteValue(N); }
			W->WriteArrayEnd();
			W->WriteObjectEnd();
		}
		W->WriteArrayEnd();

		// --- lanes -----------------------------------------------------
		W->WriteArrayStart(TEXT("lanes"));
		for (int32 li = 0; li < S.Lanes.Num(); ++li)
		{
			const FZoneLaneData& L = S.Lanes[li];
			W->WriteObjectStart();
			W->WriteValue(TEXT("index"), li);
			W->WriteValue(TEXT("width_cm"), L.Width);
			W->WriteValue(TEXT("zone_index"), L.ZoneIndex);

			W->WriteArrayStart(TEXT("tags"));
			for (const FString& N : TagMaskToNames(L.Tags)) { W->WriteValue(N); }
			W->WriteArrayEnd();

			// Centreline, with the authored tangents. The tangents are what
			// let the OpenDRIVE side emit paramPoly3 with continuous headings
			// instead of a polyline that jitters at every vertex.
			W->WriteArrayStart(TEXT("points"));
			for (int32 pi = L.PointsBegin; pi < L.PointsEnd && pi < S.LanePoints.Num(); ++pi)
			{
				W->WriteObjectStart();
				W->WriteIdentifierPrefix(TEXT("pos"));
				WriteVector(W, S.LanePoints[pi]);
				if (S.LaneTangentVectors.IsValidIndex(pi))
				{
					W->WriteIdentifierPrefix(TEXT("tangent"));
					WriteVector(W, S.LaneTangentVectors[pi]);
				}
				if (S.LaneUpVectors.IsValidIndex(pi))
				{
					W->WriteIdentifierPrefix(TEXT("up"));
					WriteVector(W, S.LaneUpVectors[pi]);
				}
				if (S.LanePointProgressions.IsValidIndex(pi))
				{
					W->WriteValue(TEXT("s"), S.LanePointProgressions[pi]);
				}
				W->WriteObjectEnd();
				++TotalPoints;
			}
			W->WriteArrayEnd();

			W->WriteArrayStart(TEXT("links"));
			for (int32 ki = L.LinksBegin; ki < L.LinksEnd && ki < S.LaneLinks.Num(); ++ki)
			{
				const FZoneLaneLinkData& K = S.LaneLinks[ki];
				W->WriteObjectStart();
				W->WriteValue(TEXT("dest"), K.DestLaneIndex);
				W->WriteValue(TEXT("type"), LinkTypeToString(K.Type));
				W->WriteArrayStart(TEXT("flags"));
				for (const FString& F : LinkFlagsToNames(K.Flags)) { W->WriteValue(F); }
				W->WriteArrayEnd();
				W->WriteObjectEnd();
			}
			W->WriteArrayEnd();

			W->WriteObjectEnd();
			++TotalLanes;
		}
		W->WriteArrayEnd();

		W->WriteObjectEnd();
	}
	W->WriteArrayEnd();
	W->WriteObjectEnd();
	W->Close();
	Ar->Close();

	const int64 Size = IFileManager::Get().FileSize(*OutPath);
	UE_LOG(LogCityRoadExport, Display,
		TEXT("Wrote %s -- %lld lanes, %lld centreline points, %.1f MB"),
		*OutPath, TotalLanes, TotalPoints, (double)Size / (1024.0 * 1024.0));

	if (TotalLanes == 0)
	{
		UE_LOG(LogCityRoadExport, Warning,
			TEXT("Zero lanes exported. The ZoneGraphData actors exist but their storage "
			     "is empty - the lane graph may need rebuilding in the editor."));
		return 1;
	}

	return 0;
}

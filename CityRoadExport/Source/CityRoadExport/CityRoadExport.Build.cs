// Editor-only module providing the ExportZoneGraph commandlet.

using UnrealBuildTool;

public class CityRoadExport : ModuleRules
{
	public CityRoadExport(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
		});

		PrivateDependencyModuleNames.AddRange(new string[]
		{
			// UCommandlet, FScopedEditorWorld, LoadWorldPackageForEditor
			"UnrealEd",
			// Paired with UnrealEd, as ZoneGraph.Build.cs itself does
			"EditorFramework",
			// AZoneGraphData, FZoneGraphStorage, UZoneGraphSettings.
			// MassCore/MassEntity/DeveloperSettings come transitively from
			// ZoneGraph's own PublicDependencyModuleNames - do not list them.
			"ZoneGraph",
			// TJsonWriter for streaming the export
			"Json",
			"Projects",
		});
		// World Partition types (FWorldPartitionHelpers) live in Engine, not in
		// a WorldPartition module. WorldPartitionEditor is UI only - not wanted.
	}
}

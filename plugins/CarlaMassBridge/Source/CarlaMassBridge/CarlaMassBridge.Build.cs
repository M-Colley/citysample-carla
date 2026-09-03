// Copyright (c) 2026 City Sample -> CARLA.
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

using UnrealBuildTool;

public class CarlaMassBridge : ModuleRules
{
    public CarlaMassBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        // Carla is built with RTTI and exceptions on (see Carla.Build.cs); a
        // module that includes its headers has to match or the Boost/rpclib
        // types they pull in will not compile.
        bUseRTTI = true;
        bEnableExceptions = true;

        // FastGeoStreaming ships no Public/ folder at all - every header lives
        // under Internal/, which UBT does not expose to dependent modules. Add
        // the path explicitly; the plugin does exactly this itself for
        // Engine/Internal (see its own Build.cs).
        PrivateIncludePaths.Add(System.IO.Path.Combine(
            EngineDirectory,
            "Plugins/Experimental/FastGeoStreaming/Source/FastGeoStreaming/Internal"));

        PublicDependencyModuleNames.AddRange(new string[]
        {
            "Core",
            "CoreUObject",
            "Engine",

            // Mass. MassCore holds Mass/EntityHandle.h and Mass/EntityFragments.h
            // in 5.8; the old MassEntityHandle.h / MassEntityTypes.h paths are
            // deprecated stubs that are compiled out at IncludeOrderVersion.Latest.
            "MassCore",
            "MassEntity",
            "MassCommon",
            "MassLOD",
            "MassRepresentation",
            "MassSpawner",

            // Epic's traffic simulation - the source of the entities we mirror.
            "MassTraffic",

            // CARLA's actor registry and semantic tagger.
            "Carla",

            // City Sample streams its world geometry through this rather than
            // through actors, which is why the tagger has to reach into it.
            "FastGeoStreaming",
        });

        PrivateDependencyModuleNames.AddRange(new string[]
        {
            "MassMovement",
            "MassSimulation",
        });
    }
}

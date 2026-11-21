<#
.SYNOPSIS
    Turn Epic's City Sample project into a CARLA server.

.DESCRIPTION
    Merges CARLA into the City Sample project rather than the other way round.
    That direction is deliberate: City Sample is 83 GB of content and plugins,
    CARLA is 10 GB, and Unreal asset references are absolute ("/Game/..."), so
    whichever tree moves has to keep its relative paths. Moving the small one
    means nothing is re-pathed and the 83 GB is never copied.

    The two Content trees share no top-level name, so the merge is clean: City
    Sample keeps Map/, Vehicle/, Crowd/ ...; CARLA adds Carla/.

    What it changes in the City Sample project:
      Plugins/{Carla,CarlaTools,CarlaExporter,StreetMap}   copied (76 MB)
      Plugins/CarlaMassBridge                              copied from this repo
      Content/Carla                                        directory junction
      Source/CarlaDeviceProfileSelector                    copied
      CitySample.uproject                                  plugins + module + engine
      Config/DefaultEngine.ini                             game instance / mode
      Source/CitySampleEditor.Target.cs                    extra module
      Saved/OpenDrive/{Small,Big}_City_LVL.xodr            the exported roads
      Content/Map/Nav/{Small,Big}_City_LVL.bin             the pedestrian navmeshes

    Run it again after editing anything under CARLA's own tree or under
    plugins/CarlaMassBridge: the project holds COPIES, so a rebuild without a
    re-integrate cheerfully builds the old code.

    Everything is backed up as *.pre-carla.bak; -Undo puts it all back.

.PARAMETER Undo
    Revert: remove what was added and restore the backups.

.PARAMETER CheckOnly
    Check the source, assets, patch and map inputs without changing either checkout.

.PARAMETER OpenDrive
    Optional path to the exported Small_City_LVL OpenDRIVE file to install.

.PARAMETER NavMesh
    Optional path to the matching Small_City_LVL Recast navmesh to install.

.PARAMETER BigCityOpenDrive
    Optional path to the exported Big_City_LVL OpenDRIVE file to install.

.PARAMETER BigCityNavMesh
    Optional path to the matching Big_City_LVL Recast navmesh to install.

.EXAMPLE
    powershell -File tools\Integrate-CarlaIntoCitySample.ps1
    powershell -File tools\Integrate-CarlaIntoCitySample.ps1 -Undo
#>
[CmdletBinding()]
param(
    [string] $CarlaRoot  = $(if ($env:CARLA_DIR) { $env:CARLA_DIR }
                      else { "C:\carla-ue58\carla" }),
    [string] $CitySample = $(if ($env:CITYSAMPLE_DIR) { $env:CITYSAMPLE_DIR }
                       else { Join-Path $env:USERPROFILE "Documents\Unreal Projects\CitySample" }),
    [string] $EnginePath = $(if ($env:CARLA_UE_DIR) { $env:CARLA_UE_DIR }
                       elseif ($env:CARLA_UNREAL_ENGINE_PATH) { $env:CARLA_UNREAL_ENGINE_PATH }
                       else { "C:\carla-ue58\UnrealEngine5_carla" }),
    [string] $OpenDrive,
    [string] $NavMesh,
    [string] $BigCityOpenDrive,
    [string] $BigCityNavMesh,
    [switch] $CheckOnly,
    [switch] $Undo,
    [switch] $RevertCarlaPatch,
    [switch] $RemoveEngineRegistration
)

$ErrorActionPreference = "Stop"
$CitySample = (Get-Item -LiteralPath $CitySample -Force).FullName

$PLUGINS = @("Carla", "CarlaTools", "CarlaExporter", "StreetMap")
$MODULE  = "CarlaDeviceProfileSelector"
$BAK     = ".pre-carla.bak"
$releaseCommit = "3ea166118d2c3c094b13b15c5b7a66a9a279998d"
$releaseEngine = "b76fce0e3bcbb400efe1fe38bc2d662b33e6d989"
$releaseContent = "55228c1ccca1b698e5ad8c638c576d523bcdebb9"

$uproject  = Join-Path $CitySample "CitySample.uproject"
$engineIni = Join-Path $CitySample "Config\DefaultEngine.ini"
$scalabilityIni = Join-Path $CitySample "Config\DefaultScalability.ini"
$targetCs  = Join-Path $CitySample "Source\CitySampleEditor.Target.cs"
$carlaUE   = Join-Path $CarlaRoot  "Unreal\CarlaUnreal"
$srcContent = Join-Path $carlaUE "Content\Carla"
$srcScalabilityIni = Join-Path $carlaUE "Config\DefaultScalability.ini"
$scalabilityBegin = '; BEGIN CARLA scalability defaults'
$scalabilityEnd = '; END CARLA scalability defaults'

function Say  ([string]$m, [string]$c = "Gray") { Write-Host $m -ForegroundColor $c }
function Ok   ([string]$m) { Write-Host "  OK   $m" -ForegroundColor Green }
function Note ([string]$m) { Write-Host "  --   $m" -ForegroundColor DarkGray }

function Get-GitHead([string]$path) {
    $ErrorActionPreference = 'Continue'
    $commit = & git -C $path rev-parse --verify HEAD 2>$null
    if ($LASTEXITCODE -eq 0) { return $commit }
}

function Remove-ScalabilityDefaults([string]$content) {
    $begin = [regex]::Escape($scalabilityBegin)
    $end = [regex]::Escape($scalabilityEnd)
    return [regex]::Replace($content, "(?ms)^$begin\r?\n.*?^$end\r?\n(?:\r?\n)?", '')
}

# Deleting a junction must use rmdir, not a recursive delete: a recursive
# delete follows the link and would wipe the real content behind it.
function Remove-Tree([string]$p) {
    if (-not (Test-Path -LiteralPath $p)) { return }
    $item = Get-Item -LiteralPath $p -Force
    $projectRoot = (Resolve-Path -LiteralPath $CitySample).Path.TrimEnd('\') + '\'
    if (-not $item.FullName.StartsWith($projectRoot, [StringComparison]::OrdinalIgnoreCase)) {
        throw "path is outside the City Sample project: $($item.FullName)"
    }
    for ($parent = $item.Parent; $parent.FullName.StartsWith($projectRoot, [StringComparison]::OrdinalIgnoreCase); $parent = $parent.Parent) {
        if ($parent.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
            throw "refusing to remove $p through the linked directory $($parent.FullName)"
        }
    }
    if ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
        [System.IO.Directory]::Delete($item.FullName, $false)
    } else {
        [System.IO.Directory]::Delete($item.FullName, $true)
    }
}

# Insert $new after index $at. Slicing $lines[($at + 1)..($lines.Count - 1)]
# looks equivalent and is not: when $at is the last line that range runs
# BACKWARDS (Count..Count-1), PowerShell silently skips the out-of-range index,
# and the file's last line comes out twice. Negative indices wrap from the end,
# so $lines[0..-1] is no safer at the other end.
function Insert-After([object[]]$lines, [int]$at, [object[]]$new) {
    $head = if ($at -ge 0) { @($lines[0..$at]) } else { @() }
    $tail = if ($at + 1 -le $lines.Count - 1) { @($lines[($at + 1)..($lines.Count - 1)]) } else { @() }
    return @($head) + @($new) + @($tail)
}

# Backups are taken once, from the pristine file, so repeated runs stay
# revertible to the original rather than to the last patched state.
function Backup([string]$path) {
    $b = "$path$BAK"
    if ((Test-Path $path) -and -not (Test-Path $b)) {
        Copy-Item $path $b -Force
        Ok "backed up $(Split-Path $path -Leaf)"
    }
}
function Restore([string]$path) {
    $b = "$path$BAK"
    if (Test-Path $b) {
        Copy-Item $b $path -Force
        Remove-Item $b -Force
        Ok "restored $(Split-Path $path -Leaf)"
    } else {
        Note "no backup for $(Split-Path $path -Leaf)"
    }
}

# ---------------------------------------------------------------------------
# Undo
# ---------------------------------------------------------------------------
if ($Undo) {
    if ($CheckOnly) { throw "-CheckOnly and -Undo are mutually exclusive" }
    Say "`n=== reverting CARLA integration ===`n" "Cyan"
    foreach ($p in ($PLUGINS + "CarlaMassBridge")) {
        $t = Join-Path $CitySample "Plugins\$p"
        if (Test-Path $t) { Remove-Tree $t; Ok "removed Plugins\$p" }
    }
    $link = Join-Path $CitySample "Content\Carla"
    if (Test-Path $link) { Remove-Tree $link; Ok "removed Content\Carla" }
    $mod = Join-Path $CitySample "Source\$MODULE"
    if (Test-Path $mod) { Remove-Tree $mod; Ok "removed Source\$MODULE" }
    # The road network and the navmesh are dropped into Epic's own folders, so
    # take them back out - leaving them behind would make an "un-integrated"
    # project still behave like a CARLA one.
    foreach ($f in @("Saved\OpenDrive\Small_City_LVL.xodr",
                     "Saved\OpenDrive\Big_City_LVL.xodr",
                     "Content\Map\Nav\Small_City_LVL.bin",
                     "Content\Map\Nav\Big_City_LVL.bin")) {
        $p = Join-Path $CitySample $f
        if (Test-Path $p) { Remove-Item $p -Force; Ok "removed $f" }
    }
    Restore $uproject
    Restore $engineIni
    Restore $targetCs
    Restore (Join-Path $CitySample "Config\DefaultGame.ini")
    if (Test-Path "$scalabilityIni$BAK") {
        Restore $scalabilityIni
    } elseif (Test-Path $scalabilityIni) {
        $current = [System.IO.File]::ReadAllText($scalabilityIni)
        $remaining = Remove-ScalabilityDefaults $current
        if ($remaining -ne $current) { [System.IO.File]::WriteAllText($scalabilityIni, $remaining) }
    }
    if ($RevertCarlaPatch -and (Test-Path -LiteralPath $CarlaRoot)) {
        $head = Get-GitHead $CarlaRoot
        $pName = if ($head -eq $releaseCommit) { "carla-1.0-citysample.patch" } else { "carla-ue58-plugin-build-fixes.patch" }
        $pPath = Join-Path $PSScriptRoot "..\patches\$pName"
        if (Test-Path -LiteralPath $pPath) {
            $prevEap = $ErrorActionPreference
            $ErrorActionPreference = 'Continue'
            try {
                & git -C $CarlaRoot apply --reverse --check $pPath 2>$null
                if ($LASTEXITCODE -eq 0) {
                    & git -C $CarlaRoot apply --reverse $pPath
                    Ok "reverted $pName in $CarlaRoot"
                } else {
                    Note "$pName not currently applied in $CarlaRoot"
                }
            } finally {
                $ErrorActionPreference = $prevEap
            }
        }
    }
    if ($RemoveEngineRegistration) {
        $buildsKey = "HKCU:\Software\Epic Games\Unreal Engine\Builds"
        if (Test-Path $buildsKey) {
            $entries = @((Get-ItemProperty $buildsKey).PSObject.Properties |
                         Where-Object { $_.Name -notlike "PS*" -and $_.Value -eq $EnginePath })
            if ($entries.Count -gt 0) {
                foreach ($entry in $entries) {
                    Remove-ItemProperty -Path $buildsKey -Name $entry.Name -Force
                    Ok "removed HKCU Unreal Engine registration $($entry.Name) -> $EnginePath"
                }
            } else {
                Note "no HKCU Unreal Engine registration found for $EnginePath"
            }
        }
    }
    Say "`nreverted.`n" "Cyan"
    exit 0
}

# ---------------------------------------------------------------------------
# 1. Validate
# ---------------------------------------------------------------------------
Say "`n=== 1/10 checking inputs ===" "Cyan"
$checks = @(
    @{ Path = $uproject;                                                    Label = "CitySample.uproject" },
    @{ Path = $engineIni;                                                   Label = "City Sample engine config" },
    @{ Path = $targetCs;                                                    Label = "City Sample editor target" },
    @{ Path = (Join-Path $carlaUE "Plugins\Carla");                         Label = "CARLA plugin" },
    @{ Path = (Join-Path $carlaUE "Content\Carla");                         Label = "carla-content" },
    @{ Path = (Join-Path $carlaUE "Source\$MODULE");                         Label = "CARLA device profile module" },
    @{ Path = (Join-Path $EnginePath "Engine\Binaries\Win64\UnrealEditor.exe"); Label = "CARLA engine fork" }
)
foreach ($c in $checks) {
    if (-not (Test-Path $c.Path)) { throw "missing $($c.Label): $($c.Path)" }
    Ok $c.Label
}

# CarlaGameMode is the asset the whole integration hinges on. If git-lfs has
# not resolved it, the project still opens - to a black screen with no server -
# which is a miserable thing to debug, so fail loudly here instead.
$gm = Join-Path $carlaUE "Content\Carla\Blueprints\Game\CarlaGameMode.uasset"
if (-not (Test-Path $gm)) {
    throw "CarlaGameMode.uasset missing - run tools/fetch-carla-content.sh first"
}
$sig = [System.Text.Encoding]::ASCII.GetString([System.IO.File]::ReadAllBytes($gm)[0..20])
if ($sig.StartsWith("version https")) {
    throw "CarlaGameMode.uasset is still a git-lfs pointer - run 'git lfs pull' in the content repo"
}
Ok "CarlaGameMode.uasset resolved"

# Check the source and generated inputs before modifying the project or registry.
$sourceHead = Get-GitHead $CarlaRoot
if (-not $sourceHead) { throw "CARLA source must be a Git checkout: $CarlaRoot" }
$isRelease = $sourceHead -eq $releaseCommit
$patchName = if ($isRelease) { "carla-1.0-citysample.patch" } else { "carla-ue58-plugin-build-fixes.patch" }
$patch = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..\patches\$patchName")).Path
Ok "CARLA $sourceHead ($patchName)"

if ($isRelease) {
    $engineHead = Get-GitHead $EnginePath
    if ($engineHead -ne $releaseEngine) {
        throw "CARLA 1.0 needs the built engine at $releaseEngine. Found '$engineHead' at $EnginePath."
    }
    $contentHead = Get-GitHead $srcContent
    if ($contentHead -ne $releaseContent) {
        throw "CARLA 1.0 needs content at $releaseContent. Found '$contentHead' at $srcContent. Run tools/fetch-carla-content.sh."
    }
    if (-not (Test-Path -LiteralPath $srcScalabilityIni -PathType Leaf)) {
        throw "missing CARLA scalability settings: $srcScalabilityIni"
    }
    Ok "CARLA 1.0 engine and content revisions"
}

$skipCreated = @()
$eap = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
try {
    & git -C $CarlaRoot apply --reverse --check $patch 2>$null
    $patchApplied = $LASTEXITCODE -eq 0
    if (-not $patchApplied) {
        # Git reset leaves patch-created files untracked. Skip a creation
        # hunk only when Git confirms the file matches the patch.
        $creates = [regex]::Matches((Get-Content -LiteralPath $patch -Raw), '(?m)^--- /dev/null\r?\n\+\+\+ b/([^\r\n]+)')
        foreach ($creation in $creates) {
            $rel = $creation.Groups[1].Value
            if (-not (Test-Path -LiteralPath (Join-Path $CarlaRoot $rel) -PathType Leaf)) { continue }
            & git -C $CarlaRoot apply --reverse --check "--include=$rel" $patch 2>$null
            if ($LASTEXITCODE -eq 0) { $skipCreated += "--exclude=$rel" }
        }
        & git -C $CarlaRoot apply --check @skipCreated $patch
        if ($LASTEXITCODE -ne 0) {
            throw "Cannot apply $patchName at $sourceHead. Use CARLA tag 1.0 or the legacy revision de3f38e64."
        }
    }
} finally {
    $ErrorActionPreference = $eap
}
if ($patchApplied) { Ok "CARLA patch already applied" } else { Ok "CARLA patch applies cleanly" }

$mapSpecs = @(
    @{
        MapName      = "Small_City_LVL"
        Prefix       = "SmallCity"
        XodrParam    = $OpenDrive
        NavParam     = $NavMesh
        DeployedXodr = (Join-Path $CitySample "Saved\OpenDrive\Small_City_LVL.xodr")
        DeployedNav  = (Join-Path $CitySample "Content\Map\Nav\Small_City_LVL.bin")
        XodrSrc      = $null
        NavSrc       = $null
        NavBytes     = 0
        SignalCount  = 0
    },
    @{
        MapName      = "Big_City_LVL"
        Prefix       = "BigCity"
        XodrParam    = $BigCityOpenDrive
        NavParam     = $BigCityNavMesh
        DeployedXodr = (Join-Path $CitySample "Saved\OpenDrive\Big_City_LVL.xodr")
        DeployedNav  = (Join-Path $CitySample "Content\Map\Nav\Big_City_LVL.bin")
        XodrSrc      = $null
        NavSrc       = $null
        NavBytes     = 0
        SignalCount  = 0
    }
)

foreach ($ms in $mapSpecs) {
    $xodrSrc = $ms.XodrParam
    if (-not $xodrSrc) {
        foreach ($name in @("$($ms.Prefix)-signals.xodr", "$($ms.Prefix).xodr")) {
            $candidate = Join-Path $PSScriptRoot "..\$name"
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { $xodrSrc = $candidate; break }
        }
    }
    $navSrc = $ms.NavParam
    if (-not $navSrc) {
        $candidate = Join-Path $PSScriptRoot "..\$($ms.Prefix).bin"
        if (Test-Path -LiteralPath $candidate -PathType Leaf) { $navSrc = $candidate }
    }
    $ms.XodrSrc = $xodrSrc
    $ms.NavSrc = $navSrc

    $xodrCheck = $xodrSrc
    if (-not $xodrCheck -and (Test-Path -LiteralPath $ms.DeployedXodr -PathType Leaf)) {
        $xodrCheck = $ms.DeployedXodr
    }
    if ($xodrCheck) {
        $xml = New-Object System.Xml.XmlDocument
        $xml.Load((Resolve-Path -LiteralPath $xodrCheck).Path)
        $ids = [System.Collections.Generic.HashSet[string]]::new()
        foreach ($road in $xml.SelectNodes('/OpenDRIVE/road')) { $null = $ids.Add($road.GetAttribute('id')) }
        if ($ids.Count -eq 0) { throw "No OpenDRIVE roads in $xodrCheck." }
        $collisions = @($xml.SelectNodes('/OpenDRIVE/junction') | Where-Object { $ids.Contains($_.GetAttribute('id')) })
        if ($collisions.Count -gt 0) {
            throw "$xodrCheck contains $($collisions.Count) overlapping road and junction IDs. Regenerate the OpenDRIVE file."
        }
        $ms.SignalCount = $xml.SelectNodes('/OpenDRIVE/road/signals/signal').Count
        Ok "$($ms.MapName) OpenDRIVE road and junction IDs are disjoint"
    }
    if ($navSrc) {
        $ms.NavBytes = (Get-Item -LiteralPath $navSrc).Length
        if ($ms.NavBytes -le 64) { throw "The Recast mesh at $navSrc is empty. Rebuild it from the OpenDRIVE file." }
    }
}

# UBT reads these files from each plugin. Generate them before copying.
$defSrc = Join-Path $CarlaRoot "Build\Unreal"
$defs = @("Includes.def", "Libraries.def", "Options.def", "Definitions.def")
$missingDefs = @($defs | Where-Object { -not (Test-Path -LiteralPath (Join-Path $defSrc $_) -PathType Leaf) })
if ($missingDefs -and -not $CheckOnly) {
    & cmake --build (Join-Path $CarlaRoot "Build") --target carla-unreal-configure
    if ($LASTEXITCODE -eq 0) {
        $missingDefs = @($defs | Where-Object { -not (Test-Path -LiteralPath (Join-Path $defSrc $_) -PathType Leaf) })
    }
}
if ($missingDefs) {
    throw "Missing $($missingDefs -join ', ') in $defSrc. Run SetupWithBuildTools.bat."
}
Ok "generated CARLA build definitions"

if ($CheckOnly) { exit 0 }
if (-not $patchApplied) {
    & git -C $CarlaRoot apply @skipCreated $patch
    if ($LASTEXITCODE -ne 0) { throw "git apply failed after its own check passed: $patchName" }
    Ok "applied $patchName"
}

# ---------------------------------------------------------------------------
# 2. Register the engine fork so EngineAssociation resolves
# ---------------------------------------------------------------------------
Say "`n=== 2/10 registering the CARLA engine fork ===" "Cyan"
$buildsKey = "HKCU:\Software\Epic Games\Unreal Engine\Builds"
if (-not (Test-Path $buildsKey)) { New-Item -Path $buildsKey -Force | Out-Null }
$already = (Get-ItemProperty $buildsKey).PSObject.Properties |
           Where-Object { $_.Name -notlike "PS*" -and $_.Value -eq $EnginePath }
if ($already) {
    $engineId = @($already)[0].Name
    Note "already registered as $engineId"
} else {
    $engineId = "{" + [guid]::NewGuid().ToString().ToUpper() + "}"
    New-ItemProperty -Path $buildsKey -Name $engineId -Value $EnginePath -PropertyType String -Force | Out-Null
    Ok "registered $engineId"
}

# ---------------------------------------------------------------------------
# 3. Plugins - copied, not linked, so each project keeps its own build output
# ---------------------------------------------------------------------------
Say "`n=== 3/10 copying CARLA plugins ===" "Cyan"
foreach ($p in $PLUGINS) {
    $src = Join-Path $carlaUE "Plugins\$p"
    $dst = Join-Path $CitySample "Plugins\$p"
    if (-not (Test-Path $src)) { Note "$p not present, skipping"; continue }
    Remove-Tree $dst
    Copy-Item $src $dst -Recurse -Force
    foreach ($junk in @("Binaries", "Intermediate")) {
        Remove-Tree (Join-Path $dst $junk)
    }
    Ok "Plugins\$p"
}

# CarlaMassBridge is OURS, and it lives in this repository rather than in
# CARLA's tree - so it is copied from here. It carries two things CARLA cannot
# do by itself on a City Sample map: the Mass -> CARLA actor bridge, and the
# FastGeo semantic tagger.
#
# Note the asymmetry with the loop above: those plugins are re-copied from the
# CARLA tree on every run, so editing them in the project is pointless (the
# next integrate overwrites them - edit CARLA's tree and re-run). This one is
# copied from the repo for the same reason. Edit it HERE.
$bridgeSrc = Join-Path $PSScriptRoot "..\plugins\CarlaMassBridge"
if (Test-Path $bridgeSrc) {
    $bridgeDst = Join-Path $CitySample "Plugins\CarlaMassBridge"
    # Keep Binaries/Intermediate: this is the plugin under active development
    # and blowing away its build output on every integrate costs a full
    # rebuild for nothing.
    if (-not (Test-Path $bridgeDst)) { New-Item -ItemType Directory -Path $bridgeDst -Force | Out-Null }
    foreach ($item in Get-ChildItem (Resolve-Path $bridgeSrc)) {
        if ($item.Name -in @("Binaries", "Intermediate")) { continue }
        $target = Join-Path $bridgeDst $item.Name
        # Copy-Item -Recurse onto an EXISTING directory copies INTO it, so a
        # second run would produce Source\Source\... and UBT then finds two
        # CarlaMassBridge.Build.cs files and fails with CS0101. Clear the
        # target first.
        if ($item.PSIsContainer) { Remove-Tree $target }
        Copy-Item $item.FullName $target -Recurse -Force
    }
    Ok "Plugins\CarlaMassBridge"
} else {
    Note "plugins\CarlaMassBridge not found - no Mass bridge, no FastGeo tagger"
}

foreach ($p in @("Carla", "CarlaTools")) {
    $dst = Join-Path $CitySample "Plugins\$p"
    if (-not (Test-Path $dst)) { continue }
    foreach ($d in $defs) {
        $s = Join-Path $defSrc $d
        if (-not (Test-Path $s)) { throw "missing generated $d - run 'cmake --build Build --target carla-unreal-configure' in $CarlaRoot" }
        Copy-Item $s (Join-Path $dst $d) -Force
    }
    Ok "Plugins\$p\*.def (4 files)"
}

# ---------------------------------------------------------------------------
# 4. Content - junctioned. 10 GB, read-only in practice, and it stays under
#    git-lfs control in one place.
# ---------------------------------------------------------------------------
Say "`n=== 4/10 linking CARLA content ===" "Cyan"
$dstContent = Join-Path $CitySample "Content\Carla"
Remove-Tree $dstContent
New-Item -ItemType Junction -Path $dstContent -Target $srcContent | Out-Null
Ok "Content\Carla -> $srcContent"

# ---------------------------------------------------------------------------
# 5. Device profile module (CARLA's per-quality-tier render CVars)
# ---------------------------------------------------------------------------
Say "`n=== 5/10 copying the $MODULE module ===" "Cyan"
$dstMod = Join-Path $CitySample "Source\$MODULE"
Remove-Tree $dstMod
Copy-Item (Join-Path $carlaUE "Source\$MODULE") $dstMod -Recurse -Force
Ok "Source\$MODULE"

# ---------------------------------------------------------------------------
# 6. .uproject and the editor target
# ---------------------------------------------------------------------------
Say "`n=== 6/10 patching CitySample.uproject ===" "Cyan"
Backup $uproject
$j = Get-Content $uproject -Raw -Encoding UTF8 | ConvertFrom-Json

$j.EngineAssociation = $engineId
Ok "EngineAssociation = $engineId"

$mods = [System.Collections.ArrayList]@($j.Modules)
if (-not ($mods | Where-Object { $_.Name -eq $MODULE })) {
    $null = $mods.Add([pscustomobject]@{ Name = $MODULE; Type = "Runtime"; LoadingPhase = "PostConfigInit" })
    Ok "module $MODULE"
} else {
    Note "module $MODULE already listed"
}
$j.Modules = $mods.ToArray()

$plugs = [System.Collections.ArrayList]@($j.Plugins)
# CarlaMassBridge has no "EnabledByDefault" in its .uplugin, so a project
# that does not list it simply never loads it - the Mass bridge and the
# FastGeo tagger are then silently absent, CVars and all.
foreach ($p in @("Carla", "CarlaTools", "CarlaMassBridge", "EditorScriptingUtilities")) {
    $hit = @($plugs | Where-Object { $_.Name -eq $p })
    if ($hit.Count -gt 0) {
        $hit[0].Enabled = $true
        Note "plugin $p already listed, ensured enabled"
    } else {
        $null = $plugs.Add([pscustomobject]@{ Name = $p; Enabled = $true })
        Ok "plugin $p"
    }
}
$j.Plugins = $plugs.ToArray()

$j | ConvertTo-Json -Depth 12 | Set-Content $uproject -Encoding UTF8
Ok "written"

Backup $targetCs
$tc = Get-Content $targetCs -Raw
if ($tc -notmatch [regex]::Escape($MODULE)) {
    # UBT only compiles modules the target lists, so adding it to the .uproject
    # alone is not enough.
    $anchor = 'ExtraModuleNames.Add("CitySampleAnimGraphRuntime");'
    $tc = $tc.Replace($anchor, $anchor + "`r`n`t`tExtraModuleNames.Add(`"$MODULE`");")
    Set-Content $targetCs -Value $tc -Encoding UTF8
    Ok "CitySampleEditor.Target.cs += $MODULE"
} else {
    Note "target already lists $MODULE"
}

# ---------------------------------------------------------------------------
# 7. DefaultEngine.ini
# ---------------------------------------------------------------------------
Say "`n=== 7/10 patching Config\DefaultEngine.ini ===" "Cyan"
Backup $engineIni
$lines = @(Get-Content $engineIni)

# CARLA's RPC listener on port 2000 is started by UCarlaGameInstance, and the
# episode/actor registry lives in ACarlaGameModeBase. Both have to replace City
# Sample's own, or the editor runs the city with nothing listening and no way
# to spawn a vehicle.
$replacements = [ordered]@{
    "GameInstanceClass"      = "/Script/Carla.CarlaGameInstance"
    "GlobalDefaultGameMode"  = "/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C"
    "WorldSettingsClassName" = "/Script/Carla.AutowareWorldSettings"
}
foreach ($k in $replacements.Keys) {
    $hit = $false
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($lines[$i] -match "^\s*$k\s*=") { $lines[$i] = "$k=$($replacements[$k])"; $hit = $true }
    }
    if ($hit) { Ok "$k = $($replacements[$k])" } else { Note "$k not found - left alone" }
}

# --- CARLA ray-cast sensor collision channels -----------------------------
# CARLA hardcodes ECC_GameTraceChannel2/3. City Sample already owns channels
# 1..7, so channel 2 means "BallisticLandingTarget" (DefaultResponse=Ignore)
# here and EVERY ray-cast sensor - lidar, semantic lidar, HSS lidar, radar -
# returned frames with zero points. 8/9/10 are the next free slots, and they
# must be >7: CollisionProfile sorts DefaultChannelResponses ascending before
# filling ObjectTypeMapping/TraceTypeMapping, so appending leaves every baked
# EObjectTypeQuery index in City Sample's Blueprints untouched.
if (($lines -join "`n") -notmatch 'Name="SensorTrace"') {
    $chanIdx = ($lines | Select-String -Pattern '^\[/Script/Engine\.CollisionProfile\]' | Select-Object -First 1).LineNumber
    if ($chanIdx) {
        # Insert after the last entry of that section.
        $end = $chanIdx
        for ($i = $chanIdx; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match '^\[' -and $i -gt $chanIdx - 1) { break }
            $end = $i
        }
        $block = @(
            '; --- CARLA ray-cast sensor collision channels (added by Integrate-CarlaIntoCitySample.ps1) ---',
            '; City Sample owns GameTraceChannel1..7 (Ballistic, BallisticLandingTarget,',
            '; Interactions, Footstep, VehicleComponent, VehicleFrame, VehicleWheels), so',
            "; CARLA's own slots are unavailable here: its hardcoded ECC_GameTraceChannel2",
            '; means "BallisticLandingTarget" in this project, and that channel defaults to',
            '; ECR_Ignore - which is why every ray-cast sensor returned zero points, with no',
            '; error anywhere. 8/9/10 are the next free slots.',
            '; They must be >7. CollisionProfile.cpp sorts DefaultChannelResponses ascending',
            '; before filling ObjectTypeMapping/TraceTypeMapping, so appending at 8/9/10',
            "; leaves every baked EObjectTypeQuery / ETraceTypeQuery index in City Sample's",
            '; Blueprints unchanged. Inserting below 7 would silently rewire them.',
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel8,DefaultResponse=ECR_Ignore,bTraceType=False,bStaticObject=False,Name="SensorObject")',
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel9,DefaultResponse=ECR_Block,bTraceType=True,bStaticObject=False,Name="SensorTrace")',
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel10,DefaultResponse=ECR_Overlap,bTraceType=True,bStaticObject=False,Name="OverlapChannel")'
        )
        foreach ($prof in @("OverlapAll", "OverlapAllDynamic", "OverlapOnlyPawn", "Trigger", "UI",
                            "Spectator", "InvisibleWall", "InvisibleWallDynamic", "IgnoreAll",
                            "CapsuleWhileRagdolled", "PawnInteraction", "CitySampleInteraction")) {
            $block += "+EditProfiles=(Name=`"$prof`",CustomResponses=((Channel=`"SensorTrace`",Response=ECR_Ignore),(Channel=`"OverlapChannel`",Response=ECR_Ignore)))"
        }
        $lines = Insert-After $lines $end $block
        Ok "collision channels 8/9/10 (SensorObject / SensorTrace / OverlapChannel)"
    } else {
        Note "[/Script/Engine.CollisionProfile] not found - add the channels by hand"
    }
} else { Note "collision channels already present" }

if (($lines -join "`n") -notmatch 'Name="CustomSensorCollision"') {
    $channelSection = ($lines | Select-String -Pattern '^\[/Script/Engine\.CollisionProfile\]' | Select-Object -First 1).LineNumber
    if ($channelSection) {
        $ignoredChannels = @("WorldStatic", "WorldDynamic", "Pawn", "Visibility", "Camera",
                             "PhysicsBody", "Vehicle", "Destructible", "Ballistic", "BallisticLandingTarget",
                             "Interactions", "Footstep", "VehicleComponent", "VehicleFrame", "VehicleWheels")
        $responses = @($ignoredChannels | ForEach-Object { '(Channel="{0}",Response=ECR_Ignore)' -f $_ })
        $responses += '(Channel="SensorObject",Response=ECR_Block)', '(Channel="SensorTrace",Response=ECR_Block)'
        $profile = '+Profiles=(Name="CustomSensorCollision",CollisionEnabled=QueryOnly,bCanModify=True,ObjectTypeName="SensorObject",CustomResponses=({0}),HelpMessage="Collision geometry for CARLA ray cast sensors")' -f ($responses -join ',')
        $lines = Insert-After $lines ($channelSection - 1) @($profile)
        Ok "CustomSensorCollision profile"
    }
}

# --- segmentation render pass --------------------------------------------
# The engine fork declares r.CARLA.EnableSegmentationRendering with a default
# of 0 and gates the segmentation pass on it. At 0 the pass never draws, but
# the buffer is still bound, so the segmentation camera sampled stale memory
# and returned labels outside the valid 0..29 range. Must be [SystemSettings]:
# it is a plain console variable, so [/Script/Engine.RendererSettings] drops it.
if (($lines -join "`n") -notmatch 'r\.CARLA\.EnableSegmentationRendering') {
    $sysIdx = ($lines | Select-String -Pattern '^\[SystemSettings\]' | Select-Object -First 1).LineNumber
    if ($sysIdx) {
        # LineNumber is 1-based, so $sysIdx - 1 is the header's own index.
        $lines = Insert-After $lines ($sysIdx - 1) @('r.CARLA.EnableSegmentationRendering=1')
    } else {
        $lines += @('', '[SystemSettings]', 'r.CARLA.EnableSegmentationRendering=1')
    }
    Ok "r.CARLA.EnableSegmentationRendering=1"
} else { Note "segmentation CVar already present" }

Set-Content $engineIni -Value $lines -Encoding UTF8

$currentScalability = if (Test-Path -LiteralPath $scalabilityIni) {
    [System.IO.File]::ReadAllText($scalabilityIni)
} else { "" }
$projectScalability = Remove-ScalabilityDefaults $currentScalability
if ($isRelease) {
    Backup $scalabilityIni
    $carlaScalability = (Get-Content -LiteralPath $srcScalabilityIni -Raw).TrimEnd()
    # City Sample settings follow the CARLA defaults and take precedence.
    $projectScalability = @($scalabilityBegin, $carlaScalability, $scalabilityEnd, "", $projectScalability) -join [Environment]::NewLine
}
if ($projectScalability -ne $currentScalability) {
    [System.IO.File]::WriteAllText($scalabilityIni, $projectScalability)
    Ok "updated CARLA defaults in Config\DefaultScalability.ini"
}

# ---------------------------------------------------------------------------
# 8. The road network CARLA drives on
# ---------------------------------------------------------------------------
Say "`n=== 7b/10 installing the semantic label rules ===" "Cyan"
#
# These go in the PROJECT's Config\DefaultGame.ini, not CARLA's. GConfig reads
# GGameIni, which for a launch of CitySample.uproject is the City Sample
# project's file - a [CarlaTagger] section in the CARLA source tree's
# DefaultGame.ini is never loaded and has no effect at all.
$tagIni = Join-Path $PSScriptRoot "..\config\CarlaTagger.DefaultGame.ini"
$gameIni = Join-Path $CitySample "Config\DefaultGame.ini"
if (Test-Path $tagIni) {
    if (Test-Path $gameIni) {
        Backup $gameIni
        $existing = Get-Content $gameIni -Raw
    } else {
        $existing = ""
    }
    if ($existing -match '(?m)^\[CarlaTagger\]') {
        Note "[CarlaTagger] already present, left as it is"
    } else {
        # Append. Rewriting the section would discard any rules the user added.
        $block = Get-Content (Resolve-Path $tagIni) -Raw
        $sep = if ($existing -and -not $existing.EndsWith("`n")) { "`r`n`r`n" } else { "`r`n" }
        [System.IO.File]::AppendAllText($gameIni, $sep + $block)
        Ok "Config\DefaultGame.ini  [CarlaTagger] appended"
    }
} else {
    Note "config\CarlaTagger.DefaultGame.ini not found - label rules use the built-in table only"
}

Say "`n=== 8/10 installing the extracted road network ===" "Cyan"
# Rendering the City Sample is only half of it. CARLA's own map - waypoints,
# spawn points, lane topology, the traffic manager - comes from OpenDRIVE, and
# a City Sample level ships none. Without this, world.get_map() fails outright
# with "failed to generate map".
#
# UOpenDrive::FindPathToXODRFile checks Saved/OpenDrive/<MapName>.xodr first,
# ahead of the content directory, so the extracted network drops in there under
# the level's own name and needs no cooking or asset import.
#
# Prefer <City>-signals.xodr when it exists. add_signals_to_xodr.py writes
# the traffic lights into that file and leaves the raw export alone, so
# installing the raw one silently turns every traffic light back off - the
# lights are only in the .xodr, nowhere else.
foreach ($ms in $mapSpecs) {
    if ($ms.XodrSrc) {
        $savedDir = Split-Path $ms.DeployedXodr -Parent
        if (-not (Test-Path $savedDir)) { New-Item -ItemType Directory -Path $savedDir -Force | Out-Null }
        $resolvedSrc = (Resolve-Path -LiteralPath $ms.XodrSrc).Path
        if ($resolvedSrc -ne $ms.DeployedXodr) { Copy-Item -LiteralPath $resolvedSrc -Destination $ms.DeployedXodr -Force }
        Ok "Saved\OpenDrive\$($ms.MapName).xodr  ($(Split-Path $resolvedSrc -Leaf), $($ms.SignalCount) signals)"
        if ($ms.SignalCount -eq 0) {
            Note "no traffic lights in $($ms.MapName).xodr - see README part B step 8 to add them"
        }
    } elseif (-not (Test-Path -LiteralPath $ms.DeployedXodr -PathType Leaf)) {
        Note "$($ms.Prefix).xodr not found - run the export pipeline to enable $($ms.MapName)"
    }
}

Say "`n=== 9/10 installing the pedestrian navmesh ===" "Cyan"
#
# NOT Saved\Nav\. The client asks for the navmesh via get_required_files("Nav"),
# and that RPC only searches Saved/ for generated OpenDRIVE worlds; for a map
# that already exists it walks the map's CONTENT folder and nothing else. A .bin
# in Saved\Nav\ is ignored without a word in any log, on either side.
foreach ($ms in $mapSpecs) {
    if ($ms.NavSrc) {
        $navDir = Split-Path $ms.DeployedNav -Parent
        if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir -Force | Out-Null }
        $resolvedNav = (Resolve-Path -LiteralPath $ms.NavSrc).Path
        if ($resolvedNav -ne $ms.DeployedNav) { Copy-Item -LiteralPath $resolvedNav -Destination $ms.DeployedNav -Force }
        Ok "Content\Map\Nav\$($ms.MapName).bin  ($('{0:N0}' -f $ms.NavBytes) bytes)"
    } elseif (-not (Test-Path -LiteralPath $ms.DeployedNav -PathType Leaf)) {
        Note "$($ms.Prefix).bin not found - walkers on $($ms.MapName) need a Recast navmesh; see README part B step 11"
    }
}

Say "`n=== done ===" "Cyan"
Write-Host @"

Next:
  1. build    powershell -File tools\Build-CarlaCity.ps1
  2. run      powershell -File tools\Run-CarlaCity.ps1
  3. connect  python tools\carla_probe.py

Revert at any time with -Undo.
"@ -ForegroundColor White

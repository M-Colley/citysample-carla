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
      Saved/OpenDrive/Small_City_LVL.xodr                  the exported roads
      Content/Map/Nav/Small_City_LVL.bin                   the pedestrian navmesh

    Run it again after editing anything under CARLA's own tree or under
    plugins/CarlaMassBridge: the project holds COPIES, so a rebuild without a
    re-integrate cheerfully builds the old code.

    Everything is backed up as *.pre-carla.bak; -Undo puts it all back.

.PARAMETER Undo
    Revert: remove what was added and restore the backups.

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
                       else { "C:\carla-ue58\UnrealEngine5_carla" }),
    [switch] $Undo
)

$ErrorActionPreference = "Stop"

$PLUGINS = @("Carla", "CarlaTools", "CarlaExporter", "StreetMap")
$MODULE  = "CarlaDeviceProfileSelector"
$BAK     = ".pre-carla.bak"

$uproject  = Join-Path $CitySample "CitySample.uproject"
$engineIni = Join-Path $CitySample "Config\DefaultEngine.ini"
$targetCs  = Join-Path $CitySample "Source\CitySampleEditor.Target.cs"
$carlaUE   = Join-Path $CarlaRoot  "Unreal\CarlaUnreal"

function Say  ([string]$m, [string]$c = "Gray") { Write-Host $m -ForegroundColor $c }
function Ok   ([string]$m) { Write-Host "  OK   $m" -ForegroundColor Green }
function Note ([string]$m) { Write-Host "  --   $m" -ForegroundColor DarkGray }

# Deleting a junction must use rmdir, not a recursive delete: a recursive
# delete follows the link and would wipe the real content behind it.
function Remove-Tree([string]$p) {
    if (-not (Test-Path $p)) { return }
    $item = Get-Item $p -Force
    if ($item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
        [System.IO.Directory]::Delete($p, $false)
    } else {
        [System.IO.Directory]::Delete($p, $true)
    }
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
                     "Content\Map\Nav\Small_City_LVL.bin")) {
        $p = Join-Path $CitySample $f
        if (Test-Path $p) { Remove-Item $p -Force; Ok "removed $f" }
    }
    Restore $uproject
    Restore $engineIni
    Restore $targetCs
    Restore (Join-Path $CitySample "Config\DefaultGame.ini")
    Say "`nreverted.`n" "Cyan"
    exit 0
}

# ---------------------------------------------------------------------------
# 1. Validate
# ---------------------------------------------------------------------------
Say "`n=== 1/10 checking inputs ===" "Cyan"
$checks = @(
    @{ Path = $uproject;                                                    Label = "CitySample.uproject" },
    @{ Path = (Join-Path $carlaUE "Plugins\Carla");                         Label = "CARLA plugin" },
    @{ Path = (Join-Path $carlaUE "Content\Carla");                         Label = "carla-content" },
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
# 2b. Source fixes the plugin needs to compile at all
# ---------------------------------------------------------------------------
Say "`n=== 2b/10 patching the CARLA plugin source ===" "Cyan"
# Three genuine compile errors on ue58-dev-carla @ de3f38e64 that UBT hits and
# CARLA's own cmake build never does, because cmake builds LibCarla and the
# Python API but not the Unreal plugin. See ISSUES-TO-FILE.md, issue 3.
$patch = Join-Path $PSScriptRoot "..\patches\carla-ue58-plugin-build-fixes.patch"
if (Test-Path $patch) {
    $patch = (Resolve-Path $patch).Path
    Push-Location $CarlaRoot
    # In Windows PowerShell 5.1, redirecting a NATIVE command's stderr (2>$null
    # or 2>&1) wraps each stderr line in a NativeCommandError ErrorRecord - and
    # under $ErrorActionPreference='Stop' that record is TERMINATING. It kills
    # the script on the redirection itself, before $LASTEXITCODE is ever read.
    #
    # That mattered here on every clean checkout: `git apply --reverse --check`
    # is *expected* to fail when the patch is not applied yet, and it writes to
    # stderr when it does. So the first run of this script died at step 2b with
    # a NativeCommandError stack trace, having registered the engine and done
    # nothing else. git's own stderr is useful; let it through.
    $eap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        # --reverse --check succeeds when the patch is already in the tree.
        & git apply --reverse --check $patch
        if ($LASTEXITCODE -eq 0) {
            Note "already patched"
        } else {
            # The patch CREATES files as well as modifying them. A hand-undo
            # with `git reset --hard` restores the tracked files but leaves the
            # created ones behind, because reset does not touch untracked files.
            # Both checks then fail forever: reverse-check because the tracked
            # files are pristine again, forward-check because the created file
            # "already exists in working directory". The script used to throw
            # "the CARLA source has moved on; re-derive it", sending the user to
            # regenerate a patch that was perfectly fine.
            #
            # --3way does NOT rescue this: for a creation hunk it looks the path
            # up in the index, finds it untracked, and bails the same way. So
            # remove the patch's own leftovers first - only ones git agrees are
            # untracked, so a real source file is never deleted.
            & git apply --check $patch
            if ($LASTEXITCODE -ne 0) {
                $created = @()
                foreach ($line in (Get-Content $patch)) {
                    if ($line -match '^\+\+\+ b/(.+)$') { $created += $Matches[1] }
                }
                $cleaned = @()
                foreach ($rel in ($created | Select-Object -Unique)) {
                    if ([string]::IsNullOrWhiteSpace($rel)) { continue }
                    if ($rel -eq "/dev/null") { continue }
                    $full = Join-Path $CarlaRoot $rel
                    if (-not (Test-Path $full -PathType Leaf)) { continue }
                    # Never delete outside the CARLA tree, whatever the patch says.
                    $resolved = (Resolve-Path $full).Path
                    $rootFull = (Resolve-Path $CarlaRoot).Path.TrimEnd('\') + '\'
                    if (-not $resolved.StartsWith($rootFull, [StringComparison]::OrdinalIgnoreCase)) { continue }
                    & git ls-files --error-unmatch -- $rel 2>&1 | Out-Null
                    if ($LASTEXITCODE -ne 0) {      # untracked => the patch made it
                        Remove-Item -LiteralPath $resolved -Force
                        $cleaned += $rel
                    }
                }
                if ($cleaned.Count) {
                    Note "removed $($cleaned.Count) leftover file(s) from a previous apply"
                    & git apply --check $patch
                }
            }
            if ($LASTEXITCODE -ne 0) {
                # Do NOT fall back to --3way. It is not atomic: it applies what
                # it can, writes conflict markers into what it cannot, stages
                # the lot, and returns non-zero - so the user is left with a
                # half-patched CARLA tree and a conflicted index, which is far
                # worse than not applying at all. Plain `git apply` is
                # all-or-nothing, so failing the check means nothing was touched.
                throw ("cannot apply $patch to $CarlaRoot - see git's output above. " +
                       "Nothing was changed. Most often this is a CARLA checkout " +
                       "at a different commit: this patch is against " +
                       "ue58-dev-carla @ de3f38e64. Check with " +
                       "``git -C $CarlaRoot rev-parse --short HEAD``.")
            }
            & git apply $patch
            if ($LASTEXITCODE -ne 0) { throw "git apply failed after its own --check passed - see above" }
            Ok "applied carla-ue58-plugin-build-fixes.patch"
        }
    } finally {
        $ErrorActionPreference = $eap
        Pop-Location
    }
} else {
    Note "no patch file - assuming the plugin already compiles"
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

# The Carla and CarlaTools build rules read four .def files from their plugin
# root - the include paths, libraries, preprocessor definitions and options
# that CARLA's cmake build resolved. They are generated and .gitignore'd, and
# only the "carla-unreal-configure" cmake target writes them, which a plain
# "cmake --build Build" does not run. Without them UBT fails immediately with
#   Unable to instantiate module 'Carla': Could not find file '...Definitions.def'
# The paths inside are absolute, so they are valid from any project.
$defSrc = Join-Path $CarlaRoot "Build\Unreal"
$defs   = @("Includes.def", "Libraries.def", "Options.def", "Definitions.def")
if (-not (Test-Path (Join-Path $defSrc "Definitions.def"))) {
    Note "generating .def files (cmake target carla-unreal-configure)"
    Push-Location $CarlaRoot
    # Same native-stderr trap as the git calls above: `2>&1 | Out-Null` under
    # EAP=Stop turns cmake's ordinary warning output into a terminating error,
    # so the script died here instead of reaching the far more helpful throw
    # below - and this branch runs exactly when the .def files are missing,
    # which is the normal state on a fresh checkout.
    $eap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        & cmake --build Build --target carla-unreal-configure
        if ($LASTEXITCODE -ne 0) {
            Note "cmake exited $LASTEXITCODE - see its output above"
        }
    } finally {
        $ErrorActionPreference = $eap
        Pop-Location
    }
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
$srcContent = Join-Path $carlaUE "Content\Carla"
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
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel8,DefaultResponse=ECR_Ignore,bTraceType=False,bStaticObject=False,Name="SensorObject")',
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel9,DefaultResponse=ECR_Block,bTraceType=True,bStaticObject=False,Name="SensorTrace")',
            '+DefaultChannelResponses=(Channel=ECC_GameTraceChannel10,DefaultResponse=ECR_Overlap,bTraceType=True,bStaticObject=False,Name="OverlapChannel")'
        )
        foreach ($prof in @("OverlapAll", "OverlapAllDynamic", "OverlapOnlyPawn", "Trigger", "UI",
                            "Spectator", "InvisibleWall", "InvisibleWallDynamic", "IgnoreAll",
                            "CapsuleWhileRagdolled", "PawnInteraction", "CitySampleInteraction")) {
            $block += "+EditProfiles=(Name=`"$prof`",CustomResponses=((Channel=`"SensorTrace`",Response=ECR_Ignore),(Channel=`"OverlapChannel`",Response=ECR_Ignore)))"
        }
        $lines = $lines[0..$end] + $block + $lines[($end + 1)..($lines.Count - 1)]
        Ok "collision channels 8/9/10 (SensorObject / SensorTrace / OverlapChannel)"
    } else {
        Note "[/Script/Engine.CollisionProfile] not found - add the channels by hand"
    }
} else { Note "collision channels already present" }

# --- segmentation render pass --------------------------------------------
# The engine fork declares r.CARLA.EnableSegmentationRendering with a default
# of 0 and gates the segmentation pass on it. At 0 the pass never draws, but
# the buffer is still bound, so the segmentation camera sampled stale memory
# and returned labels outside the valid 0..29 range. Must be [SystemSettings]:
# it is a plain console variable, so [/Script/Engine.RendererSettings] drops it.
if (($lines -join "`n") -notmatch 'r\.CARLA\.EnableSegmentationRendering') {
    $sysIdx = ($lines | Select-String -Pattern '^\[SystemSettings\]' | Select-Object -First 1).LineNumber
    if ($sysIdx) {
        $lines = $lines[0..($sysIdx - 1)] + @('r.CARLA.EnableSegmentationRendering=1') + $lines[$sysIdx..($lines.Count - 1)]
    } else {
        $lines += @('', '[SystemSettings]', 'r.CARLA.EnableSegmentationRendering=1')
    }
    Ok "r.CARLA.EnableSegmentationRendering=1"
} else { Note "segmentation CVar already present" }

Set-Content $engineIni -Value $lines -Encoding UTF8

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
# Prefer SmallCity-signals.xodr when it exists. add_signals_to_xodr.py writes
# the traffic lights into that file and leaves the raw export alone, so
# installing the raw one silently turns every traffic light back off - the
# lights are only in the .xodr, nowhere else.
$xodrSrc = $null
foreach ($cand in @("..\SmallCity-signals.xodr", "..\SmallCity.xodr")) {
    $p = Join-Path $PSScriptRoot $cand
    if (Test-Path $p) { $xodrSrc = $p; break }
}
if ($xodrSrc) {
    $savedDir = Join-Path $CitySample "Saved\OpenDrive"
    if (-not (Test-Path $savedDir)) { New-Item -ItemType Directory -Path $savedDir -Force | Out-Null }
    Copy-Item (Resolve-Path $xodrSrc).Path (Join-Path $savedDir "Small_City_LVL.xodr") -Force
    $nSignals = ([regex]::Matches((Get-Content $xodrSrc -Raw), '<signal ')).Count
    Ok "Saved\OpenDrive\Small_City_LVL.xodr  ($(Split-Path $xodrSrc -Leaf), $nSignals signals)"
    if ($nSignals -eq 0) {
        Note "no traffic lights in that file - see README part B step 8 to add them"
    }
} else {
    Note "SmallCity.xodr not found - run the export pipeline first (see README part A)"
}

Say "`n=== 9/10 installing the pedestrian navmesh ===" "Cyan"
#
# NOT Saved\Nav\. The client asks for the navmesh via get_required_files("Nav"),
# and that RPC only searches Saved/ for generated OpenDRIVE worlds; for a map
# that already exists it walks the map's CONTENT folder and nothing else. A .bin
# in Saved\Nav\ is ignored without a word in any log, on either side.
$navSrc = Join-Path $PSScriptRoot "..\SmallCity.bin"
if (Test-Path $navSrc) {
    $navDir = Join-Path $CitySample "Content\Map\Nav"
    if (-not (Test-Path $navDir)) { New-Item -ItemType Directory -Path $navDir -Force | Out-Null }
    Copy-Item (Resolve-Path $navSrc).Path (Join-Path $navDir "Small_City_LVL.bin") -Force
    $navBytes = (Get-Item $navSrc).Length
    Ok "Content\Map\Nav\Small_City_LVL.bin  ($('{0:N0}' -f $navBytes) bytes)"
    # 40 bytes is a NavMeshSetHeader with zero tiles: RecastBuilder ran and
    # found no walkable surface. It loads without error and navigates nothing.
    if ($navBytes -le 64) {
        Note "that navmesh is empty (no tiles) - rebuild it, see README part B step 11"
    }
} else {
    Note "SmallCity.bin not found - walkers will not move; see README part B step 11"
}

Say "`n=== done ===" "Cyan"
Write-Host @"

Next:
  1. build    powershell -File tools\Build-CarlaCity.ps1
  2. run      powershell -File tools\Run-CarlaCity.ps1
  3. connect  python tools\carla_probe.py

Revert at any time with -Undo.
"@ -ForegroundColor White

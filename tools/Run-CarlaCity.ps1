<#
.SYNOPSIS
    Run the City Sample map as a CARLA server.

.DESCRIPTION
    Launches the CARLA fork's editor binary straight into -game mode on a City
    Sample map. Because Integrate-CarlaIntoCitySample.ps1 set the project's
    GameInstanceClass to UCarlaGameInstance, the CARLA RPC server comes up on
    port 2000 by itself; no separate CarlaUnreal.exe is involved.

    Waits until the port actually accepts a connection before returning, so a
    caller can chain a client script immediately.

.PARAMETER Map
    Which City Sample level. Small_City_LVL loads far faster than Big_City_LVL
    and is the one the road-network export was validated against.

.PARAMETER RenderOffScreen
    No window. Useful over RDP or for headless validation runs.

.EXAMPLE
    powershell -File tools\Run-CarlaCity.ps1
    powershell -File tools\Run-CarlaCity.ps1 -Map Big_City_LVL -Quality Epic
#>
[CmdletBinding()]
param(
    [string] $CitySample = $(if ($env:CITYSAMPLE_DIR) { $env:CITYSAMPLE_DIR }
                       else { Join-Path $env:USERPROFILE "Documents\Unreal Projects\CitySample" }),
    [string] $EnginePath = $(if ($env:CARLA_UE_DIR) { $env:CARLA_UE_DIR }
                       else { "C:\carla-ue58\UnrealEngine5_carla" }),
    # Only used to warn when the project's copy of the CARLA plugin is stale.
    [string] $CarlaRepo  = $(if ($env:CARLA_DIR) { $env:CARLA_DIR }
                      else { "C:\carla-ue58\carla" }),
    [ValidateSet("Small_City_LVL", "Big_City_LVL", "City_Open_World_Template")]
    [string] $Map = "Small_City_LVL",
    [ValidateSet("Development", "DebugGame")]
    [string] $Configuration = "Development",
    [ValidateSet("Low", "Medium", "High", "Epic")]
    [string] $Quality = "Epic",
    [int]    $Port = 2000,
    [int]    $ResX = 1600,
    [int]    $ResY = 900,
    # The FIRST launch compiles City Sample's Nanite meshes, distance fields
    # and 8K textures into a cold derived-data cache and takes 1-2 hours. A
    # 15-minute default made that look like a failure, threw, and left the
    # editor running - so the next run then hit 'port already in use'.
    [int]    $TimeoutSeconds = 9000,
    # Console commands to run at startup. This server has no interactive
    # console - it launches -game with stdout redirected to a file - so this is
    # the only way to set a CVar without editing DefaultEngine.ini.
    #   -ExecCmds "carla.MassBridge.Enable 1"
    [string[]] $ExecCmds = @(),
    [switch] $RenderOffScreen,
    [string] $LogDir = (Join-Path $PSScriptRoot "..\logs")
)

$ErrorActionPreference = "Stop"

$uproject = Join-Path $CitySample "CitySample.uproject"
$exe = if ($Configuration -eq "Development") {
    Join-Path $EnginePath "Engine\Binaries\Win64\UnrealEditor.exe"
} else {
    Join-Path $EnginePath "Engine\Binaries\Win64\UnrealEditor-Win64-DebugGame.exe"
}

foreach ($p in @($uproject, $exe)) {
    if (-not (Test-Path $p)) { throw "missing: $p  (build first: tools\Build-CarlaCity.ps1)" }
}
if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }

# Integrate-CarlaIntoCitySample.ps1 COPIES the CARLA plugins into the project;
# they are not junctions. So editing CARLA's own tree and rebuilding produces a
# successful build of the OLD code, with no warning and nothing in the log to
# say why the change had no effect. Costly to debug, trivial to detect.
$newest = { param($p) (Get-ChildItem $p -Recurse -File -Include *.cpp,*.h,*.cs -ErrorAction SilentlyContinue |
                       Measure-Object LastWriteTimeUtc -Maximum).Maximum }
# BOTH plugin sources, because both are copies and both have caught people out.
# CarlaMassBridge is the one edited most often, and a stale copy there fails in
# the most confusing way available: the build succeeds, the server starts, and
# the feature you just wrote simply is not there.
$pairs = @(
    @{ Name = "Carla";           Src = (Join-Path $CarlaRepo "Unreal\CarlaUnreal\Plugins\Carla\Source");
                                 Dst = (Join-Path $CitySample "Plugins\Carla\Source") },
    @{ Name = "CarlaMassBridge"; Src = (Join-Path $PSScriptRoot "..\plugins\CarlaMassBridge\Source");
                                 Dst = (Join-Path $CitySample "Plugins\CarlaMassBridge\Source") }
)
foreach ($pair in $pairs) {
    if (-not ((Test-Path $pair.Src) -and (Test-Path $pair.Dst))) { continue }
    $srcTime = & $newest $pair.Src
    $dstTime = & $newest $pair.Dst
    if ($srcTime -and $dstTime -and $srcTime -gt $dstTime.AddSeconds(2)) {
        Write-Warning ("$($pair.Name): the source in $($pair.Src) is newer than the " +
                       "project's copy ($($srcTime.ToLocalTime()) vs $($dstTime.ToLocalTime())).`n" +
                       "         Your edits will NOT be built. Re-run:  " +
                       "powershell -File tools\Integrate-CarlaIntoCitySample.ps1")
    }
}

# A server already on the port would make the readiness check below pass
# instantly against the wrong process.
$busy = Test-NetConnection -ComputerName 127.0.0.1 -Port $Port -InformationLevel Quiet -WarningAction SilentlyContinue
if ($busy) { throw "port $Port is already in use - stop the running CARLA server first" }

$log = Join-Path $LogDir "run-carlacity.log"

# The CARLA RPC server is started from ACarlaGameModeBase::InitGame, which
# calls GameInstance->NotifyInitGame() -> FCarlaEngine::NotifyInitGame ->
# Server.Start(2000, 2001, 2002). So the game mode - not just the game instance
# - has to be CARLA's, or nothing ever listens.
#
# GlobalDefaultGameMode in DefaultEngine.ini is not enough on its own: a level's
# own World Settings override it, and the City Sample levels set their own. The
# ?game= URL option beats both, and does it without editing Epic's .umap.
$gameMode = "/Game/Carla/Blueprints/Game/CarlaGameMode.CarlaGameMode_C"
$mapUrl   = "/Game/Map/$Map`?game=$gameMode"

$args = @(
    "`"$uproject`"",
    $mapUrl,
    "-game",
    "-carla-server",
    "-carla-rpc-port=$Port",
    "-quality-level=$Quality",
    "-nosound",
    # Live Coding starts a console and a named pipe during engine PreInit and
    # deadlocks when stdout is redirected: the process sits at 0% CPU forever
    # with the splash screen still up, having logged nothing past
    # "InternalLoadLibrary: 'LiveCoding'". It is a hot-reload convenience with
    # no use in a server run.
    "-NoLiveCoding",
    "-nosplash",
    # No modal dialogs. One waiting off-screen looks exactly like a hang.
    "-unattended",
    "-stdout", "-FullStdOutLogOutput"
)
if ($ExecCmds.Count -gt 0) {
    # UE splits -ExecCmds on COMMAS (ExecCmds.ParseIntoArray(..., TEXT(","))).
    #
    # Joining with "|" instead does not fail - it is worse than that. The engine
    # takes the whole string as ONE command, so
    #   "carla.FastGeoTagger.Enable 1|carla.MassBridge.Enable 1"
    # sets carla.FastGeoTagger.Enable from the argument "1|carla.MassBridge.Enable",
    # whose atoi is 1. The first CVar is therefore set correctly, the second is
    # silently never set at all, and nothing is logged either way. That cost an
    # afternoon: semantic segmentation worked and the Mass bridge did not,
    # in a run that asked for both.
    $joined = ($ExecCmds -join ",")
    $args += "-ExecCmds=`"$joined`""
}
if ($RenderOffScreen) { $args += "-RenderOffScreen" }
else { $args += @("-windowed", "-ResX=$ResX", "-ResY=$ResY") }

Write-Host "`n=== launching CARLA on the City Sample ===" -ForegroundColor Cyan
Write-Host "    map     : /Game/Map/$Map"
Write-Host "    gamemode: $gameMode"
Write-Host "    binary  : $(Split-Path $exe -Leaf)"
Write-Host "    port    : $Port"
if ($ExecCmds.Count -gt 0) { Write-Host "    execcmds: $($ExecCmds -join '  ,  ')" }
Write-Host "    log     : $log"

$proc = Start-Process -FilePath $exe -ArgumentList $args `
    -RedirectStandardOutput $log -RedirectStandardError "$log.err" `
    -NoNewWindow -PassThru

Write-Host "`nwaiting for the server (pid $($proc.Id)); the first load builds shaders and can take many minutes..." -ForegroundColor DarkGray

$sw = [Diagnostics.Stopwatch]::StartNew()
$ready = $false
while ($sw.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
    if ($proc.HasExited) {
        Write-Host "`nthe editor exited early (code $($proc.ExitCode))" -ForegroundColor Red
        if (Test-Path $log) {
            (Get-Content $log -Tail 25) | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkYellow }
        }
        throw "server process died - see $log"
    }
    $t = New-Object System.Net.Sockets.TcpClient
    try {
        $t.Connect("127.0.0.1", $Port)
        if ($t.Connected) { $ready = $true }
    } catch { } finally { $t.Close() }
    if ($ready) { break }

    Start-Sleep -Seconds 5
    if ([int]$sw.Elapsed.TotalSeconds % 60 -lt 5) {
        Write-Host ("    {0:N0}s ..." -f $sw.Elapsed.TotalSeconds) -ForegroundColor DarkGray
    }
}

if (-not $ready) {
    Write-Host "`nno server on port $Port after $TimeoutSeconds s" -ForegroundColor Red
    if (Test-Path $log) { (Get-Content $log -Tail 25) | ForEach-Object { Write-Host "    $_" -ForegroundColor DarkYellow } }
    throw "timed out waiting for the CARLA server"
}

Write-Host ("`nserver up on port {0} after {1:N0}s (pid {2})" -f $Port, $sw.Elapsed.TotalSeconds, $proc.Id) -ForegroundColor Green
Write-Host "connect with:  python tools\carla_probe.py" -ForegroundColor White
Write-Host "stop with   :  Stop-Process -Id $($proc.Id)" -ForegroundColor White

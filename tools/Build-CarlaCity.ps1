<#
.SYNOPSIS
    Build the City Sample editor with CARLA merged in, against the CARLA engine fork.

.DESCRIPTION
    Must be built with the CARLA fork of UE 5.8, not the launcher build: the
    Carla plugin is compiled against the fork's modified engine headers.
    Integrate-CarlaIntoCitySample.ps1 points the .uproject at the fork; this
    script drives that fork's Build.bat directly so the right UBT runs even if
    the file association is wrong.

    Development is tried first. If MSVC throws internal compiler errors
    (C1001), it retries in DebugGame, which compiles project and plugin code
    unoptimised and sidesteps the optimiser backend that trips on some of the
    City Sample plugins.

.PARAMETER Configuration
    Force a configuration instead of trying Development then DebugGame.

.PARAMETER Clean
    Delete Intermediate/Binaries for the project first.

.EXAMPLE
    powershell -File tools\Build-CarlaCity.ps1
    powershell -File tools\Build-CarlaCity.ps1 -Configuration DebugGame
#>
[CmdletBinding()]
param(
    [string] $CitySample = $(if ($env:CITYSAMPLE_DIR) { $env:CITYSAMPLE_DIR }
                       else { Join-Path $env:USERPROFILE "Documents\Unreal Projects\CitySample" }),
    [string] $EnginePath = $(if ($env:CARLA_UE_DIR) { $env:CARLA_UE_DIR }
                       else { "C:\carla-ue58\UnrealEngine5_carla" }),
    [ValidateSet("Development", "DebugGame", "")]
    [string] $Configuration = "",
    [string] $LogDir = (Join-Path $PSScriptRoot "..\logs"),
    [switch] $Clean
)

$ErrorActionPreference = "Stop"

$uproject = Join-Path $CitySample "CitySample.uproject"
$buildBat = Join-Path $EnginePath "Engine\Build\BatchFiles\Build.bat"

foreach ($p in @($uproject, $buildBat)) {
    if (-not (Test-Path $p)) { throw "missing: $p" }
}
if (-not (Test-Path (Join-Path $CitySample "Plugins\Carla"))) {
    throw "CARLA is not integrated yet - run tools\Integrate-CarlaIntoCitySample.ps1 first"
}
if (-not (Test-Path $LogDir)) { New-Item -ItemType Directory -Path $LogDir -Force | Out-Null }

if ($Clean) {
    Write-Host "cleaning..." -ForegroundColor Cyan
    foreach ($d in @("Intermediate", "Binaries")) {
        $t = Join-Path $CitySample $d
        if (Test-Path $t) { [System.IO.Directory]::Delete($t, $true); Write-Host "  removed $d" -ForegroundColor DarkGray }
    }
}

$configs = if ($Configuration) { @($Configuration) } else { @("Development", "DebugGame") }

foreach ($cfg in $configs) {
    $log = Join-Path $LogDir "build-carlacity-$($cfg.ToLower()).log"
    Write-Host "`n=== building CitySampleEditor | Win64 | $cfg ===" -ForegroundColor Cyan
    Write-Host "    engine : $EnginePath"
    Write-Host "    log    : $log"

    $sw = [Diagnostics.Stopwatch]::StartNew()
    $p = Start-Process -FilePath $buildBat `
        -ArgumentList @("CitySampleEditor", "Win64", $cfg, "-Project=`"$uproject`"", "-WaitMutex", "-FromMsBuild") `
        -WorkingDirectory $EnginePath `
        -RedirectStandardOutput $log -RedirectStandardError "$log.err" `
        -NoNewWindow -Wait -PassThru
    $sw.Stop()

    $text = if (Test-Path $log) { Get-Content $log -Raw } else { "" }
    $ice  = ([regex]::Matches($text, "error C1001")).Count
    $errs = ([regex]::Matches($text, "error [A-Z]+\d+")).Count

    Write-Host ("    exit {0}, {1:N1} min, {2} errors ({3} ICE)" -f $p.ExitCode, $sw.Elapsed.TotalMinutes, $errs, $ice) `
        -ForegroundColor $(if ($p.ExitCode -eq 0) { "Green" } else { "Yellow" })

    if ($p.ExitCode -eq 0) {
        Write-Host "`nbuild succeeded ($cfg)" -ForegroundColor Green
        Write-Host "run it with:  powershell -File tools\Run-CarlaCity.ps1 -Configuration $cfg" -ForegroundColor White
        exit 0
    }

    # Show what actually failed rather than making the caller open the log.
    Write-Host "`n--- first errors ---" -ForegroundColor Yellow
    ($text -split "`r?`n" | Where-Object { $_ -match "error [A-Z]+\d+|Error:|FAILED" } | Select-Object -First 12) |
        ForEach-Object { Write-Host "    $_" -ForegroundColor DarkYellow }

    if ($ice -gt 0 -and $cfg -ne $configs[-1]) {
        Write-Host "`n$ice internal compiler errors - retrying in DebugGame" -ForegroundColor Yellow
        continue
    }
    # An ordinary compile error is terminal. Falling through to DebugGame here
    # would rebuild the same broken source in another configuration, take just
    # as long, fail the same way, and report the SECOND configuration's log -
    # while the script's own documentation says the retry is for ICEs only.
    if ($ice -eq 0) { throw "build failed in $cfg with $errs error(s), no ICE - see $log" }
    if ($cfg -eq $configs[-1]) { throw "build failed in $cfg - see $log" }
}

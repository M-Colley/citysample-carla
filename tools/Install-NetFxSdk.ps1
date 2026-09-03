<#
.SYNOPSIS
  Install the .NET Framework 4.8 SDK + targeting pack into Visual Studio 2022.

.DESCRIPTION
  Unreal Build Tool needs NETFXSDK to build any UnrealEd-dependent module.
  Without it UBT fails before compiling anything:

      Unable to instantiate module 'SwarmInterface': Could not find NetFxSDK
      install dir ...  Result: Failed (RulesError)

  CARLA's own Util\SetupUtils\InstallPrerequisites.bat installs the same
  components, so this is needed for the CARLA build too.

  This script SELF-ELEVATES: just run it normally and accept the UAC prompt.

      pwsh -File tools\Install-NetFxSdk.ps1
      powershell -ExecutionPolicy Bypass -File tools\Install-NetFxSdk.ps1

  (Replaces install-netfx-sdk.bat, which was defeated by cmd expanding the
  "(x86)" in a path inside an if-block and closing the block on that paren.)
#>
[CmdletBinding()]
param(
    [switch]$Elevated,
    # Start each modify and return immediately, instead of blocking until it
    # finishes. The installs then run on in the background.
    [switch]$NoWait
)

$ErrorActionPreference = 'Stop'

function Test-Admin {
    $id = [Security.Principal.WindowsIdentity]::GetCurrent()
    (New-Object Security.Principal.WindowsPrincipal $id).IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Test-NetFxSdk {
    foreach ($k in @(
        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\NETFXSDK',
        'HKLM:\SOFTWARE\Microsoft\Microsoft SDKs\NETFXSDK')) {
        if (Test-Path $k) {
            $vers = Get-ChildItem $k -ErrorAction SilentlyContinue
            if ($vers) { return $vers }
        }
    }
    return $null
}

# --- already done? --------------------------------------------------------
$have = Test-NetFxSdk
if ($have) {
    Write-Host "NETFXSDK is already installed:" -ForegroundColor Green
    $have | ForEach-Object {
        $f = (Get-ItemProperty $_.PSPath -ErrorAction SilentlyContinue).KitsInstallationFolder
        Write-Host "  $($_.PSChildName)  ->  $f"
    }
    Write-Host "Nothing to do."
    return
}

# --- elevate if needed ----------------------------------------------------
if (-not (Test-Admin)) {
    if ($Elevated) {
        Write-Host "Elevation was requested but this process is still not admin. Aborting." -ForegroundColor Red
        return
    }
    Write-Host "Not elevated - relaunching with UAC..." -ForegroundColor Yellow
    $psi = @{
        FilePath     = (Get-Process -Id $PID).Path
        Verb         = 'RunAs'
        ArgumentList = @('-NoProfile', '-ExecutionPolicy', 'Bypass',
                         '-File', "`"$PSCommandPath`"", '-Elevated')
    }
    try {
        Start-Process @psi -Wait
    } catch {
        Write-Host "UAC was declined or elevation failed: $($_.Exception.Message)" -ForegroundColor Red
        return
    }
    # The elevated child does the work; re-check from here.
    $now = Test-NetFxSdk
    if ($now) {
        Write-Host "`nNETFXSDK is now installed." -ForegroundColor Green
    } else {
        Write-Host "`nStill not installed - see the elevated window's output." -ForegroundColor Yellow
    }
    return
}

# --- elevated from here ---------------------------------------------------
Write-Host ""
Write-Host ".NET Framework 4.8 SDK installer for Visual Studio 2022" -ForegroundColor Cyan
Write-Host "------------------------------------------------------"

$setup   = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\setup.exe'
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'

foreach ($p in @($setup, $vswhere)) {
    if (-not (Test-Path $p)) { throw "Not found: $p" }
}

$instances = & $vswhere -products * -property installationPath
if (-not $instances) { throw "vswhere reported no Visual Studio installations." }

Write-Host "Found $($instances.Count) VS2022 instance(s):"
$instances | ForEach-Object { Write-Host "  $_" }
Write-Host ""

$components = @(
    'Microsoft.Net.Component.4.8.SDK',
    'Microsoft.Net.Component.4.8.TargetingPack'
)

foreach ($inst in $instances) {
    Write-Host "Modifying $inst" -ForegroundColor Cyan
    $args = @('modify', '--installPath', $inst)
    foreach ($c in $components) { $args += @('--add', $c) }
    $args += @('--passive', '--norestart')

    Write-Host "  setup.exe $($args -join ' ')" -ForegroundColor DarkGray
    if ($NoWait) {
        Start-Process -FilePath $setup -ArgumentList $args
        Write-Host "  launched (not waiting)"
    } else {
        $p = Start-Process -FilePath $setup -ArgumentList $args -Wait -PassThru
        Write-Host "  exit code: $($p.ExitCode)"
        # 3010 = success, reboot required. 87 = bad arguments.
        if ($p.ExitCode -eq 87) {
            Write-Host "  Exit 87 means the installer rejected the arguments." -ForegroundColor Red
        }
    }
}

Write-Host ""
$after = Test-NetFxSdk
if ($after) {
    Write-Host "OK - NETFXSDK is registered:" -ForegroundColor Green
    $after | ForEach-Object {
        $f = (Get-ItemProperty $_.PSPath -ErrorAction SilentlyContinue).KitsInstallationFolder
        Write-Host "  $($_.PSChildName)  ->  $f"
    }
} else {
    Write-Host "NETFXSDK still not registered." -ForegroundColor Yellow
    Write-Host "If the VS Installer is still working, wait for it and re-run this script."
    Write-Host "Otherwise use the GUI: Visual Studio Installer > Build Tools 2022 > Modify"
    Write-Host "> Individual components > search '4.8' > tick '.NET Framework 4.8 SDK'"
    Write-Host "and '.NET Framework 4.8 targeting pack' > Modify."
}

if ($Elevated) {
    Write-Host ""
    Read-Host "Press Enter to close"
}

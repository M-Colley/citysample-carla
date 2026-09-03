@echo off
REM ---------------------------------------------------------------------------
REM Installs the .NET Framework 4.8 SDK + targeting pack into Visual Studio 2022.
REM
REM Unreal Build Tool needs NETFXSDK to build any UnrealEd-dependent module.
REM Without it UBT fails before compiling anything:
REM     Unable to instantiate module 'SwarmInterface': Could not find NetFxSDK
REM     install dir ... Result: Failed (RulesError)
REM
REM CARLA's own Util\SetupUtils\InstallPrerequisites.bat installs the same
REM components, so this is needed for the CARLA build too.
REM
REM RIGHT-CLICK THIS FILE -> "Run as administrator".
REM ---------------------------------------------------------------------------
setlocal EnableDelayedExpansion

echo.
echo  .NET Framework 4.8 SDK installer for Visual Studio 2022
echo  ------------------------------------------------------

REM --- must be elevated ------------------------------------------------------
net session >nul 2>&1
if errorlevel 1 (
    echo.
    echo  ERROR: not running as administrator.
    echo  Right-click this file and choose "Run as administrator".
    echo.
    pause
    exit /b 1
)

REM --- locate the VS installer ----------------------------------------------
set "SETUP=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\setup.exe"
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"

if not exist "%SETUP%" (
    echo  ERROR: VS Installer not found at:
    echo    %SETUP%
    pause
    exit /b 1
)

REM --- find every VS2022 instance and patch each one -------------------------
REM NETFXSDK installs machine-wide, but patching the instance UBT actually
REM resolves to is what matters, and UBT here resolves to BuildTools.
set "FOUND="
for /f "usebackq delims=" %%I in (`"%VSWHERE%" -products * -property installationPath`) do (
    set "FOUND=1"
    echo.
    echo  Modifying: %%I
    echo  This opens the VS Installer UI and may take several minutes.
    echo.
    "%SETUP%" modify --installPath "%%I" --add Microsoft.Net.Component.4.8.SDK --add Microsoft.Net.Component.4.8.TargetingPack --passive --norestart
    echo  Exit code: !errorlevel!
)

if not defined FOUND (
    echo  ERROR: vswhere found no Visual Studio installations.
    pause
    exit /b 1
)

REM --- verify ----------------------------------------------------------------
echo.
echo  Verifying NETFXSDK...
reg query "HKLM\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\NETFXSDK" >nul 2>&1
if errorlevel 1 (
    echo  NOT FOUND yet. If the installer is still running, wait for it to
    echo  finish and re-run this script to re-check.
) else (
    echo  OK - NETFXSDK is registered:
    reg query "HKLM\SOFTWARE\WOW6432Node\Microsoft\Microsoft SDKs\NETFXSDK"
)

echo.
pause
endlocal

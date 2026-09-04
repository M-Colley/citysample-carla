@echo off
setlocal EnableDelayedExpansion

rem ---------------------------------------------------------------------------
rem CarlaSetup.bat replacement for machines with VS 2022 BUILD TOOLS.
rem
rem Two things the stock script gets wrong on this machine:
rem
rem   1. It probes only Community / Professional / Enterprise for vcvars64.bat
rem      and calls "exit 1" otherwise, so it aborts on a Build Tools install
rem      even though Build Tools has a perfectly good C++ toolset - and before
rem      that it tries to DOWNLOAD Visual Studio. (carla issue 1 in
rem      ISSUES-TO-FILE.md)
rem
rem   2. It clones carla-content in full. That repo tracks 513,074 LFS files,
rem      91% of which are World Partition actor data for CARLA's own towns.
rem      Pulling all of it runs at ~67 GB/hr and fills the disk. We delegate to
rem      tools/fetch-carla-content.sh, which sparse-checks-out the 44,407 files
rem      that matter.
rem
rem The Unreal Engine step is skipped entirely: CARLA_UNREAL_ENGINE_PATH already
rem points at a built engine.
rem
rem Usage:
rem   SetupWithBuildTools.bat                 content, then configure and build
rem   SetupWithBuildTools.bat --skip-content  build only (content handled apart)
rem ---------------------------------------------------------------------------

rem CARLA_DIR overrides the default, matching the PowerShell tools and the README.
if not "%CARLA_DIR%"=="" (set "CARLA_ROOT=%CARLA_DIR%") else (set "CARLA_ROOT=C:\carla-ue58\carla")
set "CONTENT_DIR=%CARLA_ROOT%\Unreal\CarlaUnreal\Content"
set "VCVARS=%PROGRAMFILES(X86)%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
rem Beside this script, wherever it was copied to (%~dp0 ends with a backslash).
rem This was an absolute path to one machine, so every other copy went looking
rem for the fetch script somewhere that did not exist.
set "FETCH=%~dp0tools\fetch-carla-content.sh"

set "SKIP_CONTENT="
if /I "%~1"=="--skip-content" set "SKIP_CONTENT=1"

if "%CARLA_UNREAL_ENGINE_PATH%"=="" (
    echo ERROR: CARLA_UNREAL_ENGINE_PATH is not set.
    exit /b 1
)
if not exist "%CARLA_UNREAL_ENGINE_PATH%\Engine\Binaries\Win64\UnrealEditor.exe" (
    echo ERROR: no UnrealEditor.exe under "%CARLA_UNREAL_ENGINE_PATH%".
    exit /b 1
)
if not exist "%VCVARS%" (
    echo ERROR: Build Tools vcvars64.bat not found at "%VCVARS%".
    exit /b 1
)

cd /d "%CARLA_ROOT%"
if errorlevel 1 exit /b 1

echo === 1/4 content ===
if defined SKIP_CONTENT (
    echo Skipping content on request.
) else (
    if exist "%CONTENT_DIR%\Carla\Config" (
        echo Content already present, skipping.
    ) else (
        echo Fetching carla-content sparsely...
        bash "%FETCH%"
        if errorlevel 1 exit /b 1
    )
)

echo === 2/4 activating Build Tools x64 environment ===
call "%VCVARS%"
if errorlevel 1 exit /b 1

echo === 3/4 cmake configure ===
cmake -G Ninja -S . -B Build --toolchain=CMake/Toolchain.cmake -DCMAKE_BUILD_TYPE=Release -DCARLA_UNREAL_ENGINE_PATH="%CARLA_UNREAL_ENGINE_PATH%"
if errorlevel 1 exit /b 1

echo === 4/4 cmake build ===
cmake --build Build
if errorlevel 1 exit /b 1

echo === python api ===
cmake --build Build --target carla-python-api-install
if errorlevel 1 exit /b 1

echo.
echo CARLA build succeeded.
endlocal

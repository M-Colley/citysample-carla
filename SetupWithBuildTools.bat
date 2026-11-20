@echo off
setlocal DisableDelayedExpansion

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
rem   2. It clones carla-content in full. The sparse fetch excludes World
rem      Partition actors for CARLA's towns and checks the 1.0 content tag.
rem
rem The Unreal Engine step is skipped entirely: CARLA_UNREAL_ENGINE_PATH already
rem points at a built engine.
rem
rem Usage:
rem   SetupWithBuildTools.bat                 content, then configure and build
rem   SetupWithBuildTools.bat --skip-content  build with existing content
rem CARLA 1.0 writes its Python wheel to Build\PythonAPI\dist.
rem Legacy builds install into the active Python environment.
rem ---------------------------------------------------------------------------

rem CARLA_DIR overrides the default, matching the PowerShell tools and the README.
if not "%CARLA_DIR%"=="" (set "CARLA_ROOT=%CARLA_DIR%") else (set "CARLA_ROOT=C:\carla-ue58\carla")
for %%D in ("%CARLA_ROOT%") do set "CARLA_ROOT=%%~fD"
set "CONTENT_DIR=%CARLA_ROOT%\Unreal\CarlaUnreal\Content"
set "VCVARS=%PROGRAMFILES(X86)%\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
rem Beside this script, wherever it was copied to (%~dp0 ends with a backslash).
rem This was an absolute path to one machine, so every other copy went looking
rem for the fetch script somewhere that did not exist.
set "FETCH=%~dp0tools\fetch-carla-content.sh"

set "SKIP_CONTENT="
if /I "%~1"=="--skip-content" set "SKIP_CONTENT=1"

if not "%CARLA_UE_DIR%"=="" set "CARLA_UNREAL_ENGINE_PATH=%CARLA_UE_DIR%"
if "%CARLA_UNREAL_ENGINE_PATH%"=="" (
    echo ERROR: Set CARLA_UNREAL_ENGINE_PATH or CARLA_UE_DIR to the built engine.
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

set "SOURCE_REV="
for /f %%G in ('git rev-parse --verify HEAD') do set "SOURCE_REV=%%G"
if not defined SOURCE_REV (
    echo ERROR: Could not read the CARLA commit at "%CARLA_ROOT%".
    exit /b 1
)
set "DLSS_OPTION="
set "PYTHON_TARGET=carla-python-api-install"
if /I "%SOURCE_REV%"=="3ea166118d2c3c094b13b15c5b7a66a9a279998d" (
    call :check_release_engine
    if errorlevel 1 exit /b 1
    set "PYTHON_TARGET=carla-python-api"
)

echo === 1/4 content ===
if "%PYTHON_TARGET%"=="carla-python-api-install" (
    if defined SKIP_CONTENT (
        echo Skipping content on request.
        goto build
    )
    if exist "%CONTENT_DIR%\Carla\Config" (
        echo Legacy content already present.
        goto build
    )
)

rem The Windows bash.exe alias can launch WSL instead of Git Bash.
set "GIT_BASH="
for /f "delims=" %%G in ('where git.exe 2^>nul') do if not defined GIT_BASH if exist "%%~dpG..\bin\bash.exe" set "GIT_BASH=%%~dpG..\bin\bash.exe"
if not defined GIT_BASH (
    echo ERROR: Git Bash not found. Install Git for Windows and put git.exe on PATH.
    exit /b 1
)
set "CONTENT_ACTION="
if defined SKIP_CONTENT set "CONTENT_ACTION=--check"
"%GIT_BASH%" "%FETCH%" %CONTENT_ACTION%
if errorlevel 1 exit /b 1

:build
echo === 2/4 activating Build Tools x64 environment ===
call "%VCVARS%"
if errorlevel 1 exit /b 1

echo === 3/4 cmake configure ===
cmake -G Ninja -S . -B Build --toolchain=CMake/Toolchain.cmake -DCMAKE_BUILD_TYPE=Release -DCARLA_UNREAL_ENGINE_PATH="%CARLA_UNREAL_ENGINE_PATH%" %DLSS_OPTION%
if errorlevel 1 exit /b 1

echo === 4/4 cmake build ===
cmake --build Build
if errorlevel 1 exit /b 1
cmake --build Build --target carla-unreal-configure
if errorlevel 1 exit /b 1
cmake --build Build --target RecastBuilder
if errorlevel 1 exit /b 1

echo === python api ===
cmake --build Build --target %PYTHON_TARGET%
if errorlevel 1 exit /b 1

echo.
echo CARLA build succeeded.
if "%PYTHON_TARGET%"=="carla-python-api" echo Python wheel: "%CARLA_ROOT%\Build\PythonAPI\dist"
endlocal
exit /b 0

:check_release_engine
set "ENGINE_REV="
for /f %%G in ('git -C "%CARLA_UNREAL_ENGINE_PATH%" rev-parse --verify HEAD') do set "ENGINE_REV=%%G"
if /I not "%ENGINE_REV%"=="b76fce0e3bcbb400efe1fe38bc2d662b33e6d989" (
    echo ERROR: CARLA 1.0 needs engine b76fce0e3bcbb400efe1fe38bc2d662b33e6d989.
    echo Found "%ENGINE_REV%" at "%CARLA_UNREAL_ENGINE_PATH%".
    exit /b 1
)
if not defined DLSS_SDK set "DLSS_SDK=disabled"
set DLSS_OPTION="-DCARLA_DLSS_SDK_PATH=%DLSS_SDK%"
exit /b 0

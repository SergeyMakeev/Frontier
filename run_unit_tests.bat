@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
if defined FRONTIER_UNIT_BUILD_DIR (
    set "BUILD_DIR=%FRONTIER_UNIT_BUILD_DIR%"
) else (
    set "BUILD_DIR=%ROOT%\build-unit-debug"
)

where cmake >nul 2>nul
if errorlevel 1 (
    echo ERROR: CMake was not found in PATH.
    exit /b 1
)

where cl >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if exist "!VSWHERE!" (
        for /f "usebackq delims=" %%I in (`"!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"
    )
    if defined VS_INSTALL call "!VS_INSTALL!\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
)
where cl >nul 2>nul
if errorlevel 1 (
    echo ERROR: MSVC cl.exe was not found. Install the Visual Studio C++ workload.
    exit /b 1
)

set "GENERATOR_ARGS="
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    where ninja >nul 2>nul
    if not errorlevel 1 set "GENERATOR_ARGS=-G Ninja"
)

echo Configuring Debug unit tests ^(BVH4 + BVH8^)...
cmake -S "%ROOT%" -B "%BUILD_DIR%" !GENERATOR_ARGS! ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DFRONTIER_BUILD_TESTS=ON ^
    -DFRONTIER_BUILD_BENCH=OFF ^
    -DFRONTIER_BVH_WIDTH=AUTO ^
    -DFRONTIER_SSE2_ONLY=ON ^
    -DFRONTIER_STATS=OFF ^
    -DFRONTIER_CONTRACT_CHECKS=ON ^
    -DFRONTIER_VALIDATE_SUBTREES=ON
if errorlevel 1 exit /b 1

cmake --build "%BUILD_DIR%" --config Debug --target frontier_unit_tests --parallel
if errorlevel 1 exit /b 1

ctest --test-dir "%BUILD_DIR%" -C Debug --output-on-failure --parallel
exit /b %errorlevel%

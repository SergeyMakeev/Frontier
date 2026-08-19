@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
if defined FRONTIER_CITY_BUILD_DIR (
    set "BUILD_DIR=%FRONTIER_CITY_BUILD_DIR%"
) else (
    set "BUILD_DIR=%ROOT%\build-city"
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

echo Configuring the Frontier bgfx city sample...
cmake -S "%ROOT%" -B "%BUILD_DIR%" !GENERATOR_ARGS! ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DFRONTIER_BUILD_CITY_SAMPLE=ON ^
    -DFRONTIER_BUILD_TESTS=OFF ^
    -DFRONTIER_BUILD_BENCH=OFF
if errorlevel 1 exit /b 1

echo Building frontier_city...
cmake --build "%BUILD_DIR%" --config Release --target frontier_city --parallel
if errorlevel 1 exit /b 1

set "SAMPLE_EXE=%BUILD_DIR%\examples\city\Release\frontier_city.exe"
if not exist "!SAMPLE_EXE!" set "SAMPLE_EXE=%BUILD_DIR%\examples\city\frontier_city.exe"
if not exist "!SAMPLE_EXE!" (
    echo ERROR: Built sample executable was not found under "%BUILD_DIR%\examples\city".
    exit /b 1
)

echo Launching Frontier Dynamic City...
"%SAMPLE_EXE%" %*
exit /b %errorlevel%

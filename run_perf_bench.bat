@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "BUILD_DIR=%ROOT%\build-perf"

where cmake >nul 2>nul
if errorlevel 1 (
    echo ERROR: CMake was not found in PATH.
    exit /b 1
)

rem A normal terminal does not put cl.exe on PATH. If the Visual Studio C++
rem workload is installed, enter its latest x64 developer environment first.
where cl >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if exist "!VSWHERE!" (
        for /f "usebackq delims=" %%I in (`"!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"
    )
    if defined VS_INSTALL (
        echo Initializing Visual Studio x64 build tools...
        call "!VS_INSTALL!\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
        if errorlevel 1 exit /b 1
    )
)

rem Prefer Ninja for a compact single-config build when it is available.
rem An existing build directory keeps the generator recorded in its cache.
set "GENERATOR_ARGS="
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    where ninja >nul 2>nul
    if not errorlevel 1 set "GENERATOR_ARGS=-G Ninja"
)

echo Configuring Release AVX2 performance build...
cmake -S "%ROOT%" -B "%BUILD_DIR%" !GENERATOR_ARGS! ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DHLOD_BUILD_TESTS=OFF ^
    -DHLOD_BUILD_BENCH=ON ^
    -DHLOD_AVX2=ON ^
    -DHLOD_FORCE_SCALAR=OFF
if errorlevel 1 exit /b 1

echo Building hlod_bench...
cmake --build "%BUILD_DIR%" --config Release --target hlod_bench --parallel
if errorlevel 1 exit /b 1

set "BENCH_EXE=%BUILD_DIR%\bench\Release\hlod_bench.exe"
if not exist "%BENCH_EXE%" set "BENCH_EXE=%BUILD_DIR%\bench\hlod_bench.exe"
if not exist "%BENCH_EXE%" (
    echo ERROR: Built benchmark executable was not found under "%BUILD_DIR%\bench".
    exit /b 1
)

if not "%~1"=="" goto custom_args

echo Running the documented performance suite with five repetitions...
echo Pass Google Benchmark arguments to this script to override the default suite.
"%BENCH_EXE%" ^
    --benchmark_filter="BM_(View_Breakdown|View_MultiView|FlatForest100k|MixedForest100k|RootDecisionForest100k)" ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=false
goto done

:custom_args
echo Running hlod_bench with caller-supplied arguments...
"%BENCH_EXE%" %*

:done
exit /b %errorlevel%

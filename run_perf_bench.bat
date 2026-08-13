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
    -DFRONTIER_IPO=ON ^
    -DFRONTIER_BUILD_TESTS=OFF ^
    -DFRONTIER_BUILD_BENCH=ON ^
    -DFRONTIER_AVX2=ON ^
    -DFRONTIER_BVH_WIDTH=AUTO ^
    -DFRONTIER_FORCE_SCALAR=OFF ^
    -DFRONTIER_STATS=OFF ^
    -DFRONTIER_CONTRACT_CHECKS=OFF ^
    -DFRONTIER_VALIDATE_SUBTREES=OFF
if errorlevel 1 exit /b 1

echo Building frontier_bench...
cmake --build "%BUILD_DIR%" --config Release --target frontier_bench --parallel
if errorlevel 1 exit /b 1

set "BENCH_EXE=%BUILD_DIR%\bench\Release\frontier_bench.exe"
if not exist "%BENCH_EXE%" set "BENCH_EXE=%BUILD_DIR%\bench\frontier_bench.exe"
if not exist "%BENCH_EXE%" (
    echo ERROR: Built benchmark executable was not found under "%BUILD_DIR%\bench".
    exit /b 1
)

if not "%~1"=="" goto custom_args

echo Running the documented performance suite with five repetitions...
echo Pass Google Benchmark arguments to this script to override the default suite.
echo Writing %ROOT%\real_world_perf.json
"%BENCH_EXE%" ^
    --benchmark_filter="BM_SubtreeAssembly" ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%ROOT%\real_world_perf.json" ^
    --benchmark_out_format=json
goto done

:custom_args
echo Running frontier_bench with caller-supplied arguments...
"%BENCH_EXE%" %*

:done
exit /b %errorlevel%

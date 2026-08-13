@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
set "BUILD_DIR=%ROOT%\build-perf"
if defined FRONTIER_PERF_PAYLOAD_BITS (
    set "PAYLOAD_MODE=%FRONTIER_PERF_PAYLOAD_BITS%"
) else (
    set "PAYLOAD_MODE=both"
)
if /I not "%PAYLOAD_MODE%"=="both" if not "%PAYLOAD_MODE%"=="32" if not "%PAYLOAD_MODE%"=="64" (
    echo ERROR: FRONTIER_PERF_PAYLOAD_BITS must be both, 32, or 64.
    exit /b 2
)

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

set "BUILD_TARGETS="
if /I not "%PAYLOAD_MODE%"=="32" set "BUILD_TARGETS=!BUILD_TARGETS! frontier_bench"
if /I not "%PAYLOAD_MODE%"=="64" set "BUILD_TARGETS=!BUILD_TARGETS! frontier_bench_payload32"
echo Building payload %PAYLOAD_MODE% performance benchmark^(s^)...
cmake --build "%BUILD_DIR%" --config Release --target !BUILD_TARGETS! --parallel
if errorlevel 1 exit /b 1

set "BENCH64_EXE=%BUILD_DIR%\bench\Release\frontier_bench.exe"
set "BENCH32_EXE=%BUILD_DIR%\bench\Release\frontier_bench_payload32.exe"
if not exist "%BENCH64_EXE%" set "BENCH64_EXE=%BUILD_DIR%\bench\frontier_bench.exe"
if not exist "%BENCH32_EXE%" set "BENCH32_EXE=%BUILD_DIR%\bench\frontier_bench_payload32.exe"
if /I not "%PAYLOAD_MODE%"=="32" if not exist "%BENCH64_EXE%" (
    echo ERROR: The 8-byte payload benchmark was not found under "%BUILD_DIR%\bench".
    exit /b 1
)
if /I not "%PAYLOAD_MODE%"=="64" if not exist "%BENCH32_EXE%" (
    echo ERROR: The 4-byte payload benchmark was not found under "%BUILD_DIR%\bench".
    exit /b 1
)

if not "%~1"=="" goto custom_args

echo Running payload %PAYLOAD_MODE% documented performance suite with five repetitions...
echo Pass Google Benchmark arguments to this script to override the default suite.
if /I not "%PAYLOAD_MODE%"=="32" (
    echo Writing %ROOT%\real_world_perf_payload64.json
    "%BENCH64_EXE%" ^
        --benchmark_filter="BM_SubtreeAssembly" ^
        --benchmark_repetitions=5 ^
        --benchmark_report_aggregates_only=true ^
        --benchmark_out="%ROOT%\real_world_perf_payload64.json" ^
        --benchmark_out_format=json
    if errorlevel 1 exit /b 1
)
if /I not "%PAYLOAD_MODE%"=="64" (
    echo Writing %ROOT%\real_world_perf_payload32.json
    "%BENCH32_EXE%" ^
        --benchmark_filter="BM_SubtreeAssembly" ^
        --benchmark_repetitions=5 ^
        --benchmark_report_aggregates_only=true ^
        --benchmark_out="%ROOT%\real_world_perf_payload32.json" ^
        --benchmark_out_format=json
    if errorlevel 1 exit /b 1
)
goto done

:custom_args
set "HAS_BENCHMARK_OUT=0"
for %%A in (%*) do (
    set "CURRENT_ARG=%%~A"
    if /I "!CURRENT_ARG!"=="--benchmark_out" set "HAS_BENCHMARK_OUT=1"
    if /I "!CURRENT_ARG:~0,16!"=="--benchmark_out=" set "HAS_BENCHMARK_OUT=1"
)
if /I "%PAYLOAD_MODE%"=="both" if "!HAS_BENCHMARK_OUT!"=="1" (
    echo ERROR: --benchmark_out would be overwritten in both-width mode.
    echo Set FRONTIER_PERF_PAYLOAD_BITS to 32 or 64, or omit --benchmark_out.
    exit /b 2
)
echo Running payload %PAYLOAD_MODE% benchmark^(s^) with caller-supplied arguments...
if /I not "%PAYLOAD_MODE%"=="32" (
    "%BENCH64_EXE%" %*
    if errorlevel 1 exit /b 1
)
if /I not "%PAYLOAD_MODE%"=="64" (
    "%BENCH32_EXE%" %*
    if errorlevel 1 exit /b 1
)

:done
exit /b %errorlevel%

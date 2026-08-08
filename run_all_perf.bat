@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "RNG_POLICY=xorshift32-explicit-seeds-v1"

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
if defined HLOD_ALL_PERF_BUILD_DIR (
    set "BUILD_DIR=%HLOD_ALL_PERF_BUILD_DIR%"
) else (
    set "BUILD_DIR=%ROOT%\build-perf-report"
)
if defined HLOD_PERF_REPORT_ROOT (
    set "REPORT_ROOT=%HLOD_PERF_REPORT_ROOT%"
) else (
    set "REPORT_ROOT=%ROOT%\perf_reports"
)

if not "%~2"=="" (
    echo Usage: %~nx0 [machine-label]
    exit /b 2
)

where cmake >nul 2>nul
if errorlevel 1 (
    echo ERROR: CMake was not found in PATH.
    exit /b 1
)

if defined HLOD_PERF_LABEL (
    set "RAW_LABEL=%HLOD_PERF_LABEL%"
) else if not "%~1"=="" (
    set "RAW_LABEL=%~1"
) else (
    set "RAW_LABEL=Windows-%PROCESSOR_ARCHITECTURE%"
)
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "$s=$env:RAW_LABEL -replace '[^A-Za-z0-9._-]','_'; $s.Trim('_')"`) do set "LABEL=%%I"
if not defined LABEL (
    echo ERROR: The machine label contains no usable characters.
    exit /b 2
)
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssZ')"`) do set "TIMESTAMP=%%I"
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "[DateTimeOffset]::UtcNow.ToUnixTimeSeconds()"`) do set "START_EPOCH=%%I"
set "GIT_ROOT=%ROOT:\=/%"

set "REPORT_NAME=hlod-perf-%LABEL%-%TIMESTAMP%"
set "REPORT_DIR=%REPORT_ROOT%\%REPORT_NAME%"
if not exist "%REPORT_DIR%" mkdir "%REPORT_DIR%"
if errorlevel 1 exit /b 1
set "RUN_STATUS=FAILED"
set "FAILURE_STAGE=setup"
set "EXIT_CODE=1"

rem A normal terminal does not put cl.exe on PATH. Enter the newest installed
rem Visual Studio x64 developer environment when necessary.
where cl >nul 2>nul
if errorlevel 1 (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if exist "!VSWHERE!" (
        for /f "usebackq delims=" %%I in (`"!VSWHERE!" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_INSTALL=%%I"
    )
    if defined VS_INSTALL (
        echo Initializing Visual Studio x64 build tools...
        call "!VS_INSTALL!\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 >nul
        if errorlevel 1 goto package
    )
)
where cl >nul 2>nul
if errorlevel 1 (
    echo ERROR: MSVC cl.exe was not found. Install the Visual Studio C++ workload.
    goto package
)

(
    echo HLOD cross-machine performance collection
    echo Label: %RAW_LABEL%
    echo Started UTC: %TIMESTAMP%
    echo Host: Windows %PROCESSOR_ARCHITECTURE%
    echo.
    systeminfo
) > "%REPORT_DIR%\hardware.txt" 2>&1
powershell -NoProfile -Command "$ErrorActionPreference='Continue'; ''; 'Processor'; Get-CimInstance Win32_Processor | Format-List Name,Manufacturer,NumberOfCores,NumberOfLogicalProcessors,MaxClockSpeed,L2CacheSize,L3CacheSize; 'Computer'; Get-CimInstance Win32_ComputerSystem | Format-List Manufacturer,Model,SystemType,TotalPhysicalMemory; 'Physical memory'; Get-CimInstance Win32_PhysicalMemory | Format-Table Manufacturer,PartNumber,Capacity,Speed,ConfiguredClockSpeed -AutoSize; 'Operating system'; Get-CimInstance Win32_OperatingSystem | Format-List Caption,Version,BuildNumber,OSArchitecture,LastBootUpTime" >> "%REPORT_DIR%\hardware.txt" 2>&1
echo.>> "%REPORT_DIR%\hardware.txt"
echo Active power scheme>> "%REPORT_DIR%\hardware.txt"
powercfg /getactivescheme >> "%REPORT_DIR%\hardware.txt" 2>&1

(
    for /f "delims=" %%I in ('git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" rev-parse HEAD 2^>nul') do echo Commit: %%I
    for /f "delims=" %%I in ('git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" branch --show-current 2^>nul') do echo Branch: %%I
    echo.
    echo Working tree:
    git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" status --short
    echo.
    git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" diff --stat
) > "%REPORT_DIR%\source.txt" 2>&1

(
    echo Correctness:
    echo   ctest --test-dir ^<build^> -C Release --output-on-failure
    echo.
    echo Real world:
    echo   hlod_bench
    echo     --benchmark_filter=BM_^(View_Breakdown^|View_MultiView^|FlatForest100k^|MixedForest100k^|RootDecisionForest100k^)
    echo     --benchmark_repetitions=5
    echo     --benchmark_report_aggregates_only=true
    echo.
    echo Machine characterization:
    echo   hlod_machine_bench --benchmark_min_time=0.15s --benchmark_repetitions=5
    echo     --benchmark_report_aggregates_only=true
    echo.
    echo Focused architecture kernels:
    echo   hlod_machine_bench
    echo     --benchmark_filter=BM_^(Kernel^(WideAabb^|DistanceError^|CacheHit^)^|OutputAppend^)
    echo     --benchmark_min_time=0.75s --benchmark_repetitions=11
    echo     --benchmark_report_aggregates_only=true
    echo.
    echo Random workloads:
    echo   %RNG_POLICY%
    echo   Float mapping and shuffle are repository-owned and standard-library independent.
) > "%REPORT_DIR%\commands.txt"

set "GENERATOR_ARGS="
if not exist "%BUILD_DIR%\CMakeCache.txt" (
    where ninja >nul 2>nul
    if not errorlevel 1 set "GENERATOR_ARGS=-G Ninja"
)

set "FAILURE_STAGE=configure"
echo Configuring Release AVX2 build...
cmake -S "%ROOT%" -B "%BUILD_DIR%" !GENERATOR_ARGS! ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DHLOD_BUILD_TESTS=ON ^
    -DHLOD_BUILD_BENCH=ON ^
    -DHLOD_AVX2=ON ^
    -DHLOD_FORCE_SCALAR=OFF > "%REPORT_DIR%\configure.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\configure.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=build"
echo Building tests and performance executables...
cmake --build "%BUILD_DIR%" --config Release --parallel ^
    --target hlod_tests hlod_bench hlod_machine_bench > "%REPORT_DIR%\build.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\build.log"
if not "!RUN_RC!"=="0" goto package

set "BENCH_EXE=%BUILD_DIR%\bench\Release\hlod_bench.exe"
set "MACHINE_EXE=%BUILD_DIR%\bench\Release\hlod_machine_bench.exe"
if not exist "%BENCH_EXE%" set "BENCH_EXE=%BUILD_DIR%\bench\hlod_bench.exe"
if not exist "%MACHINE_EXE%" set "MACHINE_EXE=%BUILD_DIR%\bench\hlod_machine_bench.exe"
if not exist "%BENCH_EXE%" (
    echo ERROR: hlod_bench.exe was not found under "%BUILD_DIR%\bench".
    goto package
)
if not exist "%MACHINE_EXE%" (
    echo ERROR: hlod_machine_bench.exe was not found under "%BUILD_DIR%\bench".
    goto package
)

(
    cmake --version
    echo.
    cl 2^>^&1
    echo.
    git --version
    echo.
    findstr /R /B "CMAKE_BUILD_TYPE: CMAKE_CXX_COMPILER: CMAKE_CXX_COMPILER_ID: CMAKE_CXX_COMPILER_VERSION: CMAKE_GENERATOR: CMAKE_OSX_ARCHITECTURES: HLOD_AVX2: HLOD_FORCE_SCALAR:" "%BUILD_DIR%\CMakeCache.txt"
) > "%REPORT_DIR%\toolchain.txt" 2>&1

set "FAILURE_STAGE=correctness-tests"
echo Running correctness tests...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure > "%REPORT_DIR%\tests.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\tests.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=real-world-benchmarks"
echo Running real-world performance suite...
"%BENCH_EXE%" ^
    --benchmark_filter="BM_(View_Breakdown|View_MultiView|FlatForest100k|MixedForest100k|RootDecisionForest100k)" ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%REPORT_DIR%\real_world_perf.json" ^
    --benchmark_out_format=json > "%REPORT_DIR%\real_world_perf.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\real_world_perf.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=machine-benchmarks"
echo Running machine characterization suite...
"%MACHINE_EXE%" ^
    --benchmark_min_time=0.15s ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%REPORT_DIR%\machine_perf.json" ^
    --benchmark_out_format=json > "%REPORT_DIR%\machine_perf.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\machine_perf.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=architecture-benchmarks"
echo Running focused production-kernel suite...
"%MACHINE_EXE%" ^
    --benchmark_filter="BM_(Kernel(WideAabb|DistanceError|CacheHit)|OutputAppend)" ^
    --benchmark_min_time=0.75s ^
    --benchmark_repetitions=11 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%REPORT_DIR%\arch_kernel_perf.json" ^
    --benchmark_out_format=json > "%REPORT_DIR%\arch_kernel_perf.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\arch_kernel_perf.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=validate-results"
powershell -NoProfile -Command "$names='real_world_perf.json','machine_perf.json','arch_kernel_perf.json'; foreach($name in $names) { $path=Join-Path '%REPORT_DIR%' $name; if(!(Test-Path -LiteralPath $path)) { Write-Error ($name + ' is missing'); exit 1 }; try { $json=Get-Content -Raw -LiteralPath $path | ConvertFrom-Json } catch { Write-Error ($name + ' is invalid JSON'); exit 1 }; if(@($json.benchmarks).Count -eq 0) { Write-Error ($name + ' contains no benchmark records'); exit 1 } }"
if errorlevel 1 goto package

set "RUN_STATUS=COMPLETE"
set "FAILURE_STAGE=none"
set "EXIT_CODE=0"

:package
for /f "usebackq delims=" %%I in (`powershell -NoProfile -Command "[DateTimeOffset]::UtcNow.ToUnixTimeSeconds() - %START_EPOCH%"`) do set "ELAPSED_SECONDS=%%I"
for /f "delims=" %%I in ('git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" rev-parse HEAD 2^>nul') do set "GIT_COMMIT=%%I"
if not defined GIT_COMMIT set "GIT_COMMIT=unknown"
set "GIT_DIRTY=no"
for /f "delims=" %%I in ('git -c "safe.directory=%GIT_ROOT%" -C "%ROOT%" status --porcelain 2^>nul') do set "GIT_DIRTY=yes"

(
    echo # HLOD performance report
    echo.
    echo - Format: hlod-perf-report-v1
    echo - Status: %RUN_STATUS%
    echo - Failed stage: %FAILURE_STAGE%
    echo - Machine label: %RAW_LABEL%
    echo - Captured UTC: %TIMESTAMP%
    echo - Elapsed seconds: %ELAPSED_SECONDS%
    echo - Source commit: %GIT_COMMIT%
    echo - Source dirty: %GIT_DIRTY%
    echo - Host OS: Windows
    echo - Host architecture: %PROCESSOR_ARCHITECTURE%
    echo - CPU affinity: scheduler default
    echo - Workload RNG: %RNG_POLICY%
    echo - Benchmark order: registration order
    echo - Build: Release
    echo.
    echo ## Result files
    echo.
    echo - real_world_perf.json: end-to-end, real-world-like workloads
    echo - machine_perf.json: ALU, SIMD, branch, cache, latency, and bandwidth probes
    echo - arch_kernel_perf.json: focused production-kernel probes with longer sampling
    echo - tests.log: correctness-suite result
    echo - hardware.txt: CPU, memory, topology, OS, and power information
    echo - toolchain.txt: compiler, CMake, and build configuration
    echo - source.txt: exact Git revision and working-tree state
    echo - commands.txt: benchmark filters and sampling parameters
    echo.
    echo The benchmarks ran without an affinity mask, matching normal scheduler behavior.
    echo Compare JSON median aggregates first and use coefficient of variation to identify unstable cases.
) > "%REPORT_DIR%\REPORT.md"

(
    echo format=hlod-perf-report-v1
    echo status=%RUN_STATUS%
    echo failure_stage=%FAILURE_STAGE%
    echo label=%RAW_LABEL%
    echo timestamp_utc=%TIMESTAMP%
    echo elapsed_seconds=%ELAPSED_SECONDS%
    echo git_commit=%GIT_COMMIT%
    echo git_dirty=%GIT_DIRTY%
    echo host_os=Windows
    echo host_arch=%PROCESSOR_ARCHITECTURE%
    echo build_type=Release
    echo affinity=scheduler-default
    echo rng_policy=%RNG_POLICY%
    echo benchmark_order=registration-order
) > "%REPORT_DIR%\manifest.txt"

set "ARCHIVE=%REPORT_DIR%.zip"
powershell -NoProfile -Command "Compress-Archive -LiteralPath '%REPORT_DIR%' -DestinationPath '%ARCHIVE%' -CompressionLevel Optimal -Force" >nul
if exist "%ARCHIVE%" (
    echo.
    echo Performance report: %ARCHIVE%
) else (
    echo.
    echo Performance report directory: %REPORT_DIR%
)
if not "%EXIT_CODE%"=="0" echo ERROR: Collection failed during '%FAILURE_STAGE%'; partial data was retained.
exit /b %EXIT_CODE%

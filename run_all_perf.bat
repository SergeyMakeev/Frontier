@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "RNG_POLICY=xorshift32-explicit-seeds-v1"

set "ROOT=%~dp0"
if "%ROOT:~-1%"=="\" set "ROOT=%ROOT:~0,-1%"
if defined FRONTIER_ALL_PERF_BUILD_DIR (
    set "BUILD_DIR=%FRONTIER_ALL_PERF_BUILD_DIR%"
) else (
    set "BUILD_DIR=%ROOT%\build-perf-report"
)
if defined FRONTIER_ALL_TEST_BUILD_DIR (
    set "TEST_BUILD_DIR=%FRONTIER_ALL_TEST_BUILD_DIR%"
) else (
    set "TEST_BUILD_DIR=%BUILD_DIR%-unit-debug"
)
if defined FRONTIER_PERF_REPORT_ROOT (
    set "REPORT_ROOT=%FRONTIER_PERF_REPORT_ROOT%"
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

if defined FRONTIER_PERF_LABEL (
    set "RAW_LABEL=%FRONTIER_PERF_LABEL%"
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

set "REPORT_NAME=frontier-perf-%LABEL%-%TIMESTAMP%"
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
    echo Frontier cross-machine performance collection
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
    echo   ctest --test-dir ^<unit-build^> -C Debug --output-on-failure
    echo.
    echo Inventory:
    echo   frontier_bench and frontier_bench_payload32 --benchmark_list_tests=true
    echo.
    echo Real world:
    echo   frontier_bench ^(8-byte payload^) and frontier_bench_payload32 ^(4-byte payload^)
    echo     complete registered benchmark suite ^(no benchmark filter^)
    echo     --benchmark_min_time=0.5s
    echo     --benchmark_repetitions=5
    echo     --benchmark_report_aggregates_only=true
    echo.
    echo Machine characterization:
    echo   frontier_machine_bench --benchmark_min_time=0.15s --benchmark_repetitions=5
    echo     --benchmark_report_aggregates_only=true
    echo.
    echo Focused architecture kernels:
    echo   frontier_machine_bench
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
set "TEST_GENERATOR_ARGS="
if not exist "%TEST_BUILD_DIR%\CMakeCache.txt" (
    where ninja >nul 2>nul
    if not errorlevel 1 set "TEST_GENERATOR_ARGS=-G Ninja"
)

set "FAILURE_STAGE=configure-unit-tests"
echo Configuring Debug unit tests ^(BVH4 + BVH8^)...
cmake -S "%ROOT%" -B "%TEST_BUILD_DIR%" !TEST_GENERATOR_ARGS! ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DFRONTIER_BUILD_TESTS=ON ^
    -DFRONTIER_TEST_PAYLOAD32=ON ^
    -DFRONTIER_BUILD_BENCH=OFF ^
    -DFRONTIER_AVX2=ON ^
    -DFRONTIER_BVH_WIDTH=AUTO ^
    -DFRONTIER_FORCE_SCALAR=OFF ^
    -DFRONTIER_STATS=OFF ^
    -DFRONTIER_CONTRACT_CHECKS=ON ^
    -DFRONTIER_VALIDATE_SUBTREES=ON > "%REPORT_DIR%\configure-tests.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\configure-tests.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=build-unit-tests"
echo Building Debug unit tests...
cmake --build "%TEST_BUILD_DIR%" --config Debug --parallel ^
    --target frontier_unit_tests > "%REPORT_DIR%\build-tests.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\build-tests.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=correctness-tests"
echo Running Debug correctness tests...
ctest --test-dir "%TEST_BUILD_DIR%" -C Debug --output-on-failure > "%REPORT_DIR%\tests.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\tests.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=configure-performance"
echo Configuring unchecked Release AVX2 performance build...
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
    -DFRONTIER_VALIDATE_SUBTREES=OFF > "%REPORT_DIR%\configure.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\configure.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=build-performance"
echo Building performance executables...
cmake --build "%BUILD_DIR%" --config Release --parallel ^
    --target frontier_bench frontier_bench_payload32 frontier_machine_bench > "%REPORT_DIR%\build.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\build.log"
if not "!RUN_RC!"=="0" goto package

set "BENCH_EXE=%BUILD_DIR%\bench\Release\frontier_bench.exe"
set "BENCH32_EXE=%BUILD_DIR%\bench\Release\frontier_bench_payload32.exe"
set "MACHINE_EXE=%BUILD_DIR%\bench\Release\frontier_machine_bench.exe"
if not exist "%BENCH_EXE%" set "BENCH_EXE=%BUILD_DIR%\bench\frontier_bench.exe"
if not exist "%BENCH32_EXE%" set "BENCH32_EXE=%BUILD_DIR%\bench\frontier_bench_payload32.exe"
if not exist "%MACHINE_EXE%" set "MACHINE_EXE=%BUILD_DIR%\bench\frontier_machine_bench.exe"
if not exist "%BENCH_EXE%" (
    echo ERROR: frontier_bench.exe was not found under "%BUILD_DIR%\bench".
    goto package
)
if not exist "%BENCH32_EXE%" (
    echo ERROR: frontier_bench_payload32.exe was not found under "%BUILD_DIR%\bench".
    goto package
)
if not exist "%MACHINE_EXE%" (
    echo ERROR: frontier_machine_bench.exe was not found under "%BUILD_DIR%\bench".
    goto package
)

set "FAILURE_STAGE=benchmark-inventory"
"%BENCH_EXE%" --benchmark_list_tests=true > "%REPORT_DIR%\benchmark_inventory_payload64.txt" 2> "%REPORT_DIR%\benchmark_inventory_payload64.log"
if errorlevel 1 goto package
"%BENCH32_EXE%" --benchmark_list_tests=true > "%REPORT_DIR%\benchmark_inventory_payload32.txt" 2> "%REPORT_DIR%\benchmark_inventory_payload32.log"
if errorlevel 1 goto package

(
    cmake --version
    echo.
    cl
    echo.
    git --version
    echo.
    findstr /R /B "CMAKE_BUILD_TYPE: CMAKE_CXX_COMPILER: CMAKE_CXX_COMPILER_ID: CMAKE_CXX_COMPILER_VERSION: CMAKE_GENERATOR: CMAKE_OSX_ARCHITECTURES: FRONTIER_AVX2: FRONTIER_BVH_WIDTH: FRONTIER_CONTRACT_CHECKS: FRONTIER_FORCE_SCALAR: FRONTIER_IPO: FRONTIER_SSE2_ONLY: FRONTIER_STATS: FRONTIER_VALIDATE_SUBTREES:" "%BUILD_DIR%\CMakeCache.txt"
) > "%REPORT_DIR%\toolchain.txt" 2>&1

set "FAILURE_STAGE=real-world-benchmarks-payload64"
echo Running real-world performance suite with 8-byte payloads...
"%BENCH_EXE%" ^
    --benchmark_min_time=0.5s ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%REPORT_DIR%\real_world_perf_payload64.json" ^
    --benchmark_out_format=json > "%REPORT_DIR%\real_world_perf_payload64.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\real_world_perf_payload64.log"
if not "!RUN_RC!"=="0" goto package

set "FAILURE_STAGE=real-world-benchmarks-payload32"
echo Running real-world performance suite with 4-byte payloads...
"%BENCH32_EXE%" ^
    --benchmark_min_time=0.5s ^
    --benchmark_repetitions=5 ^
    --benchmark_report_aggregates_only=true ^
    --benchmark_out="%REPORT_DIR%\real_world_perf_payload32.json" ^
    --benchmark_out_format=json > "%REPORT_DIR%\real_world_perf_payload32.log" 2>&1
set "RUN_RC=!errorlevel!"
type "%REPORT_DIR%\real_world_perf_payload32.log"
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
powershell -NoProfile -Command "$names='real_world_perf_payload64.json','real_world_perf_payload32.json','machine_perf.json','arch_kernel_perf.json'; $required='BM_SubtreeAssembly_','BM_SharedNodeReadiness','BM_MotionGroupSteady','BM_MovingObjectsSelectionScale','BM_MovingCameraSelectionScale','BM_LiveCityDrivingFrame','BM_LiveCityMotionFrame','BM_FlatTlasSelectionScale','BM_InstanceForestSelectionScale','BM_FlatInstanceLifecycle','BM_BoundsOverrideBatch'; foreach($name in $names) { $path=Join-Path '%REPORT_DIR%' $name; if(-not (Test-Path -LiteralPath $path)) { Write-Error ($name + ' is missing'); exit 1 }; $json=$null; try { $json=Get-Content -Raw -LiteralPath $path | ConvertFrom-Json -ErrorAction Stop } catch { Write-Error ($name + ' is invalid JSON'); exit 1 }; if(@($json.benchmarks).Count -eq 0) { Write-Error ($name + ' contains no benchmark records'); exit 1 }; if($name -like 'real_world_perf_*') { foreach($prefix in $required) { $found=$false; foreach($record in $json.benchmarks) { if($record.name.StartsWith($prefix)) { $found=$true; break } }; if(-not $found) { Write-Error ($name + ' is missing required family ' + $prefix); exit 1 } }; $bits=if($name -like '*64.json'){'64'}else{'32'}; $inventory=Get-Content -LiteralPath (Join-Path '%REPORT_DIR%' ('benchmark_inventory_payload' + $bits + '.txt')); foreach($caseName in $inventory) { $found=$false; foreach($record in $json.benchmarks) { if($record.run_name -eq $caseName) { $found=$true; break } }; if(-not $found) { Write-Error ($name + ' is missing listed case ' + $caseName); exit 1 } } } }"
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
    echo # Frontier performance report
    echo.
    echo - Format: frontier-perf-report-v3
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
    echo - End-to-end scope: complete registered frontier_bench suite
    echo - Unit-test build: Debug; BVH4/BVH8 and 4-byte/8-byte payloads
    echo - Performance builds: matched 4-byte and 8-byte payloads; Release, contract checks and subtree validation disabled
    echo.
    echo ## Result files
    echo.
    echo - real_world_perf_payload64.json: end-to-end workloads with 8-byte payloads
    echo - real_world_perf_payload32.json: end-to-end workloads with 4-byte payloads
    echo - benchmark_inventory_payload64.txt: expected 8-byte end-to-end cases
    echo - benchmark_inventory_payload32.txt: expected 4-byte end-to-end cases
    echo - machine_perf.json: ALU, SIMD, branch, cache, latency, and bandwidth probes
    echo - arch_kernel_perf.json: focused production-kernel probes with longer sampling
    echo - tests.log: Debug BVH4/BVH8 and payload32/payload64 correctness result
    echo - hardware.txt: CPU, memory, topology, OS, and power information
    echo - toolchain.txt: compiler, CMake, and build configuration
    echo - source.txt: exact Git revision and working-tree state
    echo - commands.txt: benchmark filters and sampling parameters
    echo.
    echo The benchmarks ran without an affinity mask, matching normal scheduler behavior.
    echo Compare JSON median aggregates first and use coefficient of variation to identify unstable cases.
) > "%REPORT_DIR%\REPORT.md"

(
    echo format=frontier-perf-report-v3
    echo status=%RUN_STATUS%
    echo failure_stage=%FAILURE_STAGE%
    echo label=%RAW_LABEL%
    echo timestamp_utc=%TIMESTAMP%
    echo elapsed_seconds=%ELAPSED_SECONDS%
    echo git_commit=%GIT_COMMIT%
    echo git_dirty=%GIT_DIRTY%
    echo host_os=Windows
    echo host_arch=%PROCESSOR_ARCHITECTURE%
    echo unit_build_type=Debug
    echo unit_payload_bytes=4,8
    echo perf_build_type=Release
    echo perf_payload_bytes=4,8
    echo end_to_end_scope=complete
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

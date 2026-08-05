# Isolates the WideBlock footprint change: 256 bytes (four cache lines, always
# aligned) versus the 288 it occupied before, with identical logic on both sides
# -- the PAD288 arm just pads the struct back up. Interleaved rounds, and
# TlasScale rides along as a control on a code path neither arm touches.
$ErrorActionPreference = "Continue"
$root = "C:\Work\GitHub\HLod-tree"
Set-Location $root
$bin = "$root\bench_results\_abbin"
New-Item -ItemType Directory -Force -Path $bin | Out-Null
$filter = "BM_AssetSharing_CutCost/[01]/4000|BM_DeepTree_CutOnly/6|BM_DeepTree_StaticCamera/6|BM_TypicalForest_Breakdown/50000|BM_TlasScale/500000"
$rounds = 11

foreach ($v in @("PAD288", "BLOCK256")) {
    Write-Host "--- building $v"
    $flag = "-DHLOD_WIDE_PAD_288=" + $(if ($v -eq "PAD288") { "ON" } else { "OFF" })
    cmake -S . -B build-opt $flag | Out-Null
    $log = cmake --build build-opt --config Release --target hlod_tests hlod_bench 2>&1 | Out-String
    if ($log -match "error C|error LNK|Build FAILED") { Write-Host $log; exit 1 }
    $t = & "$root\build-opt\tests\Release\hlod_tests.exe" 2>&1 | Out-String
    if ($t -match "\[  PASSED  \] (\d+) tests") { Write-Host "  tests: PASSED $($Matches[1])" } else { Write-Host "  tests: FAILED" }
    Copy-Item "$root\build-opt\bench\Release\hlod_bench.exe" "$bin\$v.exe" -Force
}

foreach ($r in 1..$rounds) {
    foreach ($v in @("PAD288", "BLOCK256")) {
        & "$bin\$v.exe" --benchmark_filter=$filter --benchmark_min_time=0.4s `
            --benchmark_out="$bin\${r}_$v.json" --benchmark_out_format=json 2>&1 | Out-Null
    }
    Write-Host "round $r"
}

function Median($xs) { $s = @($xs | Sort-Object); return $s[[int][Math]::Floor($s.Count / 2)] }
$m = @{}
foreach ($r in 1..$rounds) {
    foreach ($v in @("PAD288", "BLOCK256")) {
        $p = "$bin\${r}_$v.json"; if (!(Test-Path $p)) { continue }
        foreach ($b in (Get-Content $p -Raw | ConvertFrom-Json).benchmarks) {
            if (-not $m.ContainsKey($b.name)) { $m[$b.name] = @{} }
            if (-not $m[$b.name].ContainsKey($v)) { $m[$b.name][$v] = @{} }
            $m[$b.name][$v][$r] = $b.real_time
        }
    }
}
Write-Host ""
Write-Host ("{0,-40} {1,11} {2,11} {3,8} {4,7}" -f "benchmark", "288B us", "256B us", "ratio", "won")
foreach ($k in ($m.Keys | Sort-Object)) {
    $ratios = @(1..$rounds | Where-Object { $m[$k]["BLOCK256"].ContainsKey($_) -and $m[$k]["PAD288"].ContainsKey($_) } |
                ForEach-Object { $m[$k]["BLOCK256"][$_] / $m[$k]["PAD288"][$_] })
    Write-Host ("{0,-40} {1,11:N1} {2,11:N1} {3,7:N3}x {4,3}/{5}" -f $k,
        (Median @($m[$k]["PAD288"].Values)), (Median @($m[$k]["BLOCK256"].Values)),
        (Median $ratios), @($ratios | Where-Object { $_ -lt 1.0 }).Count, $ratios.Count)
}

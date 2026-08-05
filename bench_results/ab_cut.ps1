# A/B harness for cut-path experiments.
#
# Two things about this machine forced the design. First, absolute timings
# drift up to 16% between batches minutes apart, so a variant measured after a
# baseline cannot be compared to it. Second, whole-binary layout is worth about
# 4-5% on code paths that were never edited, so a single pair of runs proves
# nothing either way.
#
# Hence: build every variant to its own binary FIRST, then interleave their
# runs round by round and score each variant against the SAME round's baseline.
# Report the median ratio AND the number of rounds in which the variant won --
# the win count is what separates a real effect from layout luck, because a
# null effect lands near half the rounds no matter what the median says.
#
# Always include a control benchmark whose code path the variant does not
# touch. If the control moves as much as the subject, nothing was measured.
#
#   .\bench_results\ab_cut.ps1                       # default: MASK64 vs base
#   .\bench_results\ab_cut.ps1 -Rounds 15 -Variants MASK64
param(
    [string[]]$Variants = @("MASK64"),
    [int]$Rounds = 15,
    # Two AssetSharing sizes are the only cut benchmarks on this host with a
    # resolution better than the noise floor: at ~2M cut entries per frame each
    # iteration averages over enough work to sit near 1%. TlasScale is the
    # control. DeepTree_CutOnly is informative but noisy.
    [string]$Filter = "BM_AssetSharing_CutCost/[01]/4000|BM_DeepTree_CutOnly/6|BM_TlasScale/500000"
)

$ErrorActionPreference = "Continue"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$knobs = @("MASK64")
$all   = @("BASE") + $Variants
$bin   = "$root\bench_results\_abbin"
New-Item -ItemType Directory -Force -Path $bin | Out-Null

foreach ($v in $all) {
    Write-Host "--- building $v"
    $defs = $knobs | ForEach-Object { "-DHLOD_OPT_$_=" + $(if ($v -eq $_) { "ON" } else { "OFF" }) }
    cmake -S . -B build-opt @defs | Out-Null
    $log = cmake --build build-opt --config Release --target hlod_tests hlod_bench 2>&1 | Out-String
    if ($log -match "error C|error LNK|Build FAILED") { Write-Host $log; exit 1 }
    $t = & "$root\build-opt\tests\Release\hlod_tests.exe" 2>&1 | Out-String
    if ($t -match "\[  PASSED  \] (\d+) tests") { Write-Host "  tests: PASSED $($Matches[1])" }
    else { Write-Host "  tests: FAILED"; Write-Host $t }
    Copy-Item "$root\build-opt\bench\Release\hlod_bench.exe" "$bin\$v.exe" -Force
}

foreach ($r in 1..$Rounds) {
    foreach ($v in $all) {
        & "$bin\$v.exe" --benchmark_filter=$Filter --benchmark_min_time=0.4s `
            --benchmark_out="$bin\${r}_$v.json" --benchmark_out_format=json 2>&1 | Out-Null
    }
    Write-Host "round $r"
}

function Median($xs) {
    $s = @($xs | Sort-Object); if ($s.Count -eq 0) { return $null }
    return $s[[int][Math]::Floor($s.Count / 2)]
}

$m = @{}
foreach ($r in 1..$Rounds) {
    foreach ($v in $all) {
        $p = "$bin\${r}_$v.json"; if (!(Test-Path $p)) { continue }
        foreach ($b in (Get-Content $p -Raw | ConvertFrom-Json).benchmarks) {
            if (-not $m.ContainsKey($b.name))     { $m[$b.name] = @{} }
            if (-not $m[$b.name].ContainsKey($v)) { $m[$b.name][$v] = @{} }
            $m[$b.name][$v][$r] = $b.real_time
        }
    }
}

Write-Host ""
Write-Host ("{0,-34} {1,-8} {2,11} {3,8} {4,8}" -f "benchmark", "variant", "median us", "ratio", "won")
foreach ($k in ($m.Keys | Sort-Object)) {
    foreach ($v in $Variants) {
        if (-not $m[$k].ContainsKey($v)) { continue }
        $ratios = @(1..$Rounds | Where-Object { $m[$k][$v].ContainsKey($_) -and $m[$k]["BASE"].ContainsKey($_) } |
                    ForEach-Object { $m[$k][$v][$_] / $m[$k]["BASE"][$_] })
        Write-Host ("{0,-34} {1,-8} {2,11:N1} {3,7:N3}x {4,4}/{5}" -f $k, $v,
            (Median @($m[$k][$v].Values)), (Median $ratios),
            @($ratios | Where-Object { $_ -lt 1.0 }).Count, $ratios.Count)
    }
}

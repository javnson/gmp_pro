param(
    [string[]]$Suites = @(
        "dps_clllc", "dps_fsbb", "mcs_acm_nt", "mcs_pmsm_id",
        "mcs_pmsm_nt", "pgs_inv_GFL_inverter", "pgs_inv_GFM_inverter", "pgs_sinv_rc"),
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug",
    [switch]$CollectOnly
)

$ErrorActionPreference = "Stop"
$suiteDirectory = $PSScriptRoot
$powershell = Join-Path $PSHOME "pwsh.exe"
if (-not (Test-Path -LiteralPath $powershell)) { $powershell = "powershell.exe" }
$runStatus = @()

if (-not $CollectOnly) {
    foreach ($suite in $Suites) {
        $runner = Join-Path $suiteDirectory "$suite\project\simulate\run_sil_validation_matrix.ps1"
        if (-not (Test-Path -LiteralPath $runner)) { throw "SIL validation runner missing: $runner" }
        Write-Host "`n=== $suite ===" -ForegroundColor Cyan
        & $powershell -NoProfile -ExecutionPolicy Bypass -File $runner -Configuration $Configuration
        $runStatus += [pscustomobject]@{ suite = $suite; runner_exit_code = $LASTEXITCODE }
    }
}

$levelCatalog = @{
    dps_clllc = 1..4; dps_fsbb = 1..3; mcs_acm_nt = 1..4
    mcs_pmsm_id = 1..4; mcs_pmsm_nt = 1..4
    pgs_inv_GFL_inverter = 1..6; pgs_inv_GFM_inverter = 1..5
    pgs_sinv_rc = 1..5
}
$cases = @()
foreach ($suite in $Suites) {
    $validation = Join-Path $suiteDirectory "$suite\project\simulate\validation"
    $summaryPath = Join-Path $validation "sil_validation_summary.json"
    $summaryCases = @()
    if (Test-Path -LiteralPath $summaryPath) {
        $summaryCases = @((Get-Content -Raw -LiteralPath $summaryPath | ConvertFrom-Json).cases)
    }
    foreach ($level in $levelCatalog[$suite]) {
        $metricsPath = Join-Path $validation "build_level_${level}_metrics.json"
        $logPath = Join-Path $validation "build_level_${level}_matlab.log"
        $old = $summaryCases | Where-Object build_level -eq $level | Select-Object -Last 1
        if (Test-Path -LiteralPath $metricsPath) {
            $metrics = Get-Content -Raw -LiteralPath $metricsPath | ConvertFrom-Json
            if ($null -ne $metrics.simulation_pass) {
                $cases += [pscustomobject]@{
                    suite=$suite; build_level=$level; build_pass=$true
                    simulation_pass=[bool]$metrics.simulation_pass
                    runtime_assertion_triggered=[bool]$metrics.runtime_assertion_triggered
                    dynamic_pass=[bool]$metrics.dynamic_pass
                    steady_state_pass=[bool]$metrics.steady_state_pass
                    pass=[bool]$metrics.pass
                    steady_state_error_percent=$metrics.steady_state_error_percent
                    duration_s=if ($old) {$old.duration_s} else {$null}
                    metrics_file=$metricsPath; log_file=$logPath; error=""
                }
                continue
            }
        }
        if ($old) { $cases += $old; continue }
        $log = if (Test-Path -LiteralPath $logPath) { Get-Content -Raw -LiteralPath $logPath } else { "" }
        $asserted = $log -match '(?i)assertion (detected|failed|triggered)|assert failed'
        $reason = if ($log -match '(?i)UDP frame read timed out|Cannot establish GMP SIL session') {
            "SIL UDP session failed."
        } elseif ($log) { "Validator did not produce standardized metrics." } else { "Not executed." }
        $cases += [pscustomobject]@{
            suite=$suite; build_level=$level; build_pass=$false; simulation_pass=$false
            runtime_assertion_triggered=$asserted; dynamic_pass=$false
            steady_state_pass=$false; pass=$false; steady_state_error_percent=$null
            duration_s=$null; metrics_file=$metricsPath; log_file=$logPath; error=$reason
        }
    }
}

$resultDirectory = Join-Path $suiteDirectory "validation"
New-Item -ItemType Directory -Force -Path $resultDirectory | Out-Null
$fleet = [ordered]@{
    generated_at = (Get-Date).ToString("o")
    matlab_release = "R2024b"
    suite_count = $Suites.Count
    build_level_count = $cases.Count
    pass_count = @($cases | Where-Object pass).Count
    fail_count = @($cases | Where-Object { -not $_.pass }).Count
    all_pass = ($cases.Count -gt 0 -and @($cases | Where-Object { -not $_.pass }).Count -eq 0)
    runner_status = $runStatus
    cases = $cases
}
$fleet | ConvertTo-Json -Depth 50 | Set-Content -LiteralPath (Join-Path $resultDirectory "sil_validation_fleet_summary.json") -Encoding utf8
$cases | Export-Csv -NoTypeInformation -Encoding utf8 -LiteralPath (Join-Path $resultDirectory "sil_validation_fleet_summary.csv")

$lines = @(
    "# CTL Suite SIL validation summary", "",
    "- Generated: $($fleet.generated_at)",
    "- MATLAB: R2024b", "- Suites: $($fleet.suite_count)",
    "- BUILD_LEVEL cases: $($fleet.build_level_count)",
    "- Passed: $($fleet.pass_count)", "- Failed: $($fleet.fail_count)", "",
    "| Suite | BUILD_LEVEL | Build | Simulation | Runtime assert | Dynamic | Steady state | Overall |",
    "|---|---:|:---:|:---:|:---:|:---:|:---:|:---:|"
)
foreach ($case in $cases) {
    $lines += "| $($case.suite) | $($case.build_level) | $($case.build_pass) | $($case.simulation_pass) | $($case.runtime_assertion_triggered) | $($case.dynamic_pass) | $($case.steady_state_pass) | $($case.pass) |"
}
$lines | Set-Content -LiteralPath (Join-Path $resultDirectory "sil_validation_fleet_summary.md") -Encoding utf8
$cases | Format-Table suite,build_level,build_pass,simulation_pass,runtime_assertion_triggered,dynamic_pass,steady_state_pass,pass -AutoSize
if (-not $fleet.all_pass) { exit 1 }

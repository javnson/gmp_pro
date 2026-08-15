param(
    [Parameter(Mandatory = $true)]
    [string]$SimulationDirectory,
    [int[]]$BuildLevels,
    [double]$StopTime = 0.0,
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"
$simulation = (Resolve-Path -LiteralPath $SimulationDirectory).Path
$suiteRoot = (Resolve-Path (Join-Path $simulation "..\.." )).Path
$suite = Split-Path -Leaf $suiteRoot
$repo = (Resolve-Path (Join-Path $simulation "..\..\..\..\..")).Path

$catalog = @{
    dps_clllc = @{ Levels = @(1,2,3,4); Stop = 0.20; Function = "run_clllc_validation" }
    dps_fsbb = @{ Levels = @(1,2,3); Stop = 1.50; Function = "run_fsbb_validation" }
    mcs_acm_nt = @{ Levels = @(1,2,3,4); Stop = 0.60; Function = "run_acim_validation" }
    mcs_pmsm_id = @{ Levels = @(1,2,3,4); Stop = 20.00; Function = "run_pmsm_id_validation" }
    mcs_pmsm_nt = @{ Levels = @(1,2,3,4); Stop = 0.60; Function = "run_pmsm_validation" }
    pgs_inv_GFL_inverter = @{ Levels = @(1,2,3,4,5,6); Stop = 1.00; Function = "run_gfl_validation" }
    pgs_inv_GFM_inverter = @{ Levels = @(1,2,3,4,5); Stop = 2.00; Function = "run_gfm_validation" }
    pgs_sinv_rc = @{ Levels = @(1,2,3,4,5); Stop = 2.00; Function = "run_sinv_validation" }
}
if (-not $catalog.ContainsKey($suite)) { throw "Unknown SIL suite: $suite" }
$definition = $catalog[$suite]
if (-not $BuildLevels -or $BuildLevels.Count -eq 0) { $BuildLevels = $definition.Levels }
if ($StopTime -le 0) { $StopTime = $definition.Stop }
foreach ($level in $BuildLevels) {
    if ($level -notin $definition.Levels) { throw "BUILD_LEVEL $level is not valid for $suite." }
}

$matlab = "C:\Program Files\MATLAB\R2024b\bin\matlab.exe"
$sdpe = Join-Path $repo "tools\SDPE_v2\sdpe.py"
$settings = Join-Path $repo "tools\SDPE_v2\sdpe_settings.json"
$targetRequirement = Join-Path $simulation "sdpe_mgr\sdpe_requirement.json"
$commonRequirement = Join-Path $suiteRoot "sdpe_general\sdpe_requirement.json"
$solution = Get-ChildItem -LiteralPath $simulation -Filter *.sln | Select-Object -First 1 -ExpandProperty FullName
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$msbuild = (& $vswhere -latest -products * -requires Microsoft.Component.MSBuild -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1)
if (-not (Test-Path -LiteralPath $matlab)) { throw "MATLAB R2024b not found: $matlab" }
if (-not (Test-Path -LiteralPath $sdpe)) { throw "SDPE not found: $sdpe" }
if (-not (Test-Path -LiteralPath $msbuild)) { throw "MSBuild not found." }
if (-not $solution) { throw "Visual Studio solution not found in $simulation" }

function Find-BuildLevelRequirement {
    foreach ($path in @($targetRequirement, $commonRequirement)) {
        if (-not (Test-Path -LiteralPath $path)) { continue }
        $document = Get-Content -Raw -LiteralPath $path | ConvertFrom-Json
        if ($document.option_macros | Where-Object macro -eq "BUILD_LEVEL") { return $path }
    }
    throw "No BUILD_LEVEL option macro was found for $suite."
}

function Set-BuildLevel([string]$path, [string]$source, [int]$level) {
    $document = $source | ConvertFrom-Json
    $option = $document.option_macros | Where-Object macro -eq "BUILD_LEVEL"
    $option.value = "($level)"
    [IO.File]::WriteAllText($path, ($document | ConvertTo-Json -Depth 50), [Text.UTF8Encoding]::new($false))
}

function Invoke-GenerationAndBuild([int]$level) {
    & python $sdpe --settings $settings generate-project-local $targetRequirement `
        --project-dir (Join-Path $simulation "sdpe_mgr") --out (Join-Path $simulation "sdpe_mgr")
    if ($LASTEXITCODE -ne 0) { throw "SDPE generation failed for BUILD_LEVEL $level." }
    # Source-manager synchronization preserves authoritative timestamps; a
    # generated source can therefore be older than an existing object file.
    # Rebuild prevents stale controller objects from contaminating SIL results.
    & $msbuild $solution /m /t:Rebuild "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
    if ($LASTEXITCODE -ne 0) { throw "Build failed for BUILD_LEVEL $level." }
}

$requirement = Find-BuildLevelRequirement
$original = [IO.File]::ReadAllText($requirement)
$resultDirectory = Join-Path $simulation "validation"
New-Item -ItemType Directory -Force -Path $resultDirectory | Out-Null
$env:GMP_PRO_LOCATION = $repo
$records = @()

try {
    $sourceGenerator = Join-Path $simulation "gmp_src_mgr\gmp_generate_src.bat"
    if (Test-Path -LiteralPath $sourceGenerator) {
        & $sourceGenerator
        if ($LASTEXITCODE -ne 0) { throw "GMP source synchronization failed for $suite." }
    }
    foreach ($level in $BuildLevels) {
        $label = "build_level_$level"
        $metricsFile = Join-Path $resultDirectory "${label}_metrics.json"
        $logFile = Join-Path $resultDirectory "${label}_matlab.log"
        $record = [ordered]@{
            suite = $suite; build_level = $level; build_pass = $false
            simulation_pass = $false; runtime_assertion_triggered = $false
            dynamic_pass = $false; steady_state_pass = $false; pass = $false
            duration_s = 0.0; metrics_file = $metricsFile; log_file = $logFile; error = ""
        }
        $caseTimer = [Diagnostics.Stopwatch]::StartNew()
        try {
            Set-BuildLevel $requirement $original $level
            Invoke-GenerationAndBuild $level
            $record.build_pass = $true
            $matlabDirectory = $simulation.Replace("'", "''").Replace('\','/')
            # A performance failure is data, not a MATLAB runtime failure.
            # Let MATLAB exit normally so model/process cleanup is reliable;
            # the JSON metrics below still make the matrix return non-zero.
            $command = "cd('$matlabDirectory'); m=$($definition.Function)($level,$($StopTime.ToString([Globalization.CultureInfo]::InvariantCulture)),'$label'); disp(m.pass)"
            $output = & $matlab -batch $command 2>&1
            $exitCode = $LASTEXITCODE
            $output | Set-Content -LiteralPath $logFile -Encoding utf8
            if (Test-Path -LiteralPath $metricsFile) {
                $metrics = Get-Content -Raw -LiteralPath $metricsFile | ConvertFrom-Json
                $record.simulation_pass = [bool]$metrics.simulation_pass
                $record.runtime_assertion_triggered = [bool]$metrics.runtime_assertion_triggered
                $record.dynamic_pass = [bool]$metrics.dynamic_pass
                $record.steady_state_pass = [bool]$metrics.steady_state_pass
                $record.pass = [bool]$metrics.pass -and ($exitCode -eq 0)
            }
            if ($exitCode -ne 0) {
                $joined = $output -join "`n"
                $record.runtime_assertion_triggered = $record.runtime_assertion_triggered -or ($joined -match '(?i)assertion (detected|failed|triggered)|assert failed')
                $record.error = "MATLAB exited with code $exitCode."
            }
        }
        catch {
            $record.error = $_.Exception.Message
            $record.runtime_assertion_triggered = $record.runtime_assertion_triggered -or ($record.error -match '(?i)assertion (detected|failed|triggered)|assert failed')
        }
        $caseTimer.Stop()
        $record.duration_s = [Math]::Round($caseTimer.Elapsed.TotalSeconds, 3)
        $records += [pscustomobject]$record
        $records[-1] | Format-List | Out-String | Write-Host
    }
}
finally {
    [IO.File]::WriteAllText($requirement, $original, [Text.UTF8Encoding]::new($false))
    try { Invoke-GenerationAndBuild 0 } catch { Write-Warning "Failed to restore generated/build output: $($_.Exception.Message)" }
}

$summaryPath = Join-Path $resultDirectory "sil_validation_summary.json"
if (Test-Path -LiteralPath $summaryPath) {
    try {
        $previous = (Get-Content -Raw -LiteralPath $summaryPath | ConvertFrom-Json).cases
        $retained = @($previous | Where-Object { $_.build_level -notin $BuildLevels })
        $records = @($retained) + @($records) | Sort-Object build_level
    }
    catch { Write-Warning "Could not merge the previous suite summary: $($_.Exception.Message)" }
}

$summary = [ordered]@{
    suite = $suite
    generated_at = (Get-Date).ToString("o")
    matlab = $matlab
    stop_time_s = $StopTime
    all_pass = ($records.Count -gt 0 -and @($records | Where-Object { -not $_.pass }).Count -eq 0)
    cases = $records
}
$summary | ConvertTo-Json -Depth 50 | Set-Content -LiteralPath $summaryPath -Encoding utf8
$records | Export-Csv -NoTypeInformation -Encoding utf8 -LiteralPath (Join-Path $resultDirectory "sil_validation_summary.csv")
$records | Format-Table suite,build_level,build_pass,simulation_pass,runtime_assertion_triggered,dynamic_pass,steady_state_pass,pass -AutoSize
if (-not $summary.all_pass) { exit 1 }

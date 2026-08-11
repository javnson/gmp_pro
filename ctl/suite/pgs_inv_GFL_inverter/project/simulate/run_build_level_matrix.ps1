param(
    [int[]]$BuildLevels = @(1, 2, 3, 4, 5, 6),
    [double]$StopTime = 1.0,
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $root "..\..\..\..\..")).Path
$requirement = Join-Path $root "sdpe_mgr\sdpe_requirement.json"
$sdpe = Join-Path $repo "tools\SDPE_v2\sdpe.py"
$settings = Join-Path $repo "tools\SDPE_v2\sdpe_settings.json"
$solution = Join-Path $root "GMP_Digital_Power_simulink.sln"
$matlab = "C:\Program Files\MATLAB\R2024b\bin\matlab.exe"
$msbuild = "C:\Program Files\Microsoft Visual Studio\2022\Professional\MSBuild\Current\Bin\MSBuild.exe"

if (-not (Test-Path -LiteralPath $matlab)) { throw "MATLAB R2024b not found: $matlab" }
if (-not (Test-Path -LiteralPath $msbuild)) { throw "MSBuild not found: $msbuild" }

$env:GMP_PRO_LOCATION = $repo
$original = [IO.File]::ReadAllText($requirement)
$summary = @()
try {
    foreach ($level in $BuildLevels) {
        if ($level -lt 1 -or $level -gt 6) {
            throw "BUILD_LEVEL must be in 1..6: $level"
        }
        $json = $original | ConvertFrom-Json
        $buildOption = $json.option_macros | Where-Object macro -eq "BUILD_LEVEL"
        $buildOption.value = "($level)"
        [IO.File]::WriteAllText(
            $requirement,
            ($json | ConvertTo-Json -Depth 20),
            [Text.UTF8Encoding]::new($false))

        & python $sdpe --settings $settings generate-project-local `
            $requirement --project-dir (Join-Path $root "sdpe_mgr") `
            --out (Join-Path $root "sdpe_mgr")
        if ($LASTEXITCODE -ne 0) { throw "SDPE generation failed for BL$level" }

        & $msbuild $solution /m /t:Build `
            "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
        if ($LASTEXITCODE -ne 0) { throw "Build failed for BL$level" }

        $command = "cd('$($root.Replace('\','/'))'); " +
            "m=run_gfl_validation($level,$StopTime,'build_level_$level'); " +
            "assert(m.pass,'GFL:ValidationFailed','BUILD_LEVEL $level failed');"
        & $matlab -batch $command
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed for BL$level" }
        $summary += [pscustomobject]@{ BuildLevel = $level; Pass = $true }
    }
}
finally {
    [IO.File]::WriteAllText(
        $requirement, $original, [Text.UTF8Encoding]::new($false))
    & python $sdpe --settings $settings generate-project-local `
        $requirement --project-dir (Join-Path $root "sdpe_mgr") `
        --out (Join-Path $root "sdpe_mgr")
}

$summary | Format-Table -AutoSize

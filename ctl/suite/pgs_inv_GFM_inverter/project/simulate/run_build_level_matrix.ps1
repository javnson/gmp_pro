param(
    [int[]]$BuildLevels = @(1, 2, 3, 4, 5),
    [double]$StopTime = 0.8,
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
$python = Join-Path $repo "bin\python\python.exe"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$msbuild = (& $vswhere -latest -products * -requires Microsoft.Component.MSBuild `
    -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1)

if (-not (Test-Path -LiteralPath $matlab)) { throw "MATLAB R2024b not found: $matlab" }
if (-not (Test-Path -LiteralPath $python)) { throw "GMP Python not found: $python" }
if (-not (Test-Path -LiteralPath $msbuild)) { throw "MSBuild not found: $msbuild" }

$env:GMP_PRO_LOCATION = $repo
$original = [IO.File]::ReadAllText($requirement)
$summary = @()
try {
    foreach ($level in $BuildLevels) {
        if ($level -lt 1 -or $level -gt 5) {
            throw "BUILD_LEVEL must be in 1..5: $level"
        }
        $json = $original | ConvertFrom-Json
        $buildOption = $json.option_macros | Where-Object macro -eq "BUILD_LEVEL"
        $buildOption.value = "($level)"
        [IO.File]::WriteAllText(
            $requirement,
            ($json | ConvertTo-Json -Depth 20),
            [Text.UTF8Encoding]::new($false))

        & $python $sdpe --settings $settings generate-project-local `
            $requirement --project-dir (Join-Path $root "sdpe_mgr") `
            --out (Join-Path $root "sdpe_mgr")
        if ($LASTEXITCODE -ne 0) { throw "SDPE generation failed for BL$level" }

        & $msbuild $solution /m /t:Build `
            "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
        if ($LASTEXITCODE -ne 0) { throw "Build failed for BL$level" }

        $caseStopTime = if ($level -eq 5) {
            [Math]::Max($StopTime, 2.0)
        } else {
            $StopTime
        }
        $command = "cd('$($root.Replace('\','/'))'); " +
            "m=run_gfm_validation($level,$caseStopTime,'build_level_$level'); " +
            "assert(m.pass,'GFM:ValidationFailed','BUILD_LEVEL $level failed');"
        & $matlab -batch $command
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed for BL$level" }
        $summary += [pscustomobject]@{ BuildLevel = $level; Pass = $true }
    }
}
finally {
    [IO.File]::WriteAllText(
        $requirement, $original, [Text.UTF8Encoding]::new($false))
    & $python $sdpe --settings $settings generate-project-local `
        $requirement --project-dir (Join-Path $root "sdpe_mgr") `
        --out (Join-Path $root "sdpe_mgr")
    & $msbuild $solution /m /t:Build `
        "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
}

$summary | Format-Table -AutoSize

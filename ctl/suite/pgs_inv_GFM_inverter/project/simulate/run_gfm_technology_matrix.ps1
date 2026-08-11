param(
    [int[]]$Technologies = @(1, 2, 3),
    [double]$StopTime = 2.0,
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$repo = (Resolve-Path (Join-Path $root "..\..\..\..\..")).Path
$commonRequirement = Join-Path $root "..\..\sdpe_general\sdpe_requirement.json"
$targetRequirement = Join-Path $root "sdpe_mgr\sdpe_requirement.json"
$sdpe = Join-Path $repo "tools\SDPE_v2\sdpe.py"
$settings = Join-Path $repo "tools\SDPE_v2\sdpe_settings.json"
$python = Join-Path $repo "bin\python\python.exe"
$matlab = "C:\Program Files\MATLAB\R2024b\bin\matlab.exe"
$solution = Join-Path $root "GMP_Digital_Power_simulink.sln"
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
$msbuild = (& $vswhere -latest -products * -requires Microsoft.Component.MSBuild `
    -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1)
$names = @{1 = "droop"; 2 = "vsm"; 3 = "virtual_impedance"}

$originalCommon = [IO.File]::ReadAllText($commonRequirement)
$originalTarget = [IO.File]::ReadAllText($targetRequirement)
$utf8 = [Text.UTF8Encoding]::new($false)
$summary = @()
try {
    foreach ($technology in $Technologies) {
        if (-not $names.ContainsKey($technology)) {
            throw "GFM technology must be 1, 2, or 3: $technology"
        }

        $common = $originalCommon | ConvertFrom-Json
        ($common.option_macros | Where-Object macro -eq "GFM_CONTROL_TECHNOLOGY").value = "($technology)"
        [IO.File]::WriteAllText($commonRequirement, ($common | ConvertTo-Json -Depth 20), $utf8)

        $target = $originalTarget | ConvertFrom-Json
        ($target.option_macros | Where-Object macro -eq "BUILD_LEVEL").value = "(5)"
        [IO.File]::WriteAllText($targetRequirement, ($target | ConvertTo-Json -Depth 20), $utf8)

        & $python $sdpe --settings $settings generate-project-local `
            $commonRequirement --project-dir (Split-Path $commonRequirement) `
            --out (Join-Path $root "..\..\src")
        if ($LASTEXITCODE) { throw "Common SDPE generation failed" }
        & $python $sdpe --settings $settings generate-project-local `
            $targetRequirement --project-dir (Join-Path $root "sdpe_mgr") `
            --out (Join-Path $root "sdpe_mgr")
        if ($LASTEXITCODE) { throw "Target SDPE generation failed" }

        & $msbuild $solution /m /t:Build `
            "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
        if ($LASTEXITCODE) { throw "Build failed for technology $technology" }

        $label = "build_level_5_$($names[$technology])"
        $command = "cd('$($root.Replace('\','/'))'); " +
            "m=run_gfm_validation(5,$StopTime,'$label'); " +
            "assert(m.pass,'GFM:ValidationFailed','$label failed');"
        & $matlab -batch $command
        if ($LASTEXITCODE) { throw "Simulation failed for $label" }
        $summary += [pscustomobject]@{
            Technology = $technology
            Name = $names[$technology]
            Pass = $true
        }
    }
}
finally {
    [IO.File]::WriteAllText($commonRequirement, $originalCommon, $utf8)
    [IO.File]::WriteAllText($targetRequirement, $originalTarget, $utf8)
    & $python $sdpe --settings $settings generate-project-local `
        $commonRequirement --project-dir (Split-Path $commonRequirement) `
        --out (Join-Path $root "..\..\src")
    & $python $sdpe --settings $settings generate-project-local `
        $targetRequirement --project-dir (Join-Path $root "sdpe_mgr") `
        --out (Join-Path $root "sdpe_mgr")
    & $msbuild $solution /m /t:Build `
        "/p:Configuration=$Configuration" /p:Platform=x64 /v:minimal
}

$summary | Format-Table -AutoSize

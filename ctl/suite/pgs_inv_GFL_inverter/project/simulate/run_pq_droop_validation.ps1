param(
    [double]$StopTime = 1.2,
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

$originalCommon = [IO.File]::ReadAllText($commonRequirement)
$originalTarget = [IO.File]::ReadAllText($targetRequirement)
$utf8 = [Text.UTF8Encoding]::new($false)
try {
    $common = $originalCommon | ConvertFrom-Json
    ($common.feature_macros | Where-Object macro -eq "GFL_ENABLE_PQ_DROOP").enabled = $true
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
    if ($LASTEXITCODE) { throw "GFL PQ droop build failed" }

    $command = "cd('$($root.Replace('\','/'))'); " +
        "m=run_gfl_validation(5,$StopTime,'build_level_5_pq_droop'); " +
        "assert(m.pass,'GFL:ValidationFailed','PQ droop validation failed');"
    & $matlab -batch $command
    if ($LASTEXITCODE) { throw "GFL PQ droop simulation failed" }
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

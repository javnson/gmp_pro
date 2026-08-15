param(
    [ValidateSet(
        "F2800137C",
        "F280025C",
        "F280039C",
        "F280049C",
        "F28377S",
        "F28379D",
        "F28P55X",
        "F28P65X"
    )]
    [string]$Board = "F280049C",
    [string]$CcsRoot = "C:\ti\ccs1281\ccs",
    [ValidateSet("Debug", "Release", "All")]
    [string]$Mode = "All"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$eclipse = Join-Path $CcsRoot "eclipse\eclipsec.exe"
if (-not (Test-Path -LiteralPath $eclipse -PathType Leaf)) {
    throw "CCS headless executable not found: $eclipse"
}

$workspace = Join-Path $env:TEMP ("gmp_launchpad_ccs_" + [guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Force -Path $workspace | Out-Null
$workspaceArg = $workspace.Replace("\", "/")
$projectArg = $projectRoot.Replace("\", "/")

$importCommand = '""{0}" -noSplash -data "{1}" -application com.ti.ccstudio.apps.projectImport -ccs.location "{2}""' -f $eclipse, $workspaceArg, $projectArg
& cmd.exe /d /s /c $importCommand
if ($LASTEXITCODE -ne 0) {
    throw "CCS project import failed with exit code $LASTEXITCODE"
}

$configurations = switch ($Mode) {
    "Debug"   { @("${Board}_Debug") }
    "Release" { @("${Board}_Release") }
    default   { @("${Board}_Debug", "${Board}_Release") }
}

foreach ($configuration in $configurations) {
    $buildCommand = '""{0}" -noSplash -data "{1}" -application com.ti.ccstudio.apps.projectBuild -ccs.projects GMP_C2000_LAUNCHPAD_REFERENCE -ccs.configuration {2} -ccs.buildType full -ccs.listProblems"' -f $eclipse, $workspaceArg, $configuration
    & cmd.exe /d /s /c $buildCommand
    if ($LASTEXITCODE -ne 0) {
        throw "CCS build failed for $configuration with exit code $LASTEXITCODE"
    }

    $image = Join-Path $projectRoot "$configuration\GMP_C2000_LAUNCHPAD_REFERENCE.out"
    if (-not (Test-Path -LiteralPath $image -PathType Leaf)) {
        throw "Expected image was not produced: $image"
    }
}

Write-Host "CCS build passed: $($configurations -join ', ')"

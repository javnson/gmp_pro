param(
    [string]$CcsRoot = "C:\ti\ccs1281\ccs",
    [ValidateSet("Debug", "Release")]
    [string]$Mode = "Debug"
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$dslite = Join-Path $CcsRoot "ccs_base\DebugServer\bin\DSLite.exe"
$ccxml = Join-Path $projectRoot "C2000Lib_F280049C\targetConfigs\TMS320F280049C_LaunchPad.ccxml"
$image = Join-Path $projectRoot "F280049C_$Mode\GMP_C2000_LAUNCHPAD_REFERENCE.out"

foreach ($required in @($dslite, $ccxml, $image)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

& $dslite load --config=$ccxml --timeout=120 $image
if ($LASTEXITCODE -ne 0) {
    throw "F280049C flash failed with exit code $LASTEXITCODE"
}

Write-Host "F280049C $Mode image flashed and started."

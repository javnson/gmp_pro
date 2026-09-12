param(
    [ValidateSet("Tcp", "Udp")]
    [string]$EthernetProtocol = "Tcp",
    [string]$UniFlashRoot = "C:\ti\uniflash_8.8.1",
    [string]$CcsRoot = "C:\ti\ccs1281",
    [string]$ArtifactDirectory = "",
    [int]$TimeoutSeconds = 300,
    [switch]$ListCoresOnly
)

$ErrorActionPreference = "Stop"
$targetRoot = Split-Path -Parent $PSScriptRoot
$dslite = Join-Path $UniFlashRoot "dslite.bat"
$dss = Join-Path $CcsRoot "ccs\ccs_base\scripting\bin\dss.bat"
$ccxml = Join-Path $targetRoot `
    "C2000Lib_F28388D\targetConfigs\TMS320F28388D.ccxml"
if (-not $ArtifactDirectory) {
    $ArtifactDirectory = Join-Path $targetRoot "artifacts"
}

foreach ($required in @($dslite, $dss, $ccxml)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

if ($ListCoresOnly) {
    & $dslite "--config=$ccxml" --list-cores
    exit $LASTEXITCODE
}

$cmSuffix = $EthernetProtocol.ToLowerInvariant()
$images = @(
    @{ Name = "CM ($EthernetProtocol u16 DL)"; Core = 4; File = "gmp_f28388d_controlcard_cm_${cmSuffix}_u16.out"; Run = $false },
    @{ Name = "CPU2 computation"; Core = 2; File = "gmp_f28388d_controlcard_cpu2.out"; Run = $false },
    @{ Name = "CPU1 control/serial DL"; Core = 0; File = "gmp_f28388d_controlcard_cpu1.out"; Run = $true }
)

foreach ($image in $images) {
    $path = Join-Path $ArtifactDirectory $image.File
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Image not found: $path. Run tools\build.ps1 first."
    }

    Write-Host "Flashing $($image.Name) on core index $($image.Core)..."
    $arguments = @(
        "--config=$ccxml",
        "--core=$($image.Core)",
        "--timeout=$TimeoutSeconds",
        "--flash",
        "--verify"
    )
    if ($image.Run) {
        $arguments += "--run"
    }
    $arguments += $path
    & $dslite @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Flash failed for $($image.Name) with exit code $LASTEXITCODE"
    }
}

Write-Host "Resuming CPU1, CM, and CPU2 through the two IPC boot barriers..."
$cpu1Image = Join-Path $ArtifactDirectory "gmp_f28388d_controlcard_cpu1.out"
$cpu2Image = Join-Path $ArtifactDirectory "gmp_f28388d_controlcard_cpu2.out"
$cmImage = Join-Path $ArtifactDirectory "gmp_f28388d_controlcard_cm_${cmSuffix}_u16.out"
& $dss (Join-Path $PSScriptRoot "resume_tricore.js") $ccxml `
    $cpu1Image $cpu2Image $cmImage
if ($LASTEXITCODE -ne 0) {
    throw "Tri-core resume failed with exit code $LASTEXITCODE"
}

Write-Host "F28388D images verified; CPU1, CPU2, and CM were released."

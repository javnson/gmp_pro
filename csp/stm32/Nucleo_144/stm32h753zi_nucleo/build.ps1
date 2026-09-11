[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",

    [ValidateSet("TCP", "UDP")]
    [string]$DatalinkTransport = "TCP"
)
$ErrorActionPreference = "Stop"
$boardDir = $PSScriptRoot
$repoRoot = (Resolve-Path (Join-Path $boardDir "..\..\..\..")).Path
$python = (Get-Command python).Source

if (-not (Test-Path -LiteralPath (Join-Path $boardDir "Core\Src\main.c"))) {
    throw "CubeMX output is missing. Generate the IOC with generate_cubemx.txt before building."
}

& $python (Join-Path $repoRoot "csp\stm32\Nucleo_144\tools\validate_ioc.py") $boardDir
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $python (Join-Path $repoRoot "tools\SDPE_v2\sdpe.py") --settings (Join-Path $repoRoot "tools\SDPE_v2\sdpe_settings.json") validate
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $python (Join-Path $repoRoot "tools\SDPE_v2\sdpe.py") --settings (Join-Path $repoRoot "tools\SDPE_v2\sdpe_settings.json") generate-entity nucleo_h753zi
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Push-Location (Join-Path $boardDir "sdpe_mgr")
try {
    & $python (Join-Path $repoRoot "tools\SDPE_v2\sdpe.py") --settings (Join-Path $repoRoot "tools\SDPE_v2\sdpe_settings.json") generate-project-local sdpe_requirement.json
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
finally { Pop-Location }

Push-Location (Join-Path $repoRoot "csp\stm32\Nucleo_144\src\gmp_src_mgr")
try {
    & $python (Join-Path $repoRoot "tools\facilities_generator\src_mgr\framework_sync_inc_v3.py")
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $python (Join-Path $repoRoot "tools\facilities_generator\src_mgr\framework_sync_src_v3.py")
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
finally { Pop-Location }

& $python (Join-Path $boardDir "tools\patch_generated_main.py") (Join-Path $boardDir "Core\Src\main.c")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$buildDir = Join-Path $boardDir ("build\" + $Configuration.ToLowerInvariant())
cmake -S $boardDir -B $buildDir -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$(Join-Path $boardDir 'cmake\gcc-arm-none-eabi.cmake')" "-DCMAKE_BUILD_TYPE=$Configuration" "-DGMP_DATALINK_TRANSPORT=$DatalinkTransport"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build $buildDir
exit $LASTEXITCODE

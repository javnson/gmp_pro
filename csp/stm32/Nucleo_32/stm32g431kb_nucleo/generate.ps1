[CmdletBinding()]
param(
    [string]$CubeMXHome = $(
        if ($env:STM32CUBEMX_HOME) { $env:STM32CUBEMX_HOME }
        else { "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX" }
    )
)
$ErrorActionPreference = "Stop"
$boardDir = $PSScriptRoot
$java = Join-Path $CubeMXHome "jre\bin\java.exe"
$cubeMx = Join-Path $CubeMXHome "STM32CubeMX.exe"
if (-not (Test-Path -LiteralPath $java)) { throw "CubeMX Java runtime not found: $java" }
if (-not (Test-Path -LiteralPath $cubeMx)) { throw "STM32CubeMX not found: $cubeMx" }

$canonicalIoc = Join-Path $boardDir "stm32g431kb_nucleo.ioc"
$backupIoc = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("gmp_nucleo32_" + [guid]::NewGuid().ToString("N") + ".ioc")
$stagingDir = Join-Path $boardDir "MXTmpFiles"
New-Item -ItemType Directory -Force $stagingDir | Out-Null
Copy-Item $canonicalIoc $backupIoc -Force
Copy-Item $canonicalIoc `
    (Join-Path $stagingDir "canonical_stm32g431kb_nucleo.ioc") -Force

try {
    Push-Location $boardDir
    try {
        & $java -jar $cubeMx -q (Join-Path $boardDir "generate_cubemx.txt")
        $generateExitCode = $LASTEXITCODE
    }
    finally { Pop-Location }
}
finally {
    Copy-Item $backupIoc $canonicalIoc -Force
    Remove-Item -LiteralPath $backupIoc -Force
}
if ($generateExitCode -ne 0) { exit $generateExitCode }

$expected = @(
    "Core\Src\main.c",
    "Core\Src\stm32g4xx_hal_msp.c",
    "Core\Src\stm32g4xx_it.c",
    "Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal.c"
)
foreach ($relative in $expected) {
    $path = Join-Path $boardDir $relative
    if (-not (Test-Path -LiteralPath $path)) { throw "CubeMX did not generate: $path" }
}
Write-Output "CubeMX generated STM32G431KB sources successfully."

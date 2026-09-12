[CmdletBinding()]
param(
    [string]$CubeMXHome = $(
        if ($env:STM32CUBEMX_HOME) { $env:STM32CUBEMX_HOME }
        else { "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX" }
    ),
    [ValidateSet("CubeIDE", "Keil", "All")]
    [string]$Ide = "All"
)
$ErrorActionPreference = "Stop"
$boardDir = $PSScriptRoot
$java = Join-Path $CubeMXHome "jre\bin\java.exe"
$cubeMx = Join-Path $CubeMXHome "STM32CubeMX.exe"
if (-not (Test-Path -LiteralPath $java)) { throw "CubeMX Java runtime not found: $java" }
if (-not (Test-Path -LiteralPath $cubeMx)) { throw "STM32CubeMX not found: $cubeMx" }

$canonicalIoc = Join-Path $boardDir "stm32h753zi_nucleo.ioc"
$backupIoc = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("gmp_nucleo144_rtos_" + [guid]::NewGuid().ToString("N") + ".ioc")
$stagingDir = Join-Path $boardDir "MXTmpFiles"
New-Item -ItemType Directory -Force $stagingDir | Out-Null
Copy-Item $canonicalIoc $backupIoc -Force
Copy-Item $canonicalIoc `
    (Join-Path $stagingDir "canonical_stm32h753zi_nucleo.ioc") -Force

try {
    $commandFiles = switch ($Ide) {
        "CubeIDE" { @("generate_cubemx.txt") }
        "Keil" { @("generate_keil.txt") }
        default { @("generate_cubemx.txt", "generate_keil.txt") }
    }
    $generateExitCode = 0
    foreach ($commandFile in $commandFiles) {
        Push-Location $boardDir
        try {
            & $java -jar $cubeMx -q (Join-Path $boardDir $commandFile)
            $generateExitCode = $LASTEXITCODE
        }
        finally { Pop-Location }
        if ($generateExitCode -ne 0) { break }
    }
}
finally {
    Copy-Item $backupIoc $canonicalIoc -Force
    Remove-Item -LiteralPath $backupIoc -Force
}
if ($generateExitCode -ne 0) { exit $generateExitCode }

$expected = @(
    "Core\Src\main.c",
    "Core\Src\stm32h7xx_hal_msp.c",
    "Core\Src\stm32h7xx_it.c",
    "Core\Src\stm32h7xx_hal_timebase_tim.c"
)
foreach ($relative in $expected) {
    $path = Join-Path $boardDir $relative
    if (-not (Test-Path -LiteralPath $path)) { throw "CubeMX did not generate: $path" }
}

$python = (Get-Command python).Source
& $python (Join-Path $boardDir "tools\patch_generated_main.py") `
    (Join-Path $boardDir "Core\Src\main.c")
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
Write-Output "CubeMX generated the STM32H753ZI RTOS project successfully."

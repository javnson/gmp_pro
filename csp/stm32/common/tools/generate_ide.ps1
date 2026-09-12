[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BoardDir,

    [ValidateSet("CubeIDE", "Keil", "All")]
    [string]$Ide = "All",

    [string]$CubeMXHome = $(
        if ($env:STM32CUBEMX_HOME) { $env:STM32CUBEMX_HOME }
        else { "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeMX" }
    )
)

$ErrorActionPreference = "Stop"
$board = (Resolve-Path -LiteralPath $BoardDir).Path
$iocFiles = @(Get-ChildItem -LiteralPath $board -Filter *.ioc -File)
if ($iocFiles.Count -ne 1) {
    throw "Expected exactly one IOC file under $board; found $($iocFiles.Count)."
}

$java = Join-Path $CubeMXHome "jre\bin\java.exe"
$cubeMx = Join-Path $CubeMXHome "STM32CubeMX.exe"
if (-not (Test-Path -LiteralPath $java)) { throw "CubeMX Java runtime not found: $java" }
if (-not (Test-Path -LiteralPath $cubeMx)) { throw "STM32CubeMX not found: $cubeMx" }

$ioc = $iocFiles[0]
$backupIoc = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("gmp_cubemx_ioc_" + [guid]::NewGuid().ToString("N") + ".ioc")
$commandFile = Join-Path ([System.IO.Path]::GetTempPath()) `
    ("gmp_cubemx_command_" + [guid]::NewGuid().ToString("N") + ".txt")
Copy-Item -LiteralPath $ioc.FullName -Destination $backupIoc -Force

$toolchains = switch ($Ide) {
    "CubeIDE" { @("STM32CubeIDE") }
    "Keil" { @("MDK-ARM") }
    default { @("STM32CubeIDE", "MDK-ARM") }
}

try {
    foreach ($toolchain in $toolchains) {
        @(
            "config load $($ioc.Name)",
            "project name $($ioc.BaseName)",
            "project toolchain $toolchain",
            "project path ..",
            "project generate",
            "exit"
        ) | Set-Content -LiteralPath $commandFile -Encoding ascii

        Push-Location $board
        try {
            & $java -jar $cubeMx -q $commandFile
            if ($LASTEXITCODE -ne 0) {
                throw "CubeMX $toolchain generation failed with exit code $LASTEXITCODE."
            }
        }
        finally { Pop-Location }
    }
}
finally {
    Copy-Item -LiteralPath $backupIoc -Destination $ioc.FullName -Force
    Remove-Item -LiteralPath $backupIoc -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $commandFile -Force -ErrorAction SilentlyContinue
}

Write-Output "CubeMX generated $Ide metadata for $($ioc.BaseName); the canonical IOC was restored."

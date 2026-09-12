[CmdletBinding()]
param(
    [ValidateSet("CubeIDE", "Keil", "All")]
    [string]$Ide = "All"
)
$ErrorActionPreference = "Stop"
$generator = Join-Path $PSScriptRoot "..\..\common\tools\generate_ide.ps1"
& $generator -BoardDir $PSScriptRoot -Ide $Ide
exit $LASTEXITCODE

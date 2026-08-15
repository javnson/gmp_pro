param([int[]]$BuildLevels, [double]$StopTime = 0, [string]$Configuration = "Debug")
& (Join-Path $PSScriptRoot "..\..\..\sil_validation\run_suite_validation.ps1") -SimulationDirectory $PSScriptRoot -BuildLevels $BuildLevels -StopTime $StopTime -Configuration $Configuration
exit $LASTEXITCODE

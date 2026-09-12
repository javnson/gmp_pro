[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$BoardDir,

    [Parameter(Mandatory = $true)]
    [string]$BuildDir
)

$ErrorActionPreference = "Stop"
$board = (Resolve-Path -LiteralPath $BoardDir).Path
$build = (Resolve-Path -LiteralPath $BuildDir).Path
$compileCommands = Join-Path $build "compile_commands.json"
if (-not (Test-Path -LiteralPath $compileCommands)) {
    throw "CMake compile database not found: $compileCommands"
}

$python = (Get-Command python).Source
$sync = Join-Path $PSScriptRoot "sync_cmake_ide.py"
$projectName = (Get-ChildItem -LiteralPath $board -Filter *.ioc -File | Select-Object -First 1).BaseName

function Invoke-IdeSync {
    param([string[]]$Arguments)
    & $python $sync @Arguments
    if ($LASTEXITCODE -ne 0) { throw "IDE synchronization failed." }
}

$common = @("--board", $board, "--compile-commands", $compileCommands)
$cubeRoot = Join-Path $board "STM32CubeIDE"
if (Test-Path -LiteralPath (Join-Path $cubeRoot ".project")) {
    $cm4 = Join-Path $cubeRoot "CM4"
    $cm7 = Join-Path $cubeRoot "CM7"
    if ((Test-Path -LiteralPath (Join-Path $cm4 ".project")) -and
        (Test-Path -LiteralPath (Join-Path $cm7 ".project"))) {
        Invoke-IdeSync ($common + @("--cubeide", $cm4, "--target-object-fragment", "$($projectName)_cm4.dir"))
        Invoke-IdeSync ($common + @("--cubeide", $cm7, "--target-object-fragment", "$($projectName)_cm7.dir"))
    }
    else {
        Invoke-IdeSync ($common + @("--cubeide", $cubeRoot))
    }
}

$keilProjects = @(Get-ChildItem -LiteralPath (Join-Path $board "MDK-ARM") -Filter *.uvprojx -File -ErrorAction SilentlyContinue)
if ($keilProjects.Count -gt 1) {
    throw "Expected at most one uvprojx under $board; found $($keilProjects.Count)."
}
if ($keilProjects.Count -eq 1) {
    [xml]$keilXml = Get-Content -LiteralPath $keilProjects[0].FullName
    $targets = @($keilXml.Project.Targets.Target | ForEach-Object { $_.TargetName })
    $coreTargets = @($targets | Where-Object { $_ -match "_CM[47]$" })
    if ($coreTargets.Count -gt 0) {
        foreach ($target in $coreTargets) {
            $core = ([regex]::Match($target, "CM[47]$")).Value.ToLowerInvariant()
            Invoke-IdeSync ($common + @(
                "--keil", $keilProjects[0].FullName,
                "--keil-target", $target,
                "--target-object-fragment", "$($projectName)_$core.dir"
            ))
        }
    }
    else {
        Invoke-IdeSync ($common + @("--keil", $keilProjects[0].FullName))
    }
}

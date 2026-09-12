param(
    [string]$CcsRoot = "C:\ti\ccs1281\ccs",
    [string]$Workspace = "",
    [string]$ArtifactDirectory = "",
    [switch]$SkipSdpe,
    [switch]$SkipGmpSources
)

$ErrorActionPreference = "Stop"
$targetRoot = Split-Path -Parent $PSScriptRoot
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $targetRoot "..\..\..")).Path
$eclipse = Join-Path $CcsRoot "eclipse\eclipsec.exe"
$projectSpec = Join-Path $targetRoot "gmp_f28388d_tricore.projectspec"
$python = Join-Path $repositoryRoot "bin\python\python.exe"
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    $python = (Get-Command python -ErrorAction Stop).Source
}

foreach ($required in @($eclipse, $projectSpec)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw "Required file not found: $required"
    }
}

$env:GMP_PRO_LOCATION = $repositoryRoot

if (-not $SkipSdpe) {
    $sdpe = Join-Path $repositoryRoot "tools\SDPE_v2\sdpe.py"
    $settings = Join-Path $repositoryRoot "tools\SDPE_v2\sdpe_settings.json"
    $requirement = Join-Path $targetRoot "src\sdpe_mgr\sdpe_requirement.json"
    $commonOutput = Join-Path $targetRoot "src\common"

    & $python $sdpe --settings $settings validate
    if ($LASTEXITCODE -ne 0) {
        throw "SDPE validation failed with exit code $LASTEXITCODE"
    }
    & $python $sdpe --settings $settings generate-project-local $requirement `
        --project-dir (Split-Path -Parent $requirement) --out $commonOutput
    if ($LASTEXITCODE -ne 0) {
        throw "SDPE generation failed with exit code $LASTEXITCODE"
    }
}

if (-not $SkipGmpSources) {
    $sourceGenerator = Join-Path $targetRoot "gmp_src_mgr\gmp_generate_all.bat"
    & $sourceGenerator
    if ($LASTEXITCODE -ne 0) {
        throw "GMP source generation failed with exit code $LASTEXITCODE"
    }
}

if (-not $Workspace) {
    $Workspace = Join-Path $targetRoot ("build\ccs_workspace_" + [guid]::NewGuid().ToString("N"))
}
if (-not $ArtifactDirectory) {
    $ArtifactDirectory = Join-Path $targetRoot "artifacts"
}
New-Item -ItemType Directory -Force -Path $Workspace, $ArtifactDirectory | Out-Null
$Workspace = (Resolve-Path -LiteralPath $Workspace).Path
$ArtifactDirectory = (Resolve-Path -LiteralPath $ArtifactDirectory).Path

$importCommand = '""{0}" -noSplash -data "{1}" -application com.ti.ccstudio.apps.importProject -ccs.location "{2}""' -f `
    $eclipse, $Workspace, $projectSpec
& cmd.exe /d /s /c $importCommand
if ($LASTEXITCODE -ne 0) {
    throw "CCS project import failed with exit code $LASTEXITCODE"
}

$builds = @(
    @{ Project = "gmp_f28388d_controlcard_cpu1"; Configuration = "CPU1_FLASH"; Artifact = "gmp_f28388d_controlcard_cpu1.out" },
    @{ Project = "gmp_f28388d_controlcard_cpu2"; Configuration = "CPU2_FLASH"; Artifact = "gmp_f28388d_controlcard_cpu2.out" },
    @{ Project = "gmp_f28388d_controlcard_cm"; Configuration = "CM_FLASH_TCP_U16"; Artifact = "gmp_f28388d_controlcard_cm_tcp_u16.out" },
    @{ Project = "gmp_f28388d_controlcard_cm"; Configuration = "CM_FLASH_UDP_U16"; Artifact = "gmp_f28388d_controlcard_cm_udp_u16.out" }
)

foreach ($build in $builds) {
    $buildCommand = '""{0}" -noSplash -data "{1}" -application com.ti.ccstudio.apps.projectBuild -ccs.projects {2} -ccs.configuration {3} -ccs.buildType full -ccs.listProblems"' -f `
        $eclipse, $Workspace, $build.Project, $build.Configuration
    & cmd.exe /d /s /c $buildCommand
    if ($LASTEXITCODE -ne 0) {
        throw "CCS build failed for $($build.Project)/$($build.Configuration) with exit code $LASTEXITCODE"
    }

    $sourceImage = Join-Path $Workspace (
        "$($build.Project)\$($build.Configuration)\$($build.Project).out"
    )
    if (-not (Test-Path -LiteralPath $sourceImage -PathType Leaf)) {
        throw "Expected image was not produced: $sourceImage"
    }
    Copy-Item -LiteralPath $sourceImage `
        -Destination (Join-Path $ArtifactDirectory $build.Artifact) -Force
}

Write-Host "F28388D ControlCARD build passed."
Write-Host "Workspace: $Workspace"
Write-Host "Artifacts: $ArtifactDirectory"

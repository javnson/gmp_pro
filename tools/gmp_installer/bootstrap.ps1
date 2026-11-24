param(
    [Parameter(Mandatory = $true)]
    [ValidateSet('online', 'system', 'deploy', 'repair')]
    [string] $Mode,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $Options
)

$ErrorActionPreference = 'Stop'
$root = $env:GMP_PRO_LOCATION
$bin = Join-Path $root 'bin'
$python = Join-Path $bin 'python\python.exe'
$manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'environment_manifest.json') -Raw | ConvertFrom-Json
$downloadDir = Join-Path $bin 'cache\downloads'
$wheelDir = Join-Path $bin 'cache\wheels'

function Get-Sha256([string] $path) {
    $stream = [IO.File]::OpenRead($path)
    $sha = [Security.Cryptography.SHA256]::Create()
    try {
        return [BitConverter]::ToString($sha.ComputeHash($stream)).Replace('-', '')
    } finally {
        $stream.Dispose()
        $sha.Dispose()
    }
}

function Get-VerifiedArtifact($artifact, [string] $directory) {
    if ($artifact.sha256 -notmatch '^[a-f0-9]{64}$') {
        throw "Missing SHA-256 for $($artifact.url)"
    }
    $uri = [uri]$artifact.url
    if ($uri.Scheme -ne 'https') {
        throw "An HTTPS download URL is required for $($artifact.url)"
    }
    $name = if ($artifact.file) { $artifact.file } else { [IO.Path]::GetFileName($uri.AbsolutePath) }
    $destination = Join-Path $directory $name
    if (Test-Path -LiteralPath $destination -PathType Leaf) {
        if ((Get-Sha256 $destination) -eq $artifact.sha256) {
            Write-Host "[VERIFIED CACHE] $name"
            return $destination
        }
        Write-Host "[INVALID CACHE] $name. Downloading a verified replacement."
    }

    New-Item -ItemType Directory -Force -Path $directory | Out-Null
    $temporary = "$destination.$PID.part"
    try {
        Write-Host "[DOWNLOAD] $name"
        $arguments = @('--fail', '--location', '--retry', '2', '--connect-timeout', '20',
            '--max-time', '900', '--silent', '--show-error', '--output', $temporary)
        & curl.exe @arguments $uri.AbsoluteUri
        $verified = $LASTEXITCODE -eq 0 -and (Get-Sha256 $temporary) -eq $artifact.sha256
        if (-not $verified -and $artifact.asset_api) {
            Write-Host "[DOWNLOAD] Retrying $name through the official release asset API."
            & curl.exe @arguments -H 'Accept: application/octet-stream' -H 'User-Agent: GMP-Installer' $artifact.asset_api
            $verified = $LASTEXITCODE -eq 0 -and (Get-Sha256 $temporary) -eq $artifact.sha256
        }
        if (-not $verified) {
            throw "Download failed or SHA-256 did not match for $name (curl exit $LASTEXITCODE)."
        }
        if ([IO.File]::Exists($destination)) {
            [IO.File]::Replace($temporary, $destination, [NullString]::Value)
        } else {
            [IO.File]::Move($temporary, $destination)
        }
        Write-Host "[VERIFIED] $name"
        return $destination
    } finally {
        if (Test-Path -LiteralPath $temporary -PathType Leaf) {
            Remove-Item -LiteralPath $temporary
        }
    }
}

function Get-LockedWheels {
    $lock = Join-Path $PSScriptRoot $manifest.python.lock
    foreach ($line in Get-Content -LiteralPath $lock) {
        if ($line -match '^\s*(?:#.*)?$') { continue }
        if ($line -notmatch '^[\w.-]+==[\w.+!-]+ --hash=sha256:([a-f0-9]{64}) # (https://files\.pythonhosted\.org/\S+\.whl)$') {
            throw "Invalid wheel entry in $lock"
        }
        [pscustomobject]@{ sha256 = $Matches[1]; url = $Matches[2]; file = $null }
    }
}

function Show-Plan {
    Write-Host "[PLAN] $Mode installation in $root"
    Write-Host "Python $($manifest.python.version) and locked Windows x64 wheels from $($manifest.python.lock)"
    foreach ($app in $manifest.applications) {
        Write-Host "$($app.name) $($app.version) -> bin/$($app.destination)"
    }
    Write-Host "vcpkg ports $($manifest.vcpkg.repository_version), tool $($manifest.vcpkg.tool_version), $($manifest.vcpkg.triplet)"
    if ($Mode -eq 'deploy') {
        Write-Host 'Verify the copied bin, refresh proxy settings and reconfigure this checkout offline.'
    } else {
        Write-Host 'Download and verify every locked installer, archive and wheel before installation.'
        if ('--download-only' -in $Options) {
            Write-Host 'Stop after preparing bin/cache. No installation or persistent settings.'
            return
        }
        if ('--skip-vcpkg-packages' -notin $Options) {
            Write-Host 'Restore shared simulation packages if Visual Studio C++ is available.'
        }
    }
    if ('--skip-project-setup' -notin $Options) {
        Write-Host 'Configure CCS metadata, facilities, source manager, SDPE and project ignore files.'
    }
    if ('--skip-project-setup' -in $Options -or '--skip-vcpkg-packages' -in $Options) {
        Write-Host 'Diagnostic mode. No completion marker or new persistent settings.'
        return
    }
    Write-Host 'Publish GMP_PRO_LOCATION for the current user after a complete installation.'
    if ($Mode -eq 'system') {
        Write-Host 'Also publish the tool directories to user PATH and enable user-wide vcpkg integration if Visual Studio C++ is available.'
    }
}

$installLock = $null
try {
    $allowed = @()
    if ($Mode -ne 'repair') { $allowed += @('--plan', '--skip-project-setup') }
    $installing = $Mode -in @('online', 'system')
    if ($installing) { $allowed += @('--download-only', '--skip-vcpkg-packages') }
    foreach ($option in $Options) {
        if ($option -cnotin $allowed) { throw "Unknown option for $($Mode): $option" }
    }
    if ('--plan' -in $Options) {
        Show-Plan
        exit 0
    }

    New-Item -ItemType Directory -Force -Path (Join-Path $bin 'cache') | Out-Null
    try {
        $installLock = [IO.File]::Open((Join-Path $bin 'cache\installer.lock'), 'OpenOrCreate', 'ReadWrite', 'None')
    } catch {
        throw "Cannot acquire the GMP installer lock. Check that no other installation is running. $($_.Exception.Message)"
    }
    Remove-Item Env:PYTHONHOME, Env:PYTHONPATH -ErrorAction SilentlyContinue
    $env:PYTHONNOUSERSITE = '1'
    if ($installing) {
        $installer = Get-VerifiedArtifact $manifest.python.installer $downloadDir
        foreach ($app in $manifest.applications) {
            $null = Get-VerifiedArtifact $app $downloadDir
        }
        $null = Get-VerifiedArtifact $manifest.vcpkg.repository $downloadDir
        $null = Get-VerifiedArtifact $manifest.vcpkg.tool $downloadDir
        foreach ($wheel in Get-LockedWheels) {
            $null = Get-VerifiedArtifact $wheel $wheelDir
        }
        Write-Host '[VERIFIED] All locked installation files are ready.'
        if ('--download-only' -in $Options) { exit 0 }

        foreach ($name in 'gmp_virtual_env_installed.flag', 'gmp_system_env_installed.flag', 'gmp_environment.json') {
            $path = Join-Path $bin $name
            if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path -Force }
        }
        $version = if (Test-Path -LiteralPath $python -PathType Leaf) {
            & $python -E --version
        }
        if ($version -ne "Python $($manifest.python.version)") {
            Write-Host "[INSTALL] Python $($manifest.python.version)"
            $arguments = @(
                '/quiet', 'InstallAllUsers=0', "TargetDir=""$bin\python""",
                'PrependPath=0', 'AppendPath=0', 'Include_launcher=0', 'Include_doc=0',
                'Include_test=0', 'Include_pip=1', 'Include_tcltk=1', 'Include_tools=1',
                'Include_symbols=0', 'Include_debug=0', 'Shortcuts=0', 'AssociateFiles=0'
            )
            $result = Start-Process -FilePath $installer -ArgumentList $arguments -Wait -PassThru -WindowStyle Hidden
            if ($result.ExitCode -ne 0) {
                throw "Python installer exited with code $($result.ExitCode)."
            }
        }
    }

    if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
        throw "Required Python is missing: $python"
    }
    if ((& $python -E --version) -ne "Python $($manifest.python.version)") {
        throw "Expected Python $($manifest.python.version) at $python."
    }
    $command = if ($Mode -eq 'repair') { 'restore-vcpkg' } else { $Mode }
    & $python -B (Join-Path $PSScriptRoot 'environment_manager.py') $command @Options
    exit $LASTEXITCODE
} catch {
    Write-Host "[ERROR] $($_.Exception.Message)"
    exit 1
} finally {
    if ($null -ne $installLock) { $installLock.Dispose() }
}

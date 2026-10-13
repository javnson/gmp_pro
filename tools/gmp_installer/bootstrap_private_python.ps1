param([switch]$Plan)

$ErrorActionPreference = 'Stop'

function Test-PrivatePython([string]$Directory, [string]$Version) {
    $executable = Join-Path $Directory 'python.exe'
    & $executable -I -c "import ctypes, ensurepip, pathlib, ssl, sys, tkinter; root = pathlib.Path(sys.argv[2]).resolve(); assert sys.version.split()[0] == sys.argv[1], sys.version; assert ctypes.sizeof(ctypes.c_void_p) == 8; assert pathlib.Path(sys.prefix).resolve() == root, sys.prefix; assert pathlib.Path(sys.base_prefix).resolve() == root, sys.base_prefix; assert pathlib.Path(sys.executable).resolve() == root / 'python.exe', sys.executable; tkinter.Tcl()" $Version $Directory
    if ($LASTEXITCODE -ne 0) {
        throw "Private Python validation failed at $Directory. Move bin/python aside and rerun the installer."
    }
}

$stage = $null
try {
    $root = [IO.Path]::GetFullPath($env:GMP_PRO_LOCATION)
    $expectedRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
    if ($root.TrimEnd('\') -ne $expectedRoot.TrimEnd('\')) {
        throw 'GMP_PRO_LOCATION points to a different repository.'
    }
    $manifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'environment_manifest.json') -Raw | ConvertFrom-Json
    $config = $manifest.python
    $pythonHome = Join-Path $root 'bin/python'
    if ($Plan) {
        Write-Host "Python       : $($config.version), standalone x64 runtime in bin\python"
        exit 0
    }
    if (Test-Path -LiteralPath (Join-Path $pythonHome 'python.exe')) {
        Test-PrivatePython $pythonHome $config.version
        Write-Host "[OK] Private Python $($config.version) already installed."
        exit 0
    }
    if (Test-Path -LiteralPath $pythonHome) {
        throw 'Incomplete bin/python directory. Move it aside and rerun the installer.'
    }

    $curl = (Get-Command curl.exe -ErrorAction Stop).Source
    $tar = (Get-Command tar.exe -ErrorAction Stop).Source
    $cache = Join-Path $root 'bin/cache/downloads'
    New-Item -ItemType Directory -Path $cache -Force | Out-Null
    $filename = [IO.Path]::GetFileName([Uri]::UnescapeDataString(([Uri]$config.url).AbsolutePath))
    $archive = Join-Path $cache $filename
    if (!(Test-Path -LiteralPath $archive) -or (Get-FileHash -LiteralPath $archive -Algorithm SHA256).Hash -ne $config.sha256) {
        Write-Host "[DOWNLOAD] Standalone Python $($config.version)..."
        & $curl --fail --location --retry 3 --connect-timeout 20 --speed-time 30 --speed-limit 1024 --continue-at - --output "$archive.part" $config.url
        if ($LASTEXITCODE -ne 0) { throw 'Private Python download failed. Check the selected proxy or network connection.' }
        if ((Get-FileHash -LiteralPath "$archive.part" -Algorithm SHA256).Hash -ne $config.sha256) {
            Remove-Item -LiteralPath "$archive.part" -Force
            throw 'Private Python archive failed SHA-256 validation.'
        }
        Move-Item -LiteralPath "$archive.part" -Destination $archive -Force
    }

    $stage = [IO.Path]::GetFullPath((Join-Path $cache ('python-extract-' + [Guid]::NewGuid().ToString('N'))))
    New-Item -ItemType Directory -Path $stage | Out-Null
    Write-Host '[EXTRACT] Installing repository-private Python...'
    & $tar -xf $archive -C $stage
    if ($LASTEXITCODE -ne 0) { throw 'Private Python archive extraction failed.' }
    $extracted = Join-Path $stage 'python'
    Test-PrivatePython $extracted $config.version
    Move-Item -LiteralPath $extracted -Destination $pythonHome
    Test-PrivatePython $pythonHome $config.version
    Write-Host "[OK] Private Python installed at $pythonHome."
} catch {
    Write-Host "[ERROR] $($_.Exception.Message)"
    exit 1
} finally {
    if ($stage -and (Test-Path -LiteralPath $stage)) {
        $cachePrefix = [IO.Path]::GetFullPath($cache).TrimEnd('\') + '\'
        if (!$stage.StartsWith($cachePrefix, [StringComparison]::OrdinalIgnoreCase)) {
            throw 'Refusing to remove a Python staging directory outside the private downloads cache.'
        }
        Remove-Item -LiteralPath $stage -Recurse -Force
    }
}

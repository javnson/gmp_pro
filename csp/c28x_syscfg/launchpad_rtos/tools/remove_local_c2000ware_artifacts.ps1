$ErrorActionPreference = 'Stop'

$projectRoot = (Resolve-Path -LiteralPath (Split-Path -Parent $PSScriptRoot)).Path
$boardDirectories = Get-ChildItem -LiteralPath $projectRoot -Directory -Filter 'C2000Lib_*'
$filesToRemove = [System.Collections.Generic.List[System.IO.FileInfo]]::new()
$directoriesToRemove = [System.Collections.Generic.List[System.IO.DirectoryInfo]]::new()

foreach ($boardDirectory in $boardDirectories) {
    foreach ($relativeDirectory in @('driverlib', 'device_support')) {
        $directory = Join-Path $boardDirectory.FullName $relativeDirectory
        if (-not (Test-Path -LiteralPath $directory -PathType Container)) {
            continue
        }

        $resolvedDirectory = (Resolve-Path -LiteralPath $directory).Path
        if (-not $resolvedDirectory.StartsWith(
            $projectRoot + [System.IO.Path]::DirectorySeparatorChar,
            [System.StringComparison]::OrdinalIgnoreCase
        )) {
            throw "Refusing to clean a directory outside the LaunchPad project: $resolvedDirectory"
        }

        foreach ($header in Get-ChildItem -LiteralPath $resolvedDirectory -Recurse -File -Filter '*.h') {
            $filesToRemove.Add($header)
        }
    }

    foreach ($library in Get-ChildItem -LiteralPath $boardDirectory.FullName -Recurse -File -Filter 'driverlib*.lib') {
        $filesToRemove.Add($library)
    }

    foreach ($relativeDirectory in @('driverlib/ccs', 'driverlib/inc')) {
        $directory = Join-Path $boardDirectory.FullName $relativeDirectory
        if (Test-Path -LiteralPath $directory -PathType Container) {
            $resolvedDirectory = (Resolve-Path -LiteralPath $directory).Path
            if (-not $resolvedDirectory.StartsWith(
                $projectRoot + [System.IO.Path]::DirectorySeparatorChar,
                [System.StringComparison]::OrdinalIgnoreCase
            )) {
                throw "Refusing to clean a directory outside the LaunchPad project: $resolvedDirectory"
            }
            $directoriesToRemove.Add((Get-Item -LiteralPath $resolvedDirectory))
        }
    }
}

$uniqueFiles = @($filesToRemove | Sort-Object FullName -Unique)
$headerCount = @($uniqueFiles | Where-Object Extension -EQ '.h').Count
$libraryCount = @($uniqueFiles | Where-Object Extension -EQ '.lib').Count
Write-Host "Removing $headerCount copied C2000Ware headers and $libraryCount local DriverLib archives."

foreach ($file in $uniqueFiles) {
    Remove-Item -LiteralPath $file.FullName -Force
}

foreach ($directory in @($directoriesToRemove | Sort-Object FullName -Unique)) {
    Remove-Item -LiteralPath $directory.FullName -Recurse -Force
}

Write-Host 'Local C2000Ware header, DriverLib archive and DriverLib project cleanup complete.'

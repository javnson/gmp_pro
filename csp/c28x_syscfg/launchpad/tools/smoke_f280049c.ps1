param(
    [string]$Port = "COM5",
    [int]$BaudRate = 115200
)

$ErrorActionPreference = "Stop"
$projectRoot = Split-Path -Parent $PSScriptRoot
$repoRoot = Resolve-Path (Join-Path $projectRoot "..\..\..")
$smokeTest = Join-Path $repoRoot "tools\gmp_pil_server\f280049_dl_dbger\smoke_test.py"

python $smokeTest --baudrate $BaudRate --port $Port
if ($LASTEXITCODE -ne 0) {
    throw "F280049C Data Link smoke test failed with exit code $LASTEXITCODE"
}

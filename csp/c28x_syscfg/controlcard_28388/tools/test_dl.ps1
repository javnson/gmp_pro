param(
    [ValidateSet("Both", "Serial", "Ethernet")]
    [string]$Link = "Both",
    [ValidateSet("Tcp", "Udp")]
    [string]$EthernetProtocol = "Tcp",
    [string]$SerialPort = "COM74",
    [int]$BaudRate = 115200,
    [string]$HostAddress = "192.168.137.2",
    [int]$NetworkPort = 0,
    [switch]$CaptureScope
)

$ErrorActionPreference = "Stop"
$targetRoot = Split-Path -Parent $PSScriptRoot
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $targetRoot "..\..\..")).Path
$studio = Join-Path $repositoryRoot "tools\gmp_datalink\datalink_studio"
$python = Join-Path $repositoryRoot "bin\python\python.exe"
if (-not (Test-Path -LiteralPath $python -PathType Leaf)) {
    $python = (Get-Command python -ErrorAction Stop).Source
}
$env:GMP_PRO_LOCATION = $repositoryRoot
$captureArguments = if ($CaptureScope) { @() } else { @("--skip-capture") }

Push-Location $studio
try {
    if ($Link -in @("Both", "Serial")) {
        $serialArguments = @(
            "-m", "apis.examples.ai_debug_session",
            "--protocol", "serial",
            "--port", $SerialPort,
            "--baudrate", $BaudRate
        ) + $captureArguments
        & $python @serialArguments
        if ($LASTEXITCODE -ne 0) {
            throw "Serial DL verification failed with exit code $LASTEXITCODE"
        }
    }

    if ($Link -in @("Both", "Ethernet")) {
        if ($NetworkPort -eq 0) {
            $NetworkPort = if ($EthernetProtocol -eq "Tcp") { 50001 } else { 50002 }
        }
        $protocolName = $EthernetProtocol.ToLowerInvariant()
        $networkArguments = @(
            "-m", "apis.examples.ai_debug_session",
            "--protocol", $protocolName,
            "--host", $HostAddress,
            "--network-port", $NetworkPort
        ) + $captureArguments
        & $python @networkArguments
        if ($LASTEXITCODE -ne 0) {
            throw "Ethernet DL verification failed with exit code $LASTEXITCODE"
        }
    }
}
finally {
    Pop-Location
}

Write-Host "Requested GMP DL links passed discovery and readback."

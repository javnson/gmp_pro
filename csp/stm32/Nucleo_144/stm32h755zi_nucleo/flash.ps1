[CmdletBinding()]
param(
    [string]$Programmer = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    [string]$Cm7Image = (Join-Path $PSScriptRoot "build\release\stm32h755zi_nucleo_cm7.hex"),
    [string]$Cm4Image = (Join-Path $PSScriptRoot "build\release\stm32h755zi_nucleo_cm4.hex")
)
$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $Programmer)) {
    throw "STM32CubeProgrammer CLI not found: $Programmer"
}
if (-not (Test-Path -LiteralPath $Cm7Image)) {
    throw "CM7 firmware image not found: $Cm7Image"
}
if (-not (Test-Path -LiteralPath $Cm4Image)) {
    throw "CM4 firmware image not found: $Cm4Image"
}

# Keep the device under reset while both banks are updated.  Reset only after
# the CM4 image is present so the CubeMX HSEM boot handshake can start both cores.
& $Programmer -c port=SWD mode=UR reset=HWrst -w $Cm7Image -v
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $Programmer -c port=SWD mode=UR reset=HWrst -w $Cm4Image -v -rst
exit $LASTEXITCODE

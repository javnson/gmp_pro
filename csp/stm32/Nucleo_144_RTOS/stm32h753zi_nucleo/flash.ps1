[CmdletBinding()]
param(
    [string]$Programmer = "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    [string]$Image = (Join-Path $PSScriptRoot "build\release\stm32h753zi_nucleo_rtos.hex")
)

$ErrorActionPreference = "Stop"
if (-not (Test-Path -LiteralPath $Programmer)) {
    throw "STM32CubeProgrammer CLI not found: $Programmer"
}
if (-not (Test-Path -LiteralPath $Image)) {
    throw "Firmware image not found: $Image"
}

& $Programmer -c port=SWD mode=UR reset=HWrst -w $Image -v -rst
exit $LASTEXITCODE

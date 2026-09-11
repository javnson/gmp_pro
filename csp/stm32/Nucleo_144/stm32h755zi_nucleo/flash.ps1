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

# Keep the device under reset while both banks are updated. Reset only after
# the CM4 image is present so neither core starts with a stale companion image.
& $Programmer -c port=SWD mode=UR reset=HWrst -w $Cm7Image -v
if ($LASTEXITCODE -eq 0) {
    & $Programmer -c port=SWD mode=UR reset=HWrst -w $Cm4Image -v -rst
    if ($LASTEXITCODE -eq 0) { exit 0 }
}

# CubeProgrammer can occasionally attach but fail to erase an H755 flash bank.
# OpenOCD uses the same ST-Link and provides a reliable dual-bank fallback.
$openOcd = Get-Command openocd -ErrorAction SilentlyContinue
if ($null -eq $openOcd) {
    throw "STM32CubeProgrammer failed and OpenOCD is not available for fallback"
}
$cm7OpenOcd = $Cm7Image.Replace("\", "/")
$cm4OpenOcd = $Cm4Image.Replace("\", "/")
$commands = "init; reset halt; " +
    "flash write_image erase {$cm7OpenOcd}; verify_image {$cm7OpenOcd}; " +
    "flash write_image erase {$cm4OpenOcd}; verify_image {$cm4OpenOcd}; " +
    "reset run; shutdown"
& $openOcd.Source -f board/st_nucleo_h745zi.cfg -c $commands
exit $LASTEXITCODE

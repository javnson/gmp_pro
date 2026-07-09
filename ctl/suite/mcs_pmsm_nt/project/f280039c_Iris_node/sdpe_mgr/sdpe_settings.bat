@echo off
rem SDPE project manager settings.
rem This file is intentionally project-local. Adjust paths for your project branch.

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_LIBRARY=%GMP_PRO_LOCATION%\tools\SDPE_v2\examples"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
set "SDPE_OUT=%~dp0..\xplt\sdpe_generated"
set "SDPE_INCLUDE_PREFIX=ctl/component"
exit /b 0

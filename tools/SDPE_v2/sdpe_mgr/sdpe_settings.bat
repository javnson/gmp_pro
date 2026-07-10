@echo off
rem SDPE built-in demo manager settings.
rem This manager is intentionally stored under %GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_mgr.

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_LIBRARY=%GMP_PRO_LOCATION%\tools\SDPE_v2\examples"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
set "SDPE_OUT=%~dp0build"
set "SDPE_INCLUDE_PREFIX=ctl/component"
exit /b 0

@echo off
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] GMP_PRO_LOCATION is not set.
    exit /b 1
)
set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
set "SDPE_OUT=%~dp0..\src"
exit /b 0

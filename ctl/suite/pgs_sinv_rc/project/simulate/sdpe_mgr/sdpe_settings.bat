@echo off
rem SDPE project manager settings.
rem This file is intentionally project-local. Adjust paths for your project branch.

if "%GMP_PRO_LOCATION%"=="" set "GMP_PRO_LOCATION=%~dp0..\..\..\..\..\.."

set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
set "SDPE_OUT=%~dp0."
exit /b 0

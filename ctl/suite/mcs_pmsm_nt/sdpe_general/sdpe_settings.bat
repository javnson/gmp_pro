@echo off
rem SDPE project manager settings.
rem This file is intentionally project-local. Adjust paths for your project branch.

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
rem Emit the shared header directly into src so every project can include it
rem through the existing common-source include path.
set "SDPE_OUT=%~dp0..\src"
exit /b 0

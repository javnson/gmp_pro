@echo off
rem Canonical SDPE project-manager settings distributed by GMP installation.

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "SDPE_REQUIREMENT=%~dp0sdpe_requirement.json"
for %%I in ("%~dp0.") do set "SDPE_MANAGER_NAME=%%~nxI"
if /I "%SDPE_MANAGER_NAME%"=="sdpe_general" (
    set "SDPE_OUT=%~dp0..\src"
    set "SDPE_MATLAB_OUT=%~dp0."
) else (
    set "SDPE_OUT=%~dp0."
    set "SDPE_MATLAB_OUT=%~dp0."
)
exit /b 0

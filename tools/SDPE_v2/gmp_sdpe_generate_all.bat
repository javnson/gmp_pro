@echo off
setlocal

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

cd /d "%~dp0"

set "SDPE_SETTINGS=%~1"
if "%SDPE_SETTINGS%"=="" set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"

echo [SDPE] Settings: %SDPE_SETTINGS%
echo [SDPE] Source  : %GMP_PRO_LOCATION%\ctl\hardware_preset\sdpe_src
echo [SDPE] Output  : %GMP_PRO_LOCATION%\ctl\hardware_preset

python "%~dp0sdpe.py" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 exit /b %errorlevel%

python "%~dp0sdpe.py" --settings "%SDPE_SETTINGS%" generate-global-hardware
exit /b %errorlevel%

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

set "SDPE_MODE=%~2"
if "%SDPE_MODE%"=="" set "SDPE_MODE=library"

echo [SDPE] Starting GUI with settings: %SDPE_SETTINGS%
echo [SDPE] Mode: %SDPE_MODE%
python "%~dp0gui_pyqt\sdpe_gui.py" --settings "%SDPE_SETTINGS%" --mode "%SDPE_MODE%"
exit /b %errorlevel%

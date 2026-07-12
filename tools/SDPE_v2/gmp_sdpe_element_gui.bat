@echo off
setlocal

cd /d "%~dp0"

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    exit /b 1
)

set "SDPE_SETTINGS=%~1"
if "%SDPE_SETTINGS%"=="" set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"

set "SDPE_MODE=%~2"
if "%SDPE_MODE%"=="" set "SDPE_MODE=library"

echo [SDPE] Starting GUI with settings: %SDPE_SETTINGS%
echo [SDPE] Mode: %SDPE_MODE%
python "%~dp0gui_pyqt\sdpe_gui.py" --settings "%SDPE_SETTINGS%" --mode "%SDPE_MODE%"
exit /b %errorlevel%

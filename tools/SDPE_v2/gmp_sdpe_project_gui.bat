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

set "SDPE_PROJECTS=%~2"
if "%SDPE_PROJECTS%"=="" set "SDPE_PROJECTS=examples\projects"

echo [SDPE] Starting project requirement GUI
echo [SDPE] Settings: %SDPE_SETTINGS%
echo [SDPE] Projects: %SDPE_PROJECTS%
python "%~dp0gui_pyqt\sdpe_gui.py" --settings "%SDPE_SETTINGS%" --mode project --projects "%SDPE_PROJECTS%"
exit /b %errorlevel%

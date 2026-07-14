@echo off
setlocal EnableDelayedExpansion

title SDPE Project Requirement Editor
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Starting project requirement GUI...
echo =======================================================
echo [SDPE] Settings   : %SDPE_SETTINGS%
echo [SDPE] Requirement: %SDPE_REQUIREMENT%
echo [SDPE] Output     : %SDPE_OUT%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\gui_pyqt\sdpe_gui.py" --settings "%SDPE_SETTINGS%" --mode project --projects "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE GUI exited abnormally. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)
exit /b 0

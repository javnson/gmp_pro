@echo off
setlocal

cd /d "%~dp0"

set "SDPE_LIBRARY=%~1"
if "%SDPE_LIBRARY%"=="" set "SDPE_LIBRARY=examples"

set "SDPE_MODE=%~2"
if "%SDPE_MODE%"=="" set "SDPE_MODE=library"

echo [SDPE] Starting GUI with library: %SDPE_LIBRARY%
echo [SDPE] Mode: %SDPE_MODE%
python "%~dp0gui_pyqt\sdpe_gui.py" --library "%SDPE_LIBRARY%" --mode "%SDPE_MODE%"
exit /b %errorlevel%

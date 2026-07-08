@echo off
setlocal

cd /d "%~dp0"

set "SDPE_LIBRARY=%~1"
if "%SDPE_LIBRARY%"=="" set "SDPE_LIBRARY=examples"

set "SDPE_PROJECTS=%~2"
if "%SDPE_PROJECTS%"=="" set "SDPE_PROJECTS=examples\projects"

echo [SDPE] Starting project requirement GUI
echo [SDPE] Library : %SDPE_LIBRARY%
echo [SDPE] Projects: %SDPE_PROJECTS%
python "%~dp0gui_pyqt\sdpe_gui.py" --library "%SDPE_LIBRARY%" --mode project --projects "%SDPE_PROJECTS%"
exit /b %errorlevel%

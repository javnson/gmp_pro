@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] The viewer must run from the completed GMP private environment.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    pause
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 (
    pause
    exit /b 1
)

set "GMP_PYTHON_EXE=%GMP_PRO_LOCATION%\bin\python\python.exe"
if /I not "%GMP_ENV_MODE%"=="virtual" goto :PRIVATE_PYTHON_MISSING
if not exist "%GMP_PYTHON_EXE%" goto :PRIVATE_PYTHON_MISSING

"%GMP_PYTHON_EXE%" -c "import numpy, pyqtgraph; from PyQt5 import QtCore, QtGui, QtWidgets"
if errorlevel 1 (
    echo [ERROR] GMP Result Viewer dependencies are incomplete.
    echo         Re-run install_gmp_virtual_env.bat or deploy_gmp_env.bat.
    pause
    exit /b 1
)

echo [GMP] Starting CCTL Result Viewer with private Python.
echo [GMP] Python: %GMP_PYTHON_EXE%
pushd "%~dp0"
if errorlevel 1 (
    echo [ERROR] Cannot enter the Result Viewer directory.
    pause
    exit /b 1
)
"%GMP_PYTHON_EXE%" "%~dp0result_viewer.py" %*
set "RESULT=%ERRORLEVEL%"
popd

if "%RESULT%"=="0" exit /b 0
echo [ERROR] GMP CCTL Result Viewer exited with code %RESULT%.
pause
exit /b %RESULT%

:PRIVATE_PYTHON_MISSING
echo [ERROR] The completed GMP private Python environment is required.
echo         Expected: %GMP_PYTHON_EXE%
echo         Run install_gmp_virtual_env.bat or deploy_gmp_env.bat first.
pause
exit /b 1

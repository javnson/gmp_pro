@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] CCTL Studio runs in the completed GMP private environment.
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

set "GMP_CCTL_STUDIO_PYTHON=%GMP_PRO_LOCATION%\bin\python\python.exe"
if /I not "%GMP_ENV_MODE%"=="virtual" goto :PRIVATE_PYTHON_MISSING
if not exist "%GMP_CCTL_STUDIO_PYTHON%" goto :PRIVATE_PYTHON_MISSING

"%GMP_CCTL_STUDIO_PYTHON%" -c "from PyQt5 import QtCore; assert QtCore.QT_VERSION_STR.startswith('5.')"
if errorlevel 1 (
    echo [ERROR] The GMP private Python does not provide PyQt5 / Qt 5.
    pause
    exit /b 1
)

echo [GMP] Starting CCTL Studio.
pushd "%~dp0"
if errorlevel 1 (
    echo [ERROR] Cannot enter the CCTL Studio directory.
    pause
    exit /b 1
)
"%GMP_CCTL_STUDIO_PYTHON%" "%~dp0cctl_core\qt_studio.py" %*
set "GMP_CCTL_STUDIO_RESULT=%ERRORLEVEL%"
popd

if "%GMP_CCTL_STUDIO_RESULT%"=="0" (
    echo [GMP] CCTL Studio closed successfully.
    pause
    exit /b 0
)
echo [ERROR] GMP CCTL Studio exited with code %GMP_CCTL_STUDIO_RESULT%.
pause
exit /b %GMP_CCTL_STUDIO_RESULT%

:PRIVATE_PYTHON_MISSING
echo [ERROR] The completed GMP private Python environment is required.
echo         Expected: %GMP_CCTL_STUDIO_PYTHON%
echo         Run install_gmp_virtual_env.bat or
echo         tools\gmp_installer\utilities\deploy_gmp_env.bat first.
pause
exit /b 1

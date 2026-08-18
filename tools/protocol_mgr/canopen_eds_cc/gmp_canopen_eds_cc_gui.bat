@echo off
setlocal

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Please launch from a GMP installer-enabled environment.
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 (
    exit /b 1
)

cd /d "%~dp0"

set "PROJECT=%~1"
if "%PROJECT%"=="" (
    python "%~dp0canopen_eds_cc_gui.py"
) else (
    python "%~dp0canopen_eds_cc_gui.py" --project "%PROJECT%"
)

exit /b %errorlevel%

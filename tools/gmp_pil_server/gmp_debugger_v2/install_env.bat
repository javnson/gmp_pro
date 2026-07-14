@echo off
setlocal
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
set "GMP_ROOT=%GMP_PRO_LOCATION%"

if not exist "%GMP_ROOT%\bin\python\python.exe" (
    echo [ERROR] GMP private Python is not installed.
    echo         Run "%GMP_ROOT%\install_gmp_virtual_env.bat" first.
    exit /b 1
)

call "%GMP_ROOT%\tools\gmp_installer\activate_env.bat"
if errorlevel 1 exit /b %ERRORLEVEL%

python -c "import PyQt5, serial, pyqtgraph, numpy"
if errorlevel 1 (
    echo [ERROR] Debugger dependencies are incomplete. Re-run install_gmp.bat.
    exit /b 1
)

echo [OK] GMP debugger dependencies are provided by the private environment.
exit /b 0

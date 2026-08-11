@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

python -c "import PyQt5, serial, pyqtgraph, numpy"
if errorlevel 1 (
    echo [ERROR] Debugger dependencies are incomplete.
    echo         Re-run install_gmp.bat or install_gmp_virtual_env.bat.
    exit /b 1
)

echo [OK] GMP debugger dependencies are available in the %GMP_ENV_MODE% environment.
exit /b 0

@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Dependencies are owned by the GMP installer, not by this tool.
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

echo [OK] GMP Debugger u8 dependencies are available in the %GMP_ENV_MODE% environment.
exit /b 0

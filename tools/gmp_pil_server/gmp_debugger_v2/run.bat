@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
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

python -c "import PyQt5, serial, pyqtgraph, numpy"
if errorlevel 1 (
    echo [ERROR] GMP Debugger v2 dependencies are incomplete.
    echo         Re-run install_gmp.bat or install_gmp_virtual_env.bat.
    pause
    exit /b 1
)

echo [GMP] Starting GMP Debugger v2 with the %GMP_ENV_MODE% environment.
pushd "%~dp0"
if errorlevel 1 exit /b 1
python "%~dp0main.py" %*
set "RESULT=%ERRORLEVEL%"
popd

if "%RESULT%"=="0" exit /b 0
echo [ERROR] GMP Debugger v2 exited with code %RESULT%.
pause
exit /b %RESULT%

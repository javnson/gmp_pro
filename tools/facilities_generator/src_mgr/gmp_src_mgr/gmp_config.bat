@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

title GMP Framework Configurator
echo =======================================================
echo [GMP] Starting Framework Configurator (GUI)...
echo =======================================================

:: Change to correct position, where the script's location
cd /d "%~dp0"

:: Launch Configurator GUI
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_user_gui_v10.py"

:: Error handling
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Framework Configurator exited abnormally. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

exit /b 0

@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

title GMP Header Tree Generator
echo =======================================================
echo [GMP] Generating/Syncing header file tree (Header Mirror Mode)...
echo =======================================================

:: Change to correct position, where the script's location
cd /d "%~dp0"

:: Execute header file sync script
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py"
set "RESULT=%ERRORLEVEL%"

:: Error handling
if not "%RESULT%"=="0" (
    echo.
    echo [ERROR] Header tree generation failed. Process terminated!
    exit /b %RESULT%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] Header file tree deployed successfully!
echo =======================================================
exit /b 0

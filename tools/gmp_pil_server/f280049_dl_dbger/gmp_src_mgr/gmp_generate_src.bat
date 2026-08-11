@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

title GMP Source Code Generator
echo =======================================================
echo [GMP] Generating/Syncing source files (Source Flatten Mode)...
echo =======================================================

:: Change to correct position, where the script's location
cd /d "%~dp0"

:: Execute source file sync script
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_src_v3.py"
set "RESULT=%ERRORLEVEL%"

:: Error handling
if not "%RESULT%"=="0" (
    echo.
    echo [ERROR] Source file generation failed. Process terminated!
    exit /b %RESULT%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] Source files deployed successfully!
echo =======================================================
exit /b 0

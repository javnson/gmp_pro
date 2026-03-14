@echo off
setlocal EnableDelayedExpansion

title GMP Header Tree Generator
echo =======================================================
echo [GMP] Generating/Syncing header file tree (Header Mirror Mode)...
echo =======================================================

:: Check environment variable
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the root directory of your GMP core library.
    pause
    exit /b 1
)

:: Execute header file sync script
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py"

:: Error handling
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Header tree generation failed. Process terminated!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] Header file tree deployed successfully!
echo =======================================================
exit /b 0
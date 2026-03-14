@echo off
setlocal EnableDelayedExpansion

title GMP Source Code Generator
echo =======================================================
echo [GMP] Generating/Syncing source files (Source Flatten Mode)...
echo =======================================================

:: Check environment variable
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the root directory of your GMP core library.
    pause
    exit /b 1
)

:: Execute source file sync script
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_src_v3.py"

:: Error handling
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Source file generation failed. Process terminated!
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo 🎉 [SUCCESS] Source files deployed successfully!
echo =======================================================
exit /b 0
@echo off
setlocal EnableDelayedExpansion

title GMP Framework Asset Manager (Developer)
echo =======================================================
echo ⚙️ [GMP Dev Tool] Starting Core Framework Asset Manager...
echo =======================================================

:: 1. Check environment variable
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the root directory of your GMP core library.
    pause
    exit /b 1
)

:: 2. Launch Developer GUI (Framework Asset Manager)
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_dev_gui_v17.py"

:: 3. Error handling
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Asset Manager exited abnormally. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

:: Close window directly on normal exit
exit /b 0
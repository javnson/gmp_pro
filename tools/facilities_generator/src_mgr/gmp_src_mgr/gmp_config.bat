@echo off
setlocal EnableDelayedExpansion

title GMP Framework Configurator
echo =======================================================
echo [GMP] Starting Framework Configurator (GUI)...
echo =======================================================

:: Check environment variable
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the root directory of your GMP core library.
    pause
    exit /b 1
)

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
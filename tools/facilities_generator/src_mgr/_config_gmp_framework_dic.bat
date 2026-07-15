@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

title GMP Framework Asset Manager (Developer)
echo =======================================================
echo ⚙️ [GMP Dev Tool] Starting Core Framework Asset Manager...
echo =======================================================

:: 1. Launch Developer GUI (Framework Asset Manager)
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_dev_gui_v17.py"

:: 2. Error handling
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] Asset Manager exited abnormally. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

:: Close window directly on normal exit
exit /b 0

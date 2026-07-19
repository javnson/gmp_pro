@echo off
setlocal
if defined GMP_PRO_LOCATION (
    call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
    if errorlevel 1 goto :failed
)
set "PYTHONPATH=%~dp0host;%PYTHONPATH%"
python -m gmp_router.service %*
if errorlevel 1 goto :failed
echo.
echo GMP peripheral router service stopped.
pause
exit /b 0

:failed
echo.
echo [ERROR] GMP peripheral router service failed.
pause
exit /b 1

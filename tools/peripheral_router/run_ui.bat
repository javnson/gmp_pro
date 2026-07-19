@echo off
setlocal
if defined GMP_PRO_LOCATION (
    call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
    if errorlevel 1 goto :failed
)
python "%~dp0ui\main.py" %*
if errorlevel 1 goto :failed
echo.
echo GMP peripheral router UI closed.
pause
exit /b 0

:failed
echo.
echo [ERROR] GMP peripheral router UI failed.
pause
exit /b 1

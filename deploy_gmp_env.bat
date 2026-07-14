@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    exit /b 1
)

setlocal
call "%GMP_PRO_LOCATION%\tools\gmp_installer\deploy_portable.bat" %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo.
    echo [GMP] Portable environment deployment failed with exit code %RESULT%.
)
exit /b %RESULT%

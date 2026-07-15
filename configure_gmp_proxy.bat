@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    pause
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_private_proxy.bat"
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo [GMP] GMP private proxy configuration failed with exit code %RESULT%.
    pause
)
exit /b %RESULT%

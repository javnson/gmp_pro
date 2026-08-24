@echo off
call "%~dp0..\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_private_proxy.bat"
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" (
    echo [GMP] GMP private proxy configuration failed with exit code %RESULT%.
)
if "%RESULT%"=="0" (
    echo [GMP] GMP private proxy configuration completed successfully.
)
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
exit /b %RESULT%

@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 (
    echo [GMP] GMP_PRO_LOCATION registration failed.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

setlocal
set "GMP_INSTALLER_PROMPT_FOR_PROXY=1"
call "%GMP_PRO_LOCATION%\tools\gmp_installer\install_online.bat" %*
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" (
    echo.
    echo [GMP] Private environment installation completed successfully.
) else (
    echo.
    echo [GMP] Private environment installation failed with exit code %RESULT%.
)
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
exit /b %RESULT%

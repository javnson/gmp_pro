@echo off
setlocal EnableExtensions DisableDelayedExpansion
call "%~dp0tools\gmp_installer\install_online.bat" %*
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" echo [GMP] Installer exited with code %RESULT%.
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
exit /b %RESULT%

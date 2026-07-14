@echo off
if not defined GMP_PRO_LOCATION call "%~dp0register_gmp_root.bat"
if errorlevel 1 exit /b 1
call "%GMP_PRO_LOCATION%\tools\gmp_installer\install_system.bat" %*
exit /b %ERRORLEVEL%

@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat"
if errorlevel 1 exit /b 1
set "GMP_ACTIVATE=%GMP_PRO_LOCATION%\tools\gmp_installer\activate_env.bat"

if "%~1"=="" goto :INTERACTIVE

call "%GMP_ACTIVATE%"
if errorlevel 1 exit /b 1
%*
exit /b %ERRORLEVEL%

:INTERACTIVE
start "GMP Pro Development Environment" "%ComSpec%" /d /k call "%GMP_ACTIVATE%"
exit /b 0

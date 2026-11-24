@echo off
call "%~dp0tools\gmp_installer\register_gmp_root.bat" --process-only
if errorlevel 1 exit /b 1
set "GMP_ACTIVATE=%GMP_PRO_LOCATION%\tools\gmp_installer\activate_env.bat"

if "%~1"=="" goto :INTERACTIVE

call "%GMP_ACTIVATE%"
if errorlevel 1 exit /b 1
%*
exit /b %ERRORLEVEL%

:INTERACTIVE
if not exist "%GMP_PRO_LOCATION%\bin\gmp_virtual_env_installed.flag" (
    echo [ERROR] GMP virtual setup has not completed. Run install_gmp_virtual_env.bat first.
    exit /b 1
)
start "GMP Pro Development Environment" "%ComSpec%" /d /k call "%GMP_ACTIVATE%"
exit /b %ERRORLEVEL%

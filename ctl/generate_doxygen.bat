@echo off
setlocal

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

cd /d "%~dp0"
doxygen doxyfile.config
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" echo [ERROR] CTL documentation generation failed with exit code %RESULT%.
pause
exit /b %RESULT%

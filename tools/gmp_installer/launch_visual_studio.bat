@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\activate_env.bat"
if errorlevel 1 exit /b 1

where devenv.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Visual Studio devenv.exe was not found after GMP activation.
    exit /b 1
)

if "%~1"=="" (
    start "GMP Visual Studio" devenv.exe
) else (
    start "GMP Visual Studio" devenv.exe %*
)
exit /b %ERRORLEVEL%

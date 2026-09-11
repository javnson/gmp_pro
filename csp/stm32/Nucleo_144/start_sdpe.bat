@echo off
setlocal
title GMP Nucleo-144 SDPE Project Editor

for %%I in ("%~dp0..\..\..") do set "GMP_PRO_LOCATION=%%~fI"
set "SDPE_SETTINGS=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json"
set "SDPE_LAUNCHER=%GMP_PRO_LOCATION%\tools\SDPE_v2\gmp_sdpe_project_gui.bat"

if not exist "%SDPE_LAUNCHER%" (
    echo [ERROR] SDPE launcher was not found: %SDPE_LAUNCHER%
    pause
    exit /b 1
)

echo [SDPE] Repository: %GMP_PRO_LOCATION%
echo [SDPE] Nucleo projects: %~dp0
call "%SDPE_LAUNCHER%" "%SDPE_SETTINGS%" "%~dp0"
set "SDPE_EXIT_CODE=%ERRORLEVEL%"

if not "%SDPE_EXIT_CODE%"=="0" (
    echo.
    echo [ERROR] SDPE exited with code %SDPE_EXIT_CODE%.
) else (
    echo.
    echo [OK] SDPE closed normally.
)
pause
exit /b %SDPE_EXIT_CODE%



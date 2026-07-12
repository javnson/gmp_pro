@echo off
setlocal EnableDelayedExpansion

title SDPE Demo Project Validator
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Validating demo SDPE project...
echo =======================================================

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" inspect-project "%SDPE_REQUIREMENT%"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE demo project requirement is readable.
echo =======================================================
exit /b 0

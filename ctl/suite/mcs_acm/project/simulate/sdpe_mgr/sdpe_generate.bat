@echo off
setlocal EnableDelayedExpansion

title SDPE Project Header Generator
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Generating project SDPE headers...
echo =======================================================
echo [SDPE] Settings   : %SDPE_SETTINGS%
echo [SDPE] Requirement: %SDPE_REQUIREMENT%
echo [SDPE] Output     : %SDPE_OUT%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-local "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE project generation failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE project headers generated successfully.
echo =======================================================
exit /b 0

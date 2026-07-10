@echo off
setlocal EnableDelayedExpansion

title SDPE Demo Project Header Generator
cd /d "%~dp0"
call "%~dp0sdpe_settings.bat"
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

echo =======================================================
echo [SDPE] Generating demo SDPE headers...
echo =======================================================
echo [SDPE] Library    : %SDPE_LIBRARY%
echo [SDPE] Requirement: %SDPE_REQUIREMENT%
echo [SDPE] Output     : %SDPE_OUT%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" validate
if errorlevel 1 (
    pause
    exit /b %errorlevel%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --library "%SDPE_LIBRARY%" generate-project "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%" --include-prefix "%SDPE_INCLUDE_PREFIX%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE demo generation failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE demo headers generated successfully.
echo =======================================================
exit /b 0

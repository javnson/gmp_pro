@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

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

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-local "%SDPE_REQUIREMENT%" --project-dir "%~dp0." --out "%SDPE_OUT%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE project generation failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-matlab-local "%SDPE_REQUIREMENT%" --project-dir "%~dp0." --out "%SDPE_MATLAB_OUT%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE MATLAB initialization generation failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo [SUCCESS] SDPE project headers generated successfully.
echo =======================================================
exit /b 0

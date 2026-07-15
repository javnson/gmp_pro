@echo off
setlocal EnableDelayedExpansion

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

title SDPE Project Manager Deployer

set "SDPE_PROJECT_DIR=%~1"
if "%SDPE_PROJECT_DIR%"=="" set "SDPE_PROJECT_DIR=%CD%"

echo =======================================================
echo [SDPE] Deploying project-local sdpe_mgr...
echo =======================================================
echo [SDPE] Project dir: %SDPE_PROJECT_DIR%

python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" deploy-project "%SDPE_PROJECT_DIR%"
if errorlevel 1 (
    echo.
    echo [ERROR] SDPE manager deployment failed. Error code: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo =======================================================
echo [SUCCESS] sdpe_mgr deployed successfully.
echo =======================================================
exit /b 0

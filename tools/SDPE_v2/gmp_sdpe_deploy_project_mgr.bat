@echo off
setlocal EnableDelayedExpansion

title SDPE Project Manager Deployer

if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] Environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to the GMP repository root directory.
    pause
    exit /b 1
)

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

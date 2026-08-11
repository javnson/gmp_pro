@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

call "%~dp0ccs_product_installer\install_ccs_products.bat" %*
set "GMP_CCS_PRODUCT_RC=%errorlevel%"

if not "%GMP_CCS_PRODUCT_RC%"=="0" (
    echo [ERROR] GMP CCS Product registration failed.
) else (
    echo [OK] GMP C28x and C29x CCS Product metadata is ready.
)

pause
exit /b %GMP_CCS_PRODUCT_RC%

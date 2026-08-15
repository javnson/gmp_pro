@echo off
setlocal EnableExtensions

set "GMP_BUILD_CONFIGURATION=%~1"
set "GMP_PRODUCT_ROOT=%~2"
set "GMP_ENABLE_SDPE=%~3"
set "GMP_ENABLE_SRC_MGR=%~4"

call :is_enabled "%GMP_ENABLE_SDPE%" GMP_ENABLE_SDPE
call :is_enabled "%GMP_ENABLE_SRC_MGR%" GMP_ENABLE_SRC_MGR

echo [GMP pre-build] configuration=%GMP_BUILD_CONFIGURATION% SDPE=%GMP_ENABLE_SDPE% source-manager=%GMP_ENABLE_SRC_MGR%
if "%GMP_ENABLE_SDPE%%GMP_ENABLE_SRC_MGR%"=="00" exit /b 0

set "GMP_PROJECT_ROOT=%~dp0.."
for %%I in ("%GMP_PROJECT_ROOT%") do set "GMP_PROJECT_ROOT=%%~fI"
for /f "tokens=1 delims=_" %%I in ("%GMP_BUILD_CONFIGURATION%") do set "GMP_BOARD=%%I"

if not defined GMP_BUILD_CONFIGURATION (
    echo [ERROR] CCS did not provide the active configuration name.
    goto :preflight_failed
)
if not defined GMP_PRODUCT_ROOT (
    echo [ERROR] GMP-Core-C28x product root was not resolved by CCS.
    goto :preflight_failed
)
if not exist "%GMP_PRODUCT_ROOT%\.metadata\product.json" (
    echo [ERROR] Invalid GMP-Core-C28x product root: %GMP_PRODUCT_ROOT%
    goto :preflight_failed
)

for %%I in ("%GMP_PRODUCT_ROOT%\..\..") do set "GMP_PRO_LOCATION=%%~fI"

if not exist "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" (
    echo [ERROR] GMP repository root could not be derived from the registered product: %GMP_PRO_LOCATION%
    goto :preflight_failed
)
if not exist "%GMP_PROJECT_ROOT%\C2000Lib_%GMP_BOARD%" (
    echo [ERROR] Unknown LaunchPad configuration or missing board directory: %GMP_BUILD_CONFIGURATION%
    goto :preflight_failed
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 goto :preflight_failed

if "%GMP_ENABLE_SRC_MGR%"=="1" (
    echo [GMP pre-build] Regenerating project-local GMP headers and sources...
    call "%GMP_PROJECT_ROOT%\src\gmp_src_mgr\gmp_generate_all.bat"
    if errorlevel 1 goto :source_manager_failed
    if not exist "%GMP_PROJECT_ROOT%\src\gmp_src_mgr\gmp_inc\gmp_core.h" (
        echo [ERROR] GMP source-manager did not produce gmp_inc\gmp_core.h.
        goto :source_manager_failed
    )
    dir /b /s "%GMP_PROJECT_ROOT%\src\gmp_src_mgr\gmp_src\*.c" >nul 2>&1
    if errorlevel 1 (
        echo [ERROR] GMP source-manager did not produce any C source file.
        goto :source_manager_failed
    )
)

if "%GMP_ENABLE_SDPE%"=="1" (
    echo [GMP pre-build] Regenerating shared SDPE settings...
    call "%GMP_PROJECT_ROOT%\src\sdpe_mgr\sdpe_generate.bat"
    if errorlevel 1 goto :shared_sdpe_failed
    if not exist "%GMP_PROJECT_ROOT%\src\sdpe_mgr\ctrl_settings.h" (
        echo [ERROR] SDPE did not produce src\sdpe_mgr\ctrl_settings.h.
        goto :shared_sdpe_failed
    )

    echo [GMP pre-build] Regenerating %GMP_BOARD% board bindings...
    call "%GMP_PROJECT_ROOT%\C2000Lib_%GMP_BOARD%\sdpe\sdpe_generate.bat"
    if errorlevel 1 goto :board_sdpe_failed
    if not exist "%GMP_PROJECT_ROOT%\C2000Lib_%GMP_BOARD%\launchpad_board.h" (
        echo [ERROR] SDPE did not produce C2000Lib_%GMP_BOARD%\launchpad_board.h.
        goto :board_sdpe_failed
    )
)

echo [GMP pre-build] Generation completed successfully.
exit /b 0

:preflight_failed
if "%GMP_ENABLE_SRC_MGR%"=="1" del /q "%GMP_PROJECT_ROOT%\src\gmp_src_mgr\gmp_inc\gmp_core.h" >nul 2>&1
if "%GMP_ENABLE_SDPE%"=="1" del /q "%GMP_PROJECT_ROOT%\src\sdpe_mgr\ctrl_settings.h" >nul 2>&1
echo [ERROR] GMP pre-build prerequisites failed; stale entry headers removed.
exit /b 1

:source_manager_failed
rem CCS managed-build marks custom pre-build recipes as error-tolerant.  Remove
rem one generated entry header so compilation cannot silently use stale output.
del /q "%GMP_PROJECT_ROOT%\src\gmp_src_mgr\gmp_inc\gmp_core.h" >nul 2>&1
echo [ERROR] GMP source-manager generation failed; stale entry header removed.
exit /b 1

:shared_sdpe_failed
del /q "%GMP_PROJECT_ROOT%\src\sdpe_mgr\ctrl_settings.h" >nul 2>&1
echo [ERROR] Shared SDPE generation failed; stale settings header removed.
exit /b 1

:board_sdpe_failed
del /q "%GMP_PROJECT_ROOT%\C2000Lib_%GMP_BOARD%\launchpad_board.h" >nul 2>&1
echo [ERROR] Board SDPE generation failed; stale board header removed.
exit /b 1

:is_enabled
set "%~2=0"
if /I "%~1"=="1" set "%~2=1"
if /I "%~1"=="true" set "%~2=1"
if /I "%~1"=="yes" set "%~2=1"
if /I "%~1"=="on" set "%~2=1"
exit /b 0

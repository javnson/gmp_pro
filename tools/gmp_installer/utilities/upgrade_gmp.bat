@echo off

setlocal enabledelayedexpansion

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Resolve the GMP repository root from tools\gmp_installer\utilities.
for %%I in ("%~dp0..\..\..") do set "SCRIPT_DIR=%%~fI"

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Current path compliance check
set "HAS_SPACE=0"

if "%SCRIPT_DIR%" neq "%SCRIPT_DIR: =%" (
    set "HAS_SPACE=1"
)

if %HAS_SPACE% equ 1 (
    echo Spaces are present in the directory path.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
) else (
    echo .
)

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Check invalid characters

set "HAS_CHINESE=0"

for /l %%i in (0,1,%~z0-1) do (
    set "CHAR=!SCRIPT_DIR:~%%i,1!"
    if defined CHAR (
        set "ASCII=0"
        for /f "delims=" %%c in ("!CHAR!") do (
            for /f "delims=" %%d in ("!CHAR!") do (
                echo %%c | findstr /r "[^ -~]" >nul
                if !errorlevel! equ 0 (
                    set "ASCII=1"
                )
            )
        )
        if !ASCII! equ 1 (
            set "HAS_CHINESE=1"
            echo Chinese character found: !CHAR!
            goto :END_CHECK
        )
    )
)

:END_CHECK
if %HAS_CHINESE% equ 1 (
    echo Chinese characters are present in the directory path.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
) else (
    echo .
)

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Check if enviroment variable GMP_PRO_LOCATION is existed.
echo Checking if GMP_PRO_LOCATION environment variable exists...
set "ENV_VAR="
for /f "tokens=2 delims==" %%a in ('set GMP_PRO_LOCATION 2^>nul') do (
    set "ENV_VAR=%%a"
)

:: if no environment exists, create a new one,
:: or change it to current dir.
if defined ENV_VAR (
    echo Environment variable GMP_PRO_LOCATION already exists.
    echo Updating its value to: %SCRIPT_DIR%
    setx GMP_PRO_LOCATION "%SCRIPT_DIR%"
) else (
    echo Environment variable GMP_PRO_LOCATION does not exist.
    echo Creating it with value: %SCRIPT_DIR%
    setx GMP_PRO_LOCATION "%SCRIPT_DIR%"
)
if errorlevel 1 (
    echo [ERROR] Failed to register GMP_PRO_LOCATION.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

echo Environment variable GMP_PRO_LOCATION has been set to: %SCRIPT_DIR%

endlocal & set "SCRIPT_DIR=%SCRIPT_DIR%" & set "GMP_PRO_LOCATION=%SCRIPT_DIR%"

:: Start a new local environment
setlocal enabledelayedexpansion


:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Call facilities generator
cd /d "%SCRIPT_DIR%\tools\facilities_generator"
if errorlevel 1 (
    echo [ERROR] Cannot enter the facilities generator directory.
    if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
    exit /b 1
)

:: Generate the C28x and C29x CCS Product metadata from the shared registry.
python .\ccs_product_installer\ccs_product_installer.py --root "%GMP_PRO_LOCATION%"
if errorlevel 1 (
    set "RESULT=!ERRORLEVEL!"
    goto :FINISH
)

:: Validate the source-manager registry and its direct include dependencies.
python .\src_mgr\facility_dependency_audit.py --repo "%GMP_PRO_LOCATION%"
if errorlevel 1 (
    set "RESULT=!ERRORLEVEL!"
    goto :FINISH
)

cd src_mgr
if errorlevel 1 (
    set "RESULT=!ERRORLEVEL!"
    goto :FINISH
)

python .\framework_distribute_tools_v3.py
set "RESULT=!ERRORLEVEL!"

:FINISH
if "%RESULT%"=="0" (
    echo [GMP] Repository metadata and source-manager tools were refreshed successfully.
) else (
    echo [GMP] Repository refresh failed with exit code %RESULT%.
)
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause
endlocal & exit /b %RESULT%

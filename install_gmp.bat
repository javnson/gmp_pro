@echo off

setlocal enabledelayedexpansion

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Get current path 
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Current path compliance check
set "HAS_SPACE=0"

if "%SCRIPT_DIR%" neq "%SCRIPT_DIR: =%" (
    set "HAS_SPACE=1"
)

if %HAS_SPACE% equ 1 (
    echo Spaces are present in the directory path.
    pause
    exit
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
    pause
    exit
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

echo Environment variable GMP_PRO_LOCATION has been set to: %SCRIPT_DIR%

endlocal

:: Start a new local environment
setlocal enabledelayedexpansion


:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

call ".\tools\gmp_installer\scoop_installer.bat"

:: Check if Python has installed correctly.
where python >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo Python is not installed.
    echo You can find Python from the official website: https://www.python.org/downloads/
    exit /b
) else (
    echo Python is installed.
    echo Python Version:
    python --version
)

:: upgrade all submodules
::git submodule update --progress --init --recursive --force

::echo All the necessary git submodule has installed.


:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:: Call facilities generator
cd tools/facilities_generator

:: install fac for CCS
python ./gmp_fac_install_ccs_product.py

:: generate facility_cfg.json
python ./gmp_fac_generate_cfg_json.py

pause
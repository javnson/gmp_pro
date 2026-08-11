:: Source file: gmp_fac_generate_srcs_example2.bat

:: User should copy this file to your project folder

::
:: This tool will copy all the necessary files.
:: If differences exist, then copy, 
:: If file has existed, then keep it.
:: If file is not existed, than create it (copy it).
::


@echo off


:: Record Script Path
set "SCRIPT_DIR=%~dp0"
set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"

:: Check Environment variable
if "%GMP_PRO_LOCATION%"=="" (
    echo Error: GMP_PRO_LOCATION environment variable is not set.
    exit /b 1
)

:: call cmake generator
python "%GMP_PRO_LOCATION%/tools/facilities_generator/gmp_fac_generate_srcs.py" %SCRIPT_DIR%/gmp_facility_cfg.json .

echo source code generate done.


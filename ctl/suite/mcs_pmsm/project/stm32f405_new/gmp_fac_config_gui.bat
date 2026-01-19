:: Source file: gmp_fac_config_gui_example.bat

:: User should copy this file to your project folder


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
python "%GMP_PRO_LOCATION%/tools/facilities_generator/gmp_fac_config_gui.py" %SCRIPT_DIR%/gmp_facility_cfg.json

echo config script done.


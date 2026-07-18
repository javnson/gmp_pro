@echo off
setlocal

if not defined GMP_PRO_LOCATION (
    echo ERROR: GMP_PRO_LOCATION is not defined.
    echo Install or activate the GMP environment first.
    pause
    exit /b 1
)

python "%~dp0matlab_component_builder.py" gui
set "MCB_EXIT_CODE=%ERRORLEVEL%"
if not "%MCB_EXIT_CODE%"=="0" echo MATLAB Component Builder exited with code %MCB_EXIT_CODE%.

exit /b %MCB_EXIT_CODE%


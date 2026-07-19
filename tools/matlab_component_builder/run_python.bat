@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    call "%~dp0..\gmp_installer\register_gmp_root.bat"
    if errorlevel 1 exit /b 1
)

rem [GMP_ENV_GUARD] Prefer the completed GMP private environment when installed.
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

where python.exe >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python is unavailable in the selected GMP %GMP_ENV_MODE% environment.
    echo         Run install_gmp_virtual_env.bat for private mode or install_gmp.bat for system mode.
    exit /b 1
)

for /f "usebackq delims=" %%I in (`python -c "import sys; print(sys.executable)"`) do set "GMP_MCB_PYTHON=%%I"
python -c "import jinja2" >nul 2>&1
if errorlevel 1 (
    echo [ERROR] The selected GMP %GMP_ENV_MODE% Python is missing Jinja2.
    echo         Python: %GMP_MCB_PYTHON%
    echo         Do not install packages ad hoc from MATLAB.
    if /i "%GMP_ENV_MODE%"=="virtual" (
        echo         Repair it with install_gmp_virtual_env.bat.
    ) else (
        echo         Repair it with install_gmp.bat.
    )
    exit /b 1
)

echo [GMP MCB] Environment: %GMP_ENV_MODE%
echo [GMP MCB] Python     : %GMP_MCB_PYTHON%
python "%~dp0matlab_component_builder.py" %*
exit /b %ERRORLEVEL%

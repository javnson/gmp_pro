@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run register_gmp_root.bat first.
    exit /b 1
)
set "GMP_ROOT=%GMP_PRO_LOCATION%"
if not exist "%GMP_ROOT%\tools\gmp_installer\environment_manager.py" (
    echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository.
    exit /b 1
)
set "PYTHON_EXE=%GMP_ROOT%\bin\python\python.exe"

echo ========================================================
echo       GMP Pro Portable Environment - Local Deploy
echo ========================================================
echo Root: %GMP_ROOT%
echo.

del /q "%GMP_ROOT%\bin\gmp_virtual_env_installed.flag" >nul 2>&1

if not exist "%PYTHON_EXE%" (
    echo [ERROR] The copied bin folder is incomplete.
    echo         Missing: %PYTHON_EXE%
    echo         Copy a prepared bin folder into %GMP_ROOT% and retry.
    exit /b 1
)

call "%GMP_ROOT%\tools\gmp_installer\configure_proxy.bat"
if errorlevel 1 exit /b 1

"%PYTHON_EXE%" "%~dp0environment_manager.py" deploy %*
exit /b %ERRORLEVEL%

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
set "GMP_BIN=%GMP_ROOT%\bin"
set "PYTHON_HOME=%GMP_BIN%\python"
set "PYTHON_EXE=%PYTHON_HOME%\python.exe"

echo ========================================================
echo       GMP Pro Private Environment - Online Install
echo ========================================================
echo Root: %GMP_ROOT%
echo Bin : %GMP_BIN%
echo.

if /i "%~1"=="--plan" (
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0bootstrap_private_python.ps1" -Plan
    if errorlevel 1 exit /b 1
    echo Applications : portable Git, CMake, Ninja, Doxygen, Graphviz
    echo vcpkg         : repository-local bin\vcpkg and shared bin\vcpkg_installed
    echo Visual Studio : optional; enables suite simulation package restore
    echo Python packages: tools\gmp_installer\requirements-gmp.txt
    echo Repository    : CCS registration, source/SDPE tools, and project .gitignore distribution
    exit /b 0
)

del /q "%GMP_BIN%\gmp_virtual_env_installed.flag" >nul 2>&1

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_proxy.bat"
if errorlevel 1 exit /b 1

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0bootstrap_private_python.ps1"
if errorlevel 1 exit /b 1

if not exist "%PYTHON_EXE%" (
    echo [ERROR] Private Python executable was not created: %PYTHON_EXE%
    exit /b 1
)

set "PYTHONHOME=%PYTHON_HOME%"
set "PYTHONPATH="
set "PYTHONNOUSERSITE=1"
set "PYTHONUTF8=1"
"%PYTHON_EXE%" "%~dp0environment_manager.py" online %*
exit /b %ERRORLEVEL%

@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
set "PYTHON_EXE=%GMP_PRO_LOCATION%\bin\python\python.exe"
if not exist "%PYTHON_EXE%" (
    echo [ERROR] GMP private Python is missing. Run install_gmp_virtual_env.bat first.
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\configure_proxy.bat"
if errorlevel 1 exit /b 1

"%PYTHON_EXE%" "%GMP_PRO_LOCATION%\tools\gmp_installer\environment_manager.py" configure-proxy
exit /b %ERRORLEVEL%

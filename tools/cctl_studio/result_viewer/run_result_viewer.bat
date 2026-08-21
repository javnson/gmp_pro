@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 exit /b %ERRORLEVEL%

python "%GMP_PRO_LOCATION%\tools\cctl_studio\result_viewer\result_viewer.py" %*
exit /b %ERRORLEVEL%


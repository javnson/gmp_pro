@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 exit /b %ERRORLEVEL%

python -m py_compile "%~dp0result_data.py" "%~dp0result_viewer.py"
if errorlevel 1 exit /b %ERRORLEVEL%
python -m unittest discover -s "%~dp0tests" -v
exit /b %ERRORLEVEL%

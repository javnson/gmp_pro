@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Keep tests on the same private interpreter as the viewer.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

set "GMP_PYTHON_EXE=%GMP_PRO_LOCATION%\bin\python\python.exe"
if /I not "%GMP_ENV_MODE%"=="virtual" goto :PRIVATE_PYTHON_MISSING
if not exist "%GMP_PYTHON_EXE%" goto :PRIVATE_PYTHON_MISSING

"%GMP_PYTHON_EXE%" -m py_compile "%~dp0result_data.py" "%~dp0result_viewer.py"
if errorlevel 1 exit /b %ERRORLEVEL%
"%GMP_PYTHON_EXE%" -m unittest discover -s "%~dp0tests" -v
exit /b %ERRORLEVEL%

:PRIVATE_PYTHON_MISSING
echo [ERROR] Result Viewer tests require the completed GMP private Python environment.
echo         Expected: %GMP_PYTHON_EXE%
echo         Run install_gmp_virtual_env.bat or deploy_gmp_env.bat first.
exit /b 1

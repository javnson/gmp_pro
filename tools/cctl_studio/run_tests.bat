@echo off
setlocal EnableExtensions

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

set "GMP_CCTL_TEST_PYTHON=%GMP_PRO_LOCATION%\bin\python\python.exe"
if /I not "%GMP_ENV_MODE%"=="virtual" goto :PRIVATE_PYTHON_MISSING
if not exist "%GMP_CCTL_TEST_PYTHON%" goto :PRIVATE_PYTHON_MISSING

"%GMP_CCTL_TEST_PYTHON%" -m py_compile "%~dp0cctl_core\cctl_studio.py" "%~dp0cctl_core\editor_model.py" "%~dp0cctl_core\hierarchy_model.py" "%~dp0cctl_core\component_catalog.py" "%~dp0cctl_core\mna_export.py" "%~dp0cctl_core\qt_studio.py"
if errorlevel 1 exit /b %ERRORLEVEL%
"%GMP_CCTL_TEST_PYTHON%" -m unittest discover -s "%~dp0cctl_core\tests" -v
exit /b %ERRORLEVEL%

:PRIVATE_PYTHON_MISSING
echo [ERROR] CCTL Studio tests require the completed GMP private Python environment.
echo         Expected: %GMP_CCTL_TEST_PYTHON%
exit /b 1

@echo off
setlocal EnableExtensions

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    set "RESULT=1"
    goto :failed_with_result
)
set "MNA_TOOL_DIR=%GMP_PRO_LOCATION%\tools\cctl_studio\mna_solver"
if not exist "%MNA_TOOL_DIR%\mna_solver.py" (
    echo [ERROR] MNA solver was not found under GMP_PRO_LOCATION:
    echo         %MNA_TOOL_DIR%
    set "RESULT=1"
    goto :failed_with_result
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed

echo [1/2] Compiling MNA Python sources...
python -m py_compile "%MNA_TOOL_DIR%\mna_solver.py" "%~dp0validate_basic.py"
if errorlevel 1 goto :failed

echo [2/2] Building and simulating all basic netlists...
python "%~dp0validate_basic.py"
if errorlevel 1 goto :failed

echo.
echo All basic MNA tests passed.
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo Basic MNA tests failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

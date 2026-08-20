@echo off
setlocal EnableExtensions

set "NETLIST_FILE=SINV.CIR"
set "MATRIX_TOLERANCE=1E-12"
set "MATRIX_BACKEND=eigen"
rem Default netlist for this case. Change the line above when copying the case.

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if not "%~1"=="" if /I not "%~1"=="--no-pause" set "NETLIST_FILE=%~1"
if /I "%~2"=="--no-pause" set "NO_PAUSE=1"
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    set "RESULT=1"
    goto :failed_with_result
)
set "MNA_TOOL_DIR=%GMP_PRO_LOCATION%\tools\cctl_studio\mna_solver"
if not exist "%MNA_TOOL_DIR%\circuit_data.py" (
    echo [ERROR] MNA solver was not found under GMP_PRO_LOCATION:
    echo         %MNA_TOOL_DIR%
    set "RESULT=1"
    goto :failed_with_result
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed

set "CASE_DIR=%~dp0"
for %%I in ("%NETLIST_FILE%") do set "NETLIST_STEM=%%~nI"
set "NETLIST_PATH=%CASE_DIR%%NETLIST_FILE%"
set "GENERATED_DIR=%CASE_DIR%generated"
set "JSON_PATH=%GENERATED_DIR%\%NETLIST_STEM%.json"
if not exist "%NETLIST_PATH%" (
    echo [ERROR] Netlist does not exist: %NETLIST_PATH%
    set "RESULT=1"
    goto :failed_with_result
)
if not exist "%GENERATED_DIR%" mkdir "%GENERATED_DIR%"

echo [1/2] Exporting portable circuit data...
python "%MNA_TOOL_DIR%\circuit_data.py" export "%NETLIST_PATH%" "%JSON_PATH%" --normal-dt 100N --short-dt 1N --method backward_euler --matrix-tolerance "%MATRIX_TOLERANCE%"
if errorlevel 1 goto :failed
echo [2/2] Generating the %MATRIX_BACKEND% matrix-backend C++ calculation class...
python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%GENERATED_DIR%" --backend "%MATRIX_BACKEND%"
if errorlevel 1 goto :failed

echo.
echo Circuit data:       %JSON_PATH%
echo Generated code:     %GENERATED_DIR%
echo Handwritten tests:  %CASE_DIR%test
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo Circuit generation failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

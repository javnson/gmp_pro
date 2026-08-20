@echo off
setlocal EnableExtensions

set "NETLIST_FILE=PMSM.CIR"
set "MATRIX_TOLERANCE=1E-12"
if not defined MATRIX_BACKEND set "MATRIX_BACKEND=eigen"
rem MATRIX_BACKEND accepts eigen (default), fixed, or all.

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
if /I not "%MATRIX_BACKEND%"=="eigen" if /I not "%MATRIX_BACKEND%"=="fixed" if /I not "%MATRIX_BACKEND%"=="all" (
    echo [ERROR] MATRIX_BACKEND must be eigen, fixed, or all: %MATRIX_BACKEND%
    set "RESULT=1"
    goto :failed_with_result
)
set "TOTAL_STAGES=2"
if /I "%MATRIX_BACKEND%"=="all" set "TOTAL_STAGES=3"
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed

set "CASE_DIR=%~dp0"
for %%I in ("%NETLIST_FILE%") do set "NETLIST_STEM=%%~nI"
set "NETLIST_PATH=%CASE_DIR%%NETLIST_FILE%"
set "GENERATED_DIR=%CASE_DIR%generated"
set "FIXED_DIR=%GENERATED_DIR%\fixed"
set "EIGEN_DIR=%GENERATED_DIR%\eigen"
set "JSON_PATH=%GENERATED_DIR%\%NETLIST_STEM%.json"
if not exist "%NETLIST_PATH%" (
    echo [ERROR] Netlist does not exist: %NETLIST_PATH%
    set "RESULT=1"
    goto :failed_with_result
)
if not exist "%GENERATED_DIR%" mkdir "%GENERATED_DIR%"

echo [1/%TOTAL_STAGES%] Exporting portable PMSM-inverter circuit data...
python "%MNA_TOOL_DIR%\circuit_data.py" export "%NETLIST_PATH%" "%JSON_PATH%" --normal-dt 100N --short-dt 1N --method backward_euler --matrix-tolerance "%MATRIX_TOLERANCE%"
if errorlevel 1 goto :failed

if /I "%MATRIX_BACKEND%"=="all" goto :generate_all
if /I "%MATRIX_BACKEND%"=="fixed" (
    if not exist "%FIXED_DIR%" mkdir "%FIXED_DIR%"
    echo [2/2] Generating the optional fixed-matrix C++ calculation class...
    python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%FIXED_DIR%" --backend fixed
    if errorlevel 1 goto :failed
    set "GENERATED_SOLVER=%FIXED_DIR%"
) else (
    if not exist "%EIGEN_DIR%" mkdir "%EIGEN_DIR%"
    echo [2/2] Generating the Eigen C++ calculation class...
    python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%EIGEN_DIR%" --backend eigen
    if errorlevel 1 goto :failed
    set "GENERATED_SOLVER=%EIGEN_DIR%"
)
goto :generated

:generate_all
if not exist "%EIGEN_DIR%" mkdir "%EIGEN_DIR%"
if not exist "%FIXED_DIR%" mkdir "%FIXED_DIR%"
echo [2/3] Generating the Eigen C++ calculation class...
python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%EIGEN_DIR%" --backend eigen
if errorlevel 1 goto :failed
echo [3/3] Generating the optional fixed-matrix C++ calculation class...
python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%FIXED_DIR%" --backend fixed
if errorlevel 1 goto :failed
set "GENERATED_SOLVER=%EIGEN_DIR% and %FIXED_DIR%"

:generated
echo.
echo Circuit data:       %JSON_PATH%
echo Matrix backend:     %MATRIX_BACKEND%
echo Generated solver:   %GENERATED_SOLVER%
echo Handwritten tests:  %CASE_DIR%test
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo PMSM circuit generation failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

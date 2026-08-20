@echo off
setlocal EnableExtensions

rem Generate circuit data and Eigen C++ sources for a netlist beside this BAT.
set "NETLIST_FILE=%~1"
set "NO_PAUSE=0"
if /I "%~2"=="--no-pause" set "NO_PAUSE=1"

if not defined NETLIST_FILE (
    echo [ERROR] Usage: generate_code.bat ^<netlist.CIR^> [--no-pause]
    set "RESULT=1"
    goto :failed_with_result
)
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
set "JSON_PATH=%CASE_DIR%%NETLIST_STEM%.json"
set "CPP_DIR=%CASE_DIR%generated\cpp"

if not exist "%NETLIST_PATH%" (
    echo [ERROR] Netlist does not exist: %NETLIST_PATH%
    set "RESULT=1"
    goto :failed_with_result
)

echo [1/2] Exporting portable circuit data...
python "%MNA_TOOL_DIR%\circuit_data.py" export "%NETLIST_PATH%" "%JSON_PATH%" --normal-dt 100N --short-dt 1N --method backward_euler
if errorlevel 1 goto :failed

echo [2/2] Generating Eigen C++ sources and test bench...
python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%CPP_DIR%"
if errorlevel 1 goto :failed

echo.
echo Circuit data: %JSON_PATH%
echo C++ project:  %CPP_DIR%
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

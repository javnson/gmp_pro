@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.

set "SOLVER_DIR=%~dp0"
set "GMP_ROOT=%SOLVER_DIR%..\..\.."
for %%I in ("%GMP_ROOT%") do set "GMP_ROOT=%%~fI"
set "GMP_PRO_LOCATION=%GMP_ROOT%"
call "%GMP_ROOT%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 exit /b %ERRORLEVEL%
set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"

set "GENERATED_DIR=%SOLVER_DIR%\tb_buck\generated"
if not exist "%GENERATED_DIR%" mkdir "%GENERATED_DIR%"
if errorlevel 1 goto :failed

echo [1/2] Exporting portable Buck circuit data...
python "%SOLVER_DIR%circuit_data.py" export "%SOLVER_DIR%tb_buck\buck.CIR" "%GENERATED_DIR%\buck.json" --normal-dt 100N --short-dt 1N --method backward_euler
if errorlevel 1 goto :failed

echo [2/2] Generating the Eigen C++ class and test bench...
python "%SOLVER_DIR%cpp_codegen.py" "%GENERATED_DIR%\buck.json" "%GENERATED_DIR%" --class-name BuckCircuit
if errorlevel 1 goto :failed

echo.
echo Buck data and C++ files generated in:
echo   %GENERATED_DIR%
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
echo.
echo Buck generation failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

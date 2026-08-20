@echo off
setlocal EnableExtensions

rem Change only this variable when copying the BAT beside another netlist.
set "NETLIST_FILE=buck.CIR"

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
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
set "VSLANG=1033"

set "CASE_DIR=%~dp0"
for %%I in ("%NETLIST_FILE%") do set "NETLIST_STEM=%%~nI"
set "NETLIST_PATH=%CASE_DIR%%NETLIST_FILE%"
set "JSON_PATH=%CASE_DIR%%NETLIST_STEM%.json"
set "CPP_DIR=%CASE_DIR%generated\cpp"
set "BUILD_DIR=%TEMP%\gmp_mna_%NETLIST_STEM%_cpp_build"

if not exist "%NETLIST_PATH%" (
    echo [ERROR] Netlist does not exist: %NETLIST_PATH%
    set "RESULT=1"
    goto :failed_with_result
)

echo [1/5] Exporting portable circuit data...
python "%MNA_TOOL_DIR%\circuit_data.py" export "%NETLIST_PATH%" "%JSON_PATH%" --normal-dt 100N --short-dt 1N --method backward_euler
if errorlevel 1 goto :failed

echo [2/5] Generating Eigen C++ sources and test bench...
python "%MNA_TOOL_DIR%\cpp_codegen.py" "%JSON_PATH%" "%CPP_DIR%"
if errorlevel 1 goto :failed

if /I not "%GMP_ENV_MODE%"=="virtual" goto :system_environment
if not exist "%VCPKG_INSTALLED_DIR%\x64-windows\include\eigen3\Eigen\Dense" (
    echo [ERROR] Eigen3 is not installed in the GMP vcpkg shared tree.
    echo         Run repair_gmp_vcpkg.bat, then retry.
    set "RESULT=1"
    goto :failed_with_result
)
echo [3/5] Configuring generated test bench with GMP's private vcpkg...
cmake --fresh -S "%CPP_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%CMAKE_TOOLCHAIN_FILE%" -DVCPKG_INSTALLED_DIR="%VCPKG_INSTALLED_DIR%" -DVCPKG_MANIFEST_MODE=OFF
if errorlevel 1 goto :failed
goto :build

:system_environment
where vcpkg.exe >nul 2>nul
if errorlevel 1 (
    echo [ERROR] System vcpkg is unavailable. Run install_gmp.bat, then retry.
    set "RESULT=1"
    goto :failed_with_result
)
for /f "delims=" %%I in ('where vcpkg.exe') do if not defined SYSTEM_VCPKG_EXE set "SYSTEM_VCPKG_EXE=%%I"
for %%I in ("%SYSTEM_VCPKG_EXE%") do set "SYSTEM_VCPKG_ROOT=%%~dpI"
echo [3/5] Configuring generated test bench with system vcpkg...
cmake --fresh -S "%CPP_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%SYSTEM_VCPKG_ROOT%scripts\buildsystems\vcpkg.cmake"
if errorlevel 1 goto :failed

:build
echo [4/5] Compiling generated test bench...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [5/5] Running 50%% duty, 10 kHz, 50 ms simulation...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Circuit data: %JSON_PATH%
echo C++ project:  %CPP_DIR%
echo Build tree:   %BUILD_DIR%
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo Generated test bench failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

@echo off
setlocal EnableExtensions

rem [GMP_ENV_GUARD] Prefer the GMP virtual environment when installed.

set "SOLVER_DIR=%~dp0"
set "GMP_ROOT=%SOLVER_DIR%..\..\.."
for %%I in ("%GMP_ROOT%") do set "GMP_ROOT=%%~fI"
set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
set "GENERATED_DIR=%SOLVER_DIR%generated\buck"
set "BUILD_DIR=%TEMP%\gmp_mna_buck_cpp_build"

if not exist "%GENERATED_DIR%\buckcircuit.hpp" (
    echo [ERROR] Generated Buck files are missing. Run generate_buck.bat first.
    goto :failed
)

set "GMP_PRO_LOCATION=%GMP_ROOT%"
call "%GMP_ROOT%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed
set "VSLANG=1033"

if /I not "%GMP_ENV_MODE%"=="virtual" goto :system_environment
if exist "%VCPKG_INSTALLED_DIR%\x64-windows\include\eigen3\Eigen\Dense" goto :configure_private
echo [ERROR] Eigen3 is not installed in the GMP vcpkg shared tree.
echo         Run repair_gmp_vcpkg.bat, then retry.
goto :failed

:configure_private
echo [1/3] Configuring generated Buck test bench...
cmake -S "%GENERATED_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%CMAKE_TOOLCHAIN_FILE%" -DVCPKG_INSTALLED_DIR="%VCPKG_INSTALLED_DIR%" -DVCPKG_MANIFEST_MODE=OFF
if errorlevel 1 goto :failed
goto :build

:system_environment
where vcpkg.exe >nul 2>nul
if errorlevel 1 (
    echo [ERROR] System vcpkg is unavailable. Run install_gmp.bat, then retry.
    goto :failed
)
for /f "delims=" %%I in ('where vcpkg.exe') do if not defined SYSTEM_VCPKG_EXE set "SYSTEM_VCPKG_EXE=%%I"
for %%I in ("%SYSTEM_VCPKG_EXE%") do set "SYSTEM_VCPKG_ROOT=%%~dpI"
echo [1/3] Configuring generated Buck test bench with system vcpkg...
cmake -S "%GENERATED_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%SYSTEM_VCPKG_ROOT%scripts\buildsystems\vcpkg.cmake"
if errorlevel 1 goto :failed

:build
echo [2/3] Compiling generated Buck test bench...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [3/3] Running 50%% duty, 10 kHz, 50 ms simulation...
pushd "%BUILD_DIR%"
buckcircuit_testbench.exe
set "RESULT=%ERRORLEVEL%"
popd
if not "%RESULT%"=="0" goto :failed_with_result

echo.
echo Generated Buck C++ test bench passed.
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo Generated Buck C++ test bench failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

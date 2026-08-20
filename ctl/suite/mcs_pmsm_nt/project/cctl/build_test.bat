@echo off
setlocal EnableExtensions

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    set "RESULT=1"
    goto :failed_with_result
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed
set "VSLANG=1033"

for %%I in ("%~dp0.") do set "PROJECT_DIR=%%~fI"
set "SDPE_DIR=%PROJECT_DIR%\sdpe_mgr"
set "GMP_SRC_MGR=%PROJECT_DIR%\gmp_src_mgr"
set "HW_DIR=%PROJECT_DIR%\hw"
set "BUILD_DIR=%TEMP%\gmp_mcs_pmsm_nt_cctl_build"

echo [1/7] Generating target hardware and simulation macros with SDPE...
call "%SDPE_DIR%\sdpe_generate.bat"
if errorlevel 1 goto :failed

echo [2/7] Deploying the selected GMP, CCTL DSA, and CCTL CSP sources...
call "%GMP_SRC_MGR%\gmp_generate_src.bat"
if errorlevel 1 goto :failed

echo [3/7] Generating the project source-manager CMake integration...
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_generate_cmake.py" --workspace "%PROJECT_DIR%"
if errorlevel 1 goto :failed

echo [4/7] Regenerating fixed and Eigen main-circuit solvers...
call "%HW_DIR%\generate_code.bat" --no-pause
if errorlevel 1 goto :failed

if /I not "%GMP_ENV_MODE%"=="virtual" goto :system_environment
if not exist "%VCPKG_INSTALLED_DIR%\x64-windows\include\eigen3\Eigen\Dense" (
    echo [ERROR] Eigen3 is not installed in the GMP vcpkg shared tree.
    echo         Run repair_gmp_vcpkg.bat, then retry.
    set "RESULT=1"
    goto :failed_with_result
)
echo [5/7] Configuring with GMP's private vcpkg Eigen...
cmake --fresh -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%CMAKE_TOOLCHAIN_FILE%" -DVCPKG_INSTALLED_DIR="%VCPKG_INSTALLED_DIR%" -DVCPKG_MANIFEST_MODE=OFF
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
echo [5/7] Configuring with system vcpkg Eigen...
cmake --fresh -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%SYSTEM_VCPKG_ROOT%scripts\buildsystems\vcpkg.cmake"
if errorlevel 1 goto :failed

:build
echo [6/7] Compiling controller, peripherals, main circuit, motor, and CCTL CSP...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [7/7] Running fixed/Eigen 500:1 multirate closed-loop regressions...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Direct CCTL PMSM test passed.
echo Build tree: %BUILD_DIR%
echo Fixed CSV: %BUILD_DIR%\mcs_pmsm_nt_cctl_fixed.csv
echo Eigen CSV: %BUILD_DIR%\mcs_pmsm_nt_cctl_eigen.csv
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
set "RESULT=%ERRORLEVEL%"
if "%RESULT%"=="0" set "RESULT=1"
:failed_with_result
echo.
echo Direct CCTL PMSM test failed with exit code %RESULT%.
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%

@echo off
setlocal EnableExtensions

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if /I "%~2"=="--no-pause" set "NO_PAUSE=1"
set "BUILD_FIXED=0"
if /I "%~1"=="--with-fixed" set "BUILD_FIXED=1"
if /I "%~2"=="--with-fixed" set "BUILD_FIXED=1"
set "MATRIX_BACKEND=eigen"
set "CMAKE_FIXED_OPTION=OFF"
if "%BUILD_FIXED%"=="1" (
    set "MATRIX_BACKEND=all"
    set "CMAKE_FIXED_OPTION=ON"
)
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

echo [1/8] Generating target hardware and simulation macros with SDPE...
call "%SDPE_DIR%\sdpe_generate.bat"
if errorlevel 1 goto :failed

echo [2/8] Deploying the selected GMP, CCTL DSA, and CCTL CSP sources...
call "%GMP_SRC_MGR%\gmp_generate_src.bat"
if errorlevel 1 goto :failed

echo [3/8] Generating the project source-manager CMake integration...
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_generate_cmake.py" --workspace "%PROJECT_DIR%"
if errorlevel 1 goto :failed

echo [4/8] Regenerating the %MATRIX_BACKEND% main-circuit solver selection...
call "%HW_DIR%\generate_code.bat" --no-pause
if errorlevel 1 goto :failed

echo [5/8] Verifying generated ADC transfer gains against private SDPE hardware...
python "%HW_DIR%\validate_generated_model.py" "%HW_DIR%\generated\PMSM.json" "%SDPE_DIR%\private_hardware\inverter_3ph\mcs_pmsm_nt_cctl_inverter.json"
if errorlevel 1 goto :failed

if /I not "%GMP_ENV_MODE%"=="virtual" goto :system_environment
if not exist "%VCPKG_INSTALLED_DIR%\x64-windows\include\eigen3\Eigen\Dense" (
    echo [ERROR] Eigen3 is not installed in the GMP vcpkg shared tree.
    echo         Run repair_gmp_vcpkg.bat, then retry.
    set "RESULT=1"
    goto :failed_with_result
)
echo [6/8] Configuring with GMP's private vcpkg Eigen...
cmake --fresh -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%CMAKE_TOOLCHAIN_FILE%" -DVCPKG_INSTALLED_DIR="%VCPKG_INSTALLED_DIR%" -DVCPKG_MANIFEST_MODE=OFF -DCCTL_BUILD_FIXED_BACKEND=%CMAKE_FIXED_OPTION%
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
echo [6/8] Configuring with system vcpkg Eigen...
cmake --fresh -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="%SYSTEM_VCPKG_ROOT%scripts\buildsystems\vcpkg.cmake" -DCCTL_BUILD_FIXED_BACKEND=%CMAKE_FIXED_OPTION%
if errorlevel 1 goto :failed

:build
echo [7/8] Compiling controller, peripherals, main circuit, motor, and CCTL CSP...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [8/8] Running the selected closed-loop regression set...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Direct CCTL PMSM test passed.
echo Build tree: %BUILD_DIR%
echo Eigen CSV: %BUILD_DIR%\mcs_pmsm_nt_cctl.csv
if "%BUILD_FIXED%"=="1" echo Fixed CSV: %BUILD_DIR%\mcs_pmsm_nt_cctl_fixed.csv
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

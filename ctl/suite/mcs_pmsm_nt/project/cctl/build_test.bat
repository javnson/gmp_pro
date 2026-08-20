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

echo [4/7] Regenerating the project-local fixed-matrix main circuit...
call "%HW_DIR%\generate_code.bat" --no-pause
if errorlevel 1 goto :failed

echo [5/7] Configuring the direct CCTL simulation...
cmake --fresh -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 goto :failed

echo [6/7] Compiling controller, peripherals, main circuit, motor, and CCTL CSP...
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [7/7] Running the three-thread 500:1 multirate closed-loop test...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

echo.
echo Direct CCTL PMSM test passed.
echo Build tree: %BUILD_DIR%
echo Result CSV: %BUILD_DIR%\mcs_pmsm_nt_cctl.csv
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

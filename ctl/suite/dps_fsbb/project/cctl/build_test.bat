@echo off
setlocal EnableExtensions

set "NO_PAUSE=0"
if /I "%~1"=="--no-pause" set "NO_PAUSE=1"
if /I "%~2"=="--no-pause" set "NO_PAUSE=1"
set "BUILD_LEVEL=3"
if not "%~1"=="" if /I not "%~1"=="--no-pause" set "BUILD_LEVEL=%~1"
if not "%BUILD_LEVEL%"=="1" if not "%BUILD_LEVEL%"=="2" if not "%BUILD_LEVEL%"=="3" (
    echo Usage: build_test.bat [1^|2^|3] [--no-pause]
    goto :failed
)
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    goto :failed
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 goto :failed
set "VSLANG=1033"

for %%I in ("%~dp0.") do set "PROJECT_DIR=%%~fI"
set "SDPE=%GMP_PRO_LOCATION%\tools\SDPE_v2"
set "MGR=%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr"
set "BUILD_DIR=%GMP_PRO_LOCATION%\tmp\cctl\suite\dps_fsbb\level_%BUILD_LEVEL%"
set "TARGET_SDPE=%PROJECT_DIR%\sdpe_mgr"

echo [1/5] Generating the FSBB CCTL settings...
python "%SDPE%\sdpe.py" --settings "%SDPE%\sdpe_settings.json" generate-project-local "%TARGET_SDPE%\sdpe_requirement.json" --project-dir "%TARGET_SDPE%" --out "%TARGET_SDPE%"
if errorlevel 1 goto :failed

echo [2/5] Generating local GMP sources and CMake integration...
pushd "%PROJECT_DIR%\gmp_src_mgr"
if errorlevel 1 goto :failed
python "%MGR%\framework_sync_src_v3.py"
set "SYNC_RESULT=%ERRORLEVEL%"
popd
if not "%SYNC_RESULT%"=="0" goto :failed
python "%MGR%\framework_generate_cmake.py" --workspace "%PROJECT_DIR%"
if errorlevel 1 goto :failed

echo [3/5] Generating the FSBB switching circuit...
python "%PROJECT_DIR%\hw\generate_closed_loop.py" "%BUILD_DIR%\generated"
if errorlevel 1 goto :failed

echo [4/5] Configuring and building FSBB CCTL Level %BUILD_LEVEL%...
cmake -S "%PROJECT_DIR%" -B "%BUILD_DIR%" -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_TRY_COMPILE_CONFIGURATION=Release -DCMAKE_TOOLCHAIN_FILE="%GMP_PRO_LOCATION%\tools\gmp_installer\gmp_vcpkg_toolchain.cmake" -DFSBB_BUILD_LEVEL=%BUILD_LEVEL%
if errorlevel 1 goto :failed
cmake --build "%BUILD_DIR%" --config Release
if errorlevel 1 goto :failed

echo [5/5] Running the FSBB closed-loop simulation...
ctest --test-dir "%BUILD_DIR%" -C Release --output-on-failure
if errorlevel 1 goto :failed

echo Results: %BUILD_DIR%\fsbb_level_%BUILD_LEVEL%_circuit.csv and fsbb_level_%BUILD_LEVEL%_control.csv
if "%NO_PAUSE%"=="0" pause
exit /b 0

:failed
echo [ERROR] FSBB CCTL build or simulation failed.
if "%NO_PAUSE%"=="0" pause
exit /b 1

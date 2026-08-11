@echo off
setlocal EnableExtensions

for %%I in ("%~dp0..\..\..") do set "GMP_SIL_ROOT=%%~fI"
set "GMP_SIL_SOURCE=%~dp0."
set "GMP_SIL_BUILD=%~dp0build"
set "GMP_SIL_CONFIG=%~1"
if not defined GMP_SIL_CONFIG set "GMP_SIL_CONFIG=Release"

if not exist "%GMP_SIL_ROOT%\gmp_env.bat" (
    echo [GMP SIL] Repository environment entry was not found:
    echo   %GMP_SIL_ROOT%\gmp_env.bat
    exit /b 1
)

echo [GMP SIL] Configuring %GMP_SIL_CONFIG% in "%GMP_SIL_BUILD%".
call "%GMP_SIL_ROOT%\gmp_env.bat" cmake ^
    -S "%GMP_SIL_SOURCE%" ^
    -B "%GMP_SIL_BUILD%" ^
    -DCMAKE_TOOLCHAIN_FILE="%GMP_SIL_ROOT%\bin\vcpkg\scripts\buildsystems\vcpkg.cmake" ^
    -DGMP_SIL_HELPER_BUILD_TESTS=ON
if errorlevel 1 exit /b 1

call "%GMP_SIL_ROOT%\gmp_env.bat" cmake --build "%GMP_SIL_BUILD%" --config "%GMP_SIL_CONFIG%"
if errorlevel 1 exit /b 1

call "%GMP_SIL_ROOT%\gmp_env.bat" ctest --test-dir "%GMP_SIL_BUILD%" -C "%GMP_SIL_CONFIG%" --output-on-failure
if errorlevel 1 exit /b 1

echo [GMP SIL] Build and tests completed successfully.
exit /b 0

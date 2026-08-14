@echo off
setlocal

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

set "TEST_ROOT=%~dp0"
set "TEST_ROOT=%TEST_ROOT:~0,-1%"
set "BUILD_ROOT=%TEST_ROOT%\out\cmake"
cmake -S "%TEST_ROOT%" -B "%BUILD_ROOT%" -G Ninja ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -DCMAKE_TOOLCHAIN_FILE="%GMP_PRO_LOCATION%\bin\vcpkg\scripts\buildsystems\vcpkg.cmake" ^
    -DVCPKG_TARGET_TRIPLET=x64-windows ^
    -DVCPKG_MANIFEST_DIR="%GMP_PRO_LOCATION%\bin\cache\vcpkg-manifest" ^
    -DVCPKG_INSTALLED_DIR="%GMP_PRO_LOCATION%\bin\vcpkg_installed\x64-windows"
if errorlevel 1 exit /b 1

cmake --build "%BUILD_ROOT%"
if errorlevel 1 exit /b 1

ctest --test-dir "%BUILD_ROOT%" --output-on-failure
exit /b %ERRORLEVEL%

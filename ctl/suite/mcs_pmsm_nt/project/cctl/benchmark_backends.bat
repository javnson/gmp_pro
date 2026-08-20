@echo off
setlocal EnableExtensions

set "BENCHMARK_RUNS=3"
set "BENCHMARK_PRIORITY=realtime"
if not "%~1"=="" set "BENCHMARK_RUNS=%~1"
if not "%~2"=="" set "BENCHMARK_PRIORITY=%~2"
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)
call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat" >nul
if errorlevel 1 exit /b %ERRORLEVEL%

set "BUILD_DIR=%TEMP%\gmp_mcs_pmsm_nt_cctl_build"
if not exist "%BUILD_DIR%\mcs_pmsm_nt_cctl.exe" (
    echo [ERROR] Fixed executable was not found. Run build_test.bat first.
    exit /b 1
)
if not exist "%BUILD_DIR%\mcs_pmsm_nt_cctl_eigen.exe" (
    echo [ERROR] Eigen executable was not found. Run build_test.bat first.
    exit /b 1
)

python "%~dp0benchmark_backends.py" --build-dir "%BUILD_DIR%" --runs "%BENCHMARK_RUNS%" --priority "%BENCHMARK_PRIORITY%"
exit /b %ERRORLEVEL%

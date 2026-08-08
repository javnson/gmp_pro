@echo off
setlocal

rem [GMP_ENV_GUARD] Source generation requires an installed GMP repository.
if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Run a GMP installer first.
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

pushd "%~dp0"
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py"
set "RESULT=%ERRORLEVEL%"
popd
exit /b %RESULT%

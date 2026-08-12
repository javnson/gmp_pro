@echo off
setlocal

if not defined GMP_PRO_LOCATION (
    set "GMP_PRO_LOCATION=%~dp0..\..\..\.."
)

if not exist "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py" (
    echo [ERROR] GMP_PRO_LOCATION does not point to a GMP repository: "%GMP_PRO_LOCATION%"
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b 1

pushd "%~dp0"
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_inc_v3.py"
if errorlevel 1 goto :failed
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_sync_src_v3.py"
if errorlevel 1 goto :failed
popd

echo [OK] GMP headers and sources were generated.
exit /b 0

:failed
set "GMP_GENERATE_RESULT=%ERRORLEVEL%"
popd
echo [ERROR] GMP source generation failed with code %GMP_GENERATE_RESULT%.
exit /b %GMP_GENERATE_RESULT%

@echo off
setlocal

if not defined GMP_PRO_LOCATION (
    echo [ERROR] GMP_PRO_LOCATION is not defined. Install GMP or set the repository path first.
    exit /b 1
)

call "%GMP_PRO_LOCATION%\tools\gmp_installer\ensure_gmp_environment.bat"
if errorlevel 1 exit /b %errorlevel%

pushd "%~dp0"
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe_settings.json" generate-project-local "sdpe_requirement.json" --project-dir "." --out "."
set "GMP_CTL_PORTABLE_SDPE_RESULT=%errorlevel%"
popd
exit /b %GMP_CTL_PORTABLE_SDPE_RESULT%

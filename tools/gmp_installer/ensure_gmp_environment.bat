@echo off

if not defined GMP_PRO_LOCATION goto :MISSING_ROOT

for %%I in ("%GMP_PRO_LOCATION%") do set "GMP_ENV_GUARD_ROOT=%%~fI"
if not exist "%GMP_ENV_GUARD_ROOT%\tools\gmp_installer\activate_env.bat" goto :INVALID_ROOT

set "GMP_ENV_GUARD_STATE=%GMP_ENV_GUARD_ROOT%\bin\gmp_virtual_env_installed.flag"
set "GMP_ENV_GUARD_PYTHON=%GMP_ENV_GUARD_ROOT%\bin\python\python.exe"
if not exist "%GMP_ENV_GUARD_PYTHON%" goto :CORRUPT_VIRTUAL
if /i "%GMP_ENV_ACTIVE%"=="%GMP_ENV_GUARD_ROOT%" goto :ACTIVATE_VIRTUAL
if exist "%GMP_ENV_GUARD_STATE%" goto :ACTIVATE_VIRTUAL
echo [ERROR] The GMP private environment installation is incomplete.
echo         Run install_gmp_virtual_env.bat or tools\gmp_installer\utilities\deploy_gmp_env.bat.
goto :FAILURE

:ACTIVATE_VIRTUAL
if not exist "%GMP_ENV_GUARD_PYTHON%" goto :CORRUPT_VIRTUAL
set "GMP_ENV_GUARD_CWD=%CD%"
call "%GMP_ENV_GUARD_ROOT%\tools\gmp_installer\activate_env.bat"
set "GMP_ENV_GUARD_RESULT=%ERRORLEVEL%"
cd /d "%GMP_ENV_GUARD_CWD%"
if not "%GMP_ENV_GUARD_RESULT%"=="0" goto :ACTIVATION_FAILED
set "GMP_ENV_MODE=virtual"
goto :SUCCESS

:MISSING_ROOT
echo [ERROR] GMP_PRO_LOCATION is not defined.
echo         Run install_gmp_virtual_env.bat first.
exit /b 1

:INVALID_ROOT
echo [ERROR] GMP_PRO_LOCATION does not point to a valid GMP Pro repository:
echo         %GMP_ENV_GUARD_ROOT%
goto :FAILURE

:CORRUPT_VIRTUAL
echo [ERROR] GMP private Python is missing.
echo         Re-run install_gmp_virtual_env.bat or
echo         tools\gmp_installer\utilities\deploy_gmp_env.bat.
goto :FAILURE

:ACTIVATION_FAILED
echo [ERROR] Failed to activate the GMP virtual environment.
goto :FAILURE

:SUCCESS
set "GMP_ENV_GUARD_ROOT="
set "GMP_ENV_GUARD_STATE="
set "GMP_ENV_GUARD_PYTHON="
set "GMP_ENV_GUARD_CWD="
set "GMP_ENV_GUARD_RESULT="
exit /b 0

:FAILURE
set "GMP_ENV_GUARD_ROOT="
set "GMP_ENV_GUARD_STATE="
set "GMP_ENV_GUARD_PYTHON="
set "GMP_ENV_GUARD_CWD="
set "GMP_ENV_GUARD_RESULT="
exit /b 1

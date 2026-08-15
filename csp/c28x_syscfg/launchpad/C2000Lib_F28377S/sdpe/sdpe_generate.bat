@echo off
setlocal
if not defined GMP_PRO_LOCATION (
  echo [ERROR] GMP_PRO_LOCATION is not defined.
  exit /b 1
)
set "SDPE=%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py"
set "GLOBAL_SCHEMA=%GMP_PRO_LOCATION%\ctl\hardware_preset\sdpe_schemas"
set "GLOBAL_ENTITY=%GMP_PRO_LOCATION%\ctl\hardware_preset\sdpe_src"
set "LOCAL_SCHEMA=%~dp0sdpe_schemas"
set "LOCAL_ENTITY=%~dp0sdpe_src"
set "REQ=%~dp0launchpad_board_requirement.json"
set "BOARD_ROOT=%~dp0.."
python "%SDPE%" --schema-dir "%GLOBAL_SCHEMA%" --schema-dir "%LOCAL_SCHEMA%" --entity-dir "%GLOBAL_ENTITY%" --entity-dir "%LOCAL_ENTITY%" generate-project-local "%REQ%" --project-dir "%BOARD_ROOT%" --out "%BOARD_ROOT%"
if errorlevel 1 exit /b %errorlevel%
python "%SDPE%" --schema-dir "%GLOBAL_SCHEMA%" --schema-dir "%LOCAL_SCHEMA%" --entity-dir "%GLOBAL_ENTITY%" --entity-dir "%LOCAL_ENTITY%" generate-project-matlab-local "%REQ%" --project-dir "%~dp0." --out "%~dp0."
exit /b %errorlevel%

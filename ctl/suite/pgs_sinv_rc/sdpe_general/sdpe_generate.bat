@echo off
setlocal
call "%~dp0sdpe_settings.bat"
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" validate || exit /b %errorlevel%
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-local "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
if errorlevel 1 exit /b %errorlevel%
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-matlab-local "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
exit /b %errorlevel%

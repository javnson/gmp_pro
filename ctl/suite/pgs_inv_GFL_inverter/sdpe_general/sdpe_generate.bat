@echo off
setlocal
call "%~dp0sdpe_settings.bat"
if errorlevel 1 exit /b %errorlevel%
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" validate
if errorlevel 1 exit /b %errorlevel%
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\sdpe.py" --settings "%SDPE_SETTINGS%" generate-project-local "%SDPE_REQUIREMENT%" --out "%SDPE_OUT%"
exit /b %errorlevel%

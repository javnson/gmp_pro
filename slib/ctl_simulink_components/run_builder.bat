@echo off
setlocal

call "%~dp0run_python.bat" gui
set "MCB_EXIT_CODE=%ERRORLEVEL%"
if not "%MCB_EXIT_CODE%"=="0" echo CTL Simulink Components exited with code %MCB_EXIT_CODE%.
if /i not "%GMP_INSTALLER_NO_PAUSE%"=="1" pause

exit /b %MCB_EXIT_CODE%

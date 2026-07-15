@echo off
setlocal
call "%~dp0sdpe_settings.bat"
if errorlevel 1 exit /b %errorlevel%
python "%GMP_PRO_LOCATION%\tools\SDPE_v2\gui_pyqt\sdpe_gui.py" --settings "%SDPE_SETTINGS%" --mode project --projects ^
  "%SDPE_REQUIREMENT%" ^
  "%~dp0..\project\f280039c_Iris_node\sdpe_mgr\sdpe_requirement.json" ^
  "%~dp0..\project\f280049c\sdpe_mgr\sdpe_requirement.json" ^
  "%~dp0..\project\simulate\sdpe_mgr\sdpe_requirement.json" ^
  "%~dp0..\project\stm32g431\sdpe_mgr\sdpe_requirement.json" ^
  --out "%SDPE_OUT%"
exit /b %errorlevel%

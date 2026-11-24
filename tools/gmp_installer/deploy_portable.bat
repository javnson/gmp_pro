@echo off
call "%~dp0run_installer.bat" deploy %*
exit /b %ERRORLEVEL%

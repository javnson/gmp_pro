@echo off
setlocal

call "%~dp0gmp_generate_inc.bat"
if errorlevel 1 exit /b 1

call "%~dp0gmp_generate_src.bat"
if errorlevel 1 exit /b 1

python "%~dp0..\patch_keil_project.py"
if errorlevel 1 exit /b 1

echo [SUCCESS] Project-local GMP headers and sources are ready.
exit /b 0

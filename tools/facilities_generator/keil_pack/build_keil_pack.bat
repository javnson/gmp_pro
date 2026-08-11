@echo off
setlocal

set "GMP_KEIL_PACK_DIR=%~dp0"
for %%I in ("%GMP_KEIL_PACK_DIR%\..\..\..") do set "GMP_KEIL_REPO_ROOT=%%~fI"

where py >nul 2>nul
if not errorlevel 1 (
    py -3 "%GMP_KEIL_PACK_DIR%gmp_keil_pack.py" --repo-root "%GMP_KEIL_REPO_ROOT%" %*
    exit /b %errorlevel%
)

where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python 3 was not found. Install Python 3 or add it to PATH.
    exit /b 2
)

python "%GMP_KEIL_PACK_DIR%gmp_keil_pack.py" --repo-root "%GMP_KEIL_REPO_ROOT%" %*
exit /b %errorlevel%

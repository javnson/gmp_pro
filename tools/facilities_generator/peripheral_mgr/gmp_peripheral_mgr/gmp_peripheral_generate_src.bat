@echo off
setlocal EnableDelayedExpansion

echo =======================================================
echo [GMP Build Task] Peripheral Code Generator Started
echo =======================================================

:: 1. 检查环境变量是否存在
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] The environment variable GMP_PRO_LOCATION is not set!
    echo [ERROR] Please set it to your GMP software root directory.
    exit /b 1
)

:: 2. 调用核心 Python 脚本进行同步
:: (假设用户的电脑已经安装了 Python 并添加到了 PATH)
python "%GMP_PRO_LOCATION%\tools\facilities_generator\peripheral_mgr\core_sync_forward.py"

:: 3. 捕获 Python 脚本的退出码 (Exit Code)
set PY_ERRORLEVEL=%ERRORLEVEL%
if %PY_ERRORLEVEL% neq 0 (
    echo.
    echo [GMP Build Task] FATAL ERROR: Synchronization failed with code %PY_ERRORLEVEL%.
    echo [GMP Build Task] Build Process Aborted.
    echo =======================================================
    exit /b %PY_ERRORLEVEL%
)

:: 4. 成功退出
echo [GMP Build Task] Synchronization completed successfully.
echo =======================================================
exit /b 0
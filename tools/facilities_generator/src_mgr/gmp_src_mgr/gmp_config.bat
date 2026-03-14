@echo off
setlocal EnableDelayedExpansion

title GMP Framework Configurator
echo =======================================================
echo [GMP] 正在启动框架配置编辑器 (GUI)...
echo =======================================================

:: 检查环境变量
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] 未设置环境变量 GMP_PRO_LOCATION !
    echo [ERROR] 请先将其设置为您的 GMP 核心库根目录。
    pause
    exit /b 1
)

:: 拉起配置器 GUI
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_user_gui_v10.py"

:: 错误捕获
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] 框架配置器异常退出，错误码: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

exit /b 0
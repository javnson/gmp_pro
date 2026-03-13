@echo off
setlocal EnableDelayedExpansion

title GMP Framework Asset Manager (Developer)
echo =======================================================
echo ⚙️ [GMP Dev Tool] 正在启动核心框架资产管理器...
echo =======================================================

:: 1. 检查环境变量
if "%GMP_PRO_LOCATION%"=="" (
    echo [ERROR] 未找到环境变量 GMP_PRO_LOCATION !
    echo [ERROR] 请先将其设置为您的 GMP 核心库根目录。
    pause
    exit /b 1
)

:: 2. 拉起开发者 GUI (框架资产管理器)
python "%GMP_PRO_LOCATION%\tools\facilities_generator\src_mgr\framework_dev_gui_v14.py"

:: 3. 错误捕获
if %ERRORLEVEL% neq 0 (
    echo.
    echo [ERROR] 资产管理器异常退出，错误码: %ERRORLEVEL%
    pause
    exit /b %ERRORLEVEL%
)

:: 正常退出时直接关闭窗口，不留黑框
exit /b 0